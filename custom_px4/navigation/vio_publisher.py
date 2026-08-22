"""
PX4 EKF2에 external vision odometry(VehicleOdometry)를 퍼블리시하는 공용 모듈.

camera_line_scanning_vio.py(라인 추적)와 follow_waypoints.py(아루코 마커
기반 mode1)에서 공통으로 재사용한다. 사용하는 쪽에서 계산한 값(고도,
좌우/전후 offset, yaw, pitch, roll)을 update_latest()로 넘겨주기만 하면,
내부 스레드가 알아서 지정된 rate로 PX4에 재퍼블리시한다.

중요(실전에서 확인된 결론):
- quality 필드를 0으로 내리는 방식(라인 소실 시 무효 처리)은 EKF2가 EV
  position aiding을 "처음 활성화(latch)"하는 조건 자체를 깨뜨려서 arming
  자체가 막히는 부작용이 있었다 (estimator_aid_src_ev_pos.fused가 영원히
  False로 남는 현상 확인됨). 그래서 quality는 항상 1로 고정하고, 신뢰도
  조절은 오직 position_variance로만 한다. variance는 EKF2 내부 latch
  상태에 영향을 주지 않고 "이번 샘플을 얼마나 믿을지"만 조절하기 때문에
  안전하다.
"""

import time
import threading

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry


class VisionOdometryPublisher:
    """
    계산된 altitude / offset_m(y) / yaw_deg / pitch_deg / roll_deg 값을
    PX4 EKF2가 fusion할 수 있는 VehicleOdometry 메시지로 변환해서 퍼블리시한다.

    주의(중요):
    - x(전진 방향, forward_m), y(좌우 offset_m), z(고도), yaw/pitch/roll
      모두 동일한 규칙을 따른다: 값이 들어오면 갱신하고, None이면(계산 실패)
      "직전 유효값을 유지"해서 항상 finite한 값을 보낸다
      (EKF2가 NaN이 섞인 샘플을 통째로 무효 처리하기 때문).
    - x는 기본값이 0.0이며, 호출부가 forward_m을 넘겨주지 않으면 계속
      0.0으로 유지된다(= 기존과 동일한 placeholder 동작).
    - quality는 항상 1로 고정한다 (위 설명 참조).
    """

    def __init__(self, node: rclpy.node.Node, publish_rate_hz: float = 30.0,
                 enable_attitude_fusion: bool = True,
                 line_lost_timeout_sec: float = 0.5,
                 line_lost_variance_scale: float = 20.0):
        self.node = node

        # 디버그용: False로 두면 pitch/roll/yaw 값이 들어와도 무시하고
        # identity quaternion([1,0,0,0])만 퍼블리시한다.
        self._enable_attitude_fusion = enable_attitude_fusion

        # 라인을 마지막으로 정상 인식한 시각. line_lost_timeout_sec을 넘기면
        # "값은 그대로 보내되 variance를 크게 키워서 신뢰도만 낮춘다"
        # (quality는 절대 건드리지 않는다 - 위 주석 참조).
        self._line_lost_timeout_sec = line_lost_timeout_sec
        self._line_lost_variance_scale = line_lost_variance_scale
        self._last_line_valid_time = None

        # PX4 uXRCE-DDS 쪽과 호환되는 QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher_ = node.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile,
        )

        # x(전진 방향), y(offset), z(altitude) 계산 실패 시 유지할 "직전 유효값"
        self._last_pos_x_m = 0.0
        self._last_pos_y_m = 0.0
        self._last_altitude_m = 0.0

        # ============================================================
        # 순수 Python 스레드 방식: 무거운 영상처리 루프가 spin 기회를 막아도
        # 고정 rate로 퍼블리시하기 위함. publisher.publish()는 다른
        # 스레드에서 호출해도 안전하다.
        # ============================================================
        self._latest_forward = None
        self._latest_altitude = None
        self._latest_offset = None
        self._latest_yaw = None
        self._latest_pitch = None
        self._latest_roll = None
        self._has_data = False

        # takeoff 명령 전에는 EKF2에 x=0, y=0을 고정해서 보내고,
        # z는 update_latest()로 들어오는 실제 고도값을 계속 반영한다.
        # 호출부에서 takeoff 명령을 받으면 set_takeoff_started(True)를 호출한다.
        self._takeoff_started = False
        self._lock = threading.Lock()

        self._publish_rate_hz = publish_rate_hz
        self._stop_event = threading.Event()
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True
        )
        self._publish_thread.start()

    def set_x_position(self, x_m: float):
        """x(전진 방향) 위치를 수동으로 한 번 지정하고 싶을 때 사용."""
        self._last_pos_x_m = float(x_m)

    def set_takeoff_started(self, started: bool = True):
        """
        takeoff 상태를 설정한다.

        False: x=0, y=0으로 고정하고 z는 최신 고도값을 계속 발행
        True : update_latest()로 받은 실제 VIO 값을 발행
        """
        with self._lock:
            self._takeoff_started = bool(started)

            # takeoff 대기 상태에서는 X/Y만 원점으로 초기화한다.
            # Z는 실제 고도 입력을 계속 유지해야 하므로 초기화하지 않는다.
            if not self._takeoff_started:
                self._last_pos_x_m = 0.0
                self._last_pos_y_m = 0.0

    def update_latest(self, altitude_m, offset_m, yaw_deg, pitch_deg=None,
                       roll_deg=None, forward_m=None):
        """호출부에서 매 프레임/틱마다 호출. 저장만 하고 퍼블리시는 안 함."""
        with self._lock:
            self._latest_altitude = altitude_m
            self._latest_offset = offset_m
            self._latest_yaw = yaw_deg
            self._latest_pitch = pitch_deg
            self._latest_roll = roll_deg
            self._latest_forward = forward_m
            self._has_data = True

    def _publish_loop(self):
        period = 1.0 / self._publish_rate_hz
        while not self._stop_event.is_set():
            start = time.time()

            with self._lock:
                takeoff_started = self._takeoff_started
                has_data = self._has_data
                altitude_m = self._latest_altitude
                offset_m = self._latest_offset
                yaw_deg = self._latest_yaw
                pitch_deg = self._latest_pitch
                roll_deg = self._latest_roll
                forward_m = self._latest_forward

            if not takeoff_started:
                self.publish_pre_takeoff_position(altitude_m)
            elif has_data:
                self.publish(
                    altitude_m, offset_m, yaw_deg,
                    pitch_deg, roll_deg, forward_m
                )
            else:
                # takeoff 명령 직후 첫 VIO 데이터가 들어오기 전까지도
                # 메시지 스트림이 끊기지 않도록 마지막 위치를 유지한다.
                self.publish(
                    None, None, None, None, None, None
                )

            elapsed = time.time() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._stop_event.set()
        self._publish_thread.join(timeout=1.0)

    def publish_pre_takeoff_position(self, altitude_m):
        """takeoff 전에는 X/Y=0, Z=최신 실제 고도로 지속 발행한다."""
        msg = VehicleOdometry()

        now_us = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD

        # X/Y는 이륙 명령 전까지 원점으로 고정한다.
        msg.position[0] = 0.0
        msg.position[1] = 0.0

        # Z는 update_latest()로 들어온 실제 고도를 계속 반영한다.
        # FRD/NED에서는 아래쪽이 +Z이므로 고도는 음수로 변환한다.
        if altitude_m is not None and altitude_m > 0:
            self._last_altitude_m = -float(altitude_m)
        msg.position[2] = self._last_altitude_m

        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.velocity = [float('nan')] * 3
        msg.angular_velocity = [float('nan')] * 3

        msg.position_variance = [0.05, 0.05, 0.1]
        msg.orientation_variance = [0.05, 0.05, 0.05]
        msg.velocity_variance = [float('nan')] * 3
        msg.quality = 1

        self.publisher_.publish(msg)

    def publish(self, altitude_m, offset_m, yaw_deg, pitch_deg=None, roll_deg=None,
                forward_m=None):
        msg = VehicleOdometry()

        now_us = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us

        msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD

        # ---------------- 위치 인식 상태 추적 (variance 조절용) ----------------
        # 수정된 부분(핵심): yaw_deg를 조건에서 뺐다.
        # 라인 기반 모드에서는 offset/altitude/yaw가 보통 같이 들어오지만,
        # 아루코 기반 x/y/z 오버라이드 시에는 aruco_detector.py의 설계
        # 원칙상 yaw를 절대 계산하지 않으므로 yaw_deg는 항상 None이다.
        # yaw_deg를 조건에 넣으면 "위치는 방금 막 정밀하게 갱신됐는데
        # yaw가 없다는 이유만으로 position_variance를 20배 키우는" 모순이
        # 생긴다. position 유효성은 offset_m/altitude_m만으로 판단한다.
        position_valid_this_frame = (
            offset_m is not None
            and altitude_m is not None and altitude_m > 0
        )
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        if position_valid_this_frame:
            self._last_line_valid_time = now_sec

        line_is_stale = (
            self._last_line_valid_time is None or
            (now_sec - self._last_line_valid_time) > self._line_lost_timeout_sec
        )

        # ---------------- 위치 ----------------
        # NED/FRD position[0] = X(전진), position[1] = Y(좌우)
        if forward_m is not None:
            self._last_pos_x_m = float(forward_m)
        msg.position[0] = self._last_pos_x_m

        # offset_m은 좌우(Y) 위치 입력이다.
        # 입력이 None인 프레임에서도 마지막 유효 Y 위치를 유지하기 위해
        # _last_pos_y_m에 저장한다.
        if offset_m is not None:
            self._last_pos_y_m = float(offset_m)
        msg.position[1] = self._last_pos_y_m

        # z: 고도 (FRD/NED 기준 아래쪽이 +z 이므로 고도는 음수로 변환)
        if altitude_m is not None and altitude_m > 0:
            self._last_altitude_m = -float(altitude_m)
        msg.position[2] = self._last_altitude_m

        # ---------------- 자세 (roll + pitch + yaw) ----------------
        if self._enable_attitude_fusion and (
                yaw_deg is not None or pitch_deg is not None or roll_deg is not None):
            yaw_rad = np.deg2rad(yaw_deg) if yaw_deg is not None else 0.0
            pitch_rad = np.deg2rad(pitch_deg) if pitch_deg is not None else 0.0
            roll_rad = np.deg2rad(roll_deg) if roll_deg is not None else 0.0

            cy, sy = np.cos(yaw_rad / 2.0), np.sin(yaw_rad / 2.0)
            cp, sp = np.cos(pitch_rad / 2.0), np.sin(pitch_rad / 2.0)
            cr, sr = np.cos(roll_rad / 2.0), np.sin(roll_rad / 2.0)

            qw = float(cr * cp * cy + sr * sp * sy)
            qx = float(sr * cp * cy - cr * sp * sy)
            qy = float(cr * sp * cy + sr * cp * sy)
            qz = float(cr * cp * sy - sr * sp * cy)
            msg.q = [qw, qx, qy, qz]
        else:
            msg.q = [1.0, 0.0, 0.0, 0.0]

        # ---------------- 속도 ----------------
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

        # ---------------- 분산(신뢰도) ----------------
        # 라인을 최근에 정상 인식했으면 기본 variance,
        # 놓친 지 오래됐으면 variance를 크게 키워서 "값은 보내되 덜 믿어라"만
        # 표현한다 (quality는 절대 건드리지 않음 - 상단 주석 참조).
        base_pos_var = [0.005, 0.005, 0.01]
        base_ori_var = [0.005, 0.005, 0.005]
        if line_is_stale:
            scale = self._line_lost_variance_scale
            msg.position_variance = [v * scale for v in base_pos_var]
            msg.orientation_variance = [v * scale for v in base_ori_var]
        else:
            msg.position_variance = base_pos_var
            msg.orientation_variance = base_ori_var
        msg.velocity_variance = [float('nan')] * 3

        # ---------------- quality ----------------
        # 항상 1로 고정 (실전 검증: quality를 0으로 내리면 EKF2가
        # EV position aiding을 처음 latch하는 조건이 깨져서 arming/takeoff가
        # 막히는 현상이 재현됨. 신뢰도 조절은 variance로만 한다).
        msg.quality = 1

        self.publisher_.publish(msg)
