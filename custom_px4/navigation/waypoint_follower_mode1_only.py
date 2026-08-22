import math
import threading
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)

from camera import Camera
from aruco_detector import detect_aruco, compute_center_offset_m
from line_table_io import load_aruco_target_area


# ================= 사용자 조정값 =================
TARGET_HEIGHT_M = 2.0
TAKEOFF_THRUST = 0.80
TAKEOFF_RAMP_START_THRUST = 0.50
TAKEOFF_RAMP_DURATION_SEC = 3.0
HOVER_THRUST = 0.50
HEIGHT_TOLERANCE_M = 0.10

# 고도별 ArUco 보정 활성화 기준
ARUCO_XY_ENABLE_HEIGHT_M = 0.70
ARUCO_Z_ENABLE_HEIGHT_M = 1.50

OFFBOARD_PREROLL_TICKS = 10
POSITION_TRANSITION_TICKS = 20

# ArUco 중심 정렬
ARUCO_XY_GAIN = 1

# 마커 중심 오차가 이 값 이하이면 해당 축은 보정하지 않는다.
# 중심 근처의 검출 노이즈로 setpoint 부호가 반복 반전되는 것을 방지한다.
ARUCO_XY_DEADBAND_M = 0.0

# 이미지 오차 -> NED 위치 보정 축/부호
# 이미지 세로 오차(y_offset_m) -> NED x(전후)
# 이미지 가로 오차(x_offset_m) -> NED y(좌우)
# 실제 이동 방향이 반대면 해당 SIGN만 -1.0으로 변경
ARUCO_FORWARD_SIGN = 1.0
ARUCO_LATERAL_SIGN = -1.0

# 마커 면적으로 고도 보정: 이동량 제한/데드밴드 없음
ARUCO_Z_GAIN = 1



class AttitudeTakeoffArucoMode1(Node):
    """
    1) mode1/takeoff 명령 수신
    2) Attitude Offboard, level attitude, thrust_body z=-0.80으로 이륙
    3) 시작 지점 대비 2m 도달
    4) Position Offboard로 전환
    5) ArUco 중심 오차로 XY 정렬
    6) line_table.csv 목표 area가 있으면 마커 면적으로 고도 보정
    """

    def __init__(self):
        super().__init__('attitude_takeoff_aruco_mode1_080')

        self.status = None
        self.odom = None
        self.vehicle_attitude = None

        self.active = False
        self.phase = 'idle'
        self.counter = 0
        self.arm_seq_counter = 0
        self.transition_counter = 0

        self.ground_z = None
        self.takeoff_yaw = None
        self.commanded_thrust = 0.0
        self.takeoff_ramp_start_ns = None

        self.hold_x = None
        self.hold_y = None
        self.hold_z = None

        self.aruco_detected_once = False
        self.last_marker_id = None
        self.xy_correction_enabled = False
        self.z_correction_enabled = False

        # 동일 카메라 프레임 중복 처리 방지용 상태
        self.last_frame_signature = None

        self.target_aruco_area = load_aruco_target_area('line_table.csv')
        if self.target_aruco_area is None:
            self.get_logger().warn(
                'line_table.csv에서 목표 ArUco area를 찾지 못했습니다. '
                'XY 정렬은 동작하지만 area 기반 고도 보정은 비활성화됩니다.'
            )
        else:
            self.get_logger().info(
                f'ArUco 목표 area={self.target_aruco_area:.1f}px'
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # PX4/SITL 환경에 따라 vehicle_status 또는 vehicle_status_v1 중
        # 하나만 발행될 수 있으므로 두 토픽을 모두 구독한다.
        vehicle_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, vehicle_status_qos)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback, qos)
        self.create_subscription(
            String, '/mode_command', self.mode_command_callback, 10)

        self.cam = Camera()

        # 수정된 부분(핵심 버그 픽스 v2): rclpy.spin(self.cam)을 그냥
        # 스레드에서 돌리면 "ValueError: generator already executing"이
        # 발생한다 - rclpy.spin()이 내부적으로 기본 context의 전역
        # executor/제너레이터를 참조하는데, 메인 스레드에서 이미
        # rclpy.spin(node)가 그걸 쓰고 있는 상태에서 다른 스레드가 같은
        # context에 대해 또 spin을 시도하면 충돌한다.
        # 그래서 Camera 전용 SingleThreadedExecutor를 별도로 만들어
        # self.cam만 명시적으로 add_node()하고, 그 executor 객체
        # (전역이 아니라 이 인스턴스)만 별도 스레드에서 spin한다.
        # 이러면 메인 노드의 executor와 완전히 분리되어 서로 간섭하지
        # 않는다.
        self._cam_executor = rclpy.executors.SingleThreadedExecutor()
        self._cam_executor.add_node(self.cam)
        self._cam_spin_thread = threading.Thread(
            target=self._cam_executor.spin, daemon=True)
        self._cam_spin_thread.start()

        # 카메라 처리 결과는 별도로 저장하고, 비행 제어 타이머에서는
        # 저장된 결과만 소비한다. 이렇게 해서 setpoint 발행 주기를
        # 카메라/ArUco 처리 흐름과 분리한다.
        self.latest_aruco_detection = None

        # Offboard mode + setpoint는 20 Hz로 지속 발행한다.
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 영상 처리는 10 Hz로 별도 실행한다.
        self.vision_timer = self.create_timer(0.1, self.vision_timer_callback)

    def reset(self):
        self.phase = 'attitude_takeoff'
        self.counter = 0
        self.arm_seq_counter = 0
        self.transition_counter = 0
        self.ground_z = None
        self.takeoff_yaw = None
        self.commanded_thrust = 0.0
        self.takeoff_ramp_start_ns = None
        self.hold_x = None
        self.hold_y = None
        self.hold_z = None
        self.aruco_detected_once = False
        self.last_marker_id = None
        self.xy_correction_enabled = False
        self.z_correction_enabled = False
        self.last_frame_signature = None

    def mode_command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd in ('mode1', 'takeoff'):
            # mode_commander가 같은 명령을 반복 발행해도 비행 상태와
            # setpoint를 다시 초기화하지 않는다.
            if self.active:
                return

            self.reset()
            self.latest_aruco_detection = None
            self.active = True
            self.get_logger().warn(
                f'mode1 시작: attitude thrust=-{TAKEOFF_THRUST:.2f}, '
                f'target={TARGET_HEIGHT_M:.1f}m -> ArUco position control')

        elif cmd in ('stop', 'off', 'none'):
            if not self.active:
                return

            self.active = False
            self.phase = 'idle'
            self.latest_aruco_detection = None
            self.get_logger().warn('명령 발행 중지')

    def vehicle_status_callback(self, msg):
        self.status = msg

    def odometry_callback(self, msg):
        self.odom = msg

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude = msg

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def get_position(self):
        if self.odom is None:
            return None
        return tuple(float(v) for v in self.odom.position[:3])

    def get_current_yaw(self):
        if self.vehicle_attitude is None:
            return None
        w, x, y, z = map(float, self.vehicle_attitude.q)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def quaternion_from_level_yaw(yaw):
        half = yaw * 0.5
        return [math.cos(half), 0.0, 0.0, math.sin(half)]

    def publish_attitude_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_position_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_attitude_setpoint(self, thrust):
        thrust = float(np.clip(thrust, 0.0, 1.0))
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp_us()
        msg.q_d = self.quaternion_from_level_yaw(self.takeoff_yaw)
        msg.thrust_body = [0.0, 0.0, -thrust]
        msg.yaw_sp_move_rate = 0.0
        self.attitude_setpoint_pub.publish(msg)

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [
            float(self.hold_x), float(self.hold_y), float(self.hold_z)]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float(self.takeoff_yaw)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_offboard(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def is_offboard(self):
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def is_armed(self):
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def ensure_attitude_offboard_and_armed(self):
        self.publish_attitude_mode()
        self.publish_attitude_setpoint(0.0)
        self.arm_seq_counter += 1

        if self.arm_seq_counter < OFFBOARD_PREROLL_TICKS:
            return False

        # 상태 토픽이 늦게 갱신되더라도 OFFBOARD와 ARM 요청을 각각
        # 주기적으로 계속 보낸다. 한 조건에서 return해 다음 요청이
        # 막히지 않도록 두 요청을 독립적으로 처리한다.
        if self.arm_seq_counter % 10 == 0:
            if not self.is_offboard():
                self.request_offboard()
                self.get_logger().info('OFFBOARD 요청')

            if not self.is_armed():
                self.request_arm()
                self.get_logger().info('ARM 요청')

        return self.is_offboard() and self.is_armed()

    @staticmethod
    def frame_signature(frame):
        # 전체 프레임 해시 비용을 줄이기 위해 일정 간격으로 샘플링한다.
        sampled = np.ascontiguousarray(frame[::16, ::16])
        return (
            frame.shape,
            int(zlib.adler32(sampled.tobytes())),
        )

    def read_aruco(self):
        # 수정된 부분: self.cam은 이제 __init__에서 시작한 별도 스레드가
        # 계속 spin하고 있으므로, 여기서 spin_once를 또 호출할 필요가
        # 없다 (오히려 두 곳에서 동시에 같은 노드를 spin하면 스레드
        # 안전성 문제가 생길 수 있어 반드시 제거해야 한다).
        frame = self.cam.read()
        if frame is None:
            return None

        # 같은 프레임이 timer 주기마다 반복 반환되면 보정을 재적용하지 않는다.
        signature = self.frame_signature(frame)
        if signature == self.last_frame_signature:
            return None
        self.last_frame_signature = signature

        ids, area, corners = detect_aruco(frame)
        if ids is None or area <= 0 or len(corners) == 0:
            return None

        offset = compute_center_offset_m(frame.shape, corners[0])
        marker_id = int(np.asarray(ids).reshape(-1)[0])
        return {
            'id': marker_id,
            'area': float(area),
            'x_offset_m': float(offset['x_offset_m']),
            'y_offset_m': float(offset['y_offset_m']),
            'total_offset_m': float(offset['total_offset_m']),
        }

    def attitude_takeoff_step(self):
        pos = self.get_position()
        if pos is None or self.vehicle_attitude is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'지면 NED z={self.ground_z:.3f}m')

        if self.takeoff_yaw is None:
            self.takeoff_yaw = self.get_current_yaw()
            if self.takeoff_yaw is None:
                return
            self.get_logger().info(
                f'초기 yaw={math.degrees(self.takeoff_yaw):.1f}deg')

        if not self.ensure_attitude_offboard_and_armed():
            return

        current_height = self.ground_z - pos[2]
        self.publish_attitude_mode()

        if current_height < ARUCO_XY_ENABLE_HEIGHT_M:
            # ARM/OFFBOARD 완료 시점을 기준으로 추력을 선형 증가시킨다.
            now_ns = self.get_clock().now().nanoseconds
            if self.takeoff_ramp_start_ns is None:
                self.takeoff_ramp_start_ns = now_ns

            elapsed_sec = (now_ns - self.takeoff_ramp_start_ns) / 1e9
            ramp_ratio = float(np.clip(
                elapsed_sec / max(TAKEOFF_RAMP_DURATION_SEC, 1e-3),
                0.0,
                1.0,
            ))

            self.commanded_thrust = (
                TAKEOFF_RAMP_START_THRUST
                + (TAKEOFF_THRUST - TAKEOFF_RAMP_START_THRUST) * ramp_ratio
            )
            self.publish_attitude_setpoint(self.commanded_thrust)
        else:
            # 0.7m부터 Position Offboard로 전환한다.
            # XY는 현재 위치에서 시작하고, Z 목표는 최종 목표 고도로 지정해
            # Position 제어 상태에서도 계속 상승하도록 한다.
            self.hold_x = pos[0]
            self.hold_y = pos[1]
            self.hold_z = self.ground_z - TARGET_HEIGHT_M
            self.phase = 'position_transition'
            self.transition_counter = 0
            self.get_logger().warn(
                f'{current_height:.2f}m 도달: Position Offboard 전환 시작, '
                'ArUco XY 보정 준비')

        if self.counter % 10 == 0:
            self.get_logger().info(
                f'[takeoff] height={current_height:.2f}m '
                f'thrust=-{self.commanded_thrust:.2f}')

    def position_transition_step(self):
        # Position control mode/setpoint를 충분히 선행 발행한 뒤 ArUco 제어 시작
        self.publish_position_mode()
        self.publish_position_setpoint()
        self.transition_counter += 1

        if self.transition_counter >= POSITION_TRANSITION_TICKS:
            self.phase = 'aruco_hold'
            self.get_logger().warn('Position Offboard 전환 완료: ArUco 보정 시작')

    def aruco_hold_step(self):
        self.publish_position_mode()

        pos = self.get_position()
        if pos is None or self.ground_z is None:
            self.publish_position_setpoint()
            return

        current_height = self.ground_z - pos[2]

        # 고도에 따라 보정 축을 단계적으로 활성화한다.
        xy_enabled_now = current_height >= ARUCO_XY_ENABLE_HEIGHT_M
        z_enabled_now = current_height >= ARUCO_Z_ENABLE_HEIGHT_M

        if xy_enabled_now and not self.xy_correction_enabled:
            self.get_logger().warn(
                f'{current_height:.2f}m: ArUco XY 보정 활성화')
        if z_enabled_now and not self.z_correction_enabled:
            self.get_logger().warn(
                f'{current_height:.2f}m: ArUco Z 보정 활성화')

        self.xy_correction_enabled = xy_enabled_now
        self.z_correction_enabled = z_enabled_now

        # 비전 타이머가 만든 최신 결과를 한 번만 소비한다.
        detection = self.latest_aruco_detection
        self.latest_aruco_detection = None
        if detection is not None:
            if not self.aruco_detected_once:
                self.aruco_detected_once = True
                self.get_logger().warn(
                    f'ArUco 최초 검출: id={detection["id"]}')

            self.last_marker_id = detection['id']

            # 0.7m 이상: XY 보정만 수행
            if self.xy_correction_enabled:
                # 카메라/기체 좌표계에서 측정된 마커 오차
                body_forward_error = (
                    detection['y_offset_m'] * ARUCO_FORWARD_SIGN
                )
                body_lateral_error = (
                    detection['x_offset_m'] * ARUCO_LATERAL_SIGN
                )

                # 카메라/기체 좌표 오차를 NED 좌표계로 회전한다.
                # 이륙 yaw를 Position setpoint에서도 유지하므로 takeoff_yaw 사용.
                yaw = self.takeoff_yaw
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)

                ned_x_error = (
                    cos_yaw * body_forward_error
                    - sin_yaw * body_lateral_error
                )
                ned_y_error = (
                    sin_yaw * body_forward_error
                    + cos_yaw * body_lateral_error
                )

                # 축별 데드밴드: 중심 오차가 작은 축은 목표점을 갱신하지 않는다.
                # 이렇게 해야 중심 근처 검출 노이즈 때문에 setpoint가
                # 반대 부호로 계속 왕복하는 현상을 줄일 수 있다.
                x_in_deadband = abs(ned_x_error) <= ARUCO_XY_DEADBAND_M
                y_in_deadband = abs(ned_y_error) <= ARUCO_XY_DEADBAND_M

                if not x_in_deadband:
                    forward_step = ned_x_error * ARUCO_XY_GAIN
                    self.hold_x = pos[0] - forward_step
                    #self.hold_x = pos[0]

                if not y_in_deadband:
                    lateral_step = ned_y_error * ARUCO_XY_GAIN
                    self.hold_y = pos[1] - lateral_step
                    #self.hold_y = pos[1]
                    			
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'[aruco-xy] '
                        f'body_err=({body_forward_error:+.3f}, '
                        f'{body_lateral_error:+.3f}) '
                        f'ned_err=({ned_x_error:+.3f}, '
                        f'{ned_y_error:+.3f}) '
                        f'deadband=({x_in_deadband}, {y_in_deadband}) '
                        f'sp=({self.hold_x:+.3f}, {self.hold_y:+.3f})'
                    )

            # 1.5m 이상: 기존 XY 보정에 Z 보정까지 추가
            if (
                self.z_correction_enabled
                and self.target_aruco_area is not None
            ):
                area_error = detection['area'] - self.target_aruco_area
                normalized_area_error = (
                    area_error / max(self.target_aruco_area, 1.0)
                )
                self.hold_z -= normalized_area_error * ARUCO_Z_GAIN

            if self.counter % 10 == 0:
                control_mode = (
                    'XYZ' if self.z_correction_enabled
                    else 'XY' if self.xy_correction_enabled
                    else 'HOLD'
                )
                self.get_logger().info(
                    f'[aruco] mode={control_mode} '
                    f'height={current_height:.2f}m '
                    f'id={detection["id"]} '
                    f'area={detection["area"]:.0f} '
                    f'dx={detection["x_offset_m"]:+.3f}m '
                    f'dy={detection["y_offset_m"]:+.3f}m '
                    f'sp=({self.hold_x:.2f}, {self.hold_y:.2f}, '
                    f'{self.hold_z:.2f})')
        elif self.counter % 10 == 0:
            self.get_logger().info(
                f'[aruco] height={current_height:.2f}m '
                '마커 미검출 - 마지막 position setpoint 유지')

        self.publish_position_setpoint()

    def vision_timer_callback(self):
        if not self.active:
            return

        if self.phase not in ('position_transition', 'aruco_hold'):
            return

        try:
            self.latest_aruco_detection = self.read_aruco()
        except Exception as e:
            self.latest_aruco_detection = None
            self.get_logger().warn(f'ArUco 처리 오류: {e}')

    def timer_callback(self):
        if not self.active:
            return

        if self.phase == 'attitude_takeoff':
            self.attitude_takeoff_step()
        elif self.phase == 'position_transition':
            self.position_transition_step()
        elif self.phase == 'aruco_hold':
            self.aruco_hold_step()

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeTakeoffArucoMode1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 카메라 전용 executor/스레드도 함께 정리한다. shutdown()을
        # 먼저 호출해 spin 루프를 빠져나오게 한 뒤 노드를 destroy한다.
        node._cam_executor.shutdown()
        node.cam.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
