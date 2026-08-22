import csv
import math

import rclpy
from px4_msgs.msg import (
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

import camera
import control
import debug
import vision
from config import (
    ARUCO_Z_ENABLE_HEIGHT_M,
    CONTROL_RATE,
    MODE1_ERROR_THRESHOLD,
    MODE1_TARGET_ALTITUDE,
    MODE3_LINE_CONFIRM_COUNT,
    MODE3_LINE_LOST_COUNT,
    VIO_PUBLISH_RATE,
    VIO_POSITION_VARIANCE_NONE,
    VIO_VARIANCE_HIGH_X,
    VIO_VARIANCE_HIGH_Y,
    VIO_VARIANCE_LOW_X,
    VIO_VARIANCE_LOW_Y,
    VIO_VARIANCE_LOW_Z,
    VISION_RATE,
)
from coordinate import (
    body_xy_to_ned,
    quaternion_to_euler,
)
from vio_pub import VisionOdometryPublisher


VEHICLE_LOCAL_POSITION_TOPIC = (
    '/fmu/out/vehicle_local_position'
)
VEHICLE_ATTITUDE_TOPIC = (
    '/fmu/out/vehicle_attitude'
)
VEHICLE_STATUS_TOPIC = (
    '/fmu/out/vehicle_status'
)
MODE_COMMAND_TOPIC = '/mode_command'

MISSION_IDLE = -1
MISSION_MODE1 = 0
MISSION_MODE3 = 3

MODE1_TARGET_X = 0.0
MODE1_TARGET_Y = 0.0
MODE1_TARGET_Z = -MODE1_TARGET_ALTITUDE

MODE3_ARRIVAL_THRESHOLD = 0.20
MODE3_GRID_ERROR_THRESHOLD = 0.10
MODE3_HOVER_SEC = 2.0
MODE3_ALTITUDE_ERROR_THRESHOLD = 0.15
MODE3_ROLL_PITCH_THRESHOLD_RAD = math.radians(5.0)
MODE3_YAW_THRESHOLD_RAD = math.radians(5.0)


class MainNode(Node):
    def __init__(self):
        super().__init__('flight_mission')

        self.table = self.load_vision_table(
            'line_table.csv'
        )

        self.x_hat = 0.0
        self.y_hat = 0.0
        self.z_hat = 0.0
        self.local_position_received = False
        self.ground_z = None

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.takeoff_yaw = None
        self.attitude_received = False

        self.vehicle_status = None

        self.pattern = -1
        self.marker_id = -1

        self.vision_altitude = -1.0
        self.body_err_x = 0.0
        self.body_err_y = 0.0
        self.err_ned_x = 0.0
        self.err_ned_y = 0.0

        self.vision_valid = False
        self.vision_roll = 0.0
        self.vision_pitch = 0.0
        self.vision_yaw = 0.0
        self.vision_attitude_valid = False
        self.vision_position_valid = False
        self.vision_altitude_valid = False

        self.vision_source = 'none'
        self.grid_detected = False
        self.line_detected = False
        self.detected_line_direction = None

        self.mission_state = MISSION_IDLE

        self.offboard_prestream_count = 0
        self.offboard_prestream_target = max(
            10,
            int(CONTROL_RATE),
        )
        self.command_retry_counter = 0

        self.mode1_target_x = 0.0
        self.mode1_target_y = 0.0
        self.mode1_target_z = 0.0
        self.mode1_target_initialized = False
        self.mode1_marker_id = -1
        self.mode1_error = float('inf')

        self.mode3_targets = self.load_mode3_table(
            'mode3.csv'
        )
        self.mode3_target_index = 0
        self.mode3_target_x = 0.0
        self.mode3_target_y = 0.0
        self.mode3_target_z = 0.0

        self.mode3_sub_state = 'moving'
        self.mode3_hover_start_time = None
        self.mode3_complete = False
        self.mode3_yaw_target = 0.0

        self.mode3_confirmed_direction = None
        self.mode3_candidate_direction = None
        self.mode3_line_confirm_count = 0
        self.mode3_line_lost_count = 0

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        control.init_cmd(self)
        self.vio = VisionOdometryPublisher(self)

        if not camera.init_cam(self):
            raise RuntimeError(
                '카메라 초기화에 실패했습니다.'
            )

        self.local_position_subscription = (
            self.create_subscription(
                VehicleLocalPosition,
                VEHICLE_LOCAL_POSITION_TOPIC,
                self.local_position_callback,
                self.px4_qos,
            )
        )

        self.attitude_subscription = (
            self.create_subscription(
                VehicleAttitude,
                VEHICLE_ATTITUDE_TOPIC,
                self.attitude_callback,
                self.px4_qos,
            )
        )

        self.status_subscription = (
            self.create_subscription(
                VehicleStatus,
                VEHICLE_STATUS_TOPIC,
                self.status_callback,
                volatile_qos,
            )
        )

        self.mode_command_subscription = (
            self.create_subscription(
                String,
                MODE_COMMAND_TOPIC,
                self.mode_command_callback,
                10,
            )
        )

        self.vision_timer = self.create_timer(
            1.0 / VISION_RATE,
            self.vision_callback,
        )

        self.vio_timer = self.create_timer(
            1.0 / VIO_PUBLISH_RATE,
            self.vio.publish,
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE,
            self.control_callback,
        )

    def mode_command_callback(self, msg):
        command = msg.data.strip().lower()

        if command == 'm1':
            if not self.local_position_received:
                self.get_logger().warning(
                    'm1 시작 실패: PX4 local position을 '
                    '아직 받지 못했습니다.'
                )
                return

            if self.mission_state == MISSION_MODE3:
                self.restore_normal_position_variance()

            self.reset_m1_sequence()

            self.ground_z = self.z_hat
            self.takeoff_yaw = (
                self.current_yaw
                if self.attitude_received
                else 0.0
            )

            # MODE1 시작점 기준으로 이륙 목표를 잡는다.
            # X/Y는 이후 ArUco 중심 오차로 계속 갱신된다.
            self.mode1_target_x = self.x_hat
            self.mode1_target_y = self.y_hat
            self.mode1_target_z = (
                self.ground_z - MODE1_TARGET_ALTITUDE
            )
            self.mode1_target_initialized = True

            self.vio.set_takeoff_started(True)
            self.mission_state = MISSION_MODE1

            self.get_logger().warning(
                'm1 시작: ArUco 중심 정렬 + '
                f'{MODE1_TARGET_ALTITUDE:.2f}m 이륙'
            )
            return

        if command == 'm3':
            if not self.local_position_received:
                self.get_logger().warning(
                    'm3 시작 실패: PX4 local position을 '
                    '아직 받지 못했습니다.'
                )
                return

            self.start_mode3()
            return

        if command == 'debug_on':
            debug.set_debug_enabled(True)

            self.get_logger().warning(
                '디버그 화면 켜짐'
            )
            return

        if command == 'debug_off':
            debug.set_debug_enabled(False)

            self.get_logger().warning(
                '디버그 화면 꺼짐'
            )
            return

        if command in (
            'stop',
            'idle',
        ):
            if self.mission_state == MISSION_MODE3:
                self.restore_normal_position_variance()

            self.mission_state = MISSION_IDLE

            self.get_logger().warning(
                '명령 발행 중지: IDLE'
            )
            return

        self.get_logger().warning(
            f'알 수 없는 명령: {command!r} '
            '(m1, m3, stop 사용)'
        )

    def reset_m1_sequence(self):
        self.offboard_prestream_count = 0
        self.command_retry_counter = 0

        self.ground_z = None
        self.takeoff_yaw = None

        self.mode1_target_initialized = False
        self.mode1_marker_id = -1
        self.mode1_error = float('inf')

    def is_offboard(self):
        return (
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_armed(self):
        return (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    def request_modes_until_ready(self):
        self.command_retry_counter += 1

        retry_period = max(
            1,
            int(CONTROL_RATE / 2.0),
        )

        if (
            self.command_retry_counter
            % retry_period
            != 0
        ):
            return

        if not self.is_offboard():
            control.set_offboard()

        if not self.is_armed():
            control.arm()

    def control_callback(self):
        if self.mission_state == MISSION_IDLE:
            return

        if self.mission_state == MISSION_MODE1:
            self.mode1_control()
            return

        if self.mission_state == MISSION_MODE3:
            self.mode3_control()

    def mode1_control(self):
        if not self.mode1_target_initialized:
            return

        aruco_valid = (
            self.vision_valid
            and self.pattern == 0
            and 1 <= self.marker_id <= 50
            and self.vision_altitude > 0.0
            and self.local_position_received
        )

        if aruco_valid:
            if self.mode1_marker_id < 0:
                self.mode1_marker_id = self.marker_id

            if self.marker_id == self.mode1_marker_id:
                dx = self.err_ned_x
                dy = self.err_ned_y
                dz = (
                    self.vision_altitude
                    - MODE1_TARGET_ALTITUDE
                )

                # get_err_xy()가 반환한 값은 이미 기체가
                # 마커 중심으로 이동하기 위한 NED 보정량이다.
                # 현재 PX4 위치에 더해 실제 위치 목표로 사용한다.
                self.mode1_target_x = self.x_hat + dx
                self.mode1_target_y = self.y_hat + dy

                # 고도는 MODE1 시작 시 지면 Z를 기준으로 유지한다.
                self.mode1_target_z = (
                    self.ground_z - MODE1_TARGET_ALTITUDE
                )

                self.mode1_error = math.sqrt(
                    dx * dx
                    + dy * dy
                    + dz * dz
                )

                if (
                    self.mode1_error
                    <= MODE1_ERROR_THRESHOLD
                ):
                    self.get_logger().info(
                        'MODE1 정렬 확인: '
                        f'error={self.mode1_error:.3f}m'
                    )

        self.publish_position_target()

        if (
            self.offboard_prestream_count
            < self.offboard_prestream_target
        ):
            self.offboard_prestream_count += 1
            return

        self.request_modes_until_ready()

    def publish_position_target(self):
        control.cmd_pos_offboard(
            self.x_hat,
            self.y_hat,
            self.z_hat,
            self.mode1_target_x - self.x_hat,
            self.mode1_target_y - self.y_hat,
            self.mode1_target_z - self.z_hat,
            (
                self.takeoff_yaw
                if self.takeoff_yaw is not None
                else self.current_yaw
            ),
        )

    def start_mode3(self):
        if not self.mode3_targets:
            self.get_logger().warning(
                'MODE3 목표가 없습니다.'
            )
            return

        self.mode3_target_index = 0
        self.mode3_complete = False
        self.mode3_sub_state = 'moving'
        self.mode3_hover_start_time = None

        self.reset_mode3_variance_state()

        if self.takeoff_yaw is None:
            self.takeoff_yaw = (
                self.current_yaw
                if self.attitude_received
                else 0.0
            )

        self.mode3_yaw_target = self.takeoff_yaw

        self.set_mode3_target()
        self.mission_state = MISSION_MODE3

        self.get_logger().warning(
            f'MODE3 시작: 목표 '
            f'{len(self.mode3_targets)}개, '
            '라인 미확정 분산 '
            f'X={VIO_VARIANCE_HIGH_X}, '
            f'Y={VIO_VARIANCE_HIGH_Y}, '
            f'Z={VIO_VARIANCE_LOW_Z}'
        )

    def mode3_control(self):
        if not self.local_position_received:
            return

        self.publish_mode3_target()

        if self.mode3_complete:
            return

        if self.mode3_sub_state == 'moving':
            dx = self.mode3_target_x - self.x_hat
            dy = self.mode3_target_y - self.y_hat
            dz = self.mode3_target_z - self.z_hat

            position_error = math.sqrt(
                dx * dx
                + dy * dy
                + dz * dz
            )

            if (
                position_error
                <= MODE3_ARRIVAL_THRESHOLD
            ):
                self.mode3_sub_state = 'grid_align'

                self.get_logger().info(
                    'MODE3 고정 좌표 도착: '
                    f'{self.mode3_target_index + 1}/'
                    f'{len(self.mode3_targets)}, '
                    '격자 확인 시작'
                )
            return

        if self.mode3_sub_state == 'grid_align':
            if (
                not self.grid_detected
                or not self.vision_position_valid
            ):
                return

            grid_error = math.hypot(
                self.body_err_x,
                self.body_err_y,
            )

            altitude_error = 0.0
            altitude_aligned = True

            if self.vision_altitude_valid:
                desired_altitude = abs(
                    float(self.mode3_target_z)
                )

                altitude_error = (
                    self.vision_altitude
                    - desired_altitude
                )

                altitude_aligned = (
                    abs(altitude_error)
                    <= MODE3_ALTITUDE_ERROR_THRESHOLD
                )

            attitude_aligned = False

            if self.vision_attitude_valid:
                attitude_aligned = (
                    abs(self.vision_roll)
                    <= MODE3_ROLL_PITCH_THRESHOLD_RAD
                    and abs(self.vision_pitch)
                    <= MODE3_ROLL_PITCH_THRESHOLD_RAD
                    and abs(self.vision_yaw)
                    <= MODE3_YAW_THRESHOLD_RAD
                )

            if (
                grid_error
                <= MODE3_GRID_ERROR_THRESHOLD
                and altitude_aligned
                and attitude_aligned
            ):
                self.mode3_sub_state = 'hovering'
                self.mode3_hover_start_time = (
                    self.get_clock().now().nanoseconds
                    / 1e9
                )

                self.get_logger().info(
                    'MODE3 격자 위치 확인: '
                    f'xy={grid_error:.3f}m, '
                    f'z={abs(altitude_error):.3f}m, '
                    f'roll='
                    f'{math.degrees(self.vision_roll):+.1f}deg, '
                    f'pitch='
                    f'{math.degrees(self.vision_pitch):+.1f}deg, '
                    f'yaw='
                    f'{math.degrees(self.vision_yaw):+.1f}deg, '
                    f'{MODE3_HOVER_SEC:.1f}초 호버링 시작'
                )
            return

        if self.mode3_sub_state == 'hovering':
            now_sec = (
                self.get_clock().now().nanoseconds
                / 1e9
            )

            if self.mode3_hover_start_time is None:
                self.mode3_hover_start_time = now_sec
                return

            if (
                now_sec
                - self.mode3_hover_start_time
                < MODE3_HOVER_SEC
            ):
                return

            self.get_logger().info(
                'MODE3 호버링 완료: '
                f'{self.mode3_target_index + 1}/'
                f'{len(self.mode3_targets)}'
            )

            self.mode3_target_index += 1

            if (
                self.mode3_target_index
                >= len(self.mode3_targets)
            ):
                self.mode3_complete = True

                self.restore_normal_position_variance()

                self.get_logger().warning(
                    'MODE3 모든 고정 목표 방문 및 '
                    '호버링 완료, 정상 분산 복원'
                )
                return

            self.set_mode3_target()
            self.mode3_sub_state = 'moving'
            self.mode3_hover_start_time = None

            self.reset_mode3_variance_state()

    def set_mode3_target(self):
        (
            self.mode3_target_x,
            self.mode3_target_y,
            self.mode3_target_z,
        ) = self.mode3_targets[
            self.mode3_target_index
        ]

        self.get_logger().info(
            'MODE3 목표 설정: '
            f'x={self.mode3_target_x:.2f}, '
            f'y={self.mode3_target_y:.2f}, '
            f'z={self.mode3_target_z:.2f}'
        )

    def publish_mode3_target(self):
        if (
            self.vision_attitude_valid
            and self.attitude_received
        ):
            self.mode3_yaw_target = (
                self.current_yaw
                + self.vision_yaw
            )
        elif self.takeoff_yaw is not None:
            self.mode3_yaw_target = (
                self.takeoff_yaw
            )
        else:
            self.mode3_yaw_target = (
                self.current_yaw
            )

        control.cmd_pos_offboard(
            self.x_hat,
            self.y_hat,
            self.z_hat,
            self.mode3_target_x - self.x_hat,
            self.mode3_target_y - self.y_hat,
            self.mode3_target_z - self.z_hat,
            self.mode3_yaw_target,
        )

    def reset_mode3_variance_state(self):
        self.mode3_confirmed_direction = None
        self.mode3_candidate_direction = None
        self.mode3_line_confirm_count = 0
        self.mode3_line_lost_count = 0

        self.apply_mode3_position_variance()

    def normalize_mode3_line_direction(
        self,
        detected_direction,
    ):
        if detected_direction is None:
            return None

        direction = str(
            detected_direction
        ).strip().lower()

        if direction in (
            'ver',
            'vertical',
            'v',
        ):
            return 'ver'

        if direction in (
            'hor',
            'horizontal',
            'h',
        ):
            return 'hor'

        return None

    def update_mode3_line_direction(
        self,
        detected_direction,
    ):
        direction = (
            self.normalize_mode3_line_direction(
                detected_direction
            )
        )

        if direction is not None:
            self.mode3_line_lost_count = 0

            if (
                direction
                == self.mode3_candidate_direction
            ):
                self.mode3_line_confirm_count += 1
            else:
                self.mode3_candidate_direction = (
                    direction
                )
                self.mode3_line_confirm_count = 1

            if (
                self.mode3_line_confirm_count
                >= MODE3_LINE_CONFIRM_COUNT
                and self.mode3_confirmed_direction
                != self.mode3_candidate_direction
            ):
                self.mode3_confirmed_direction = (
                    self.mode3_candidate_direction
                )

                self.apply_mode3_position_variance()

                self.get_logger().info(
                    'MODE3 라인 방향 확정: '
                    f'{self.mode3_confirmed_direction}'
                )

        else:
            self.mode3_line_lost_count += 1

            if (
                self.mode3_line_lost_count
                >= MODE3_LINE_LOST_COUNT
            ):
                changed = (
                    self.mode3_confirmed_direction
                    is not None
                )

                self.mode3_confirmed_direction = None
                self.mode3_candidate_direction = None
                self.mode3_line_confirm_count = 0

                self.apply_mode3_position_variance()

                if changed:
                    self.get_logger().warning(
                        'MODE3 라인 소실: '
                        'X/Y 큰 분산으로 전환'
                    )

        return self.mode3_confirmed_direction

    def apply_mode3_position_variance(self):
        if self.mode3_confirmed_direction == 'ver':
            variance_x = VIO_VARIANCE_HIGH_X
            variance_y = VIO_VARIANCE_LOW_Y
            variance_z = VIO_VARIANCE_LOW_Z

        elif self.mode3_confirmed_direction == 'hor':
            variance_x = VIO_VARIANCE_LOW_X
            variance_y = VIO_VARIANCE_HIGH_Y
            variance_z = VIO_VARIANCE_LOW_Z

        else:
            variance_x = VIO_VARIANCE_HIGH_X
            variance_y = VIO_VARIANCE_HIGH_Y
            variance_z = VIO_VARIANCE_LOW_Z

        self.vio.set_position_variance(
            variance_x,
            variance_y,
            variance_z,
        )

        return (
            variance_x,
            variance_y,
            variance_z,
        )

    def restore_normal_position_variance(self):
        self.vio.set_position_variance(
            VIO_VARIANCE_LOW_X,
            VIO_VARIANCE_LOW_Y,
            VIO_VARIANCE_LOW_Z,
        )

    def extract_line_direction(
        self,
        measurement,
    ):
        direct_value = measurement.get(
            'line_direction'
        )

        normalized = (
            self.normalize_mode3_line_direction(
                direct_value
            )
        )

        if normalized is not None:
            return normalized

        line_data = measurement.get(
            'line_data'
        )

        if isinstance(line_data, dict):
            for key in (
                'direction',
                'line_direction',
                'orientation',
                'mode',
                'type',
            ):
                normalized = (
                    self.normalize_mode3_line_direction(
                        line_data.get(key)
                    )
                )

                if normalized is not None:
                    return normalized

        return None

    def local_position_callback(self, msg):
        self.x_hat = float(msg.x)
        self.y_hat = float(msg.y)
        self.z_hat = float(msg.z)
        self.local_position_received = True

    def attitude_callback(self, msg):
        try:
            (
                self.current_roll,
                self.current_pitch,
                self.current_yaw,
            ) = quaternion_to_euler(
                float(msg.q[0]),
                float(msg.q[1]),
                float(msg.q[2]),
                float(msg.q[3]),
            )

        except (
            IndexError,
            TypeError,
            ValueError,
        ) as exc:
            self.get_logger().warning(
                f'기체 quaternion 변환 실패: {exc}'
            )
            return

        self.attitude_received = True

    def status_callback(self, msg):
        self.vehicle_status = msg

    def vision_callback(self):
        frame, mask = camera.get_frame()

        if frame is None or mask is None:
            self.vision_valid = False
            self.vision_position_valid = False
            self.vision_altitude_valid = False
            self.vision_attitude_valid = False
            self.grid_detected = False
            self.line_detected = False
            self.vision_source = 'none'
            self.detected_line_direction = None

            if self.mission_state == MISSION_MODE1:
                self.vio.set_position_variance(
                    VIO_POSITION_VARIANCE_NONE[0],
                    VIO_POSITION_VARIANCE_NONE[1],
                    VIO_POSITION_VARIANCE_NONE[2],
                )

            if self.mission_state == MISSION_MODE3:
                self.update_mode3_line_direction(
                    None
                )

            debug.show_debug(
                None,
                None,
                -1,
                -1,
            )
            return

        pattern, marker_id = camera.get_pattern(
            frame,
            mask,
        )

        align_grid = (
            self.mission_state == MISSION_MODE3
            and self.mode3_sub_state in (
                'grid_align',
                'hovering',
            )
        )

        measurement = vision.get_vision_measurement(
            pattern,
            marker_id,
            mask,
            frame,
            self.table,
            align_pos=align_grid,
        )

        self.pattern = int(pattern)
        self.marker_id = int(marker_id)

        self.vision_altitude = float(
            measurement['altitude']
        )
        self.body_err_x = float(
            measurement['err_x']
        )
        self.body_err_y = float(
            measurement['err_y']
        )

        self.vision_valid = bool(
            measurement['vision_valid']
        )

        self.vision_roll = float(
            measurement['roll']
        )
        self.vision_pitch = float(
            measurement['pitch']
        )
        self.vision_yaw = float(
            measurement['yaw']
        )

        self.vision_attitude_valid = bool(
            measurement['attitude_valid']
        )
        self.vision_position_valid = bool(
            measurement['position_valid']
        )
        self.vision_altitude_valid = bool(
            measurement['altitude_valid']
        )

        vision_angle_deg = max(
            abs(math.degrees(self.vision_roll)),
            abs(math.degrees(self.vision_pitch)),
        )

        self.table['current_angle_deg'] = int(
            max(
                0,
                min(
                    30,
                    round(
                        vision_angle_deg / 5.0
                    ) * 5,
                ),
            )
        )

        self.grid_detected = (
            1 <= self.pattern <= 9
        )

        self.line_detected = (
            self.pattern == -1
            and self.vision_position_valid
        )

        if (
            self.pattern == 0
            and 1 <= self.marker_id <= 50
        ):
            self.vision_source = 'aruco'

        elif self.grid_detected:
            self.vision_source = 'grid'

        elif self.line_detected:
            self.vision_source = 'line'

        else:
            self.vision_source = 'none'

        if self.attitude_received:
            (
                self.err_ned_x,
                self.err_ned_y,
            ) = body_xy_to_ned(
                self.body_err_x,
                self.body_err_y,
                self.current_yaw,
            )
        else:
            self.err_ned_x = 0.0
            self.err_ned_y = 0.0

        if self.mission_state == MISSION_MODE1:
            aruco_visible = (
                self.pattern == 0
                and 1 <= self.marker_id <= 50
                and self.vision_position_valid
            )

            if aruco_visible:
                self.vio.set_position_variance(
                    VIO_VARIANCE_LOW_X,
                    VIO_VARIANCE_LOW_Y,
                    VIO_VARIANCE_LOW_Z,
                )
            else:
                self.vio.set_position_variance(
                    VIO_POSITION_VARIANCE_NONE[0],
                    VIO_POSITION_VARIANCE_NONE[1],
                    VIO_POSITION_VARIANCE_NONE[2],
                )

        if self.mission_state == MISSION_MODE3:
            if (
                self.mode3_sub_state == 'moving'
                and self.line_detected
            ):
                self.detected_line_direction = (
                    self.extract_line_direction(
                        measurement
                    )
                )
            else:
                self.detected_line_direction = None

            self.update_mode3_line_direction(
                self.detected_line_direction
            )

        if self.vision_altitude_valid:
            self.vio.update(
                x=-self.err_ned_x,
                y=-self.err_ned_y,
                altitude=self.vision_altitude,
            )

        debug.show_debug(
            frame,
            mask,
            self.pattern,
            self.marker_id,
            altitude=self.vision_altitude,
            err_x=self.body_err_x,
            err_y=self.body_err_y,
            roll=self.current_roll,
            pitch=self.current_pitch,
            yaw=self.current_yaw,
        )

    def current_height(self):
        if (
            self.vision_valid
            and self.vision_altitude > 0.0
        ):
            return self.vision_altitude

        if (
            self.ground_z is not None
            and self.local_position_received
        ):
            return (
                self.ground_z
                - self.z_hat
            )

        return None

    @staticmethod
    def load_vision_table(csv_path):
        with open(
            csv_path,
            'r',
            encoding='utf-8-sig',
            newline='',
        ) as csv_file:
            rows = list(
                csv.reader(csv_file)
            )

        table = {
            'aruco': [],
            'grid': {
                'L': [],
                'T': [],
                'X': [],
            },
            'altitude_ver': {},
            'altitude_hor': {},
            'angle_ver': None,
            'angle_hor': None,
            'current_angle_deg': 0,
        }

        def parse_line_section(section_name):
            try:
                section_index = next(
                    index
                    for index, row in enumerate(rows)
                    if (
                        row
                        and row[0].strip().lower()
                        == section_name
                    )
                )
            except StopIteration:
                return {}, None

            header = rows[
                section_index + 1
            ]

            angles = []
            columns = []

            for column in range(
                1,
                len(header),
                3,
            ):
                value = (
                    header[column].strip()
                    if column < len(header)
                    else ''
                )

                if not value:
                    continue

                try:
                    angles.append(
                        int(float(value))
                    )
                    columns.append(column)
                except ValueError:
                    continue

            by_angle = {
                angle: []
                for angle in angles
            }

            altitude_rows = []
            row_index = section_index + 2

            while row_index < len(rows):
                row = rows[row_index]

                if (
                    not row
                    or not row[0].strip()
                ):
                    break

                try:
                    altitude = float(row[0])
                except ValueError:
                    break

                row_values = {}

                for angle, column in zip(
                    angles,
                    columns,
                ):
                    try:
                        top = float(
                            row[column]
                        )
                        middle = float(
                            row[column + 1]
                        )
                        bottom = float(
                            row[column + 2]
                        )
                    except (
                        IndexError,
                        ValueError,
                    ):
                        continue

                    by_angle[angle].append(
                        (
                            altitude,
                            top,
                            middle,
                            bottom,
                        )
                    )

                    row_values[angle] = (
                        top,
                        middle,
                        bottom,
                    )

                altitude_rows.append(
                    (
                        altitude,
                        row_values,
                    )
                )

                row_index += 1

            angle_table = None

            if altitude_rows:
                _, nearest = min(
                    altitude_rows,
                    key=lambda item: abs(
                        item[0] - 2.0
                    ),
                )

                valid_angles = [
                    angle
                    for angle in angles
                    if angle in nearest
                ]

                angle_table = (
                    valid_angles,
                    [
                        nearest[angle][0]
                        for angle in valid_angles
                    ],
                    [
                        nearest[angle][1]
                        for angle in valid_angles
                    ],
                    [
                        nearest[angle][2]
                        for angle in valid_angles
                    ],
                )

            return by_angle, angle_table

        (
            table['altitude_ver'],
            table['angle_ver'],
        ) = parse_line_section('ver')

        (
            table['altitude_hor'],
            table['angle_hor'],
        ) = parse_line_section('hor')

        try:
            aru_index = next(
                index
                for index, row in enumerate(rows)
                if (
                    row
                    and row[0].strip().lower()
                    == 'aru'
                )
            )

        except StopIteration as exc:
            raise RuntimeError(
                'ArUco/격자 테이블을 '
                f'찾지 못했습니다: {csv_path}'
            ) from exc

        for row in rows[
            aru_index + 1:
        ]:
            if (
                not row
                or not row[0].strip()
            ):
                continue

            try:
                altitude = float(row[0])
                aruco_area = float(row[1])
                l_pixels = float(row[2])
                t_pixels = float(row[3])
                x_pixels = float(row[4])

            except (
                IndexError,
                ValueError,
            ):
                break

            table['aruco'].append(
                (
                    altitude,
                    aruco_area,
                )
            )

            table['grid']['L'].append(
                (
                    altitude,
                    l_pixels,
                )
            )

            table['grid']['T'].append(
                (
                    altitude,
                    t_pixels,
                )
            )

            table['grid']['X'].append(
                (
                    altitude,
                    x_pixels,
                )
            )

        if not table['aruco']:
            raise RuntimeError(
                '비전 테이블을 읽지 '
                f'못했습니다: {csv_path}'
            )

        table['aruco'].sort(
            key=lambda item: item[0]
        )

        for group in table['grid'].values():
            group.sort(
                key=lambda item: item[0]
            )

        for section in (
            'altitude_ver',
            'altitude_hor',
        ):
            for entries in table[
                section
            ].values():
                entries.sort(
                    key=lambda item: item[0]
                )

        return table

    @staticmethod
    def load_mode3_table(csv_path):
        targets = []

        with open(
            csv_path,
            'r',
            encoding='utf-8-sig',
            newline='',
        ) as csv_file:
            reader = csv.DictReader(
                csv_file
            )

            for row_number, row in enumerate(
                reader,
                start=2,
            ):
                try:
                    targets.append(
                        (
                            float(row['x']),
                            float(row['y']),
                            float(row['z']),
                        )
                    )

                except (
                    KeyError,
                    TypeError,
                    ValueError,
                ) as exc:
                    raise RuntimeError(
                        'MODE3 CSV '
                        f'{row_number}행 형식 오류: '
                        f'{row}'
                    ) from exc

        if not targets:
            raise RuntimeError(
                'MODE3 목표를 읽지 '
                f'못했습니다: {csv_path}'
            )

        return targets

    def destroy_node(self):
        camera.close_cam()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MainNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
