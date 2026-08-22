from camera import Camera
from centering import Centering
from aruco_detector import detect_aruco, compute_center_offset
from aruco_detector import compute_center_offset_m
from camera_line_scanning import process_frame
from grid_centering import GridCentering
from pattern_detector import PatternSmoother
from dijkstra import load_table_from_csv
from line_table_io import load_aruco_target_area
from altitude_estimator import load_aruco_altitude_table, estimate_altitude_from_aruco

import csv
import math
import struct
import time
import numpy as np
import rclpy
from rclpy.node import Node

from pymavlink import mavutil

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry


def clamp_z_setpoint_to_safety(ground_z, z_setpoint, max_height_m):
    if ground_z is None:
        return z_setpoint
    min_allowed_z = ground_z - max_height_m
    return max(z_setpoint, min_allowed_z)

MODE1_HOLD_YAW_NED = float('nan')
MODE1_BLIND_CLIMB_STEP = 0.01
MODE1_XY_MAX_STEP = 1
MODE1_BLIND_MAX_HEIGHT = 2.0

MODE1_XY_P_TARGET = 3
MODE1_Z_P_TARGET = 4
MODE1_XY_VEL_I_TARGET = 0
MODE1_XY_VEL_D_TARGET = 1.5
MODE1_Z_VEL_I_TARGET = 0
MODE1_Z_VEL_D_TARGET = 2

# 추가된 부분: mode11도 mode1과 같은 값으로 시작하지만, 이제 별도
# 상수로 분리해서 mode1 값을 안 건드리고 mode11만 따로 튜닝할 수 있게 함
MODE11_XY_P_TARGET = 2
MODE11_Z_P_TARGET = 4
MODE11_XY_VEL_I_TARGET = 0
MODE11_XY_VEL_D_TARGET = 1.3
MODE11_Z_VEL_I_TARGET = 0
MODE11_Z_VEL_D_TARGET = 2

# 추가된 부분: mode11의 격자점 이동 - mode3와 동일한 CSV/웨이포인트를
# 재사용하되, 도착 판정은 mode3의 비전 패턴 방식이 아니라 "setpoint까지
# 거리"로 단순하게 한다. 도착하면 10초(0.1s/tick * 100) 호버링 후 다음 점.
MODE11_ARRIVAL_TOLERANCE_M = 0.3
MODE11_HOVER_TICKS = 100
# 추가된 부분: 이륙(2.0m 도달) 완료 직후, 격자점으로 출발하기 전에
# 안정화를 위해 대기하는 시간(10초 = 100틱)
MODE11_TAKEOFF_HOVER_TICKS = 100
# 수정된 부분(핵심): mode11은 mode1의 NaN(현재값 유지)도, mode3의
# wrap_pi(pi/2) 고정값도 아니라 0으로 고정한다. camera_line_scanning.py가
# vio_pub을 통해 라인 기반 yaw 추정치를 EKF2에 계속 넣어주고 있으므로,
# 우리가 매 틱 목표 yaw=0을 계속 명령하면 PX4가 그 오차를 스스로
# 보정한다 - 별도로 yaw 보정값을 여기서 구독/계산할 필요는 없다.
MODE11_YAW_NED = 0.0

MODE1_XY_ALIGN_TOLERANCE_M = 0
MODE10_HEIGHT_TOLERANCE = 0.1

MODE12_BLIND_CLIMB_STEP = 0.01
MODE12_HEIGHT_TOLERANCE = 0.1

AUTO_TAKEOFF_HEIGHT = 2.0

MODE2_FORWARD_DISTANCE_M = 5.5
MODE2_TILTMAX_AIR_TARGET = 20.0
MODE2_XY_P_TARGET = 3
MODE2_XY_VEL_I_TARGET = 0.0
MODE2_XY_VEL_D_TARGET = 1.5
MODE2_UNKNOWN_STREAK_REQUIRED = 20
MODE2_DETECTION_DELAY_SEC = 3
MODE2_PASS_STREAK_REQUIRED = MODE2_UNKNOWN_STREAK_REQUIRED
MODE2_SEARCH_INITIAL_PITCH_DEG = 30.0
MODE2_SEARCH_PITCH_STEP_DEG = 5.0
# 감속 시작 시점(피치=30도)의 전방 목표 거리(m). 감속될수록 비례해서 줄어듦.
MODE2_SEARCH_LOOKAHEAD_M = 1.0

# hover 단계에서 grid/아루코로 미세보정하는 최대 틱 수(0.1s/tick 기준 3초)
MODE2_FINE_CORRECT_MAX_TICKS = 30
MODE2_GRID_COMPUTE_INTERVAL = 10

# 수정된 부분(핵심, 확인 필요): 카메라 이미지 오프셋(dx, dy)을 드론 NED
# 위치 setpoint(x=전후, y=좌우)로 변환할 때의 축 매핑/부호.
# vio_publisher.py에서 "이미지 세로축(dy)=드론 전후, 가로축(dx)=드론 좌우"로
# 확인된 축 매핑을 그대로 재사용하지만(같은 카메라니까), 부호는
# vio_publisher 내부의 별도 반전 로직과 무관하게 여기서 새로 검증해야 한다.
# 시뮬레이션으로 실제 이동 방향 보고 반대면 -1로 뒤집을 것.
MODE2_FORWARD_OFFSET_SIGN = 1.0   # dy(이미지 세로) -> x(전후) 부호
MODE2_LATERAL_OFFSET_SIGN = 1.0   # dx(이미지 가로) -> y(좌우) 부호

# 수정된 부분(핵심): mode11을 두 개로 분리했다.
# - mode3: 격자점(또는 현재 위치)에서 순수 호버링만 담당. 자동 진행 없음.
# - mode4: /mode_command로 'mode4'가 새로 들어올 때마다 테이블(경로 CSV)의
#   다음 격자점으로 딱 한 칸만 이동(=setpoint 전송)하고, 도착판정/호버링은
#   하지 않는다. 호버링이 필요하면 이어서 'mode3'을 보내면 된다.
# mode1/mode11과 동일한 패턴으로, 각자 독립된 P/I/D 게인 상수 세트를 쓴다.
MODE3_XY_P_TARGET = 3
MODE3_Z_P_TARGET = 4
MODE3_XY_VEL_I_TARGET = 0
MODE3_XY_VEL_D_TARGET = 1.5
MODE3_Z_VEL_I_TARGET = 0
MODE3_Z_VEL_D_TARGET = 2
MODE3_HOLD_YAW_NED = float('nan')  # 진입 시점의 yaw를 그대로 유지

MODE4_XY_P_TARGET = 2
MODE4_Z_P_TARGET = 4
MODE4_XY_VEL_I_TARGET = 0
MODE4_XY_VEL_D_TARGET = 1.3
MODE4_Z_VEL_I_TARGET = 0
MODE4_Z_VEL_D_TARGET = 2

MODE4_PATH_RESULT_CSV = '/home/jiseungwoo/Desktop/com1/PX4-Autopilot/navigation/path_result.csv'
MODE4_WAYPOINT_CSV = '/home/jiseungwoo/Desktop/com1/PX4-Autopilot/navigation/way_point.csv'


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('baylands_waypoint_follower_odom')

        self.target_aruco_area = load_aruco_target_area('line_table.csv')
        if self.target_aruco_area is not None:
            self.get_logger().info(f'목표 아루코 마커 면적(area_px): {self.target_aruco_area}')
        else:
            self.get_logger().warn(
                'line_table.csv에서 아루코 목표 면적을 찾지 못했습니다 '
                '(mode1 / mode3 마커 z보정이 동작하지 않습니다)')

        # 추가된 부분(핵심): mode1의 고도 판단 기준을 EKF2 fused pos[2]가
        # 아니라 아루코 마커 픽셀 면적(=카메라가 직접 보는 값)으로 바꾸기
        # 위한 테이블. camera_line_scanning.py에서 쓰는 것과 동일한
        # (alt, area) 테이블을 여기서도 그대로 로드해서 쓴다.
        self.aruco_altitude_table = load_aruco_altitude_table('line_table.csv')
        if not self.aruco_altitude_table:
            self.get_logger().warn(
                'line_table.csv에서 아루코 고도 테이블(aru 블록)을 못 찾음 - '
                'mode1이 픽셀 기반 고도를 못 쓰고 EKF2로 폴백합니다.')

        self.odom = None
        self.status = None
        self.counter = 0

        self._init_mavlink_param_link()
        self.ground_z = None

        self.active_mode = None
        self.arm_seq_counter = 0

        self.mode1_hold_x = None
        self.mode1_hold_y = None
        self.mode1_z_setpoint = None
        self.mode1_prev_height = None
        self.mode1_reached_target = False
        self.mode1_aruco_first_detected = False
        # 추가된 부분: 목표 도달 후 고도가 안전밴드(1.5~2.5m)를 벗어나면
        # 다시 목표 고도(2.0m)로 재보정하는 상태 플래그
        self.mode1_correcting = False
        # 추가된 부분: x/y 오차가 5cm 이내인지(정렬 여부) 상태 추적용
        self.mode1_xy_aligned = False

        self.mode10_hold_x = None
        self.mode10_hold_y = None
        self.mode10_z_setpoint = None

        self.mode11_hold_x = None
        self.mode11_hold_y = None
        self.mode11_z_setpoint = None
        self.mode11_prev_height = None
        self.mode11_reached_target = False
        # 추가된 부분: mode3와 동일한 방식(build_mode3_waypoints)으로
        # 격자점 경로를 받아와서, 이륙+호버링 이후 각 웨이포인트로
        # 이동 -> 도착(setpoint 거리 기준) -> 10초 호버링 -> 다음 점
        # 순서로 진행한다. 좌표축/부호 확인이나 아루코-grid 세부 로직은
        # 나중에 추가 예정 - 지금은 순수 위치이동+정지호버링만.
        self.mode11_waypoints = None
        self.mode11_index = 0
        self.mode11_mission_complete = False
        self.mode11_hold_wp = None
        self.mode11_hovering = False
        self.mode11_hover_timer = 0
        # 추가된 부분: 이륙 완료 후 격자점 이동 시작 전 대기용 타이머
        self.mode11_takeoff_hover_start = None

        self.mode12_hold_x = None
        self.mode12_hold_y = None
        self.mode12_z_setpoint = None
        self.mode12_hgt_ref_backup = None
        self.mode12_reached_target = False
        self.mode12_prev_height = None
        self.line_first_detected = False
        self.create_subscription(
            Bool, '/vision/line_first_detected',
            self._line_first_detected_callback, 10)

        self.mode2_start_x = None
        self.mode2_y_setpoint = None
        self.mode2_z_setpoint = None
        self.mode2_hold_x = None
        self.mode2_hold_y = None
        self.mode2_stopped = False
        self.mode2_armed = False
        self.mode2_unknown_streak = 0
        self.mode2_pattern_smoother = None
        self.mode2_traveled_m = 0.0
        # 수정된 부분: 카운트다운(틱 수) 대신 지금 명령 중인 피치 각도
        # 자체를 상태로 들고 있는다 (30도 -> 5도씩 감소 -> 0도)
        self.mode2_current_pitch_deg = 0.0
        self.mode2_start_time = None

        # 추가된 부분(핵심): mode2 상태머신(blind -> align_h -> approach_v -> hover)
        self.mode2_phase = 'blind'
        self.mode2_pass_streak = 0
        self.mode2_fine_correct_ticks = 0
        self.mode2_grid_centering = GridCentering()

        # mode3: 격자점(또는 임의 위치)에서 mode11과 동일한 이륙 로직
        # (블라인드로 AUTO_TAKEOFF_HEIGHT까지 상승) 후 순수 호버링만
        # 하는 모드. 진입 시점의 x/y를 그대로 hold하고 계속 유지한다.
        self.mode3_hold_x = None
        self.mode3_hold_y = None
        self.mode3_z_setpoint = None
        self.mode3_prev_height = None
        self.mode3_reached_target = False

        # mode4: mode_command로 'mode4'가 들어올 때마다 테이블(경로 CSV)의
        # 다음 격자점으로 한 칸씩 이동. 이동만 담당하고 도착판정/호버링은
        # 하지 않는다(호버링은 사용자가 이어서 mode3을 보내서 수행).
        self.mode4_waypoints = None
        self.mode4_initialized = False
        self.mode4_index = 0
        self.mode4_z_setpoint = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.mode11_xy_gain_pub = self.create_publisher(
            Float32, '/mode11/xy_gain', 10)
        self.mode1_xy_gain_pub = self.create_publisher(
            Float32, '/mode1/xy_gain', 10)

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.create_subscription(
            String, '/mode_command',
            self.mode_command_callback, 10)

        self.cam = Camera()
        self.centering = Centering(self.cam)
        self.speed_set = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cam_timer = self.create_timer(0.1, self.centering_tick)
        self.speed_timer = self.create_timer(1.0, self.set_speed_limit)

    def _line_first_detected_callback(self, msg):
        self.line_first_detected = bool(msg.data)

    def mode_command_callback(self, msg):
        cmd = msg.data.strip().lower()

        # 복원된 부분: mode1 이탈 시 게인 복원. 진입 시 강제 억제(잠금)를
        # 안 하므로, "released 여부와 무관하게 백업이 있으면 항상
        # 원래값으로 되돌린다"는 단순한 규칙으로 통일한다 (아루코 검출
        # 안 됐으면 애초에 안 바뀌었으니 복원해도 그대로, 검출됐으면
        # 목표값 -> 원래값으로 되돌아감).
        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_xy_p_backup is not None:
            self.set_px4_param('MPC_XY_P', self._mode1_xy_p_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_XY_P을 '
                f'{self._mode1_xy_p_backup}(원래값)으로 복원')
            self._mode1_xy_p_backup = None
            self._mode1_xy_p_released = False

        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_z_p_backup is not None:
            self.set_px4_param('MPC_Z_P', self._mode1_z_p_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_Z_P을 '
                f'{self._mode1_z_p_backup}(원래값)으로 복원')
            self._mode1_z_p_backup = None
            self._mode1_z_p_released = False

        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_xy_vel_i_backup is not None:
            self.set_px4_param('MPC_XY_VEL_I_ACC', self._mode1_xy_vel_i_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_XY_VEL_I_ACC을 '
                f'{self._mode1_xy_vel_i_backup}(원래값)으로 복원')
            self._mode1_xy_vel_i_backup = None
            self._mode1_xy_vel_i_released = False

        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_xy_vel_d_backup is not None:
            self.set_px4_param('MPC_XY_VEL_D_ACC', self._mode1_xy_vel_d_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_XY_VEL_D_ACC을 '
                f'{self._mode1_xy_vel_d_backup}(원래값)으로 복원')
            self._mode1_xy_vel_d_backup = None
            self._mode1_xy_vel_d_released = False

        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_z_vel_i_backup is not None:
            self.set_px4_param('MPC_Z_VEL_I_ACC', self._mode1_z_vel_i_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_Z_VEL_I_ACC을 '
                f'{self._mode1_z_vel_i_backup}(원래값)으로 복원')
            self._mode1_z_vel_i_backup = None
            self._mode1_z_vel_i_released = False

        if self.active_mode == 'mode1' and cmd != 'mode1' \
                and self._mode1_z_vel_d_backup is not None:
            self.set_px4_param('MPC_Z_VEL_D_ACC', self._mode1_z_vel_d_backup)
            self.get_logger().info(
                f'[param] mode1 이탈 - MPC_Z_VEL_D_ACC을 '
                f'{self._mode1_z_vel_d_backup}(원래값)으로 복원')
            self._mode1_z_vel_d_backup = None
            self._mode1_z_vel_d_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_tiltmax_air_backup is not None:
            self.set_px4_param('MPC_TILTMAX_AIR', self._mode11_tiltmax_air_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_TILTMAX_AIR을 '
                f'{self._mode11_tiltmax_air_backup}(원래값)으로 복원')
            self._mode11_tiltmax_air_backup = None

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_xy_p_backup is not None:
            self.set_px4_param('MPC_XY_P', self._mode11_xy_p_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_XY_P을 '
                f'{self._mode11_xy_p_backup}(원래값)으로 복원')
            self._mode11_xy_p_backup = None
            self._mode11_xy_p_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_z_p_backup is not None:
            self.set_px4_param('MPC_Z_P', self._mode11_z_p_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_Z_P을 '
                f'{self._mode11_z_p_backup}(원래값)으로 복원')
            self._mode11_z_p_backup = None
            self._mode11_z_p_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_xy_vel_i_backup is not None:
            self.set_px4_param('MPC_XY_VEL_I_ACC', self._mode11_xy_vel_i_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_XY_VEL_I_ACC을 '
                f'{self._mode11_xy_vel_i_backup}(원래값)으로 복원')
            self._mode11_xy_vel_i_backup = None
            self._mode11_xy_vel_i_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_xy_vel_d_backup is not None:
            self.set_px4_param('MPC_XY_VEL_D_ACC', self._mode11_xy_vel_d_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_XY_VEL_D_ACC을 '
                f'{self._mode11_xy_vel_d_backup}(원래값)으로 복원')
            self._mode11_xy_vel_d_backup = None
            self._mode11_xy_vel_d_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_z_vel_i_backup is not None:
            self.set_px4_param('MPC_Z_VEL_I_ACC', self._mode11_z_vel_i_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_Z_VEL_I_ACC을 '
                f'{self._mode11_z_vel_i_backup}(원래값)으로 복원')
            self._mode11_z_vel_i_backup = None
            self._mode11_z_vel_i_released = False

        if self.active_mode == 'mode11' and cmd != 'mode11' \
                and self._mode11_z_vel_d_backup is not None:
            self.set_px4_param('MPC_Z_VEL_D_ACC', self._mode11_z_vel_d_backup)
            self.get_logger().info(
                f'[param] mode11 이탈 - MPC_Z_VEL_D_ACC을 '
                f'{self._mode11_z_vel_d_backup}(원래값)으로 복원')
            self._mode11_z_vel_d_backup = None
            self._mode11_z_vel_d_released = False


        if self.active_mode == 'mode2' and cmd != 'mode2':
            if self._mode2_tiltmax_air_backup is not None:
                self.set_px4_param('MPC_TILTMAX_AIR', self._mode2_tiltmax_air_backup)
                self.get_logger().info(
                    f'[param] mode2 이탈 - MPC_TILTMAX_AIR을 '
                    f'{self._mode2_tiltmax_air_backup}으로 복원')
                self._mode2_tiltmax_air_backup = None
            if self._mode2_xy_p_backup is not None:
                self.set_px4_param('MPC_XY_P', self._mode2_xy_p_backup)
                self.get_logger().info(
                    f'[param] mode2 이탈 - MPC_XY_P을 {self._mode2_xy_p_backup}으로 복원')
                self._mode2_xy_p_backup = None
            if self._mode2_xy_vel_i_backup is not None:
                self.set_px4_param('MPC_XY_VEL_I_ACC', self._mode2_xy_vel_i_backup)
                self.get_logger().info(
                    f'[param] mode2 이탈 - MPC_XY_VEL_I_ACC을 {self._mode2_xy_vel_i_backup}으로 복원')
                self._mode2_xy_vel_i_backup = None
            if self._mode2_xy_vel_d_backup is not None:
                self.set_px4_param('MPC_XY_VEL_D_ACC', self._mode2_xy_vel_d_backup)
                self.get_logger().info(
                    f'[param] mode2 이탈 - MPC_XY_VEL_D_ACC을 {self._mode2_xy_vel_d_backup}으로 복원')
                self._mode2_xy_vel_d_backup = None

        if self.active_mode == 'mode3' and cmd != 'mode3':
            if self._mode3_xy_p_backup is not None:
                self.set_px4_param('MPC_XY_P', self._mode3_xy_p_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_XY_P을 '
                    f'{self._mode3_xy_p_backup}(원래값)으로 복원')
                self._mode3_xy_p_backup = None
            if self._mode3_z_p_backup is not None:
                self.set_px4_param('MPC_Z_P', self._mode3_z_p_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_Z_P을 '
                    f'{self._mode3_z_p_backup}(원래값)으로 복원')
                self._mode3_z_p_backup = None
            if self._mode3_xy_vel_i_backup is not None:
                self.set_px4_param('MPC_XY_VEL_I_ACC', self._mode3_xy_vel_i_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_XY_VEL_I_ACC을 '
                    f'{self._mode3_xy_vel_i_backup}(원래값)으로 복원')
                self._mode3_xy_vel_i_backup = None
            if self._mode3_xy_vel_d_backup is not None:
                self.set_px4_param('MPC_XY_VEL_D_ACC', self._mode3_xy_vel_d_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_XY_VEL_D_ACC을 '
                    f'{self._mode3_xy_vel_d_backup}(원래값)으로 복원')
                self._mode3_xy_vel_d_backup = None
            if self._mode3_z_vel_i_backup is not None:
                self.set_px4_param('MPC_Z_VEL_I_ACC', self._mode3_z_vel_i_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_Z_VEL_I_ACC을 '
                    f'{self._mode3_z_vel_i_backup}(원래값)으로 복원')
                self._mode3_z_vel_i_backup = None
            if self._mode3_z_vel_d_backup is not None:
                self.set_px4_param('MPC_Z_VEL_D_ACC', self._mode3_z_vel_d_backup)
                self.get_logger().info(
                    f'[param] mode3 이탈 - MPC_Z_VEL_D_ACC을 '
                    f'{self._mode3_z_vel_d_backup}(원래값)으로 복원')
                self._mode3_z_vel_d_backup = None

        if self.active_mode == 'mode4' and cmd != 'mode4':
            if self._mode4_xy_p_backup is not None:
                self.set_px4_param('MPC_XY_P', self._mode4_xy_p_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_XY_P을 '
                    f'{self._mode4_xy_p_backup}(원래값)으로 복원')
                self._mode4_xy_p_backup = None
            if self._mode4_z_p_backup is not None:
                self.set_px4_param('MPC_Z_P', self._mode4_z_p_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_Z_P을 '
                    f'{self._mode4_z_p_backup}(원래값)으로 복원')
                self._mode4_z_p_backup = None
            if self._mode4_xy_vel_i_backup is not None:
                self.set_px4_param('MPC_XY_VEL_I_ACC', self._mode4_xy_vel_i_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_XY_VEL_I_ACC을 '
                    f'{self._mode4_xy_vel_i_backup}(원래값)으로 복원')
                self._mode4_xy_vel_i_backup = None
            if self._mode4_xy_vel_d_backup is not None:
                self.set_px4_param('MPC_XY_VEL_D_ACC', self._mode4_xy_vel_d_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_XY_VEL_D_ACC을 '
                    f'{self._mode4_xy_vel_d_backup}(원래값)으로 복원')
                self._mode4_xy_vel_d_backup = None
            if self._mode4_z_vel_i_backup is not None:
                self.set_px4_param('MPC_Z_VEL_I_ACC', self._mode4_z_vel_i_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_Z_VEL_I_ACC을 '
                    f'{self._mode4_z_vel_i_backup}(원래값)으로 복원')
                self._mode4_z_vel_i_backup = None
            if self._mode4_z_vel_d_backup is not None:
                self.set_px4_param('MPC_Z_VEL_D_ACC', self._mode4_z_vel_d_backup)
                self.get_logger().info(
                    f'[param] mode4 이탈 - MPC_Z_VEL_D_ACC을 '
                    f'{self._mode4_z_vel_d_backup}(원래값)으로 복원')
                self._mode4_z_vel_d_backup = None

        if self.active_mode == 'mode12' and cmd != 'mode12' \
                and self.mode12_hgt_ref_backup is not None:
            self.set_px4_param('EKF2_HGT_REF', self.mode12_hgt_ref_backup)
            self.get_logger().info(
                f'[param] mode12 이탈 - EKF2_HGT_REF을 '
                f'{self.mode12_hgt_ref_backup}(원래값)으로 복원')
            self.mode12_hgt_ref_backup = None

        if cmd == 'mode1':
            self.active_mode = 'mode1'
            self.arm_seq_counter = 0
            self.mode1_hold_x = None
            self.mode1_hold_y = None
            self.mode1_z_setpoint = None
            self.mode1_prev_height = None
            self.mode1_reached_target = False
            self.mode1_aruco_first_detected = False
            self.mode1_xy_aligned = False
            self.mode1_correcting = False
            # 복원된 부분: 진입 시 게인을 억제값으로 강제로 누르는 잠금
            # (예전의 MPC_XY_P=4.0 같은 "방어막")은 넣지 않는다. 그냥
            # 원래값만 백업해두고, 아루코 마커 인식 후 mode1_step에서
            # 목표 P/I/D 값으로 전환한다 (전환 로직은 mode1_step 참고).
            self._mode1_xy_p_released = False
            backed_up_xy_p = self.get_px4_param('MPC_XY_P')
            if backed_up_xy_p is not None:
                self._mode1_xy_p_backup = backed_up_xy_p
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_XY_P 원래값({backed_up_xy_p}) '
                    f'백업만 해둠 (아루코 검출 시 {MODE1_XY_P_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_P 현재값 읽기 실패 - 백업 스킵')

            self._mode1_z_p_released = False
            backed_up_z_p = self.get_px4_param('MPC_Z_P')
            if backed_up_z_p is not None:
                self._mode1_z_p_backup = backed_up_z_p
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_Z_P 원래값({backed_up_z_p}) '
                    f'백업만 해둠 (아루코 검출 시 {MODE1_Z_P_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_P 현재값 읽기 실패 - 백업 스킵')

            self._mode1_xy_vel_i_released = False
            backed_up_vel_i = self.get_px4_param('MPC_XY_VEL_I_ACC')
            if backed_up_vel_i is not None:
                self._mode1_xy_vel_i_backup = backed_up_vel_i
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_XY_VEL_I_ACC 원래값'
                    f'({backed_up_vel_i}) 백업만 해둠 (아루코 검출 시 '
                    f'{MODE1_XY_VEL_I_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_I_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode1_xy_vel_d_released = False
            backed_up_vel_d = self.get_px4_param('MPC_XY_VEL_D_ACC')
            if backed_up_vel_d is not None:
                self._mode1_xy_vel_d_backup = backed_up_vel_d
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_XY_VEL_D_ACC 원래값'
                    f'({backed_up_vel_d}) 백업만 해둠 (아루코 검출 시 '
                    f'{MODE1_XY_VEL_D_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_D_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode1_z_vel_i_released = False
            backed_up_z_vel_i = self.get_px4_param('MPC_Z_VEL_I_ACC')
            if backed_up_z_vel_i is not None:
                self._mode1_z_vel_i_backup = backed_up_z_vel_i
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_Z_VEL_I_ACC 원래값'
                    f'({backed_up_z_vel_i}) 백업만 해둠 (아루코 검출 시 '
                    f'{MODE1_Z_VEL_I_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_VEL_I_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode1_z_vel_d_released = False
            backed_up_z_vel_d = self.get_px4_param('MPC_Z_VEL_D_ACC')
            if backed_up_z_vel_d is not None:
                self._mode1_z_vel_d_backup = backed_up_z_vel_d
                self.get_logger().info(
                    f'[param] mode1 진입 - MPC_Z_VEL_D_ACC 원래값'
                    f'({backed_up_z_vel_d}) 백업만 해둠 (아루코 검출 시 '
                    f'{MODE1_Z_VEL_D_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_VEL_D_ACC 현재값 읽기 실패 - 백업 스킵')

            self.get_logger().info(
                f'mode1 활성화 (진입 시 게인 억제 없음 - 아루코 마커 인식 시 '
                f'MPC_XY_P/Z_P/VEL_I,D를 목표값으로 전환, '
                f'{AUTO_TAKEOFF_HEIGHT}m까지 상승 후 호버링)')
        elif cmd == 'mode2':
            self.active_mode = 'mode2'
            self.arm_seq_counter = 0
            self.mode2_start_x = None
            self.mode2_y_setpoint = None
            self.mode2_z_setpoint = None
            self.mode2_hold_x = None
            self.mode2_hold_y = None
            self.mode2_stopped = False
            self.mode2_armed = False
            self.mode2_unknown_streak = 0
            self.mode2_pattern_smoother = PatternSmoother(history_size=7)
            self.mode2_traveled_m = 0.0
            self.mode2_start_time = time.monotonic()
            self.mode2_phase = 'blind'
            self.mode2_pass_streak = 0
            self.mode2_fine_correct_ticks = 0
            self.mode2_current_pitch_deg = 0.0

            # 수정된 부분(핵심, 버그 수정): 백업 읽기가 실패(None)했는데도
            # 무조건 set_px4_param으로 값을 바꿔버리고 있었다. 그러면
            # 이탈 시 복원 로직(`if backup is not None:`)이 "바뀐 적
            # 없다"고 착각해서 절대 복원을 안 해준다 - 파라미터가 mode2
            # 값에 영구히 갇히는 사고로 이어질 수 있었다. mode1과 동일하게
            # 백업이 성공했을 때만 값을 바꾸도록 방어한다.
            self._mode2_tiltmax_air_backup = self.get_px4_param('MPC_TILTMAX_AIR')
            if self._mode2_tiltmax_air_backup is not None:
                self.set_px4_param('MPC_TILTMAX_AIR', MODE2_TILTMAX_AIR_TARGET)
            else:
                self.get_logger().warn(
                    '[param] MPC_TILTMAX_AIR 현재값 읽기 실패 - '
                    'mode2 tilt 제한 적용 스킵')

            self._mode2_xy_p_backup = self.get_px4_param('MPC_XY_P')
            if self._mode2_xy_p_backup is not None:
                self.set_px4_param('MPC_XY_P', MODE2_XY_P_TARGET)
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_P 현재값 읽기 실패 - mode2 게인 적용 스킵')

            self._mode2_xy_vel_i_backup = self.get_px4_param('MPC_XY_VEL_I_ACC')
            if self._mode2_xy_vel_i_backup is not None:
                self.set_px4_param('MPC_XY_VEL_I_ACC', MODE2_XY_VEL_I_TARGET)
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_I_ACC 현재값 읽기 실패 - mode2 게인 적용 스킵')

            self._mode2_xy_vel_d_backup = self.get_px4_param('MPC_XY_VEL_D_ACC')
            if self._mode2_xy_vel_d_backup is not None:
                self.set_px4_param('MPC_XY_VEL_D_ACC', MODE2_XY_VEL_D_TARGET)
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_D_ACC 현재값 읽기 실패 - mode2 게인 적용 스킵')

            self.get_logger().info(
                f'mode2 활성화 (blind 전진(+x, 최대 {MODE2_FORWARD_DISTANCE_M:.2f}m 방향) '
                f'-> 수평선 감지 시 좌우 정렬(align_h) -> 수평선 지나침 감지 후 '
                f'수직선 감지 시 정지(approach_v) -> grid+아루코 미세보정(hover), '
                f'tilt 제한={MODE2_TILTMAX_AIR_TARGET:.1f}도, '
                f'P={MODE2_XY_P_TARGET}, I={MODE2_XY_VEL_I_TARGET}, '
                f'D={MODE2_XY_VEL_D_TARGET})')
        elif cmd == 'mode3':
            self._enter_mode3()
        elif cmd == 'mode4':
            self._enter_mode4()
        elif cmd == 'mode10':
            self.active_mode = 'mode10'
            self.arm_seq_counter = 0
            self.mode10_hold_x = None
            self.mode10_hold_y = None
            self.mode10_z_setpoint = None
            self.get_logger().info(
                f'mode10 활성화 (아루코 없이 블라인드 {AUTO_TAKEOFF_HEIGHT}m '
                f'상승 후 mode3 자동 전환)')
        elif cmd == 'mode11':
            self.active_mode = 'mode11'
            self.arm_seq_counter = 0
            self.mode11_hold_x = None
            self.mode11_hold_y = None
            self.mode11_z_setpoint = None
            self.mode11_prev_height = None
            self.mode11_reached_target = False
            # 추가된 부분: mode3와 동일한 경로 CSV를 재사용해서 격자점
            # 웨이포인트를 로드해둔다. 실제로 웨이포인트를 향해 이동하는
            # 건 mode11_step에서 이륙+호버링 확인(mode11_reached_target)
            # 이후에 시작한다.
            if self.mode11_waypoints is None:
                try:
                    self.mode11_waypoints = self.build_mode3_waypoints(
                        MODE4_PATH_RESULT_CSV, MODE4_WAYPOINT_CSV)
                except Exception as e:
                    self.get_logger().error(f'[mode11] 경로 로드 실패: {e}')
                    self.mode11_waypoints = None
            self.mode11_index = 0
            self.mode11_mission_complete = False
            self.mode11_hold_wp = None
            self.mode11_hovering = False
            self.mode11_hover_timer = 0
            self.mode11_takeoff_hover_start = None
            # 수정된 부분(핵심): mode1과 동일한 방식(진입 시 억제 없이
            # 백업만)이지만, 목표값은 이제 mode1과 분리된 mode11 전용
            # 상수(MODE11_XY_P_TARGET 등)를 쓴다. mode11 이탈 시 원래값
            # 으로 복원.
            self._mode11_xy_p_released = False
            backed_up_xy_p = self.get_px4_param('MPC_XY_P')
            if backed_up_xy_p is not None:
                self._mode11_xy_p_backup = backed_up_xy_p
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_XY_P 원래값({backed_up_xy_p}) '
                    f'백업만 해둠 (라인 인식 시 {MODE11_XY_P_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_P 현재값 읽기 실패 - 백업 스킵')

            self._mode11_z_p_released = False
            backed_up_z_p = self.get_px4_param('MPC_Z_P')
            if backed_up_z_p is not None:
                self._mode11_z_p_backup = backed_up_z_p
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_Z_P 원래값({backed_up_z_p}) '
                    f'백업만 해둠 (라인 인식 시 {MODE11_Z_P_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_P 현재값 읽기 실패 - 백업 스킵')

            self._mode11_xy_vel_i_released = False
            backed_up_vel_i = self.get_px4_param('MPC_XY_VEL_I_ACC')
            if backed_up_vel_i is not None:
                self._mode11_xy_vel_i_backup = backed_up_vel_i
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_XY_VEL_I_ACC 원래값'
                    f'({backed_up_vel_i}) 백업만 해둠 (라인 인식 시 '
                    f'{MODE11_XY_VEL_I_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_I_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode11_xy_vel_d_released = False
            backed_up_vel_d = self.get_px4_param('MPC_XY_VEL_D_ACC')
            if backed_up_vel_d is not None:
                self._mode11_xy_vel_d_backup = backed_up_vel_d
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_XY_VEL_D_ACC 원래값'
                    f'({backed_up_vel_d}) 백업만 해둠 (라인 인식 시 '
                    f'{MODE11_XY_VEL_D_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_XY_VEL_D_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode11_z_vel_i_released = False
            backed_up_z_vel_i = self.get_px4_param('MPC_Z_VEL_I_ACC')
            if backed_up_z_vel_i is not None:
                self._mode11_z_vel_i_backup = backed_up_z_vel_i
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_Z_VEL_I_ACC 원래값'
                    f'({backed_up_z_vel_i}) 백업만 해둠 (라인 인식 시 '
                    f'{MODE11_Z_VEL_I_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_VEL_I_ACC 현재값 읽기 실패 - 백업 스킵')

            self._mode11_z_vel_d_released = False
            backed_up_z_vel_d = self.get_px4_param('MPC_Z_VEL_D_ACC')
            if backed_up_z_vel_d is not None:
                self._mode11_z_vel_d_backup = backed_up_z_vel_d
                self.get_logger().info(
                    f'[param] mode11 진입 - MPC_Z_VEL_D_ACC 원래값'
                    f'({backed_up_z_vel_d}) 백업만 해둠 (라인 인식 시 '
                    f'{MODE11_Z_VEL_D_TARGET}로 변경 예정)')
            else:
                self.get_logger().warn(
                    '[param] MPC_Z_VEL_D_ACC 현재값 읽기 실패 - 백업 스킵')

            self.get_logger().info(
                f'mode11 활성화 (mode1과 동일 로직 - 진입 시 게인 억제 없음, '
                f'라인 인식 시 P/I/D 목표값 전환, {AUTO_TAKEOFF_HEIGHT}m까지 '
                f'상승 후 호버링)')
        elif cmd == 'mode12':
            self.active_mode = 'mode12'
            self.arm_seq_counter = 0
            self.mode12_hold_x = None
            self.mode12_hold_y = None
            self.mode12_z_setpoint = None
            self.mode12_reached_target = False
            self.mode12_prev_height = None
            backed_up = self.get_px4_param('EKF2_HGT_REF')
            if backed_up is not None:
                self.mode12_hgt_ref_backup = backed_up
                self.set_px4_param('EKF2_HGT_REF', 0.0)
                self.get_logger().info(
                    f'[param] mode12 진입 - EKF2_HGT_REF: {backed_up} -> 0(Baro) '
                    f'(vision 완전 배제, 계속 유지)')
            else:
                self.get_logger().warn(
                    '[param] EKF2_HGT_REF 현재값 읽기 실패 - '
                    'baro 고도기준 자동전환 스킵')
            self.get_logger().info(
                f'mode12 활성화 (순수 EKF2/baro 기준 블라인드 상승 - '
                f'{AUTO_TAKEOFF_HEIGHT}m까지, vision 절대 미사용, x/y/yaw 고정)')
        else:
            self.get_logger().warn(
                f'알 수 없는 명령어: {cmd} '
                f'(mode1 / mode2 / mode3 / mode4 / mode10 / mode11 / mode12 만 가능)')

    def _enter_mode3(self):
        # 격자점(또는 현재 위치)에서 순수 호버링만 하는 모드. 진입 시점의
        # 위치를 hold 좌표로 새로 캡처하도록 초기화만 해두고, 실제 캡처는
        # mode3_step 첫 틱에서 한다. mode1/mode11과 동일한 패턴으로 mode3
        # 전용 게인(MODE3_XY_P_TARGET 등)으로 즉시 전환하고, 이탈 시
        # mode_command_callback 상단의 복원 블록에서 원래값으로 되돌린다.
        self.active_mode = 'mode3'
        self.arm_seq_counter = 0
        self.mode3_hold_x = None
        self.mode3_hold_y = None
        self.mode3_z_setpoint = None
        self.mode3_prev_height = None
        self.mode3_reached_target = False

        self._mode3_xy_p_backup = self.get_px4_param('MPC_XY_P')
        if self._mode3_xy_p_backup is not None:
            self.set_px4_param('MPC_XY_P', MODE3_XY_P_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_P 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self._mode3_z_p_backup = self.get_px4_param('MPC_Z_P')
        if self._mode3_z_p_backup is not None:
            self.set_px4_param('MPC_Z_P', MODE3_Z_P_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_P 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self._mode3_xy_vel_i_backup = self.get_px4_param('MPC_XY_VEL_I_ACC')
        if self._mode3_xy_vel_i_backup is not None:
            self.set_px4_param('MPC_XY_VEL_I_ACC', MODE3_XY_VEL_I_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_VEL_I_ACC 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self._mode3_xy_vel_d_backup = self.get_px4_param('MPC_XY_VEL_D_ACC')
        if self._mode3_xy_vel_d_backup is not None:
            self.set_px4_param('MPC_XY_VEL_D_ACC', MODE3_XY_VEL_D_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_VEL_D_ACC 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self._mode3_z_vel_i_backup = self.get_px4_param('MPC_Z_VEL_I_ACC')
        if self._mode3_z_vel_i_backup is not None:
            self.set_px4_param('MPC_Z_VEL_I_ACC', MODE3_Z_VEL_I_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_VEL_I_ACC 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self._mode3_z_vel_d_backup = self.get_px4_param('MPC_Z_VEL_D_ACC')
        if self._mode3_z_vel_d_backup is not None:
            self.set_px4_param('MPC_Z_VEL_D_ACC', MODE3_Z_VEL_D_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_VEL_D_ACC 현재값 읽기 실패 - mode3 게인 적용 스킵')

        self.get_logger().info(
            f'mode3 활성화 (mode11과 동일한 이륙 로직 - '
            f'{AUTO_TAKEOFF_HEIGHT}m까지 블라인드 상승 후 순수 호버링, '
            f'P={MODE3_XY_P_TARGET}/{MODE3_Z_P_TARGET}, '
            f'VEL_I={MODE3_XY_VEL_I_TARGET}/{MODE3_Z_VEL_I_TARGET}, '
            f'VEL_D={MODE3_XY_VEL_D_TARGET}/{MODE3_Z_VEL_D_TARGET})')

    def _enter_mode4(self):
        # mode_command로 'mode4'가 들어올 때마다 테이블(경로 CSV)의 다음
        # 격자점으로 딱 한 칸만 이동한다. 최초 진입 시에는 index=0(첫
        # 번째 격자점)으로 시작하고, 그 다음부터 'mode4'가 다시 들어올
        # 때마다(=이 함수가 다시 호출될 때마다) index를 1씩 증가시킨다.
        # 도착판정/호버링은 하지 않고 그냥 target setpoint만 계속 보낸다
        # (호버링이 필요하면 이어서 'mode3'을 보내면 된다).
        self.active_mode = 'mode4'
        self.arm_seq_counter = 0

        if not self.mode4_initialized:
            try:
                self.mode4_waypoints = self.build_mode3_waypoints(
                    MODE4_PATH_RESULT_CSV, MODE4_WAYPOINT_CSV)
            except Exception as e:
                self.get_logger().error(f'[mode4] 경로 로드 실패: {e}')
                self.mode4_waypoints = None
                self.active_mode = None
                return
            self.mode4_index = 0
            self.mode4_initialized = True
        elif self.mode4_waypoints:
            self.mode4_index = min(
                self.mode4_index + 1, len(self.mode4_waypoints) - 1)

        if self.ground_z is not None:
            self.mode4_z_setpoint = self.ground_z - AUTO_TAKEOFF_HEIGHT
        else:
            pos = self.get_position_from_odometry()
            self.mode4_z_setpoint = pos[2] if pos is not None else None

        self._mode4_xy_p_backup = self.get_px4_param('MPC_XY_P')
        if self._mode4_xy_p_backup is not None:
            self.set_px4_param('MPC_XY_P', MODE4_XY_P_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_P 현재값 읽기 실패 - mode4 게인 적용 스킵')

        self._mode4_z_p_backup = self.get_px4_param('MPC_Z_P')
        if self._mode4_z_p_backup is not None:
            self.set_px4_param('MPC_Z_P', MODE4_Z_P_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_P 현재값 읽기 실패 - mode4 게인 적용 스킵')

        self._mode4_xy_vel_i_backup = self.get_px4_param('MPC_XY_VEL_I_ACC')
        if self._mode4_xy_vel_i_backup is not None:
            self.set_px4_param('MPC_XY_VEL_I_ACC', MODE4_XY_VEL_I_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_VEL_I_ACC 현재값 읽기 실패 - mode4 게인 적용 스킵')

        self._mode4_xy_vel_d_backup = self.get_px4_param('MPC_XY_VEL_D_ACC')
        if self._mode4_xy_vel_d_backup is not None:
            self.set_px4_param('MPC_XY_VEL_D_ACC', MODE4_XY_VEL_D_TARGET)
        else:
            self.get_logger().warn('[param] MPC_XY_VEL_D_ACC 현재값 읽기 실패 - mode4 게인 적용 스킵')

        self._mode4_z_vel_i_backup = self.get_px4_param('MPC_Z_VEL_I_ACC')
        if self._mode4_z_vel_i_backup is not None:
            self.set_px4_param('MPC_Z_VEL_I_ACC', MODE4_Z_VEL_I_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_VEL_I_ACC 현재값 읽기 실패 - mode4 게인 적용 스킵')

        self._mode4_z_vel_d_backup = self.get_px4_param('MPC_Z_VEL_D_ACC')
        if self._mode4_z_vel_d_backup is not None:
            self.set_px4_param('MPC_Z_VEL_D_ACC', MODE4_Z_VEL_D_TARGET)
        else:
            self.get_logger().warn('[param] MPC_Z_VEL_D_ACC 현재값 읽기 실패 - mode4 게인 적용 스킵')

        if self.mode4_waypoints:
            wp = self.mode4_waypoints[self.mode4_index]
            self.get_logger().info(
                f'mode4 활성화 - 격자점 {self.mode4_index + 1}/'
                f'{len(self.mode4_waypoints)}로 이동: '
                f'x={wp["x"]:.2f}, y={wp["y"]:.2f}, z={self.mode4_z_setpoint}')
        else:
            self.get_logger().warn('[mode4] 웨이포인트가 없습니다.')

    def centering_tick(self):
        if self.active_mode == 'mode11':
            self.centering.update()

    def set_speed_limit(self):
        if self.speed_set:
            return
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=1.0, param2=2.0, param3=-1.0)
        self.speed_set = True
        self.get_logger().info('Speed limit set to 2.0 m/s')

    def odometry_callback(self, msg):
        self.odom = msg

    def vehicle_status_callback(self, msg):
        self.status = msg

    def px4_timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.px4_timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, wp):
        msg = TrajectorySetpoint()
        msg.timestamp = self.px4_timestamp_us()
        msg.position = [float(wp[0]), float(wp[1]), float(wp[2])]
        msg.yaw = float(wp[3])
        self.setpoint_pub.publish(msg)

    def publish_mode2_velocity_setpoint(self, velocity_x):
        msg = TrajectorySetpoint()
        msg.timestamp = self.px4_timestamp_us()
        msg.position = [float('nan'), float('nan'), float(self.mode2_z_setpoint)]
        msg.velocity = [float(velocity_x), 0.0, float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        self.setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                param3=0.0, param4=0.0, param5=0.0,
                                param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.px4_timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def is_armed(self):
        if self.status is None:
            return False
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_offboard(self):
        if self.status is None:
            return False
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def get_position_from_odometry(self):
        if self.odom is None:
            return None
        return (float(self.odom.position[0]),
                float(self.odom.position[1]),
                float(self.odom.position[2]))

    def ensure_armed_and_offboard(self, hold_wp):
        self.publish_setpoint(hold_wp)
        self.arm_seq_counter += 1

        if self.arm_seq_counter < 10:
            return False

        if not self.is_offboard():
            if self.arm_seq_counter % 10 == 0:
                self.set_offboard_mode()
                self.get_logger().info('Requesting OFFBOARD mode...')
            return False

        if not self.is_armed():
            if self.arm_seq_counter % 10 == 0:
                self.arm()
                self.get_logger().info('Requesting ARM...')
            return False

        return True

    def _init_mavlink_param_link(self):
        self._mav_param_link = None
        self._mpc_tiltmax_air_backup = None
        self._mode11_tiltmax_air_backup = None
        self._mode11_xy_p_backup = None
        self._mode11_xy_p_released = False
        # 추가된 부분: mode11도 mode1과 동일하게 Z_P/XY_VEL_I,D/Z_VEL_I,D
        # 까지 백업/전환 대상으로 확장
        self._mode11_z_p_backup = None
        self._mode11_z_p_released = False
        self._mode11_xy_vel_i_backup = None
        self._mode11_xy_vel_i_released = False
        self._mode11_xy_vel_d_backup = None
        self._mode11_xy_vel_d_released = False
        self._mode11_z_vel_i_backup = None
        self._mode11_z_vel_i_released = False
        self._mode11_z_vel_d_backup = None
        self._mode11_z_vel_d_released = False
        self._mode1_xy_p_backup = None
        self._mode1_xy_p_released = False
        # 추가된 부분: z(고도) 게인도 xy와 동일한 잠금/해제 구조
        self._mode1_z_p_backup = None
        self._mode1_z_p_released = False
        # 추가된 부분: xy 속도루프 I/D 게인 백업 (z게인과 동일하게 잠금 없음)
        self._mode1_xy_vel_i_backup = None
        self._mode1_xy_vel_i_released = False
        self._mode1_xy_vel_d_backup = None
        self._mode1_xy_vel_d_released = False
        # 추가된 부분: z 속도루프 I/D 게인 백업 (xy vel과 동일하게 잠금 없음)
        self._mode1_z_vel_i_backup = None
        self._mode1_z_vel_i_released = False
        self._mode1_z_vel_d_backup = None
        self._mode1_z_vel_d_released = False
        self._mode2_tiltmax_air_backup = None
        self._mode2_xy_p_backup = None
        self._mode2_xy_vel_i_backup = None
        self._mode2_xy_vel_d_backup = None
        # 추가된 부분: mode3(호버 전용)/mode4(격자점 한 칸 이동) 전용
        # 게인 백업. mode1/mode11과 별개로 독립 관리한다.
        self._mode3_xy_p_backup = None
        self._mode3_z_p_backup = None
        self._mode3_xy_vel_i_backup = None
        self._mode3_xy_vel_d_backup = None
        self._mode3_z_vel_i_backup = None
        self._mode3_z_vel_d_backup = None
        self._mode4_xy_p_backup = None
        self._mode4_z_p_backup = None
        self._mode4_xy_vel_i_backup = None
        self._mode4_xy_vel_d_backup = None
        self._mode4_z_vel_i_backup = None
        self._mode4_z_vel_d_backup = None
        # 추가된 부분: 마그네토미터 융합을 mode1/mode11 진입 시 끄기 위한
        # 백업 변수. 실비행 확인 결과, EKF2가 마그네토미터 기반 요(yaw)를
        # 신뢰하면서 오히려 자세가 흔들려 랜덤 드리프트로 이어지는 것으로
        # 보여서 추가함.
        # 수정된 부분: 마그네토미터는 PX4 콘솔에서 영구적으로 끄기로 해서,
        # 코드에서 켰다 껐다 할 백업 변수가 더 이상 필요 없다.
        try:
            self._mav_param_link = mavutil.mavlink_connection(
                'udpin:0.0.0.0:14540', source_system=250)
            self.get_logger().info(
                'MAVLink param 링크 연결 시도 (mode1 tilt 자동 전환용, '
                'udpin:0.0.0.0:14540 바인드) - 첫 heartbeat 대기 중...')
            hb = self._mav_param_link.wait_heartbeat(timeout=5)
            if hb is not None:
                self.get_logger().info(
                    f'MAVLink heartbeat 수신 완료 '
                    f'(system={self._mav_param_link.target_system}, '
                    f'component={self._mav_param_link.target_component})')
            else:
                self.get_logger().warn(
                    'MAVLink heartbeat 5초 내 미수신 - PX4가 아직 안 떴거나 '
                    '포트가 안 맞을 수 있음 (tilt 자동 전환 불안정할 수 있음)')
        except Exception as e:
            self.get_logger().warn(
                f'MAVLink param 링크 연결 실패 (tilt 자동 전환 비활성화): {e}')

    def set_px4_param(self, name: str, value, param_type=None):
        if self._mav_param_link is None:
            return False
        if param_type is None:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        int_types = (
            mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
            mavutil.mavlink.MAV_PARAM_TYPE_INT16,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        try:
            if param_type in int_types:
                param_value = struct.unpack('<f', struct.pack('<i', int(value)))[0]
            else:
                param_value = float(value)

            self._mav_param_link.mav.param_set_send(
                self._mav_param_link.target_system,
                self._mav_param_link.target_component,
                name.encode('utf-8'),
                param_value,
                param_type,
            )
            self.get_logger().info(f'[param] {name} -> {value} 전송')
            return True
        except Exception as e:
            self.get_logger().warn(f'[param] {name} 설정 실패: {e}')
            return False

    def get_px4_param(self, name: str, timeout_sec: float = 3.0):
        if self._mav_param_link is None:
            return None

        int_types = (
            mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
            mavutil.mavlink.MAV_PARAM_TYPE_INT16,
            mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        try:
            deadline = time.time() + timeout_sec
            resend_interval = 0.5
            next_send_time = 0.0  # 0으로 두면 아래 while 첫 루프에서 바로 전송됨

            while True:
                now = time.time()
                remaining = deadline - now
                if remaining <= 0:
                    self.get_logger().warn(
                        f'[param] {name} 읽기 타임아웃 - {timeout_sec}초 안에 '
                        f'일치하는 응답을 못 받음 (요청 패킷 유실 가능성 포함)')
                    break

                if now >= next_send_time:
                    self._mav_param_link.mav.param_request_read_send(
                        self._mav_param_link.target_system,
                        self._mav_param_link.target_component,
                        name.encode('utf-8'), -1)
                    next_send_time = now + resend_interval

                wait_time = min(resend_interval, remaining)
                msg = self._mav_param_link.recv_match(
                    type='PARAM_VALUE', blocking=True, timeout=wait_time)
                if msg is None:
                    continue  # 이번 대기 구간엔 아무 응답도 없었음 - 재전송하며 계속 대기
                if msg.param_id.strip('\x00') == name:
                    if msg.param_type in int_types:
                        return struct.unpack(
                            '<i', struct.pack('<f', msg.param_value))[0]
                    return msg.param_value
                # 이름이 다른 PARAM_VALUE면 무시하고 계속 기다린다
        except Exception as e:
            self.get_logger().warn(f'[param] {name} 읽기 실패: {e}')
        return None

    def mode1_step(self):
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'[mode1] 지면 NED z 기록: {self.ground_z:.3f}m')

        rclpy.spin_once(self.cam, timeout_sec=0.0)
        frame = self.cam.read()

        aruco_detected_now = False
        xy_error_m = None
        if frame is not None:
            ids, area, corners = detect_aruco(frame)
            if ids is not None and area > 0 and len(corners) > 0:
                aruco_detected_now = True
                offset_info = compute_center_offset_m(frame.shape, corners[0])
                xy_error_m = offset_info['total_offset_m']
                self.mode1_xy_aligned = (
                    abs(offset_info['x_offset_m']) <= MODE1_XY_ALIGN_TOLERANCE_M
                    and abs(offset_info['y_offset_m']) <= MODE1_XY_ALIGN_TOLERANCE_M
                )

        if self.mode1_hold_x is None:
            self.mode1_hold_x = pos[0]
            self.mode1_hold_y = pos[1]
            self.mode1_z_setpoint = pos[2]

        hold_wp = [self.mode1_hold_x, self.mode1_hold_y,
                   self.mode1_z_setpoint, MODE1_HOLD_YAW_NED]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        current_height = self.ground_z - pos[2]

        if self.mode1_prev_height is not None:
            jump = abs(current_height - self.mode1_prev_height)
            if jump > 0.5:
                self.get_logger().warn(
                    f'[mode1] 고도 점프 감지! {self.mode1_prev_height:.2f}m -> '
                    f'{current_height:.2f}m (Δ{jump:.2f}m/tick) - '
                    f'line-scanning VIO 추정치가 불안정할 수 있음')
        self.mode1_prev_height = current_height

        if not self.mode1_reached_target:
            if current_height < AUTO_TAKEOFF_HEIGHT:
                self.mode1_z_setpoint -= 1
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'[mode1] 상승 중 (높이={current_height:.2f}m / '
                        f'목표={AUTO_TAKEOFF_HEIGHT}m)')
            else:
                self.mode1_reached_target = True
                self.get_logger().info(
                    f'[mode1] 목표 고도 도달 (높이={current_height:.2f}m) - '
                    f'이후 계속 호버링하며 안정성만 관찰 (mode3 전환 없음)')
        else:
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode1] 호버링 중 - 높이={current_height:.2f}m '
                    f'(목표={AUTO_TAKEOFF_HEIGHT}m, 오차={current_height - AUTO_TAKEOFF_HEIGHT:+.3f}m, '
                    f'재보정 비활성화 상태)')

        self.mode1_z_setpoint = clamp_z_setpoint_to_safety(
            self.ground_z, self.mode1_z_setpoint, MODE1_BLIND_MAX_HEIGHT)

        if aruco_detected_now and not self.mode1_aruco_first_detected:
            self.mode1_aruco_first_detected = True

        if (self.mode1_aruco_first_detected and not self._mode1_xy_p_released
                and self._mode1_xy_p_backup is not None):
            self.set_px4_param('MPC_XY_P', MODE1_XY_P_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_XY_P: '
                f'{self._mode1_xy_p_backup} -> {MODE1_XY_P_TARGET}(목표값) 전환')
            self._mode1_xy_p_released = True

        if (self.mode1_aruco_first_detected and not self._mode1_z_p_released
                and self._mode1_z_p_backup is not None):
            self.set_px4_param('MPC_Z_P', MODE1_Z_P_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_Z_P: '
                f'{self._mode1_z_p_backup} -> {MODE1_Z_P_TARGET}(목표값) 전환')
            self._mode1_z_p_released = True

        if (self.mode1_aruco_first_detected and not self._mode1_xy_vel_i_released
                and self._mode1_xy_vel_i_backup is not None):
            self.set_px4_param('MPC_XY_VEL_I_ACC', MODE1_XY_VEL_I_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_XY_VEL_I_ACC: '
                f'{self._mode1_xy_vel_i_backup} -> {MODE1_XY_VEL_I_TARGET} 전환')
            self._mode1_xy_vel_i_released = True

        if (self.mode1_aruco_first_detected and not self._mode1_xy_vel_d_released
                and self._mode1_xy_vel_d_backup is not None):
            self.set_px4_param('MPC_XY_VEL_D_ACC', MODE1_XY_VEL_D_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_XY_VEL_D_ACC: '
                f'{self._mode1_xy_vel_d_backup} -> {MODE1_XY_VEL_D_TARGET} 전환')
            self._mode1_xy_vel_d_released = True

        if (self.mode1_aruco_first_detected and not self._mode1_z_vel_i_released
                and self._mode1_z_vel_i_backup is not None):
            self.set_px4_param('MPC_Z_VEL_I_ACC', MODE1_Z_VEL_I_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_Z_VEL_I_ACC: '
                f'{self._mode1_z_vel_i_backup} -> {MODE1_Z_VEL_I_TARGET} 전환')
            self._mode1_z_vel_i_released = True

        if (self.mode1_aruco_first_detected and not self._mode1_z_vel_d_released
                and self._mode1_z_vel_d_backup is not None):
            self.set_px4_param('MPC_Z_VEL_D_ACC', MODE1_Z_VEL_D_TARGET)
            self.get_logger().info(
                f'[param] mode1 아루코 마커 최초 검출 - MPC_Z_VEL_D_ACC: '
                f'{self._mode1_z_vel_d_backup} -> {MODE1_Z_VEL_D_TARGET} 전환')
            self._mode1_z_vel_d_released = True

        if self.counter % 10 == 0:
            xy_error_str = f'{xy_error_m:.3f}m' if xy_error_m is not None else 'N/A'
            self.get_logger().info(
                f'[mode1] aruco_detected={aruco_detected_now} '
                f'(first_detected={self.mode1_aruco_first_detected}) '
                f'xy_error={xy_error_str} aligned(<={MODE1_XY_ALIGN_TOLERANCE_M}m)='
                f'{self.mode1_xy_aligned} '
                f'hold=({self.mode1_hold_x:.2f}, {self.mode1_hold_y:.2f})')

        self.publish_setpoint(
            [self.mode1_hold_x, self.mode1_hold_y, self.mode1_z_setpoint,
             MODE1_HOLD_YAW_NED])
        self.counter += 1

    def mode10_step(self):
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'[mode10] 지면 NED z 기록: {self.ground_z:.3f}m')

        if self.mode10_hold_x is None:
            self.mode10_hold_x = pos[0]
            self.mode10_hold_y = pos[1]
            self.mode10_z_setpoint = pos[2]

        hold_wp = [self.mode10_hold_x, self.mode10_hold_y,
                   self.mode10_z_setpoint, MODE1_HOLD_YAW_NED]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        current_height = self.ground_z - self.mode10_z_setpoint

        if current_height < AUTO_TAKEOFF_HEIGHT - MODE10_HEIGHT_TOLERANCE:
            self.mode10_z_setpoint -= MODE1_BLIND_CLIMB_STEP
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode10] 블라인드 상승 중 (높이={current_height:.2f}m / '
                    f'목표={AUTO_TAKEOFF_HEIGHT}m)')
        else:
            self.get_logger().info(
                f'[mode10] 목표 고도 도달 (높이={current_height:.2f}m) -> mode3(호버) 자동 전환')
            self._enter_mode3()
            return

        self.mode10_z_setpoint = clamp_z_setpoint_to_safety(
            self.ground_z, self.mode10_z_setpoint, MODE1_BLIND_MAX_HEIGHT)

        self.publish_setpoint(
            [self.mode10_hold_x, self.mode10_hold_y, self.mode10_z_setpoint,
             MODE1_HOLD_YAW_NED])
        self.counter += 1

    def mode11_step(self):
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'[mode11] 지면 NED z 기록: {self.ground_z:.3f}m')

        if self.mode11_hold_x is None:
            self.mode11_hold_x = pos[0]
            self.mode11_hold_y = pos[1]
            self.mode11_z_setpoint = pos[2]

        hold_wp = [self.mode11_hold_x, self.mode11_hold_y,
                   self.mode11_z_setpoint, MODE11_YAW_NED]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        current_height = self.ground_z - pos[2]

        if self.mode11_prev_height is not None:
            jump = abs(current_height - self.mode11_prev_height)
            if jump > 0.5:
                self.get_logger().warn(
                    f'[mode11] 고도 점프 감지! {self.mode11_prev_height:.2f}m -> '
                    f'{current_height:.2f}m (Δ{jump:.2f}m/tick) - '
                    f'line-scanning VIO 추정치가 불안정할 수 있음')
        self.mode11_prev_height = current_height

        if not self.mode11_reached_target:
            # 수정된 부분: mode1과 동일하게 허용오차 없이, 정확히
            # AUTO_TAKEOFF_HEIGHT(2.0m)에 도달할 때까지 계속 상승 명령.
            if current_height < AUTO_TAKEOFF_HEIGHT:
                self.mode11_z_setpoint -= 1
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'[mode11] 상승 중 (높이={current_height:.2f}m / '
                        f'목표={AUTO_TAKEOFF_HEIGHT}m)')
            else:
                self.mode11_reached_target = True
                self.get_logger().info(
                    f'[mode11] 목표 고도 도달 (높이={current_height:.2f}m) - '
                    f'이후 계속 호버링 (재보정 없음, 격자점 로직은 추후 추가 예정)')
        else:
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode11] 호버링 중 - 높이={current_height:.2f}m '
                    f'(목표={AUTO_TAKEOFF_HEIGHT}m, 오차={current_height - AUTO_TAKEOFF_HEIGHT:+.3f}m, '
                    f'재보정 비활성화 상태)')

        self.mode11_z_setpoint = clamp_z_setpoint_to_safety(
            self.ground_z, self.mode11_z_setpoint, MODE1_BLIND_MAX_HEIGHT)

        # 수정된 부분(핵심): mode1과 별개인 mode11 전용 목표 상수
        # (MODE11_XY_P_TARGET 등)로 전환한다. 값은 지금 mode1과 동일하게
        # 시작하지만, 이제 mode11만 따로 튜닝할 수 있다.
        if (self.line_first_detected and not self._mode11_xy_p_released
                and self._mode11_xy_p_backup is not None):
            self.set_px4_param('MPC_XY_P', MODE11_XY_P_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_XY_P: '
                f'{self._mode11_xy_p_backup} -> {MODE11_XY_P_TARGET}(목표값) 전환')
            self._mode11_xy_p_released = True

        if (self.line_first_detected and not self._mode11_z_p_released
                and self._mode11_z_p_backup is not None):
            self.set_px4_param('MPC_Z_P', MODE11_Z_P_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_Z_P: '
                f'{self._mode11_z_p_backup} -> {MODE11_Z_P_TARGET}(목표값) 전환')
            self._mode11_z_p_released = True

        if (self.line_first_detected and not self._mode11_xy_vel_i_released
                and self._mode11_xy_vel_i_backup is not None):
            self.set_px4_param('MPC_XY_VEL_I_ACC', MODE11_XY_VEL_I_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_XY_VEL_I_ACC: '
                f'{self._mode11_xy_vel_i_backup} -> {MODE11_XY_VEL_I_TARGET} 전환')
            self._mode11_xy_vel_i_released = True

        if (self.line_first_detected and not self._mode11_xy_vel_d_released
                and self._mode11_xy_vel_d_backup is not None):
            self.set_px4_param('MPC_XY_VEL_D_ACC', MODE11_XY_VEL_D_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_XY_VEL_D_ACC: '
                f'{self._mode11_xy_vel_d_backup} -> {MODE11_XY_VEL_D_TARGET} 전환')
            self._mode11_xy_vel_d_released = True

        if (self.line_first_detected and not self._mode11_z_vel_i_released
                and self._mode11_z_vel_i_backup is not None):
            self.set_px4_param('MPC_Z_VEL_I_ACC', MODE11_Z_VEL_I_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_Z_VEL_I_ACC: '
                f'{self._mode11_z_vel_i_backup} -> {MODE11_Z_VEL_I_TARGET} 전환')
            self._mode11_z_vel_i_released = True

        if (self.line_first_detected and not self._mode11_z_vel_d_released
                and self._mode11_z_vel_d_backup is not None):
            self.set_px4_param('MPC_Z_VEL_D_ACC', MODE11_Z_VEL_D_TARGET)
            self.get_logger().info(
                f'[param] mode11 라인 최초 인식 - MPC_Z_VEL_D_ACC: '
                f'{self._mode11_z_vel_d_backup} -> {MODE11_Z_VEL_D_TARGET} 전환')
            self._mode11_z_vel_d_released = True

        if self.counter % 10 == 0:
            self.get_logger().info(
                f'[mode11] line_first_detected={self.line_first_detected} '
                f'hold=({self.mode11_hold_x:.2f}, {self.mode11_hold_y:.2f})')

        if not self.mode11_reached_target:
            # 아직 상승 중이면 이륙 지점에 완전 고정 (yaw는 0 고정)
            self.publish_setpoint(
                [self.mode11_hold_x, self.mode11_hold_y, self.mode11_z_setpoint,
                 MODE11_YAW_NED])
            self.counter += 1
            return

        # 추가된 부분: 이륙(목표 고도 도달) 완료 직후, 바로 격자점으로
        # 출발하지 않고 MODE11_TAKEOFF_HOVER_TICKS(10초)만큼 그 자리에서
        # 먼저 안정화 대기한다.
        if self.mode11_takeoff_hover_start is None:
            self.mode11_takeoff_hover_start = self.counter
            self.get_logger().info(
                f'[mode11] 이륙 완료 - {MODE11_TAKEOFF_HOVER_TICKS}틱'
                f'({MODE11_TAKEOFF_HOVER_TICKS / 10:.0f}초) 대기 후 격자점 이동 시작')

        if self.counter - self.mode11_takeoff_hover_start < MODE11_TAKEOFF_HOVER_TICKS:
            self.publish_setpoint(
                [self.mode11_hold_x, self.mode11_hold_y, self.mode11_z_setpoint,
                 MODE11_YAW_NED])
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode11] 이륙 후 대기 중: '
                    f'{self.counter - self.mode11_takeoff_hover_start}/'
                    f'{MODE11_TAKEOFF_HOVER_TICKS} tick')
            self.counter += 1
            return

        # 수정된 부분(핵심): 상승 완료 후엔 mode3와 동일한 웨이포인트
        # (build_mode3_waypoints로 로드해둔 격자점 경로)를 향해 이동한다.
        # 도착 판정은 mode3의 비전 패턴 방식이 아니라 "setpoint까지의
        # 거리"로 단순하게 하고, 도착하면 MODE11_HOVER_TICKS(10초)만큼
        # 호버링 후 다음 점으로 넘어간다. 좌표축/부호나 라인 보정은
        # 나중에 추가 예정 - 지금은 순수 위치이동+정지호버링만 확인.
        if not self.mode11_waypoints:
            if self.counter % 50 == 0:
                self.get_logger().warn(
                    '[mode11] 격자점 웨이포인트가 없음 - 이륙 지점에 계속 호버링')
            self.publish_setpoint(
                [self.mode11_hold_x, self.mode11_hold_y, self.mode11_z_setpoint,
                 MODE11_YAW_NED])
            self.counter += 1
            return

        if self.mode11_mission_complete:
            wp = self.mode11_hold_wp
        else:
            wp = self.mode11_waypoints[self.mode11_index]

        target_wp = [wp['x'], wp['y'], self.mode11_z_setpoint, MODE11_YAW_NED]

        if self.mode11_hovering:
            self.publish_setpoint(target_wp)
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode11] 격자점 호버링 중: '
                    f'{self.counter - self.mode11_hover_timer}/{MODE11_HOVER_TICKS} tick '
                    f'(idx={self.mode11_index}/{len(self.mode11_waypoints)})')
            if self.counter - self.mode11_hover_timer < MODE11_HOVER_TICKS:
                self.counter += 1
                return
            self.mode11_hovering = False
            self._mode11_advance()
            self.counter += 1
            return

        dist = math.hypot(pos[0] - wp['x'], pos[1] - wp['y'])
        if dist <= MODE11_ARRIVAL_TOLERANCE_M:
            self.get_logger().info(
                f'[mode11] 격자점 도착(거리={dist:.2f}m, 허용={MODE11_ARRIVAL_TOLERANCE_M}m) - '
                f'{self.mode11_index + 1}/{len(self.mode11_waypoints)} - '
                f'{MODE11_HOVER_TICKS}틱({MODE11_HOVER_TICKS / 10:.0f}초) 호버링 시작')
            self.mode11_hovering = True
            self.mode11_hover_timer = self.counter
            self.publish_setpoint(target_wp)
            self.counter += 1
            return

        self.publish_setpoint(target_wp)
        if self.counter % 10 == 0:
            self.get_logger().info(
                f'[mode11] 격자점 이동 중 (idx={self.mode11_index}) '
                f'거리={dist:.2f}m target=({wp["x"]:.2f}, {wp["y"]:.2f})')
        self.counter += 1

    def _mode11_advance(self):
        if self.mode11_index >= len(self.mode11_waypoints) - 1:
            self.get_logger().info('[mode11] 격자점 미션 완료 - 마지막 지점 유지')
            last = self.mode11_waypoints[-1]
            self.mode11_hold_wp = {
                'x': last['x'],
                'y': last['y'],
                'yaw': last['yaw'],
                'mode': last['mode'],
                'is_marker': False,
            }
            self.mode11_mission_complete = True
        else:
            self.mode11_index += 1

    def mode12_step(self):
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'[mode12] 지면 NED z 기록: {self.ground_z:.3f}m')

        if self.mode12_hold_x is None:
            self.mode12_hold_x = pos[0]
            self.mode12_hold_y = pos[1]
            self.mode12_z_setpoint = pos[2]

        hold_wp = [self.mode12_hold_x, self.mode12_hold_y,
                   self.mode12_z_setpoint, MODE1_HOLD_YAW_NED]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        current_height = self.ground_z - pos[2]

        if self.mode12_prev_height is not None:
            jump = abs(current_height - self.mode12_prev_height)
            if jump > 0.5:
                self.get_logger().warn(
                    f'[mode12] 고도 점프 감지! '
                    f'{self.mode12_prev_height:.2f}m -> {current_height:.2f}m '
                    f'(Δ{jump:.2f}m/tick) - vision 미사용 상태이므로 '
                    f'baro/IMU 쪽 이상일 가능성')
        self.mode12_prev_height = current_height

        if not self.mode12_reached_target:
            if current_height < AUTO_TAKEOFF_HEIGHT - MODE12_HEIGHT_TOLERANCE:
                self.mode12_z_setpoint -= MODE12_BLIND_CLIMB_STEP
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'[mode12] 상승 중 (높이={current_height:.2f}m / '
                        f'{AUTO_TAKEOFF_HEIGHT}m, EKF2 baro 기준, vision 미사용)')
            else:
                self.mode12_reached_target = True
                self.get_logger().info(
                    f'[mode12] 목표 고도 도달 (높이={current_height:.2f}m) - '
                    f'이후 계속 호버링 (vision 계속 미사용)')
        else:
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode12] 호버링 중 - 높이={current_height:.2f}m '
                    f'(오차={current_height - AUTO_TAKEOFF_HEIGHT:+.3f}m, '
                    f'EKF2 baro 기준)')

        self.mode12_z_setpoint = clamp_z_setpoint_to_safety(
            self.ground_z, self.mode12_z_setpoint, MODE1_BLIND_MAX_HEIGHT)

        self.publish_setpoint(
            [self.mode12_hold_x, self.mode12_hold_y, self.mode12_z_setpoint,
             MODE1_HOLD_YAW_NED])
        self.counter += 1

    def mode2_step(self):
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.mode2_hold_x is None:
            self.mode2_hold_x = pos[0] + MODE2_FORWARD_DISTANCE_M
            self.mode2_hold_y = pos[1]
            if self.mode1_z_setpoint is not None:
                self.mode2_z_setpoint = self.mode1_z_setpoint
                self.get_logger().info(
                    f'[mode2] z를 mode1 고도로 고정: {self.mode2_z_setpoint:.3f}m')
            else:
                self.mode2_z_setpoint = pos[2]
                self.get_logger().warn(
                    '[mode2] mode1_z_setpoint가 없음 - 현재 EKF 고도로 폴백')
            self.get_logger().info(
                f'[mode2] 전진 방향 목표 설정: x={self.mode2_hold_x:.2f} '
                f'(현재 x={pos[0]:.2f} + {MODE2_FORWARD_DISTANCE_M:.2f}m), '
                f'z={self.mode2_z_setpoint:.2f}')

        hold_wp = [
            self.mode2_hold_x,
            self.mode2_hold_y,
            self.mode2_z_setpoint,
            float('nan')
        ]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        rclpy.spin_once(self.cam, timeout_sec=0.0)
        frame = self.cam.read()

        stable_pattern = 'unknown'
        line_detected = False
        is_vertical = None
        mask = None
        pattern_cell_dy = 0
        pattern_cell_dx = 0

        if frame is not None:
            (_, data, pattern_name, live_pattern, live_dist, live_grid,
             _img, _yaw, is_vertical, mask, pattern_cell_dy,
             pattern_cell_dx) = process_frame(frame)
            stable_pattern = self.mode2_pattern_smoother.update(live_pattern)

            line_values = data[:6] if data is not None and len(data) >= 6 else []
            line_detected = any(
                value is not None and float(value) > 0.0
                for value in line_values
            )

        horizontal_now = line_detected and is_vertical is False
        vertical_now = line_detected and is_vertical is True

        if self.mode2_phase == 'blind':
            if horizontal_now:
                self.mode2_phase = 'align_h'
                self.mode2_pass_streak = 0
                self.get_logger().info(
                    '[mode2] 수평선 첫 감지 - 좌우 정렬 시작(align_h)')

        elif self.mode2_phase == 'align_h':
            lateral_dy = self.centering.last_vision_dy
            self.mode2_hold_y = pos[1] + lateral_dy

            if horizontal_now:
                self.mode2_pass_streak = 0
            else:
                self.mode2_pass_streak += 1

            if self.mode2_pass_streak >= MODE2_PASS_STREAK_REQUIRED:
                self.mode2_phase = 'approach_v'
                self.get_logger().info(
                    '[mode2] 수평선 지나침 - 수직선 탐색 시작(approach_v)')

        elif self.mode2_phase == 'approach_v':
            if vertical_now:
                self.mode2_phase = 'search_grid'
                self.mode2_current_pitch_deg = MODE2_SEARCH_INITIAL_PITCH_DEG
                self.get_logger().info(
                    f'[mode2] 수직선 첫 감지 - 격자점 수색 시작(search_grid), '
                    f'피치 {MODE2_SEARCH_INITIAL_PITCH_DEG:.0f}도부터 점진 감속')

        elif self.mode2_phase == 'search_grid':
            self.mode2_current_pitch_deg = max(
                0.0,
                self.mode2_current_pitch_deg - MODE2_SEARCH_PITCH_STEP_DEG
            )
            decel_ratio = self.mode2_current_pitch_deg / MODE2_SEARCH_INITIAL_PITCH_DEG
            lookahead_m = MODE2_SEARCH_LOOKAHEAD_M * decel_ratio
            self.mode2_hold_x = pos[0] + lookahead_m
            self.mode2_hold_y = pos[1]

            grid_point_found = stable_pattern in (
                'cross', 'T_up', 'T_down', 'T_left', 'T_right',
                'corner_TL', 'corner_TR', 'corner_BL', 'corner_BR'
            )

            decel_finished = self.mode2_current_pitch_deg <= 0.0

            if grid_point_found or decel_finished:
                self.mode2_hold_x = pos[0]
                self.mode2_hold_y = pos[1]
                self.mode2_phase = 'hover'
                self.mode2_fine_correct_ticks = 0

                self.set_px4_param('MPC_XY_P', MODE1_XY_P_TARGET)
                self.set_px4_param('MPC_XY_VEL_I_ACC', MODE1_XY_VEL_I_TARGET)
                self.set_px4_param('MPC_XY_VEL_D_ACC', MODE1_XY_VEL_D_TARGET)

                self.get_logger().info(
                    f'[mode2] 호버링 전환 (격자점 발견={grid_point_found}, '
                    f'감속 완주={decel_finished}) '
                    f'(x={pos[0]:.2f}, y={pos[1]:.2f}), '
                    f'XY 게인을 mode1과 동일하게 전환 '
                    f'(P={MODE1_XY_P_TARGET}, I={MODE1_XY_VEL_I_TARGET}, '
                    f'D={MODE1_XY_VEL_D_TARGET})')

        elif self.mode2_phase == 'hover':
            corrected = False
            should_compute = (
                self.mode2_fine_correct_ticks % MODE2_GRID_COMPUTE_INTERVAL == 0)

            if should_compute:
                if mask is not None:
                    g_found, _gcx, _gcy, gdx_m, gdy_m, _gkind = \
                        self.mode2_grid_centering.quick_update(
                            mask, frame.shape, stable_pattern,
                            cell_dy=pattern_cell_dy,
                            cell_dx=pattern_cell_dx,
                            line_width_m=0.10)
                    if g_found:
                        self.mode2_hold_x = (
                            pos[0] - MODE2_FORWARD_OFFSET_SIGN * gdy_m)
                        self.mode2_hold_y = (
                            pos[1] + MODE2_LATERAL_OFFSET_SIGN * gdx_m)
                        corrected = True

                if frame is not None:
                    ids, area, corners = detect_aruco(frame)
                    if ids is not None and area > 0 and len(corners) > 0:
                        offset_info = compute_center_offset_m(frame.shape, corners[0])
                        self.mode2_hold_x = (
                            pos[0] - MODE2_FORWARD_OFFSET_SIGN
                            * offset_info['y_offset_m'])
                        self.mode2_hold_y = (
                            pos[1] + MODE2_LATERAL_OFFSET_SIGN
                            * offset_info['x_offset_m'])
                        corrected = True

            self.mode2_fine_correct_ticks += 1
            if self.mode2_fine_correct_ticks >= MODE2_FINE_CORRECT_MAX_TICKS:
                self.mode2_stopped = True
                self.get_logger().info(
                    f'[mode2] 최종 위치 보정 완료(마지막 계산 반영={corrected}) - '
                    f'(0,0) 호버링 고정 (x={self.mode2_hold_x:.2f}, '
                    f'y={self.mode2_hold_y:.2f})')

        self.publish_setpoint([
            self.mode2_hold_x,
            self.mode2_hold_y,
            self.mode2_z_setpoint,
            float('nan')
        ])

        if self.counter % 10 == 0:
            self.get_logger().info(
                f'[mode2] phase={self.mode2_phase} '
                f'is_vertical={is_vertical} line_detected={line_detected} '
                f'pass_streak={self.mode2_pass_streak} '
                f'stopped={self.mode2_stopped} '
                f'target=({self.mode2_hold_x:.2f}, {self.mode2_hold_y:.2f}, '
                f'{self.mode2_z_setpoint:.2f}) '
                f'cur=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        self.counter += 1

    def build_mode3_waypoints(self, path_result_csv, way_point_csv):
        full_path = load_table_from_csv(path_result_csv)
        if not full_path:
            raise ValueError(f'{path_result_csv}에 격자점이 없습니다.')

        marker_coords = load_table_from_csv(way_point_csv)
        marker_set = {(round(x, 3), round(y, 3)) for x, y in marker_coords}

        waypoints = []
        for i, (gx, gy) in enumerate(full_path):
            ned_x = gy
            ned_y = gx

            if i < len(full_path) - 1:
                nx, ny = full_path[i + 1]
                dx_abs, dy_abs = abs(nx - gx), abs(ny - gy)
                mode = 'V' if dy_abs >= dx_abs else 'H'
            elif i > 0:
                px, py = full_path[i - 1]
                pnx, pny = gx - px, gy - py
                mode = 'V' if abs(pny) >= abs(pnx) else 'H'
            else:
                mode = 'V'

            yaw_ned = wrap_pi(math.pi / 2.0)

            is_marker = (round(gx, 3), round(gy, 3)) in marker_set

            waypoints.append({
                'x': float(ned_x),
                'y': float(ned_y),
                'yaw': yaw_ned,
                'mode': mode,
                'is_marker': is_marker,
            })

        self.get_logger().info(
            f'[mode3] path_result.csv 로드 완료: 격자점 {len(waypoints)}개 '
            f'(마커 {sum(w["is_marker"] for w in waypoints)}개)')
        return waypoints

    def mode3_step(self):
        # mode11과 동일한 이륙 로직: ground_z 기록 -> AUTO_TAKEOFF_HEIGHT
        # 까지 블라인드로 상승 -> 도달하면 이후 계속 그 자리에서 순수
        # 호버링만 한다 (격자점 이동은 mode4가 담당).
        pos = self.get_position_from_odometry()
        if pos is None:
            return

        if self.ground_z is None:
            self.ground_z = pos[2]
            self.get_logger().info(f'[mode3] 지면 NED z 기록: {self.ground_z:.3f}m')

        if self.mode3_hold_x is None:
            self.mode3_hold_x = pos[0]
            self.mode3_hold_y = pos[1]
            self.mode3_z_setpoint = pos[2]

        hold_wp = [self.mode3_hold_x, self.mode3_hold_y,
                   self.mode3_z_setpoint, MODE3_HOLD_YAW_NED]

        if not self.ensure_armed_and_offboard(hold_wp):
            return

        current_height = self.ground_z - pos[2]

        if self.mode3_prev_height is not None:
            jump = abs(current_height - self.mode3_prev_height)
            if jump > 0.5:
                self.get_logger().warn(
                    f'[mode3] 고도 점프 감지! {self.mode3_prev_height:.2f}m -> '
                    f'{current_height:.2f}m (Δ{jump:.2f}m/tick)')
        self.mode3_prev_height = current_height

        if not self.mode3_reached_target:
            if current_height < AUTO_TAKEOFF_HEIGHT:
                self.mode3_z_setpoint -= 1
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'[mode3] 상승 중 (높이={current_height:.2f}m / '
                        f'목표={AUTO_TAKEOFF_HEIGHT}m)')
            else:
                self.mode3_reached_target = True
                self.get_logger().info(
                    f'[mode3] 목표 고도 도달 (높이={current_height:.2f}m) - '
                    f'이후 계속 호버링')
        else:
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'[mode3] 호버링 중 - hold=({self.mode3_hold_x:.2f}, '
                    f'{self.mode3_hold_y:.2f}) 높이={current_height:.2f}m '
                    f'(목표={AUTO_TAKEOFF_HEIGHT}m, '
                    f'오차={current_height - AUTO_TAKEOFF_HEIGHT:+.3f}m)')

        self.mode3_z_setpoint = clamp_z_setpoint_to_safety(
            self.ground_z, self.mode3_z_setpoint, MODE1_BLIND_MAX_HEIGHT)

        self.publish_setpoint(
            [self.mode3_hold_x, self.mode3_hold_y, self.mode3_z_setpoint,
             MODE3_HOLD_YAW_NED])
        self.counter += 1

    def mode4_step(self):
        # mode_command로 'mode4'가 들어올 때마다(=_enter_mode4가 호출될
        # 때마다) 갱신되는 현재 목표 격자점(self.mode4_index)을 향해
        # 계속 setpoint를 보낸다. 도착판정/자동호버링은 하지 않는다
        # (호버링이 필요하면 이어서 'mode3'을 보내면 된다).
        if not self.mode4_waypoints:
            self.get_logger().error('[mode4] waypoints가 계산되지 않았습니다.')
            return

        pos = self.get_position_from_odometry()
        if pos is None:
            return

        wp = self.mode4_waypoints[self.mode4_index]
        z_setpoint = self.mode4_z_setpoint if self.mode4_z_setpoint is not None else pos[2]
        target_wp = [wp['x'], wp['y'], z_setpoint, wp['yaw']]

        if not self.ensure_armed_and_offboard(target_wp):
            return

        self.publish_setpoint(target_wp)
        if self.counter % 10 == 0:
            dist = math.hypot(pos[0] - wp['x'], pos[1] - wp['y'])
            self.get_logger().info(
                f'[mode4] 이동 중 (idx={self.mode4_index + 1}/'
                f'{len(self.mode4_waypoints)}) 거리={dist:.2f}m '
                f'target=({wp["x"]:.2f}, {wp["y"]:.2f})')
        self.counter += 1

    def timer_callback(self):
        if self.active_mode is None:
            return

        self.publish_offboard_control_mode()

        if self.active_mode == 'mode1':
            self.mode1_step()
        elif self.active_mode == 'mode2':
            self.mode2_step()
        elif self.active_mode == 'mode3':
            self.mode3_step()
        elif self.active_mode == 'mode4':
            self.mode4_step()
        elif self.active_mode == 'mode10':
            self.mode10_step()
        elif self.active_mode == 'mode11':
            self.mode11_step()
        elif self.active_mode == 'mode12':
            self.mode12_step()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
