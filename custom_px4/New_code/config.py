import numpy as np

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


############################
#          ROS QoS         #
############################

PX4_PUBLISH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


############################
#        camera.py         #
############################

DEBUG_ENABLED_DEFAULT = True

USE_VIDEO_DEVICE = False
CAMERA_INDEX = 0
CAMERA_TOPIC = '/camera/imx219'
CAMERA_ZOOM_FACTOR = 1.0

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

VISION_FRAME_WIDTH = 1280
VISION_FRAME_HEIGHT = 720

CAMERA_FPS = 30
FRAME_COUNT = 5
MAJORITY_COUNT = 3

LOWER_WHITE = np.array(
    [0, 0, 200],
    dtype=np.uint8,
)

UPPER_WHITE = np.array(
    [180, 60, 255],
    dtype=np.uint8,
)


############################
#       get_pattern        #
############################

INTERSECTION_GRID_SIZE = 5
INTERSECTION_THRESHOLD = 0.1
INTERSECTION_MAX_DISTANCE = 6


############################
#        vision.py         #
############################

VISION_RATE = 6.0


############################
#        vio_pub.py        #
############################

VIO_PUBLISH_RATE = 30.0
VIO_TOPIC = '/fmu/in/vehicle_visual_odometry'

VIO_QUALITY = 1
VIO_STALE_TIMEOUT_SEC = 0.5

VIO_VARIANCE_LOW_X = 0.005
VIO_VARIANCE_LOW_Y = 0.005
VIO_VARIANCE_LOW_Z = 0.01

VIO_VARIANCE_HIGH_X = 90
VIO_VARIANCE_HIGH_Y = 90

VIO_POSITION_VARIANCE = [
    0.005,
    0.005,
    0.001,
]
VIO_ORIENTATION_VARIANCE = [
    0.005,
    0.05,
    0.05,
]

VIO_POSITION_VARIANCE_NONE = [
    80.0,
    80.0,
    100.0,
]

VIO_ORIENTATION_VARIANCE_NONE = [
    80.0,
    80.0,
    100.0,
]


############################
#        control.py        #
############################

CONTROL_RATE = 100.0

OFFBOARD_MODE_TOPIC = (
    '/fmu/in/offboard_control_mode'
)
POSITION_SETPOINT_TOPIC = (
    '/fmu/in/trajectory_setpoint'
)
ATTITUDE_SETPOINT_TOPIC = (
    '/fmu/in/vehicle_attitude_setpoint'
)
VEHICLE_COMMAND_TOPIC = (
    '/fmu/in/vehicle_command'
)

TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 1

CUSTOM_MAIN_MODE = 1.0
OFFBOARD_MODE = 6.0

MIN_THRUST = 0.0
MAX_THRUST = 1.0


############################
#          takeoff         #
############################

TAKEOFF_THRUST_STEPS = [
    (0.50, 1.0),
    (0.60, 1.0),
    (0.70, 1.0),
    (0.80, 2.0),
]


############################
#    position transition   #
############################

POSITION_TRANSITION_HEIGHT_M = 0.70
POSITION_TRANSITION_SEC = 1.0


############################
#          mode1           #
############################

MODE1_TARGET_ALTITUDE = 2.0
MODE1_ERROR_THRESHOLD = 0.10

ARUCO_Z_ENABLE_HEIGHT_M = 0


############################
#          mode2           #
############################

MODE2_TARGET_X = 5.5
MODE2_TARGET_Y = 0.0
MODE2_TARGET_Z = -2.0

MODE2_PATTERN_DETECT_FRAMES = 3
MODE2_PATTERN_LOST_FRAMES = 3


############################
#          mode3           #
############################

MODE3_LINE_CONFIRM_COUNT = 5
MODE3_LINE_LOST_COUNT = 10


############################
#      mission states      #
############################

MISSION_IDLE = -1
MISSION_TAKEOFF = 0
MISSION_POSITION_TRANSITION = 1
MISSION_MODE1 = 2
MISSION_MODE2 = 3
MISSION_MODE3 = 4
MISSION_MODE4 = 5


############################
#    mode2 state machine   #
############################

MODE2_WAIT_FIRST_PATTERN = 0
MODE2_WAIT_PATTERN_LOST = 1
MODE2_WAIT_SECOND_PATTERN = 2
MODE2_HOLD_POSITION = 3
