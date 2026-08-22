import cv2
import numpy as np

from config import (
    USE_VIDEO_DEVICE,
    CAMERA_INDEX,
    CAMERA_TOPIC,
    CAMERA_ZOOM_FACTOR,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    VISION_FRAME_WIDTH,
    VISION_FRAME_HEIGHT,
    CAMERA_FPS,
    FRAME_COUNT,
    MAJORITY_COUNT,
    LOWER_WHITE,
    UPPER_WHITE,
    INTERSECTION_GRID_SIZE,
    INTERSECTION_THRESHOLD,
    INTERSECTION_MAX_DISTANCE,
)


_cap = None
_ros_node = None
_ros_subscription = None
_ros_bridge = None
_latest_ros_frame = None
_mask_history = []

_aruco_dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

_aruco_parameters = cv2.aruco.DetectorParameters()

_aruco_detector = cv2.aruco.ArucoDetector(
    _aruco_dictionary,
    _aruco_parameters,
)


def _resize_for_vision(frame):
    """
    카메라 원본 프레임을 비전 처리용 해상도로 변환한다.

    원본 입력:
        FRAME_WIDTH x FRAME_HEIGHT

    비전 처리:
        VISION_FRAME_WIDTH x VISION_FRAME_HEIGHT
    """
    if frame is None:
        return None

    height, width = frame.shape[:2]

    if (
        width == VISION_FRAME_WIDTH
        and height == VISION_FRAME_HEIGHT
    ):
        return frame

    return cv2.resize(
        frame,
        (
            int(VISION_FRAME_WIDTH),
            int(VISION_FRAME_HEIGHT),
        ),
        interpolation=cv2.INTER_LINEAR,
    )


def _make_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    return cv2.inRange(
        hsv,
        LOWER_WHITE,
        UPPER_WHITE,
    )


def _apply_zoom(frame):
    if frame is None or CAMERA_ZOOM_FACTOR <= 1.0:
        return frame

    height, width = frame.shape[:2]
    crop_width = max(1, int(width / CAMERA_ZOOM_FACTOR))
    crop_height = max(1, int(height / CAMERA_ZOOM_FACTOR))

    x1 = (width - crop_width) // 2
    y1 = (height - crop_height) // 2

    cropped = frame[
        y1:y1 + crop_height,
        x1:x1 + crop_width,
    ]

    return cv2.resize(cropped, (width, height))


def _ros_image_callback(msg):
    global _latest_ros_frame
    global _mask_history

    try:
        frame = _ros_bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception as exc:
        if _ros_node is not None:
            _ros_node.get_logger().error(
                f'카메라 영상 변환 실패: {exc}'
            )
        return

    frame = _apply_zoom(frame)
    frame = _resize_for_vision(frame)
    _latest_ros_frame = frame

    mask = _make_mask(frame)
    _mask_history.append(mask)

    if len(_mask_history) > FRAME_COUNT:
        _mask_history.pop(0)


def init_cam(node=None):
    """
    카메라 입력을 초기화한다.

    USE_VIDEO_DEVICE가 True이면 /dev/video 장치를 사용한다.
    False이면 CAMERA_TOPIC의 ROS 2 Image 토픽을 구독한다.

    ROS 토픽 모드에서는 생성된 ROS 2 Node를 node로 전달해야 한다.
    """
    global _cap
    global _ros_node
    global _ros_subscription
    global _ros_bridge
    global _latest_ros_frame
    global _mask_history

    close_cam()

    if USE_VIDEO_DEVICE:
        cap = cv2.VideoCapture(
            CAMERA_INDEX,
            cv2.CAP_V4L2,
        )

        if not cap.isOpened():
            cap.release()
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        _cap = cap
        return True

    if node is None:
        raise ValueError(
            'ROS 토픽 카메라 모드에서는 init_cam(node)이 필요합니다.'
        )

    try:
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except ImportError as exc:
        raise RuntimeError(
            'ROS 카메라 모드에 sensor_msgs와 cv_bridge가 필요합니다.'
        ) from exc

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    _ros_node = node
    _ros_bridge = CvBridge()
    _latest_ros_frame = None
    _mask_history = []

    _ros_subscription = node.create_subscription(
        Image,
        CAMERA_TOPIC,
        _ros_image_callback,
        qos,
    )

    node.get_logger().info(
        f'ROS 카메라 토픽 구독: {CAMERA_TOPIC}, '
        f'비전 처리 해상도: '
        f'{VISION_FRAME_WIDTH}x{VISION_FRAME_HEIGHT}'
    )

    return True


def _majority_mask(masks):
    if not masks:
        return None

    vote_count = np.zeros(
        masks[0].shape,
        dtype=np.uint16,
    )

    for mask in masks:
        vote_count += (mask > 0).astype(np.uint16)

    required_count = min(MAJORITY_COUNT, len(masks))

    return np.where(
        vote_count >= required_count,
        255,
        0,
    ).astype(np.uint8)


def get_frame():
    if USE_VIDEO_DEVICE:
        if _cap is None or not _cap.isOpened():
            return None, None

        frames = []
        masks = []

        for _ in range(FRAME_COUNT):
            ok, frame = _cap.read()

            if not ok or frame is None:
                return None, None

            frame = _apply_zoom(frame)
            frame = _resize_for_vision(frame)
            frames.append(frame)
            masks.append(_make_mask(frame))

        return frames[-1], _majority_mask(masks)

    if _latest_ros_frame is None:
        return None, None

    return (
        _latest_ros_frame.copy(),
        _majority_mask(_mask_history),
    )


def get_pattern(frame, mask):
    if frame is None:
        return -1, -1

    corners, ids, rejected = _aruco_detector.detectMarkers(frame)

    if ids is not None and len(ids) > 0:
        aruco_id = int(np.asarray(ids).reshape(-1)[0])
        return 0, aruco_id

    if mask is None or mask.size == 0:
        return -1, -1

    pattern_index = {
        'cross': 1,
        'T_up': 2,
        'T_down': 3,
        'T_left': 4,
        'T_right': 5,
        'corner_TL': 6,
        'corner_TR': 7,
        'corner_BL': 8,
        'corner_BR': 9,
    }

    patterns = {
        'cross': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'T_up': np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'T_down': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype=np.uint8),

        'T_left': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 1, 1],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'T_right': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'corner_TL': np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'corner_TR': np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 1, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
        ], dtype=np.uint8),

        'corner_BL': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 1, 1],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype=np.uint8),

        'corner_BR': np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ], dtype=np.uint8),
    }

    height, width = mask.shape

    cell_height = height // INTERSECTION_GRID_SIZE
    cell_width = width // INTERSECTION_GRID_SIZE

    intersection_grid = np.zeros(
        (INTERSECTION_GRID_SIZE, INTERSECTION_GRID_SIZE),
        dtype=np.uint8,
    )

    for row in range(INTERSECTION_GRID_SIZE):
        for col in range(INTERSECTION_GRID_SIZE):
            y1 = row * cell_height
            x1 = col * cell_width

            y2 = height if row == INTERSECTION_GRID_SIZE - 1 else y1 + cell_height
            x2 = width if col == INTERSECTION_GRID_SIZE - 1 else x1 + cell_width

            cell = mask[y1:y2, x1:x2]

            white_ratio = cv2.countNonZero(cell) / cell.size

            if white_ratio > INTERSECTION_THRESHOLD:
                intersection_grid[row, col] = 1

    best_pattern_name = None
    best_distance = INTERSECTION_MAX_DISTANCE + 1

    for pattern_name, reference_grid in patterns.items():
        distance = int(np.sum(intersection_grid != reference_grid))

        if distance < best_distance:
            best_distance = distance
            best_pattern_name = pattern_name

    if best_distance > INTERSECTION_MAX_DISTANCE:
        return -1, -1

    pattern_id = pattern_index.get(
        best_pattern_name,
        )

    if 1 <= pattern_id <= 9:
        return pattern_id, -1

    return -1, -1

def close_cam():
    global _cap
    global _ros_node
    global _ros_subscription
    global _ros_bridge
    global _latest_ros_frame
    global _mask_history

    if _cap is not None:
        _cap.release()
        _cap = None

    if _ros_node is not None and _ros_subscription is not None:
        try:
            _ros_node.destroy_subscription(_ros_subscription)
        except Exception:
            pass

    _ros_node = None
    _ros_subscription = None
    _ros_bridge = None
    _latest_ros_frame = None
    _mask_history = []
