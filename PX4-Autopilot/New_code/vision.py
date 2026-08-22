import math

import cv2
import numpy as np


# pattern 값 1~9와 격자 패턴 이름의 대응 관계
PATTERN_ID_TO_NAME = {
    1: 'cross',
    2: 'T_up',
    3: 'T_down',
    4: 'T_left',
    5: 'T_right',
    6: 'corner_TL',
    7: 'corner_TR',
    8: 'corner_BL',
    9: 'corner_BR',
}

GRID_SHAPE_GROUP = {
    'cross': 'X',
    'T_up': 'T',
    'T_down': 'T',
    'T_left': 'T',
    'T_right': 'T',
    'corner_TL': 'L',
    'corner_TR': 'L',
    'corner_BL': 'L',
    'corner_BR': 'L',
}

LINE_WIDTH_M = 0.10
ARUCO_MARKER_SIZE_M = 0.40
BAND_SIZE_PX = 20
MIN_LINE_PIXELS = 20


# ArUco 검출기 초기화
_aruco_dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)
_aruco_parameters = cv2.aruco.DetectorParameters()
_aruco_parameters.minMarkerPerimeterRate = 0.01
_aruco_parameters.adaptiveThreshWinSizeMin = 3
_aruco_parameters.adaptiveThreshWinSizeMax = 53
_aruco_parameters.adaptiveThreshWinSizeStep = 10
_aruco_parameters.maxMarkerPerimeterRate = 4.0
_aruco_parameters.polygonalApproxAccuracyRate = 0.05
_aruco_detector = cv2.aruco.ArucoDetector(
    _aruco_dictionary,
    _aruco_parameters,
)


def _detect_aruco(frame, target_id=None):
    """ArUco를 검출하고 지정 ID의 corners와 면적을 반환한다."""
    if frame is None or frame.size == 0:
        return -1, -1.0, None

    corners, ids, _ = _aruco_detector.detectMarkers(frame)

    if ids is None or len(corners) == 0:
        return -1, -1.0, None

    ids_flat = ids.flatten()
    index = 0

    if target_id is not None:
        matches = np.where(ids_flat == int(target_id))[0]
        if len(matches) == 0:
            return -1, -1.0, None
        index = int(matches[0])

    marker_id = int(ids_flat[index])
    corners_single = corners[index]
    points = np.asarray(corners_single, dtype=np.float32).reshape(4, 2)

    width_px = np.linalg.norm(points[1] - points[0])
    height_px = np.linalg.norm(points[3] - points[0])
    area = float(width_px * height_px)

    return marker_id, area, corners_single


def _estimate_marker_side_px(corners_single):
    points = np.asarray(corners_single, dtype=np.float32).reshape(4, 2)
    width_px = np.linalg.norm(points[1] - points[0])
    height_px = np.linalg.norm(points[3] - points[0])
    return float((width_px + height_px) * 0.5)


def _compute_center_offset_m(frame_shape, corners_single):
    """영상 중심 대비 마커 중심 편차를 m 단위로 반환한다."""
    h, w = frame_shape[:2]
    points = np.asarray(corners_single, dtype=np.float32).reshape(4, 2)
    marker_center = points.mean(axis=0)

    dx_px = float(marker_center[0] - w * 0.5)
    dy_px = float(marker_center[1] - h * 0.5)

    marker_side_px = _estimate_marker_side_px(corners_single)
    if marker_side_px <= 0:
        return None, None

    meters_per_pixel = ARUCO_MARKER_SIZE_M / marker_side_px
    return dx_px * meters_per_pixel, dy_px * meters_per_pixel


def _aruco_yaw(corners_single):
    """마커 윗변의 기울기로 영상 기준 Yaw를 계산한다 [rad]."""
    points = np.asarray(corners_single, dtype=np.float32).reshape(4, 2)
    dx = float(points[1][0] - points[0][0])
    dy = float(points[1][1] - points[0][1])
    return math.atan2(dy, dx)


def _compute_vertical_yaw(l_top, r_top, y_top, l_bot, r_bot, y_bot):
    if min(l_top, r_top, l_bot, r_bot) < 0:
        return None

    mid_top_x = (l_top + r_top) * 0.5
    mid_bot_x = (l_bot + r_bot) * 0.5
    dy = y_bot - y_top

    if dy == 0:
        return None

    return math.atan2(mid_bot_x - mid_top_x, dy)


def _compute_horizontal_yaw(t_left, b_left, x_left, t_right, b_right, x_right):
    if min(t_left, b_left, t_right, b_right) < 0:
        return None

    mid_left_y = (t_left + b_left) * 0.5
    mid_right_y = (t_right + b_right) * 0.5
    dx = x_right - x_left

    if dx == 0:
        return None

    return math.atan2(-(mid_right_y - mid_left_y), dx)


def _extract_line_data(mask):
    """마스크에서 라인 방향, 폭/높이, 중심편차와 Yaw를 추출한다."""
    if mask is None or mask.size == 0:
        return None

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    h, w = mask.shape[:2]
    contour = max(contours, key=cv2.contourArea)
    x, y, rect_w, rect_h = cv2.boundingRect(contour)
    is_vertical = rect_h > rect_w

    def band_width(y_center):
        y1 = max(0, y_center - BAND_SIZE_PX)
        y2 = min(h, y_center + BAND_SIZE_PX)
        cols = np.where(mask[y1:y2, :] > 0)[1]
        if len(cols) < MIN_LINE_PIXELS:
            return -1.0, -1, -1
        left = int(np.percentile(cols, 2))
        right = int(np.percentile(cols, 98))
        return float(right - left), left, right

    def band_height(x_center):
        x1 = max(0, x_center - BAND_SIZE_PX)
        x2 = min(w, x_center + BAND_SIZE_PX)
        rows = np.where(mask[:, x1:x2] > 0)[0]
        if len(rows) < MIN_LINE_PIXELS:
            return -1.0, -1, -1
        top = int(np.percentile(rows, 2))
        bottom = int(np.percentile(rows, 98))
        return float(bottom - top), top, bottom

    if is_vertical:
        y_top = y + 20
        y_mid = y + rect_h // 2
        y_bot = y + rect_h - 20

        top, l_top, r_top = band_width(y_top)
        mid, l_mid, r_mid = band_width(y_mid)
        bottom, l_bot, r_bot = band_width(y_bot)

        error_px = 0.0
        if l_mid >= 0 and r_mid >= 0:
            error_px = (l_mid + r_mid - w) * 0.5

        yaw = _compute_vertical_yaw(
            l_top, r_top, y_top,
            l_bot, r_bot, y_bot,
        )

        return {
            'is_vertical': True,
            'top': top,
            'mid': mid,
            'bottom': bottom,
            'mid_scale_px': mid,
            'error_px': error_px,
            'yaw': yaw,
        }

    x_left = x + 20
    x_mid = x + rect_w // 2
    x_right = x + rect_w - 20

    left, t_left, b_left = band_height(x_left)
    mid, t_mid, b_mid = band_height(x_mid)
    right, t_right, b_right = band_height(x_right)

    error_px = 0.0
    if t_mid >= 0 and b_mid >= 0:
        error_px = (t_mid + b_mid - h) * 0.5

    yaw = _compute_horizontal_yaw(
        t_left, b_left, x_left,
        t_right, b_right, x_right,
    )

    return {
        'is_vertical': False,
        'top': left,
        'mid': mid,
        'bottom': right,
        'mid_scale_px': mid,
        'error_px': error_px,
        'yaw': yaw,
    }


def _line_out_of_range(top_val, mid_val, bot_val, entries):
    _, t_near, m_near, b_near = entries[0]
    _, t_far, m_far, b_far = entries[-1]

    too_high = []
    too_low = []

    if top_val > 0:
        too_high.append(top_val < t_far)
        too_low.append(top_val > t_near)
    if mid_val > 0:
        too_high.append(mid_val < m_far)
        too_low.append(mid_val > m_near)
    if bot_val > 0:
        too_high.append(bot_val < b_far)
        too_low.append(bot_val > b_near)

    return bool((too_high and all(too_high)) or (too_low and all(too_low)))


def _estimate_altitude(top_val, mid_val, bot_val, angle_deg, table):
    if table is None or angle_deg not in table:
        return -1.0

    entries = table[angle_deg]
    if not entries or _line_out_of_range(top_val, mid_val, bot_val, entries):
        return -1.0

    best_altitude = -1.0
    best_score = float('inf')

    for altitude, top_ref, mid_ref, bot_ref in entries:
        score = 0.0
        count = 0

        if top_val > 0:
            score += abs(top_val - top_ref)
            count += 1
        if mid_val > 0:
            score += abs(mid_val - mid_ref)
            count += 1
        if bot_val > 0:
            score += abs(bot_val - bot_ref)
            count += 1

        if count == 0:
            continue

        score /= count
        if score < best_score:
            best_score = score
            best_altitude = float(altitude)

    return best_altitude


def _estimate_altitude_from_aruco(area, table):
    if area <= 0 or not table:
        return -1.0

    if area < table[-1][1] or area > table[0][1]:
        return -1.0

    return float(min(table, key=lambda row: abs(area - row[1]))[0])


def _estimate_altitude_from_grid(pixel_count, pattern_name, table):
    if pixel_count <= 0 or not table:
        return -1.0

    group = GRID_SHAPE_GROUP.get(pattern_name)
    if group is None:
        return -1.0

    entries = table.get(group, [])
    if not entries:
        return -1.0

    if pixel_count < entries[-1][1] or pixel_count > entries[0][1]:
        return -1.0

    return float(min(entries, key=lambda row: abs(pixel_count - row[1]))[0])


def _estimate_angle(top_val, mid_val, bot_val, angle_table):
    if not angle_table or len(angle_table) != 4:
        return -1.0

    angles, top_table, mid_table, bot_table = angle_table
    best_angle = -1.0
    best_score = float('inf')

    for index, angle in enumerate(angles):
        score = 0.0
        count = 0

        if top_val > 0:
            score += abs(top_val - top_table[index])
            count += 1
        if mid_val > 0:
            score += abs(mid_val - mid_table[index])
            count += 1
        if bot_val > 0:
            score += abs(bot_val - bot_table[index])
            count += 1

        if count == 0:
            continue

        score /= count
        if score < best_score:
            best_score = score
            best_angle = float(angle)

    return best_angle


def _grid_center_offset(mask, frame_shape):
    """격자 흰 영역의 무게중심과 선폭으로 화면 중심 편차를 계산한다."""
    moments = cv2.moments(mask)
    if moments['m00'] == 0:
        return None, None

    center_x = int(moments['m10'] / moments['m00'])
    center_y = int(moments['m01'] / moments['m00'])

    widths = []
    h, w = mask.shape
    for row_index in range(max(0, center_y - 20), min(h, center_y + 20)):
        row = mask[row_index]
        columns = np.where(row > 0)[0]
        if len(columns) > 0:
            widths.append(np.percentile(columns, 98) - np.percentile(columns, 2))

    if not widths:
        return None, None

    width_px = float(np.median(widths))
    if width_px <= 0:
        return None, None

    meters_per_pixel = LINE_WIDTH_M / width_px
    frame_h, frame_w = frame_shape[:2]

    dx_m = (center_x - frame_w * 0.5) * meters_per_pixel
    dy_m = (center_y - frame_h * 0.5) * meters_per_pixel
    return dx_m, dy_m


def get_altitude(pattern, marker_id, mask, frame, table):
    """
    고도를 추정한다.

    table 형식:
        {
            'aruco': [(alt, area), ...],
            'grid': {'L': [...], 'T': [...], 'X': [...]},
            'altitude_ver': {angle: [(alt, top, mid, bot), ...]},
            'altitude_hor': {angle: [(alt, top, mid, bot), ...]},
            'current_angle_deg': 0,
        }
    """
    if frame is None or mask is None or not table:
        return -1.0

    if pattern == 0 and 1 <= marker_id <= 50:
        _, area, _ = _detect_aruco(frame, marker_id)
        return _estimate_altitude_from_aruco(area, table.get('aruco'))

    if 1 <= pattern <= 9:
        pattern_name = PATTERN_ID_TO_NAME.get(pattern)
        pixel_count = cv2.countNonZero(mask)
        return _estimate_altitude_from_grid(
            pixel_count,
            pattern_name,
            table.get('grid'),
        )

    line = _extract_line_data(mask)
    if line is None:
        return -1.0

    angle_deg = int(round(table.get('current_angle_deg', 0)))
    altitude_table = table.get(
        'altitude_ver' if line['is_vertical'] else 'altitude_hor'
    )

    return _estimate_altitude(
        line['top'],
        line['mid'],
        line['bottom'],
        angle_deg,
        altitude_table,
    )


def get_attitude(pattern, marker_id, mask, frame, table):
    """영상으로부터 (roll, pitch, yaw) [rad]를 반환한다."""
    if frame is None or mask is None:
        return 0.0, 0.0, 0.0

    if pattern == 0 and 1 <= marker_id <= 50:
        _, _, corners = _detect_aruco(frame, marker_id)
        if corners is None:
            return 0.0, 0.0, 0.0
        return 0.0, 0.0, _aruco_yaw(corners)

    line = _extract_line_data(mask)
    if line is None:
        return 0.0, 0.0, 0.0

    angle_table = None
    if table:
        angle_table = table.get(
            'angle_ver' if line['is_vertical'] else 'angle_hor'
        )

    angle_deg = _estimate_angle(
        line['top'],
        line['mid'],
        line['bottom'],
        angle_table,
    )
    angle_rad = math.radians(angle_deg) if angle_deg != -1 else 0.0
    yaw = line['yaw'] if line['yaw'] is not None else 0.0

    if line['is_vertical']:
        return 0.0, angle_rad, yaw

    return angle_rad, 0.0, yaw


def get_err_xy(pattern, marker_id, mask, frame, table=None, align_pos=False):
    """
    화면 중심으로 이동하기 위한 Local FRD 보정량 (err_x, err_y) [m]를 반환한다.

    아래보기 카메라 장착을 전제로 한다.
    영상 오른쪽은 FRD +Y, 영상 아래쪽은 FRD -X로 해석한다.
    """
    if frame is None or mask is None:
        return 0.0, 0.0

    if pattern == 0 and 1 <= marker_id <= 50:
        _, _, corners = _detect_aruco(frame, marker_id)
        if corners is None:
            return 0.0, 0.0

        image_dx_m, image_dy_m = _compute_center_offset_m(
            frame.shape,
            corners,
        )
        if image_dx_m is None or image_dy_m is None:
            return 0.0, 0.0

        # 객체가 화면 아래에 있으면 기체는 뒤로 가야 하므로 FRD x 보정은 음수
        # 객체가 화면 오른쪽에 있으면 기체는 오른쪽으로 가야 하므로 FRD y 보정은 양수
        return -float(image_dy_m), float(image_dx_m)

    if align_pos and 1 <= pattern <= 9:
        image_dx_m, image_dy_m = _grid_center_offset(mask, frame.shape)
        if image_dx_m is None or image_dy_m is None:
            return 0.0, 0.0
        return -float(image_dy_m), float(image_dx_m)

    line = _extract_line_data(mask)
    if line is None or line['mid_scale_px'] <= 0:
        return 0.0, 0.0

    error_m = (
        line['error_px']
        * LINE_WIDTH_M
        / line['mid_scale_px']
    )

    if line['is_vertical']:
        # 수직 라인은 좌우 오차만 보정
        return 0.0, -float(error_m)

    # 수평 라인은 전후 오차만 보정
    return -float(error_m), 0.0

def get_vision_measurement(
    pattern,
    marker_id,
    mask,
    frame,
    table,
    align_pos=False,
):
    """
    영상에서 사용할 수 있는 보정정보를 한 번에 반환한다.

    고도, XY 위치 보정, 자세 보정 중 하나라도 유효하면
    vision_valid=True가 된다. 보정값이 실제로 0.0이어도 검출 자체가
    유효하면 해당 *_valid 값은 True가 될 수 있다.
    """
    result = {
        'vision_valid': False,
        'altitude_valid': False,
        'position_valid': False,
        'attitude_valid': False,
        'altitude': -1.0,
        'err_x': 0.0,
        'err_y': 0.0,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    }

    if frame is None or mask is None:
        return result

    altitude = get_altitude(
        pattern,
        marker_id,
        mask,
        frame,
        table,
    )
    err_x, err_y = get_err_xy(
        pattern,
        marker_id,
        mask,
        frame,
        table,
        align_pos=align_pos,
    )
    roll, pitch, yaw = get_attitude(
        pattern,
        marker_id,
        mask,
        frame,
        table,
    )

    altitude_valid = altitude > 0.0
    position_valid = False
    attitude_valid = False

    if pattern == 0 and 1 <= marker_id <= 50:
        _, _, corners = _detect_aruco(frame, marker_id)
        if corners is not None:
            dx_m, dy_m = _compute_center_offset_m(frame.shape, corners)
            position_valid = dx_m is not None and dy_m is not None
            attitude_valid = True

    elif 1 <= pattern <= 9:
        if align_pos:
            dx_m, dy_m = _grid_center_offset(mask, frame.shape)
            position_valid = dx_m is not None and dy_m is not None

        # 격자 패턴 자체가 검출되었고 고도 추정이 가능하면
        # 영상 보정정보가 유효한 것으로 본다.
        attitude_valid = altitude_valid

    else:
        line = _extract_line_data(mask)
        if line is not None:
            position_valid = line['mid_scale_px'] > 0

            angle_table = None
            if table:
                angle_table = table.get(
                    'angle_ver' if line['is_vertical'] else 'angle_hor'
                )

            angle_deg = _estimate_angle(
                line['top'],
                line['mid'],
                line['bottom'],
                angle_table,
            )
            attitude_valid = (
                line['yaw'] is not None
                or angle_deg != -1.0
            )

    vision_valid = bool(
        altitude_valid
        or position_valid
        or attitude_valid
    )

    result.update({
        'vision_valid': vision_valid,
        'altitude_valid': bool(altitude_valid),
        'position_valid': bool(position_valid),
        'attitude_valid': bool(attitude_valid),
        'altitude': float(altitude),
        'err_x': float(err_x),
        'err_y': float(err_y),
        'roll': float(roll),
        'pitch': float(pitch),
        'yaw': float(yaw),
    })
    return result
