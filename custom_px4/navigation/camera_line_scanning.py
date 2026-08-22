import cv2
import numpy as np
import rclpy
import time

from std_msgs.msg import String

from camera import Camera
from angle_estimator import load_angle_table, estimate_angle
from altitude_estimator import load_altitude_table, estimate_altitude
from altitude_estimator import load_aruco_altitude_table, estimate_altitude_from_aruco
from altitude_estimator import load_grid_pixel_altitude_table, estimate_altitude_from_grid_pixels
from intersection_detector import filter_contours_by_mode
from pattern_detector import detect_pattern, detect_pattern_with_shift, draw_grid, PatternSmoother
from aruco_detector import detect_aruco, draw_aruco_center_offset
from aruco_detector import compute_center_offset_m
from vision_logger import VisionLogger
from vision_yaw import compute_vertical_yaw, compute_horizontal_yaw
from mode3_coord_overlay import read_current_waypoint, draw_waypoint_overlay
from aruco_detector import detect_aruco_debug

from grid_centering import GridCentering



# 추가된 부분: VIO 데이터를 PX4로 퍼블리시하는 클래스는 follow_waypoints.py와
# 공용으로 쓰기 위해 vio_publisher.py로 분리했다.
from vio_publisher import VisionOdometryPublisher


def process_frame(img):
    # 수정된 부분(핵심): 여기서 또 리사이즈하지 않는다. main()에서 이미
    # frame_1080 = cv2.resize(frame, (1920,1080))으로 만들어둔 걸
    # 그대로 넘겨받는다는 전제. 같은 프레임을 같은 크기로 두 번
    # 리사이즈하던 중복 연산을 없앤 것.

    # 수정된 부분(핵심): 배경을 흰색 -> 잔디(초록)로 바꾸면서, 라인 색도
    # 검은색 -> 흰색으로 바뀌었다. 기존에는 "V(명도) < 50이면 검은 라인"
    # 으로 판정했는데, 이제는 반대로 "V가 높고 S(채도)가 낮으면 흰 라인"
    # 으로 판정해야 한다.
    # - V(명도)를 높게(>=200): 흰색은 밝으므로
    # - S(채도)를 낮게(<=60): 잔디(초록)는 채도가 있는 색이라 흰색과
    #   구분됨. 잔디가 아무리 밝게 비춰도(그림자 반대쪽 등) 채도가 있으면
    #   걸러진다.
    # - H(색상)는 0~180 전체 허용: 흰색은 색상 자체가 무의미하므로.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 60, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    h, w = mask.shape

    edges_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    live_pattern, live_dist, live_grid, live_cell_dy, live_cell_dx = \
        detect_pattern_with_shift(mask)

    top_length    = -1.0
    mid_length    = -1.0
    bottom_length = -1.0
    left_height   = -1.0
    mid_height    = -1.0
    right_height  = -1.0
    diff          = -1.0
    l_mid         = -1.0
    r_mid         = -1.0
    t_mid         = -1.0
    b_mid         = -1.0

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        data = [top_length, mid_length, bottom_length,
                left_height, mid_height, right_height, diff, -1.0, -1.0, 0.0, 0.0]
        return edges_color, data, 'none', live_pattern, live_dist, live_grid, img, None, None, mask, live_cell_dy, live_cell_dx

    c_temp = max(contours, key=cv2.contourArea)
    x_temp, y_temp, w_temp, h_temp = cv2.boundingRect(c_temp)
    is_vertical = h_temp > w_temp

    filtered = filter_contours_by_mode(contours, is_vertical)
    if not filtered:
        filtered = contours

    c = max(filtered, key=cv2.contourArea)
    cv2.drawContours(edges_color, [c], -1, (0, 255, 255), 2)
    x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(c)

    band = 20

    def band_width(y_center):
        y1 = max(0, y_center - band)
        y2 = min(h, y_center + band)
        row = mask[y1:y2, :]
        cols = np.where(row > 0)[1]
        if len(cols) < 20:
            return -1.0, -1, -1
        left  = int(np.percentile(cols, 2))
        right = int(np.percentile(cols, 98))
        return float(right - left), left, right

    def band_height(x_center):
        x1 = max(0, x_center - band)
        x2 = min(w, x_center + band)
        col = mask[:, x1:x2]
        rows = np.where(col > 0)[0]
        if len(rows) < 20:
            return -1.0, -1, -1
        top_  = int(np.percentile(rows, 2))
        bot_  = int(np.percentile(rows, 98))
        return float(bot_ - top_), top_, bot_

    pattern_name = 'none'
    vision_yaw_deg = None

    if is_vertical:
        y_top = y_rect + 20
        y_mid = y_rect + h_rect // 2
        y_bot = y_rect + h_rect - 20

        top_length,    l_top, r_top = band_width(y_top)
        mid_length,    l_mid, r_mid = band_width(y_mid)
        bottom_length, l_bot, r_bot = band_width(y_bot)

        if top_length > 400 or mid_length > 400 or bottom_length > 400:
            pattern_name = 'skip'
            cv2.putText(edges_color, "INTERSECTION - SKIP", (10, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            data = [top_length, mid_length, bottom_length,
                    left_height, mid_height, right_height, diff, -1.0, -1.0, 0.0, 0.0]
            return edges_color, data, pattern_name, live_pattern, live_dist, live_grid, img, None, is_vertical, mask, live_cell_dy, live_cell_dx

        if top_length > 0 and bottom_length > 0:
            diff = bottom_length - top_length
            vision_yaw_deg = compute_vertical_yaw(l_top, r_top, y_top, l_bot, r_bot, y_bot)
            cv2.line(edges_color, (l_top, y_top), (r_top, y_top), (0, 0, 255), 2)
            cv2.line(edges_color, (l_bot, y_bot), (r_bot, y_bot), (255, 0, 0), 2)
            cv2.line(edges_color, (l_top, y_top), (l_bot, y_bot), (0, 255, 0), 1)
            cv2.line(edges_color, (r_top, y_top), (r_bot, y_bot), (0, 255, 0), 1)
            cv2.putText(edges_color, f"Diff: {diff:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(edges_color, f"Top: {top_length:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(edges_color, f"Bot: {bottom_length:.1f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        if mid_length > 0:
            cv2.line(edges_color, (l_mid, y_mid), (r_mid, y_mid), (0, 255, 0), 2)
            cv2.putText(edges_color, f"Mid: {mid_length:.1f}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(edges_color, "MODE: VERTICAL", (10, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        yaw_str = f"{vision_yaw_deg:.1f}" if vision_yaw_deg is not None else "N/A"
        cv2.putText(edges_color, f"Yaw: {yaw_str}deg", (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    else:
        x_left  = x_rect + 20
        x_mid   = x_rect + w_rect // 2
        x_right = x_rect + w_rect - 20

        left_height,  t_left,  b_left  = band_height(x_left)
        mid_height,   t_mid,   b_mid   = band_height(x_mid)
        right_height, t_right, b_right = band_height(x_right)

        if left_height > 400 or mid_height > 400 or right_height > 400:
            pattern_name = 'skip'
            cv2.putText(edges_color, "INTERSECTION - SKIP", (10, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            data = [top_length, mid_length, bottom_length,
                    left_height, mid_height, right_height, diff, -1.0, -1.0, 0.0, 0.0]
            return edges_color, data, pattern_name, live_pattern, live_dist, live_grid, img, None, is_vertical, mask, live_cell_dy, live_cell_dx

        if left_height > 0 and right_height > 0:
            diff = right_height - left_height
            vision_yaw_deg = compute_horizontal_yaw(t_left, b_left, x_left, t_right, b_right, x_right)
            cv2.line(edges_color, (x_left,  t_left),  (x_left,  b_left),  (0, 0, 255), 2)
            cv2.line(edges_color, (x_right, t_right), (x_right, b_right), (255, 0, 0), 2)
            cv2.line(edges_color, (x_left,  t_left),  (x_right, t_right), (0, 255, 0), 1)
            cv2.line(edges_color, (x_left,  b_left),  (x_right, b_right), (0, 255, 0), 1)
            cv2.putText(edges_color, f"Diff: {diff:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(edges_color, f"Left: {left_height:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(edges_color, f"Right: {right_height:.1f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        if mid_height > 0:
            cv2.line(edges_color, (x_mid, t_mid), (x_mid, b_mid), (0, 255, 0), 2)
            cv2.putText(edges_color, f"Mid: {mid_height:.1f}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(edges_color, "MODE: HORIZONTAL", (10, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        yaw_str = f"{vision_yaw_deg:.1f}" if vision_yaw_deg is not None else "N/A"
        cv2.putText(edges_color, f"Yaw: {yaw_str}deg", (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    if is_vertical:
        if l_mid != -1 and r_mid != -1:
            offset_px = l_mid + r_mid - w
        else:
            offset_px = 0.0
        mid_lo, mid_hi = l_mid, r_mid
        mid_scale_px = mid_length
    else:
        if t_mid != -1 and b_mid != -1:
            offset_px = t_mid + b_mid - h
        else:
            offset_px = 0.0
        mid_lo, mid_hi = t_mid, b_mid
        mid_scale_px = mid_height

    data = [top_length, mid_length, bottom_length,
            left_height, mid_height, right_height, diff, mid_lo, mid_hi, offset_px, mid_scale_px]

    return edges_color, data, pattern_name, live_pattern, live_dist, live_grid, img, vision_yaw_deg, is_vertical, mask, live_cell_dy, live_cell_dx





def main():
    angles_ver, top_ver, mid_ver, bot_ver = load_angle_table('line_table.csv', block='ver')
    angles_hor, top_hor, mid_hor, bot_hor = load_angle_table('line_table.csv', block='hor')
    alt_table_ver = load_altitude_table('line_table.csv', block='ver')
    alt_table_hor = load_altitude_table('line_table.csv', block='hor')

    # 수정된 부분(핵심): 아루코 고도 추정을 기존 "기준 면적 1개 + 역제곱
    # 공식" 방식에서, 라인 테이블(ver/hor)과 동일한 "테이블 로드 -> 범위
    # 밖 판정 -> 최근접 매칭" 방식으로 교체. line_table.csv의 'aru' 블록을
    # 다시 측정해서 (alt, area) 테이블 전체를 채워뒀다고 하셔서, 이제
    # 그 테이블을 그대로 쓴다.
    aruco_altitude_table = load_aruco_altitude_table('line_table.csv')
    if not aruco_altitude_table:
        print("경고: line_table.csv에서 아루코 고도 테이블('aru' 블록)을 못 찾음. "
              "아루코 기반 고도 추정이 항상 -1.0(범위 밖)이 됩니다.")

    # 추가된 부분(핵심): 격자 패턴(cross/T/corner/line) 흰 픽셀수 기반
    # 고도 테이블도 같은 'aru' 블록(L/T/X 컬럼)에서 로드. 아직 측정 안 된
    # 그룹(예: X=cross)은 빈 리스트라서 그 그룹은 항상 -1.0(범위 밖)으로
    # 나온다 - 나중에 line_table.csv 채워지면 자동으로 살아난다.
    grid_pixel_altitude_table = load_grid_pixel_altitude_table('line_table.csv')
    for _group, _entries in grid_pixel_altitude_table.items():
        if not _entries:
            print(f"경고: line_table.csv의 '{_group}' 그룹 데이터가 아직 없음 - "
                  f"이 그룹 기반 고도 추정은 항상 -1.0(범위 밖)이 됩니다.")

    rclpy.init()
    cam = Camera()

    # ============================================================
    # 추가된 부분(핵심): follow_waypoints.py가 발행하는 '/mode_command'를
    # 이 프로세스도 구독해서, 지금 mode1(순수 아루코 정렬)인지 알 수
    # 있게 한다. camera_line_scanning.py는 원래 follow_waypoints.py랑
    # 완전히 독립된 별도 프로세스라서 이 구독이 없으면 지금 어떤 모드인지
    # 전혀 알 방법이 없었다.
    #
    # mode_state는 dict로 만들어서 콜백(클로저) 안에서도 참조·수정이
    # 되게 한다 (단순 지역변수면 nonlocal 선언이 계속 필요해서 번거로움).
    # ============================================================
    mode_state = {'active_mode': None, 'mode2_br_detected': False}
    # 추가된 부분(핵심): mode1에서 아루코 마커가 이번 틱에 안 보여도,
    # None을 보내는 대신 "마지막으로 봤던 유효한 아루코 x/y"를 무제한으로
    # 계속 보내기 위한 캐시. (altitude는 이미 라인 기반 값이 항상 갱신되고
    # 있어서 여기 캐시할 필요 없음.)
    last_valid_aruco = {'x': None, 'y': None}

    def _mode_command_callback(msg):
        new_mode = msg.data.strip().lower()
        # 추가된 부분: mode2로 새로 진입할 때마다 BR 패턴 검출 래치를
        # 리셋한다 (이전에 한 번 검출됐던 상태가 다음 mode2 진입까지
        # 남아있으면 안 되니까).
        if new_mode == 'mode2' and mode_state['active_mode'] != 'mode2':
            mode_state['mode2_br_detected'] = False
        mode_state['active_mode'] = new_mode

    cam.create_subscription(
        String, '/mode_command', _mode_command_callback, 10)

    # 추가된 부분: VIO 퍼블리셔 생성 (Camera 노드에 얹어서 사용)
    vio_pub = VisionOdometryPublisher(cam)

    print("비전 알고리즘 가동...")

    logger = VisionLogger()
    smoother = PatternSmoother(history_size=7)
    grid_centering = GridCentering()
    DEBUG_WINDOWS = ("line_debug", "vision_debug")
    SHOW_DEBUG = True # False로 바꾸면 디버그 창을 아예 띄우지 않는다

    if SHOW_DEBUG:
        for name in DEBUG_WINDOWS:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 1280, 720)

    def mode_features(mode):
        if mode == 'mode1':
            return {
                'line': False,
                'pattern': False,
                'aruco': True,
            }

        if mode == 'mode2':
            return {
                'line': True,
                'pattern': True,
                'aruco': False,
            }

        return {
            'line': True,
            'pattern': True,
            'aruco': True,
        }

    def make_disabled_frame(text):
        blank = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.putText(
            blank,
            text,
            (140, 190),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (120, 120, 120),
            2
        )
        return blank

    ALPHA = 0.2
    smooth_data = None
    start_time = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(cam, timeout_sec=0.01)
            frame = cam.read()
            if frame is None:
                continue

            active_mode = mode_state['active_mode']
            features = mode_features(active_mode)

            data = [-1.0, -1.0, -1.0,
                    -1.0, -1.0, -1.0, -1.0,
                    -1.0, -1.0, 0.0, 0.0]
            pattern_name = 'none'
            live_pattern = 'none'
            live_dist = -1
            grid = None
            frame_1080 = cv2.resize(frame, (1280, 720))
            vision_yaw_deg = None
            is_vertical = None
            mask = np.zeros((1080, 1920), dtype=np.uint8)
            edges_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            pattern_cell_dy = 0
            pattern_cell_dx = 0

            if features['line'] or features['pattern']:
                (edges_color, data, pattern_name, live_pattern, live_dist,
                 grid, frame_1080, vision_yaw_deg, is_vertical,
                 mask, pattern_cell_dy, pattern_cell_dx) = process_frame(frame_1080)

            if features['pattern']:
                stable_pattern = smoother.update(live_pattern)
            else:
                stable_pattern = 'none'

            aruco_ids = None
            aruco_area = 0.0
            aruco_corners = None
            aruco_offset_info_m = None
            detected_aruco_id = None

            if features['aruco']:
                aruco_ids, aruco_area, aruco_corners = detect_aruco(frame)
                if aruco_ids is not None:
                    detected_aruco_id = int(aruco_ids[0][0])

            vision_display = frame.copy()

            if features['pattern']:
                pattern_mask_shape = np.zeros(
                    (frame_1080.shape[0], frame_1080.shape[1]),
                    dtype=np.uint8
                )
                pattern_display = draw_grid(
                    frame_1080.copy(),
                    pattern_mask_shape,
                    pattern_grid=grid
                )
                pattern_display = cv2.resize(
                    pattern_display,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_AREA
                )
                vision_display = cv2.addWeighted(
                    vision_display,
                    0.55,
                    pattern_display,
                    0.45,
                    0.0
                )
            # 추가된 부분(핵심): vision_display 텍스트 정렬 정리.
            # 예전엔 Pattern/ArUco/Grid 텍스트가 30,60,90,120으로
            # 고정 좌표였는데, 폰트 크기가 서로 달라서(0.8 vs 0.7)
            # 줄 간격이 들쭉날쭉해 보였다. 이제 text_x/text_y를
            # 하나로 통일하고, 텍스트를 실제로 그릴 때마다
            # text_y += LINE_H로 누적시켜서 항상 일정한 간격으로
            # 왼쪽 정렬되게 한다.
            text_x = 10
            text_y = 30
            LINE_H = 32
            TEXT_SCALE = 0.7
            TEXT_THICK = 2

            def draw_info(img, text, color):
                nonlocal text_y
                cv2.putText(
                    img, text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    TEXT_SCALE, color, TEXT_THICK
                )
                text_y += LINE_H

            if features['pattern']:
                draw_info(
                    vision_display,
                    f"Pattern: {stable_pattern} (d={live_dist})",
                    (255, 200, 0)
                )

            if features['aruco']:
                if aruco_ids is not None:
                    cv2.aruco.drawDetectedMarkers(
                        vision_display,
                        aruco_corners,
                        aruco_ids
                    )
                    aid = int(aruco_ids[0][0])
                    vision_display, _ = draw_aruco_center_offset(
                        vision_display,
                        aruco_corners[0]
                    )
                    draw_info(
                        vision_display,
                        f"ArUco ID:{aid} Area:{aruco_area:.0f}px",
                        (0, 255, 0)
                    )

                    if aruco_area > 0:
                        aruco_offset_info_m = compute_center_offset_m(
                            frame.shape,
                            aruco_corners[0]
                        )
                        draw_info(
                            vision_display,
                            f"ArUco offset(m): "
                            f"x={aruco_offset_info_m['x_offset_m']:+.3f} "
                            f"y={aruco_offset_info_m['y_offset_m']:+.3f} "
                            f"total={aruco_offset_info_m['total_offset_m']:.3f}",
                            (0, 255, 255)
                        )
                else:
                    draw_info(vision_display, "No ArUco", (100, 100, 100))

            grid_found = False
            grid_center_x = None
            grid_center_y = None
            grid_dx_m = 0.0
            grid_dy_m = 0.0
            grid_kind = "unknown"

            # 추가된 부분(핵심): 아루코 마커가 뭐든(ID 상관없이) 하나라도
            # 보이면 그 프레임엔 grid_centering.quick_update()를 돌리지 않는다.
            # + mode1(순수 아루코 정렬)일 때는 애초에 line/pattern
            # 자체가 꺼져있지만(mode_features), 여기서도 명시적으로
            # 한 번 더 막아서 실수로 켜지는 일이 없게 한다.
            grid_blocked_reason = None

            if mode_state['active_mode'] == 'mode1':
                grid_blocked_reason = "mode1"
            elif detected_aruco_id is not None:
                grid_blocked_reason = f"ArUco #{detected_aruco_id}"

            grid_blocked_by_marker = grid_blocked_reason is not None

            if (features['line'] or features['pattern']) and not grid_blocked_by_marker:
                (grid_found,
                 grid_center_x,
                 grid_center_y,
                 grid_dx_m,
                 grid_dy_m,
                 grid_kind) = grid_centering.quick_update(
                    mask,
                    frame_1080.shape,
                    stable_pattern,
                    cell_dy=pattern_cell_dy,
                    cell_dx=pattern_cell_dx,
                    line_width_m=0.10
                )

            # 추가된 부분(핵심): 마스크(라인/격자) 흰 픽셀 총 개수를
            # 항상 화면에 표시. 나중에 고도별로 이 값을 기록해서
            # (고도, pixel_count, pattern) 테이블을 만들 때 쓸 참고용.
            # grid_found 여부와 상관없이 항상 보이게 - 순수 라인 구간
            # (line_V/H)에서도 값이 필요할 수 있어서.
            mask_pixel_count = cv2.countNonZero(mask) if mask is not None else 0
            draw_info(
                vision_display,
                f"Mask px count: {mask_pixel_count} (pattern={stable_pattern})",
                (200, 200, 0)
            )
            grid_pixel_alt_debug = estimate_altitude_from_grid_pixels(
                mask_pixel_count, stable_pattern, grid_pixel_altitude_table
            ) if mask_pixel_count > 0 else -1.0
            draw_info(
                vision_display,
                f"Grid-px altitude: {grid_pixel_alt_debug:.2f}m"
                if grid_pixel_alt_debug > 0 else "Grid-px altitude: N/A",
                (200, 200, 0) if grid_pixel_alt_debug > 0 else (100, 100, 100)
            )

            if grid_found:
                # frame_1080(1920x1080) 좌표 기준 원본 픽셀 오프셋 -
                # calculate_offset() 내부에서 쓰는 것과 동일한 계산
                img_cx_720 = frame_1080.shape[1] * 0.5
                img_cy_720 = frame_1080.shape[0] * 0.5
                dx_px = grid_center_x - img_cx_720
                dy_px = grid_center_y - img_cy_720

                draw_info(
                    vision_display,
                    f"Grid: {grid_kind} dx={grid_dx_m:+.3f}m dy={grid_dy_m:+.3f}m",
                    (0, 200, 255)
                )
                draw_info(
                    vision_display,
                    f"  px: center=({grid_center_x},{grid_center_y}) "
                    f"img_c=({img_cx_720:.0f},{img_cy_720:.0f}) "
                    f"dx_px={dx_px:+.0f} dy_px={dy_px:+.0f}",
                    (0, 160, 200)
                )
                # 추가된 부분: grid_center는 frame_1080(1920x1080) 좌표라서,
                # vision_display(native, frame 크기)에 그리려면 스케일을
                # 맞춰야 한다.
                scale_x = frame.shape[1] / frame_1080.shape[1]
                scale_y = frame.shape[0] / frame_1080.shape[0]
                gx = int(grid_center_x * scale_x)
                gy = int(grid_center_y * scale_y)
                img_cx = frame.shape[1] // 2
                img_cy = frame.shape[0] // 2
                cv2.arrowedLine(
                    vision_display, (img_cx, img_cy), (gx, gy),
                    (0, 200, 255), 2, tipLength=0.15
                )
                cv2.circle(vision_display, (gx, gy), 6, (0, 200, 255), 2)
            else:
                draw_info(
                    vision_display,
                    f"Grid: blocked ({grid_blocked_reason})"
                    if grid_blocked_by_marker else "Grid: not found",
                    (0, 165, 255) if grid_blocked_by_marker else (100, 100, 100)
                )

            if smooth_data is None:
                smooth_data = data
            else:
                smooth_data = [
                    ALPHA * d + (1 - ALPHA) * s if d != -1.0 else s
                    for d, s in zip(data, smooth_data)
                ]

            if is_vertical is True:
                mode_label = 'ver'
                top_val, mid_val, bot_val = smooth_data[0], smooth_data[1], smooth_data[2]
                angles, top_table, mid_table, bot_table = angles_ver, top_ver, mid_ver, bot_ver
                alt_table = alt_table_ver
            elif is_vertical is False:
                mode_label = 'hor'
                top_val, mid_val, bot_val = smooth_data[3], smooth_data[4], smooth_data[5]
                angles, top_table, mid_table, bot_table = angles_hor, top_hor, mid_hor, bot_hor
                alt_table = alt_table_hor
            else:
                mode_label = 'none'
                top_val, mid_val, bot_val = -1.0, -1.0, -1.0
                angles, top_table, mid_table, bot_table = angles_ver, top_ver, mid_ver, bot_ver
                alt_table = alt_table_ver

            if top_val > 0 and bot_val > 0:
                if mode_label == 'ver':
                    forward_or_right = top_val >= bot_val
                elif mode_label == 'hor':
                    forward_or_right = bot_val >= top_val
                else:
                    forward_or_right = True

                if forward_or_right:
                    direction = 1
                else:
                    direction = -1
                    top_val, bot_val = bot_val, top_val
            else:
                direction = 1

            if top_val > 0 or mid_val > 0 or bot_val > 0:
                angle, score = estimate_angle(
                    top_val, mid_val, bot_val, angles, top_table, mid_table, bot_table)
                altitude = estimate_altitude(top_val, mid_val, bot_val, angle, alt_table)
            else:
                angle, score = -1, -1.0
                altitude = -1.0

            if mode_label == 'ver':
                direction_label = 'FWD' if direction == 1 else 'BWD'
            elif mode_label == 'hor':
                direction_label = 'RIGHT' if direction == 1 else 'LEFT'
            else:
                direction_label = 'N/A'

            LINE_WIDTH_M = 0.10
            l_mid_val, r_mid_val, offset_px_val = data[7], data[8], data[9]
            mid_length_px = mid_val

            if pattern_name == 'skip':
                offset_m = 0.0
            elif l_mid_val == -1 or r_mid_val == -1 or mid_length_px <= 0:
                offset_m = None
            else:
                offset_m = offset_px_val * LINE_WIDTH_M / (2 * mid_length_px)

            timestamp_us = int((time.time() - start_time) * 1e6)
            logger.log(timestamp_us, angle, altitude, score,
                       top_val, mid_val, bot_val)

            # ============================================================
            # 추가된 부분: angle(estimate_angle 결과)을 pitch/roll로 변환.
            # - mode_label == 'ver'(전후진 라인) -> 이 각도는 드론 pitch
            # - mode_label == 'hor'(좌우 라인)   -> 이 각도는 드론 roll
            # angle 테이블 값은 부호 없는 크기(magnitude)이고, 실제 +/- 방향은
            # direction(1/-1)에 담겨 있어서 곱해서 부호를 살린다.
            # 주의: 이 부호가 실제 FRD 좌표계의 pitch/roll 부호(기수 아래=+,
            # 오른쪽으로 기욺=+)와 일치하는지는 아직 검증 안 됐다. 검증 전까지는
            # "방향이 반대로 나올 수도 있다"고 가정하고, 실제 기울여보면서
            # vehicle_odometry에 찍히는 pitch/roll 부호가 맞는지 확인 필요.
            # ============================================================
            if angle is not None and angle != -1:
                signed_angle_deg = float(angle) * (1.0 if direction == 1 else -1.0)
            else:
                signed_angle_deg = None

            pitch_deg = signed_angle_deg if mode_label == 'ver' else None
            roll_deg = signed_angle_deg if mode_label == 'hor' else None

            # ============================================================
            # 수정된 부분(핵심): offset_m은 "라인에 수직인 방향의 편차"를
            # 재는 값인데, 그 방향이 실제로 어느 축(x/y)을 의미하는지는
            # mode_label('ver'/'hor')에 따라 달라진다.
            #   - 'ver'(라인이 이미지에서 세로로 보임): 라인에 수직 = 좌우
            #     -> 물리적으로 y(좌우) 편차 -> offset_m 자리로 보낸다
            #   - 'hor'(라인이 이미지에서 가로로 보임): 라인에 수직 = 상하
            #     -> 물리적으로 x(전후) 편차 -> forward_m 자리로 보낸다
            # vio_publisher.py의 forward_m(x)/offset_m(y) 처리 로직은 이미
            # 완전히 동일한 "직전값 유지 + variance" 구조라서, 여기서는
            # 계산된 값을 어느 파라미터로 넘길지만 mode_label로 갈라주면 된다.
            # 갱신 안 되는 쪽은 None을 넘겨서 vio_publisher.py가 자동으로
            # "그 축은 직전 유효값 유지"하게 둔다 (기존 altitude/offset과
            # 동일한 패턴).
            #
            # 주의(검증 필요): y(offset_m)는 부호를 반전해서 보내는 관례가
            # 이미 확인돼 있었지만(-float(offset_m)), x(forward_m)는 부호
            # 반전이 필요한지 아직 실비행으로 검증 안 됐다. 실제로 hor
            # 구간에서 카메라가 offset을 어느 방향으로 양수로 재는지에 따라
            # 부호가 반대로 나올 수 있으니, 실비행 시 vehicle_odometry의
            # position[0] 부호가 예상과 맞는지 꼭 확인할 것.
            # ============================================================
            if mode_label == 'ver':
                offset_for_y = offset_m
                offset_for_x = None
            elif mode_label == 'hor':
                offset_for_y = None
                offset_for_x = offset_m
            else:
                offset_for_y = None
                offset_for_x = None

            if grid_found and mode_state['active_mode'] != 'mode1':
                offset_for_x = grid_dy_m
                offset_for_y = grid_dx_m

            # ============================================================
            # 추가된 부분(핵심): 아루코 마커가 검출되면 x/y/z를 라인 추적
            # 값 대신 항상 아루코 값으로 덮어쓴다 (사용자 결정: 아루코가
            # 더 정밀하므로 검출되는 순간 우선 적용).
            #
            # - yaw는 여기서도 건드리지 않는다. aruco_detector.py 설계
            #   원칙(주석 참조)대로 아루코는 yaw를 절대 계산하지 않으므로
            #   vision_yaw_deg(라인 기반 값 또는 None)를 그대로 둔다.
            # - pitch_deg/roll_deg도 그대로 둔다 -- 아루코 정렬 단계에서는
            #   보통 라인이 안 보여 mode_label == 'none'이 되어 이미 None
            #   으로 들어가고, 그러면 vio_publisher가 자동으로 "직전 유효
            #   자세값 유지"를 해준다 (vio_publisher.py의 설계와 일치).
            # - 아루코 고도 추정(estimate_altitude_from_aruco_area)이
            #   None을 반환하면(기준값 로드 실패 등) 라인 기반 altitude를
            #   그대로 쓴다 -- "값 없음"으로 무효 altitude를 보내지 않기
            #   위함.
            # ============================================================
            # ============================================================
            # 추가된 부분(핵심): mode1(순수 아루코 정렬)일 때는 라인 기반
            # 값이 vio_pub에 아예 안 들어가게 원천 차단한다. 이걸 안 하면
            # mode1 중에도 이 프로세스가 계속 라인을 검출해서 그 결과가
            # (아루코가 하필 이번 틱에 안 보이는 순간 등에) 섞여 들어갈
            # 여지가 있었다. 라인 검출 자체(process_frame 호출)는 디버그
            # 화면 표시를 위해 그대로 두고, vio_pub로 넘기는 값만 여기서
            # None으로 강제한다.
            # ============================================================
            # ============================================================
            # 수정된 부분(핵심, 버그 수정): mode1일 때도 altitude(고도)는
            # 라인 기반 값을 계속 흘려보내도록 되돌렸다. 이전엔 altitude까지
            # None으로 막았는데, 그러면 vio_publisher.py가 "직전 유효값
            # 유지"를 해서 EKF2로 들어가는 vision 고도가 이륙 전 지면
            # 근처 값에 멈춰버렸다. 그 결과 EKF2 fused pos[2]가 실제 상승을
            # 못 따라가서, mode1_step()의 "목표 고도 도달" 판정이 계속
            # 안 되고 z_setpoint를 끝없이 올리는(오버슈팅 후 안 내려오는)
            # 사고로 이어졌다.
            #
            # x/y/yaw/pitch/roll은 그대로 차단 유지 - 이건 라인 노이즈가
            # 섞이면 안 되는 부분이라 원래 의도(순수 아루코만 반영)가 맞다.
            # altitude만 예외로 둔 이유: 라인 기반 고도 추정은 마커 유무와
            # 무관하게 계속 갱신되는 연속적인 신호라서, 아루코가 안 보일
            # 때도 EKF가 "그동안 실제로 올라갔다"를 계속 따라갈 수 있게
            # 해준다. 아루코가 보이면 여전히 아래에서 더 정밀한 아루코
            # 고도로 덮어쓴다(기존 로직 그대로).
            # ============================================================
            final_altitude = altitude
            if mode_state['active_mode'] == 'mode1':
                # 수정된 부분(핵심): 아루코가 이번 틱에 안 보이면 None이
                # 아니라 마지막으로 봤던 유효한 x/y를 무제한(타임아웃 없이)
                # 계속 사용한다. 사용자 확인: 마커가 오래 안 보여도 그
                # 자리를 계속 유지하는 쪽을 선택함 (variance가 올라가면서
                # 서서히 신뢰도가 떨어지는 대신, 마지막으로 확인된 위치를
                # 계속 신뢰하게 됨 - 트레이드오프 인지한 상태로 진행).
                final_offset_x = last_valid_aruco['x']
                final_offset_y = last_valid_aruco['y']
                vision_yaw_deg = None
                pitch_deg = None
                roll_deg = None
            elif mode_state['active_mode'] == 'mode2':
                # ============================================================
                # 추가된 부분(핵심): mode2 - BR 패턴을 발견하기 전까지는
                # 비전 기반 값(고도+x/y+자세) 전부를 None으로 보내 vio_pub가
                # "직전 유효값 유지" 상태로 두고, 0.5초 지나면
                # vio_publisher.py의 기존 line_lost_variance_scale 메커니즘이
                # 저절로 position_variance를 20배로 키워서 EKF2가 비전을
                # 거의 신뢰 안 하게 만든다 (그 사이엔 IMU/baro 전파에 맡김).
                # BR 패턴을 한 번 발견하면(mode2_br_detected 래치) 그 순간
                # 부터는 실제 값을 계속 흘려보내서 variance가 정상으로
                # 돌아오고 비전을 완전히 신뢰하게 된다.
                # ============================================================
                if not mode_state['mode2_br_detected'] and stable_pattern == 'corner_BR':
                    mode_state['mode2_br_detected'] = True

                if not mode_state['mode2_br_detected']:
                    final_altitude = None
                    final_offset_x = None
                    final_offset_y = None
                    vision_yaw_deg = None
                    pitch_deg = None
                    roll_deg = None
                else:
                    final_offset_x = offset_for_x
                    final_offset_y = offset_for_y
            else:
                final_offset_x = offset_for_x
                final_offset_y = offset_for_y

            # 추가된 부분(핵심): 격자 패턴(cross/T/corner/line) 흰
            # 픽셀수 기반 고도. 기존 ver/hor(라인 폭 기반) 추정보다는
            # 우선순위가 높고, 아래 아루코 추정보다는 낮다 - 아루코는
            # 물리적으로 고정된 마커 크기(0.4m)가 기준이라 가장 정밀하고,
            # 격자 픽셀수는 같은 원리(폭/면적 기반)를 선/패턴 전체로
            # 확장한 것뿐이라 그보다는 한 단계 아래로 둔다.
            #
            # final_altitude가 이미 None이면(mode2에서 BR 패턴 발견
            # 전처럼 "비전 자체를 아직 신뢰하지 않는다"고 위에서 이미
            # 결정된 상태) 그 상태를 존중해서 여기서 덮어쓰지 않는다.
            if final_altitude is not None and mask is not None:
                mask_pixel_count = cv2.countNonZero(mask)
                grid_pixel_altitude = estimate_altitude_from_grid_pixels(
                    mask_pixel_count, stable_pattern,
                    grid_pixel_altitude_table)
                if grid_pixel_altitude is not None and grid_pixel_altitude > 0:
                    final_altitude = grid_pixel_altitude

            if aruco_ids is not None and aruco_area > 0:
                # 수정된 부분: 위 디버그 화면 표시용으로 이미 계산해둔
                # aruco_offset_info_m을 그대로 재사용 (중복 계산 제거)
                aruco_offset_info = aruco_offset_info_m
                # 수정된 부분(핵심): 이미지 세로축(dy/y_offset_m)이 실제
                # 드론의 전후(x, forward), 가로축(dx/x_offset_m)이 좌우
                # (y, lateral)에 대응한다는 걸로 확인돼 축 자체를 바꿔
                # 매핑한다. 각 드론 축에 적용하던 부호(전후=반전,
                # 좌우=반전 없음)는 그대로 유지 - 그 부호는 실비행으로
                # 검증된 거라 축을 바꿔도 그대로 옮겨간다.
                #
                # 참고: 좌우(y)는 vio_publisher.py에서 한 번 더 부호 반전
                # (-offset_m)이 적용되므로, 여기서 그대로 넣으면 최종적
                # 으로는 vio_publisher에서 -(y_offset_m 소스값)이 나간다.
                final_offset_x = aruco_offset_info['y_offset_m']
                final_offset_y = aruco_offset_info['x_offset_m']
                # 추가된 부분: 이번에 실제로 본 값을 캐시에 저장 - 다음에
                # 아루코가 안 보이는 틱에 이 값을 계속 재사용한다.
                last_valid_aruco['x'] = final_offset_x
                last_valid_aruco['y'] = final_offset_y

                # 수정된 부분: 라인 테이블(estimate_altitude)과 동일한 규약 -
                # 유효하지 않으면 None이 아니라 -1.0을 반환하므로 조건도 맞춤
                aruco_altitude = estimate_altitude_from_aruco(
                    aruco_area, aruco_altitude_table)
                if aruco_altitude is not None and aruco_altitude > 0:
                    final_altitude = aruco_altitude

            vio_pub.update_latest(
                altitude_m=final_altitude,
                offset_m=final_offset_y,
                yaw_deg=vision_yaw_deg,
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                forward_m=final_offset_x,
            )

            cv2.putText(edges_color, f"Angle: {angle}deg [{direction_label}] (err:{score:.1f})", (10, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
            cv2.putText(edges_color, f"Altitude: {altitude:.2f}m", (10, 300),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 165), 2)

            if offset_m is None:
                offset_text = "Offset: N/A"
                offset_color = (128, 128, 128)
            else:
                offset_text = f"Offset: {offset_m:+.3f}m"
                offset_color = (0, 200, 255)
            cv2.putText(edges_color, offset_text, (10, 340),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, offset_color, 2)

            offset_str = "N/A" if offset_m is None else f"{offset_m:+.3f}m"
            yaw_log_str = "N/A" if vision_yaw_deg is None else f"{vision_yaw_deg:.1f}deg"
            print(f"모드: {mode_label} | 방향: {direction_label} | "
                  f"각도: {angle}° | 고도: {altitude:.2f}m | 오차: {score:.1f} | "
                  f"편차: {offset_str} | yaw: {yaw_log_str} | "
                  f"intersection: {pattern_name == 'skip'} | corner: {stable_pattern}")

            mode3_info = read_current_waypoint()
            if mode3_info is not None and features['line']:
                overlay_x = edges_color.shape[1] - 700
                draw_waypoint_overlay(
                    edges_color,
                    mode3_info,
                    cv2,
                    org=(overlay_x, 30)
                )

            if SHOW_DEBUG:
                if features['line']:
                    cv2.imshow(
                        "line_debug",
                        cv2.resize(edges_color, (1280, 720))
                    )
                else:
                    cv2.imshow(
                        "line_debug",
                        make_disabled_frame("mode1: line disabled")
                    )

                cv2.putText(
                    vision_display,
                    f"Active mode: {active_mode or 'waiting'}",
                    (10, vision_display.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )
                cv2.imshow(
                    "vision_debug",
                    cv2.resize(vision_display, (1280, 720))
                )

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        logger.save()
        vio_pub.stop()
        cam.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
