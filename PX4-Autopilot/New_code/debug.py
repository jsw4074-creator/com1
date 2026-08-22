import math

import cv2
import numpy as np
from config import DEBUG_ENABLED_DEFAULT

BAND_SIZE_PX = 20
MIN_LINE_PIXELS = 20

GRID_DEBUG_SIZE = 5
GRID_DEBUG_THRESHOLD = 0.1

_PATTERN_NAMES = {
    0: 'ArUco',
    1: 'cross',
    2: 'T_up',
    3: 'T_down',
    4: 'T_left',
    5: 'T_right',
    6: 'corner_TL',
    7: 'corner_TR',
    8: 'corner_BL',
    9: 'corner_BR',
    -1: 'none',
}

_aruco_dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)
_aruco_parameters = cv2.aruco.DetectorParameters()
_aruco_detector = cv2.aruco.ArucoDetector(
    _aruco_dictionary,
    _aruco_parameters,
)

_debug_windows_created = False

_debug_enabled = DEBUG_ENABLED_DEFAULT

def set_debug_enabled(enabled):
    """
    디버그 창 표시 여부를 켜고 끈다.

    False로 끄면 show_debug()가 아무 연산/그리기도 하지 않고 즉시
    리턴한다(창 그리기 비용 자체를 없앰) - 이미 떠 있던 창은 닫는다.
    """
    global _debug_enabled
    _debug_enabled = bool(enabled)
    if not _debug_enabled:
        close_debug_windows()


def is_debug_enabled():
    return _debug_enabled

def _ensure_debug_windows():
    global _debug_windows_created
    if _debug_windows_created:
        return
    for name in ('line_debug', 'aruco_debug'):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 1280, 720)
    _debug_windows_created = True


def _mask_to_debug_grid(mask, grid_size=GRID_DEBUG_SIZE, threshold=GRID_DEBUG_THRESHOLD):
    """마스크를 grid_size x grid_size 셀로 나눠 흰 비율이 threshold를
    넘는 셀을 1로 표시한 배열을 반환한다 (pattern_detector.py의
    mask_to_grid()와 동일한 방식 - 디버그 시각화 전용)."""
    h, w = mask.shape
    cell_h = h // grid_size
    cell_w = w // grid_size
    grid = np.zeros((grid_size, grid_size), dtype=int)
    for row in range(grid_size):
        for col in range(grid_size):
            y1 = row * cell_h
            y2 = y1 + cell_h
            x1 = col * cell_w
            x2 = x1 + cell_w
            cell = mask[y1:y2, x1:x2]
            ratio = cv2.countNonZero(cell) / cell.size
            grid[row, col] = 1 if ratio > threshold else 0
    return grid


def _draw_grid_overlay(display, grid, grid_size=GRID_DEBUG_SIZE):
    """5x5 격자선 + 활성 셀 반투명 하이라이트를 display에 직접 그린다
    (pattern_detector.py의 draw_grid()와 동일한 스타일)."""
    h, w = display.shape[:2]
    cell_h = h // grid_size
    cell_w = w // grid_size

    overlay = display.copy()
    for row in range(grid_size):
        for col in range(grid_size):
            if grid[row, col] == 1:
                x1, y1 = col * cell_w, row * cell_h
                x2, y2 = x1 + cell_w, y1 + cell_h
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 140, 255), -1)
    cv2.addWeighted(overlay, 0.15, display, 0.85, 0, display)

    mid = grid_size // 2
    for i in range(1, grid_size):
        if i == mid or i == mid + (0 if grid_size % 2 else 1):
            color, thick = (0, 220, 220), 2
        else:
            color, thick = (90, 90, 90), 1
        cv2.line(display, (i * cell_w, 0), (i * cell_w, h), color, thick, cv2.LINE_AA)
        cv2.line(display, (0, i * cell_h), (w, i * cell_h), color, thick, cv2.LINE_AA)


def draw_debug_overlay(pattern, mask):
    """
    디버그용 오버레이를 그린 컬러 이미지를 반환한다 (BGR).

    - 교차로(pattern 1~9): 5x5 격자선 + 활성 셀 하이라이트만 그린다
      (화살표/중심점은 그리지 않음 - offset 수치는 show_debug()의
      텍스트로 이미 표시됨).
    - 일반 라인(pattern == -1): 컨투어(노랑) + 측정 밴드
      (빨강=top/left, 초록=mid, 파랑=bottom/right) + Top/Mid/Bottom/
      Diff 수치 + MODE 라벨.
    """
    if mask is None or mask.size == 0:
        return None

    display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    h, w = mask.shape[:2]

    if 1 <= pattern <= 9:
        grid = _mask_to_debug_grid(mask)
        _draw_grid_overlay(display, grid)
        return display

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return display

    contour = max(contours, key=cv2.contourArea)
    cv2.drawContours(display, [contour], -1, (0, 255, 255), 2)

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
        top_ = int(np.percentile(rows, 2))
        bottom_ = int(np.percentile(rows, 98))
        return float(bottom_ - top_), top_, bottom_

    if is_vertical:
        y_top, y_mid, y_bot = y + 20, y + rect_h // 2, y + rect_h - 20
        top_len, l_top, r_top = band_width(y_top)
        mid_len, l_mid, r_mid = band_width(y_mid)
        bot_len, l_bot, r_bot = band_width(y_bot)

        if l_top >= 0:
            cv2.line(display, (l_top, y_top), (r_top, y_top), (0, 0, 255), 2)
        if l_mid >= 0:
            cv2.line(display, (l_mid, y_mid), (r_mid, y_mid), (0, 255, 0), 2)
        if l_bot >= 0:
            cv2.line(display, (l_bot, y_bot), (r_bot, y_bot), (255, 0, 0), 2)

        diff = bot_len - top_len if (top_len > 0 and bot_len > 0) else -1.0
        cv2.putText(display, f"Top: {top_len:.1f}", (10, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(display, f"Mid: {mid_len:.1f}", (10, 290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"Bot: {bot_len:.1f}", (10, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(display, f"Diff: {diff:.1f}", (10, 350),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display, "MODE: VERTICAL", (10, 380),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    else:
        x_left, x_mid, x_right = x + 20, x + rect_w // 2, x + rect_w - 20
        left_len, t_left, b_left = band_height(x_left)
        mid_len, t_mid, b_mid = band_height(x_mid)
        right_len, t_right, b_right = band_height(x_right)

        if t_left >= 0:
            cv2.line(display, (x_left, t_left), (x_left, b_left), (0, 0, 255), 2)
        if t_mid >= 0:
            cv2.line(display, (x_mid, t_mid), (x_mid, b_mid), (0, 255, 0), 2)
        if t_right >= 0:
            cv2.line(display, (x_right, t_right), (x_right, b_right), (255, 0, 0), 2)

        diff = right_len - left_len if (left_len > 0 and right_len > 0) else -1.0
        cv2.putText(display, f"Left: {left_len:.1f}", (10, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(display, f"Mid: {mid_len:.1f}", (10, 290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"Right: {right_len:.1f}", (10, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(display, f"Diff: {diff:.1f}", (10, 350),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display, "MODE: HORIZONTAL", (10, 380),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    return display


def show_debug(frame, mask, pattern, marker_id,
               altitude=None, err_x=None, err_y=None,
               roll=None, pitch=None, yaw=None):
    if not _debug_enabled:
        return

    _ensure_debug_windows()

    pattern_name = _PATTERN_NAMES.get(pattern, str(pattern))

    if mask is not None:
        mask_display = draw_debug_overlay(pattern, mask)
        if mask_display is None:
            mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        text_x, text_y, line_h = 10, 30, 32

        def draw_mask(text, color=(0, 255, 255)):
            nonlocal text_y
            cv2.putText(
                mask_display, text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            text_y += line_h

        draw_mask(f'pattern: {pattern_name} ({pattern})')
        mask_pixel_count = cv2.countNonZero(mask)
        draw_mask(f'mask px count: {mask_pixel_count}')

        if altitude is not None and altitude > 0:
            draw_mask(f'altitude: {altitude:.2f}m', (0, 255, 165))
        else:
            draw_mask('altitude: N/A', (100, 100, 100))

        if err_x is not None and err_y is not None:
            draw_mask(
                f'err_x={err_x:+.3f}m err_y={err_y:+.3f}m',
                (0, 200, 255))

        if yaw is not None:
            draw_mask(f'yaw={math.degrees(yaw):+.1f}deg', (255, 0, 255))
        if pitch is not None:
            draw_mask(f'pitch={math.degrees(pitch):+.1f}deg', (255, 0, 255))
        if roll is not None:
            draw_mask(f'roll={math.degrees(roll):+.1f}deg', (255, 0, 255))

        cv2.imshow('line_debug', mask_display)

    if frame is not None:
        aruco_display = frame.copy()
        corners, ids, _ = _aruco_detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(aruco_display, corners, ids)

        text_x, text_y, line_h = 10, 30, 32

        def draw_aruco(text, color=(0, 255, 0)):
            nonlocal text_y
            cv2.putText(
                aruco_display, text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            text_y += line_h

        draw_aruco(f'marker_id: {marker_id}')

        if pattern == 0 and err_x is not None and err_y is not None:
            draw_aruco(
                f'ArUco offset x={err_x:+.3f}m y={err_y:+.3f}m',
                (0, 255, 255))

        cv2.imshow('aruco_debug', aruco_display)

    cv2.waitKey(1)


def close_debug_windows():
    global _debug_windows_created
    cv2.destroyAllWindows()
    _debug_windows_created = False
