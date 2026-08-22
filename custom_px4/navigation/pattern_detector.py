import cv2
import numpy as np
from collections import Counter, deque

# ============================================================
# 인덱스 매핑 (패턴/ArUco 통합 인덱스)
#
# 0  -> ArUco 마커 (예외: 인덱스만이 아니라 (0, marker_id) 튜플로 반환)
# 1  -> cross      (십자 교차로)
# 2  -> T_up       (T자, 위쪽으로 열림)
# 3  -> T_down     (T자, 아래쪽으로 열림)
# 4  -> T_left     (T자, 왼쪽으로 열림)
# 5  -> T_right    (T자, 오른쪽으로 열림)
# 6  -> corner_TL  (모서리, 좌상단)
# 7  -> corner_TR  (모서리, 우상단)
# 8  -> corner_BL  (모서리, 좌하단)
# 9  -> corner_BR  (모서리, 우하단)
# -1 -> 그 외 전부 (unknown, line_V, line_H 포함 - 진짜 교차점이 아니므로 인덱스 없음)
# ============================================================
PATTERN_INDEX = {
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

GRID_SIZE = 5
THRESHOLD = 0.1

_BASE_PATTERNS = {
    'cross': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'T_up': np.array([
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'T_down': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]),
    'T_left': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'T_right': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 1, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'corner_TL': np.array([
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'corner_TR': np.array([
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'corner_BL': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 1, 1],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]),
    'corner_BR': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]),
    'line_V': np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ]),
    'line_H': np.array([
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]),
}

# 대각선 이동도 계산하는 패턴들(직선 이동뿐 아니라)
_CORNER_NAMES = {'corner_TL', 'corner_TR', 'corner_BL', 'corner_BR'}


def _shift_pattern(pattern, dy, dx):
    """패턴을 (dy, dx) 칸만큼 이동시키고, 반대편에서 '넘어온' 칸은 0으로 채운다."""
    p = np.roll(pattern, dy, axis=0)
    p = np.roll(p, dx, axis=1)
    p = p.copy()
    if dy > 0:
        p[:dy, :] = 0
    elif dy < 0:
        p[dy:, :] = 0
    if dx > 0:
        p[:, :dx] = 0
    elif dx < 0:
        p[:, dx:] = 0
    return p


def generate_shifted_patterns(patterns):
    shifted = {}
    for name, pattern in patterns.items():
        shifted[name] = pattern

        if name in _CORNER_NAMES:
            # 모서리 패턴은 8방향(직선 4 + 대각선 4) 전부 - 드론이 어느
            # 방향으로든 벗어날 수 있으므로
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    key = f'{name}_dy{dy}_dx{dx}'
                    shifted[key] = _shift_pattern(pattern, dy, dx)
        else:
            # 나머지 패턴은 기존처럼 직선 4방향만
            p_left = _shift_pattern(pattern, 0, -1)
            shifted[f'{name}_sl'] = p_left
            p_right = _shift_pattern(pattern, 0, 1)
            shifted[f'{name}_sr'] = p_right
            p_up = _shift_pattern(pattern, -1, 0)
            shifted[f'{name}_su'] = p_up
            p_down = _shift_pattern(pattern, 1, 0)
            shifted[f'{name}_sd'] = p_down
    return shifted


PATTERNS = generate_shifted_patterns(_BASE_PATTERNS)


def normalize_pattern(name):
    for suffix in ('_sl', '_sr', '_su', '_sd'):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    # '_dy-1_dx1' 같은 대각선 접미사
    idx = name.find('_dy')
    if idx != -1:
        return name[:idx]
    return name


def parse_pattern_shift(raw_name):
    """
    generate_shifted_patterns()가 만든 raw 매칭 키(예: 'cross_sl',
    'corner_TL_dy1_dx-1')에서 격자 셀 기준 이동량(dy, dx)을 뽑아낸다.
    이동이 없는 기본(중앙) 패턴이면 (0, 0).

    이 값은 "매칭된 모양이 5x5 격자에서 중앙 셀 기준 몇 칸 옮겨진
    버전이었는지"를 알려준다 - 즉 실제 교차점이 화면 중앙에서 대략
    몇 칸 벗어나 있었는지를 빠르게 추정하는 데 쓸 수 있다
    (grid_centering.quick_update의 seed 좌표 계산용).
    """
    if raw_name.endswith('_sl'):
        return 0, -1
    if raw_name.endswith('_sr'):
        return 0, 1
    if raw_name.endswith('_su'):
        return -1, 0
    if raw_name.endswith('_sd'):
        return 1, 0

    idx = raw_name.find('_dy')
    if idx != -1:
        rest = raw_name[idx + 3:]  # "{dy}_dx{dx}" 형태
        dy_str, dx_str = rest.split('_dx')
        return int(dy_str), int(dx_str)

    return 0, 0


def _find_best_pattern(grid, max_distance):
    best_name = 'unknown'
    best_dist = max_distance + 1
    for name, pattern in PATTERNS.items():
        dist = hamming_distance(grid, pattern)
        if dist < best_dist:
            best_dist = dist
            best_name = name
    return best_name, best_dist


def mask_to_grid(mask, grid_size=GRID_SIZE, threshold=THRESHOLD):
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


def hamming_distance(a, b):
    return int(np.sum(a != b))


def detect_pattern(mask, max_distance=6):
    grid = mask_to_grid(mask)
    best_name, best_dist = _find_best_pattern(grid, max_distance)
    if best_dist > max_distance:
        return 'unknown', best_dist, grid
    return normalize_pattern(best_name), best_dist, grid


def detect_pattern_with_shift(mask, max_distance=6):
    """
    detect_pattern()과 완전히 동일하게 동작하지만, 매칭된 템플릿이
    기준(중앙) 위치에서 몇 칸 이동(shift)된 버전이었는지(cell_dy,
    cell_dx)도 같이 반환한다. 기존 detect_pattern()의 반환 형식(3-tuple)을
    쓰는 다른 호출부를 안 건드리려고 별도 함수로 뺐다.

    반환: name, dist, grid, cell_dy, cell_dx
    """
    grid = mask_to_grid(mask)
    best_name, best_dist = _find_best_pattern(grid, max_distance)
    if best_dist > max_distance:
        return 'unknown', best_dist, grid, 0, 0
    cell_dy, cell_dx = parse_pattern_shift(best_name)
    return normalize_pattern(best_name), best_dist, grid, cell_dy, cell_dx




def get_indexed_result(pattern_name, aruco_id=None):
    """
    패턴 이름(및 필요시 ArUco marker_id)을 통합 인덱스로 변환한다.

    Args:
        pattern_name: detect_pattern()/detect_pattern_with_shift()가 반환한
                       이름 문자열 (normalize_pattern()을 거친 정규화된 이름,
                       예: 'cross', 'T_up', 'line_V', 'unknown' 등)
        aruco_id: ArUco 마커가 검출됐으면 그 ID(int), 아니면 None

    Returns:
        ArUco가 검출된 경우: (0, marker_id) 튜플
        9개 교차점 패턴 중 하나인 경우: 그 인덱스(int, 1~9)
        그 외(unknown, line_V, line_H 등): -1
    """
    if aruco_id is not None:
        return (0, aruco_id)
    return PATTERN_INDEX.get(pattern_name, -1)


def draw_grid(edges_color, mask, grid_size=GRID_SIZE, pattern_grid=None):
    h, w = mask.shape
    cell_h = h // grid_size
    cell_w = w // grid_size

    overlay = edges_color.copy()

    # 활성 셀 강조(패턴 grid가 전달된 경우)
    if pattern_grid is not None:
        for row in range(grid_size):
            for col in range(grid_size):
                if pattern_grid[row, col] == 1:
                    x1, y1 = col * cell_w, row * cell_h
                    x2, y2 = x1 + cell_w, y1 + cell_h
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 140, 255), -1)
        cv2.addWeighted(overlay, 0.15, edges_color, 0.85, 0, edges_color)

    mid = grid_size // 2
    for i in range(1, grid_size):
        # 중앙 라인(보통 십자가 지나는 곳) - 더 밝고 두껍게
        if i == mid or i == mid + (0 if grid_size % 2 else 1):
            color, thick = (0, 220, 220), 2
        else:
            color, thick = (90, 90, 90), 1
        cv2.line(edges_color, (i * cell_w, 0), (i * cell_w, h), color, thick, cv2.LINE_AA)
        cv2.line(edges_color, (0, i * cell_h), (w, i * cell_h), color, thick, cv2.LINE_AA)

    return edges_color


class PatternSmoother:
    def __init__(self, history_size=7):
        self.history = deque(maxlen=history_size)

    def update(self, pattern_name):
        self.history.append(pattern_name)
        if not self.history:
            return 'unknown'
        return Counter(self.history).most_common(1)[0][0]

    def reset(self):
        self.history.clear()


if __name__ == '__main__':
    print(f'총 패턴 수: {len(PATTERNS)}')
    for name in PATTERNS:
        print(name)
