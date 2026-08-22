from line_table_io import ANGLE_COL, parse_csv_blocks


def load_altitude_table(csv_path, block='ver'):
    """block: 'ver'(수직/전후진) 또는 'hor'(수평/좌우) 테이블 선택.

    각도별로 (alt, top, mid, bot) 원본 데이터를 그대로 보관한다 (평균 아님).
    """
    blocks = parse_csv_blocks(csv_path)
    data_rows = blocks[block]

    table = {}
    for angle, col_start in ANGLE_COL.items():
        entries = []
        for row in data_rows:
            if not row or row[0] == '':
                continue
            alt = float(row[0])
            top = float(row[col_start])
            mid = float(row[col_start + 1])
            bot = float(row[col_start + 2])
            entries.append((alt, top, mid, bot))
        table[angle] = entries

    return table


def _out_of_range(top_val, mid_val, bot_val, entries):
    """entries는 CSV 행 순서(alt 오름차순: 1.5 -> 2.5)를 그대로 보존하고 있다는
    전제 하에, 첫 항목(alt 최소=1.5m, 픽셀값 최대)과 마지막 항목(alt 최대=2.5m,
    픽셀값 최소)을 범위 경계로 쓴다.

    고도가 높아질수록(멀어질수록) 픽셀값이 단조 감소하므로:
      - 측정값이 2.5m 행 값보다 더 작다 -> 실제 고도는 2.5m보다 높음 (범위 밖)
      - 측정값이 1.5m 행 값보다 더 크다 -> 실제 고도는 1.5m보다 낮음 (범위 밖)
    노이즈로 인한 오탐을 줄이기 위해, 유효한 채널(top/mid/bot 중 >0인 것) 전부가
    같은 방향으로 범위를 벗어날 때만 '범위 밖'으로 판단한다.

    top_val/bot_val은 호출하는 쪽(camera_line_scanning.py)에서 방향 판정을 마치고
    필요하면 이미 스왑해서 넘어온 값이라는 전제 - 여기서는 추가로 스왑하지 않는다.
    """
    _, t_min, m_min, b_min = entries[0]   # alt 1.5m 경계 (픽셀값 최대)
    _, t_max, m_max, b_max = entries[-1]  # alt 2.5m 경계 (픽셀값 최소)

    too_high = []  # 2.5m보다 높음(멀음) 후보
    too_low = []   # 1.5m보다 낮음(가까움) 후보

    if top_val > 0:
        too_high.append(top_val < t_max)
        too_low.append(top_val > t_min)
    if mid_val > 0:
        too_high.append(mid_val < m_max)
        too_low.append(mid_val > m_min)
    if bot_val > 0:
        too_high.append(bot_val < b_max)
        too_low.append(bot_val > b_min)

    if too_high and all(too_high):
        return True
    if too_low and all(too_low):
        return True
    return False


def estimate_altitude(top_val, mid_val, bot_val, angle_deg, table):
    """top_val/bot_val은 호출하는 쪽에서 방향 판정을 마치고 필요하면 이미
    스왑해서 넘어온 값이라는 전제 - 여기서는 원본 테이블 그대로 비교한다.

    실제 고도가 테이블 범위(1.5~2.5m)를 벗어난 것으로 판단되면(_out_of_range)
    가장 가까운 경계값으로 억지 매칭하지 않고 -1.0을 반환한다.
    """
    if angle_deg not in table:
        return -1.0

    entries = table[angle_deg]
    if not entries:
        return -1.0

    if _out_of_range(top_val, mid_val, bot_val, entries):
        return -1.0

    best_alt = -1.0
    best_score = float('inf')

    for alt, t_ref, m_ref, b_ref in entries:
        score = 0
        count = 0
        if top_val > 0:
            score += abs(top_val - t_ref)
            count += 1
        if mid_val > 0:
            score += abs(mid_val - m_ref)
            count += 1
        if bot_val > 0:
            score += abs(bot_val - b_ref)
            count += 1
        if count == 0:
            continue
        score /= count
        if score < best_score:
            best_score = score
            best_alt = alt

    return best_alt


# ------------------------------------------------------------------
# 추가된 부분: 아루코 마커 고도 추정도 라인 테이블(ver/hor)과 완전히 같은
# "테이블 로드 -> 범위 밖 판정 -> 최근접 매칭" 구조로 통일한다.
# 기존에 aruco_detector.py 쪽에서 쓰던 기준 면적 1개(aruco_ref_area_px2)
# + 역제곱 공식 방식 대신, line_table.csv의 'aru' 블록(alt, area) 전체를
# 그대로 테이블로 사용한다. top/mid/bot 세 채널이 area 한 채널로 줄어든
# 것만 다르고 판정 로직 자체는 동일하다.
# ------------------------------------------------------------------
def load_aruco_altitude_table(csv_path, block='aru'):
    """'aru' 블록은 ver/hor처럼 각도별 컬럼이 없고 (alt, area) 한 쌍만 있다.
    ANGLE_COL을 쓰지 않고 row[0]=alt, row[1]=area만 그대로 읽는다.

    line_table.csv를 다시 측정/작성하는 중이라면, 아직 채우지 못한 구간은
    area가 1처럼 실제 값이 아닌 placeholder일 수 있다 - 이 함수는 그런
    placeholder 여부를 판단하지 않고 CSV에 있는 값을 그대로 테이블에 담으므로,
    실제 측정값으로 채워 넣기 전까지는 그 구간의 추정이 부정확할 수 있다.
    """
    blocks = parse_csv_blocks(csv_path)
    data_rows = blocks[block]

    table = []
    for row in data_rows:
        if not row or row[0] == '':
            continue
        alt = float(row[0])
        area = float(row[1])
        table.append((alt, area))

    return table


def _aruco_out_of_range(area_val, entries):
    """entries는 CSV 행 순서(alt 오름차순)를 그대로 보존한다는 전제 하에,
    첫 항목(alt 최소, area 최대)과 마지막 항목(alt 최대, area 최소)을
    범위 경계로 쓴다 (라인 테이블의 _out_of_range와 동일한 방식).

    고도가 높아질수록(멀어질수록) 마커가 작게 보여 area가 단조 감소하므로:
      - 측정값이 alt 최대 행의 area보다 더 작다 -> 실제 고도는 테이블 최대
        alt보다 높음 (범위 밖)
      - 측정값이 alt 최소 행의 area보다 더 크다 -> 실제 고도는 테이블 최소
        alt보다 낮음 (범위 밖)
    """
    _, area_min_alt_max = entries[-1]  # alt 최대 경계 (area 최소)
    _, area_max_alt_min = entries[0]   # alt 최소 경계 (area 최대)

    if area_val < area_min_alt_max:
        return True
    if area_val > area_max_alt_min:
        return True
    return False


def estimate_altitude_from_aruco(area_val, table):
    """area_val: 검출된 아루코 마커의 픽셀 면적(px^2).
    table: load_aruco_altitude_table()의 결과.

    라인 테이블의 estimate_altitude와 동일하게, 테이블 범위를 벗어난
    것으로 판단되면(_aruco_out_of_range) 가장 가까운 경계값으로 억지
    매칭하지 않고 -1.0을 반환한다. 보간은 하지 않고 라인 테이블과 같이
    가장 가까운 행을 그대로 선택한다.
    """
    if area_val <= 0 or not table:
        return -1.0

    if _aruco_out_of_range(area_val, table):
        return -1.0

    best_alt = -1.0
    best_score = float('inf')

    for alt, area_ref in table:
        score = abs(area_val - area_ref)
        if score < best_score:
            best_score = score
            best_alt = alt

    return best_alt


# ------------------------------------------------------------------
# 추가된 부분(핵심): 격자 패턴(cross/T/corner/line)의 흰 픽셀 총 개수 ->
# 고도(m) 추정. line_table.csv의 'aru' 블록에 아루코 area 옆에 추가된
# L(corner/line, 5칸), T(T자, 7칸), X(cross, 9칸) 세 컬럼을 그대로 쓴다.
#
# 왜 3개로 나눴는지: pattern_detector.py의 11개 기본 패턴 중, 5x5 템플릿
# 에서 실제로 채워진 칸 수(=화면에서 차지하는 면적)가 cross=9, T계열
# (T_up/down/left/right)=7, corner계열(TL/TR/BL/BR)과 line계열(V/H)=5로
# 방향만 다르고 면적은 3종류뿐이다. 그래서 방향 상관없이 이 3그룹
# (X/T/L) 중 하나로만 매핑하면 같은 고도-픽셀수 테이블을 재사용할 수 있다.
#
# 아루코/라인(ver·hor) 테이블과 똑같이 "범위 밖 판정 -> 최근접 매칭"
# 구조를 그대로 따른다 - 보간하지 않고 CSV에 있는 행 중 가장 가까운
# 걸 그대로 고른다.
# ------------------------------------------------------------------

# pattern_detector.normalize_pattern()이 반환하는 이름 -> L/T/X 그룹 매핑
GRID_SHAPE_GROUP = {
    'cross': 'X',
    'T_up': 'T', 'T_down': 'T', 'T_left': 'T', 'T_right': 'T',
    'corner_TL': 'L', 'corner_TR': 'L', 'corner_BL': 'L', 'corner_BR': 'L',
    'line_V': 'L', 'line_H': 'L',
}

# 'aru' 블록 컬럼 순서: alt(0), area(1), L(2), T(3), X(4)
_GRID_GROUP_COL = {'L': 2, 'T': 3, 'X': 4}


def load_grid_pixel_altitude_table(csv_path, block='aru'):
    """'aru' 블록에서 L/T/X 세 그룹별 (alt, pixel_count) 테이블을 읽어온다.

    아직 측정 안 해서 0으로 채워둔 칸(placeholder)은 진짜 데이터가 아니므로
    테이블에서 아예 제외한다 - 안 그러면 area=0을 진짜 값처럼 매칭 후보로
    써버려서 엉뚱한 고도가 나온다.

    반환: {'L': [(alt, px), ...], 'T': [...], 'X': [...]}
          아직 데이터가 하나도 없는 그룹은 빈 리스트.
    """
    blocks = parse_csv_blocks(csv_path)
    data_rows = blocks.get(block, [])

    table = {'L': [], 'T': [], 'X': []}

    for row in data_rows:
        if not row or row[0] == '':
            continue
        alt = float(row[0])
        for group, col in _GRID_GROUP_COL.items():
            if col >= len(row) or row[col] == '':
                continue
            value = float(row[col])
            if value <= 0:
                continue  # 아직 안 채운 placeholder - 테이블에서 제외
            table[group].append((alt, value))

    return table


def _grid_out_of_range(pixel_val, entries):
    """_aruco_out_of_range와 동일한 방식 - entries는 alt 오름차순
    (픽셀수 내림차순) 정렬돼있다는 전제.
    """
    _, px_min_alt_max = entries[-1]  # alt 최대 경계 (픽셀수 최소)
    _, px_max_alt_min = entries[0]   # alt 최소 경계 (픽셀수 최대)

    if pixel_val < px_min_alt_max:
        return True
    if pixel_val > px_max_alt_min:
        return True
    return False


def estimate_altitude_from_grid_pixels(pixel_val, pattern_name, table):
    """pixel_val: 마스크에서 센 흰 픽셀 총 개수(cv2.countNonZero(mask)).
    pattern_name: pattern_detector가 반환한 정규화된 패턴 이름
                  (예: 'cross', 'T_up', 'corner_BR', 'line_V').
    table: load_grid_pixel_altitude_table()의 결과.

    pattern_name을 GRID_SHAPE_GROUP으로 L/T/X 중 하나로 변환한 다음,
    그 그룹 테이블에서 아루코 고도 추정과 동일하게 범위 밖 판정 후
    최근접 매칭한다. 매칭할 데이터가 없거나(그룹 테이블이 비어있음)
    범위를 벗어나면 -1.0을 반환한다(아루코/라인 테이블과 동일 규약).
    """
    if pixel_val <= 0 or not table:
        return -1.0

    group = GRID_SHAPE_GROUP.get(pattern_name)
    if group is None:
        return -1.0

    entries = table.get(group, [])
    if not entries:
        return -1.0

    if _grid_out_of_range(pixel_val, entries):
        return -1.0

    best_alt = -1.0
    best_score = float('inf')

    for alt, px_ref in entries:
        score = abs(pixel_val - px_ref)
        if score < best_score:
            best_score = score
            best_alt = alt

    return best_alt
