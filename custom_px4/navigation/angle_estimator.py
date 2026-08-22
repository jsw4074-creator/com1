import numpy as np

from line_table_io import ANGLE_COL, parse_csv_blocks


def load_angle_table(csv_path, block='ver'):
    """block: 'ver'(수직/전후진) 또는 'hor'(수평/좌우) 테이블 선택.

    각도별 top/mid/bot의 (모든 고도에 대한) 평균값을 반환한다.
    """
    blocks = parse_csv_blocks(csv_path)
    data_rows = blocks[block]

    angles = sorted(ANGLE_COL.keys())
    top_table = []
    mid_table = []
    bot_table = []

    for angle in angles:
        col_start = ANGLE_COL[angle]
        tops, mids, bots = [], [], []
        for row in data_rows:
            if not row or row[0] == '':
                continue
            tops.append(float(row[col_start]))
            mids.append(float(row[col_start + 1]))
            bots.append(float(row[col_start + 2]))
        top_table.append(np.mean(tops))
        mid_table.append(np.mean(mids))
        bot_table.append(np.mean(bots))

    return angles, top_table, mid_table, bot_table


def estimate_angle(top_val, mid_val, bot_val, angles, top_table, mid_table, bot_table):
    """단일(원본) 테이블과의 최근접(1-NN) 매칭.

    방향(정방향/반대방향) 판정은 더 이상 여기서 오차를 비교해 정하지 않는다.
    호출하는 쪽(camera_line_scanning.py)이 top_val/bot_val 부호 규칙으로 방향을
    먼저 정하고, 반대방향이면 top_val/bot_val을 미리 스왑해서 넘겨준다는 전제.
    (스왑된 입력을 원본 테이블과 비교하는 것과, 원본 입력을 스왑된 테이블과
    비교하는 것은 수학적으로 동일한 결과를 준다.)
    """
    best_angle = -1
    best_score = float('inf')

    for i, angle in enumerate(angles):
        score = 0
        count = 0
        if top_val > 0:
            score += abs(top_val - top_table[i])
            count += 1
        if mid_val > 0:
            score += abs(mid_val - mid_table[i])
            count += 1
        if bot_val > 0:
            score += abs(bot_val - bot_table[i])
            count += 1
        if count == 0:
            continue
        score /= count
        if score < best_score:
            best_score = score
            best_angle = angle

    return best_angle, best_score
