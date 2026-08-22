import csv

# 각도 → 컬럼 시작 인덱스 (top, mid, bot 순서). ver/hor 블록 모두 동일한 컬럼 구조.
ANGLE_COL = {
    0:  1,
    5:  4,
    10: 7,
    15: 10,
    20: 13,
    25: 16,
    30: 19,
}


def parse_csv_blocks(csv_path):
    """line_table.csv를 'ver' / 'hor' / 'aru' 세 블록으로 분리해서 반환.

    CSV 구조:
        ver,ang,...          <- 블록 마커
        alt,0,,,5,,,10,...   <- 헤더(각도)
        1.5,180,180,180,...  <- 데이터 (alt 마다 한 줄)
        ...
        hor,ang,...          <- 다음 블록 마커
        alt,0,,,5,,,10,...
        1.5,180,...
        ...
        aru,,,,...           <- 아루코 블록 마커 (ver/hor와 달리 'alt,0,...'
                                 같은 별도 헤더 행이 없다 - 마커 바로 다음
                                 줄부터 데이터)
        1,378225,,,...       <- 데이터 (alt, area)
        1.1,328329,,,...
        ...

    수정된 부분(핵심): 예전에는 'aru'/'aruco' 마커를 "값 하나짜리 설정 행"으로
    보고 그 한 줄만 건너뛰었다. 그런데 aru가 ver/hor처럼 alt별 전체 테이블
    (여러 행)로 커지면서, 마커 다음에 오는 실제 데이터 행들이 새 블록으로
    들어가지 못하고 직전 블록(hor)의 current_rows에 계속 이어붙는 버그가
    있었다 - hor 블록 뒤에 aru 데이터가 섞여 들어가서 hor 쪽에서
    'could not convert string to float' 에러가 나던 원인이 이거였다.
    이제 ver/hor와 동일하게 aru도 마커를 만나면 새 블록을 시작한다. 다만
    ver/hor와 달리 헤더 행이 따로 없으므로 마커 행 1개만 건너뛴다(ver/hor는
    마커+헤더 2개를 건너뜀).

    Returns:
        {'ver': [row, row, ...], 'hor': [row, row, ...], 'aru': [row, row, ...]}
        각 row는 csv.reader가 읽은 원본 리스트.
        ver/hor: (alt, top0, mid0, bot0, top5, ...)
        aru: (alt, area, ...)
    """
    with open(csv_path, 'r') as f:
        rows = list(csv.reader(f))

    blocks = {}
    current_block = None
    current_rows = []

    i = 0
    while i < len(rows):
        row = rows[i]
        if not row:
            i += 1
            continue
        if row[0] in ('ver', 'hor'):
            if current_block is not None:
                blocks[current_block] = current_rows
            current_block = row[0]
            current_rows = []
            i += 2  # 마커 행 + 'alt,0,...' 헤더 행 스킵
            continue
        if row[0] in ('aru', 'aruco'):
            if current_block is not None:
                blocks[current_block] = current_rows
            current_block = 'aru'
            current_rows = []
            i += 1  # 마커 행만 스킵 (별도 헤더 행 없음)
            continue
        current_rows.append(row)
        i += 1

    if current_block is not None:
        blocks[current_block] = current_rows

    return blocks


def load_aruco_target_area(csv_path):
    """line_table.csv 안의 'aru,<면적>' (또는 'aruco,<면적>') 행에서
    아루코 마커 목표 픽셀 면적을 읽어온다. 그런 행이 없으면 None 반환.

    mode1(고도 제어)과 mode3(마커 격자점 z보정)이 공통으로 쓰는 목표값이다.
    """
    with open(csv_path, 'r') as f:
        rows = list(csv.reader(f))

    for row in rows:
        if row and row[0] in ('aru', 'aruco') and len(row) > 1 and row[1] != '':
            try:
                return float(row[1])
            except ValueError:
                return None
    return None
