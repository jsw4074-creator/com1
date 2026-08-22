"""
marker_csv_store.py

아루코 마커 감지 결과(marker_id, x, y, z)를 CSV로 저장/로드하는 저장소.
마커 인식(detect_aruco 등) 로직과는 완전히 분리되어 있음 - 이미 갖고 있는
인식 코드에서 marker_id와 그때의 위치(x, y, z)만 넘겨주면 됨.

나중에 이 CSV를 퀵소트 등으로 후처리할 걸 감안해서, 저장 시점엔 정렬하지
않고 marker_id -> (x, y, z, tick) 딕셔너리로만 관리한다. 정렬은
sort_by_marker_id() / quicksort_by_marker_id() 같은 별도 함수에서 처리.

CSV 포맷: marker_id,x,y,z,detected_at_tick
"""

import csv
import os


DEFAULT_CSV_PATH = '/home/jiseungwoo/Desktop/com1/PX4-Autopilot/navigation/marker_positions.csv'


class MarkerCsvStore:
    """
    마커 감지 결과를 관리하는 저장소.

    같은 marker_id가 여러 번 감지돼도 기본적으로는 '처음' 위치만 기록하고
    이후 감지는 무시한다(흔들리는 값으로 덮어써서 좌표가 틀어지는 걸 방지).
    덮어쓰고 싶으면 add(..., overwrite=True)로 호출.
    """

    def __init__(self, csv_path=DEFAULT_CSV_PATH):
        self.csv_path = csv_path
        self.records = {}  # marker_id(int) -> (x, y, z, detected_at_tick)
        self._load_existing()

    def _load_existing(self):
        if not os.path.exists(self.csv_path):
            return
        with open(self.csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                marker_id = int(row['marker_id'])
                self.records[marker_id] = (
                    float(row['x']),
                    float(row['y']),
                    float(row.get('z', 0.0) or 0.0),
                    int(row.get('detected_at_tick', 0) or 0),
                )

    def has(self, marker_id):
        return marker_id in self.records

    def reset(self):
        """이전 실행에서 남은 기록을 전부 지우고 빈 상태로 새로 시작한다.
        (mode3처럼 매번 새 미션인 경우, __init__이 기존 파일을 불러오는 게
        오히려 옛날 판 데이터를 계속 끌고 다니게 만들 수 있어서 필요함)"""
        self.records = {}
        self.save()

    def add(self, marker_id, x, y, z=0.0, tick=0, overwrite=False):
        """
        새 마커 감지 결과를 등록한다.

        Returns:
            True  - 새로 기록됨(또는 overwrite=True로 갱신됨)
            False - 이미 있어서 기록 안 함(overwrite=False인 경우)
        """
        marker_id = int(marker_id)
        if marker_id in self.records and not overwrite:
            return False
        self.records[marker_id] = (float(x), float(y), float(z), int(tick))
        return True

    def get(self, marker_id):
        return self.records.get(int(marker_id))

    def all_ids(self):
        return list(self.records.keys())

    def save(self):
        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['marker_id', 'x', 'y', 'z', 'detected_at_tick'])
            for marker_id in sorted(self.records.keys()):
                x, y, z, tick = self.records[marker_id]
                writer.writerow([marker_id, x, y, z, tick])


def quicksort_by_marker_id(records):
    """
    records: (marker_id, x, y, z) 튜플 리스트.
    marker_id 기준 퀵소트. 나중에 다른 기준(예: 거리)으로 정렬할 때도
    이 함수 시그니처만 맞춰서 key를 바꾸면 됨.
    """
    if len(records) <= 1:
        return list(records)

    pivot = records[len(records) // 2]
    pivot_id = pivot[0]

    less = [r for r in records if r[0] < pivot_id]
    equal = [r for r in records if r[0] == pivot_id]
    greater = [r for r in records if r[0] > pivot_id]

    return quicksort_by_marker_id(less) + equal + quicksort_by_marker_id(greater)


if __name__ == '__main__':
    # 간단한 사용 예시 (실제 인식 로직 없이 저장소 동작만 확인)
    store = MarkerCsvStore('/tmp/marker_positions_test.csv')
    store.add(3, x=23.5, y=12.0, z=-2.0, tick=100)
    store.add(1, x=11.5, y=6.0, z=-2.0, tick=50)
    store.add(2, x=8.5, y=12.0, z=-2.0, tick=150)
    store.add(1, x=999.0, y=999.0, z=999.0, tick=200)  # 이미 있어서 무시됨
    store.save()

    print("저장된 마커:", store.all_ids())
    sorted_records = quicksort_by_marker_id(
        [(mid, *store.get(mid)[:3]) for mid in store.all_ids()]
    )
    print("퀵소트 결과:", sorted_records)
