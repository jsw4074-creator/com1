import csv

from dijkstra import explore, load_table_from_csv
from marker_csv_store import MarkerCsvStore, DEFAULT_CSV_PATH


def load_points_from_marker_store(csv_path=DEFAULT_CSV_PATH):
    """
    marker_csv_store.py가 실시간으로 채워둔 marker_positions.csv(실제 아루코
    인식 결과)를 {marker_id: (x, y)} 딕셔너리로 읽어온다.

    예전 load_points_from_csv()는 way_point.csv 순서대로 1,2,3...을 임시로
    붙였지만(TODO(OpenCV) 참고), 이제 marker_id는 카메라로 실제 인식한
    아루코 마커 번호(1, 2, 3, 4 ...) 그대로다 - 더 이상 placeholder가 아님.
    """
    store = MarkerCsvStore(csv_path)
    points = {}
    for marker_id in store.all_ids():
        x, y, _z, _tick = store.get(marker_id)
        points[marker_id] = (x, y)
    return points


# POINTS : {marker_id: (x, y)}  <- marker_csv_store.py가 실제 인식해서 저장한 좌표
POINTS = load_points_from_marker_store()


def quicksort(arr):
    if len(arr) <= 1:
        return arr

    pivot = arr[len(arr) // 2]
    pivot_key = pivot[0]  

    left  = [x for x in arr if x[0] < pivot_key]  
    mid   = [x for x in arr if x[0] == pivot_key] 
    right = [x for x in arr if x[0] > pivot_key]   

    return quicksort(left) + mid + quicksort(right)

if __name__ == "__main__":
    if not POINTS:
        print(f"아직 인식된 마커가 없습니다. 먼저 마커 인식 노드를 돌려서 "
              f"{DEFAULT_CSV_PATH} 를 채운 뒤 다시 실행하세요.")
    else:
        print("=== Exploration start ===")
        coords = list(POINTS.values())          
        order  = explore(coords)               

        coord_to_id = {coord: mid for mid, coord in POINTS.items()}
        route = [(coord_to_id[coord], coord) for coord in order]
        print(f"Route (visit order): {route}")

        print("\n=== Return path sort ===")
        sorted_route = quicksort(route)
        print(f"Return path (ID ascending): {sorted_route}")
