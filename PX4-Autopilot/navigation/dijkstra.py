import heapq
import csv
from itertools import permutations

STEP = 3            # 전체지도
WIDTH = 32.5
HEIGHT = 15
START = (5.5, 0)


def make_range(start, end, step):
    n_steps = int(round((end - start) / step)) + 1
    return [round(start + i * step, 3) for i in range(n_steps)]


XS = make_range(5.5, WIDTH, STEP)     # 5.5, 8.5, 11.5, ..., 29.5
YS = make_range(0, HEIGHT, STEP)      # 0, 3, 6, ..., 15
NODES = [(x, y) for x in XS for y in YS]

def build_graph():  #그래프 생성
    graph = {n: [] for n in NODES}
    for (x, y) in NODES:
        for dx, dy in ((STEP, 0), (-STEP, 0), (0, STEP), (0, -STEP)):
            nb = (round(x + dx, 3), round(y + dy, 3))
            if nb in graph:
                graph[(x, y)].append((nb, STEP))
    return graph

def dijkstra(graph, src):        #다익스트라 알고리즘
    dist = {n: float('inf') for n in graph}
    prev = {n: None for n in graph}
    dist[src] = 0
    pq = [(0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev

def reconstruct(prev, src, dst):    #길을 알려줌
    path = []
    cur = dst
    while cur is not None:
        path.append(cur)
        if cur == src:
            break
        cur = prev[cur]
    path.reverse()
    return path

def markers_from_table(table):   #잘못된 좌표를 걸러네는 로직
    markers = []
    for i, (x, y) in enumerate(table, 1):
        key = (round(x, 3), round(y, 3))
        if key not in NODES:
            raise ValueError(f"마커{i} {(x, y)} 는 격자점이 아닙니다.")
        markers.append(key)
    return markers

def load_table_from_csv(path):     #csv에서 자료 읽기
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return [(float(row["x"]), float(row["y"])) for row in reader]

def save_path_to_csv(path_points, out_path):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for (x, y) in path_points:
            writer.writerow([x, y])

def best_tour(graph, start, markers):
    points = [start] + markers
    dist = {}
    prev = {}
    for p in points:
        d, pr = dijkstra(graph, p)
        dist[p] = d
        prev[p] = pr

    best = None
    for perm in permutations(markers):
        seq = [start] + list(perm)
        total = sum(dist[seq[i]][seq[i + 1]] for i in range(len(seq) - 1))
        if best is None or total < best[0]:
            best = (total, perm)

    total, order = best
    seq = [start] + list(order)
    segments = []          # 각 구간의 격자점 리스트
    full_path = [start]
    for i in range(len(seq) - 1):
        seg = reconstruct(prev[seq[i]], seq[i], seq[i + 1])
        segments.append(seg)
        full_path.extend(seg[1:])  # 이어지는 구간의 시작점 중복 제거
    return total, list(order), full_path, segments

def explore(coords):
    graph = build_graph()
    markers = markers_from_table(coords)   # 좌표가 격자점인지 검증
    total, order, full_path, segments = best_tour(graph, START, markers)
    return order

if __name__ == "__main__":
    graph = build_graph()
    markers = markers_from_table(load_table_from_csv("way_point.csv"))

    print(f"Start: {START}")
    print(f"Markers: {markers}\n")

    total, order, path, segments = best_tour(graph, START, markers)

    print(f"Best visit order: {START} -> " + " -> ".join(map(str, order)))
    print(f"Total distance: {total} m\n")

    print("-- Segment details (grid points) --")
    seq = [START] + order
    for i, seg in enumerate(segments):
        dist_seg = (len(seg) - 1) * STEP
        arrow = " -> ".join(map(str, seg))
        print(f"Segment {i+1}: {seq[i]} -> {seq[i+1]}  ({dist_seg}m)")
        print(f"        {arrow}")

    print(f"\nFull path ({len(path)} grid points):")
    print("  " + " -> ".join(map(str, path)))

    save_path_to_csv(path, "path_result.csv")
    print(f"\n[saved] path_result.csv ({len(path)} points)")
