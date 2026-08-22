"""
mode3_coord_overlay.py

follow_waypoints.py(mode3)와 camera_line_scanning.py는 서로 다른 프로세스라
변수를 직접 공유할 수 없다. 그래서 "지금 목표로 하는 격자점 좌표"를 작은
JSON 파일로 주고받는다:

  - follow_waypoints.py 쪽에서 매 틱 write_current_waypoint()로 갱신
  - camera_line_scanning.py 쪽에서 read_current_waypoint()로 읽어서
    draw_waypoint_overlay()로 자기 디버그 화면(edges_debug 등)에 그림

새 창을 따로 띄우지 않고, 기존에 있던 디버그 화면에 얹어서 보여주기 위한 용도.
"""

import json
import os
import time

DEFAULT_STATUS_PATH = '/tmp/mode3_current_wp.json'


def write_current_waypoint(index, total, x, y, is_marker, pattern,
                            path=DEFAULT_STATUS_PATH):
    """follow_waypoints.py(mode3)가 매 틱 호출 - 지금 목표 격자점 정보를 기록."""
    info = {
        'index': index,
        'total': total,
        'x': x,
        'y': y,
        'is_marker': is_marker,
        'pattern': pattern,
        'updated_at': time.time(),
    }
    try:
        tmp_path = path + '.tmp'
        with open(tmp_path, 'w') as f:
            json.dump(info, f)
        os.replace(tmp_path, path)  # 원자적 교체 - 읽는 쪽이 깨진 중간상태를 안 봄
    except Exception:
        pass  # 화면 표시용 부가 기능이라 실패해도 비행에 영향 주지 않음


def read_current_waypoint(path=DEFAULT_STATUS_PATH, max_age_sec=2.0):
    """camera_line_scanning.py가 매 프레임 호출 - 최근 정보면 dict, 없거나
    오래됐으면(mode3가 꺼졌거나 멈췄으면) None을 반환."""
    if not os.path.exists(path):
        return None
    try:
        with open(path, 'r') as f:
            info = json.load(f)
        if time.time() - info.get('updated_at', 0) > max_age_sec:
            return None  # 갱신이 멈춘 오래된 정보 - 표시 안 함
        return info
    except Exception:
        return None


def draw_waypoint_overlay(img, info, cv2_module, org=(10, 30)):
    """읽어온 정보를 img(edges_color 등)에 cv2.putText로 그린다.
    cv2를 인자로 받는 이유: 이 모듈 자체는 cv2에 의존하지 않게 해서, 굳이
    화면 출력이 필요 없는 쪽(follow_waypoints.py)에서는 cv2 없이도 쓸 수 있게."""
    if info is None:
        return img
    x, y_ = org
    cv2_module.putText(
        img,
        f"[mode3] WP {info['index'] + 1}/{info['total']}  "
        f"x={info['x']:.2f} y={info['y']:.2f}  marker={info['is_marker']}",
        (x, y_), cv2_module.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv2_module.putText(
        img, f"pattern: {info['pattern']}", (x, y_ + 30),
        cv2_module.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    return img
