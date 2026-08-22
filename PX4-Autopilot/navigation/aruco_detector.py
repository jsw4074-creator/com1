import math

import cv2
import numpy as np

# 모듈 로드시 1번만 초기화
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# ------------------------------------------------------------------
# 수정 이력(중요): 처음엔 "마커 테두리와 격자선이 맞닿아 십자 모양으로
# 합쳐져서 사각형 윤곽선 검출 자체가 실패한다"는 가설로 morphological
# opening(침식->팽창) 전처리를 시도했었다. 그런데 실제로 격자점 위에서도
# 마커 인식이 잘 되는 동료 코드를 확인해보니, 그쪽은 전처리가 아니라
# DetectorParameters를 관대하게 튜닝하는 방식으로 해결하고 있었다.
# 실제로 작동이 검증된 이 방식으로 교체한다 (opening 전처리는 제거).
#
# - minMarkerPerimeterRate: 0.03(기본) -> 0.01로 낮춤. 화면에서 작게
#   찍힌 마커까지 후보로 허용.
# - adaptiveThreshWinSizeMax: 23(기본) -> 53으로 넓힘. 더 넓은 지역
#   윈도우로 적응형 이진화를 시도하게 해서, 격자선처럼 큰 스케일의 검은
#   영역 안에서도 마커 패턴을 더 안정적으로 분리해낸다. 격자점 위 마커
#   인식 문제의 핵심 원인으로 보인다.
# - polygonalApproxAccuracyRate: 0.03(기본) -> 0.05로 완화. 격자선과
#   살짝 얽혀 완벽한 정사각형이 아니게 검출된 윤곽선도 사각형 후보로
#   더 관대하게 통과시킨다.
# ------------------------------------------------------------------
parameters.minMarkerPerimeterRate = 0.01
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 10
parameters.maxMarkerPerimeterRate = 4.0
parameters.polygonalApproxAccuracyRate = 0.05

detector = cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_aruco(frame):
    corners, ids, _ = detector.detectMarkers(frame)

    area = -1.0
    if ids is not None and len(corners) > 0:
        pts = corners[0][0].astype('float32')
        top_left     = pts[0]
        top_right    = pts[1]
        bottom_left  = pts[3]
        width_px  = int(np.linalg.norm(top_right - top_left))
        height_px = int(np.linalg.norm(bottom_left - top_left))
        area = width_px * height_px

    return ids, area, corners


# ------------------------------------------------------------------
# 추가된 부분(진단용): detectMarkers()의 3번째 반환값(rejected candidates)
# 까지 노출하는 디버그 전용 함수. 기존 detect_aruco()는 반환 형식을 안
# 건드리기 위해 그대로 두고 별도로 만든다.
#
# rejected candidates가 뭔지: OpenCV ArUco는 1단계에서 "사각형처럼 생긴
# 윤곽선 후보"를 다 모으고, 2단계에서 그 안의 비트 패턴을 읽어서 사전
# (dictionary)에 있는 ID와 매칭되면 "검출 성공", 매칭 안 되면 그 사각형은
# rejected로 버려진다.
#
# 이걸 보면 원인이 둘 중 뭔지 구분된다:
#   - rejected candidates가 마커 위치 근처에 하나도 없다
#     -> 애초에 "사각형 후보" 자체를 못 찾은 것 (기하학적 검출 실패,
#        예: 격자선과 합쳐져서 사각형 윤곽선이 안 나오는 경우)
#   - rejected candidates가 마커 위치에 있다(사각형은 찾았다)
#     -> 사각형은 찾았는데 내부 비트 패턴 해독에 실패한 것 (원근 왜곡,
#        해상도 부족, dictionary 불일치, 코너 순서 문제 등 완전히 다른
#        원인 계열)
# ------------------------------------------------------------------
def detect_aruco_debug(frame):
    """
    반환: (corners, ids, rejected) - detectMarkers()의 원본 반환값 그대로.
    시각화 예시:
        corners, ids, rejected = detect_aruco_debug(frame)
        vis = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)  # 초록 - 검출 성공
        cv2.aruco.drawDetectedMarkers(vis, rejected, borderColor=(0, 255, 255))  # 노랑 - 거부된 후보
    """
    corners, ids, rejected = detector.detectMarkers(frame)
    return corners, ids, rejected


# ------------------------------------------------------------------
# 아루코 마커 픽셀 면적(area, px^2) -> 실제 고도(m) 변환.
# line_table.csv의 'aru' 행에 저장된 값(ARUCO_REF_ALTITUDE_M에서 관측되는
# 마커 픽셀 면적 기준점)을 이용해, 다른 고도를 역산한다. 평면 마커를 카메라가
# 거의 수직으로 내려다볼 때, 관측 면적은 카메라-마커 거리(고도)의 제곱에
# 반비례한다(핀홀 카메라 모델의 역제곱 법칙). follow_waypoints.py에 있던
# 로직을 여기로 옮겨서 camera_line_scanning.py와 공용으로 쓴다.
# ------------------------------------------------------------------
ARUCO_REF_ALTITUDE_M = 2.0  # line_table.csv aru 값이 측정된 기준 고도(m)


def estimate_altitude_from_aruco_area(area_px2, ref_area_px2):
    """
    아루코 마커 픽셀 면적(area_px2)을 실제 고도(m)로 변환.
    ref_area_px2: ARUCO_REF_ALTITUDE_M(2.0m)에서 측정된 기준 면적
                  (line_table.csv의 'aru' 값).
    area_px2나 ref_area_px2가 유효하지 않으면 None을 반환한다.
    """
    if area_px2 is None or area_px2 <= 0:
        return None
    if ref_area_px2 is None or ref_area_px2 <= 0:
        return None
    return ARUCO_REF_ALTITUDE_M * math.sqrt(ref_area_px2 / area_px2)


# ------------------------------------------------------------------
# 추가된 부분(핵심): 마커 중심 오프셋을 "화면 비율(ratio, -1~1)"이 아니라
# 실제 물리 단위(m)로 정확하게 변환.
#
# 원리(핀홀 카메라 모델): 마커의 실제 물리적 한 변 길이(m)를 알고 있으면,
# "이번 프레임에서 마커가 몇 픽셀 크기로 찍혔는지"만 봐도
#   meters_per_pixel = 마커_실제_크기(m) / 마커_픽셀_크기(px)
# 를 그 프레임 기준으로 바로 계산할 수 있다. 이걸 dx_px/dy_px에 곱하면
# 화면 중심에서 벗어난 정도가 그대로 실제 미터 오차가 된다.
#
# altitude 추정치를 거치지 않고 "이번 프레임 마커 자체의 픽셀 크기"로
# 직접 계산하기 때문에, altitude 추정 오차가 x/y 오차 계산에 이중으로
# 누적되지 않는다 (두 계산이 서로 독립적).
# ------------------------------------------------------------------

# 실측 확인됨: 사용 중인 ArUco 마커는 0.4m x 0.4m (정사각형).
ARUCO_MARKER_SIZE_M = 0.40


def _estimate_marker_side_px(corners_single):
    """
    corners_single로부터 마커의 픽셀 한 변 길이를 추정.
    위/왼쪽 두 변의 길이를 평균낸다 (완전한 정사각형 정면 촬영이 아니어도
    약간의 원근/회전에 덜 민감하게 하기 위함).
    """
    pts = np.array(corners_single).reshape(4, 2).astype('float32')
    top_left, top_right, bottom_right, bottom_left = pts[0], pts[1], pts[2], pts[3]
    width_px = np.linalg.norm(top_right - top_left)
    height_px = np.linalg.norm(bottom_left - top_left)
    return (width_px + height_px) / 2.0


def compute_center_offset_m(frame_shape, corners_single,
                             marker_size_m=ARUCO_MARKER_SIZE_M):
    """
    마커 중심이 화면 중심에서 실제로 몇 미터(x, y) 벗어났는지 계산.
    compute_center_offset()과 달리 ratio(-1~1)가 아니라 실제 물리 단위(m)를
    반환한다. yaw/방향 관련 계산은 전혀 하지 않는다 (이 함수는 오직
    x_offset_m/y_offset_m/total_offset_m만 추가로 계산해서 반환).

    frame_shape: frame.shape (h, w[, c])
    corners_single: detect_aruco()가 반환한 corners[i]
    marker_size_m: 마커의 실제 물리적 한 변 길이(m). 기본값은 위
                   ARUCO_MARKER_SIZE_M 플레이스홀더 - 실측값으로 바꿀 것.

    반환: compute_center_offset()과 동일한 키 + 아래 3개 추가
        - marker_side_px: 이번 프레임에서 추정된 마커 픽셀 한 변 길이
        - x_offset_m: 실제 x(가로) 오차(m). 오른쪽이 양수(픽셀 좌표계 그대로).
        - y_offset_m: 실제 y(세로) 오차(m). 아래쪽이 양수(픽셀 좌표계 그대로).
        - total_offset_m: 피타고라스로 계산한 전체 직선 거리 오차(m).
    """
    offset_info = compute_center_offset(frame_shape, corners_single)

    marker_side_px = _estimate_marker_side_px(corners_single)
    if marker_side_px <= 0:
        offset_info['marker_side_px'] = 0.0
        offset_info['x_offset_m'] = None
        offset_info['y_offset_m'] = None
        offset_info['total_offset_m'] = None
        return offset_info

    meters_per_pixel = marker_size_m / marker_side_px

    x_offset_m = offset_info['dx_px'] * meters_per_pixel
    y_offset_m = offset_info['dy_px'] * meters_per_pixel
    total_offset_m = float(math.hypot(x_offset_m, y_offset_m))  # 피타고라스

    offset_info['marker_side_px'] = float(marker_side_px)
    offset_info['x_offset_m'] = float(x_offset_m)
    offset_info['y_offset_m'] = float(y_offset_m)
    offset_info['total_offset_m'] = total_offset_m

    return offset_info


# ------------------------------------------------------------------
# 아래부터는 "그리기/시각화" 전용 함수들.
# detect_aruco()는 mode1/mode3의 area 계산처럼 화면 표시 없이 빠르게
# 돌아야 하는 곳에서도 쓰이기 때문에, 그리기 로직을 여기 섞지 않고
# 필요한 곳에서만 아래 함수를 별도로 호출하도록 분리했다.
#
# 중요: 이 함수들은 yaw를 절대 계산/추정하지 않는다. 아루코 중심
# 정렬 보정 중에는 EKF2로 가짜 yaw(identity quaternion)를 보내면
# 안 되므로, 이 단계에서는 반드시:
#   1) vio_pub.update_latest(..., yaw_deg=None) 을 유지하거나
#   2) EKF2_EV_CTRL의 yaw fusion 비트를 아예 꺼둘 것
# 둘 중 하나로 "요 보정"을 명시적으로 꺼두고 사용해야 한다.
# ------------------------------------------------------------------

def compute_marker_center(corners_single):
    """
    corners_single: detect_aruco()가 반환한 corners[i] 형태
                     (1, 4, 2) 또는 (4, 2) numpy array
    반환: (cx, cy) 마커 중심 픽셀 좌표
    """
    pts = np.array(corners_single).reshape(4, 2)
    cx, cy = pts.mean(axis=0)
    return float(cx), float(cy)


def compute_center_offset(frame_shape, corners_single):
    """
    그리기 없이 오프셋 숫자만 계산하는 순수 함수.
    control(명령) 쪽(follow_waypoints.py)에서는 화면에 아무것도 안 그리고
    이 함수만 써서 x_ratio/y_ratio를 받아가면 된다.

    frame_shape: frame.shape (h, w[, c])
    corners_single: detect_aruco()가 반환한 corners[i]

    반환: offset_info dict (draw_aruco_center_offset과 동일한 키)
    """
    h, w = frame_shape[:2]
    icx, icy = w / 2.0, h / 2.0

    cx, cy = compute_marker_center(corners_single)

    dx_px = cx - icx
    dy_px = cy - icy

    x_ratio = dx_px / (w / 2.0)
    y_ratio = dy_px / (h / 2.0)
    total_ratio = float(np.hypot(x_ratio, y_ratio))

    return {
        'marker_center': (cx, cy),
        'image_center': (icx, icy),
        'dx_px': dx_px,
        'dy_px': dy_px,
        'x_ratio': x_ratio,
        'y_ratio': y_ratio,
        'total_ratio': total_ratio,
    }


def draw_aruco_center_offset(frame, corners_single,
                              marker_dot_color=(0, 0, 255),
                              line_color=(0, 255, 0),
                              center_dot_color=(255, 0, 0)):
    """
    화면 표시 전용(camera_line_scanning.py에서만 호출).
    - 마커 중앙에 점을 찍고
    - 화면 중앙에도 점을 찍고
    - 둘을 잇는 선을 그린다
    내부적으로 compute_center_offset()을 그대로 재사용해서 숫자 계산 로직은
    한 곳에만 존재하게 한다 (control 쪽과 계산 결과가 어긋날 일 없음).

    반환값: (annotated_frame, offset_info dict)
    """
    offset_info = compute_center_offset(frame.shape, corners_single)
    cx, cy = offset_info['marker_center']
    icx, icy = offset_info['image_center']

    annotated = frame.copy()
    cv2.circle(annotated, (int(icx), int(icy)), 6, center_dot_color, -1)
    cv2.circle(annotated, (int(cx), int(cy)), 6, marker_dot_color, -1)
    cv2.line(annotated, (int(icx), int(icy)), (int(cx), int(cy)), line_color, 2)
    # 수정된 부분(핵심): 여기서 텍스트를 직접 그리지 않는다. 예전엔
    # 고정 좌표(10,30)에 "offset ratio: ..."를 그렸는데, 이 좌표가
    # camera_line_scanning.py 쪽에서 통일해서 관리하는 텍스트 스택
    # (Pattern/ArUco/Grid 순서로 쌓이는 draw_info)의 첫 줄과 겹쳐서
    # 화면이 지저분해 보였다. 점/선(시각적 정렬 표시)만 여기서 그리고,
    # 텍스트는 호출하는 쪽에서 offset_info를 받아 알아서 표시하게 한다.

    return annotated, offset_info
