import cv2
import numpy as np

print("### LOADED intersection_centering_v2 (quick_update 전용, 경량화) ###")
print(__file__)


class IntersectionCenteringV2:
    """
    격자점(교차점) 중심을 화면 중심 기준 오프셋(m)으로 계산하는 클래스.

    수정된 부분(핵심): 예전엔 스켈레톤화 -> 노드탐색 -> 클러스터링 ->
    형태분류(cross/T/corner)까지 전부 이 클래스 혼자 처음부터 다시
    했었다(update()). 근데 이제 pattern_detector.py(5x5 템플릿 매칭)가
    "여기 어떤 모양이 있는지 + 대략 어디 있는지"를 이미 훨씬 빠르게
    알려주기 때문에, 그 정보를 받아서 정확한 중심만 다듬는 경량 버전
    (quick_update)만 남기고 나머지는 전부 지웠다.

    지운 것들(전부 quick_update 도입 후 아무 데서도 안 불리던 죽은
    코드였음): skeletonize, remove_spurs, find_nodes, trace_branch,
    classify_node, cluster_nodes, cluster_centers, draw_nodes,
    draw_debug, update(). 필요해지면 git 이력에서 복구 가능.
    """

    def __init__(self):
        # 실제 선/격자 폭(m). quick_update()에도 기본값으로 그대로 넘어감.
        self.LINE_WIDTH_M = 0.10

        # 마지막으로 계산된 m/px 스케일. quick_update() 호출 후에만 값이 있음.
        self.meters_per_pixel = None

    # --------------------------------------------------------
    # 무게중심으로 중심 좌표 다듬기
    # --------------------------------------------------------

    def refine_center(self, mask, center):
        """
        주어진 seed 좌표 주변 작은 창(window) 안에서 흰 픽셀의 무게중심을
        구해 중심을 다듬는다. moments 기반 벡터 연산이라 빠르다.
        """
        cx, cy = center
        radius = 35

        h, w = mask.shape

        x1 = max(0, cx - radius)
        x2 = min(w, cx + radius)
        y1 = max(0, cy - radius)
        y2 = min(h, cy + radius)

        roi = mask[y1:y2, x1:x2]

        M = cv2.moments(roi)

        if M["m00"] == 0:
            return cx, cy

        mx = int(M["m10"] / M["m00"])
        my = int(M["m01"] / M["m00"])

        return x1 + mx, y1 + my

    # --------------------------------------------------------
    # 선 폭으로 m/px 스케일 계산
    # --------------------------------------------------------

    def calculate_scale(self, mask, center, line_width_m=0.10):
        """
        실제 선 폭(m) ÷ 화면에서 측정된 선 폭(px) = m/px.
        중심 주변 여러 행에서 좌우 선 폭을 재고 중앙값을 쓴다(노이즈에 강함).

        측정 불가하면 None 반환.
        """
        cx, cy = center
        h, w = mask.shape

        widths = []

        for yy in range(max(0, cy - 20), min(h, cy + 20)):

            window = 80

            row = mask[yy]

            x1 = max(0, cx - window)
            x2 = min(w, cx + window)

            left = cx
            while left > x1 and row[left] > 0:
                left -= 1

            right = cx
            while right < x2 - 1 and row[right] > 0:
                right += 1

            width = right - left - 1

            if width > 3:
                widths.append(width)

        if len(widths) == 0:
            return None

        width_px = np.median(widths)

        return line_width_m / width_px

    # --------------------------------------------------------
    # 화면 중심 대비 오프셋(m) 계산
    # --------------------------------------------------------

    def calculate_offset(self, center, frame_shape, meters_per_pixel):
        """
        dx > 0 -> 격자점이 화면 오른쪽에 있음
        dy > 0 -> 격자점이 화면 아래쪽에 있음
        """
        cx, cy = center

        frame_h, frame_w = frame_shape[:2]

        image_cx = frame_w * 0.5
        image_cy = frame_h * 0.5

        dx_px = cx - image_cx
        dy_px = cy - image_cy

        dx_m = dx_px * meters_per_pixel
        dy_m = dy_px * meters_per_pixel

        return dx_m, dy_m

    # --------------------------------------------------------
    # 메인 함수 (경량 버전)
    # --------------------------------------------------------

    def quick_update(self, mask, frame_shape, pattern_name,
                      cell_dy=0, cell_dx=0, grid_size=5,
                      line_width_m=0.10):
        """
        pattern_detector.detect_pattern_with_shift()가 이미 "여기 격자점
        모양이 있고, 5x5 격자 기준 중앙 셀에서 (cell_dy, cell_dx)만큼
        이동해있다"고 알려준 상태에서 쓰는 함수.

        1) cell_dy/cell_dx로 "대략 여기쯤 있겠다"는 seed 픽셀 좌표를 계산
           (화면 중앙이라고 무조건 가정하지 않음 - 패턴이 화면 구석에
           걸려있어도 그 방향으로 seed가 이동함)
        2) refine_center(무게중심)로 그 seed 주변에서 정확한 중심을 찾고
        3) calculate_scale(선폭->m/px)로 오프셋을 미터 단위로 변환

        전부 파이썬 픽셀 루프 없이 벡터 연산(moments, percentile)이라 빠름.

        pattern_name이 'unknown', 'none', 'line_V', 'line_H'(교차점이
        아니라 일반 라인 구간)면 애초에 미세보정할 "점"이 없으므로
        바로 found=False를 반환한다.

        Возвращает:
            found, center_x, center_y, dx_m, dy_m, kind
        """

        if pattern_name in (None, 'unknown', 'none', 'line_V', 'line_H'):
            return False, None, None, 0.0, 0.0, (pattern_name or 'unknown')

        h, w = mask.shape
        cell_h = h / grid_size
        cell_w = w / grid_size
        mid = grid_size // 2

        seed_x = int(round((mid + cell_dx + 0.5) * cell_w))
        seed_y = int(round((mid + cell_dy + 0.5) * cell_h))
        seed_x = max(0, min(w - 1, seed_x))
        seed_y = max(0, min(h - 1, seed_y))

        center = self.refine_center(mask, (seed_x, seed_y))

        meters_per_pixel = self.calculate_scale(mask, center, line_width_m)
        if meters_per_pixel is None:
            return False, None, None, 0.0, 0.0, pattern_name

        self.meters_per_pixel = meters_per_pixel

        dx_m, dy_m = self.calculate_offset(center, frame_shape, meters_per_pixel)

        return True, center[0], center[1], dx_m, dy_m, pattern_name


GridCentering = IntersectionCenteringV2
