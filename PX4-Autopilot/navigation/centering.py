from camera_line_scanning import process_frame
import rclpy


class Centering:
    """
    라인 중심 대비 좌우(또는 상하) 편차(dy, 단위: m)를 추정하는 클래스.

    고정 PIXEL_TO_METER 상수를 쓰지 않고, 매 프레임 측정되는 라인 폭/높이
    (px)가 실제 라인 폭(LINE_WIDTH_M)에 해당한다는 점을 이용해 그때그때
    픽셀→미터 환산 비율을 자동으로 계산한다. (고도가 바뀌어도 정확)

    수직(ver)/수평(hor) 모드 모두 지원: process_frame()이 반환하는 data[7:11]
    (mid 밴드 경계, offset_px, mid 스케일)이 어느 모드든 동일한 의미를
    갖도록 통일되어 있어서, 이 클래스는 모드를 직접 신경 쓸 필요가 없다.
    """

    def __init__(self, cam, line_width_m=0.10, alpha=0.2):
        self.cam = cam
        self.LINE_WIDTH_M = line_width_m  # 실제 라인 폭 (기본 10cm)
        self.ALPHA = alpha
        self.smooth_dy = None
        self.last_vision_dy = 0.0
        self.is_intersection = False

    def update(self):
        rclpy.spin_once(self.cam, timeout_sec=0.01)
        frame = self.cam.read()
        if frame is None:
            return

        _, data, pattern_name, *_rest = process_frame(frame)

        # 교차로 감지 -> 편차 리셋
        if pattern_name == 'skip':
            self.is_intersection = True
            self.smooth_dy = 0.0
            self.last_vision_dy = 0.0
            return
        self.is_intersection = False

        mid_lo, mid_hi, offset_px = data[7], data[8], data[9]
        # data[10]: 모드(수직/수평)에 상관없이 항상 '이번 프레임에 측정된
        # 라인 폭/높이(px)'를 담고 있음 (camera_line_scanning.process_frame 참고)
        mid_length_px = data[10]

        # 라인 중심 미검출 또는 폭/높이 측정 실패 -> 직전 값 유지
        if mid_lo == -1 or mid_hi == -1 or mid_length_px <= 0:
            return

        # 실시간 자동 캘리브레이션: mid_length_px == 실제 LINE_WIDTH_M
        raw_dy = offset_px * self.LINE_WIDTH_M / (2 * mid_length_px)

        if self.smooth_dy is None:
            self.smooth_dy = raw_dy
        else:
            self.smooth_dy = self.ALPHA * raw_dy + (1 - self.ALPHA) * self.smooth_dy

        self.last_vision_dy = self.smooth_dy
