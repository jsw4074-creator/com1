import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge

TOPIC_NAME = '/camera/imx219'
ZOOM_FACTOR = 1
CAP_WIDTH =  640
CAP_HEIGHT = 480  

class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.zoom = ZOOM_FACTOR
        self.latest_frame = None
        self.bridge = CvBridge()

        # 수정된 부분(핵심): depth=10 -> depth=1.
        # 처리 루프(camera_line_scanning.py)가 카메라 프레임 속도보다
        # 느려지면, depth가 크면 클수록 "아직 처리 안 된 과거 프레임"이
        # 큐에 쌓인다. spin_once()는 그 큐에서 한 번에 하나씩만 꺼내기
        # 때문에, 밀린 순서대로(오래된 것부터) 소화하게 되고 그만큼
        # 화면이 실제 상황보다 계속 뒤처지게 된다(드론은 이미 움직였는데
        # 화면은 과거 프레임).
        # depth=1이면 새 이미지가 오는 순간 큐에 있던 이전 이미지는
        # 버려지고 최신 것만 남기 때문에, spin_once가 뭘 꺼내든 그게
        # 항상 "가장 최근에 도착한 프레임"이 된다.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Image, TOPIC_NAME, self._callback, qos_profile)
        self.get_logger().info(f'Camera subscribing: {TOPIC_NAME}')

    def _callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_frame = self._apply_zoom(frame)

    def _apply_zoom(self, img):
        if self.zoom <= 1.0:
            return img
        h, w = img.shape[:2]
        new_w = int(w / self.zoom)
        new_h = int(h / self.zoom)
        x1 = (w - new_w) // 2
        y1 = (h - new_h) // 2
        cropped = img[y1:y1 + new_h, x1:x1 + new_w]
        return cv2.resize(cropped, (w, h))

    def read(self):
        return self.latest_frame

if __name__ == '__main__':
    rclpy.init()
    cam = Camera()
    try:
        while rclpy.ok():
            rclpy.spin_once(cam, timeout_sec=0.01)
            frame = cam.read()
            if frame is None:
                continue
            cv2.imshow("camera_test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
