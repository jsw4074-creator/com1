"""비전 위치를 PX4 External Vision Odometry로 일정 주기에 발행한다."""

from config import (
    PX4_PUBLISH_QOS,
    VIO_ORIENTATION_VARIANCE,
    VIO_ORIENTATION_VARIANCE_NONE,
    VIO_POSITION_VARIANCE,
    VIO_POSITION_VARIANCE_NONE,
    VIO_QUALITY,
    VIO_STALE_TIMEOUT_SEC,
    VIO_TOPIC,
)
from px4_msgs.msg import VehicleOdometry


class VisionOdometryPublisher:
    def __init__(self, node):
        self.node = node

        self.publisher = node.create_publisher(
            VehicleOdometry,
            VIO_TOPIC,
            PX4_PUBLISH_QOS,
        )

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.last_valid_time_sec = None
        self.takeoff_started = False

        self.position_variance = [
            float(VIO_POSITION_VARIANCE[0]),
            float(VIO_POSITION_VARIANCE[1]),
            float(VIO_POSITION_VARIANCE[2]),
        ]

    def set_takeoff_started(self, started=True):
        self.takeoff_started = bool(started)

    def set_position_variance(
        self,
        variance_x,
        variance_y,
        variance_z,
    ):
        self.position_variance = [
            float(variance_x),
            float(variance_y),
            float(variance_z),
        ]

    def reset_position_variance(self):
        self.position_variance = [
            float(VIO_POSITION_VARIANCE[0]),
            float(VIO_POSITION_VARIANCE[1]),
            float(VIO_POSITION_VARIANCE[2]),
        ]

    def update(
        self,
        x=None,
        y=None,
        altitude=None,
    ):
        valid = False

        if x is not None:
            self.x = float(x)
            valid = True

        if y is not None:
            self.y = float(y)
            valid = True

        if altitude is not None and float(altitude) > 0.0:
            self.z = -float(altitude)
            valid = True

        if valid:
            self.last_valid_time_sec = (
                self.node.get_clock().now().nanoseconds
                / 1e9
            )

    def publish(self):
        msg = VehicleOdometry()

        now = self.node.get_clock().now()
        now_us = int(now.nanoseconds / 1000)
        now_sec = now.nanoseconds / 1e9

        msg.timestamp = now_us
        msg.timestamp_sample = now_us

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        msg.position = [
            self.x if self.takeoff_started else 0.0,
            self.y if self.takeoff_started else 0.0,
            self.z,
        ]

        msg.q = [
            1.0,
            0.0,
            0.0,
            0.0,
        ]

        msg.velocity_frame = (
            VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        )

        msg.velocity = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]

        msg.angular_velocity = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]

        msg.velocity_variance = [
            float('nan'),
            float('nan'),
            float('nan'),
        ]

        stale = (
            self.last_valid_time_sec is None
            or now_sec - self.last_valid_time_sec
            > VIO_STALE_TIMEOUT_SEC
        )

        if stale:
            position_variance = (
                VIO_POSITION_VARIANCE_NONE
            )
            orientation_variance = (
                VIO_ORIENTATION_VARIANCE_NONE
            )
        else:
            position_variance = (
                self.position_variance
            )
            orientation_variance = (
                VIO_ORIENTATION_VARIANCE
            )

        msg.position_variance = [
            float(position_variance[0]),
            float(position_variance[1]),
            float(position_variance[2]),
        ]

        msg.orientation_variance = [
            float(orientation_variance[0]),
            float(orientation_variance[1]),
            float(orientation_variance[2]),
        ]

        msg.reset_counter = 0
        msg.quality = int(VIO_QUALITY)

        self.publisher.publish(msg)
