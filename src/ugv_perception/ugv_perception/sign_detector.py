"""Directional sign detection node for UGV Navigation Challenge.

Detects directional signs (LEFT, RIGHT, FORWARD, STOP, INPLACE_ROTATION, GOAL)
from the robot camera feed using HSV color segmentation.

Topics:
  Subscribes: /r1_mini/camera/image_raw (sensor_msgs/Image)
  Publishes:  /ugv/sign/direction (std_msgs/String)
"""

import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

VALID_DIRECTIONS = frozenset({
    "LEFT", "RIGHT", "FORWARD", "STOP", "INPLACE_ROTATION", "GOAL",
})


@dataclass(frozen=True)
class _HsvRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass(frozen=True)
class _SignColor:
    direction: str
    ranges: tuple[_HsvRange, ...]


# HSV ranges for each sign color (matching arena sign textures)
SIGN_COLORS: tuple[_SignColor, ...] = (
    _SignColor("LEFT", (
        _HsvRange((40, 120, 120), (75, 255, 255)),
    )),
    _SignColor("RIGHT", (
        _HsvRange((100, 100, 100), (130, 255, 255)),
    )),
    _SignColor("FORWARD", (
        _HsvRange((80, 100, 100), (100, 255, 255)),
    )),
    _SignColor("STOP", (
        _HsvRange((0, 100, 100), (10, 255, 255)),
        _HsvRange((170, 100, 100), (180, 255, 255)),
    )),
    _SignColor("INPLACE_ROTATION", (
        _HsvRange((20, 100, 100), (35, 255, 255)),
    )),
    _SignColor("GOAL", (
        _HsvRange((10, 150, 150), (25, 255, 255)),
    )),
)


def _build_mask(hsv: np.ndarray, ranges: tuple[_HsvRange, ...]) -> np.ndarray:
    """Combine multiple HSV ranges into a single mask."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv, np.array(r.lower), np.array(r.upper))
    return mask


def _largest_contour_area(mask: np.ndarray) -> int:
    """Return the area of the largest contour in the mask, or 0."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0
    return int(max(cv2.contourArea(c) for c in contours))


class SignDetector(Node):

    def __init__(self) -> None:
        super().__init__("sign_detector")

        self._bridge = CvBridge()

        # ROS parameters for tuning
        self.declare_parameter("min_detection_area", 1500)
        self.declare_parameter("cooldown_sec", 5.0)

        self._last_detection_time: dict[str, float] = {}

        self._image_sub = self.create_subscription(
            Image, "/r1_mini/camera", self._image_callback, 10,
        )
        self._direction_pub = self.create_publisher(
            String, "/ugv/sign/direction", 10,
        )

        self.get_logger().info("Sign detector initialized (HSV color segmentation)")

    def _image_callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        direction = self._detect_sign(frame)

        if direction and direction in VALID_DIRECTIONS:
            direction_msg = String()
            direction_msg.data = direction
            self._direction_pub.publish(direction_msg)

    def _detect_sign(self, frame: np.ndarray) -> str | None:
        """Detect directional sign via HSV color segmentation."""
        min_area = self.get_parameter("min_detection_area").value
        cooldown = self.get_parameter("cooldown_sec").value
        max_area = (frame.shape[0] * frame.shape[1]) // 10

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        best_direction: str | None = None
        best_area = 0

        for sign in SIGN_COLORS:
            mask = _build_mask(hsv, sign.ranges)
            area = _largest_contour_area(mask)

            if area < min_area or area > max_area:
                continue
            if area > best_area:
                best_area = area
                best_direction = sign.direction

        if best_direction is None:
            return None

        # Cooldown: don't re-detect same sign within cooldown period
        now = time.monotonic()
        last = self._last_detection_time.get(best_direction, 0.0)
        if now - last < cooldown:
            return None

        self._last_detection_time[best_direction] = now
        self.get_logger().info(
            f"Sign detected: {best_direction} (area={best_area})"
        )
        return best_direction


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
