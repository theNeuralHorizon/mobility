"""ArUco marker detection node for UGV Navigation Challenge.

Subscribes to the robot camera feed, detects ArUco markers, and publishes
detected marker IDs along with their estimated poses.

Topics:
  Subscribes: /r1_mini/camera/image_raw (sensor_msgs/Image)
  Publishes:  /ugv/aruco/detections (std_msgs/Int32MultiArray)
              /ugv/aruco/markers (geometry_msgs/PoseArray)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge


class ArucoDetector(Node):

    # ArUco dictionary to use (4x4, 50 markers)
    ARUCO_DICT = cv2.aruco.DICT_4X4_50

    def __init__(self):
        super().__init__("aruco_detector")

        self._bridge = CvBridge()
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)

        # Support both old and new OpenCV ArUco API
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(
                self._aruco_dict, params)
            self._use_new_api = True
        else:
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._detector = None
            self._use_new_api = False

        # Track which markers have been visited
        self._visited_ids: set[int] = set()

        # Subscribers
        self._image_sub = self.create_subscription(
            Image,
            "/r1_mini/camera",
            self._image_callback,
            10,
        )

        # Publishers
        self._detections_pub = self.create_publisher(
            Int32MultiArray, "/ugv/aruco/detections", 10
        )
        self._markers_pub = self.create_publisher(
            PoseArray, "/ugv/aruco/markers", 10
        )

        self.get_logger().info("ArUco detector initialized (dict=4x4_50)")

    def _image_callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._use_new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is None:
            return

        detected_ids = ids.flatten().tolist()

        # Filter out small markers (likely false positives)
        valid_ids = []
        for i, mid in enumerate(detected_ids):
            corner_set = corners[i][0]
            # Compute marker size as max dimension of bounding box
            widths = corner_set[:, 0].max() - corner_set[:, 0].min()
            heights = corner_set[:, 1].max() - corner_set[:, 1].min()
            marker_size = max(widths, heights)
            if marker_size < 20:
                continue  # Too small — likely false positive
            valid_ids.append(mid)

        if not valid_ids:
            return

        detected_ids = valid_ids

        # Log newly visited markers
        for mid in detected_ids:
            if mid not in self._visited_ids:
                self._visited_ids.add(mid)
                self.get_logger().info(
                    f"NEW ArUco marker detected: ID={mid} "
                    f"(visited: {len(self._visited_ids)}/4)"
                )

        # Publish detected IDs
        id_msg = Int32MultiArray()
        id_msg.data = detected_ids
        self._detections_pub.publish(id_msg)

        # Publish marker poses (center of each marker in image frame)
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "CAM"

        for i, corner_set in enumerate(corners):
            center = corner_set[0].mean(axis=0)
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = 0.0
            pose_array.poses.append(pose)

        self._markers_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
