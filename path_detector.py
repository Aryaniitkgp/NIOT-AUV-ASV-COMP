#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge

import cv2 as cv
import numpy as np


class PathDetector(Node):

    def __init__(self):
        super().__init__("path_detector")

        self.bridge = CvBridge()

        # Use Best Effort QoS to match the publisher on /bluerov2/down_camera/image_raw
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Image,
            "/bluerov2/down_camera/image_raw",
            self.image_callback,
            self.qos_profile
        )

        self.error_pub = self.create_publisher(
            Float32,
            "/path_error",
            10
        )

        self.get_logger().info("Path Detector Started")

    def image_callback(self, msg):

        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        # Convert to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Orange HSV range (tuned for underwater simulation environment)
        lower = np.array([0, 50, 50])
        upper = np.array([25, 255, 255])

        mask = cv.inRange(hsv, lower, upper)

        # Morphological filtering
        kernel = np.ones((5, 5), np.uint8)

        mask = cv.erode(mask, kernel, iterations=1)
        mask = cv.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv.findContours(
            mask,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            cv.imshow("Camera", frame)
            cv.imshow("Mask", mask)
            cv.waitKey(1)
            return

        largest = max(contours, key=cv.contourArea)

        # Ignore tiny contours
        if cv.contourArea(largest) < 200:
            cv.imshow("Camera", frame)
            cv.imshow("Mask", mask)
            cv.waitKey(1)
            return

        M = cv.moments(largest)

        if M["m00"] == 0:
            cv.imshow("Camera", frame)
            cv.imshow("Mask", mask)
            cv.waitKey(1)
            return

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        height, width = frame.shape[:2]
        center = width // 2

        pixel_error = cx - center
        normalized_error = pixel_error / center

        # Publish
        msg_out = Float32()
        msg_out.data = float(normalized_error)

        self.error_pub.publish(msg_out)

        # Draw results
        cv.drawContours(frame, [largest], -1, (255, 0, 0), 2)

        cv.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

        cv.line(
            frame,
            (center, 0),
            (center, height),
            (0, 0, 255),
            2
        )

        cv.putText(
            frame,
            f"Error: {normalized_error:.3f}",
            (20, 40),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        cv.imshow("Camera", frame)
        cv.imshow("Mask", mask)
        cv.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PathDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()