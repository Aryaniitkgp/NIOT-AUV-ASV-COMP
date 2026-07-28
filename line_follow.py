#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import math


class LineFollower(Node):

	def __init__(self):
		super().__init__("line_follower")

		self.bridge = CvBridge()

		self.qos_profile = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=10,
		)

		self.subscription = self.create_subscription(
			Image,
			"/bluerov2/down_camera/image_raw",
			self.image_callback,
			self.qos_profile,
		)

		self.surge_pub = self.create_publisher(Float64, "/cmd_surge", 10)
		self.yaw_pub = self.create_publisher(Float64, "/cmd_yaw", 10)

		self.forward_thrust = 18.0
		self.max_yaw = 25.0
		self.max_surge = 18.0

		self.kp = 24.0
		self.ki = 0.0
		self.kd = 7.0

		self.integral = 0.0
		self.prev_error = 0.0
		self.prev_time = None
		self.lost_frames = 0
		self.search_start_time = None
		self.last_seen_error = 0.0

		self.kernel = np.ones((5, 5), np.uint8)

		self.lower_orange = np.array([0, 50, 50])
		self.upper_orange = np.array([30, 255, 255])
		self.target_x = None
		self.target_y = None
		self.lost_threshold = 3

		self.get_logger().info("Line follower started")

	def _publish_command(self, surge, yaw):
		surge_msg = Float64()
		surge_msg.data = float(surge)

		yaw_msg = Float64()
		yaw_msg.data = float(yaw)

		self.surge_pub.publish(surge_msg)
		self.yaw_pub.publish(yaw_msg)

		self.get_logger().info(f"Publishing surge={surge:.2f} yaw={yaw:.2f}")

	def _clamp(self, value, limit):
		return max(-limit, min(limit, value))

	def image_callback(self, msg):
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		except Exception as exc:
			self.get_logger().error(f"Image conversion failed: {exc}")
			return

		hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
		mask = cv.inRange(hsv, self.lower_orange, self.upper_orange)

		mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
		mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)

		contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

		height, width = frame.shape[:2]
		center_x = width // 2
		center_y = height // 2
		self.target_x = center_x
		self.target_y = center_y
		target_on_line = mask[center_y, center_x] > 0

		self._draw_target(frame, center_x, center_y, target_on_line)

		if not contours:
			self._handle_lost_line(frame, mask, center_x, center_y, "No path detected")
			return

		largest = max(contours, key=cv.contourArea)
		area = cv.contourArea(largest)

		if area < 250:
			self._handle_lost_line(frame, mask, center_x, center_y, "Path too small")
			return

		moments = cv.moments(largest)
		if moments["m00"] == 0:
			self._publish_command(0.0, 0.0)
			self._show_debug(frame, mask, center_x, center_y, None, "Invalid contour")
			return

		contour_points = largest.reshape(-1, 2)
		distances = np.sum((contour_points - np.array([center_x, center_y])) ** 2, axis=1)
		nearest_index = int(np.argmin(distances))
		cx = int(contour_points[nearest_index][0])
		cy = int(contour_points[nearest_index][1])
		self.lost_frames = 0
		self.search_start_time = None

		# Use the target point at the center of the screen.
		# If the point is not on the orange line, steer until the line covers it.
		error = (center_x - cx) / float(center_x)
		self.last_seen_error = error

		now = self.get_clock().now()
		if self.prev_time is None:
			dt = 0.0
		else:
			dt = (now - self.prev_time).nanoseconds / 1e9

		if dt > 0.0:
			self.integral += error * dt
			derivative = (error - self.prev_error) / dt
		else:
			derivative = 0.0

		yaw_cmd = self.kp * error + self.ki * self.integral + self.kd * derivative
		yaw_cmd = self._clamp(yaw_cmd, self.max_yaw)

		if target_on_line:
			forward_scale = 1.0 - min(abs(error), 1.0)
			surge_cmd = self.forward_thrust * max(0.35, forward_scale)
		else:
			surge_cmd = 0.0

		self.prev_error = error
		self.prev_time = now

		self._publish_command(surge_cmd, yaw_cmd)

		state = "ON LINE" if target_on_line else "ALIGNING"
		self._show_debug(frame, mask, center_x, center_y, (cx, cy), f"{state} err={error:.3f} yaw={yaw_cmd:.2f}")

	def _handle_lost_line(self, frame, mask, center_x, center_y, reason):
		self.lost_frames += 1
		now = self.get_clock().now()

		if self.search_start_time is None:
			self.search_start_time = now

		self.integral = 0.0
		self.prev_error = 0.0
		self.prev_time = now

		if self.lost_frames < self.lost_threshold:
			self._publish_command(0.0, 0.0)
			self._show_debug(frame, mask, center_x, center_y, None, f"{reason} waiting")
			return

		elapsed = (now - self.search_start_time).nanoseconds / 1e9
		scan_direction = 1.0 if self.last_seen_error >= 0.0 else -1.0
		yaw_cmd = scan_direction * (0.6 * self.max_yaw) * math.sin(elapsed * 1.2)
		yaw_cmd = self._clamp(yaw_cmd, self.max_yaw)

		surge_cmd = 0.25 * self.forward_thrust
		self._publish_command(surge_cmd, yaw_cmd)
		self._show_debug(frame, mask, center_x, center_y, None, f"SEARCH yaw={yaw_cmd:.2f}")

	def _draw_target(self, frame, center_x, center_y, on_line):
		color = (0, 255, 0) if on_line else (0, 0, 255)
		cv.circle(frame, (center_x, center_y), 6, color, -1)

	def _show_debug(self, frame, mask, center_x, center_y, centroid, label):
		height, _ = frame.shape[:2]

		cv.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)
		cv.line(frame, (0, center_y), (frame.shape[1], center_y), (0, 255, 255), 1)

		if centroid is not None:
			cx, cy = centroid
			cv.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

		cv.putText(
			frame,
			label,
			(20, 40),
			cv.FONT_HERSHEY_SIMPLEX,
			0.8,
			(255, 255, 255),
			2,
		)

		cv.imshow("AUV Camera", frame)
		cv.imshow("Binary Mask", mask)
		cv.waitKey(1)


def main(args=None):
	rclpy.init(args=args)

	node = LineFollower()

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
