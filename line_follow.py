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

        # Control Parameters
        self.forward_thrust = 14.0
        self.max_yaw = 15.0
        self.max_surge = 14.0

        # Proportional steering gain for geometric line following
        self.kp = 12.0

        # Weighted Error Metrics
        self.w_lateral = 0.6
        self.w_heading = 0.4

        # State Variables
        self.lost_frames = 0
        self.search_start_time = None
        self.last_seen_lateral_error = 0.0
        self.last_seen_heading_error = 0.0
        self.last_seen_error = 0.0

        self.kernel = np.ones((5, 5), np.uint8)

        # Robust HSV Range for Submerged Orange
        self.lower_orange = np.array([2, 80, 50])
        self.upper_orange = np.array([22, 255, 255])
        
        self.lookahead_pixels = 70
        self.lost_threshold = 3
        self.min_line_pixels_for_heading = 400
        self.min_surge_scale = 0.35
        self.debug_view_enabled = True

        self.get_logger().info("Line follower optimized node started.")

    def _publish_command(self, surge, yaw):
        surge_msg = Float64(data=float(surge))
        yaw_msg = Float64(data=float(yaw))
        self.surge_pub.publish(surge_msg)
        self.yaw_pub.publish(yaw_msg)

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        target_y = max(0, center_y - self.lookahead_pixels)

        # Image Processing Pipeline
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.lower_orange, self.upper_orange)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._handle_lost_line(frame, mask, center_x, center_y, "No path detected")
            return

        largest = max(contours, key=cv.contourArea)
        contour_area = cv.contourArea(largest)
        if contour_area < 250:
            self._handle_lost_line(frame, mask, center_x, center_y, "Path too small")
            return

        # Isolate target contour tracking
        line_only_mask = np.zeros_like(mask)
        cv.drawContours(line_only_mask, [largest], -1, 255, -1)
        ys_all, xs_all = np.nonzero(line_only_mask)

        if len(xs_all) < 10:
            self._handle_lost_line(frame, mask, center_x, center_y, "Insufficient pixels")
            return

        # Fit a single line through the contour and use a lookahead point for steering.
        pts = np.column_stack((xs_all, ys_all)).astype(np.float32)
        vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01).flatten()

        if vy > 0:
            vx, vy = -vx, -vy

        if abs(vy) < 1e-4:
            vy = -1e-4

        line_x_at_center = x0 + (vx / vy) * (center_y - y0)
        line_x_at_target = x0 + (vx / vy) * (target_y - y0)

        lateral_error_center = (line_x_at_center - center_x) / float(center_x)
        lateral_error_target = (line_x_at_target - center_x) / float(center_x)

        heading_angle = math.atan2(vx, -vy)
        heading_error = max(-1.0, min(1.0, heading_angle / (math.pi / 2)))

        x, y, w, h = cv.boundingRect(largest)
        long_side = float(max(w, h))
        short_side = float(max(1, min(w, h)))
        aspect_ratio = long_side / short_side
        low_confidence = pts.shape[0] < self.min_line_pixels_for_heading or aspect_ratio < 1.4

        # Clear Tracking State
        self.lost_frames = 0
        self.search_start_time = None

        # Use the lookahead point as the main cross-track error and blend in heading.
        lateral_error = (0.7 * lateral_error_target) + (0.3 * lateral_error_center)
        error = self.w_lateral * lateral_error + self.w_heading * heading_error
        error = max(-1.0, min(1.0, error))
        self.last_seen_error = error
        self.last_seen_lateral_error = lateral_error
        self.last_seen_heading_error = heading_error

        # Calculate closed-loop steering. This is a geometry problem, so a
        # simple proportional controller is more stable than a full PID here.
        yaw_cmd = self._clamp(self.kp * error, self.max_yaw)

        # Scale forward surge based on how well the line is centered and aligned.
        alignment = 1.0 - min(1.0, 0.9 * abs(lateral_error) + 0.7 * abs(heading_error))
        if low_confidence:
            alignment *= 0.6
        
        surge_cmd = self.forward_thrust * max(self.min_surge_scale, alignment)
        surge_cmd = min(surge_cmd, self.max_surge)

        self._publish_command(surge_cmd, yaw_cmd)

        # Visual Feedback System
        target_on_line = False
        target_x = int(round(line_x_at_target))
        target_y_int = int(target_y)
        if 0 <= target_x < width and 0 <= target_y_int < height:
            target_on_line = mask[target_y_int, target_x] > 0

        self._draw_target(frame, center_x, target_y, target_on_line)
        state = "ON LINE" if abs(lateral_error) < 0.15 and abs(heading_error) < 0.25 else "ALIGNING"
        if low_confidence:
            state += " (LOW-CONF)"
            
        self._show_debug(
            frame, mask, center_x, center_y,
            near_point=(int(round(line_x_at_center)), center_y),
            far_point=(target_x, target_y_int),
            label=f"{state} | Lat Error: {lateral_error:.2f} | Hdg Error: {heading_error:.2f}",
        )

    def _handle_lost_line(self, frame, mask, center_x, center_y, reason):
        self.lost_frames += 1
        now = self.get_clock().now()

        now = self.get_clock().now()

        if self.lost_frames < self.lost_threshold:
            self._publish_command(0.0, 0.0)
            self._show_debug(frame, mask, center_x, center_y, f"{reason}: Braking...")
            return

        if self.search_start_time is None:
            self.search_start_time = now

        # Hold a gentle turn toward the last seen side and rotate in place until
        # the line re-enters the camera. Forward motion here tends to push the
        # vehicle farther away from the path.
        scan_direction = 1.0 if self.last_seen_lateral_error >= 0.0 else -1.0
        yaw_cmd = scan_direction * (0.45 * self.max_yaw)
        yaw_cmd = self._clamp(yaw_cmd, self.max_yaw)
        
        surge_cmd = 0.0

        self._publish_command(surge_cmd, yaw_cmd)
        self._show_debug(frame, mask, center_x, center_y, f"SEARCH MODE | Yaw Action: {yaw_cmd:.2f}")

    def _draw_target(self, frame, center_x, center_y, on_line):
        color = (0, 255, 0) if on_line else (0, 0, 255)
        cv.circle(frame, (center_x, center_y), 6, color, -1)

    def _show_debug(self, frame, mask, center_x, center_y, label, near_point=None, far_point=None):
        if not self.debug_view_enabled:
            return

        height, _ = frame.shape[:2]
        cv.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 1)
        cv.line(frame, (0, center_y), (frame.shape[1], center_y), (0, 255, 255), 1)

        if near_point and (0 <= near_point[0] < frame.shape[1]):
            cv.circle(frame, near_point, 6, (0, 255, 0), -1) 
        if far_point and (0 <= far_point[0] < frame.shape[1]):
            cv.circle(frame, far_point, 6, (255, 0, 255), -1) 

        cv.putText(frame, label, (15, 35), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        try:
            cv.imshow("AUV Camera - Tracking View", frame)
            cv.imshow("Binary Threshold Mask", mask)
            cv.waitKey(1)
            self.debug_view_enabled = True
        except cv.error:
            self.debug_view_enabled = False


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
