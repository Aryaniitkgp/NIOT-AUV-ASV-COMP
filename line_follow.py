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


class LineResult:
    """What one frame of line following produced."""

    def __init__(self, found, surge, sway, yaw, label,
                 lateral_error=0.0, heading_error=0.0, mask=None):
        self.found = found
        self.surge = surge
        self.sway = sway
        self.yaw = yaw
        self.label = label
        self.lateral_error = lateral_error
        self.heading_error = heading_error
        self.mask = mask


class LineFollowController:
    """Follows the orange line using the down-facing camera.

    Camera frame convention, taken from the down camera pose in the model
    SDF (0 0 -0.05 0 1.571 0):

        image up    -> vehicle forward (+x)
        image right -> vehicle starboard (-y, because body +y is left)

    So a positive lateral error means the line sits to starboard, and the
    vehicle has to move/turn right. Body +sway is left and body +yaw is
    counter-clockwise (left), which is why both commands are negated.

    The vehicle is fully actuated in the horizontal plane and its sway
    authority equals its surge authority, so cross-track error is taken
    out with sway while yaw only has to keep the nose aligned with the
    line. That decouples the two loops and stops the slalom you get when
    yaw alone has to fix a sideways offset.

    This class holds no ROS state, so the mission FSM can drive it with
    the same frames it feeds the other behaviours.
    """

    def __init__(self):
        # Control Parameters
        self.forward_thrust = 8.0
        self.max_surge = 14.0
        self.max_sway = 12.0
        self.max_yaw = 10.0

        # Cross-track loop (drives sway) and heading loop (drives yaw).
        self.kp_lateral = 16.0
        self.kd_lateral = 4.0
        self.kp_heading = 9.0
        self.kd_heading = 2.0

        # Small amount of lateral error fed into yaw so the vehicle also
        # points back at the line instead of crabbing along beside it.
        self.k_lateral_to_yaw = 3.0

        # State Variables
        self.lost_frames = 0
        self.search_elapsed = 0.0
        self.searching = False
        self.last_seen_lateral_error = 0.0
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0

        self.kernel = np.ones((5, 5), np.uint8)

        # Robust HSV Range for Submerged Orange
        self.lower_orange = np.array([2, 80, 50])
        self.upper_orange = np.array([22, 255, 255])

        self.lookahead_pixels = 70
        self.band_half_height = 20
        self.lost_threshold = 3
        self.search_timeout = 12.0
        self.min_contour_area = 250
        self.min_surge_scale = 0.35

    def reset(self):
        """Clear the tracking memory when the FSM hands control back."""
        self.lost_frames = 0
        self.search_elapsed = 0.0
        self.searching = False
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _band_centroid(self, line_mask, row, half_height):
        """Mean x of the line inside a horizontal strip, or None if empty.

        Sampling the mask row-wise instead of fitting one straight line
        across the whole contour keeps this stable on curves, and it can
        never blow up the way a near-horizontal line fit does when the
        slope goes to infinity.
        """
        height = line_mask.shape[0]
        top = max(0, int(row) - half_height)
        bottom = min(height, int(row) + half_height + 1)

        if bottom <= top:
            return None

        band = line_mask[top:bottom]
        xs = np.nonzero(band)[1]

        if xs.size < 10:
            return None

        return float(xs.mean())

    def update(self, frame, dt, draw=True):
        """Run one frame. Overlays are drawn onto `frame` in place."""
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        target_y = max(self.band_half_height, center_y - self.lookahead_pixels)

        # Image Processing Pipeline
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.lower_orange, self.upper_orange)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if not contours:
            return self._handle_lost_line(frame, mask, dt, "No path detected", draw)

        largest = max(contours, key=cv.contourArea)
        if cv.contourArea(largest) < self.min_contour_area:
            return self._handle_lost_line(frame, mask, dt, "Path too small", draw)

        # Isolate target contour tracking
        line_only_mask = np.zeros_like(mask)
        cv.drawContours(line_only_mask, [largest], -1, 255, -1)

        near_x = self._band_centroid(line_only_mask, center_y, self.band_half_height)
        far_x = self._band_centroid(line_only_mask, target_y, self.band_half_height)

        if near_x is None and far_x is None:
            return self._handle_lost_line(frame, mask, dt, "Insufficient pixels", draw)

        # Steer on the lookahead point when it exists, otherwise fall back
        # to what is directly underneath.
        steer_x = far_x if far_x is not None else near_x
        lateral_error = (steer_x - center_x) / float(center_x)
        lateral_error = self._clamp(lateral_error, 1.0)

        # Heading from the near/far offset. Needs both bands, so on a short
        # stub of line the vehicle just centres itself and creeps forward.
        if near_x is not None and far_x is not None:
            dx = far_x - near_x
            dy = float(center_y - target_y)
            heading_error = self._clamp(math.atan2(dx, dy) / (math.pi / 2), 1.0)
            heading_valid = True
        else:
            heading_error = 0.0
            heading_valid = False

        # Clear Tracking State
        self.lost_frames = 0
        self.searching = False
        self.search_elapsed = 0.0
        self.last_seen_lateral_error = lateral_error

        # Derivatives, using real elapsed time so the damping does not
        # change with camera frame rate.
        if 1e-3 < dt < 0.5:
            d_lateral = (lateral_error - self.prev_lateral_error) / dt
            d_heading = (heading_error - self.prev_heading_error) / dt
        else:
            d_lateral = 0.0
            d_heading = 0.0

        self.prev_lateral_error = lateral_error
        self.prev_heading_error = heading_error

        # Image right is vehicle starboard, body +sway is left and body
        # +yaw is counter-clockwise, so both commands get negated.
        sway_cmd = -(self.kp_lateral * lateral_error + self.kd_lateral * d_lateral)
        sway_cmd = self._clamp(sway_cmd, self.max_sway)

        yaw_cmd = -(
            self.kp_heading * heading_error
            + self.kd_heading * d_heading
            + self.k_lateral_to_yaw * lateral_error
        )
        yaw_cmd = self._clamp(yaw_cmd, self.max_yaw)

        # Slow down when badly off the line so there is time to correct.
        alignment = 1.0 - min(1.0, 0.5 * abs(lateral_error) + 0.3 * abs(heading_error))
        surge_cmd = self.forward_thrust * max(self.min_surge_scale, alignment)
        surge_cmd = min(surge_cmd, self.max_surge)

        # Visual Feedback System
        target_x = int(round(steer_x))
        target_y_int = int(target_y)

        state = "ON LINE" if abs(lateral_error) < 0.15 and abs(heading_error) < 0.25 else "ALIGNING"
        if not heading_valid:
            state += " (NO HDG)"

        if draw:
            target_on_line = False
            if 0 <= target_x < width and 0 <= target_y_int < height:
                target_on_line = line_only_mask[target_y_int, target_x] > 0
            color = (0, 255, 0) if target_on_line else (0, 0, 255)
            cv.circle(frame, (center_x, target_y_int), 6, color, -1)

            near_point = (int(round(near_x)), center_y) if near_x is not None else None
            draw_overlay(
                frame, center_x, center_y,
                points=[(near_point, (0, 255, 0)), ((target_x, target_y_int), (255, 0, 255))],
            )

        label = (
            f"{state} | lat {lateral_error:+.2f} hdg {heading_error:+.2f} "
            f"| sway {sway_cmd:+.1f} yaw {yaw_cmd:+.1f}"
        )

        return LineResult(True, surge_cmd, sway_cmd, yaw_cmd, label,
                          lateral_error, heading_error, mask)

    def _handle_lost_line(self, frame, mask, dt, reason, draw):
        self.lost_frames += 1

        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        if self.lost_frames < self.lost_threshold:
            label = f"{reason}: Braking..."
            if draw:
                draw_overlay(frame, center_x, center_y)
            return LineResult(False, 0.0, 0.0, 0.0, label, mask=mask)

        if not self.searching:
            self.searching = True
            self.search_elapsed = 0.0
        elif 0.0 < dt < 0.5:
            self.search_elapsed += dt

        if self.search_elapsed > self.search_timeout:
            if draw:
                draw_overlay(frame, center_x, center_y)
            return LineResult(False, 0.0, 0.0, 0.0, "SEARCH TIMEOUT | Holding", mask=mask)

        # Move back toward whichever side the line was last on. Positive
        # last error means it went off to starboard, so sway and yaw both
        # go negative. No forward motion, that only widens the gap.
        scan_direction = 1.0 if self.last_seen_lateral_error >= 0.0 else -1.0
        sway_cmd = -scan_direction * (0.4 * self.max_sway)
        yaw_cmd = -scan_direction * (0.45 * self.max_yaw)

        if draw:
            draw_overlay(frame, center_x, center_y)

        label = f"SEARCH | sway {sway_cmd:+.1f} yaw {yaw_cmd:+.1f} | {self.search_elapsed:.1f}s"
        return LineResult(False, 0.0, sway_cmd, yaw_cmd, label, mask=mask)


def draw_overlay(frame, center_x, center_y, points=()):
    """Crosshair plus optional coloured markers, shared by all behaviours."""
    height, width = frame.shape[:2]
    cv.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 1)
    cv.line(frame, (0, center_y), (width, center_y), (0, 255, 255), 1)

    for point, color in points:
        if point and 0 <= point[0] < width and 0 <= point[1] < height:
            cv.circle(frame, point, 6, color, -1)


class LineFollowerNode(Node):
    """Standalone mission-1 node, unchanged behaviour from before."""

    def __init__(self):
        super().__init__("line_follower")

        self.bridge = CvBridge()
        self.controller = LineFollowController()

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
        self.sway_pub = self.create_publisher(Float64, "/cmd_sway", 10)
        self.yaw_pub = self.create_publisher(Float64, "/cmd_yaw", 10)
        self.heave_pub = self.create_publisher(Float64, "/cmd_heave", 10)

        self.prev_time = None
        self.debug_view_enabled = True

        self.get_logger().info("Line follower node started.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        now = self.get_clock().now()
        dt = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        result = self.controller.update(frame, dt, draw=self.debug_view_enabled)

        self.surge_pub.publish(Float64(data=float(result.surge)))
        self.sway_pub.publish(Float64(data=float(result.sway)))
        self.yaw_pub.publish(Float64(data=float(result.yaw)))
        self.heave_pub.publish(Float64(data=0.0))

        self._show_debug(frame, result)

    def _show_debug(self, frame, result):
        if not self.debug_view_enabled:
            return

        cv.putText(frame, result.label, (15, 35), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        try:
            cv.imshow("Line Following - Down Camera", frame)
            cv.waitKey(1)
        except cv.error:
            self.debug_view_enabled = False


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
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
