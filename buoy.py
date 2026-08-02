#!/usr/bin/env python3
"""Mission 2 - buoy detection and image based visual servoing.

Detection is HSV thresholding plus contour analysis on the front camera.
The buoy is a sphere, so a minimum enclosing circle fits it far better
than a bounding box: it gives the centre (u, v) and the radius R_pixels
in one shot, and the ratio between contour area and circle area is a
cheap, scale-free way to throw out the orange path and the support pipe.

Control is the outer loop of a classic IBVS scheme. Pixel errors are
taken relative to the principal point,

    e_x = u - u0     (yaw / sway error)
    e_y = v - v0     (heave error)

normalised by the half image size so the gains do not depend on the
resolution, and pushed through PID controllers straight into body rates.

Range comes from the pinhole model. For a sphere of known radius,

    Z = R_real * f / R_pixels

with f = (width / 2) / tan(hfov / 2). That is only used for the "am I
close enough to touch it" decision and for logging - the servo loop
itself never needs it, which is the whole point of doing this in image
space.

Front camera frame, from the model SDF pose (0.2 0 0.05 0 0 0), i.e. no
rotation relative to the FLU body frame:

    image right -> vehicle starboard (-y, body +y is left)
    image down  -> vehicle down      (-z, body +z is up)

so a buoy right of centre needs negative yaw and negative sway, and a
buoy below centre needs negative heave. Every command is negated.
"""

import math

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

from line_follow import draw_overlay


# HSV bands per buoy colour. Red straddles the hue wrap-around at 0/180,
# so it needs two windows.
#
# The bands are wide because the translucent water volume (0.2 0.45 0.75
# at alpha 0.3) blends toward blue and drags hue with it. Sampling the
# arena materials through that blend, the red flower runs 0 -> 174 -> 145
# as the intervening water deepens, while the orange path stays pinned at
# 11-13 and only reaches 0 once its saturation has collapsed to ~35. So
# hue separates the two at every plausible tint, and the saturation floor
# catches the one case where it does not.
COLOR_RANGES = {
    "red": [
        (np.array([0, 100, 45]), np.array([7, 255, 255])),
        (np.array([158, 90, 45]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([45, 70, 35]), np.array([92, 255, 255])),
    ],
    "yellow": [
        (np.array([22, 80, 60]), np.array([44, 255, 255])),
    ],
}


class BuoyObservation:
    """One accepted detection, in image coordinates."""

    def __init__(self, u, v, radius_px, area_px, circularity, ex, ey, distance, color):
        self.u = u
        self.v = v
        self.radius_px = radius_px
        self.area_px = area_px
        self.circularity = circularity
        self.ex = ex                # normalised, +1 at the right edge
        self.ey = ey                # normalised, +1 at the bottom edge
        self.distance = distance    # metres, from the pinhole model
        self.color = color


class PID:
    """Scalar PID with clamped integral, so a long approach cannot wind up."""

    def __init__(self, kp, ki, kd, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.primed = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.primed = False

    def update(self, error, dt):
        derivative = 0.0

        if 1e-3 < dt < 0.5:
            self.integral += error * dt
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
            if self.primed:
                derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.primed = True

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class BuoyServoController:
    """Detects the buoy and servos the vehicle onto it.

    Holds no ROS state so the mission FSM can drive it directly.
    """

    def __init__(self, colors=("red", "green", "yellow"),
                 buoy_radius_m=0.1145, horizontal_fov=1.047):
        self.colors = tuple(colors)
        self.buoy_radius_m = buoy_radius_m
        self.horizontal_fov = horizontal_fov

        # Once an approach commits to a buoy the colour is latched, so a
        # rival flower drifting into frame at a similar range cannot make
        # the servo swap targets halfway in and stall between the two.
        self.locked_color = None

        # Focal length in pixels, filled in from the first frame because
        # the bridge rescales everything to a fixed size anyway.
        self.focal_px = None

        # Detection gates
        self.min_area_px = 350.0
        self.min_radius_px = 11.0
        self.min_circularity = 0.62
        self.max_aspect_ratio = 1.6
        self.kernel = np.ones((5, 5), np.uint8)

        # IBVS gains. Yaw carries the alignment, sway only trims: with a
        # single forward camera the pixel error is an angle, so yaw is the
        # axis that actually observes it. A small sway term stops the
        # vehicle from arcing around the buoy on the way in.
        #
        # The vertical axis is cascaded instead: e_y drives a commanded
        # descent/climb rate in m/s, which the depth loop turns into
        # thrust. That keeps the buoyancy trim integrator in exactly one
        # place instead of fighting a second one here.
        self.yaw_pid = PID(kp=9.0, ki=0.6, kd=1.6, i_limit=1.5)
        self.sway_pid = PID(kp=3.5, ki=0.0, kd=0.8, i_limit=1.0)
        self.heave_pid = PID(kp=0.45, ki=0.05, kd=0.06, i_limit=1.0)

        self.max_yaw = 8.0
        self.max_sway = 6.0
        self.max_heave_rate = 0.35   # m/s

        # Surge scaling: v_x = k_surge / R_pixels, so thrust falls off as
        # the inverse of the apparent radius and therefore roughly linearly
        # with the remaining distance. Clamped at both ends so it neither
        # charges in from far away nor stalls on the last few centimetres.
        self.k_surge = 150.0
        self.max_approach_surge = 5.0
        self.min_approach_surge = 0.8

        # Do not drive forward while badly pointed at the target.
        self.align_tolerance = 0.22

        # Touch geometry. The camera sits 0.2 m ahead of the body origin,
        # essentially on the front face, so closing to here and then
        # nudging open-loop is enough to make contact without the sphere
        # overflowing the frame and killing the detection.
        self.touch_distance = 0.35

        self.lost_frames = 0
        self.last_ex = 0.0

    def reset(self):
        self.yaw_pid.reset()
        self.sway_pid.reset()
        self.heave_pid.reset()
        self.lost_frames = 0

    def lock(self, color):
        self.locked_color = color

    def unlock(self):
        self.locked_color = None

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _color_mask(self, hsv, color):
        ranges = COLOR_RANGES[color]
        mask = cv.inRange(hsv, ranges[0][0], ranges[0][1])
        for lower, upper in ranges[1:]:
            mask = cv.bitwise_or(mask, cv.inRange(hsv, lower, upper))

        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)
        return mask

    def _candidates(self, mask, color, width, height):
        """Every blob in one colour mask that survives the shape gates."""
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        found = []

        u0 = width / 2.0
        v0 = height / 2.0

        for contour in contours:
            area = cv.contourArea(contour)
            if area < self.min_area_px:
                continue

            (cx, cy), radius = cv.minEnclosingCircle(contour)
            if radius < self.min_radius_px:
                continue

            # A sphere fills its enclosing circle; the path strip and the
            # support pipe do not. This is what keeps the orange line out
            # of the red mask when the water tint drags its hue down.
            circularity = area / (math.pi * radius * radius)
            if circularity < self.min_circularity:
                continue

            _, _, w, h = cv.boundingRect(contour)
            if h == 0 or w == 0:
                continue
            aspect = max(w / float(h), h / float(w))
            if aspect > self.max_aspect_ratio:
                continue

            # Pixel errors, then normalised so the gains are resolution free.
            found.append(BuoyObservation(
                u=cx, v=cy, radius_px=radius, area_px=area,
                circularity=circularity,
                ex=(cx - u0) / u0, ey=(cy - v0) / v0,
                distance=self.buoy_radius_m * self.focal_px / radius,
                color=color,
            ))

        return found

    def detect(self, frame):
        """Find every buoy in frame and return the closest one.

        Returns (BuoyObservation or None, mask, all_candidates).

        The course carries three flowers - red, green and yellow - so the
        detector runs every colour band and then picks a target. Closest
        wins, and because Z = R_real * f / R_pixels the closest buoy is by
        definition the one with the largest apparent radius. That makes
        the choice a pure image-space comparison with no dependence on the
        range model being calibrated correctly.

        A latched colour, if set, overrides the ranking so that a commited
        approach stays on its target.
        """
        height, width = frame.shape[:2]

        if self.focal_px is None:
            self.focal_px = (width / 2.0) / math.tan(self.horizontal_fov / 2.0)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        candidates = []
        mask = np.zeros((height, width), np.uint8)

        for color in self.colors:
            color_mask = self._color_mask(hsv, color)
            mask = cv.bitwise_or(mask, color_mask)
            candidates.extend(self._candidates(color_mask, color, width, height))

        if not candidates:
            return None, mask, []

        if self.locked_color is not None:
            locked = [c for c in candidates if c.color == self.locked_color]
            if locked:
                return max(locked, key=lambda c: c.radius_px), mask, candidates

        return max(candidates, key=lambda c: c.radius_px), mask, candidates

    def compute(self, obs, dt):
        """Turn one observation into (surge, sway, yaw, heave_rate).

        surge/sway/yaw are thrust commands for the mixer; heave_rate is a
        vertical velocity setpoint in m/s for the depth loop.
        """
        self.last_ex = obs.ex

        yaw_cmd = self._clamp(-self.yaw_pid.update(obs.ex, dt), self.max_yaw)
        sway_cmd = self._clamp(-self.sway_pid.update(obs.ex, dt), self.max_sway)
        heave_rate = self._clamp(-self.heave_pid.update(obs.ey, dt), self.max_heave_rate)

        # Inverse-radius surge profile, gated on how well the optical axis
        # is already lined up with the buoy.
        surge_cmd = self.k_surge / max(obs.radius_px, 1.0)
        surge_cmd = max(self.min_approach_surge, min(self.max_approach_surge, surge_cmd))

        misalignment = math.hypot(obs.ex, obs.ey)
        if misalignment > self.align_tolerance:
            scale = max(0.0, 1.0 - (misalignment - self.align_tolerance) / 0.4)
            surge_cmd *= scale

        return surge_cmd, sway_cmd, yaw_cmd, heave_rate

    def search(self):
        """Sweep back toward the side the buoy was last seen on."""
        direction = 1.0 if self.last_ex >= 0.0 else -1.0
        return 0.0, -direction * 0.3 * self.max_sway, -direction * 0.5 * self.max_yaw, 0.0

    def annotate(self, frame, obs, candidates=()):
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)

        # Rejected candidates in grey, so it is obvious from the debug view
        # which buoys were seen and which one was picked.
        for other in candidates:
            if obs is not None and other is obs:
                continue
            cv.circle(frame, (int(other.u), int(other.v)),
                      int(other.radius_px), (140, 140, 140), 1)
            cv.putText(frame, f"{other.color} {other.distance:.1f}m",
                       (int(other.u) - 30, int(other.v) - int(other.radius_px) - 6),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1)

        if obs is None:
            draw_overlay(frame, center[0], center[1])
            return

        cv.circle(frame, (int(obs.u), int(obs.v)), int(obs.radius_px), (0, 255, 0), 2)
        cv.line(frame, center, (int(obs.u), int(obs.v)), (255, 0, 255), 2)
        cv.putText(frame, f"{obs.color} {obs.distance:.2f}m",
                   (int(obs.u) - 30, int(obs.v) - int(obs.radius_px) - 8),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        draw_overlay(frame, center[0], center[1],
                     points=[((int(obs.u), int(obs.v)), (0, 0, 255))])


class BuoyServoNode(Node):
    """Standalone mission-2 node, useful for tuning the detector alone.

    It does not hold depth - run mission_control.py for the real thing.
    """

    def __init__(self):
        super().__init__("buoy_servo")

        self.bridge = CvBridge()
        self.controller = BuoyServoController()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Image, "/bluerov2/front_camera/image_raw", self.image_callback, qos
        )

        self.surge_pub = self.create_publisher(Float64, "/cmd_surge", 10)
        self.sway_pub = self.create_publisher(Float64, "/cmd_sway", 10)
        self.yaw_pub = self.create_publisher(Float64, "/cmd_yaw", 10)
        self.heave_pub = self.create_publisher(Float64, "/cmd_heave", 10)

        self.prev_time = None
        self.debug_view_enabled = True

        self.get_logger().info("Buoy servo node started (no depth hold).")

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

        obs, mask, candidates = self.controller.detect(frame)

        if obs is None:
            surge, sway, yaw = 0.0, 0.0, 0.0
            label = "NO BUOY"
        elif obs.distance <= self.controller.touch_distance:
            surge, sway, yaw = 0.0, 0.0, 0.0
            label = f"AT {obs.color.upper()} | Z {obs.distance:.2f} m R {obs.radius_px:.0f} px"
        else:
            surge, sway, yaw, heave_rate = self.controller.compute(obs, dt)
            label = (
                f"SERVO {obs.color} ({len(candidates)} seen) | "
                f"ex {obs.ex:+.2f} ey {obs.ey:+.2f} "
                f"R {obs.radius_px:.0f} px Z {obs.distance:.2f} m "
                f"surge {surge:.1f} dz {heave_rate:+.2f}"
            )

        # Heave stays zero: the commanded rate needs the depth loop that
        # only mission_control.py runs, and driving it open loop here
        # would just make the vehicle drift.
        self.surge_pub.publish(Float64(data=float(surge)))
        self.sway_pub.publish(Float64(data=float(sway)))
        self.yaw_pub.publish(Float64(data=float(yaw)))
        self.heave_pub.publish(Float64(data=0.0))

        if not self.debug_view_enabled:
            return

        self.controller.annotate(frame, obs, candidates)
        cv.putText(frame, label, (15, 35), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        try:
            cv.imshow("Buoy Detection - Front Camera", frame)
            cv.waitKey(1)
        except cv.error:
            self.debug_view_enabled = False


def main(args=None):
    rclpy.init(args=args)
    node = BuoyServoNode()
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
