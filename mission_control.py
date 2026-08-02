#!/usr/bin/env python3
"""Mission sequencer for the SAVe course.

Finite state machine over the behaviours in line_follow.py and buoy.py:

    INIT -> DIVE -> LINE_FOLLOW -> BUOY_APPROACH -> BUOY_TOUCH
                         ^   |          |   ^            |
                         |   |          v   |            v
                         |   +----> BUOY_SEARCH     BUOY_BACKOFF
                         +---------------------------------+

Only one behaviour ever owns surge/sway/yaw. Depth is different: the
vehicle is positively buoyant, so the depth loop runs underneath every
state, and behaviours steer it by moving the depth setpoint rather than
by commanding thrust. That is the cascade that lets the buoy servo's
vertical pixel error coexist with buoyancy trim.

The course carries three flowers. The detector runs all three colour
bands every frame and targets the closest one - largest apparent radius
- then latches that colour so a rival buoy cannot steal the approach.

Everything is driven off a fixed-rate timer working on the most recent
frame from each camera, so the control period is constant even though
the two camera streams are independent.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge
import cv2 as cv

from line_follow import LineFollowController
from buoy import BuoyServoController
from depth_control import DepthController


class State:
    INIT = "INIT"
    DIVE = "DIVE"
    LINE_FOLLOW = "LINE_FOLLOW"
    BUOY_APPROACH = "BUOY_APPROACH"
    BUOY_SEARCH = "BUOY_SEARCH"
    BUOY_TOUCH = "BUOY_TOUCH"
    BUOY_BACKOFF = "BUOY_BACKOFF"


class MissionControl(Node):

    def __init__(self):
        super().__init__("mission_control")

        self.bridge = CvBridge()

        self.line = LineFollowController()
        self.buoy = BuoyServoController(colors=("red", "green", "yellow"))
        self.depth = DepthController()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/bluerov2/down_camera/image_raw", self._down_cb, qos)
        self.create_subscription(
            Image, "/bluerov2/front_camera/image_raw", self._front_cb, qos)
        self.create_subscription(
            Odometry, "/bluerov2/odom", self._odom_cb, qos)

        self.surge_pub = self.create_publisher(Float64, "/cmd_surge", 10)
        self.sway_pub = self.create_publisher(Float64, "/cmd_sway", 10)
        self.yaw_pub = self.create_publisher(Float64, "/cmd_yaw", 10)
        self.heave_pub = self.create_publisher(Float64, "/cmd_heave", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)

        # Mission parameters.
        #
        # Everything vertical is expressed as depth below the free
        # surface, because that is how the mission is specified and how
        # the depth sensor reads. World z is up-positive with the floor at
        # z = 0 and the surface at z = 2.5, so z = surface_z - depth.
        self.surface_z = 2.5           # water box spans z = 0 .. 2.5
        self.floor_z = 0.0

        # Cruise depth, measured down from the free surface.
        #
        # Careful here: the vehicle SPAWNS at z = 1.2, which is already
        # 1.3 m deep. Commanding 1.0 m of depth therefore asks it to climb
        # 0.3 m, and it obediently rises - which looks exactly like "it is
        # not diving". The dive has to target something deeper than 1.3 m
        # to be a dive at all.
        #
        # 1.5 m is the natural choice: it is 1.0 m up off the pool floor
        # and puts the front camera dead level with the flower centres at
        # z = 1.0, so the buoy sits on the optical axis instead of down
        # near the bottom edge of the frame.
        self.cruise_depth = 1.5
        self.min_depth = 0.30          # stay under the surface
        self.max_depth = 2.05          # keep clear of the floor and the pipes

        self.dive_tolerance = 0.12
        self.dive_settle_time = 1.0
        self.dive_timeout = 25.0

        # Buoy engagement. Arming is delayed so the vehicle actually flies
        # some of mission 1 first, and the radius gate means it only
        # commits once the buoy is close enough for the range estimate to
        # be worth anything (R = 35 px is about 1.8 m out).
        self.buoy_arm_delay = 3.0
        self.buoy_trigger_radius_px = 35.0
        self.buoy_trigger_frames = 5
        self.buoy_lost_frames_max = 6
        self.buoy_approach_timeout = 60.0
        self.buoy_search_timeout = 8.0
        self.max_buoy_attempts = 2

        self.touch_surge = 2.5
        self.touch_duration = 1.2
        self.backoff_surge = -2.5
        self.backoff_duration = 2.0

        self.control_period = 0.05     # 20 Hz

        # Depth limits converted once into the world z the controller uses.
        self.cruise_z = self.surface_z - self.cruise_depth
        self.min_z = self.surface_z - self.max_depth
        self.max_z = self.surface_z - self.min_depth

        # Runtime state
        self.state = State.INIT
        self.time_in_state = 0.0
        self.mission_time = 0.0
        self.prev_time = None

        self.down_frame = None
        self.front_frame = None

        self.z = None
        self.z_rate = 0.0
        self.z_target = self.cruise_z

        self.odom_wait = 0.0
        self.odom_warned = False
        self.dive_settled = 0.0

        self.buoy_done = False
        self.buoy_attempts = 0
        self.buoy_hits = 0
        self.buoy_target_color = None

        self.debug_view_enabled = True

        # Two live views, one per camera, both refreshed every control
        # cycle so neither goes stale while the other behaviour is in
        # charge. Down camera = line following, front camera = buoys.
        self.down_view = None
        self.down_label = ""
        self.front_view = None
        self.front_label = ""

        self._terminal_active = False
        self._terminal_fd = None
        self._terminal_old_settings = None
        self._last_depth_report_time = 0.0
        self._depth_report_interval = 1.0

        self.timer = self.create_timer(self.control_period, self.step)

        self.get_logger().info("Mission control started, waiting for sensors.")

    # ------------------------------------------------------------------
    # Subscriptions

    def _down_cb(self, msg):
        try:
            self.down_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Down camera conversion failed: {exc}")

    def _front_cb(self, msg):
        try:
            self.front_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Front camera conversion failed: {exc}")

    def _odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.z_rate = msg.twist.twist.linear.z

    # ------------------------------------------------------------------
    # Helpers

    def _transition(self, new_state, reason=""):
        if new_state == self.state:
            return

        self.get_logger().info(
            f"{self.state} -> {new_state}" + (f"  ({reason})" if reason else "")
        )
        self.state = new_state
        self.time_in_state = 0.0

    def _publish(self, surge, sway, yaw, heave):
        self.surge_pub.publish(Float64(data=float(surge)))
        self.sway_pub.publish(Float64(data=float(sway)))
        self.yaw_pub.publish(Float64(data=float(yaw)))
        self.heave_pub.publish(Float64(data=float(heave)))
        self.state_pub.publish(String(data=self.state))

    def _set_depth_target(self, value):
        self.z_target = max(self.min_z, min(self.max_z, value))

    def _depth_command(self, dt):
        """Heave thrust from the depth loop, or an open-loop dive if blind."""
        if self.z is None:
            # No odometry. Sink during the dive, then just hold the
            # nominal buoyancy trim (~1.4 N over four thrusters). Purely a
            # fallback so a missing pose bridge degrades instead of
            # letting the vehicle float to the surface.
            return -1.5 if self.state == State.DIVE else -0.35

        return self.depth.update(self.z, self.z_target, self.z_rate, dt)

    # ------------------------------------------------------------------
    # Main loop

    def step(self):
        now = self.get_clock().now()
        dt = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if not (0.0 < dt < 0.5):
            dt = self.control_period

        self.time_in_state += dt
        self.mission_time += dt
        self._maybe_report_depth(dt)

        handler = {
            State.INIT: self._do_init,
            State.DIVE: self._do_dive,
            State.LINE_FOLLOW: self._do_line_follow,
            State.BUOY_APPROACH: self._do_buoy_approach,
            State.BUOY_SEARCH: self._do_buoy_search,
            State.BUOY_TOUCH: self._do_buoy_touch,
            State.BUOY_BACKOFF: self._do_buoy_backoff,
        }[self.state]

        self.down_view = None
        self.front_view = None
        self.down_label = ""
        self.front_label = ""

        handler(dt)

        self._render()

    # ------------------------------------------------------------------
    # States

    def _do_init(self, dt):
        self._publish(0.0, 0.0, 0.0, 0.0)
        self.down_label = self.front_label = f"[{self.state}] waiting for sensors"

        if self.down_frame is None or self.front_frame is None:
            self.get_logger().info("Waiting for camera frames...",
                                   throttle_duration_sec=2.0)
            return

        if self.z is None:
            self.odom_wait += dt
            if self.odom_wait < 5.0:
                self.get_logger().info("Waiting for /bluerov2/odom...",
                                       throttle_duration_sec=2.0)
                return
            if not self.odom_warned:
                self.get_logger().warn(
                    "No odometry after 5 s - diving open loop. Check that "
                    "bluerov2_native_bridge.py is running."
                )
                self.odom_warned = True

        self._set_depth_target(self.cruise_z)
        self.depth.reset()
        self.dive_settled = 0.0
        self._transition(State.DIVE, "sensors up")

    def _do_dive(self, dt):
        heave = self._depth_command(dt)
        self._publish(0.0, 0.0, 0.0, heave)

        self.down_label = self.front_label = (
            f"[{self.state}] -> {self.cruise_depth:.2f} m  heave {heave:+.1f}"
        )

        if self.z is not None:
            error = abs(self.z_target - self.z)
            self.dive_settled = self.dive_settled + dt if error < self.dive_tolerance else 0.0

            self.get_logger().info(
                f"DIVE  depth {self.surface_z - self.z:.2f} -> "
                f"{self.cruise_depth:.2f} m  heave {heave:+.1f}",
                throttle_duration_sec=1.0,
            )

            if self.dive_settled >= self.dive_settle_time:
                self.line.reset()
                self._transition(
                    State.LINE_FOLLOW,
                    f"holding {self.surface_z - self.z:.2f} m depth",
                )
                return

        if self.time_in_state > self.dive_timeout:
            self.line.reset()
            self._transition(State.LINE_FOLLOW, "dive timed out, proceeding")

    def _do_line_follow(self, dt):
        # Down camera drives the line following.
        frame = self.down_frame.copy()
        result = self.line.update(frame, dt, draw=self.debug_view_enabled)

        heave = self._depth_command(dt)
        self._publish(result.surge, result.sway, result.yaw, heave)

        self.down_view = frame
        self.down_label = f"[{self.state}] {result.label}"

        # Front camera watches for the buoys in parallel. Detection runs
        # every frame so the view stays live, but the state change is
        # gated on the arming delay - without it the transition would fire
        # on the first frame of the run, when the red flower is already
        # dead ahead down the course.
        if not self.buoy_done:
            obs, _, candidates = self.buoy.detect(self.front_frame)

            front = self.front_frame.copy()
            if self.debug_view_enabled:
                self.buoy.annotate(front, obs, candidates)
            self.front_view = front

            armed = self.time_in_state > self.buoy_arm_delay
            if obs is None:
                self.front_label = "[WATCHING] no buoy"
            else:
                self.front_label = (
                    f"[WATCHING] closest {obs.color} {obs.distance:.2f}m "
                    f"R {obs.radius_px:.0f}px ({len(candidates)} seen)"
                    + ("" if armed else " | arming")
                )

            if not armed:
                return

            if obs is not None and obs.radius_px >= self.buoy_trigger_radius_px:
                self.buoy_hits += 1
            else:
                self.buoy_hits = 0

            if self.buoy_hits >= self.buoy_trigger_frames:
                self.buoy_hits = 0
                self.buoy.reset()
                self.depth.reset()
                self._set_depth_target(self.z if self.z is not None else self.cruise_z)

                # detect() already ranked by apparent radius, so obs is the
                # closest of however many flowers are in frame. Latch its
                # colour for the rest of the approach.
                self.buoy_target_color = obs.color
                self.buoy.lock(obs.color)

                others = ", ".join(
                    f"{c.color} {c.distance:.1f}m" for c in candidates if c is not obs
                )
                self._transition(
                    State.BUOY_APPROACH,
                    f"closest is {obs.color} at {obs.distance:.2f} m "
                    f"(R {obs.radius_px:.0f} px)"
                    + (f"; also saw {others}" if others else ""),
                )
                return

    def _do_buoy_approach(self, dt):
        frame = self.front_frame.copy()
        obs, _, candidates = self.buoy.detect(frame)

        self.front_view = frame
        self.down_label = f"[{self.state}] line following paused"

        if obs is None:
            self.buoy.lost_frames += 1
            heave = self._depth_command(dt)
            self._publish(0.0, 0.0, 0.0, heave)

            if self.buoy.lost_frames > self.buoy_lost_frames_max:
                self._transition(State.BUOY_SEARCH, "buoy lost")

            self.front_label = f"[{self.state}] lost {self.buoy.lost_frames}"
            return

        self.buoy.lost_frames = 0

        if obs.distance <= self.buoy.touch_distance:
            self._transition(State.BUOY_TOUCH,
                             f"{obs.color} within {obs.distance:.2f} m")
            return

        surge, sway, yaw, heave_rate = self.buoy.compute(obs, dt)

        # Outer loop: the vertical pixel error moves the depth setpoint,
        # the depth loop turns that into thrust.
        self._set_depth_target(self.z_target + heave_rate * dt)
        heave = self._depth_command(dt)

        self._publish(surge, sway, yaw, heave)

        if self.time_in_state > self.buoy_approach_timeout:
            self._abandon_buoy("approach timed out")
            return

        if self.debug_view_enabled:
            self.buoy.annotate(frame, obs, candidates)

        self.front_label = (
            f"[{self.state}] {obs.color} ex {obs.ex:+.2f} ey {obs.ey:+.2f} "
            f"R {obs.radius_px:.0f}px Z {obs.distance:.2f}m surge {surge:.1f}"
        )

    def _do_buoy_search(self, dt):
        frame = self.front_frame.copy()
        obs, _, candidates = self.buoy.detect(frame)

        if self.debug_view_enabled:
            self.buoy.annotate(frame, obs, candidates)
        self.front_view = frame
        self.down_label = f"[{self.state}] line following paused"

        if obs is not None:
            self.buoy.lost_frames = 0
            self._transition(State.BUOY_APPROACH, f"{obs.color} reacquired")
            return

        # Half the search timeout is spent looking for the latched buoy;
        # after that the lock is dropped so any other flower will do.
        if (self.buoy.locked_color is not None
                and self.time_in_state > self.buoy_search_timeout / 2.0):
            self.get_logger().info(
                f"Dropping {self.buoy.locked_color} lock, accepting any buoy."
            )
            self.buoy.unlock()

        surge, sway, yaw, _ = self.buoy.search()
        heave = self._depth_command(dt)
        self._publish(surge, sway, yaw, heave)

        if self.time_in_state > self.buoy_search_timeout:
            self._abandon_buoy("search timed out")
            return

        self.front_label = f"[{self.state}] scanning {self.time_in_state:.1f}s"

    def _do_buoy_touch(self, dt):
        # Open loop. Inside the last ~20 cm the sphere overflows the frame
        # and the detection stops being trustworthy, so the contact is
        # made on dead reckoning from the last good range estimate.
        heave = self._depth_command(dt)
        self._publish(self.touch_surge, 0.0, 0.0, heave)

        target = self.buoy_target_color or "buoy"
        self.front_label = f"[{self.state}] TOUCHING {target} {self.time_in_state:.1f}s"
        self.down_label = f"[{self.state}] line following paused"

        if self.time_in_state > self.touch_duration:
            self.get_logger().info(
                f"*** {(self.buoy_target_color or 'buoy').upper()} BUOY TOUCHED "
                f"- mission 2 complete ***"
            )
            self._transition(State.BUOY_BACKOFF, "contact made")

    def _do_buoy_backoff(self, dt):
        heave = self._depth_command(dt)
        self._publish(self.backoff_surge, 0.0, 0.0, heave)

        self.front_label = f"[{self.state}] backing off {self.time_in_state:.1f}s"
        self.down_label = f"[{self.state}] line following paused"

        if self.time_in_state > self.backoff_duration:
            self.buoy_done = True
            self.buoy.unlock()
            self.line.reset()
            self.depth.reset()
            self._set_depth_target(self.cruise_z)
            self._transition(State.LINE_FOLLOW, "resuming mission 1")

    # ------------------------------------------------------------------

    def _abandon_buoy(self, reason):
        self.buoy_attempts += 1
        self.buoy.unlock()
        self.buoy_target_color = None

        if self.buoy_attempts >= self.max_buoy_attempts:
            self.buoy_done = True
            self.get_logger().warn(f"Giving up on the buoy after {self.buoy_attempts} attempts.")

        self.line.reset()
        self.depth.reset()
        self._set_depth_target(self.cruise_z)
        self._transition(State.LINE_FOLLOW, reason)

    def _depth_info(self):
        if self.z is None:
            return None, None

        pool_depth = max(1e-6, self.surface_z - self.floor_z)
        depth_m = max(0.0, min(pool_depth, self.surface_z - self.z))
        depth_pct = 100.0 * depth_m / pool_depth
        return depth_m, depth_pct

    def _z_text(self):
        """Depth below surface, actual vs commanded."""
        if self.z is None:
            return "depth n/a"
        return (f"depth {self.surface_z - self.z:.2f}"
                f"/{self.surface_z - self.z_target:.2f} m")

    def _enable_terminal_keys(self):
        """Put stdin in cbreak mode so single keys arrive without Enter.

        Without this the terminal stays line buffered and 'r' does nothing
        until Return is pressed. Skipped when stdin is not a tty, e.g.
        under a launch file or when piped.
        """
        if self._terminal_active:
            return
        try:
            if not sys.stdin.isatty():
                return
            self._terminal_fd = sys.stdin.fileno()
            self._terminal_old_settings = termios.tcgetattr(self._terminal_fd)
            tty.setcbreak(self._terminal_fd)
            self._terminal_active = True
        except Exception:
            self._terminal_active = False
            self._terminal_fd = None
            self._terminal_old_settings = None

    def _disable_terminal_keys(self):
        if not self._terminal_active:
            return
        try:
            if self._terminal_old_settings is not None:
                termios.tcsetattr(self._terminal_fd, termios.TCSADRAIN,
                                  self._terminal_old_settings)
        except Exception:
            pass
        self._terminal_active = False

    def _read_terminal_key(self):
        if not self._terminal_active:
            self._enable_terminal_keys()
        if not self._terminal_active:
            return None

        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        except Exception:
            return None
        if not ready:
            return None

        try:
            ch = sys.stdin.read(1)
        except Exception:
            return None
        if not ch:
            return None
        return ch.lower()

    def _report_depth_now(self, reason=""):
        info = self._depth_info()
        if info[0] is None:
            print("[DEPTH] unavailable (no odometry)", flush=True)
            return

        depth_m, depth_pct = info
        pool_depth = max(1e-6, self.surface_z - self.floor_z)
        print(
            f"[DEPTH] {reason}{reason and ' - ' if reason else ''}{depth_m:.2f} m / {depth_pct:.1f}% of pool depth"
            f" | surface=0 m | floor={pool_depth:.2f} m",
            flush=True,
        )

    def _maybe_report_depth(self, dt):
        key = self._read_terminal_key()
        if key in {"r", "f"}:
            self._report_depth_now("key")
            self._last_depth_report_time = self.mission_time
            return
        if key == "x":
            raise KeyboardInterrupt

        # Paced off the mission clock, not time_in_state: the latter resets
        # on every transition, which made the interval go negative and the
        # periodic report fire erratically.
        if self.mission_time - self._last_depth_report_time >= self._depth_report_interval:
            self._report_depth_now("periodic")
            self._last_depth_report_time = self.mission_time

    def _render(self):
        """Draw both camera views.

        Down camera for the line, front camera for the buoys - two fixed
        windows so each one always shows the camera it belongs to, rather
        than one window that swaps sources as the state changes. The
        binary threshold masks are not shown; the annotated colour frames
        carry everything worth looking at.
        """
        if not self.debug_view_enabled:
            return

        depth_m, depth_pct = self._depth_info()
        if depth_m is None:
            depth_label = f"Depth n/a  ->  {self.cruise_depth:.2f} m"
        else:
            depth_label = (
                f"Depth {depth_m:.2f} m ({depth_pct:.0f}%)  ->  "
                f"{self.surface_z - self.z_target:.2f} m"
            )

        down = self.down_view if self.down_view is not None else self.down_frame
        front = self.front_view if self.front_view is not None else self.front_frame

        try:
            for window, frame, label in (
                ("Line Following - Down Camera", down, self.down_label),
                ("Buoy Detection - Front Camera", front, self.front_label),
            ):
                if frame is None:
                    continue
                if frame is self.down_frame or frame is self.front_frame:
                    frame = frame.copy()

                cv.putText(frame, label, (12, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv.putText(frame, depth_label, (12, 58),
                           cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv.imshow(window, frame)

            cv.waitKey(1)
        except cv.error:
            self.debug_view_enabled = False


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._disable_terminal_keys()
        except Exception:
            pass
        try:
            node._publish(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        cv.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
