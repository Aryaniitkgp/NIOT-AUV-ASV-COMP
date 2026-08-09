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
import os
import math
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

from sensor_msgs.msg import Imu, CameraInfo
from std_msgs.msg import Empty as EmptyMsg

from line_follow import LineFollowController
from buoy import BuoyServoController
from marker import MarkerDropController
from octagon import OctagonDetector
from depth_control import DepthController
from depth_filter import DepthFilter, vertical_accel_from_imu


class State:
    INIT = "INIT"
    DIVE = "DIVE"
    LINE_FOLLOW = "LINE_FOLLOW"
    BUOY_APPROACH = "BUOY_APPROACH"
    BUOY_SEARCH = "BUOY_SEARCH"
    BUOY_TOUCH = "BUOY_TOUCH"
    BUOY_BACKOFF = "BUOY_BACKOFF"

    # Mission 4 - marker dropping
    BIN_APPROACH = "BIN_APPROACH"   # centre over a bin at cruise depth, classify
    BIN_REJECT = "BIN_REJECT"       # wrong symbol, move on to the next bin
    BIN_DESCEND = "BIN_DESCEND"     # drop toward release altitude, stay centred
    BIN_HOLD = "BIN_HOLD"           # kill residual velocity before releasing
    BIN_DROP = "BIN_DROP"           # release one marker
    BIN_DONE = "BIN_DONE"           # climb back to cruise, resume the path

    # Mission 6 - surfacing inside the octagon
    OCTAGON_ARRIVE = "OCTAGON_ARRIVE"  # line has run out; creep to the centre
    OCTAGON_HOLD = "OCTAGON_HOLD"      # kill horizontal velocity before rising
    OCTAGON_ASCEND = "OCTAGON_ASCEND"  # rise, checking containment on the way
    SURFACED = "SURFACED"              # mission complete


class MissionControl(Node):

    def __init__(self):
        super().__init__("mission_control")

        self.bridge = CvBridge()

        self.line = LineFollowController()
        self.buoy = BuoyServoController(colors=("red", "green", "yellow"))
        self.marker = MarkerDropController()
        self.octagon = OctagonDetector()
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
        self.create_subscription(
            Float64, "/bluerov2/depth", self._depth_cb, qos)
        self.create_subscription(
            Imu, "/bluerov2/imu/data", self._imu_cb, qos)
        self.create_subscription(
            CameraInfo, "/bluerov2/front_camera/camera_info",
            self._front_info_cb, qos)

        self.surge_pub = self.create_publisher(Float64, "/cmd_surge", 10)
        self.sway_pub = self.create_publisher(Float64, "/cmd_sway", 10)
        self.yaw_pub = self.create_publisher(Float64, "/cmd_yaw", 10)
        self.heave_pub = self.create_publisher(Float64, "/cmd_heave", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.drop_pub = self.create_publisher(EmptyMsg, "/bluerov2/drop_marker", 10)

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
        # Shallowest the vehicle may ever go. 0.30 m was too generous: the
        # visual servo integrates z_target upward while a target sits high
        # in frame, and the clamp let it reach z = 2.20, which lifts the
        # hull clear of the water and loses both buoyancy and the cameras.
        # 0.90 m keeps a comfortable margin under the surface while still
        # allowing the buoy climb-over at z = 1.28 (depth 1.22 m).
        self.min_depth = 0.90
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
        # One spare attempt per flower, so a single failed approach does
        # not end mission 2 while untouched buoys remain.
        self.max_buoy_attempts = 5

        # Touch geometry.
        #
        # Cruising at z = 1.00 puts the vehicle at exactly the flower
        # CENTRE height, so closing on the target is a head-on collision
        # rather than a touch. The spheres are r = 0.1145 centred at
        # z = 1.00, so their tops are at z = 1.115; the hull bottom sits
        # 0.035 above the vehicle origin.
        #
        # Riding at z = 1.28 puts the hull bottom at 1.315 - about 20 cm
        # over the top of the sphere - so the vehicle passes OVER the buoy
        # and brushes it, instead of driving into its equator. The vehicle
        # climbs to this only for the final approach, so the buoy stays
        # near the optical axis for detection at longer range.
        self.buoy_touch_z = 1.28
        # Start climbing well before contact. A real run triggered on the
        # red flower at 1.64 m, which left only 0.44 m of closing before
        # the climb began - the target left the frame during the manoeuvre
        # and the approach collapsed into a search. Climbing from 2.0 m
        # means the vehicle is already at touch height by the time the
        # buoy fills the frame.
        self.buoy_climb_distance = 2.00
        # Range at which the approach hands over to the open-loop touch.
        # Must be comfortably ABOVE the range where the buoy leaves the
        # bottom of the frame (~0.70 m at the climb height), or the
        # approach loses the target before it can ever commit.
        self.buoy_commit_distance = 0.90
        # How far the visual servo may move the depth setpoint either side
        # of cruise depth while homing. Keeps a mis-detection from walking
        # the vehicle to the surface or the floor.
        self.buoy_servo_z_band = 0.35

        # Open-loop run from buoy_commit_distance. At surge 2.5 the vehicle
        # settles near 0.46 m/s, so ~1.9 s covers the ~0.7 m from the
        # commit point to contact with a little margin.
        self.touch_surge = 2.5
        self.touch_duration = 1.9
        self.backoff_surge = -2.5
        self.backoff_duration = 2.0

        # The mission scores "touch at least one buoy", so one is enough to
        # satisfy it and lets the run get on to the L-bar and the bins.
        # Raise to 3 to attempt every flower once the later missions are
        # working - the machinery for it is already in place.
        self.buoys_to_touch = 1

        # Route at the fork.
        #
        # The path splits at (-3, 0) into two symmetric 45 degree legs.
        # Body +y is port, and the bins sit at y = -4 (starboard), while
        # the cupid torpedo target is at y = +4 (port). Marker dropping is
        # implemented and the torpedo is not, so take the starboard leg.
        #
        # Set to "port" for the torpedo route, or None to let the vehicle
        # drift onto whichever leg dominates - which is what it did before
        # this existed, and it picked the unimplemented route.
        self.route = "starboard"
        self.line.branch_preference = self.route
        self._fork_logged = False

        # ---- Mission 6: surfacing inside the octagon ----------------
        #
        # left_2 terminates at (8, -5), which IS the octagon centre, so
        # running out of line in the final leg means the vehicle has
        # arrived. Only armed once the bins are done, or an ordinary
        # mid-course line loss would trigger a premature surfacing.
        self.octagon_lost_frames = 25     # line gone this long = end of path
        self.octagon_creep_surge = 1.5
        self.octagon_creep_time = 2.5     # ease into the centre after the line ends

        self.octagon_hold_speed_mps = 0.06
        self.octagon_hold_min_time = 1.5
        self.octagon_hold_timeout = 10.0

        # Ascent. The depth floor that stops the buoy servo walking the
        # vehicle to the surface has to be lifted here - surfacing is the
        # whole point of this state.
        self.octagon_ascend_rate = 0.18   # m/s of setpoint travel
        self.surface_depth = 0.12         # Bar30 depth counted as surfaced
        self.octagon_ascend_timeout = 40.0

        self.octagon_lost = 0
        # Set once the line is re-acquired after the bins, so the
        # end-of-course trigger cannot fire on the bin's own occlusion.
        self.octagon_line_seen = False
        self.octagon_done = False
        self._octagon_reports = []

        # ---- Mission 4: marker dropping -----------------------------
        #
        # Which bin to use. The two are told apart by the white symbol on
        # their floor: a ring ("O") at x=3.6 and a cross ("X") at x=4.6.
        self.target_symbol = "X"
        self.markers_carried = 2

        # Engagement. Bins are only looked for once the buoy is done, and
        # the area gate keeps the transition from firing on a distant
        # sliver of navy at the edge of frame.
        self.bin_trigger_area_px = 12000.0
        self.bin_trigger_frames = 4
        self.bin_lost_frames_max = 8

        # Release geometry. The bin rim is at z = 0.30 and the hull bottom
        # sits 0.035 above the vehicle origin, so z = 0.75 clears the rim
        # by about 0.45 m and leaves a 0.72 m fall onto the floor.
        self.bin_drop_z = 0.75
        self.bin_centre_tolerance_m = 0.06
        self.bin_descend_tolerance_m = 0.12

        # A marker inherits the vehicle's horizontal velocity. Falling
        # 0.72 m at a realistic 0.5-1.0 m/s takes 0.7-1.4 s, so 0.3 m/s of
        # residual drift would put it 0.2-0.4 m downrange against a bin
        # half-width of only 0.32 m - a miss caused purely by not having
        # stopped. Hence an explicit hold, gated on the visual speed
        # estimate from the down camera.
        self.bin_hold_speed_mps = 0.05
        self.bin_hold_min_time = 1.0
        self.bin_hold_timeout = 8.0

        self.bin_drop_settle = 1.0

        # Step-over after rejecting a bin. The two bins are only 1.0 m
        # apart, and surge 2.5 settles near 0.46 m/s, so anything past
        # ~1.2 s sails straight over the second bin and the vehicle spends
        # the rest of the run looking for it downrange.
        self.bin_reject_surge = 2.5
        self.bin_reject_duration = 1.2
        # Attempts before abandoning the bins, so a bin that cannot be
        # held does not trap the run in an approach/lose loop.
        self.max_bin_attempts = 3
        # Forward creep while climbing out of a completed bin, so the
        # vehicle clears the box and can see the line again.
        self.bin_exit_surge = 2.0
        self.bin_approach_timeout = 45.0

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

        # Estimated vertical state, from Bar30 + IMU. Set
        # use_ground_truth_depth = True to bypass the estimator and fly on
        # perfect odometry instead, for an A/B against the old behaviour.
        self._intrinsics_set = False
        self.use_ground_truth_depth = False
        self.depth_filter = DepthFilter()
        self.depth_meas = None
        self.accel_up = None

        self.z = None
        self.z_rate = 0.0
        self.z_target = self.cruise_z

        # Truth, for logging estimator error only - never fed to control.
        self.true_z = None
        self.true_z_rate = 0.0
        self.true_xy = None
        self._est_err_peak = 0.0

        self.odom_wait = 0.0
        self.odom_warned = False
        self.dive_settled = 0.0

        self.buoy_done = False
        self.buoy_attempts = 0
        self.buoy_hits = 0
        self.buoy_target_color = None
        # Colours already touched, so the lookout re-arms on the remaining
        # flowers instead of re-approaching one it has already scored.
        self.buoys_touched = set()

        self.bins_done = False
        self.bin_hits = 0
        self.bin_lost = 0
        # Symbols already inspected and turned down, so the lookout does
        # not walk straight back into the bin it just stepped over.
        self.rejected_symbols = set()
        self.markers_dropped = 0
        self.bin_hold_elapsed = 0.0
        self.bin_attempts = 0
        self.drop_log = []
        # One-shot latch for the marker release, armed by _transition.
        self._drop_fired = False

        self.debug_view_enabled = True
        # Disable debug GUI when running headless to avoid Qt plugin crashes.
        if os.environ.get("QT_QPA_PLATFORM", "") == "offscreen" or not os.environ.get("DISPLAY"):
            self.debug_view_enabled = False

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
        """Ground truth. Kept for scoring and estimator error only.

        This is deliberately NOT fed to the controller: it is perfect,
        instantaneous and unavailable on the real vehicle. Everything the
        depth loop sees comes from the Bar30 + IMU estimator instead, so
        the gains get tuned against something the hardware can actually
        reproduce.
        """
        self.true_z = msg.pose.pose.position.z
        self.true_z_rate = msg.twist.twist.linear.z
        self.true_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.use_ground_truth_depth:
            self.z = self.true_z
            self.z_rate = self.true_z_rate

    def _depth_cb(self, msg):
        """Noisy, quantised, delayed depth from the simulated Bar30."""
        self.depth_meas = float(msg.data)

    def _imu_cb(self, msg):
        a = msg.linear_acceleration
        self.accel_up = vertical_accel_from_imu(a.x, a.y, a.z)

    def _front_info_cb(self, msg):
        """Install the front camera's real intrinsics into the buoy servo.

        Only once - the calibration does not change mid-run, and this
        arrives at frame rate.
        """
        if self._intrinsics_set:
            return

        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return

        self.buoy.set_intrinsics(fx, fy, cx, cy, msg.d)
        self._intrinsics_set = True
        self.get_logger().info(
            f"Front camera intrinsics: fx={fx:.1f} cx={cx:.1f} cy={cy:.1f} "
            f"dist={'yes' if any(abs(d) > 1e-12 for d in msg.d) else 'none'}"
        )

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

        # Arm the one-shot marker release on every fresh entry into
        # BIN_DROP, so the second marker gets its own trigger.
        if new_state == State.BIN_DROP:
            self._drop_fired = False

    def _publish(self, surge, sway, yaw, heave):
        self.surge_pub.publish(Float64(data=float(surge)))
        self.sway_pub.publish(Float64(data=float(sway)))
        self.yaw_pub.publish(Float64(data=float(yaw)))
        self.heave_pub.publish(Float64(data=float(heave)))
        self.state_pub.publish(String(data=self.state))

    def _set_depth_target(self, value):
        self.z_target = max(self.min_z, min(self.max_z, value))

    def _update_depth_estimate(self, dt):
        """Run the Bar30 + IMU filter once per control cycle.

        Predict on every tick using the IMU (which arrives far faster than
        the control loop), correct only when a fresh Bar30 sample has
        turned up. That ordering is what keeps the estimate smooth between
        the 20 Hz depth samples instead of stepping at each one.
        """
        if self.use_ground_truth_depth:
            return

        self.depth_filter.predict(dt, self.accel_up)

        if self.depth_meas is not None:
            # Sensor reads depth below the surface; the controller works
            # in world z, which is up-positive.
            self.depth_filter.update(self.surface_z - self.depth_meas)
            self.depth_meas = None

        if self.depth_filter.ready:
            self.z = self.depth_filter.z
            self.z_rate = self.depth_filter.vz

            if self.true_z is not None:
                self._est_err_peak = max(self._est_err_peak, abs(self.z - self.true_z))

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

        self._update_depth_estimate(dt)
        self._maybe_report_depth(dt)

        handler = {
            State.INIT: self._do_init,
            State.DIVE: self._do_dive,
            State.LINE_FOLLOW: self._do_line_follow,
            State.BUOY_APPROACH: self._do_buoy_approach,
            State.BUOY_SEARCH: self._do_buoy_search,
            State.BUOY_TOUCH: self._do_buoy_touch,
            State.BUOY_BACKOFF: self._do_buoy_backoff,
            State.BIN_APPROACH: self._do_bin_approach,
            State.BIN_REJECT: self._do_bin_reject,
            State.BIN_DESCEND: self._do_bin_descend,
            State.BIN_HOLD: self._do_bin_hold,
            State.BIN_DROP: self._do_bin_drop,
            State.BIN_DONE: self._do_bin_done,
            State.OCTAGON_ARRIVE: self._do_octagon_arrive,
            State.OCTAGON_HOLD: self._do_octagon_hold,
            State.OCTAGON_ASCEND: self._do_octagon_ascend,
            State.SURFACED: self._do_surfaced,
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
                self.get_logger().info("Waiting for depth estimate...",
                                       throttle_duration_sec=2.0)
                return
            if not self.odom_warned:
                self.get_logger().warn(
                    "No depth after 5 s - diving open loop. Check that "
                    "bluerov2_native_bridge.py is running and publishing "
                    "/bluerov2/depth."
                )
                self.odom_warned = True
        else:
            source = "ground truth" if self.use_ground_truth_depth else "Bar30 + IMU"
            self.get_logger().info(f"Depth estimate live ({source}).")

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

        # Announce the fork once, when it is first resolved, so the route
        # taken is visible in a headless log rather than only inferable
        # from the trajectory afterwards.
        if result.at_fork and not self._fork_logged and self.line.fork_frames >= 3:
            self._fork_logged = True
            dest = "bins (marker drop)" if self.route == "starboard" else "cupid (torpedo)"
            self.get_logger().info(
                f"*** FORK: {result.branches} legs seen, taking {self.route} "
                f"-> {dest} ***")

        # End of course. Both branches terminate at an octagon centre, so
        # once the markers are done, the line running out is the arrival
        # signal - there is nothing further to follow.
        #
        # Armed only after the bins, otherwise an ordinary mid-course
        # dropout (glare, a gap, the vehicle crossing the L-bar) would be
        # read as the end of the run.
        if self.bins_done and not self.octagon_done:
            # The trigger only arms once the line has actually been SEEN
            # again since the bins finished.
            #
            # Without this the vehicle declares arrival immediately after
            # BIN_DONE: the path runs directly beneath the bins (measured
            # 0.12 m off centre at the O bin, against a 0.32 m half-width),
            # so a 0.64 m navy box occludes it while the vehicle climbs
            # back to cruise. A real run counted its 25 lost frames there
            # and surfaced at x=4.08 - 5.2 m outside the ring.
            if result.found:
                self.octagon_line_seen = True
                self.octagon_lost = 0
            elif self.octagon_line_seen:
                self.octagon_lost += 1

            if (self.octagon_line_seen
                    and self.octagon_lost >= self.octagon_lost_frames):
                self._transition(
                    State.OCTAGON_ARRIVE,
                    f"line ended ({self.octagon_lost} frames) - at the octagon")
                return
        else:
            self.octagon_lost = 0
            self.octagon_line_seen = False

        # Once the buoy is behind us the down camera doubles as the bin
        # lookout. Detection shares the frame the line follower just used,
        # so this costs one extra HSV pass and no extra latency.
        if self.buoy_done and not self.bins_done and self.z is not None:
            bins, _ = self.marker.detect(self.down_frame, self.z)
            big = [b for b in bins
                   if b.area_px >= self.bin_trigger_area_px
                   and b.symbol not in self.rejected_symbols]

            self.bin_hits = self.bin_hits + 1 if big else 0

            if big:
                b = big[0]
                self.down_label += f" | bin {b.symbol or '?'} seen"

            if self.bin_hits >= self.bin_trigger_frames:
                self.bin_hits = 0
                self.marker.reset()
                self._transition(
                    State.BIN_APPROACH,
                    f"bin in view ({big[0].area_px:.0f} px)",
                )
                return

        # Front camera watches for the buoys in parallel. Detection runs
        # every frame so the view stays live, but the state change is
        # gated on the arming delay - without it the transition would fire
        # on the first frame of the run, when the red flower is already
        # dead ahead down the course.
        if not self.buoy_done:
            _, _, candidates = self.buoy.detect(self.front_frame)

            # Skip flowers already scored, then re-rank: largest apparent
            # radius is nearest, since Z = R_real * f / R_pixels.
            remaining = [c for c in candidates if c.color not in self.buoys_touched]
            obs = max(remaining, key=lambda c: c.radius_px) if remaining else None

            front = self.front_frame.copy()
            if self.debug_view_enabled:
                self.buoy.annotate(front, obs, candidates)
            self.front_view = front

            armed = self.time_in_state > self.buoy_arm_delay
            touched = f"{len(self.buoys_touched)}/{self.buoys_to_touch}"
            if obs is None:
                self.front_label = f"[WATCHING] no new buoy | touched {touched}"
            else:
                self.front_label = (
                    f"[WATCHING] closest {obs.color} {obs.distance:.2f}m "
                    f"R {obs.radius_px:.0f}px | touched {touched}"
                    + ("" if armed else " | arming")
                )

            # Headless runs have no debug window, so without this there is
            # no way to tell "detector saw nothing" from "saw something and
            # rejected it" from "never reached this code at all".
            self.get_logger().info(
                f"WATCH armed={armed} cands={len(candidates)} "
                + (f"best={obs.color}@{obs.distance:.2f}m R={obs.radius_px:.0f}px "
                   f"(need {self.buoy_trigger_radius_px:.0f}) hits={self.buoy_hits}"
                   if obs else "best=none"),
                throttle_duration_sec=1.0,
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
        _, _, candidates = self.buoy.detect(frame)

        # Honour the latch here too. detect() returns the globally closest
        # candidate, so without this the approach silently retargets a
        # different flower mid-run and the servo never converges on any of
        # them.
        wanted = self.buoy.locked_color
        pool = [c for c in candidates if c.color not in self.buoys_touched]
        if wanted is not None:
            pool = [c for c in pool if c.color == wanted]
        obs = max(pool, key=lambda c: c.radius_px) if pool else None

        self.front_view = frame
        self.down_label = f"[{self.state}] line following paused"

        if obs is None:
            self.buoy.lost_frames += 1
            heave = self._depth_command(dt)
            self._publish(0.0, 0.0, 0.0, heave)

            if self.buoy.lost_frames > self.buoy_lost_frames_max:
                # Log what was actually in frame when the lock was lost -
                # otherwise a headless run gives no way to tell "target
                # left the field of view" from "detector rejected it".
                seen = ", ".join(f"{c.color}@{c.distance:.2f}m" for c in candidates)
                self._transition(
                    State.BUOY_SEARCH,
                    f"lost {wanted or 'buoy'}; in frame: {seen or 'nothing'}")

            self.front_label = f"[{self.state}] lost {self.buoy.lost_frames}"
            return

        self.buoy.lost_frames = 0

        # Commit to the touch while the buoy is still VISIBLE.
        #
        # Riding at z = 1.28 puts the camera 0.33 m above the sphere, and
        # with a 23.4 deg vertical half-FOV the target falls out of the
        # bottom of the frame at about 0.70 m range - well before the old
        # 0.35 m touch threshold. The approach therefore lost the target
        # every time and collapsed into a search, which is precisely the
        # repeating cycle seen in the logs.
        #
        # Committing at commit_distance and running the last stretch open
        # loop is the only option: no camera placement makes a target
        # directly beneath the hull visible to a forward camera.
        if obs.distance <= self.buoy_commit_distance:
            self._transition(State.BUOY_TOUCH,
                             f"{obs.color} committed at {obs.distance:.2f} m")
            return

        surge, sway, yaw, heave_rate = self.buoy.compute(obs, dt)

        # Vertical handling switches partway in.
        #
        # Far out, the visual servo owns depth: e_y drives a climb/descend
        # rate so the buoy stays near the optical axis and detection stays
        # good. Inside buoy_climb_distance that would fly the vehicle
        # straight into the sphere's equator, so depth is instead commanded
        # to buoy_touch_z and the vehicle rides OVER the target.
        if obs.distance <= self.buoy_climb_distance:
            self._set_depth_target(self.buoy_touch_z)
            vertical = "climbing over"
        else:
            # Bound the servo's vertical authority to a band around cruise
            # depth. Left unbounded it integrates z_target toward whatever
            # the target's pixel elevation implies, and a target sitting
            # high in frame will walk the vehicle all the way to the
            # surface - which is exactly what happened before this clamp.
            proposed = self.z_target + heave_rate * dt
            lo = self.cruise_z - self.buoy_servo_z_band
            hi = self.cruise_z + self.buoy_servo_z_band
            self._set_depth_target(max(lo, min(hi, proposed)))
            vertical = "servo"

        heave = self._depth_command(dt)
        self._publish(surge, sway, yaw, heave)

        if self.time_in_state > self.buoy_approach_timeout:
            self._abandon_buoy("approach timed out")
            return

        if self.debug_view_enabled:
            self.buoy.annotate(frame, obs, candidates)

        self.front_label = (
            f"[{self.state}] {obs.color} ex {obs.ex:+.2f} ey {obs.ey:+.2f} "
            f"Z {obs.distance:.2f}m surge {surge:.1f} | {vertical}"
        )

        # Same line into the log. Headless runs have no debug window, so
        # without this there is no record of how the approach progressed.
        self.get_logger().info(
            f"APPROACH {obs.color} Z={obs.distance:.2f}m R={obs.radius_px:.0f}px "
            f"ex={obs.ex:+.2f} ey={obs.ey:+.2f} surge={surge:.1f} "
            f"z={self.z:.2f}->{self.z_target:.2f} {vertical}",
            throttle_duration_sec=0.5,
        )

    def _do_buoy_search(self, dt):
        frame = self.front_frame.copy()
        _, _, candidates = self.buoy.detect(frame)

        # Re-acquire the LATCHED colour only.
        #
        # Accepting any candidate here defeats the lock: the vehicle drops
        # red, picks up yellow, loses it, picks up green, and thrashes
        # between the flowers without ever committing. Observed in a real
        # run as 40 approach/search cycles and zero touches.
        #
        # The lock is dropped further down once half the search timeout has
        # elapsed, so a genuinely unreachable buoy is not a dead end.
        wanted = self.buoy.locked_color
        if wanted is not None:
            candidates = [c for c in candidates if c.color == wanted]
        candidates = [c for c in candidates if c.color not in self.buoys_touched]
        obs = max(candidates, key=lambda c: c.radius_px) if candidates else None

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
        # Open loop, riding at buoy_touch_z so the hull passes over the
        # sphere and brushes it. Inside the last ~20 cm the sphere
        # overflows the frame and detection stops being trustworthy, so
        # contact is made on dead reckoning from the last good range.
        self._set_depth_target(self.buoy_touch_z)
        heave = self._depth_command(dt)
        self._publish(self.touch_surge, 0.0, 0.0, heave)

        target = self.buoy_target_color or "buoy"
        self.front_label = f"[{self.state}] TOUCHING {target} {self.time_in_state:.1f}s"
        self.down_label = f"[{self.state}] line following paused"

        if self.time_in_state > self.touch_duration:
            if self.buoy_target_color:
                self.buoys_touched.add(self.buoy_target_color)

            self.get_logger().info(
                f"*** {(self.buoy_target_color or 'buoy').upper()} BUOY TOUCHED "
                f"({len(self.buoys_touched)}/{self.buoys_to_touch}) ***"
            )
            self._transition(State.BUOY_BACKOFF, "contact made")

    def _do_buoy_backoff(self, dt):
        heave = self._depth_command(dt)
        self._publish(self.backoff_surge, 0.0, 0.0, heave)

        self.front_label = f"[{self.state}] backing off {self.time_in_state:.1f}s"
        self.down_label = f"[{self.state}] line following paused"

        if self.time_in_state > self.backoff_duration:
            # Mission 2 only ends once every flower has been scored (or
            # the attempt budget is spent). Otherwise re-arm and let the
            # lookout pick up the next untouched colour.
            if len(self.buoys_touched) >= self.buoys_to_touch:
                self.buoy_done = True
                reason = "all buoys touched, resuming mission 1"
            else:
                reason = (f"{len(self.buoys_touched)}/{self.buoys_to_touch} touched, "
                          f"looking for the next")

            self.buoy_target_color = None
            self.buoy.unlock()
            self.buoy_hits = 0
            self.line.reset()
            self.depth.reset()
            self._set_depth_target(self.cruise_z)
            self._transition(State.LINE_FOLLOW, reason)

    # ------------------------------------------------------------------
    # Mission 6 - surfacing inside the octagon

    def _octagon_view(self, dt, annotate=True):
        """Shared per-tick ring sensing on the FRONT camera."""
        frame = self.front_frame.copy()
        view, _ = self.octagon.detect(frame)

        if annotate and self.debug_view_enabled:
            self.octagon.annotate(frame, view)

        self.front_view = frame
        self.down_label = f"[{self.state}] path complete"
        return view

    def _do_octagon_arrive(self, dt):
        """Creep the last little way onto the octagon centre.

        The line ended, which by construction means the vehicle is at the
        centre. A short creep covers the gap between losing sight of the
        line and actually being over the terminus.
        """
        view = self._octagon_view(dt)
        heave = self._depth_command(dt)
        self._publish(self.octagon_creep_surge, 0.0, 0.0, heave)

        self.front_label = (
            f"[{self.state}] creeping {self.time_in_state:.1f}"
            f"/{self.octagon_creep_time:.1f}s | ring {view.ring_px}px "
            f"cov {view.coverage:.2f}")

        if self.time_in_state > self.octagon_creep_time:
            self.marker.reset()          # reuse its visual speed estimator
            self._transition(State.OCTAGON_HOLD, "at the centre")

    def _do_octagon_hold(self, dt):
        """Kill horizontal velocity before rising.

        Same reasoning as the marker release: commanding zero is not the
        same as being stopped, and drifting sideways during a 1.5 m ascent
        is how a vehicle ends up surfacing outside the ring.

        Speed comes from the down camera, which still sees the floor here
        even though the line has ended.
        """
        view = self._octagon_view(dt)
        heave = self._depth_command(dt)
        self._publish(0.0, 0.0, 0.0, heave)

        # Track floor texture for the speed estimate. Any bin-like blob
        # will do; the absolute reference does not matter, only its motion.
        bins, _ = self.marker.detect(self.down_frame,
                                     self.z if self.z else self.cruise_z)
        self.marker.update_speed(bins[0] if bins else None,
                                 self.z if self.z else self.cruise_z, dt)

        speed = self.marker.speed_mps
        self.front_label = (
            f"[{self.state}] v={self._speed_text()} {self.time_in_state:.1f}s "
            f"| ring cov {view.coverage:.2f}")

        settled = (self.time_in_state > self.octagon_hold_min_time
                   and (speed is None or speed < self.octagon_hold_speed_mps))

        if settled or self.time_in_state > self.octagon_hold_timeout:
            why = "stopped" if settled else "hold timed out"
            self._octagon_reports = []
            self._transition(State.OCTAGON_ASCEND, why)

    def _do_octagon_ascend(self, dt):
        """Rise to the surface, checking containment on the way up.

        The containment test only works during the ascent: at cruise depth
        the ring sits above the camera's field of view entirely, and only
        comes level around z = 2.2. Measured on real frames, inside gives
        ~600 px in the densest row over 13 rows; outside at 4 m gives ~98
        over 29. That difference is the check.
        """
        # Walk the setpoint up. min_depth normally floors this to keep the
        # buoy servo from surfacing the vehicle, so bypass the clamp here.
        self.z_target = min(self.surface_z,
                            self.z_target + self.octagon_ascend_rate * dt)
        heave = self._depth_command(dt)
        self._publish(0.0, 0.0, 0.0, heave)

        view = self._octagon_view(dt)
        depth = self.surface_z - self.z if self.z is not None else None

        if view.ring_px >= self.octagon.min_ring_px:
            self._octagon_reports.append(view.surrounded)

        self.front_label = (
            f"[{self.state}] depth {depth:.2f}m ring {view.ring_px}px "
            f"cov {view.coverage:.2f} peak {view.peak_row_px} "
            f"{'INSIDE' if view.surrounded else 'checking'}"
            if depth is not None else f"[{self.state}] rising")

        self.get_logger().info(
            f"ASCEND depth={depth:.2f}m z={self.z:.2f}->{self.z_target:.2f} "
            f"ring={view.ring_px}px cov={view.coverage:.2f} "
            f"peak={view.peak_row_px} rows={view.rows} "
            f"inside={view.surrounded} floats={view.floats_px}",
            throttle_duration_sec=1.0,
        )

        if depth is not None and depth <= self.surface_depth:
            self._finish_octagon(depth)
            return

        if self.time_in_state > self.octagon_ascend_timeout:
            self._finish_octagon(depth, timed_out=True)

    def _finish_octagon(self, depth, timed_out=False):
        votes = self._octagon_reports
        inside = sum(1 for v in votes if v)
        verdict = (f"{inside}/{len(votes)} frames saw the ring surrounding"
                   if votes else "ring never resolved")

        self.octagon_done = True
        self.get_logger().info(
            f"*** SURFACED at {depth:.2f} m depth - {verdict} ***"
            if depth is not None else f"*** SURFACED - {verdict} ***")
        if timed_out:
            self.get_logger().warn("Ascent timed out before reaching the surface.")

        self._transition(State.SURFACED, "mission complete")

    def _do_surfaced(self, dt):
        """Hold station at the surface. Nothing further to do."""
        self._publish(0.0, 0.0, 0.0, 0.0)
        self.front_label = self.down_label = "[SURFACED] mission complete"

    # ------------------------------------------------------------------
    # Mission 4 - marker dropping
    #
    # Bins are chosen by inspection, not by remembered position: the
    # vehicle has no horizontal odometry, so "fly to where the X bin was"
    # is not a sentence it can act on. Instead it centres over whatever
    # bin is underneath, reads the symbol, and either commits or steps
    # over it and carries on down the path until the next one appears.

    def _bin_view(self, dt, annotate=True):
        """Shared per-tick bin sensing. Returns the bin underneath, or None."""
        frame = self.down_frame.copy()
        bins, _ = self.marker.detect(self.down_frame, self.z if self.z else self.cruise_z)

        obs = bins[0] if bins else None
        self.marker.update_speed(obs, self.z if self.z else self.cruise_z, dt)

        if annotate and self.debug_view_enabled:
            self.marker.annotate(frame, bins, self.target_symbol)

        self.down_view = frame
        self.front_label = f"[{self.state}] front camera idle"
        return obs

    def _do_bin_approach(self, dt):
        obs = self._bin_view(dt)
        heave = self._depth_command(dt)

        if obs is None:
            self.bin_lost += 1
            self._publish(0.0, 0.0, 0.0, heave)
            self.down_label = f"[{self.state}] bin lost {self.bin_lost}"

            if self.bin_lost > self.bin_lost_frames_max:
                # Count it as an attempt. Without this the vehicle drops
                # back to the line, immediately re-triggers on the same
                # bin, loses it again, and can ping-pong until the run
                # ends - which looks from outside like it "hovers over the
                # box then wanders off along the line".
                self.bin_attempts += 1
                self.line.reset()
                self.bin_hits = 0

                if self.bin_attempts >= self.max_bin_attempts:
                    self.bins_done = True
                    self.get_logger().warn(
                        f"Giving up on the bins after {self.bin_attempts} "
                        f"attempts; {self.markers_dropped} marker(s) dropped.")

                self._transition(State.LINE_FOLLOW,
                                 f"lost the bin (attempt {self.bin_attempts})")
            return

        self.bin_lost = 0
        surge, sway, yaw, _ = self.marker.compute(obs, dt)
        self._publish(surge, sway, yaw, heave)

        err = math.hypot(obs.err_x_m, obs.err_y_m)
        self.down_label = (
            f"[{self.state}] {obs.symbol or '?'} err {err * 100:.0f} cm "
            f"c={obs.circularity:.2f} v={self._speed_text()}"
        )

        # Commit needs BOTH centring and a classified symbol; without this
        # a headless run cannot tell which of the two is holding it up.
        self.get_logger().info(
            f"BIN sym={obs.symbol or 'NONE'} circ={obs.circularity:.2f} "
            f"hole={obs.has_hole} area={obs.area_px:.0f} "
            f"track={obs.tracked_on} err={err*100:.0f}cm "
            f"(need <{self.bin_descend_tolerance_m*100:.0f}) z={self.z:.2f}",
            throttle_duration_sec=1.0,
        )

        # Commit only once centred AND classified, so the symbol is read
        # from a frame where the bin is squarely underneath rather than
        # skewed at the edge of the image.
        if self.marker.centred(obs, self.bin_descend_tolerance_m) and obs.symbol:
            if obs.symbol == self.target_symbol:
                self._transition(State.BIN_DESCEND, f"{obs.symbol} is the target")
            else:
                self.rejected_symbols.add(obs.symbol)
                self._transition(State.BIN_REJECT,
                                 f"{obs.symbol} is not {self.target_symbol}")
            return

        if self.time_in_state > self.bin_approach_timeout:
            self.line.reset()
            self._transition(State.LINE_FOLLOW, "bin approach timed out")

    def _do_bin_reject(self, dt):
        """Step over the wrong bin and rejoin the path.

        Straight surge for a fixed time, which is enough to clear the
        1.0 m between the two bins, and the line follower picks the path
        back up on the far side. The timer doubles as the cooldown that
        stops the same bin re-triggering the approach immediately.
        """
        self._bin_view(dt)
        heave = self._depth_command(dt)
        self._publish(self.bin_reject_surge, 0.0, 0.0, heave)

        self.down_label = (
            f"[{self.state}] moving past, {self.time_in_state:.1f}"
            f"/{self.bin_reject_duration:.1f}s"
        )

        if self.time_in_state > self.bin_reject_duration:
            self.line.reset()
            self.bin_hits = 0
            self._transition(State.LINE_FOLLOW, "looking for the other bin")

    def _do_bin_descend(self, dt):
        obs = self._bin_view(dt)

        self._set_depth_target(self.bin_drop_z)
        heave = self._depth_command(dt)

        if obs is None:
            self.bin_lost += 1
            self._publish(0.0, 0.0, 0.0, heave)
            self.down_label = f"[{self.state}] bin lost {self.bin_lost}"
            if self.bin_lost > self.bin_lost_frames_max:
                self._abandon_bins("lost the bin while descending")
            return

        self.bin_lost = 0
        surge, sway, yaw, _ = self.marker.compute(obs, dt)
        self._publish(surge, sway, yaw, heave)

        depth_err = abs(self.z - self.bin_drop_z) if self.z is not None else 9.9
        err = math.hypot(obs.err_x_m, obs.err_y_m)
        self.down_label = (
            f"[{self.state}] z {self._z_text()} err {err * 100:.0f} cm"
        )

        if depth_err < 0.10 and self.marker.centred(obs, self.bin_centre_tolerance_m):
            self.bin_hold_elapsed = 0.0
            self._transition(State.BIN_HOLD, "at release altitude")

    def _do_bin_hold(self, dt):
        """Bleed off horizontal velocity before letting go.

        Commanding zero is not the same as being stopped - the hull
        coasts. The gate is the measured visual speed, taken from how fast
        the bin slides across the down camera, which is the only ground
        speed this vehicle can observe without a DVL.
        """
        obs = self._bin_view(dt)
        heave = self._depth_command(dt)

        if obs is None:
            self.bin_lost += 1
            self._publish(0.0, 0.0, 0.0, heave)
            if self.bin_lost > self.bin_lost_frames_max:
                self._abandon_bins("lost the bin while holding")
            return

        self.bin_lost = 0
        self.bin_hold_elapsed += dt

        # Keep trimming position, but gently - hard corrections here just
        # put velocity back in.
        surge, sway, yaw, _ = self.marker.compute(obs, dt)
        self._publish(0.35 * surge, 0.35 * sway, yaw, heave)

        speed = self.marker.speed_mps
        err = math.hypot(obs.err_x_m, obs.err_y_m)
        self.down_label = (
            f"[{self.state}] v={self._speed_text()} err {err * 100:.0f} cm "
            f"{self.bin_hold_elapsed:.1f}s"
        )

        settled = (speed is not None
                   and speed < self.bin_hold_speed_mps
                   and self.marker.centred(obs, self.bin_centre_tolerance_m)
                   and self.bin_hold_elapsed > self.bin_hold_min_time)

        if settled:
            self._transition(State.BIN_DROP, f"stopped at {speed * 100:.1f} cm/s")
            return

        if self.bin_hold_elapsed > self.bin_hold_timeout:
            # Release anyway rather than hover forever; log it so the miss
            # is attributable.
            self._transition(State.BIN_DROP, "hold timed out, releasing regardless")

    def _do_bin_drop(self, dt):
        obs = self._bin_view(dt)
        heave = self._depth_command(dt)
        self._publish(0.0, 0.0, 0.0, heave)

        # Fire exactly once per entry into this state, tracked by an
        # explicit flag rather than inferred from the clock.
        #
        # This used to test `time_in_state <= dt`, which is a one-frame
        # window that only holds if the tick's dt happens to be the same
        # value time_in_state was incremented by. dt comes from the wall
        # clock and jitters, so the window could close before the handler
        # ever saw it: measured runs entered BIN_DROP, dwelt there for 40
        # ticks, and released nothing - the vehicle then hovered over the
        # bin and eventually wandered off along the line, which is the
        # reported symptom.
        if not self._drop_fired:
            self._drop_fired = True
            self.drop_pub.publish(EmptyMsg())
            self.markers_dropped += 1

            err = math.hypot(obs.err_x_m, obs.err_y_m) if obs else float('nan')
            speed = self.marker.speed_mps if self.marker.speed_mps is not None else float('nan')
            entry = {
                "marker": self.markers_dropped,
                "centre_err_m": err,
                "speed_mps": speed,
                "true_xy": None,
            }
            if self.true_z is not None and self.true_xy is not None:
                entry["true_xy"] = self.true_xy
            self.drop_log.append(entry)

            self.get_logger().info(
                f"*** MARKER {self.markers_dropped}/{self.markers_carried} RELEASED "
                f"- centre error {err * 100:.1f} cm, ground speed {speed * 100:.1f} cm/s ***"
            )

        self.down_label = f"[{self.state}] released {self.markers_dropped}"

        if self.time_in_state > self.bin_drop_settle:
            if self.markers_dropped >= self.markers_carried:
                self._transition(State.BIN_DONE, "all markers away")
            else:
                self.bin_hold_elapsed = 0.0
                self._transition(State.BIN_HOLD, "re-centring for the next marker")

    def _do_bin_done(self, dt):
        self._bin_view(dt, annotate=False)

        self._set_depth_target(self.cruise_z)
        heave = self._depth_command(dt)

        # Creep forward while climbing rather than hovering in place. The
        # bin sits on top of the line, so holding station here leaves the
        # down camera staring at navy with no path to re-acquire.
        self._publish(self.bin_exit_surge, 0.0, 0.0, heave)

        self.down_label = f"[{self.state}] climbing back to cruise"

        if self.z is not None and abs(self.z - self.cruise_z) < 0.15:
            self.bins_done = True
            self.line.reset()
            self._log_drop_summary()
            self._transition(State.LINE_FOLLOW, "mission 4 complete")

    def _abandon_bins(self, reason):
        self.bins_done = True
        self._set_depth_target(self.cruise_z)
        self.line.reset()
        self.get_logger().warn(f"Giving up on the bins: {reason}")
        self._log_drop_summary()
        self._transition(State.LINE_FOLLOW, reason)

    def _log_drop_summary(self):
        if not self.drop_log:
            self.get_logger().info("No markers were released.")
            return
        for d in self.drop_log:
            self.get_logger().info(
                f"  marker {d['marker']}: centre error {d['centre_err_m'] * 100:.1f} cm, "
                f"release speed {d['speed_mps'] * 100:.1f} cm/s"
            )

    def _speed_text(self):
        s = self.marker.speed_mps
        return "n/a" if s is None else f"{s * 100:.1f}cm/s"

    # ------------------------------------------------------------------

    def _abandon_buoy(self, reason):
        self.buoy_attempts += 1

        # Blacklist the colour that just failed so the next attempt goes
        # after a different flower rather than retrying the one that is
        # not working.
        if self.buoy_target_color:
            self.buoys_touched.add(self.buoy_target_color)
            self.get_logger().warn(
                f"Skipping {self.buoy_target_color} buoy: {reason}")

        self.buoy.unlock()
        self.buoy_target_color = None
        self.buoy_hits = 0

        if (self.buoy_attempts >= self.max_buoy_attempts
                or len(self.buoys_touched) >= self.buoys_to_touch):
            self.buoy_done = True
            self.get_logger().warn(
                f"Ending mission 2 after {self.buoy_attempts} failed attempts.")

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

        # Estimator error against ground truth, so a drifting or noisy
        # vertical estimate is visible in the log rather than only showing
        # up as bad depth holding.
        err = ""
        if self.true_z is not None and self.z is not None:
            err = (f" | est err {100.0 * (self.z - self.true_z):+.1f} cm"
                   f" (peak {100.0 * self._est_err_peak:.1f})")

        print(
            f"[DEPTH] {reason}{reason and ' - ' if reason else ''}{depth_m:.2f} m / {depth_pct:.1f}% of pool depth"
            f" | surface=0 m | floor={pool_depth:.2f} m{err}",
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
