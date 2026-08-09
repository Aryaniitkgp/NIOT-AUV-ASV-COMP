#!/usr/bin/env python3
"""Mission 4 - marker dropping into the O / X bins.

Perception runs on the DOWN camera, whose axes come from the model SDF
pose (0 0 -0.05 0 1.571 0):

    image up    -> vehicle forward (+x)
    image right -> vehicle starboard (-y, because body +y is left)

so unlike the buoy servo there is no yaw in this loop at all. A bin
sitting low in the frame is behind the vehicle and is closed on with
NEGATIVE surge; a bin to the right of centre is to starboard and is
closed on with NEGATIVE sway. Both errors map onto translation, which is
what makes this a station-keeping problem rather than a pointing one.

Two-stage detection:

  1. The bin is a navy square. Found by HSV plus a squareness gate,
     which is what keeps the blue L-bar - almost the same hue - out of
     the results, since a rod is not square.

  2. The symbol is white, and is only ever looked for INSIDE a bin that
     has already been found. That restriction is what makes the white
     threshold easy: within the bin the only two things present are navy
     floor (V~137) and white paint (V~218).

Classification uses two independent cues so a single bad frame cannot
flip the decision:

    circularity   O = 1.00,  X = 0.40   (contour area / enclosing circle)
    topology      O has a child contour, X has none

Range and scale come from depth, not from apparent size. The bin floor
is at a known height and the vehicle knows its own depth, so altitude is
known outright and

    metres_per_pixel = altitude / focal_length

converts pixel error straight into metres. That is far better
conditioned than inferring range from the symbol's apparent size.
"""

import math

import cv2 as cv
import numpy as np

from line_follow import draw_overlay


# Navy of the bins, through the water tint. The upper value bound is what
# separates them from the L-bar's brighter blue (V~218), which sits at
# almost the same hue.
BIN_LOWER = np.array([100, 120, 60])
BIN_UPPER = np.array([125, 255, 190])

# White paint. Only applied inside a bin, where the alternative is navy.
#
# These bounds are measured off an actual rendered frame, not derived by
# alpha-blending the material colour. The rendered scene is far darker
# than that model predicts - roughly 2-2.5x - because the arena has one
# directional light and the floor sits under a metre of translucent water:
#
#     surface        modelled HSV     ACTUAL rendered HSV
#     white symbol   (106, 47, 226)   (  0,   0,  97)
#     navy bin       (111,188, 137)   (112, 131,  68)
#     grey floor     (108, 70, 168)   (120,   6,  80)
#
# The old V floor of 165 rejected every white pixel in the frame, so the
# symbol was never found and the classifier returned NONE forever.
#
# White renders as a near-pure grey (S~0), so saturation and value are
# what separate it from the navy floor of the bin (S=131, V=68) - the
# only other thing inside the region this mask is ever applied to.
SYMBOL_LOWER = np.array([0, 0, 88])
SYMBOL_UPPER = np.array([180, 60, 255])


class BinObservation:
    """One bin seen from above."""

    def __init__(self, u, v, width_px, area_px, symbol, circularity,
                 has_hole, ex, ey, err_x_m, err_y_m, tracked_on):
        self.u = u
        self.v = v
        self.width_px = width_px
        self.area_px = area_px
        self.symbol = symbol            # 'O', 'X' or None
        self.circularity = circularity
        self.has_hole = has_hole
        self.ex = ex                    # normalised, +1 at the right edge
        self.ey = ey                    # normalised, +1 at the bottom edge
        self.err_x_m = err_x_m          # forward error, metres (+ = bin ahead)
        self.err_y_m = err_y_m          # port error, metres  (+ = bin to port)
        self.tracked_on = tracked_on    # 'symbol' or 'bin'


class MarkerDropController:
    """Centres the vehicle over a bin and decides when to release."""

    def __init__(self, focal_px=554.4, bin_floor_z=0.027, bin_rim_z=0.30):
        self.focal_px = focal_px
        self.bin_floor_z = bin_floor_z
        self.camera_offset_z = 0.05     # down camera sits below the origin

        # The bin is an open box with 0.3 m walls, so from directly above
        # the detector outlines the RIM, not the floor. The rim is 0.27 m
        # closer to the camera and therefore magnified: at cruise depth a
        # 0.64 m bin measures ~546 px across rather than ~384.
        #
        # Scaling pixel error with the floor altitude therefore
        # over-estimates every distance by about 1.4x, and the centring
        # gate never closes. Symbol tracking still uses the floor scale,
        # because the painted symbol really is on the floor.
        self.bin_rim_z = bin_rim_z

        self.kernel = np.ones((5, 5), np.uint8)

        # Detection gates
        self.min_bin_area_px = 4000.0
        self.max_aspect_ratio = 1.45    # a square; the L-bar rod is not
        self.min_extent = 0.55          # filled square, not a ring of walls
        self.min_symbol_area_px = 900.0

        # Classification
        self.circularity_split = 0.70   # O ~1.00, X ~0.40

        # Station-keeping gains. Pure translation, no yaw.
        self.kp_surge = 6.0
        self.kd_surge = 1.6
        self.kp_sway = 6.0
        self.kd_sway = 1.6
        self.max_surge = 4.0
        self.max_sway = 4.0

        # Vertical is cascaded through the depth loop, as everywhere else.
        self.max_heave_rate = 0.25

        self.prev_ex = 0.0
        self.prev_ey = 0.0

        # Visual horizontal speed, from how fast the bin slides across the
        # frame. The vehicle has no DVL and no horizontal odometry, so this
        # is the only measurement of "am I actually stopped" available -
        # and it is exactly the down-camera visual odometry that replaces
        # the optical-flow module.
        self.prev_u = None
        self.prev_v = None
        self.speed_mps = None
        self._speed_filt = 0.0
        self._speed_samples = 0

        # Frames of motion history before the speed estimate is trusted.
        # At 20 Hz this is a third of a second, enough for the 0.7/0.3
        # filter to converge from its zero initial state.
        self.speed_warmup_samples = 7

        self.lost_frames = 0

    def reset(self):
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_u = None
        self.prev_v = None
        self.speed_mps = None
        self._speed_filt = 0.0
        self._speed_samples = 0
        self.lost_frames = 0

    def altitude(self, vehicle_z):
        """Height of the down camera above the bin floor."""
        return max(0.05, vehicle_z - self.camera_offset_z - self.bin_floor_z)

    def rim_altitude(self, vehicle_z):
        """Height of the down camera above the bin rim."""
        return max(0.05, vehicle_z - self.camera_offset_z - self.bin_rim_z)

    def metres_per_pixel(self, vehicle_z, on_rim=False):
        """Pixel scale at the plane actually being measured.

        Using the floor scale for a feature that lies on the rim inflates
        every distance by the ratio of the two altitudes - about 1.4x at
        cruise depth - which is enough to stop a centring gate ever
        closing.
        """
        alt = self.rim_altitude(vehicle_z) if on_rim else self.altitude(vehicle_z)
        return alt / self.focal_px

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    # ------------------------------------------------------------------
    # Detection

    def _classify(self, frame, x, y, w, h):
        """Look for a white symbol inside one bin. Returns (symbol, circ, hole)."""
        pad = 4
        x0, y0 = max(0, x + pad), max(0, y + pad)
        x1, y1 = min(frame.shape[1], x + w - pad), min(frame.shape[0], y + h - pad)
        if x1 - x0 < 20 or y1 - y0 < 20:
            return None, 0.0, False

        roi = frame[y0:y1, x0:x1]
        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, SYMBOL_LOWER, SYMBOL_UPPER)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=1)

        # CCOMP gives a two-level hierarchy, so a symbol's holes appear as
        # children. That is the topology cue.
        contours, hierarchy = cv.findContours(mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
        if not contours or hierarchy is None:
            return None, 0.0, False

        best_i, best_area = -1, 0.0
        for i, c in enumerate(contours):
            if hierarchy[0][i][3] != -1:        # skip holes themselves
                continue
            a = cv.contourArea(c)
            if a > best_area:
                best_i, best_area = i, a

        if best_i < 0 or best_area < self.min_symbol_area_px:
            return None, 0.0, False

        contour = contours[best_i]
        (_, _), radius = cv.minEnclosingCircle(contour)
        if radius < 5:
            return None, 0.0, False

        circularity = best_area / (math.pi * radius * radius)
        has_hole = hierarchy[0][best_i][2] != -1

        # Circularity decides; topology breaks ties and is logged so a
        # disagreement between the two cues is visible rather than silent.
        symbol = 'O' if circularity >= self.circularity_split else 'X'
        if has_hole and circularity < self.circularity_split:
            symbol = 'O'

        return symbol, circularity, has_hole

    def _symbol_centroid(self, frame, x, y, w, h):
        """Centroid of the white symbol, in full-frame coordinates."""
        pad = 4
        x0, y0 = max(0, x + pad), max(0, y + pad)
        x1, y1 = min(frame.shape[1], x + w - pad), min(frame.shape[0], y + h - pad)
        if x1 - x0 < 20 or y1 - y0 < 20:
            return None

        roi = frame[y0:y1, x0:x1]
        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, SYMBOL_LOWER, SYMBOL_UPPER)
        m = cv.moments(mask, binaryImage=True)
        if m["m00"] < self.min_symbol_area_px:
            return None
        return (x0 + m["m10"] / m["m00"], y0 + m["m01"] / m["m00"])

    def detect(self, frame, vehicle_z):
        """Return (list of BinObservation sorted nearest-first, mask)."""
        height, width = frame.shape[:2]
        u0, v0 = width / 2.0, height / 2.0
        mpp = self.metres_per_pixel(vehicle_z)                 # floor plane
        mpp_rim = self.metres_per_pixel(vehicle_z, on_rim=True)  # rim plane

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, BIN_LOWER, BIN_UPPER)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        found = []

        for contour in contours:
            area = cv.contourArea(contour)
            if area < self.min_bin_area_px:
                continue

            x, y, w, h = cv.boundingRect(contour)
            if w == 0 or h == 0:
                continue

            # Square, and solid. The L-bar is a long thin rod and fails
            # the first test; a stray rim fragment fails the second.
            aspect = max(w / float(h), h / float(w))
            if aspect > self.max_aspect_ratio:
                continue
            if area / float(w * h) < self.min_extent:
                continue

            symbol, circularity, has_hole = self._classify(frame, x, y, w, h)

            # Servo on the symbol when it is resolvable. Close to the
            # floor the bin outline runs off the edge of the frame and its
            # centroid becomes badly biased, but the symbol stays whole.
            centre = self._symbol_centroid(frame, x, y, w, h)
            if centre is not None:
                cu, cv_ = centre
                tracked_on = 'symbol'
            else:
                cu, cv_ = x + w / 2.0, y + h / 2.0
                tracked_on = 'bin'

            # Scale at the plane the tracked feature actually lies on: the
            # painted symbol is on the floor, the bin outline is the rim.
            scale = mpp if tracked_on == 'symbol' else mpp_rim

            # image up = forward, image right = starboard.
            err_x_m = (v0 - cv_) * scale
            err_y_m = (u0 - cu) * scale

            found.append(BinObservation(
                u=cu, v=cv_, width_px=float(w), area_px=area,
                symbol=symbol, circularity=circularity, has_hole=has_hole,
                ex=(cu - u0) / u0, ey=(cv_ - v0) / v0,
                err_x_m=err_x_m, err_y_m=err_y_m,
                tracked_on=tracked_on,
            ))

        # Nearest first: the bin whose centre is closest to the frame
        # centre is the one underneath us.
        found.sort(key=lambda b: b.ex * b.ex + b.ey * b.ey)
        return found, mask

    # ------------------------------------------------------------------
    # Control

    def update_speed(self, obs, vehicle_z, dt):
        """Vehicle ground speed from how fast the bin slides across frame."""
        if obs is None or not (1e-3 < dt < 0.5):
            self.prev_u = None if obs is None else obs.u
            self.prev_v = None if obs is None else obs.v
            return

        if self.prev_u is not None:
            # Same plane the centroid was measured on, or the speed gate
            # inherits the identical 1.4x scale error.
            mpp = self.metres_per_pixel(
                vehicle_z, on_rim=(obs.tracked_on == 'bin'))
            du = (obs.u - self.prev_u) * mpp / dt
            dv = (obs.v - self.prev_v) * mpp / dt
            raw = math.hypot(du, dv)
            # The centroid is noisy frame to frame, so smooth before the
            # release gate reads it.
            self._speed_filt = 0.7 * self._speed_filt + 0.3 * raw
            self._speed_samples += 1

            # Do not publish a speed until the filter has actually seen
            # some motion history.
            #
            # The filter starts at 0.0, so its first output after a reset
            # is near zero no matter how fast the vehicle is travelling.
            # BIN_HOLD reads that as "stopped" and releases immediately -
            # measured releasing marker 2 at a reported 0.0 cm/s while
            # ground truth showed 42-57 cm/s. Two markers still landed in
            # the bin, but by luck rather than by the gate working.
            if self._speed_samples >= self.speed_warmup_samples:
                self.speed_mps = self._speed_filt

        self.prev_u = obs.u
        self.prev_v = obs.v

    def compute(self, obs, dt):
        """Station-keeping command. Returns (surge, sway, yaw, heave_rate)."""
        if 1e-3 < dt < 0.5:
            d_ex = (obs.ex - self.prev_ex) / dt
            d_ey = (obs.ey - self.prev_ey) / dt
        else:
            d_ex = d_ey = 0.0

        self.prev_ex = obs.ex
        self.prev_ey = obs.ey

        # Bin low in frame (ey > 0) is aft -> back up. Bin right of centre
        # (ex > 0) is to starboard -> sway starboard, which is negative.
        surge = self._clamp(-(self.kp_surge * obs.ey + self.kd_surge * d_ey), self.max_surge)
        sway = self._clamp(-(self.kp_sway * obs.ex + self.kd_sway * d_ex), self.max_sway)

        return surge, sway, 0.0, 0.0

    def centred(self, obs, tolerance_m):
        return math.hypot(obs.err_x_m, obs.err_y_m) <= tolerance_m

    # ------------------------------------------------------------------

    def annotate(self, frame, bins, target_symbol=None):
        height, width = frame.shape[:2]
        draw_overlay(frame, width // 2, height // 2)

        for i, b in enumerate(bins):
            is_target = (target_symbol is not None and b.symbol == target_symbol)
            color = (0, 255, 0) if is_target else (0, 165, 255)
            cv.circle(frame, (int(b.u), int(b.v)), 10, color, 2)
            cv.line(frame, (width // 2, height // 2), (int(b.u), int(b.v)), color, 1)
            label = f"{b.symbol or '?'} c={b.circularity:.2f}{'/hole' if b.has_hole else ''}"
            cv.putText(frame, label, (int(b.u) + 14, int(b.v)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
