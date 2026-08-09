#!/usr/bin/env python3
"""Mission 6 - surfacing inside the octagon.

The octagon is a ring of blue bars floating at the free surface with
yellow marker floats at alternate vertices. The vehicle has to come up
INSIDE it, with no part outside the structure.

Two facts from the arena shape everything here.

1. The path terminates at the octagon centre. left_2 ends at (8, -5) and
   right_2 at (8, +5), which are exactly the two octagon centres. So
   arrival is not a search problem: following the line until it runs out
   puts the vehicle where it needs to be, and the existing lost-line
   logic is the arrival signal.

2. There is no upward-facing camera, and the ring is ABOVE the vehicle
   at cruise depth. Measured on real frames:

       range   ring elevation   in frame? (half-FOV 23.4 deg)
        6.0 m       13.6 deg        yes
        4.0 m       19.9 deg        yes, but washed out
        3.0 m       25.8 deg        NO - above the frame
        2.0 m       35.9 deg        NO

   At 4 m the ring renders at V=99 against a V=225 surface band, and no
   yellow float pixels survive at all. Confirming arrival visually on the
   approach is therefore not possible - it was tried and measured.

What DOES work is checking containment during the ascent. Once the
vehicle rises to about z = 2.2 the ring comes level with the camera and
separates cleanly:

       feature        H    S    V
       surface band  101   86  225
       BLUE ring     112  157  104     <- separable on V from surface,
       grey floor    120    6   80        and on S from the floor
       yellow float   25  170  117

With the vehicle at the centre the ring spans the full frame width. That
gives a direct inside/outside test that needs no position estimate:
blue in most column bins means the ring surrounds the vehicle; blue
clustered on one bearing means it is off to one side and the vehicle is
outside.
"""

import numpy as np
import cv2 as cv


# Measured off rendered frames, not derived from SDF material colours.
# Deriving from materials has failed twice in this project - the rendered
# scene is 2-2.5x darker than an alpha blend of the material predicts.
#
# The value ceiling is what rejects the bright surface band (V=225); the
# saturation floor is what rejects the grey floor and walls (S=6).
RING_LOWER = np.array([104, 100, 55])
RING_UPPER = np.array([128, 255, 175])

# Yellow marker floats. Only visible once level with the ring, so these
# are a supporting cue, never a gate.
FLOAT_LOWER = np.array([18, 120, 70])
FLOAT_UPPER = np.array([34, 255, 200])


class OctagonView:
    """One frame's worth of ring geometry."""

    def __init__(self, ring_px, coverage, bins_hit, n_bins,
                 centroid_u, spread, floats_px, peak_row_px=0, rows=0):
        self.ring_px = ring_px          # blue pixels found
        self.coverage = coverage        # fraction of column bins holding ring
        self.bins_hit = bins_hit
        self.n_bins = n_bins
        self.centroid_u = centroid_u    # mean x of ring pixels, or None
        self.spread = spread            # normalised spread of ring across x
        self.floats_px = floats_px
        self.peak_row_px = peak_row_px  # pixels in the densest single row
        self.rows = rows                # rows containing any ring pixel

    @property
    def surrounded(self):
        """True when the vehicle is INSIDE the ring.

        Column coverage alone is not enough. From outside, the far arc of
        the ring also spans the full frame width, so both cases score 6-8
        bins. Measured:

                            inside    outside @4m
            ring pixels       6573           2998
            densest row        600            265
            rows occupied       13             29

        Inside, the ring passes close on every side, so it is bright and
        concentrated. Outside, only the distant arc is visible and it is
        much dimmer.

        Density of the densest row is the discriminator, and it is the
        ONLY one used. Absolute y moves with vehicle depth, and row count
        is contaminated by vehicle motion: those numbers above come from
        a stationary, level frame, but during a real ascent the vehicle
        is rising at ~0.18 m/s and rolling slightly, which smeared the
        ring across 43 rows. A run that surfaced correctly INSIDE the
        ring (2.58 m from centre, apothem 3.26) was rejected by a
        rows <= 22 gate despite peak_row_px = 568.

        peak_row_px separates by 5.8x - 568 inside against 98 outside -
        which is margin enough without help.
        """
        return self.coverage >= 0.75 and self.peak_row_px >= 300


class OctagonDetector:
    """Finds the surfacing ring and judges whether the vehicle is inside."""

    def __init__(self):
        self.kernel = np.ones((5, 5), np.uint8)

        # Minimum blue to call the ring present at all. The measured frame
        # at ascent height had ~6500 px, so this is a wide margin below.
        self.min_ring_px = 600

        # Column bins for the coverage test. Eight matches the octagon's
        # own eight segments and is coarse enough to tolerate gaps where a
        # vertex or float interrupts a bar.
        self.n_bins = 8
        self.min_bin_px = 40

    def detect(self, frame):
        """Return (OctagonView, ring mask)."""
        height, width = frame.shape[:2]
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        ring = cv.inRange(hsv, RING_LOWER, RING_UPPER)
        ring = cv.morphologyEx(ring, cv.MORPH_OPEN, self.kernel, iterations=1)
        ring = cv.morphologyEx(ring, cv.MORPH_CLOSE, self.kernel, iterations=2)

        floats = cv.inRange(hsv, FLOAT_LOWER, FLOAT_UPPER)

        ys, xs = np.nonzero(ring)
        ring_px = int(xs.size)

        if ring_px < self.min_ring_px:
            return OctagonView(ring_px, 0.0, 0, self.n_bins,
                               None, 0.0, int(floats.sum() // 255)), ring

        row_counts = np.bincount(ys, minlength=height)
        peak_row_px = int(row_counts.max())
        rows = int((row_counts > 0).sum())

        # Coverage: how much of the frame width the ring reaches across.
        # This is the containment test. It deliberately uses column bins
        # rather than a fitted circle - the ring is a near-horizontal band
        # at this geometry, not a circle in the image, so circle fitting
        # would be meaningless.
        edges = np.linspace(0, width, self.n_bins + 1)
        counts, _ = np.histogram(xs, bins=edges)
        bins_hit = int((counts >= self.min_bin_px).sum())
        coverage = bins_hit / float(self.n_bins)

        centroid_u = float(xs.mean())
        spread = float(xs.std()) / (width / 2.0)

        return OctagonView(ring_px, coverage, bins_hit, self.n_bins,
                           centroid_u, spread,
                           int(floats.sum() // 255),
                           peak_row_px, rows), ring

    def centering_error(self, view, width):
        """Normalised lateral error toward the middle of the visible ring.

        Only meaningful when the ring is NOT surrounding the vehicle: it
        points back toward the structure so an off-centre vehicle can
        translate into it. When surrounded, the centroid sits near the
        middle by construction and this returns roughly zero.
        """
        if view.centroid_u is None:
            return 0.0
        return (view.centroid_u - width / 2.0) / (width / 2.0)

    def annotate(self, frame, view):
        height, width = frame.shape[:2]

        cv.line(frame, (width // 2, 0), (width // 2, height), (0, 0, 255), 1)

        # Column bin occupancy strip along the bottom.
        for b in range(view.n_bins):
            x0 = int(b * width / view.n_bins)
            x1 = int((b + 1) * width / view.n_bins)
            hit = b < view.bins_hit
            colour = (0, 200, 0) if view.surrounded else (0, 165, 255)
            cv.rectangle(frame, (x0 + 2, height - 18), (x1 - 2, height - 4),
                         colour if hit else (60, 60, 60), -1)

        if view.centroid_u is not None:
            cv.circle(frame, (int(view.centroid_u), height // 2), 7,
                      (255, 0, 255), -1)
