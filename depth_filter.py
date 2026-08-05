#!/usr/bin/env python3
"""Vertical state estimator: fuses Bar30 depth with IMU acceleration.

Why this exists. The Bar30 resolves about 2 mm and is polled at 20 Hz.
Differencing it naively to get a vertical rate gives

    sigma_v = sigma_z * sqrt(2) / dt = 0.002 * 1.414 / 0.05 = 0.057 m/s

and the depth loop's D term (kd = 8, four thrusters) turns that into
about 1.8 N of thrust chatter - larger than the 1.36 N of net buoyancy
the loop is trimming out in the first place. Note the perverse scaling:
polling the sensor *faster* makes the naive derivative worse, not
better.

Low-pass filtering the derivative trades that noise against phase lag,
and there is no setting that is both quiet and quick. Feeding the IMU's
vertical acceleration in as a control input breaks the tradeoff: the
prediction step carries the fast dynamics, and the slow, absolute, drift
free Bar30 only has to correct the accelerometer's bias.

Two states, [z, vz], both world frame with z up-positive.

If no acceleration is supplied the filter still works - it falls back to
a constant-velocity model with correspondingly larger process noise,
which is the classic alpha-beta tracker.
"""


class DepthFilter:

    def __init__(self,
                 meas_stddev=0.0035,
                 accel_stddev=0.05,
                 coast_accel_stddev=0.5):
        # Measurement noise: 3 mm of sensor noise and 2 mm of quantisation
        # (uniform, so q/sqrt(12)) add to roughly 3.5 mm.
        self.R = meas_stddev ** 2

        # Process noise, as the stddev of the acceleration error. With the
        # IMU driving the prediction this is just accelerometer error;
        # without it, it has to cover the vehicle's real accelerations.
        self.accel_stddev = accel_stddev
        self.coast_accel_stddev = coast_accel_stddev

        self.z = None
        self.vz = 0.0

        # Covariance [[zz, zv], [zv, vv]]
        self.p_zz = 1.0
        self.p_zv = 0.0
        self.p_vv = 1.0

    def reset(self, z=None):
        self.z = z
        self.vz = 0.0
        self.p_zz = 1.0
        self.p_zv = 0.0
        self.p_vv = 1.0

    def predict(self, dt, accel_up=None):
        """Propagate. accel_up is world-frame vertical accel in m/s^2."""
        if self.z is None or not (0.0 < dt < 0.5):
            return

        u = accel_up if accel_up is not None else 0.0
        sigma_a = self.accel_stddev if accel_up is not None else self.coast_accel_stddev

        # State: constant acceleration over the step.
        self.z += self.vz * dt + 0.5 * u * dt * dt
        self.vz += u * dt

        # Covariance: P = F P F' + Q, with F = [[1, dt], [0, 1]] and Q the
        # standard piecewise-white-noise-acceleration form.
        p_zz = self.p_zz + 2.0 * dt * self.p_zv + dt * dt * self.p_vv
        p_zv = self.p_zv + dt * self.p_vv
        p_vv = self.p_vv

        q = sigma_a ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        self.p_zz = p_zz + q * dt4 / 4.0
        self.p_zv = p_zv + q * dt3 / 2.0
        self.p_vv = p_vv + q * dt2

    def update(self, z_meas):
        """Fold in one Bar30-derived depth measurement (world z)."""
        if self.z is None:
            # First measurement initialises the filter outright; there is
            # nothing to blend it with yet.
            self.z = z_meas
            self.vz = 0.0
            return

        innovation = z_meas - self.z
        s = self.p_zz + self.R

        k_z = self.p_zz / s
        k_v = self.p_zv / s

        self.z += k_z * innovation
        self.vz += k_v * innovation

        p_zz = self.p_zz
        p_zv = self.p_zv

        self.p_zz = (1.0 - k_z) * p_zz
        self.p_zv = (1.0 - k_z) * p_zv
        self.p_vv = self.p_vv - k_v * p_zv

    @property
    def ready(self):
        return self.z is not None


def vertical_accel_from_imu(ax, ay, az, level_tolerance=0.30):
    """World-frame vertical acceleration from the raw IMU reading.

    The IMU is mounted with <pose>0 0 0 3.142 0 0</pose>, i.e. rolled 180
    degrees into the ArduPilot FRD convention, so its z axis points DOWN.
    Gazebo reports specific force, so a stationary vehicle reads
    az = -9.81 on that axis. Undoing both gives

        a_up = -az - 9.81

    which is zero at rest and positive when the vehicle accelerates up.

    That scalar form assumes the vehicle is roughly level. It is: the
    centre of buoyancy sits 49 mm above the centre of mass, which is a
    passive righting moment, and nothing commands roll or pitch. Rather
    than trust that blindly, tilt is measured straight off the specific
    force vector - no reliance on the quaternion's frame - and the
    estimate is refused when the vehicle is tipped past the tolerance,
    letting the filter coast instead of swallowing a bad input.
    """
    lateral = (ax * ax + ay * ay) ** 0.5
    vertical = abs(az)

    if vertical < 1e-6:
        return None
    if lateral / vertical > level_tolerance:
        return None

    return -az - 9.81
