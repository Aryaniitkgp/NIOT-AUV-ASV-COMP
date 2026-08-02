#!/usr/bin/env python3
"""Depth hold for the BlueROV2.

The hull displaces 0.457 x 0.575 x 0.05 m of water, i.e. 13.14 kg against
a 13.0 kg mass, so the vehicle is about 1.4 N positively buoyant and will
drift up to the surface on its own. Nothing in the mission works without
an active depth loop, which is why every state runs this in parallel.

Drag is purely quadratic (zWabsW = -73.2), so there is no linear damping
to lean on and the D term has to come from a measured rate rather than
from differentiating a noisy setpoint error. The I term exists to trim
out the residual buoyancy without hard-coding it.

The four vertical thrusters each receive -heave and each push down for
positive thrust, so a heave command of h produces 4h newtons upward.
"""


class DepthController:

    def __init__(self, kp=10.0, ki=2.0, kd=8.0, max_heave=10.0, i_limit=4.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_heave = max_heave
        self.i_limit = i_limit

        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, z, z_target, z_rate, dt):
        """Positive output means "go up". z is world z, up positive."""
        error = z_target - z

        if 1e-3 < dt < 0.5:
            self.integral += error * dt
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        # Rate damping on the measurement, not on the error, so a setpoint
        # step does not kick the output.
        command = self.kp * error + self.ki * self.integral - self.kd * z_rate

        return max(-self.max_heave, min(self.max_heave, command))
