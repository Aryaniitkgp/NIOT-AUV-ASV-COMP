#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class ThrusterMixer(Node):
    """Maps body-frame commands onto the 8 BlueROV2 thrusters.

    Body frame is FLU: +x forward, +y left, +z up.

    The four horizontal thrusters are vectored at 45 degrees, so their
    contributions were measured straight off the model SDF (link poses +
    joint axis 0 0 -1):

        thruster   surge     sway     yaw(CCW)
        T1 FR      -0.707   -0.707   -0.164
        T2 FL      -0.707   +0.707   +0.164
        T3 RR      +0.707   -0.707   +0.164
        T4 RL      +0.707   +0.707   -0.164

    Inverting that gives the sign pattern used below. Driving all four
    with the same sign produces zero net force, which is why an even
    +s on every thruster leaves the vehicle sitting still.

    T5-T8 are vertical and point down for positive thrust, so they get
    negated to make +heave mean up.
    """

    def __init__(self):

        super().__init__("thruster_mixer")

        # Desired vehicle motion
        self.surge = 0.0
        self.sway = 0.0
        self.yaw = 0.0
        self.heave = 0.0

        # Publishers
        self.thruster_pub = []

        for i in range(1, 9):
            pub = self.create_publisher(
                Float64,
                f"/bluerov2/thruster{i}/cmd_thrust",
                10,
            )
            self.thruster_pub.append(pub)

        # Subscribers
        self.create_subscription(
            Float64,
            "/cmd_surge",
            self.surge_callback,
            10,
        )

        self.create_subscription(
            Float64,
            "/cmd_sway",
            self.sway_callback,
            10,
        )

        self.create_subscription(
            Float64,
            "/cmd_yaw",
            self.yaw_callback,
            10,
        )

        self.create_subscription(
            Float64,
            "/cmd_heave",
            self.heave_callback,
            10,
        )

        # 20 Hz update
        self.timer = self.create_timer(
            0.05,
            self.publish_thrusters,
        )

        self.max_thrust = 40.0

        # Zero the thrusters if the controller stops publishing, otherwise
        # the last command keeps running after the node above it dies.
        self.command_timeout = 1.0
        self.last_command_time = self.get_clock().now()

        self.get_logger().info("Thruster Mixer Started")

    #################################################

    def surge_callback(self, msg):
        self.surge = msg.data
        self.last_command_time = self.get_clock().now()

    def sway_callback(self, msg):
        self.sway = msg.data
        self.last_command_time = self.get_clock().now()

    def yaw_callback(self, msg):
        self.yaw = msg.data
        self.last_command_time = self.get_clock().now()

    def heave_callback(self, msg):
        self.heave = msg.data
        self.last_command_time = self.get_clock().now()

    #################################################

    def scale_to_limit(self, values):
        """Scale the whole set down instead of clipping one thruster.

        Clipping a single thruster changes the direction of the resulting
        force, so a hard turn would quietly turn into a turn plus drift.
        """
        peak = max(abs(v) for v in values)

        if peak > self.max_thrust:
            factor = self.max_thrust / peak
            return [v * factor for v in values]

        return values

    #################################################

    def publish_thrusters(self):

        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9

        if age > self.command_timeout:
            s = w = y = h = 0.0
        else:
            s = self.surge
            w = self.sway
            y = self.yaw
            h = self.heave

        # Horizontal thrusters (vectored)
        #
        # T1 Front Right
        # T2 Front Left
        # T3 Rear Right
        # T4 Rear Left

        t1 = -s - w - y
        t2 = -s + w + y
        t3 = s - w + y
        t4 = s + w - y

        # Vertical thrusters. Positive thrust pushes down, so negate
        # to keep +heave meaning up.

        t5 = -h
        t6 = -h
        t7 = -h
        t8 = -h

        values = self.scale_to_limit([t1, t2, t3, t4, t5, t6, t7, t8])

        for i in range(8):

            msg = Float64()
            msg.data = float(values[i])

            self.thruster_pub[i].publish(msg)

        self.get_logger().info(
            " ".join(f"T{i + 1}={values[i]:.2f}" for i in range(8)),
            throttle_duration_sec=1.0,
        )


def main():

    rclpy.init()

    node = ThrusterMixer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
