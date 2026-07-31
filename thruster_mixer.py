#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class ThrusterMixer(Node):

    def __init__(self):

        super().__init__("thruster_mixer")

        # Desired vehicle motion
        self.surge = 0.0
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

        self.get_logger().info("Thruster Mixer Started")

    #################################################

    def surge_callback(self, msg):
        self.surge = msg.data

    def yaw_callback(self, msg):
        self.yaw = msg.data

    def heave_callback(self, msg):
        self.heave = msg.data

    #################################################

    def clamp(self, value):

        if value > self.max_thrust:
            return self.max_thrust

        if value < -self.max_thrust:
            return -self.max_thrust

        return value

    #################################################

    def publish_thrusters(self):

        s = self.surge
        y = self.yaw
        h = self.heave

        # Horizontal thrusters
        #
        # T1 Front Left
        # T2 Front Right
        # T3 Rear Left
        # T4 Rear Right

        t1 = self.clamp(s - y)
        t2 = self.clamp(s + y)
        t3 = self.clamp(s - y)
        t4 = self.clamp(s + y)

        # Vertical thrusters

        t5 = self.clamp(h)
        t6 = self.clamp(h)
        t7 = self.clamp(h)
        t8 = self.clamp(h)

        values = [t1, t2, t3, t4, t5, t6, t7, t8]

        for i in range(8):

            msg = Float64()
            msg.data = values[i]

            self.thruster_pub[i].publish(msg)

        self.get_logger().info(
            f"T1={t1:.2f} "
            f"T2={t2:.2f} "
            f"T3={t3:.2f} "
            f"T4={t4:.2f} "
            f"T5={t5:.2f} "
            f"T6={t6:.2f} "
            f"T7={t7:.2f} "
            f"T8={t8:.2f}"
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