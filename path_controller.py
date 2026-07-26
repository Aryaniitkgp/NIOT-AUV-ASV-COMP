#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class PathController(Node):

    def __init__(self):

        super().__init__("path_controller")

        self.subscription = self.create_subscription(
            Float64,
            "/path_error",
            self.path_callback,
            10,
        )

        self.surge_pub = self.create_publisher(
            Float64,
            "/cmd_surge",
            10,
        )

        self.yaw_pub = self.create_publisher(
            Float64,
            "/cmd_yaw",
            10,
        )

        #################################

        self.forward_thrust = 20.0

        self.Kp = 15.0

        #################################

        self.get_logger().info("Path Controller Started")

    def path_callback(self, msg):

        error = msg.data

        yaw = self.Kp * error

        surge_msg = Float64()
        surge_msg.data = self.forward_thrust

        yaw_msg = Float64()
        yaw_msg.data = yaw

        self.surge_pub.publish(surge_msg)
        self.yaw_pub.publish(yaw_msg)

        self.get_logger().info(
            f"Error={error:.3f}  Surge={self.forward_thrust:.2f}  Yaw={yaw:.2f}"
        )


def main():

    rclpy.init()

    node = PathController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()