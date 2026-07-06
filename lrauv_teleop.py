#!/usr/bin/env python3
"""
Keyboard teleop for the my_lrauv AUV.

Publishes to the ros_gz_bridge command topics:
    /lrauv/cmd_thrust    std_msgs/Float64   propeller force  (+fwd / -rev)
    /lrauv/cmd_rudder    std_msgs/Float64   vertical fin angle in rad (yaw / turn)
    /lrauv/cmd_elevator  std_msgs/Float64   horizontal fin angle in rad (pitch / dive)

Controls (hold-and-release style; each key nudges a setpoint that is
continuously re-published, so the vehicle keeps doing the last command):

    w / s : thrust  +/-        (forward / reverse)
    a / d : rudder  left/right (turn)
    i / k : elevator up/down   (dive / surface)
    space : STOP everything    (all setpoints -> 0)
    q     : quit

Run (in a terminal that has ROS 2 + the bridge running):
    source /opt/ros/humble/setup.bash
    python3 lrauv_teleop.py
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Tunables --------------------------------------------------------------
THRUST_STEP = 2.0        # N per key press
THRUST_MAX = 30.0
FIN_STEP = 0.05          # rad per key press
FIN_MAX = 0.5            # ~28 deg, keep within joint limits
PUBLISH_HZ = 20.0        # how often setpoints are re-sent
# -----------------------------------------------------------------------

HELP = __doc__


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LrauvTeleop(Node):
    def __init__(self):
        super().__init__("lrauv_teleop")
        self.pub_thrust = self.create_publisher(Float64, "/lrauv/cmd_thrust", 10)
        self.pub_rudder = self.create_publisher(Float64, "/lrauv/cmd_rudder", 10)
        self.pub_elev = self.create_publisher(Float64, "/lrauv/cmd_elevator", 10)

        self.thrust = 0.0
        self.rudder = 0.0
        self.elevator = 0.0

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def _publish(self):
        self.pub_thrust.publish(Float64(data=self.thrust))
        self.pub_rudder.publish(Float64(data=self.rudder))
        self.pub_elev.publish(Float64(data=self.elevator))

    def handle_key(self, key):
        if key == "w":
            self.thrust = clamp(self.thrust + THRUST_STEP, -THRUST_MAX, THRUST_MAX)
        elif key == "s":
            self.thrust = clamp(self.thrust - THRUST_STEP, -THRUST_MAX, THRUST_MAX)
        elif key == "a":
            self.rudder = clamp(self.rudder + FIN_STEP, -FIN_MAX, FIN_MAX)
        elif key == "d":
            self.rudder = clamp(self.rudder - FIN_STEP, -FIN_MAX, FIN_MAX)
        elif key == "i":
            self.elevator = clamp(self.elevator + FIN_STEP, -FIN_MAX, FIN_MAX)
        elif key == "k":
            self.elevator = clamp(self.elevator - FIN_STEP, -FIN_MAX, FIN_MAX)
        elif key == " ":
            self.thrust = self.rudder = self.elevator = 0.0
        else:
            return

        self.get_logger().info(
            f"thrust={self.thrust:+.1f} N  rudder={self.rudder:+.2f} rad  "
            f"elevator={self.elevator:+.2f} rad"
        )


def get_key(timeout):
    """Non-blocking single-char read from stdin."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = LrauvTeleop()
    print(HELP)
    print(">>> teleop ready. focus this terminal and press keys.\n")
    try:
        while rclpy.ok():
            key = get_key(1.0 / PUBLISH_HZ)
            if key == "q":
                break
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # leave the vehicle stopped
        node.thrust = node.rudder = node.elevator = 0.0
        node._publish()
        node.destroy_node()
        rclpy.shutdown()
        print("\nstopped, bye.")


if __name__ == "__main__":
    main()
