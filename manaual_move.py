#!/usr/bin/env python3
"""Keyboard teleop for the BlueROV2 AUV.

This script commands the vehicle with direct thruster outputs so it can move:

	w / s : front / back
	a / d : left / right
	q / e : yaw left / yaw right
	r / f : deep / up
	space : stop
	x     : quit

By default it publishes to the ROS topics used by bluerov2_native_bridge.py:
	/bluerov2/thruster1/cmd_thrust ... /bluerov2/thruster8/cmd_thrust

Use --topic-style yaml if you want the alternate topic layout from
bluerov2_bridge.yaml:
	/bluerov2/cmd_thrust/t1 ... /bluerov2/cmd_thrust/t8
"""

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


PUBLISH_HZ = 20.0
DEFAULT_STEP = 5.0
DEFAULT_MAX_THRUST = 40.0


def clamp(value, lower, upper):
	return max(lower, min(upper, value))


def get_key(timeout):
	"""Read a single key without blocking."""
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(fd)
		ready, _, _ = select.select([sys.stdin], [], [], timeout)
		if ready:
			return sys.stdin.read(1)
		return ""
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class BlueROV2ManualMove(Node):
	def __init__(self, topic_style="native", step=DEFAULT_STEP, max_thrust=DEFAULT_MAX_THRUST):
		super().__init__("manaual_move")
		self.step = step
		self.max_thrust = max_thrust

		if topic_style == "yaml":
			topics = [f"/bluerov2/cmd_thrust/t{i}" for i in range(1, 9)]
		else:
			topics = [f"/bluerov2/thruster{i}/cmd_thrust" for i in range(1, 9)]

		self._publishers = [self.create_publisher(Float64, topic, 10) for topic in topics]

		self.surge = 0.0
		self.sway = 0.0
		self.yaw = 0.0
		self.heave = 0.0

		self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_thrusters)

	def publish_thrusters(self):
		# Horizontal mix from the repo's shell helpers:
		# forward  -> [-, -, +, +]
		# left     -> [-, +, -, +]
		# yaw left -> [+, -, -, +]
		t1 = clamp(-self.surge - self.sway + self.yaw, -self.max_thrust, self.max_thrust)
		t2 = clamp(-self.surge + self.sway - self.yaw, -self.max_thrust, self.max_thrust)
		t3 = clamp(self.surge - self.sway - self.yaw, -self.max_thrust, self.max_thrust)
		t4 = clamp(self.surge + self.sway + self.yaw, -self.max_thrust, self.max_thrust)

		# Positive heave means deeper; the helper scripts use negative thrust for up.
		vertical = clamp(self.heave, -self.max_thrust, self.max_thrust)
		t5 = vertical
		t6 = vertical
		t7 = vertical
		t8 = vertical

		for publisher, value in zip(self._publishers, [t1, t2, t3, t4, t5, t6, t7, t8]):
			publisher.publish(Float64(data=value))

	def stop(self):
		self.surge = 0.0
		self.sway = 0.0
		self.yaw = 0.0
		self.heave = 0.0
		self.publish_thrusters()

	def handle_key(self, key):
		if key == "w":
			self.surge = clamp(self.surge + self.step, -self.max_thrust, self.max_thrust)
		elif key == "s":
			self.surge = clamp(self.surge - self.step, -self.max_thrust, self.max_thrust)
		elif key == "a":
			self.sway = clamp(self.sway + self.step, -self.max_thrust, self.max_thrust)
		elif key == "d":
			self.sway = clamp(self.sway - self.step, -self.max_thrust, self.max_thrust)
		elif key == "q":
			self.yaw = clamp(self.yaw + self.step, -self.max_thrust, self.max_thrust)
		elif key == "e":
			self.yaw = clamp(self.yaw - self.step, -self.max_thrust, self.max_thrust)
		elif key == "r":
			self.heave = clamp(self.heave + self.step, -self.max_thrust, self.max_thrust)
		elif key == "f":
			self.heave = clamp(self.heave - self.step, -self.max_thrust, self.max_thrust)
		elif key == " ":
			self.stop()
		else:
			return

		self.get_logger().info(
			f"surge={self.surge:+.1f} sway={self.sway:+.1f} yaw={self.yaw:+.1f} heave={self.heave:+.1f}"
		)


def parse_args():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument(
		"--topic-style",
		choices=("native", "yaml"),
		default="native",
		help="Choose the BlueROV2 ROS topic layout used by this repo.",
	)
	parser.add_argument(
		"--step",
		type=float,
		default=DEFAULT_STEP,
		help="Thrust increment added or subtracted for each key press.",
	)
	parser.add_argument(
		"--max-thrust",
		type=float,
		default=DEFAULT_MAX_THRUST,
		help="Clamp for each thruster output.",
	)
	return parser.parse_args()


def main():
	args = parse_args()
	rclpy.init()
	node = BlueROV2ManualMove(
		topic_style=args.topic_style,
		step=args.step,
		max_thrust=args.max_thrust,
	)

	print(__doc__)
	print("Focus this terminal and press keys. x quits.")

	try:
		while rclpy.ok():
			key = get_key(1.0 / PUBLISH_HZ)
			if key == "x":
				break
			if key:
				node.handle_key(key)
			rclpy.spin_once(node, timeout_sec=0.0)
	except KeyboardInterrupt:
		pass
	finally:
		node.stop()
		node.destroy_node()
		rclpy.shutdown()
		print("Stopped.")


if __name__ == "__main__":
	main()
