#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS 2 Message imports
from sensor_msgs.msg import Image, Imu, FluidPressure, JointState
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge

# Gazebo Transport & Protobuf message imports
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage, PixelFormatType
from gz.msgs10.imu_pb2 import IMU as GzImu
from gz.msgs10.fluid_pressure_pb2 import FluidPressure as GzFluidPressure
from gz.msgs10.model_pb2 import Model as GzModel
from gz.msgs10.clock_pb2 import Clock as GzClock

import cv2
import numpy as np

PIXEL_FORMAT = {
    PixelFormatType.L_INT8: 'mono8',
    PixelFormatType.L_INT16: 'mono16',
    PixelFormatType.RGB_INT8: 'rgb8',
    PixelFormatType.RGBA_INT8: 'rgba8',
    PixelFormatType.BGRA_INT8: 'bgra8',
    PixelFormatType.BGR_INT8: 'bgr8',
}

class BlueROV2NativeBridge(Node):
    def __init__(self):
        super().__init__('bluerov2_native_bridge')
        self.bridge = CvBridge()
        self.gz_node = GzNode()

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 1. Image Topics
        self.pub_front_cam = self.create_publisher(Image, '/bluerov2/front_camera/image_raw', self.qos_profile)
        self.pub_down_cam = self.create_publisher(Image, '/bluerov2/down_camera/image_raw', self.qos_profile)

        self.gz_node.subscribe(GzImage, '/camera/image_raw', lambda msg: self._camera_cb(msg, self.pub_front_cam, "front_camera"))
        self.gz_node.subscribe(GzImage, '/down_camera/image_raw', lambda msg: self._camera_cb(msg, self.pub_down_cam, "down_camera"))

        # 2. Sensor Topics
        self.pub_imu = self.create_publisher(Imu, '/bluerov2/imu/data', self.qos_profile)
        self.pub_pressure = self.create_publisher(FluidPressure, '/bluerov2/air_pressure', self.qos_profile)

        gz_imu_topic = '/world/save_arena/model/bluerov2/link/base_link/sensor/imu_sensor/imu'
        gz_press_topic = '/world/save_arena/model/bluerov2/link/base_link/sensor/air_pressure_sensor/air_pressure'

        self.gz_node.subscribe(GzImu, gz_imu_topic, self._imu_cb)
        self.gz_node.subscribe(GzFluidPressure, gz_press_topic, self._pressure_cb)

        # 3. Simulation Clock & Joint States
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        self.gz_node.subscribe(GzClock, '/world/save_arena/clock', self._clock_cb)
        self.gz_node.subscribe(GzModel, '/world/save_arena/model/bluerov2/joint_state', self._joint_state_cb)

        self.get_logger().info('🚀 BlueROV2 Native Bridge running smoothly without errors!')

    def _camera_cb(self, gz_img, publisher, frame_id):
        try:
            encoding = PIXEL_FORMAT.get(gz_img.pixel_format_type, 'rgb8')
            channels = 1 if encoding.startswith('mono') else 3

            img_np = np.frombuffer(gz_img.data, dtype=np.uint8).reshape((gz_img.height, gz_img.width, channels))
            img_resized = cv2.resize(img_np, (640, 480))

            msg = self.bridge.cv2_to_imgmsg(img_resized, encoding=encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"bluerov2/{frame_id}_optical_frame"

            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Camera bridge error ({frame_id}): {str(e)}')

    def _imu_cb(self, gz_imu):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bluerov2/base_link"

            msg.orientation.x = gz_imu.orientation.x
            msg.orientation.y = gz_imu.orientation.y
            msg.orientation.z = gz_imu.orientation.z
            msg.orientation.w = gz_imu.orientation.w

            msg.angular_velocity.x = gz_imu.angular_velocity.x
            msg.angular_velocity.y = gz_imu.angular_velocity.y
            msg.angular_velocity.z = gz_imu.angular_velocity.z

            msg.linear_acceleration.x = gz_imu.linear_acceleration.x
            msg.linear_acceleration.y = gz_imu.linear_acceleration.y
            msg.linear_acceleration.z = gz_imu.linear_acceleration.z

            self.pub_imu.publish(msg)
        except Exception as e:
            self.get_logger().error(f'IMU bridge error: {str(e)}')

    def _pressure_cb(self, gz_press):
        try:
            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bluerov2/base_link"
            msg.fluid_pressure = gz_press.pressure

            self.pub_pressure.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Fluid pressure bridge error: {str(e)}')

    def _clock_cb(self, gz_clock):
        try:
            msg = Clock()
            msg.clock.sec = gz_clock.sim.sec
            msg.clock.nanosec = gz_clock.sim.nsec
            self.pub_clock.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Clock bridge error: {str(e)}')

    def _joint_state_cb(self, gz_model):
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            for joint in gz_model.joint:
                msg.name.append(joint.name)
                if joint.HasField('axis1'):
                    msg.position.append(float(joint.axis1.position))
                    msg.velocity.append(float(joint.axis1.velocity))
                else:
                    msg.position.append(0.0)
                    msg.velocity.append(0.0)

            self.pub_joint_states.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Joint state bridge error: {str(e)}')

def main():
    rclpy.init()
    node = BlueROV2NativeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
