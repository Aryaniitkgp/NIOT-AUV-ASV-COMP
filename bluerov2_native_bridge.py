#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS 2 Message imports
from sensor_msgs.msg import Image, Imu, FluidPressure, JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge

# Gazebo Transport & Protobuf message imports
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage, PixelFormatType
from gz.msgs10.imu_pb2 import IMU as GzImu
from gz.msgs10.model_pb2 import Model as GzModel
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.double_pb2 import Double as GzDouble
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV

import time

NUM_THRUSTERS = 8  # matches the topics currently on your `gz topic -l` (bluerov2 base config)

VEHICLE_NAME = 'bluerov2'
ODOM_PUBLISH_RATE = 50.0  # Hz; dynamic_pose/info arrives far faster than this

# Depth reference. The world's water box spans z = 0 .. 2.5 and the
# buoyancy plugin switches to air density above 2.5, so the free surface
# is at z = 2.5 and depth below it is (2.5 - z). Density matches the
# buoyancy plugin's <default_density>.
WATER_SURFACE_Z = 2.5
WATER_DENSITY = 1000.0
GRAVITY = 9.81
ATMOSPHERIC_PA = 101325.0

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
        #
        # There is deliberately no air_pressure subscription here. The
        # bluerov2 model SDF declares only an IMU and the two cameras, so
        # that gz topic never exists - and even if the sensor were added,
        # gz's AirPressure sensor models the standard *atmosphere*, which
        # loses about 12 Pa per metre of altitude and knows nothing about
        # a water column. Inverting it for depth would be meaningless.
        #
        # So the depth sensor is synthesised below from the vehicle pose,
        # which is the honest way to stand in for the pressure sensor a
        # real BlueROV2 carries.
        self.pub_imu = self.create_publisher(Imu, '/bluerov2/imu/data', self.qos_profile)
        self.pub_pressure = self.create_publisher(FluidPressure, '/bluerov2/pressure', self.qos_profile)
        self.pub_depth = self.create_publisher(Float64, '/bluerov2/depth', self.qos_profile)

        gz_imu_topic = '/world/save_arena/model/bluerov2/link/base_link/sensor/imu_sensor/imu'
        self.gz_node.subscribe(GzImu, gz_imu_topic, self._imu_cb)

        # 3. Simulation Clock & Joint States
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        self.gz_node.subscribe(GzClock, '/world/save_arena/clock', self._clock_cb)
        self.gz_node.subscribe(GzModel, '/world/save_arena/model/bluerov2/joint_state', self._joint_state_cb)

        # 3b. Vehicle pose -> Odometry.
        #
        # The model carries no depth sensor (and gz's air pressure sensor
        # models the atmosphere, not a water column), so the world pose
        # published by SceneBroadcaster stands in for the pressure/DVL
        # solution a real vehicle would run. Depth hold needs it.
        self.pub_odom = self.create_publisher(Odometry, '/bluerov2/odom', self.qos_profile)

        self._odom_last_wall = None
        self._odom_last_pos = None
        self._odom_vel = [0.0, 0.0, 0.0]
        self._odom_last_publish = 0.0
        self._last_pose_position = None

        self._last_pressure_pa = ATMOSPHERIC_PA

        self.gz_node.subscribe(GzPoseV, '/world/save_arena/dynamic_pose/info', self._pose_cb)
        self.create_timer(0.05, self._depth_timer_cb)

        # 4. Thruster Commands (ROS -> Gazebo, one Float64 topic per thruster)
        self.thruster_pubs_gz = {}
        self.thruster_subs_ros = []

        for i in range(1, NUM_THRUSTERS + 1):
            gz_topic = f'/model/bluerov2/joint/thruster{i}_joint/cmd_thrust'
            gz_pub = self.gz_node.advertise(gz_topic, GzDouble)
            self.thruster_pubs_gz[i] = gz_pub

            ros_topic = f'/bluerov2/thruster{i}/cmd_thrust'
            sub = self.create_subscription(
                Float64,
                ros_topic,
                self._make_thruster_cb(i),
                self.qos_profile,
            )
            self.thruster_subs_ros.append(sub)

        self.get_logger().info('🚀 BlueROV2 Native Bridge running smoothly without errors!')

    def _make_thruster_cb(self, index):
        def _cb(msg):
            try:
                gz_msg = GzDouble()
                gz_msg.data = msg.data
                self.thruster_pubs_gz[index].publish(gz_msg)
            except Exception as e:
                self.get_logger().error(f'Thruster {index} bridge error: {str(e)}')
        return _cb

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

    def _depth_timer_cb(self):
        """Synthesise the depth sensor the model does not carry.

        World z is up-positive with the pool floor at z = 0, so depth
        below the free surface is (WATER_SURFACE_Z - z), never -z. Getting
        that reference wrong pins the reading at zero for the whole run,
        because the vehicle never goes below z = 0 in the first place.
        """
        if self._last_pose_position is None:
            return

        depth_m = max(0.0, WATER_SURFACE_Z - self._last_pose_position[2])
        self._last_pressure_pa = ATMOSPHERIC_PA + WATER_DENSITY * GRAVITY * depth_m

        try:
            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bluerov2/base_link"
            msg.fluid_pressure = float(self._last_pressure_pa)
            self.pub_pressure.publish(msg)

            self.pub_depth.publish(Float64(data=float(depth_m)))
        except Exception as e:
            self.get_logger().error(f'Depth bridge error: {str(e)}')

    def _pose_cb(self, gz_pose_v):
        try:
            pose = None
            for candidate in gz_pose_v.pose:
                if candidate.name == VEHICLE_NAME:
                    pose = candidate
                    break

            if pose is None:
                return

            now = time.monotonic()
            position = (pose.position.x, pose.position.y, pose.position.z)

            # Velocity by differencing, low-pass filtered. dynamic_pose/info
            # carries no twist, and the depth loop damps on the measured
            # rate, so it has to be reconstructed here where the sample
            # rate is highest.
            if self._odom_last_wall is not None:
                dt = now - self._odom_last_wall
                if 1e-4 < dt < 0.5:
                    alpha = 0.2
                    for i in range(3):
                        raw = (position[i] - self._odom_last_pos[i]) / dt
                        self._odom_vel[i] = (1.0 - alpha) * self._odom_vel[i] + alpha * raw

            self._odom_last_wall = now
            self._odom_last_pos = position
            self._last_pose_position = position

            if now - self._odom_last_publish < 1.0 / ODOM_PUBLISH_RATE:
                return
            self._odom_last_publish = now

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.child_frame_id = 'bluerov2/base_link'

            msg.pose.pose.position.x = position[0]
            msg.pose.pose.position.y = position[1]
            msg.pose.pose.position.z = position[2]
            msg.pose.pose.orientation.x = pose.orientation.x
            msg.pose.pose.orientation.y = pose.orientation.y
            msg.pose.pose.orientation.z = pose.orientation.z
            msg.pose.pose.orientation.w = pose.orientation.w

            # World-frame velocity, not body frame. Depth only needs z.
            msg.twist.twist.linear.x = self._odom_vel[0]
            msg.twist.twist.linear.y = self._odom_vel[1]
            msg.twist.twist.linear.z = self._odom_vel[2]

            self.pub_odom.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Pose bridge error: {str(e)}')

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