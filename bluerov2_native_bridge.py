#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS 2 Message imports
from sensor_msgs.msg import Image, Imu, FluidPressure, JointState, CameraInfo
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
from gz.msgs10.entity_factory_pb2 import EntityFactory as GzEntityFactory
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean

from std_msgs.msg import Empty as EmptyMsg

import time
import random
from collections import deque

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

class Bar30Model:
    """Simulates a Blue Robotics Bar30 (MS5837-30BA) depth sensor.

    Ground truth from the physics engine is perfect, zero latency and
    infinitely fast, which flatters any depth loop tuned against it. The
    real part is none of those things, and the difference matters: a
    derivative term tuned on clean truth will chatter badly once it is
    fed a quantised, noisy, delayed measurement.

    Three effects, in the order the hardware applies them:

      1. Transport delay  - I2C conversion (~20 ms at max oversampling)
                            plus driver and ROS hops.
      2. Sensor noise     - roughly 0.3 mbar RMS, i.e. about 3 mm of
                            water, at the highest OSR setting.
      3. Quantisation     - the ADC resolves 0.2 mbar, about 2 mm.

    Noise is added before quantisation because that is the physical
    order: the ADC digitises an already-noisy analogue signal. Doing it
    the other way round would hide the dithering that real noise gives
    you for free.
    """

    def __init__(self, resolution_pa=20.0, noise_pa=30.0, latency_s=0.030, seed=None):
        self.resolution_pa = resolution_pa
        self.noise_pa = noise_pa
        self.latency_s = latency_s
        self._rng = random.Random(seed)
        self._pipe = deque()

    def sample(self, true_pressure_pa, now):
        """Feed the current truth, get back what the sensor would report.

        Returns None until the pipeline has been primed for latency_s,
        which mirrors a real sensor not having a reading yet at boot.
        """
        self._pipe.append((now + self.latency_s, true_pressure_pa))

        delayed = None
        while self._pipe and self._pipe[0][0] <= now:
            delayed = self._pipe.popleft()[1]

        if delayed is None:
            return None

        if self.noise_pa > 0.0:
            delayed += self._rng.gauss(0.0, self.noise_pa)

        if self.resolution_pa > 0.0:
            delayed = round(delayed / self.resolution_pa) * self.resolution_pa

        return delayed


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

        # Camera intrinsics, published alongside every frame.
        #
        # In sim the cameras are ideal pinholes, so K comes straight from
        # the SDF field of view and D is zero. On the vehicle neither is
        # true: a flat viewport refracts water (n=1.33) into air (n=1.0),
        # scaling the effective focal length by ~1.33 and adding radial
        # distortion. Point calibration_file at an underwater OpenCV
        # calibration and it overrides these numbers, so nothing
        # downstream has to change to move from sim to hardware.
        self.declare_parameter('camera_hfov', 1.047)   # matches model.sdf
        self.declare_parameter('calibration_file', '')

        self.pub_front_info = self.create_publisher(CameraInfo, '/bluerov2/front_camera/camera_info', self.qos_profile)
        self.pub_down_info = self.create_publisher(CameraInfo, '/bluerov2/down_camera/camera_info', self.qos_profile)

        self._camera_info = self._build_camera_info()

        self.gz_node.subscribe(GzImage, '/camera/image_raw', lambda msg: self._camera_cb(msg, self.pub_front_cam, "front_camera", self.pub_front_info))
        self.gz_node.subscribe(GzImage, '/down_camera/image_raw', lambda msg: self._camera_cb(msg, self.pub_down_cam, "down_camera", self.pub_down_info))

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

        # Bar30 characteristics, exposed so the whole sensor model can be
        # switched off (bar30_realistic:=false) to get the old perfect
        # reading back for an A/B comparison.
        self.declare_parameter('bar30_realistic', True)
        self.declare_parameter('bar30_rate_hz', 20.0)
        self.declare_parameter('bar30_resolution_pa', 20.0)   # 0.2 mbar ~ 2 mm
        self.declare_parameter('bar30_noise_pa', 30.0)        # 0.3 mbar ~ 3 mm RMS
        self.declare_parameter('bar30_latency_ms', 30.0)

        realistic = self.get_parameter('bar30_realistic').value
        rate_hz = max(1.0, float(self.get_parameter('bar30_rate_hz').value))

        self.bar30 = Bar30Model(
            resolution_pa=float(self.get_parameter('bar30_resolution_pa').value) if realistic else 0.0,
            noise_pa=float(self.get_parameter('bar30_noise_pa').value) if realistic else 0.0,
            latency_s=float(self.get_parameter('bar30_latency_ms').value) * 1e-3 if realistic else 0.0,
        )

        self.gz_node.subscribe(GzPoseV, '/world/save_arena/dynamic_pose/info', self._pose_cb)
        self.create_timer(1.0 / rate_hz, self._depth_timer_cb)

        self.get_logger().info(
            f'Bar30 model: {"realistic" if realistic else "IDEAL (noise off)"} '
            f'@ {rate_hz:.0f} Hz'
        )

        # 3c. Marker dropper.
        #
        # The vehicle carries no modelled payload, so a "drop" spawns a
        # marker into the world at the vehicle's current position and lets
        # it fall on its own. Denser than water and small, so it sinks
        # quickly and roughly straight down - which is exactly why the
        # mission has to come to a stop before releasing, or the marker
        # keeps the vehicle's horizontal velocity and lands downrange.
        self.declare_parameter('marker_mass', 0.25)
        self.declare_parameter('marker_radius', 0.03)

        self._marker_count = 0
        self.create_subscription(
            EmptyMsg, '/bluerov2/drop_marker', self._drop_marker_cb, 10)

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

    def _build_camera_info(self):
        """Intrinsics for the 640x480 frames this bridge publishes.

        A calibration_file, if given, wins. It is the plain OpenCV YAML
        you get out of cv.calibrateCamera: camera_matrix and
        distortion_coefficients. Shoot it underwater, through the real
        housing - a dry calibration does not describe the optics the
        vehicle actually flies with.
        """
        width, height = 640, 480

        path = str(self.get_parameter('calibration_file').value or '')
        if path:
            try:
                fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
                k = fs.getNode('camera_matrix').mat()
                d = fs.getNode('distortion_coefficients').mat()
                fs.release()

                info = CameraInfo()
                info.width, info.height = width, height
                info.distortion_model = 'plumb_bob'
                info.k = [float(v) for v in k.flatten()]
                info.d = [float(v) for v in d.flatten()]
                info.p = [k[0][0], 0.0, k[0][2], 0.0,
                          0.0, k[1][1], k[1][2], 0.0,
                          0.0, 0.0, 1.0, 0.0]
                self.get_logger().info(
                    f'Loaded camera calibration from {path} (fx={k[0][0]:.1f} px)')
                return info
            except Exception as e:
                self.get_logger().error(
                    f'Could not read calibration_file {path}: {e}; '
                    f'falling back to the nominal FOV.')

        hfov = float(self.get_parameter('camera_hfov').value)
        fx = (width / 2.0) / np.tan(hfov / 2.0)
        cx, cy = width / 2.0, height / 2.0

        info = CameraInfo()
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.get_logger().info(
            f'Camera intrinsics from nominal FOV: fx={fx:.1f} px, no distortion. '
            f'Pass -p calibration_file:=<yaml> to use a real one.')
        return info

    def _camera_cb(self, gz_img, publisher, frame_id, info_publisher=None):
        try:
            encoding = PIXEL_FORMAT.get(gz_img.pixel_format_type, 'rgb8')
            channels = 1 if encoding.startswith('mono') else 3

            img_np = np.frombuffer(gz_img.data, dtype=np.uint8).reshape((gz_img.height, gz_img.width, channels))
            img_resized = cv2.resize(img_np, (640, 480))

            msg = self.bridge.cv2_to_imgmsg(img_resized, encoding=encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"bluerov2/{frame_id}_optical_frame"

            publisher.publish(msg)

            # Same stamp and frame as the image, so a consumer can pair
            # them without guessing.
            if info_publisher is not None:
                info = self._camera_info
                info.header = msg.header
                info_publisher.publish(info)
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

        true_depth_m = max(0.0, WATER_SURFACE_Z - self._last_pose_position[2])
        true_pressure_pa = ATMOSPHERIC_PA + WATER_DENSITY * GRAVITY * true_depth_m

        # Run the truth through the sensor model. Pressure is what the
        # hardware actually measures, so noise/quantisation/latency are
        # applied there and depth is derived from the result - not the
        # other way round.
        measured_pa = self.bar30.sample(true_pressure_pa, time.monotonic())
        if measured_pa is None:
            return

        self._last_pressure_pa = measured_pa
        measured_depth_m = max(0.0, (measured_pa - ATMOSPHERIC_PA) / (WATER_DENSITY * GRAVITY))

        try:
            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "bluerov2/base_link"
            msg.fluid_pressure = float(measured_pa)
            msg.variance = float(self.bar30.noise_pa ** 2)
            self.pub_pressure.publish(msg)

            self.pub_depth.publish(Float64(data=float(measured_depth_m)))
        except Exception as e:
            self.get_logger().error(f'Depth bridge error: {str(e)}')

    def _drop_marker_cb(self, _msg):
        """Spawn a marker at the vehicle and let physics take it down."""
        if self._last_pose_position is None:
            self.get_logger().warn('Drop requested but vehicle pose is unknown.')
            return

        x, y, z = self._last_pose_position
        z -= 0.15                       # clear of the hull before it falls

        mass = float(self.get_parameter('marker_mass').value)
        radius = float(self.get_parameter('marker_radius').value)

        # Inertia of a solid sphere, so the spawn is not rejected for
        # having a degenerate inertial.
        i = 0.4 * mass * radius * radius

        self._marker_count += 1
        name = f'marker_{self._marker_count}'

        sdf = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="body">
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="col">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
      </collision>
      <visual name="vis">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>1.0 0.2 0.6 1.0</ambient>
          <diffuse>1.0 0.2 0.6 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

        try:
            req = GzEntityFactory()
            req.sdf = sdf
            req.name = name
            req.allow_renaming = True

            ok, result = self.gz_node.request(
                '/world/save_arena/create', req, GzEntityFactory, GzBoolean, 2000)

            if ok and result.data:
                self.get_logger().info(
                    f'Dropped {name} at ({x:.2f}, {y:.2f}, {z:.2f})')
            else:
                self.get_logger().warn(f'Spawn of {name} was refused by Gazebo.')
        except Exception as e:
            # The mission still scores the drop from its own estimate, so
            # a failed spawn costs the visual only.
            self.get_logger().error(f'Marker spawn failed: {e}')

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