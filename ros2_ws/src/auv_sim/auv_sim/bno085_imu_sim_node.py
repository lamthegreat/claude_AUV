from __future__ import annotations

import math
import random
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, MagneticField
from auv_msgs.msg import ImuExtended


MICROTESLA_TO_TESLA = 1e-6


@dataclass(frozen=True)
class MotionState:
    roll: float
    pitch: float
    yaw: float
    roll_dot: float
    pitch_dot: float
    yaw_dot: float
    linear_accel_body: tuple[float, float, float]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_quaternion(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in quat))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quat)


def rpy_rates_to_body_rates(
    roll: float,
    pitch: float,
    roll_dot: float,
    pitch_dot: float,
    yaw_dot: float,
) -> tuple[float, float, float]:
    return (
        roll_dot - yaw_dot * math.sin(pitch),
        pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch),
        -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch),
    )


def quaternion_to_rotation_matrix(quat: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = normalize_quaternion(quat)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_world_to_body(
    quat_world_from_body: tuple[float, float, float, float],
    vector_world: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotation = quaternion_to_rotation_matrix(quat_world_from_body)
    return (
        rotation[0][0] * vector_world[0] + rotation[1][0] * vector_world[1] + rotation[2][0] * vector_world[2],
        rotation[0][1] * vector_world[0] + rotation[1][1] * vector_world[1] + rotation[2][1] * vector_world[2],
        rotation[0][2] * vector_world[0] + rotation[1][2] * vector_world[1] + rotation[2][2] * vector_world[2],
    )


def profile_state(profile: str, t: float) -> MotionState:
    if profile == 'idle':
        return MotionState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))

    if profile == 'yaw_rate':
        yaw_rate = math.radians(20.0)
        return MotionState(0.0, 0.0, yaw_rate * t, 0.0, 0.0, yaw_rate, (0.0, 0.0, 0.0))

    if profile == 'attitude_sweep':
        roll_amp = math.radians(12.0)
        pitch_amp = math.radians(9.0)
        roll_freq = 0.45
        pitch_freq = 0.30
        roll = roll_amp * math.sin(2.0 * math.pi * roll_freq * t)
        pitch = pitch_amp * math.sin(2.0 * math.pi * pitch_freq * t + math.pi / 6.0)
        roll_dot = roll_amp * 2.0 * math.pi * roll_freq * math.cos(2.0 * math.pi * roll_freq * t)
        pitch_dot = pitch_amp * 2.0 * math.pi * pitch_freq * math.cos(2.0 * math.pi * pitch_freq * t + math.pi / 6.0)
        yaw = math.radians(5.0) * math.sin(2.0 * math.pi * 0.18 * t)
        yaw_dot = math.radians(5.0) * 2.0 * math.pi * 0.18 * math.cos(2.0 * math.pi * 0.18 * t)
        return MotionState(roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot, (0.0, 0.0, 0.0))

    if profile == 'accel_burst':
        cycle_s = 4.0
        phase = t % cycle_s
        accel_x = 0.0
        if 0.5 <= phase < 1.0:
            accel_x = 0.8
        elif 1.0 <= phase < 1.5:
            accel_x = -0.4
        elif 2.2 <= phase < 2.7:
            accel_x = 0.6
        pitch = math.radians(3.0) * math.sin(2.0 * math.pi * 0.2 * t)
        pitch_dot = math.radians(3.0) * 2.0 * math.pi * 0.2 * math.cos(2.0 * math.pi * 0.2 * t)
        return MotionState(0.0, pitch, 0.0, 0.0, pitch_dot, 0.0, (accel_x, 0.0, 0.0))

    raise ValueError(f'Unsupported profile: {profile}')


class Bno085ImuSimNode(Node):
    def __init__(self):
        super().__init__('bno085_imu_sim_node')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_rate_hz', 200.0)
        self.declare_parameter('mag_rate_hz', 100.0)
        self.declare_parameter('profile', 'idle')
        self.declare_parameter('start_fully_calibrated', True)
        self.declare_parameter('orientation_noise_stddev_rad', 0.01)
        self.declare_parameter('gyro_noise_stddev_rad_s', 0.003)
        self.declare_parameter('accel_noise_stddev_m_s2', 0.02)
        self.declare_parameter('mag_noise_stddev_ut', 0.15)
        self.declare_parameter('gyro_bias_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('accel_bias_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('mag_bias_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('heading_accuracy_rad_nominal', 0.035)

        self._namespace = str(self.get_parameter('namespace').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._imu_rate_hz = float(self.get_parameter('imu_rate_hz').value)
        self._mag_rate_hz = float(self.get_parameter('mag_rate_hz').value)
        self._profile = str(self.get_parameter('profile').value)
        self._dropout_probability = float(self.get_parameter('dropout_probability').value)
        self._orientation_noise_stddev_rad = float(self.get_parameter('orientation_noise_stddev_rad').value)
        self._gyro_noise_stddev_rad_s = float(self.get_parameter('gyro_noise_stddev_rad_s').value)
        self._accel_noise_stddev_m_s2 = float(self.get_parameter('accel_noise_stddev_m_s2').value)
        self._mag_noise_stddev_ut = float(self.get_parameter('mag_noise_stddev_ut').value)
        self._gyro_bias = tuple(float(v) for v in self.get_parameter('gyro_bias_xyz').value)
        self._accel_bias = tuple(float(v) for v in self.get_parameter('accel_bias_xyz').value)
        self._mag_bias = tuple(float(v) for v in self.get_parameter('mag_bias_xyz').value)
        self._heading_accuracy_nominal = float(self.get_parameter('heading_accuracy_rad_nominal').value)
        self._fully_calibrated = bool(self.get_parameter('start_fully_calibrated').value)
        self._calibration_level = 3 if self._fully_calibrated else 1
        self._start_time = self.get_clock().now()
        self._published_imu = 0
        self._published_mag = 0
        self._logged_subscribers = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_imu_raw = self.create_publisher(Imu, f'/{self._namespace}/sensors/imu/raw', qos)
        self._pub_imu_ext = self.create_publisher(ImuExtended, f'/{self._namespace}/sensors/imu/extended', qos)
        self._pub_mag = self.create_publisher(MagneticField, f'/{self._namespace}/sensors/imu/magnetic_field', qos)

        self._imu_timer = self.create_timer(1.0 / self._imu_rate_hz, self._publish_imu)
        self._mag_timer = self.create_timer(1.0 / self._mag_rate_hz, self._publish_mag)
        self._status_timer = self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            f'BNO085 IMU simulator started with profile={self._profile}, '
            f'imu_rate={self._imu_rate_hz:.1f}Hz, mag_rate={self._mag_rate_hz:.1f}Hz.'
        )

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _current_heading_accuracy(self) -> float:
        if self._fully_calibrated:
            return self._heading_accuracy_nominal
        return self._heading_accuracy_nominal * 4.0

    def _sample_motion(self) -> tuple[tuple[float, float, float, float], tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        state = profile_state(self._profile, self._elapsed_s())

        roll = state.roll
        pitch = state.pitch
        yaw = state.yaw

        quat = quaternion_from_rpy(roll, pitch, yaw)
        body_rates = rpy_rates_to_body_rates(roll, pitch, state.roll_dot, state.pitch_dot, state.yaw_dot)
        magnetic_field_world_ut = (20.0, 0.0, -45.0)
        magnetic_field_body_ut = rotate_world_to_body(quat, magnetic_field_world_ut)

        noisy_roll = roll + random.gauss(0.0, self._orientation_noise_stddev_rad)
        noisy_pitch = pitch + random.gauss(0.0, self._orientation_noise_stddev_rad)
        noisy_yaw = yaw + random.gauss(0.0, self._orientation_noise_stddev_rad)
        noisy_quat = normalize_quaternion(quaternion_from_rpy(noisy_roll, noisy_pitch, noisy_yaw))

        noisy_body_rates = tuple(
            body_rates[i] + self._gyro_bias[i] + random.gauss(0.0, self._gyro_noise_stddev_rad_s)
            for i in range(3)
        )
        noisy_linear_accel = tuple(
            state.linear_accel_body[i] + self._accel_bias[i] + random.gauss(0.0, self._accel_noise_stddev_m_s2)
            for i in range(3)
        )
        noisy_mag_ut = tuple(
            magnetic_field_body_ut[i] + self._mag_bias[i] + random.gauss(0.0, self._mag_noise_stddev_ut)
            for i in range(3)
        )

        return noisy_quat, noisy_body_rates, noisy_linear_accel, noisy_mag_ut

    def _publish_imu(self):
        if random.random() < self._dropout_probability:
            return

        quat, angular_velocity, linear_acceleration, _ = self._sample_motion()
        stamp = self.get_clock().now().to_msg()

        raw_msg = Imu()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self._frame_id
        raw_msg.orientation.x = quat[0]
        raw_msg.orientation.y = quat[1]
        raw_msg.orientation.z = quat[2]
        raw_msg.orientation.w = quat[3]
        raw_msg.angular_velocity.x = angular_velocity[0]
        raw_msg.angular_velocity.y = angular_velocity[1]
        raw_msg.angular_velocity.z = angular_velocity[2]
        raw_msg.linear_acceleration.x = linear_acceleration[0]
        raw_msg.linear_acceleration.y = linear_acceleration[1]
        raw_msg.linear_acceleration.z = linear_acceleration[2]
        raw_msg.orientation_covariance = self._diagonal_covariance(self._orientation_noise_stddev_rad ** 2)
        raw_msg.angular_velocity_covariance = self._diagonal_covariance(self._gyro_noise_stddev_rad_s ** 2)
        raw_msg.linear_acceleration_covariance = self._diagonal_covariance(self._accel_noise_stddev_m_s2 ** 2)
        self._pub_imu_raw.publish(raw_msg)

        ext_msg = ImuExtended()
        ext_msg.header.stamp = stamp
        ext_msg.header.frame_id = self._frame_id
        ext_msg.orientation = raw_msg.orientation
        ext_msg.angular_velocity = raw_msg.angular_velocity
        ext_msg.linear_acceleration = raw_msg.linear_acceleration
        ext_msg.orientation_covariance = raw_msg.orientation_covariance
        ext_msg.calibration_system = self._calibration_level
        ext_msg.calibration_gyro = self._calibration_level
        ext_msg.calibration_accel = self._calibration_level
        ext_msg.calibration_mag = self._calibration_level
        ext_msg.fully_calibrated = self._fully_calibrated
        self._pub_imu_ext.publish(ext_msg)

        self._published_imu += 1

    def _publish_mag(self):
        if random.random() < self._dropout_probability:
            return

        _, _, _, magnetic_field_body_ut = self._sample_motion()
        stamp = self.get_clock().now().to_msg()

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self._frame_id
        mag_msg.magnetic_field.x = magnetic_field_body_ut[0] * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field.y = magnetic_field_body_ut[1] * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field.z = magnetic_field_body_ut[2] * MICROTESLA_TO_TESLA
        mag_msg.magnetic_field_covariance = self._diagonal_covariance(
            (self._mag_noise_stddev_ut * MICROTESLA_TO_TESLA) ** 2
        )
        self._pub_mag.publish(mag_msg)

        self._published_mag += 1

    def _log_status(self):
        if not self._logged_subscribers:
            self.get_logger().info(
                'Topic subscribers: '
                f'raw={self.count_subscribers(f"/{self._namespace}/sensors/imu/raw")} '
                f'extended={self.count_subscribers(f"/{self._namespace}/sensors/imu/extended")} '
                f'mag={self.count_subscribers(f"/{self._namespace}/sensors/imu/magnetic_field")}'
            )
            self._logged_subscribers = True

        self.get_logger().debug(
            f'Published IMU={self._published_imu} messages, MAG={self._published_mag} messages, '
            f'heading_accuracy={math.degrees(self._current_heading_accuracy()):.2f}deg'
        )

    @staticmethod
    def _diagonal_covariance(variance: float) -> list[float]:
        covariance = [0.0] * 9
        covariance[0] = variance
        covariance[4] = variance
        covariance[8] = variance
        return covariance


def main(args=None):
    rclpy.init(args=args)
    node = Bno085ImuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
