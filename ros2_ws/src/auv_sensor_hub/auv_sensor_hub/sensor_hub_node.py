"""
Sensor Hub Bridge Node

Pi-side ROS2 bridge for the Teensy 4.1 sensor hub (microROS node).
Subscribes to raw sensor data published by the Teensy over microROS,
assembles ImuExtended, and publishes it for downstream nodes.

Subscriptions (from Teensy 4.1 via microROS agent):
  /auv/sensors/imu/raw       sensor_msgs/Imu        (9-DOF ARVR, heading_accuracy in cov[8])
  /auv/sensors/imu/game_rv   sensor_msgs/Imu        (6-DOF Game RV, no magnetometer)
  /auv/sensors/imu/magnetic_field  sensor_msgs/MagneticField

Publications:
  /auv/sensors/imu/extended  auv_msgs/ImuExtended   (assembled from raw + game_rv)

TODO (Phase 2 - Sensor Hub Firmware):
  - Add depth sensor subscription when Teensy firmware supports it
  - Add topic rate monitoring and /diagnostics publishing
  - Add IMU calibration service to trigger BNO085 save
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField
from auv_msgs.msg import ImuExtended


class SensorHubNode(Node):
    def __init__(self):
        super().__init__('sensor_hub_node')

        # Parameters
        self.declare_parameter('imu_timeout_s', 1.0)
        self.declare_parameter('namespace', 'auv')

        ns = self.get_parameter('namespace').value
        self._imu_timeout_s = self.get_parameter('imu_timeout_s').value
        self._last_imu_stamp = None

        # Latest messages from each Teensy topic
        self._latest_game_rv: Imu | None = None
        self._latest_mag: MagneticField | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions — data originates on Teensy 4.1 via microROS
        self._sub_imu_raw = self.create_subscription(
            Imu,
            f'/{ns}/sensors/imu/raw',
            self._imu_raw_callback,
            qos,
        )
        self._sub_game_rv = self.create_subscription(
            Imu,
            f'/{ns}/sensors/imu/game_rv',
            self._game_rv_callback,
            qos,
        )
        self._sub_mag = self.create_subscription(
            MagneticField,
            f'/{ns}/sensors/imu/magnetic_field',
            self._mag_callback,
            qos,
        )

        # Publication — assembled ImuExtended for state estimator and other consumers
        self._pub_imu_ext = self.create_publisher(
            ImuExtended,
            f'/{ns}/sensors/imu/extended',
            10,
        )

        # Watchdog timer — warns if IMU data stops arriving
        self._watchdog = self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info('Sensor hub node started. Waiting for microROS agent...')

    def _game_rv_callback(self, msg: Imu):
        self._latest_game_rv = msg

    def _mag_callback(self, msg: MagneticField):
        self._latest_mag = msg

    def _imu_raw_callback(self, msg: Imu):
        self._last_imu_stamp = self.get_clock().now()

        ext = ImuExtended()
        ext.header = msg.header

        # 9-DOF ARVR orientation (absolute heading, mag-referenced)
        ext.orientation = msg.orientation
        ext.orientation_covariance = msg.orientation_covariance

        # 6-DOF Game RV orientation (relative heading, mag-immune)
        if self._latest_game_rv is not None:
            ext.game_rv_orientation = self._latest_game_rv.orientation
        else:
            # Identity quaternion until first game_rv message arrives
            ext.game_rv_orientation.w = 1.0

        # Decode heading_accuracy_rad from orientation_covariance[8].
        # Firmware sets cov[8] = heading_accuracy_rad^2 (yaw variance slot).
        # cov[0] > 0 indicates the matrix is valid (not the REP-145 "unknown" signal).
        cov8 = msg.orientation_covariance[8]
        if msg.orientation_covariance[0] > 0.0 and cov8 >= 0.0:
            ext.heading_accuracy_rad = math.sqrt(cov8)
        else:
            ext.heading_accuracy_rad = 0.0

        ext.angular_velocity = msg.angular_velocity
        ext.linear_acceleration = msg.linear_acceleration

        # Calibration fields: BNO085 status is not transmitted in sensor_msgs/Imu.
        # Leave at 0 (unreliable) until a dedicated calibration topic is added.
        # heading_accuracy_rad and game_rv fallback are sufficient for selection.
        ext.calibration_rv = 0
        ext.calibration_system = 0
        ext.calibration_gyro = 0
        ext.calibration_accel = 0
        ext.calibration_mag = 0
        ext.fully_calibrated = False

        self._pub_imu_ext.publish(ext)

        # Calibration status debug log (mirrors the old _imu_extended_callback behaviour)
        if ext.fully_calibrated:
            self.get_logger().debug('BNO085 fully calibrated.')
        else:
            self.get_logger().debug(
                f'BNO085 calib: sys={ext.calibration_system} '
                f'gyro={ext.calibration_gyro} '
                f'accel={ext.calibration_accel} '
                f'mag={ext.calibration_mag} '
                f'heading_accuracy={ext.heading_accuracy_rad:.3f} rad'
            )

    def _watchdog_callback(self):
        if self._last_imu_stamp is None:
            self.get_logger().warn('No IMU data received yet. Is the microROS agent running?')
            return

        age_s = (self.get_clock().now() - self._last_imu_stamp).nanoseconds * 1e-9
        if age_s > self._imu_timeout_s:
            self.get_logger().warn(f'IMU data stale: {age_s:.1f}s since last message.')


def main(args=None):
    rclpy.init(args=args)
    node = SensorHubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
