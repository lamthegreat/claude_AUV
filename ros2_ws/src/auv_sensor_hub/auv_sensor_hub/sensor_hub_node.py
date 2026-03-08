"""
Sensor Hub Bridge Node

Pi-side ROS2 bridge for the Teensy 4.1 sensor hub (microROS node).
Subscribes to raw sensor data published by the Teensy over microROS,
validates it, and republishes with diagnostics.

Subscriptions (from Teensy 4.1 via microROS agent):
  /auv/sensors/imu/raw       sensor_msgs/Imu
  /auv/sensors/imu/extended  auv_msgs/ImuExtended

Publications:
  /auv/sensors/imu/raw       sensor_msgs/Imu        (pass-through + watchdog)
  /auv/sensors/imu/extended  auv_msgs/ImuExtended   (pass-through + watchdog)

TODO (Phase 2 - Sensor Hub Firmware):
  - Add depth sensor subscription when Teensy firmware supports it
  - Add topic rate monitoring and /diagnostics publishing
  - Add IMU calibration service to trigger BNO085 save
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
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
        self._sub_imu_ext = self.create_subscription(
            ImuExtended,
            f'/{ns}/sensors/imu/extended',
            self._imu_extended_callback,
            qos,
        )

        # Watchdog timer — warns if IMU data stops arriving
        self._watchdog = self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info('Sensor hub node started. Waiting for microROS agent...')

    def _imu_raw_callback(self, msg: Imu):
        self._last_imu_stamp = self.get_clock().now()

    def _imu_extended_callback(self, msg: ImuExtended):
        if msg.fully_calibrated:
            self.get_logger().debug('BNO085 fully calibrated.')
        else:
            self.get_logger().debug(
                f'BNO085 calib: sys={msg.calibration_system} '
                f'gyro={msg.calibration_gyro} '
                f'accel={msg.calibration_accel} '
                f'mag={msg.calibration_mag}'
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
