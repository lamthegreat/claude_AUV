"""
State Estimator Node

Fuses IMU orientation and depth measurements into a unified AUV state estimate.
Publishes to /auv/state/auv_state and broadcasts the odom→base_link TF transform.

Phase 5 implementation: complementary filter (IMU orientation + depth = full pose).
Planned upgrade: replace with robot_localization EKF when DVL or sonar is available.

Subscriptions:
  /auv/sensors/imu/extended    auv_msgs/ImuExtended
  /auv/sensors/depth           auv_msgs/DepthStamped   (future)

Publications:
  /auv/state/auv_state         auv_msgs/AuvState
  /auv/state/pose              geometry_msgs/PoseStamped
  TF: odom → base_link

TODO (Phase 5 - State Estimator):
  - Implement complementary filter for orientation smoothing
  - Integrate depth reading into pose.position.z
  - Publish odom→base_link TF
  - Add dead-reckoning from IMU accelerometer (with drift caveats)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from auv_msgs.msg import ImuExtended, DepthStamped, AuvState


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        ns = self.get_parameter('namespace').value
        self._base_frame = self.get_parameter('base_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value

        # Latest sensor readings
        self._latest_imu: ImuExtended | None = None
        self._latest_depth: DepthStamped | None = None

        # IMU source selection thresholds
        # Switch to 6-DOF Game RV when heading uncertainty exceeds ~20° or mag is poor.
        self._heading_accuracy_threshold_rad = 0.35  # ~20 degrees
        self._mag_calib_min = 2
        self._using_9dof: bool | None = None  # None = not yet decided (for first-log)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub_imu = self.create_subscription(
            ImuExtended, f'/{ns}/sensors/imu/extended', self._imu_callback, qos)
        self._sub_depth = self.create_subscription(
            DepthStamped, f'/{ns}/sensors/depth', self._depth_callback, qos)

        self._pub_state = self.create_publisher(AuvState, f'/{ns}/state/auv_state', 10)
        self._pub_pose = self.create_publisher(PoseStamped, f'/{ns}/state/pose', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_state)

        self.get_logger().info('State estimator node started.')

    def _imu_callback(self, msg: ImuExtended):
        self._latest_imu = msg

    def _depth_callback(self, msg: DepthStamped):
        self._latest_depth = msg

    def _publish_state(self):
        if self._latest_imu is None:
            return

        now = self.get_clock().now().to_msg()
        imu = self._latest_imu

        # --- AuvState ---
        state = AuvState()
        state.header.stamp = now
        state.header.frame_id = self._odom_frame

        # Select orientation source based on magnetometer reliability.
        # 9-DOF (ARVR): absolute heading, mag-referenced — prefer when mag is good.
        # 6-DOF (Game RV): relative heading, mag-immune — fallback when mag is noisy.
        use_9dof = self._should_use_9dof(
            imu.calibration_mag,
            imu.heading_accuracy_rad,
            self._mag_calib_min,
            self._heading_accuracy_threshold_rad,
        )
        orientation = imu.orientation if use_9dof else imu.game_rv_orientation

        if use_9dof != self._using_9dof:
            if use_9dof:
                self.get_logger().info(
                    'IMU source: 9-DOF ARVR (mag reliable, '
                    f'heading_accuracy={imu.heading_accuracy_rad:.3f} rad)'
                )
            else:
                self.get_logger().info(
                    'IMU source: 6-DOF Game RV (mag unreliable — '
                    f'calib_mag={imu.calibration_mag}, '
                    f'heading_accuracy={imu.heading_accuracy_rad:.3f} rad)'
                )
            self._using_9dof = use_9dof

        state.pose.orientation = orientation
        state.twist.angular.x = imu.angular_velocity.x
        state.twist.angular.y = imu.angular_velocity.y
        state.twist.angular.z = imu.angular_velocity.z

        if self._latest_depth is not None:
            state.depth_m = self._latest_depth.depth_m
            state.pose.position.z = -self._latest_depth.depth_m
            state.depth_sensor_valid = True
        else:
            state.depth_sensor_valid = False

        state.imu_calibrated = imu.fully_calibrated
        state.control_mode = AuvState.MANUAL

        self._pub_state.publish(state)

        # --- PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self._odom_frame
        pose_msg.pose = state.pose
        self._pub_pose.publish(pose_msg)

        # --- TF: odom → base_link ---
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = state.pose.position.x
        tf.transform.translation.y = state.pose.position.y
        tf.transform.translation.z = state.pose.position.z
        tf.transform.rotation = orientation
        self._tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _should_use_9dof(
        calibration_mag: int,
        heading_accuracy_rad: float,
        mag_calib_min: int,
        heading_accuracy_threshold_rad: float,
    ) -> bool:
        """Return True when the 9-DOF ARVR source is reliable enough to use.

        Prefers 9-DOF (absolute heading, mag-referenced) when the magnetometer
        is well-calibrated AND the BNO085 heading uncertainty is below the
        threshold.  Falls back to 6-DOF Game RV (mag-immune) otherwise.
        """
        return (
            calibration_mag >= mag_calib_min
            and heading_accuracy_rad < heading_accuracy_threshold_rad
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
