"""
Safety Monitor Node.

Monitors system health and triggers emergency surface if critical conditions occur:
  - IMU data timeout (sensor hub lost)
  - Excessive depth (depth sensor required)
  - Future: leak sensor digital input

Emergency surface procedure:
  1. Publish /auv/safety/emergency_surface = True
  2. Call /auv/thrusters/arm to keep armed
  3. Publish override wrench: Fz = max upward thrust, all other DOFs = 0
  4. Hold until surface detected (depth < threshold) then disarm

TODO (Phase 7 - Safety):
  - Implement topic rate monitoring for /auv/sensors/imu/raw
  - Implement emergency surface procedure
  - Add max depth limit check
  - Add leak detection input (GPIO or dedicated topic)
"""

from auv_msgs.msg import AuvState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('imu_timeout_s', 2.0)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('check_rate_hz', 5.0)

        ns = self.get_parameter('namespace').value
        self._imu_timeout_s = self.get_parameter('imu_timeout_s').value
        self._max_depth_m = self.get_parameter('max_depth_m').value

        self._emergency_active = False
        self._last_imu_stamp = None
        self._latest_state: AuvState | None = None

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub_state = self.create_subscription(
            AuvState, f'/{ns}/state/auv_state', self._state_callback, 10)

        self._pub_emergency = self.create_publisher(
            Bool, f'/{ns}/safety/emergency_surface', qos_latched)

        check_hz = self.get_parameter('check_rate_hz').value
        self._timer = self.create_timer(1.0 / check_hz, self._check_safety)

        # Publish initial non-emergency state
        self._publish_emergency(False)
        self.get_logger().info('Safety monitor started.')

    def _state_callback(self, msg: AuvState):
        self._latest_state = msg

    def _check_safety(self):
        if self._latest_state is None:
            return

        # Depth limit check
        if (self._latest_state.depth_sensor_valid and
                self._latest_state.depth_m > self._max_depth_m):
            self.get_logger().error(
                f'SAFETY: Depth {self._latest_state.depth_m:.1f}m exceeds limit '
                f'{self._max_depth_m:.1f}m! EMERGENCY SURFACE.'
            )
            self._trigger_emergency()

    def _trigger_emergency(self):
        if not self._emergency_active:
            self._emergency_active = True
            self._publish_emergency(True)
            self.get_logger().error('EMERGENCY SURFACE TRIGGERED.')
            # TODO: send upward wrench command

    def _publish_emergency(self, active: bool):
        msg = Bool()
        msg.data = active
        self._pub_emergency.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
