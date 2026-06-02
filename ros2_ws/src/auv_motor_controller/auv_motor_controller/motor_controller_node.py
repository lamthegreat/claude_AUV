"""
Motor Controller Bridge Node.

Subscribes to body-frame wrench commands from the controller,
runs the Thruster Allocation Matrix to compute per-thruster PWM values,
and publishes ThrusterCommand messages toward the Teensy 4.0 via microROS.

Subscriptions:
  /auv/control/wrench_output    geometry_msgs/WrenchStamped  (from auv_controller)
  /auv/thrusters/armed          std_msgs/Bool                (arm state)

Publications:
  /auv/thrusters/commands       auv_msgs/ThrusterCommand     (→ Teensy 4.0 via microROS)

Services:
  /auv/thrusters/arm            auv_msgs/ArmThrusters
  /auv/thrusters/set_allocation auv_msgs/SetThrusterAllocation

TODO (Phase 4 - Motor Controller Bridge):
  - Load TAM config from parameter file path
  - Wire up the ArmThrusters service properly
  - Add /auv/thrusters/feedback subscription to monitor ESC state
  - Add watchdog: if no wrench received in N ms, publish disarmed command
"""

import os

from auv_motor_controller.thruster_allocator import ThrusterAllocator
from auv_msgs.msg import ThrusterCommand
from auv_msgs.srv import ArmThrusters, SetThrusterAllocation
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import yaml


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Parameters
        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('thruster_config_path', '')
        self.declare_parameter('command_timeout_ms', 200)
        self.declare_parameter('publish_rate_hz', 50.0)

        ns = self.get_parameter('namespace').value
        config_path = self.get_parameter('thruster_config_path').value
        self._timeout_ms = self.get_parameter('command_timeout_ms').value

        self._armed = False
        self._last_wrench_stamp = None
        self._allocator = ThrusterAllocator()

        # Load thruster config
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                config = yaml.safe_load(f)
            self._allocator.load_config(config)
            self.get_logger().info(
                f'Loaded TAM: {self._allocator.num_thrusters} thrusters '
                f'({", ".join(self._allocator.thruster_names)})'
            )
        else:
            self.get_logger().warn(
                'No thruster config loaded. Set thruster_config_path parameter. '
                'Motor controller will publish neutral PWM only.'
            )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscriptions
        self._sub_wrench = self.create_subscription(
            WrenchStamped,
            f'/{ns}/control/wrench_output',
            self._wrench_callback,
            qos_reliable,
        )

        # Publications
        self._pub_cmd = self.create_publisher(
            ThrusterCommand,
            f'/{ns}/thrusters/commands',
            qos_reliable,
        )
        self._pub_armed = self.create_publisher(
            Bool,
            f'/{ns}/thrusters/armed',
            qos_latched,
        )

        # Services
        self._srv_arm = self.create_service(
            ArmThrusters,
            f'/{ns}/thrusters/arm',
            self._arm_callback,
        )
        self._srv_set_alloc = self.create_service(
            SetThrusterAllocation,
            f'/{ns}/thrusters/set_allocation',
            self._set_allocation_callback,
        )

        # Watchdog timer — safes thrusters if wrench goes silent
        self._watchdog = self.create_timer(
            self._timeout_ms / 1000.0,
            self._watchdog_callback,
        )

        self.get_logger().info('Motor controller node started.')

    def _wrench_callback(self, msg: WrenchStamped):
        self._last_wrench_stamp = self.get_clock().now()

        if not self._armed:
            self._publish_neutral(armed=False)
            return

        if not self._allocator.is_loaded:
            self._publish_neutral(armed=False)
            return

        w = msg.wrench
        wrench_vec = [w.force.x, w.force.y, w.force.z,
                      w.torque.x, w.torque.y, w.torque.z]
        pwm_us = self._allocator.wrench_to_pwm(wrench_vec)
        normalized = [(p - 1500) / 400.0 for p in pwm_us]

        cmd = ThrusterCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.armed = True
        cmd.pwm_us = [float(p) for p in pwm_us]
        cmd.normalized = [float(n) for n in normalized]
        self._pub_cmd.publish(cmd)

    def _watchdog_callback(self):
        if self._last_wrench_stamp is None:
            return
        age_ms = (self.get_clock().now() - self._last_wrench_stamp).nanoseconds * 1e-6
        if age_ms > self._timeout_ms:
            if self._armed:
                self.get_logger().warn(
                    f'Wrench command timeout ({age_ms:.0f} ms). Publishing neutral.'
                )
            self._publish_neutral(armed=self._armed)

    def _publish_neutral(self, armed: bool):
        cmd = ThrusterCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.armed = armed
        n = self._allocator.num_thrusters if self._allocator.is_loaded else 6
        cmd.pwm_us = [1500.0] * n
        cmd.normalized = [0.0] * n
        self._pub_cmd.publish(cmd)

    def _arm_callback(self, request: ArmThrusters.Request,
                      response: ArmThrusters.Response):
        if request.enable and not request.reason:
            response.success = False
            response.message = 'Arm reason must be non-empty.'
            return response

        self._armed = request.enable
        armed_msg = Bool()
        armed_msg.data = self._armed
        self._pub_armed.publish(armed_msg)

        action = 'ARMED' if self._armed else 'DISARMED'
        self.get_logger().info(f'Thrusters {action}. Reason: {request.reason}')
        response.success = True
        response.message = f'Thrusters {action}.'
        return response

    def _set_allocation_callback(self, request: SetThrusterAllocation.Request,
                                 response: SetThrusterAllocation.Response):
        # TODO: implement runtime TAM update from service request
        response.success = False
        response.message = 'Runtime TAM update not yet implemented. Use config file.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
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
