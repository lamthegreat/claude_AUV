"""
AUV Controller Node.

Implements cascaded PID controllers for depth, heading, and velocity.
Converts operator setpoints into body-frame WrenchStamped commands
which the motor controller bridge then maps to thruster PWM values.

Control modes (from AuvState constants):
  MANUAL (0)       - Pass-through: setpoint twist → wrench directly
  DEPTH_HOLD (1)   - PID on depth error; heading and surge from operator
  HEADING_HOLD (2) - PID on heading error; depth and surge from operator
  FULL_AUTO (3)    - All axes under PID control from mission setpoints

Subscriptions:
  /auv/state/auv_state          auv_msgs/AuvState
  /auv/control/setpoint/pose    geometry_msgs/PoseStamped   (auto modes)
  /auv/control/setpoint/twist   geometry_msgs/TwistStamped  (manual / teleop)

Publications:
  /auv/control/wrench_output    geometry_msgs/WrenchStamped

Services:
  /auv/control/set_mode         auv_msgs/SetControlMode

TODO (Phase 6 - Controller):
  - Implement PID class with anti-windup and derivative filtering
  - Wire depth PID: error = setpoint_depth - state.depth_m
  - Wire heading PID: error = wrap_to_pi(setpoint_yaw - current_yaw)
  - Wire surge/sway velocity control
  - Add feed-forward from setpoint twist in MANUAL mode
"""

from auv_msgs.msg import AuvState
from auv_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
import rclpy
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('publish_rate_hz', 50.0)
        # PID gains — tune empirically in the pool
        self.declare_parameter('depth_kp', 5.0)
        self.declare_parameter('depth_ki', 0.1)
        self.declare_parameter('depth_kd', 1.0)
        self.declare_parameter('heading_kp', 3.0)
        self.declare_parameter('heading_ki', 0.05)
        self.declare_parameter('heading_kd', 0.5)

        ns = self.get_parameter('namespace').value
        self._control_mode = AuvState.MANUAL

        self._latest_state: AuvState | None = None
        self._setpoint_pose: PoseStamped | None = None
        self._setpoint_twist: TwistStamped | None = None

        self._sub_state = self.create_subscription(
            AuvState, f'/{ns}/state/auv_state', self._state_callback, 10)
        self._sub_pose_sp = self.create_subscription(
            PoseStamped, f'/{ns}/control/setpoint/pose', self._pose_sp_callback, 10)
        self._sub_twist_sp = self.create_subscription(
            TwistStamped, f'/{ns}/control/setpoint/twist', self._twist_sp_callback, 10)

        self._pub_wrench = self.create_publisher(
            WrenchStamped, f'/{ns}/control/wrench_output', 10)

        self._srv_set_mode = self.create_service(
            SetControlMode, f'/{ns}/control/set_mode', self._set_mode_callback)

        rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info('Controller node started (MANUAL mode).')

    def _state_callback(self, msg: AuvState):
        self._latest_state = msg

    def _pose_sp_callback(self, msg: PoseStamped):
        self._setpoint_pose = msg

    def _twist_sp_callback(self, msg: TwistStamped):
        self._setpoint_twist = msg

    def _control_loop(self):
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = 'base_link'

        if self._control_mode == AuvState.MANUAL:
            if self._setpoint_twist is not None:
                t = self._setpoint_twist.twist
                wrench.wrench.force.x = t.linear.x
                wrench.wrench.force.y = t.linear.y
                wrench.wrench.force.z = t.linear.z
                wrench.wrench.torque.x = t.angular.x
                wrench.wrench.torque.y = t.angular.y
                wrench.wrench.torque.z = t.angular.z

        # TODO: implement DEPTH_HOLD, HEADING_HOLD, FULL_AUTO modes

        self._pub_wrench.publish(wrench)

    def _set_mode_callback(self, request: SetControlMode.Request,
                           response: SetControlMode.Response):
        valid_modes = [AuvState.MANUAL, AuvState.DEPTH_HOLD,
                       AuvState.HEADING_HOLD, AuvState.FULL_AUTO]
        if request.mode not in valid_modes:
            response.success = False
            response.message = f'Invalid mode {request.mode}.'
            return response

        response.previous_mode = self._control_mode
        self._control_mode = request.mode
        mode_names = {0: 'MANUAL', 1: 'DEPTH_HOLD', 2: 'HEADING_HOLD', 3: 'FULL_AUTO'}
        self.get_logger().info(f'Control mode → {mode_names[self._control_mode]}')
        response.success = True
        response.message = f'Mode set to {mode_names[self._control_mode]}.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
