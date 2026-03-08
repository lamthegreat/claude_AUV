"""
Mission Node

Executes YAML-defined mission sequences by sending action goals to
the Dive and NavigateTo action servers (future), or by publishing
setpoints directly to the controller in early phases.

TODO (Phase 9 - Navigation):
  - Load mission YAML from parameter
  - Implement state machine: IDLE → EXECUTING → COMPLETE/ABORTED
  - Send Dive action goals for depth changes
  - Send NavigateTo action goals for waypoint travel
  - Monitor AuvState for safety triggers
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('mission_file', '')

        ns = self.get_parameter('namespace').value
        self._mission_file = self.get_parameter('mission_file').value

        self._pub_pose_sp = self.create_publisher(
            PoseStamped, f'/{ns}/control/setpoint/pose', 10)

        self.get_logger().info('Mission node started (IDLE — no mission loaded).')

        if self._mission_file:
            self.get_logger().info(f'Mission file: {self._mission_file}')
        else:
            self.get_logger().info('No mission_file parameter set.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
