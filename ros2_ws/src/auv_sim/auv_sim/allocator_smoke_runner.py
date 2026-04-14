"""
Allocator smoke-test runner.

Runs a deterministic hardware-free scenario against the motor controller node:
1. Publish a non-zero wrench while disarmed and verify neutral output.
2. Arm via the real /auv/thrusters/arm service.
3. Publish the same wrench and verify active thruster output.
4. Stop publishing and verify timeout returns output to neutral.

The process exits 0 on success and non-zero on failure, so it can be used both
locally and in CI.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import WrenchStamped
from auv_msgs.msg import ThrusterCommand
from auv_msgs.srv import ArmThrusters


class Phase(Enum):
    WAIT_FOR_SERVICE = auto()
    PREARM_CHECK = auto()
    ARMING = auto()
    ACTIVE_CHECK = auto()
    TIMEOUT_CHECK = auto()
    PASSED = auto()
    FAILED = auto()


@dataclass
class Observations:
    saw_disarmed_neutral: bool = False
    saw_armed_active: bool = False
    saw_timeout_neutral: bool = False
    command_count: int = 0


class AllocatorSmokeRunner(Node):
    def __init__(self):
        super().__init__('allocator_smoke_runner')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('scenario_timeout_s', 8.0)
        self.declare_parameter('command_timeout_ms', 200)
        self.declare_parameter('publish_period_s', 0.05)
        self.declare_parameter('prearm_duration_s', 0.35)
        self.declare_parameter('active_duration_s', 0.35)
        self.declare_parameter('timeout_wait_factor', 3.0)
        self.declare_parameter('surge_force_n', 1.5)

        ns = self.get_parameter('namespace').value
        self._scenario_timeout_s = float(self.get_parameter('scenario_timeout_s').value)
        self._command_timeout_ms = float(self.get_parameter('command_timeout_ms').value)
        self._publish_period_s = float(self.get_parameter('publish_period_s').value)
        self._prearm_duration_s = float(self.get_parameter('prearm_duration_s').value)
        self._active_duration_s = float(self.get_parameter('active_duration_s').value)
        self._timeout_wait_factor = float(self.get_parameter('timeout_wait_factor').value)
        self._surge_force_n = float(self.get_parameter('surge_force_n').value)

        self._phase = Phase.WAIT_FOR_SERVICE
        self._observations = Observations()
        self._arm_request_future: Future | None = None
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._scenario_started_ns = self._phase_started_ns
        self._final_exit_code: int | None = None
        self._failure_reason: str | None = None

        self._pub_wrench = self.create_publisher(
            WrenchStamped,
            f'/{ns}/control/wrench_output',
            10,
        )
        self._sub_cmd = self.create_subscription(
            ThrusterCommand,
            f'/{ns}/thrusters/commands',
            self._cmd_callback,
            10,
        )
        self._arm_client = self.create_client(
            ArmThrusters,
            f'/{ns}/thrusters/arm',
        )

        self._tick = self.create_timer(self._publish_period_s, self._on_tick)
        self.get_logger().info('Allocator smoke runner started.')

    def _cmd_callback(self, msg: ThrusterCommand):
        self._observations.command_count += 1
        is_neutral = self._is_neutral(msg)
        is_active = any(abs(pwm - 1500.0) > 1.0 for pwm in msg.pwm_us)
        in_timeout_phase = self._phase == Phase.TIMEOUT_CHECK

        if self._phase == Phase.PREARM_CHECK and (not msg.armed) and is_neutral:
            self._observations.saw_disarmed_neutral = True

        if self._phase == Phase.ACTIVE_CHECK and msg.armed and is_active:
            self._observations.saw_armed_active = True

        if in_timeout_phase and msg.armed and is_neutral:
            self._observations.saw_timeout_neutral = True

    def _on_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self._scenario_started_ns) / 1e9
        if elapsed_s > self._scenario_timeout_s:
            self._fail('Scenario timed out before completing.')
            return

        if self._phase == Phase.WAIT_FOR_SERVICE:
            if self._arm_client.wait_for_service(timeout_sec=0.0):
                self._transition(Phase.PREARM_CHECK, 'Arm service available. Starting pre-arm check.')
            return

        if self._phase == Phase.PREARM_CHECK:
            self._publish_surge_wrench()
            if self._phase_elapsed_s(now_ns) >= self._prearm_duration_s:
                if not self._observations.saw_disarmed_neutral:
                    self._fail('Did not observe neutral thruster output while disarmed.')
                    return
                self._send_arm_request()
            return

        if self._phase == Phase.ARMING:
            if self._arm_request_future is None:
                self._fail('Arm request future missing.')
                return
            if not self._arm_request_future.done():
                return

            try:
                response = self._arm_request_future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS client handling
                self._fail(f'Arm service call failed: {exc}')
                return

            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                self._fail(f'Arm service rejected request: {message}')
                return

            self._transition(Phase.ACTIVE_CHECK, 'Thrusters armed. Starting active command check.')
            return

        if self._phase == Phase.ACTIVE_CHECK:
            self._publish_surge_wrench()
            if self._phase_elapsed_s(now_ns) >= self._active_duration_s:
                if not self._observations.saw_armed_active:
                    self._fail('Did not observe active thruster output after arming.')
                    return
                self._transition(Phase.TIMEOUT_CHECK, 'Stopping wrench publishes to verify timeout safeing.')
            return

        if self._phase == Phase.TIMEOUT_CHECK:
            required_wait_s = (self._command_timeout_ms / 1000.0) * self._timeout_wait_factor
            if self._observations.saw_timeout_neutral:
                self._pass()
                return
            if self._phase_elapsed_s(now_ns) >= required_wait_s:
                self._fail('Did not observe neutral thruster output after command timeout.')
            return

    def _send_arm_request(self):
        request = ArmThrusters.Request()
        request.enable = True
        request.reason = 'allocator smoke test'
        self._arm_request_future = self._arm_client.call_async(request)
        self._transition(Phase.ARMING, 'Calling /auv/thrusters/arm for smoke test.')

    def _publish_surge_wrench(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.wrench.force.x = self._surge_force_n
        self._pub_wrench.publish(msg)

    @staticmethod
    def _is_neutral(msg: ThrusterCommand) -> bool:
        return all(math.isclose(pwm, 1500.0, abs_tol=1.0) for pwm in msg.pwm_us)

    def _phase_elapsed_s(self, now_ns: int) -> float:
        return (now_ns - self._phase_started_ns) / 1e9

    def _transition(self, next_phase: Phase, message: str):
        self._phase = next_phase
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(message)

    def _pass(self):
        self._phase = Phase.PASSED
        self._final_exit_code = 0
        self.get_logger().info(
            'Allocator smoke test passed. '
            f'Observed {self._observations.command_count} thruster command messages.'
        )

    def _fail(self, reason: str):
        self._phase = Phase.FAILED
        self._failure_reason = reason
        self._final_exit_code = 1
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = AllocatorSmokeRunner()

    exit_code = 1
    try:
        while rclpy.ok() and node._final_exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0 if node._final_exit_code == 0 else 1
    except KeyboardInterrupt:
        node.get_logger().warn('Allocator smoke runner interrupted.')
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)
