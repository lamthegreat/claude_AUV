from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, MagneticField
from auv_msgs.msg import ImuExtended, AuvState


class Phase(Enum):
    WAIT_FOR_CONNECTIONS = auto()
    OBSERVE = auto()
    PASSED = auto()
    FAILED = auto()


@dataclass
class Observations:
    raw_count: int = 0
    ext_count: int = 0
    mag_count: int = 0
    state_count: int = 0
    raw_first_ns: int | None = None
    ext_first_ns: int | None = None
    state_first_ns: int | None = None
    last_ext_orientation_w: float = 0.0
    last_state_orientation_w: float = 0.0


class ImuStateSmokeRunner(Node):
    def __init__(self):
        super().__init__('imu_state_smoke_runner')

        self.declare_parameter('namespace', 'auv')
        self.declare_parameter('scenario_timeout_s', 8.0)
        self.declare_parameter('observation_window_s', 2.0)
        self.declare_parameter('min_raw_count', 120)
        self.declare_parameter('min_ext_count', 120)
        self.declare_parameter('min_mag_count', 60)
        self.declare_parameter('min_state_count', 20)

        self._namespace = str(self.get_parameter('namespace').value)
        self._scenario_timeout_s = float(self.get_parameter('scenario_timeout_s').value)
        self._observation_window_s = float(self.get_parameter('observation_window_s').value)
        self._min_raw_count = int(self.get_parameter('min_raw_count').value)
        self._min_ext_count = int(self.get_parameter('min_ext_count').value)
        self._min_mag_count = int(self.get_parameter('min_mag_count').value)
        self._min_state_count = int(self.get_parameter('min_state_count').value)

        self._phase = Phase.WAIT_FOR_CONNECTIONS
        self._observations = Observations()
        self._started_ns = self.get_clock().now().nanoseconds
        self._phase_started_ns = self._started_ns
        self._final_exit_code: int | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Imu, f'/{self._namespace}/sensors/imu/raw', self._raw_callback, qos)
        self.create_subscription(ImuExtended, f'/{self._namespace}/sensors/imu/extended', self._ext_callback, qos)
        self.create_subscription(MagneticField, f'/{self._namespace}/sensors/imu/magnetic_field', self._mag_callback, qos)
        self.create_subscription(AuvState, f'/{self._namespace}/state/auv_state', self._state_callback, qos)

        self._tick = self.create_timer(0.1, self._on_tick)
        self.get_logger().info('IMU/state smoke runner started.')

    def _raw_callback(self, _msg: Imu):
        self._observations.raw_count += 1
        if self._observations.raw_first_ns is None:
            self._observations.raw_first_ns = self.get_clock().now().nanoseconds

    def _ext_callback(self, msg: ImuExtended):
        self._observations.ext_count += 1
        self._observations.last_ext_orientation_w = msg.orientation.w
        if self._observations.ext_first_ns is None:
            self._observations.ext_first_ns = self.get_clock().now().nanoseconds
            self.get_logger().info('Observed first simulated ImuExtended message.')

    def _mag_callback(self, _msg: MagneticField):
        self._observations.mag_count += 1

    def _state_callback(self, msg: AuvState):
        self._observations.state_count += 1
        self._observations.last_state_orientation_w = msg.pose.orientation.w
        if self._observations.state_first_ns is None:
            self._observations.state_first_ns = self.get_clock().now().nanoseconds
            self.get_logger().info('Observed first AuvState publication from the state estimator.')

    def _on_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self._started_ns) / 1e9
        if elapsed_s > self._scenario_timeout_s:
            self._fail('Scenario timed out before communication checks completed.')
            return

        raw_subscribers = self.count_publishers(f'/{self._namespace}/sensors/imu/raw')
        ext_subscribers = self.count_publishers(f'/{self._namespace}/sensors/imu/extended')
        state_publishers = self.count_publishers(f'/{self._namespace}/state/auv_state')

        if self._phase == Phase.WAIT_FOR_CONNECTIONS:
            # The smoke runner waits for publishers to appear before counting traffic.
            if raw_subscribers >= 1 and ext_subscribers >= 1 and state_publishers >= 1:
                self._phase = Phase.OBSERVE
                self._phase_started_ns = now_ns
                self.get_logger().info('Required publishers detected. Observing topic traffic.')
            return

        if self._phase == Phase.OBSERVE:
            raw_consumer_count = self.count_subscribers(f'/{self._namespace}/sensors/imu/raw')
            ext_consumer_count = self.count_subscribers(f'/{self._namespace}/sensors/imu/extended')

            if raw_consumer_count < 2:
                self._fail(f'Expected IMU raw to have at least 2 subscribers, saw {raw_consumer_count}.')
                return
            if ext_consumer_count < 3:
                self._fail(f'Expected IMU extended to have at least 3 subscribers, saw {ext_consumer_count}.')
                return

            if (now_ns - self._phase_started_ns) / 1e9 < self._observation_window_s:
                return

            if self._observations.raw_count < self._min_raw_count:
                self._fail(f'Observed only {self._observations.raw_count} raw IMU messages.')
                return
            if self._observations.ext_count < self._min_ext_count:
                self._fail(f'Observed only {self._observations.ext_count} extended IMU messages.')
                return
            if self._observations.mag_count < self._min_mag_count:
                self._fail(f'Observed only {self._observations.mag_count} magnetometer messages.')
                return
            if self._observations.state_count < self._min_state_count:
                self._fail(f'Observed only {self._observations.state_count} state-estimator messages.')
                return
            if self._observations.ext_first_ns is None or self._observations.state_first_ns is None:
                self._fail('Missing ImuExtended or AuvState traffic.')
                return
            if self._observations.state_first_ns < self._observations.ext_first_ns:
                self._fail('State estimator published before the first extended IMU sample was observed.')
                return
            if not math.isclose(
                self._observations.last_ext_orientation_w,
                self._observations.last_state_orientation_w,
                rel_tol=0.0,
                abs_tol=0.05,
            ):
                self._fail('State estimator orientation does not track the simulated IMU orientation closely enough.')
                return

            self._pass(raw_consumer_count, ext_consumer_count)

    def _pass(self, raw_consumer_count: int, ext_consumer_count: int):
        self._phase = Phase.PASSED
        self._final_exit_code = 0
        self.get_logger().info(
            'IMU/state smoke test passed. '
            f'raw={self._observations.raw_count}, '
            f'extended={self._observations.ext_count}, '
            f'mag={self._observations.mag_count}, '
            f'state={self._observations.state_count}, '
            f'raw_subscribers={raw_consumer_count}, '
            f'extended_subscribers={ext_consumer_count}.'
        )

    def _fail(self, reason: str):
        self._phase = Phase.FAILED
        self._final_exit_code = 1
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = ImuStateSmokeRunner()

    exit_code = 1
    try:
        while rclpy.ok() and node._final_exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node._final_exit_code is not None:
            exit_code = node._final_exit_code
    except KeyboardInterrupt:
        node.get_logger().warn('IMU/state smoke runner interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
