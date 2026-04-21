import math

from auv_sim.bno085_imu_sim_node import normalize_quaternion, profile_state, quaternion_from_rpy


def test_quaternion_from_rpy_is_normalized():
    quat = quaternion_from_rpy(math.radians(10.0), math.radians(-5.0), math.radians(45.0))
    norm = math.sqrt(sum(component * component for component in quat))
    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-9)


def test_normalize_quaternion_handles_zero():
    quat = normalize_quaternion((0.0, 0.0, 0.0, 0.0))
    assert quat == (0.0, 0.0, 0.0, 1.0)


def test_yaw_rate_profile_increases_yaw():
    early = profile_state('yaw_rate', 0.5)
    late = profile_state('yaw_rate', 1.5)
    assert late.yaw > early.yaw
    assert math.isclose(early.yaw_dot, late.yaw_dot, rel_tol=0.0, abs_tol=1e-9)


def test_accel_burst_profile_generates_forward_acceleration():
    burst = profile_state('accel_burst', 0.75)
    coast = profile_state('accel_burst', 1.75)
    assert burst.linear_accel_body[0] > 0.0
    assert math.isclose(coast.linear_accel_body[0], 0.0, rel_tol=0.0, abs_tol=1e-9)
