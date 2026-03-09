"""
Unit tests for StateEstimatorNode._should_use_9dof

Tests the orientation-source selection logic in isolation (no ROS2 spin required).
Thresholds under test: mag_calib_min=2, heading_accuracy_threshold_rad=0.35 (~20°).
"""

import pytest
from auv_state_estimator.state_estimator_node import StateEstimatorNode

# Thresholds that match the node defaults
MAG_CALIB_MIN = 2
HEADING_THRESH = 0.35  # rad, ~20 degrees

should = StateEstimatorNode._should_use_9dof


# ── 9-DOF selected ────────────────────────────────────────────────────────────

def test_9dof_good_mag_low_heading_error():
    assert should(calibration_mag=3, heading_accuracy_rad=0.10,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is True


def test_9dof_at_exact_mag_calib_minimum():
    # calibration_mag == mag_calib_min should still select 9-DOF (>= boundary)
    assert should(calibration_mag=2, heading_accuracy_rad=0.20,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is True


def test_9dof_heading_accuracy_just_below_threshold():
    assert should(calibration_mag=3, heading_accuracy_rad=0.349,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is True


# ── Game RV fallback selected ─────────────────────────────────────────────────

def test_game_rv_mag_calib_too_low():
    assert should(calibration_mag=1, heading_accuracy_rad=0.10,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


def test_game_rv_uncalibrated_mag():
    assert should(calibration_mag=0, heading_accuracy_rad=0.10,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


def test_game_rv_heading_accuracy_exceeds_threshold():
    assert should(calibration_mag=3, heading_accuracy_rad=0.40,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


def test_game_rv_heading_accuracy_at_exact_threshold():
    # heading_accuracy_rad == threshold should NOT select 9-DOF (strict <)
    assert should(calibration_mag=3, heading_accuracy_rad=0.35,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


def test_game_rv_both_conditions_bad():
    assert should(calibration_mag=0, heading_accuracy_rad=1.57,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


# ── Parameter sensitivity ─────────────────────────────────────────────────────

def test_custom_thresholds_stricter_mag():
    # Raise the mag minimum to 3 — calib_mag=2 should now fall back
    assert should(calibration_mag=2, heading_accuracy_rad=0.10,
                  mag_calib_min=3,
                  heading_accuracy_threshold_rad=HEADING_THRESH) is False


def test_custom_thresholds_tighter_heading():
    # Tighten heading threshold to 0.17 rad (~10°) — 0.20 should now fall back
    assert should(calibration_mag=3, heading_accuracy_rad=0.20,
                  mag_calib_min=MAG_CALIB_MIN,
                  heading_accuracy_threshold_rad=0.17) is False
