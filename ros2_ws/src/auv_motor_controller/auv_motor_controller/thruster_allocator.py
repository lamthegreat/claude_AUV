"""
Thruster Allocation Matrix (TAM) Engine.

Converts a body-frame wrench [Fx, Fy, Fz, Mx, My, Mz] into per-thruster
normalized thrust commands using the Moore-Penrose pseudoinverse of the
Thruster Allocation Matrix.

Usage:
    allocator = ThrusterAllocator()
    allocator.load_config(config_dict)
    pwm_us = allocator.wrench_to_pwm([Fx, Fy, Fz, Mx, My, Mz])
"""

from __future__ import annotations

from typing import List

import numpy as np


class ThrusterAllocator:
    # PWM bounds (microseconds)
    PWM_MIN = 1100
    PWM_MAX = 1900
    PWM_NEUTRAL = 1500

    def __init__(self):
        self._tam: np.ndarray | None = None          # (6, N) allocation matrix
        self._tam_pinv: np.ndarray | None = None     # (N, 6) pseudoinverse
        self._num_thrusters: int = 0
        self._thruster_names: List[str] = []
        self._max_thrust_n: float = 1.0              # Per motor, for normalization

    def load_config(self, config: dict) -> None:
        """Load TAM from a parsed YAML config dict."""
        tc = config['thruster_allocation']
        self._num_thrusters = int(tc['num_thrusters'])
        self._thruster_names = tc['thruster_names']
        self._max_thrust_n = float(tc.get('max_thrust_n', 1.0))
        self.PWM_MIN = int(tc.get('pwm_min_us', 1100))
        self.PWM_MAX = int(tc.get('pwm_max_us', 1900))
        self.PWM_NEUTRAL = int(tc.get('pwm_neutral_us', 1500))

        # allocation_matrix in YAML is list of 6 rows, each with N columns
        rows = tc['allocation_matrix']
        self._tam = np.array(rows, dtype=np.float64)  # shape (6, N)

        if self._tam.shape != (6, self._num_thrusters):
            raise ValueError(
                f'TAM shape mismatch: expected (6, {self._num_thrusters}), '
                f'got {self._tam.shape}'
            )

        self._tam_pinv = np.linalg.pinv(self._tam)   # shape (N, 6)

    @property
    def is_loaded(self) -> bool:
        return self._tam_pinv is not None

    @property
    def num_thrusters(self) -> int:
        return self._num_thrusters

    @property
    def thruster_names(self) -> List[str]:
        return list(self._thruster_names)

    def wrench_to_pwm(self, wrench: List[float]) -> List[int]:
        """
        Convert a body-frame wrench to PWM values for each thruster.

        The wrench is [Fx, Fy, Fz, Mx, My, Mz] in Newtons and Newton-metres.
        Returns a list of PWM values in microseconds, one per thruster, with
        all values clamped to [PWM_MIN, PWM_MAX].
        """
        if not self.is_loaded:
            raise RuntimeError('ThrusterAllocator not loaded. Call load_config() first.')

        w = np.array(wrench, dtype=np.float64)
        thrust_n = self._tam_pinv @ w          # (N,) thrust per thruster in Newtons

        # Normalize by max thrust to get [-1, 1] command
        normalized = thrust_n / self._max_thrust_n

        # Scale-down if any thruster exceeds limits (preserve direction ratios)
        max_abs = np.max(np.abs(normalized))
        if max_abs > 1.0:
            normalized /= max_abs

        # Convert [-1, 1] → PWM microseconds
        half_range = (self.PWM_MAX - self.PWM_NEUTRAL)
        pwm_float = self.PWM_NEUTRAL + normalized * half_range
        pwm_us = [int(np.clip(v, self.PWM_MIN, self.PWM_MAX)) for v in pwm_float]

        return pwm_us

    def normalized_to_pwm(self, normalized: List[float]) -> List[int]:
        """Convert normalized [-1, 1] per-thruster commands directly to PWM."""
        half_range = (self.PWM_MAX - self.PWM_NEUTRAL)
        pwm_us = [
            int(np.clip(self.PWM_NEUTRAL + n * half_range, self.PWM_MIN, self.PWM_MAX))
            for n in normalized
        ]
        return pwm_us
