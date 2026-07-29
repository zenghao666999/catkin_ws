# -*- coding: utf-8 -*-
"""pytest tests for imu_math -- IMU core algorithm verification.

Tests cover:
  - inv_sqrt: basic values, boundaries (zero, negative, large, small)
  - quat_to_euler: identity, cardinal rotations, gimbal lock
  - calculate_bias: normal, edge cases (zero count, single sample, negative count, large values)
  - madgwick_ahrs_update_imu: static convergence, pure-yaw, zero-accel, NaN safety
  - mahony_ahrs_update_imu: static convergence, PI feedback, integral windup, zero-accel

All functions under test correspond to their C++ counterparts in imu_filter/math_utils.h,
Madgwick_filter.cpp, and Mahony_filter.cpp.
"""

import math
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from imu_math import (
    inv_sqrt,
    quat_to_euler,
    calculate_bias,
    madgwick_ahrs_update_imu,
    mahony_ahrs_update_imu,
)

# Tolerance for quaternion norm checks.
# The Quake III fast inverse sqrt has ~0.17% error at x=1.0, so after
# one normalization pass a unit quaternion shrinks to ~0.9983.
NORM_TOL = 0.005


# ===========================================================================
# inv_sqrt
# ===========================================================================

class TestInvSqrt:
    """Fast inverse square root (Quake III algorithm)."""

    @pytest.mark.parametrize("x, expected", [
        (4.0,   0.5),
        (1.0,   1.0),
        (0.25,  2.0),
        (100.0, 0.1),
        (25.0,  0.2),
        (16.0,  0.25),
    ])
    def test_positive_numbers_within_tolerance(self, x, expected):
        result = inv_sqrt(x)
        assert result == pytest.approx(expected, rel=0.002)

    def test_large_number_returns_small_value(self):
        result = inv_sqrt(1_000_000.0)
        assert result == pytest.approx(0.001, rel=0.002)

    def test_very_small_positive(self):
        result = inv_sqrt(1e-10)
        assert result > 0
        assert not math.isnan(result)

    def test_zero_returns_inf(self):
        result = inv_sqrt(0.0)
        assert math.isinf(result)

    def test_negative_returns_nan(self):
        result = inv_sqrt(-1.0)
        assert math.isnan(result)

    def test_negative_large_returns_nan(self):
        result = inv_sqrt(-100.0)
        assert math.isnan(result)

    def test_larger_input_smaller_output(self):
        a, b = inv_sqrt(2.0), inv_sqrt(8.0)
        assert a > b


# ===========================================================================
# quat_to_euler
# ===========================================================================

class TestQuatToEuler:
    """Quaternion to Euler (roll, pitch, yaw) conversion."""

    def test_identity_quaternion(self):
        r, p, y = quat_to_euler(0.0, 0.0, 0.0, 1.0)
        assert r == pytest.approx(0.0, abs=1e-10)
        assert p == pytest.approx(0.0, abs=1e-10)
        assert y == pytest.approx(0.0, abs=1e-10)

    def test_90_degree_yaw(self):
        half = math.pi / 4
        r, p, y = quat_to_euler(0.0, 0.0, math.sin(half), math.cos(half))
        assert r == pytest.approx(0.0, abs=1e-6)
        assert p == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(math.pi / 2, abs=1e-6)

    def test_90_degree_pitch(self):
        """At pitch=90 deg, roll and yaw are degenerate (gimbal lock).
        Roll may be pi or 0 depending on implementation. Only pitch is well-defined."""
        half = math.pi / 4
        r, p, y = quat_to_euler(0.0, math.sin(half), 0.0, math.cos(half))
        assert p == pytest.approx(math.pi / 2, abs=1e-6)
        # roll is degenerate at pitch=90; do not assert on it

    def test_90_degree_roll(self):
        half = math.pi / 4
        r, p, y = quat_to_euler(math.sin(half), 0.0, 0.0, math.cos(half))
        assert r == pytest.approx(math.pi / 2, abs=1e-6)
        assert p == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_180_degree_yaw(self):
        r, p, y = quat_to_euler(0.0, 0.0, 1.0, 0.0)
        assert y == pytest.approx(math.pi, abs=1e-6) or y == pytest.approx(-math.pi, abs=1e-6)

    def test_gimbal_lock_pitch_90(self):
        half = math.pi / 4
        r, p, y = quat_to_euler(0.0, math.sin(half), 0.0, math.cos(half))
        assert abs(p) == pytest.approx(math.pi / 2, abs=1e-6)

    def test_no_nan_on_normalised_quat(self):
        r, p, y = quat_to_euler(0.5, 0.5, 0.5, 0.5)
        assert not any(math.isnan(v) for v in (r, p, y))

    def test_roundtrip_consistency(self):
        half = math.radians(15)
        r, p, y = quat_to_euler(0.0, 0.0, math.sin(half), math.cos(half))
        assert y == pytest.approx(math.radians(30), abs=1e-6)


# ===========================================================================
# calculate_bias
# ===========================================================================

class TestCalculateBias:
    """Gyroscope bias (sample mean) calculation."""

    def test_multiple_samples_all_axes(self):
        bx, by, bz = calculate_bias(10.0, 20.0, 30.0, 5.0)
        assert bx == 2.0
        assert by == 4.0
        assert bz == 6.0

    def test_single_sample_equals_input(self):
        bx, by, bz = calculate_bias(3.5, -2.1, 0.0, 1.0)
        assert bx == 3.5
        assert by == -2.1
        assert bz == 0.0

    def test_zero_count_returns_none(self):
        result = calculate_bias(100.0, 200.0, 300.0, 0.0)
        assert result == (None, None, None)

    def test_negative_values(self):
        bx, by, bz = calculate_bias(-5.0, 10.0, -15.0, 5.0)
        assert bx == -1.0
        assert by == 2.0
        assert bz == -3.0

    def test_large_count(self):
        bx, by, bz = calculate_bias(1e6, 2e6, -3e6, 1e6)
        assert bx == pytest.approx(1.0, rel=1e-10)
        assert by == pytest.approx(2.0, rel=1e-10)
        assert bz == pytest.approx(-3.0, rel=1e-10)

    def test_negative_count_does_not_crash(self):
        result = calculate_bias(10.0, 20.0, 30.0, -1.0)
        assert result is not None

    def test_zero_sum_zero_count(self):
        result = calculate_bias(0.0, 0.0, 0.0, 0.0)
        assert result == (None, None, None)


# ===========================================================================
# madgwick_ahrs_update_imu
# ===========================================================================

class TestMadgwickFilter:
    """Madgwick AHRS IMU filter step tests."""

    BETA = 0.1
    FREQ = 400.0

    def _make_state(self):
        return 1.0, 0.0, 0.0, 0.0

    def test_static_no_rotation_quat_stays_near_identity(self):
        q = self._make_state()
        for _ in range(100):
            q = madgwick_ahrs_update_imu(
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
                self.BETA, self.FREQ, *q,
            )
        r, p, y = quat_to_euler(q[1], q[2], q[3], q[0])
        assert abs(r) < 0.01
        assert abs(p) < 0.01

    def test_static_gravity_aligned_with_quat(self):
        """When gravity matches the quaternion-estimated direction, error ~ 0.
        Note: normalization via fast inv_sqrt has ~0.17% error, so the norm
        drifts slightly below 1.0 after each cycle."""
        q = self._make_state()
        for _ in range(10):
            q = madgwick_ahrs_update_imu(
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0,
                self.BETA, self.FREQ, *q,
            )
        norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        assert norm == pytest.approx(1.0, abs=NORM_TOL)

    def test_pure_yaw_integrates_correctly(self):
        q = self._make_state()
        yaw_rate = 1.0
        for _ in range(400):
            q = madgwick_ahrs_update_imu(
                0.0, 0.0, yaw_rate,
                0.0, 0.0, 1.0,
                self.BETA, self.FREQ, *q,
            )
        r, p, y = quat_to_euler(q[1], q[2], q[3], q[0])
        assert y == pytest.approx(1.0, abs=0.05)
        assert abs(r) < 0.05
        assert abs(p) < 0.05

    def test_zero_acceleration_no_feedback(self):
        q = self._make_state()
        q2 = madgwick_ahrs_update_imu(
            0.1, 0.2, 0.3,
            0.0, 0.0, 0.0,
            self.BETA, self.FREQ, *q,
        )
        norm = math.sqrt(q2[0]**2 + q2[1]**2 + q2[2]**2 + q2[3]**2)
        assert norm == pytest.approx(1.0, abs=NORM_TOL)

    def test_zero_input_does_not_produce_nan(self):
        q = madgwick_ahrs_update_imu(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            self.BETA, self.FREQ, 1.0, 0.0, 0.0, 0.0,
        )
        assert not any(math.isnan(v) for v in q)

    def test_convergence_gravity_along_x(self):
        """Gravity sensed along body +x means the sensor is pitched back ~90 deg.
        The Madgwick filter converges to a pitch of approx -pi/2."""
        q = self._make_state()
        for _ in range(2000):
            q = madgwick_ahrs_update_imu(
                0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                self.BETA, self.FREQ, *q,
            )
        r, p, y = quat_to_euler(q[1], q[2], q[3], q[0])
        # Should converge: pitch near -pi/2 (or pi/2 depending on convention)
        assert abs(p) > 0.5

    def test_convergence_gravity_along_y(self):
        """Gravity sensed along body +y means the sensor is rolled ~90 deg."""
        q = self._make_state()
        for _ in range(2000):
            q = madgwick_ahrs_update_imu(
                0.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                self.BETA, self.FREQ, *q,
            )
        r, p, y = quat_to_euler(q[1], q[2], q[3], q[0])
        assert abs(r) > 0.5

    def test_quaternion_stays_normalised_over_time(self):
        q = self._make_state()
        gx, gy, gz = 0.01, 0.02, -0.03
        for _ in range(500):
            q = madgwick_ahrs_update_imu(
                gx, gy, gz, 0.0, 0.0, 1.0,
                self.BETA, self.FREQ, *q,
            )
            norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
            assert norm == pytest.approx(1.0, abs=NORM_TOL)


# ===========================================================================
# mahony_ahrs_update_imu
# ===========================================================================

class TestMahonyFilter:
    """Mahony AHRS IMU filter step tests."""

    TWO_KP = 2.0 * 0.5
    TWO_KI = 2.0 * 0.0
    DT = 1.0 / 400.0

    def _make_state(self):
        return 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0

    def test_static_no_rotation_quat_stays_near_identity(self):
        ex, ey, ez, q0, q1, q2, q3 = self._make_state()
        for _ in range(100):
            q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                self.TWO_KP, self.TWO_KI, self.DT,
                ex, ey, ez, q0, q1, q2, q3,
            )
        r, p, y = quat_to_euler(q1, q2, q3, q0)
        assert abs(r) < 0.01
        assert abs(p) < 0.01

    def test_integral_feedback_accumulates(self):
        two_ki = 0.1
        two_kp = 2.0 * 0.5
        ex, ey, ez, q0, q1, q2, q3 = 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
        for _ in range(50):
            _, _, _, _, ex, ey, ez = mahony_ahrs_update_imu(
                0.0, 0.0, 0.0, 0.5, 0.0, 0.87,
                two_kp, two_ki, self.DT,
                ex, ey, ez, q0, q1, q2, q3,
            )
        assert any(abs(v) > 1e-6 for v in (ex, ey, ez))

    def test_integral_resets_when_ki_zero(self):
        ex, ey, ez, q0, q1, q2, q3 = 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0
        _, _, _, _, ex, ey, ez = mahony_ahrs_update_imu(
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            self.TWO_KP, 0.0, self.DT,
            ex, ey, ez, q0, q1, q2, q3,
        )
        assert ex == 0.0
        assert ey == 0.0
        assert ez == 0.0

    def test_zero_acceleration_no_feedback(self):
        ex, ey, ez, q0, q1, q2, q3 = self._make_state()
        q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
            0.1, 0.2, 0.3, 0.0, 0.0, 0.0,
            self.TWO_KP, self.TWO_KI, self.DT,
            ex, ey, ez, q0, q1, q2, q3,
        )
        norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        assert norm == pytest.approx(1.0, abs=NORM_TOL)

    def test_zero_input_does_not_produce_nan(self):
        q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            self.TWO_KP, self.TWO_KI, self.DT,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        )
        for v in (q0, q1, q2, q3, ex, ey, ez):
            assert not math.isnan(v)

    def test_quaternion_stays_normalised_over_time(self):
        ex, ey, ez, q0, q1, q2, q3 = self._make_state()
        gx, gy, gz = 0.01, -0.01, 0.02
        for _ in range(500):
            q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
                gx, gy, gz, 0.0, 0.0, 1.0,
                self.TWO_KP, self.TWO_KI, self.DT,
                ex, ey, ez, q0, q1, q2, q3,
            )
            norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
            assert norm == pytest.approx(1.0, abs=NORM_TOL)

    def test_pure_yaw_integrates(self):
        ex, ey, ez, q0, q1, q2, q3 = self._make_state()
        yaw_rate = 1.0
        for _ in range(400):
            q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
                0.0, 0.0, yaw_rate, 0.0, 0.0, 1.0,
                self.TWO_KP, self.TWO_KI, self.DT,
                ex, ey, ez, q0, q1, q2, q3,
            )
        r, p, y = quat_to_euler(q1, q2, q3, q0)
        assert y == pytest.approx(1.0, abs=0.05)
        assert abs(r) < 0.05
        assert abs(p) < 0.05


# ===========================================================================
# Cross-filter consistency checks
# ===========================================================================

class TestCrossFilterConsistency:
    """Both filters should behave similarly for basic scenarios."""

    def test_static_convergence_parity(self):
        qm = (1.0, 0.0, 0.0, 0.0)
        for _ in range(100):
            qm = madgwick_ahrs_update_imu(0, 0, 0, 0, 0, 1, 0.1, 400.0, *qm)
        r_m, p_m, _ = quat_to_euler(qm[1], qm[2], qm[3], qm[0])

        ex, ey, ez = 0, 0, 0
        q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
        for _ in range(100):
            q0, q1, q2, q3, ex, ey, ez = mahony_ahrs_update_imu(
                0, 0, 0, 0, 0, 1, 1.0, 0.0, 1.0 / 400.0,
                ex, ey, ez, q0, q1, q2, q3,
            )
        r_h, p_h, _ = quat_to_euler(q1, q2, q3, q0)

        assert abs(r_m) < 0.01 and abs(p_m) < 0.01
        assert abs(r_h) < 0.01 and abs(p_h) < 0.01