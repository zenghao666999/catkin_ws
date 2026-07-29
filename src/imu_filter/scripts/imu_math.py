#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU math utilities -- pure Python port of imu_filter/math_utils.h."""

import math
import struct


def inv_sqrt(x: float) -> float:
    """Fast inverse square root (Quake III algorithm).

    Uses struct to emulate the C union bit-cast.
    Matches imu_filter::invSqrt behavior.
    """
    if x <= 0.0:
        if x == 0.0:
            return float("inf")
        return float("nan")

    xhalf = 0.5 * x
    i = struct.unpack("i", struct.pack("f", x))[0]
    i = 0x5f3759df - (i >> 1)
    x = struct.unpack("f", struct.pack("i", i))[0]
    x = x * (1.5 - xhalf * x * x)
    return x


def quat_to_euler(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion (x,y,z,w) to Euler angles (roll,pitch,yaw) in radians.

    Matches imu_filter::quatToEuler behavior.
    """
    t0 = 2.0 * (qw * qz + qx * qy)
    t1 = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t0, t1)

    t2 = 2.0 * (qw * qy - qz * qx)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = 2.0 * (qw * qx + qy * qz)
    t4 = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t3, t4)

    return roll, pitch, yaw


def calculate_bias(sum_x: float, sum_y: float, sum_z: float, count: float):
    """Calculate gyroscope bias (sample mean).

    Matches imu_filter::calculateBias behavior.
    Returns (None,None,None) when count == 0.
    """
    if count == 0.0:
        return None, None, None
    return sum_x / count, sum_y / count, sum_z / count


def madgwick_ahrs_update_imu(gx: float, gy: float, gz: float,
                             ax: float, ay: float, az: float,
                             beta: float, sample_freq: float,
                             q0: float, q1: float, q2: float, q3: float):
    """Madgwick AHRS filter single update (6-axis IMU).

    Matches C++ MadgwickAHRSupdateIMU behavior.
    Gyroscope units: rad/s. Accelerometer: any (normalised internally).
    """
    q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        recip = inv_sqrt(ax * ax + ay * ay + az * az)
        ax *= recip
        ay *= recip
        az *= recip

        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = (_4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1
              - _2q0 * ay - _4q1 + _8q1 * q1q1
              + _8q1 * q2q2 + _4q1 * az)
        s2 = (4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3
              - _2q3 * ay - _4q2 + _8q2 * q1q1
              + _8q2 * q2q2 + _4q2 * az)
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay


        # (avoids NaN from 0.0 * inv_sqrt(0.0) = 0.0 * inf)
        grad_zero = (s0 == 0.0 and s1 == 0.0 and s2 == 0.0 and s3 == 0.0)
        if not grad_zero:
            recip = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            s0 *= recip
            s1 *= recip
            s2 *= recip
            s3 *= recip

        q_dot1 -= beta * s0
        q_dot2 -= beta * s1
        q_dot3 -= beta * s2
        q_dot4 -= beta * s3

    dt = 1.0 / sample_freq
    q0 += q_dot1 * dt
    q1 += q_dot2 * dt
    q2 += q_dot3 * dt
    q3 += q_dot4 * dt

    recip = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 *= recip
    q1 *= recip
    q2 *= recip
    q3 *= recip

    return q0, q1, q2, q3


def mahony_ahrs_update_imu(gx: float, gy: float, gz: float,
                           ax: float, ay: float, az: float,
                           two_kp: float, two_ki: float, dt: float,
                           ex_int: float, ey_int: float, ez_int: float,
                           q0: float, q1: float, q2: float, q3: float):
    """Mahony AHRS filter single update (6-axis IMU).

    Matches C++ MahonyAHRSupdateIMU behavior.
    Gyroscope units: rad/s. Accelerometer: any (normalised internally).
    """
    if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        recip = inv_sqrt(ax * ax + ay * ay + az * az)
        ax *= recip
        ay *= recip
        az *= recip

        half_vx = q1 * q3 - q0 * q2
        half_vy = q0 * q1 + q2 * q3
        half_vz = q0 * q0 - 0.5 + q3 * q3

        half_ex = ay * half_vz - az * half_vy
        half_ey = az * half_vx - ax * half_vz
        half_ez = ax * half_vy - ay * half_vx

        if two_ki > 0.0:
            ex_int += two_ki * half_ex * dt
            ey_int += two_ki * half_ey * dt
            ez_int += two_ki * half_ez * dt
        else:
            ex_int = 0.0
            ey_int = 0.0
            ez_int = 0.0

        gx += two_kp * half_ex
        gy += two_kp * half_ey
        gz += two_kp * half_ez

        if two_ki > 0.0:
            gx += ex_int
            gy += ey_int
            gz += ez_int

    theta_x = 0.5 * gx * dt
    theta_y = 0.5 * gy * dt
    theta_z = 0.5 * gz * dt

    dq0 = -q1 * theta_x - q2 * theta_y - q3 * theta_z
    dq1 = q0 * theta_x + q2 * theta_z - q3 * theta_y
    dq2 = q0 * theta_y - q1 * theta_z + q3 * theta_x
    dq3 = q0 * theta_z + q1 * theta_y - q2 * theta_x

    q0 += dq0
    q1 += dq1
    q2 += dq2
    q3 += dq3

    recip = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 *= recip
    q1 *= recip
    q2 *= recip
    q3 *= recip

    return q0, q1, q2, q3, ex_int, ey_int, ez_int