#ifndef IMU_FILTER_MATH_UTILS_H
#define IMU_FILTER_MATH_UTILS_H

#include <cmath>

namespace imu_filter {

/// Fast inverse square root (Quake III algorithm).
/// Reference: https://en.wikipedia.org/wiki/Fast_inverse_square_root
///
/// This is extracted from Madgwick_filter.cpp so it can be tested independently.
inline float invSqrt(float x) {
  float xhalf = 0.5f * x;
  union {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  u.x = u.x * (1.5f - xhalf * u.x * u.x);
  return u.x;
}

/// Convert a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw) in radians.
/// Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
///
/// Extracted from Madgwick_filter.cpp qua2Euler().
/// Returns false if the input would produce NaN (gimbal lock edge case).
inline bool quatToEuler(float qx, float qy, float qz, float qw,
                         float& roll, float& pitch, float& yaw) {
  // yaw (z-axis rotation)
  float t0 = 2.0f * (qw * qz + qx * qy);
  float t1 = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  float t2 = 2.0f * (qw * qy - qz * qx);
  t2 = (t2 > 1.0f) ? 1.0f : t2;
  t2 = (t2 < -1.0f) ? -1.0f : t2;
  pitch = std::asin(t2);

  // roll (x-axis rotation)
  float t3 = 2.0f * (qw * qx + qy * qz);
  float t4 = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = std::atan2(t3, t4);

  return true;
}

/// Calculate the mean gyroscope bias from accumulated samples.
/// Returns false when count is zero (no data received).
inline bool calculateBias(double sum_x, double sum_y, double sum_z, double count,
                           double& bias_x, double& bias_y, double& bias_z) {
  if (count == 0.0) {
    return false;
  }
  bias_x = sum_x / count;
  bias_y = sum_y / count;
  bias_z = sum_z / count;
  return true;
}

}  // namespace imu_filter

#endif  // IMU_FILTER_MATH_UTILS_H
