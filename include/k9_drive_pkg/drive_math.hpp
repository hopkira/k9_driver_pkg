#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace k9_drive_pkg::drive_math
{
constexpr double kTwoPi = 6.283185307179586476925286766559;

inline double counts_to_radians(int64_t counts, double counts_per_revolution)
{
  return static_cast<double>(counts) * kTwoPi / counts_per_revolution;
}

inline double qpps_to_radians_per_second(int32_t qpps, double counts_per_revolution)
{
  return static_cast<double>(qpps) * kTwoPi / counts_per_revolution;
}

inline int32_t radians_per_second_to_qpps(
  double radians_per_second, double counts_per_revolution)
{
  if (!std::isfinite(radians_per_second)) {
    return 0;
  }
  const double raw = radians_per_second * counts_per_revolution / kTwoPi;
  return static_cast<int32_t>(std::lround(raw));
}

inline int32_t radians_per_second_to_qpps_limited(
  double radians_per_second, double counts_per_revolution, int32_t max_abs_qpps)
{
  const int32_t raw = radians_per_second_to_qpps(radians_per_second, counts_per_revolution);
  return std::clamp(raw, -max_abs_qpps, max_abs_qpps);
}


inline double original_k9_turn_modifier(double centreline_radius_metres)
{
  const double radius = std::abs(centreline_radius_metres);
  return 1.0 - (0.9 / (radius + 1.0));
}

// uint32 subtraction is deliberately modulo 2^32. Interpreting the result as
// int32 yields the correct signed delta across encoder rollover, provided the
// wheel cannot move >= 2^31 counts between samples (physically impossible for K9).
inline int32_t rollover_safe_delta(uint32_t current, uint32_t previous)
{
  return static_cast<int32_t>(current - previous);
}

inline bool is_slowing(int32_t previous, int32_t requested)
{
  const bool magnitude_decreasing = std::abs(requested) < std::abs(previous);
  const bool direction_change =
    (previous > 0 && requested < 0) || (previous < 0 && requested > 0);
  return magnitude_decreasing || direction_change;
}

inline uint32_t select_acceleration(
  int32_t previous_left, int32_t previous_right,
  int32_t requested_left, int32_t requested_right,
  bool have_previous,
  uint32_t acceleration, uint32_t deceleration, uint32_t emergency_deceleration)
{
  if (requested_left == 0 && requested_right == 0) {
    return emergency_deceleration;
  }
  if (have_previous &&
    (is_slowing(previous_left, requested_left) || is_slowing(previous_right, requested_right)))
  {
    return deceleration;
  }
  return acceleration;
}

inline uint32_t bounded_travel_counts(int32_t qpps, double max_seconds)
{
  if (qpps == 0) {
    return 0;
  }
  const double raw = std::abs(static_cast<double>(qpps)) * max_seconds;
  if (raw < 1.0) {
    return 1;
  }
  if (raw > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
    return std::numeric_limits<uint32_t>::max();
  }
  return static_cast<uint32_t>(std::ceil(raw));
}
}  // namespace k9_drive_pkg::drive_math
