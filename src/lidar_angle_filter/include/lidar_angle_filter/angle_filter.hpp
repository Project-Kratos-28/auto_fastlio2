#pragma once

#include <algorithm>
#include <cmath>

namespace lidar_angle_filter
{

constexpr double kPi = 3.14159265358979323846;

inline double degrees_to_radians(const double degrees)
{
  return degrees * kPi / 180.0;
}

inline double angular_distance(const double first, const double second)
{
  return std::abs(std::remainder(first - second, 2.0 * kPi));
}

inline bool point_is_masked(
  const double x, const double y, const double front_center_deg,
  const double front_sector_deg, const double back_sector_deg)
{
  if (!std::isfinite(x) || !std::isfinite(y) || (x == 0.0 && y == 0.0)) {
    return false;
  }

  const double azimuth = std::atan2(y, x);
  const double front_center = degrees_to_radians(front_center_deg);
  const double back_center = front_center + kPi;
  constexpr double boundary_tolerance = 1e-12;

  const bool in_front_sector =
    front_sector_deg > 0.0 &&
    angular_distance(azimuth, front_center) <=
    degrees_to_radians(front_sector_deg) * 0.5 + boundary_tolerance;
  const bool in_back_sector =
    back_sector_deg > 0.0 &&
    angular_distance(azimuth, back_center) <=
    degrees_to_radians(back_sector_deg) * 0.5 + boundary_tolerance;
  return in_front_sector || in_back_sector;
}

}  // namespace lidar_angle_filter
