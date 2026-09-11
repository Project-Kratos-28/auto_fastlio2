#include "lidar_angle_filter/angle_filter.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using lidar_angle_filter::degrees_to_radians;
using lidar_angle_filter::point_is_masked;

namespace
{

void point_at(const double degrees, double & x, double & y)
{
  const double radians = degrees_to_radians(degrees);
  x = std::cos(radians);
  y = std::sin(radians);
}

TEST(AngleFilter, MasksThirtyDegreeFrontAndBackSectors)
{
  double x;
  double y;

  for (const double angle : {0.0, 14.9, -14.9, 165.0, -165.0, 179.9, -179.9}) {
    point_at(angle, x, y);
    EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0)) << angle;
  }

  for (const double angle : {15.1, -15.1, 90.0, -90.0, 164.9, -164.9}) {
    point_at(angle, x, y);
    EXPECT_FALSE(point_is_masked(x, y, 0.0, 30.0, 30.0)) << angle;
  }
}

TEST(AngleFilter, SupportsRotatedLidarMounting)
{
  double x;
  double y;
  point_at(90.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 90.0, 30.0, 30.0));

  point_at(-90.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 90.0, 30.0, 30.0));

  point_at(0.0, x, y);
  EXPECT_FALSE(point_is_masked(x, y, 90.0, 30.0, 30.0));
}

TEST(AngleFilter, KeepsInvalidAndOriginPointsForDownstreamHandling)
{
  EXPECT_FALSE(point_is_masked(0.0, 0.0, 0.0, 30.0, 30.0));
  EXPECT_FALSE(point_is_masked(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 30.0, 30.0));
}

TEST(AngleFilter, ZeroWidthDisablesASector)
{
  EXPECT_FALSE(point_is_masked(1.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(point_is_masked(-1.0, 0.0, 0.0, 0.0, 0.0));
}

}  // namespace
