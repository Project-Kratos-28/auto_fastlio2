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
  // NaN can appear in either coordinate independently (e.g. a sensor row
  // where only range/y is invalid) -- both must be left for downstream
  // (GLIM) to handle rather than silently dropped here.
  EXPECT_FALSE(point_is_masked(
    0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 30.0, 30.0));
  EXPECT_FALSE(point_is_masked(
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), 0.0, 30.0, 30.0));
  // Infinite returns are as invalid as NaN and must get the same treatment.
  EXPECT_FALSE(point_is_masked(
    std::numeric_limits<double>::infinity(), 1.0, 0.0, 30.0, 30.0));
  EXPECT_FALSE(point_is_masked(
    -std::numeric_limits<double>::infinity(), 1.0, 0.0, 30.0, 30.0));
}

TEST(AngleFilter, ZeroWidthDisablesASector)
{
  EXPECT_FALSE(point_is_masked(1.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(point_is_masked(-1.0, 0.0, 0.0, 0.0, 0.0));
}

// atan2 returns azimuths in (-pi, pi]; a point sitting exactly on the +/-180
// degree seam (e.g. straight behind the sensor with a negative-zero y from
// floating point rounding) must still resolve to the same back-sector
// distance as a point on the other side of the seam. This pins down that
// std::remainder-based angular_distance() does not treat +180 and -180 as
// far apart.
TEST(AngleFilter, HandlesAzimuthWrapAtPlusMinusOneEightyDegrees)
{
  // atan2(+0.0, -1.0) == +pi
  EXPECT_TRUE(point_is_masked(-1.0, 0.0, 0.0, 30.0, 30.0));
  // atan2(-0.0, -1.0) == -pi -- the discontinuity itself.
  EXPECT_TRUE(point_is_masked(-1.0, -0.0, 0.0, 30.0, 30.0));

  double x;
  double y;
  point_at(180.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));
  point_at(-180.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));
}

// With a rotated mount the sector itself can straddle the +/-180 seam (e.g.
// front_center_deg=170 puts the front sector across [155,185], and 185 deg
// is the same direction as -175 deg). Points just inside and just outside
// that wrapped edge must be classified correctly on both sides of the seam.
TEST(AngleFilter, MasksAcrossWrapWhenSectorStraddlesSeam)
{
  double x;
  double y;
  constexpr double front_center = 170.0;

  // 179 deg: 9 deg from center, still on the "normal" side of the seam.
  point_at(179.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, front_center, 30.0, 0.0));

  // -178 deg (== 182 deg): 12 deg from center, but only reachable by
  // crossing the +/-180 seam.
  point_at(-178.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, front_center, 30.0, 0.0));

  // -175 deg (== 185 deg): exactly 15 deg from center -- the wrapped edge,
  // inclusive per the boundary tolerance.
  point_at(-175.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, front_center, 30.0, 0.0));

  // -160 deg: 30 deg from center, clearly outside the sector even after
  // wrapping.
  point_at(-160.0, x, y);
  EXPECT_FALSE(point_is_masked(x, y, front_center, 30.0, 0.0));
}

// The boundary_tolerance in point_is_masked makes the sector edge inclusive.
// Exercise the exact edge (not just epsilon-inside/outside neighbors) for
// both the front and back sectors at the default (unrotated) mounting.
TEST(AngleFilter, MasksExactSectorBoundaryInclusive)
{
  double x;
  double y;

  // Front sector is [-15, 15] deg wide; +/-15.0 exactly should be masked.
  point_at(15.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));
  point_at(-15.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));

  // Back sector is centered at 180 deg, so [165, 195] == [165, -165].
  point_at(165.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));
  point_at(-165.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 30.0, 30.0));
}

// A non-default, asymmetric front/back width is representative of what an
// operator might dial in for a rover with more body occlusion behind than
// in front. Confirms the two sector widths are applied independently.
TEST(AngleFilter, AppliesIndependentFrontAndBackWidths)
{
  double x;
  double y;

  // Front sector narrowed to 10 deg (+/-5), back widened to 60 deg (+/-30).
  point_at(6.0, x, y);
  EXPECT_FALSE(point_is_masked(x, y, 0.0, 10.0, 60.0)) << "just outside narrow front";
  point_at(4.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 10.0, 60.0)) << "inside narrow front";

  point_at(151.0, x, y);
  EXPECT_TRUE(point_is_masked(x, y, 0.0, 10.0, 60.0)) << "inside wide back";
  point_at(149.0, x, y);
  EXPECT_FALSE(point_is_masked(x, y, 0.0, 10.0, 60.0)) << "just outside wide back";
}

}  // namespace
