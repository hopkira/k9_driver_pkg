#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "k9_drive_pkg/drive_math.hpp"

using namespace k9_drive_pkg::drive_math;

TEST(K9DriveMath, ProvenDistanceCalibration)
{
  constexpr double metres_per_count = 0.002179;
  constexpr double counts_per_revolution = 200.0;
  EXPECT_NEAR(metres_per_count * counts_per_revolution, 0.4358, 1e-12);
  EXPECT_NEAR(0.4358 / kTwoPi, 0.069359724199448, 1e-12);
}

TEST(K9DriveMath, TwoHundredCountsIsOneWheelRevolution)
{
  EXPECT_NEAR(counts_to_radians(200, 200.0), kTwoPi, 1e-12);
}

TEST(K9DriveMath, OperationalSpeedMatchesOriginalK9Limit)
{
  constexpr int32_t top_qpps = 642;
  constexpr double metres_per_count = 0.002179;
  EXPECT_NEAR(top_qpps * metres_per_count, 1.398918, 1e-12);
}


TEST(K9DriveMath, OriginalTurnModifierIsPreserved)
{
  EXPECT_NEAR(original_k9_turn_modifier(0.0), 0.1, 1e-12);
  EXPECT_NEAR(original_k9_turn_modifier(1.0), 0.55, 1e-12);
  EXPECT_GT(original_k9_turn_modifier(1000.0), 0.999);
}

TEST(K9DriveMath, ForwardEncoderRolloverIsContinuous)
{
  const uint32_t previous = 0xfffffff0U;
  const uint32_t current = 0x00000010U;
  EXPECT_EQ(rollover_safe_delta(current, previous), 32);
}

TEST(K9DriveMath, ReverseEncoderRolloverIsContinuous)
{
  const uint32_t previous = 0x00000010U;
  const uint32_t current = 0xfffffff0U;
  EXPECT_EQ(rollover_safe_delta(current, previous), -32);
}

TEST(K9DriveMath, BrakingUsesLaterSafetyRatios)
{
  EXPECT_EQ(select_acceleration(100, 100, 120, 120, true, 128, 256, 512), 128U);
  EXPECT_EQ(select_acceleration(100, 100, 50, 50, true, 128, 256, 512), 256U);
  EXPECT_EQ(select_acceleration(100, 100, -50, -50, true, 128, 256, 512), 256U);
  EXPECT_EQ(select_acceleration(100, 100, 0, 0, true, 128, 256, 512), 512U);
}

TEST(K9DriveMath, BoundedCommandLimitsUncommandedTravel)
{
  EXPECT_EQ(bounded_travel_counts(642, 0.1), 65U);
  EXPECT_EQ(bounded_travel_counts(-642, 0.1), 65U);
  EXPECT_EQ(bounded_travel_counts(1, 0.1), 1U);
  EXPECT_EQ(bounded_travel_counts(0, 0.1), 0U);
}
