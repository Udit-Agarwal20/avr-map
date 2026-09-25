#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/elevation_stats.hpp"

namespace {

using avr::mapping::ElevationStats;

TEST(ElevationStatsTest, InitialZeroState) {
  ElevationStats stats;
  EXPECT_EQ(stats.count(), 0u);
  EXPECT_DOUBLE_EQ(stats.mean(), 0.0);
  EXPECT_DOUBLE_EQ(stats.m2(), 0.0);
  EXPECT_DOUBLE_EQ(stats.min(), 0.0);
  EXPECT_DOUBLE_EQ(stats.max(), 0.0);
  EXPECT_DOUBLE_EQ(stats.populationVariance(), 0.0);
  EXPECT_DOUBLE_EQ(stats.sampleVariance(), 0.0);
}

TEST(ElevationStatsTest, SingleObservationExplicitHandling) {
  ElevationStats stats;
  EXPECT_TRUE(stats.tryUpdate(42.5));

  EXPECT_EQ(stats.count(), 1u);
  EXPECT_DOUBLE_EQ(stats.mean(), 42.5);
  EXPECT_DOUBLE_EQ(stats.m2(), 0.0);
  EXPECT_DOUBLE_EQ(stats.min(), 42.5);
  EXPECT_DOUBLE_EQ(stats.max(), 42.5);
  EXPECT_DOUBLE_EQ(stats.populationVariance(), 0.0);
  EXPECT_DOUBLE_EQ(stats.sampleVariance(), 0.0);
}

// Section 31: Known sequence test
TEST(ElevationStatsTest, WelfordKnownSequence) {
  ElevationStats stats;
  const std::vector<double> sequence = {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};

  for (double val : sequence) {
    EXPECT_TRUE(stats.tryUpdate(val));
  }

  EXPECT_EQ(stats.count(), 8u);
  EXPECT_DOUBLE_EQ(stats.mean(), 5.0);
  EXPECT_DOUBLE_EQ(stats.m2(), 32.0);
  EXPECT_DOUBLE_EQ(stats.min(), 2.0);
  EXPECT_DOUBLE_EQ(stats.max(), 9.0);
  EXPECT_DOUBLE_EQ(stats.populationVariance(), 4.0);
  EXPECT_NEAR(stats.sampleVariance(), 32.0 / 7.0, 1e-12);
}

// Section 32: Numerical stability with large baseline and small deviations
TEST(ElevationStatsTest, NumericalStabilityLargeBaseline) {
  ElevationStats stats;
  constexpr double baseline = 1'000'000.0;
  const std::vector<double> deltas = {0.001, 0.002, 0.003, 0.004, 0.005,
                                      0.006, 0.007, 0.008, 0.009, 0.010};

  for (double d : deltas) {
    EXPECT_TRUE(stats.tryUpdate(baseline + d));
  }

  EXPECT_EQ(stats.count(), 10u);
  EXPECT_TRUE(std::isfinite(stats.mean()));
  EXPECT_TRUE(std::isfinite(stats.populationVariance()));
  EXPECT_TRUE(std::isfinite(stats.sampleVariance()));

  // Mean should be baseline + 0.0055
  EXPECT_NEAR(stats.mean(), baseline + 0.0055, 1e-9);
  EXPECT_NEAR(stats.populationVariance(), 8.25e-6, 1e-12);
  EXPECT_NEAR(stats.sampleVariance(), 82.5e-6 / 9.0, 1e-12);
}

// Milestone 2.1: uint64 observation counter and layout
TEST(ElevationStatsTest, Uint64CounterAndMemoryLayout) {
  ElevationStats stats;
  static_assert(std::is_same_v<decltype(stats.count()), uint64_t>);
  EXPECT_EQ(sizeof(ElevationStats), 40u);
  EXPECT_EQ(alignof(ElevationStats), 8u);
}

// Milestone 2.1: Observation count overflow safety at UINT64_MAX
TEST(ElevationStatsTest, CountOverflowSafetyAtMaxUint64) {
  // Construct a state already at uint64_t max
  ElevationStats max_stats(std::numeric_limits<uint64_t>::max(), 10.0, 5.0, 2.0, 20.0);
  EXPECT_EQ(max_stats.count(), std::numeric_limits<uint64_t>::max());

  // Attempting to update must fail safely and preserve state
  EXPECT_FALSE(max_stats.tryUpdate(15.0));
  EXPECT_EQ(max_stats.count(), std::numeric_limits<uint64_t>::max());
  EXPECT_DOUBLE_EQ(max_stats.mean(), 10.0);
  EXPECT_DOUBLE_EQ(max_stats.m2(), 5.0);
  EXPECT_DOUBLE_EQ(max_stats.min(), 2.0);
  EXPECT_DOUBLE_EQ(max_stats.max(), 20.0);
}

// Milestone 2.1: Welford numeric transaction safety on extreme values
TEST(ElevationStatsTest, NumericOverflowTransactionSafety) {
  ElevationStats stats;
  // Initialize with an extreme negative mean
  EXPECT_TRUE(stats.tryUpdate(-1e308));
  EXPECT_DOUBLE_EQ(stats.mean(), -1e308);

  // Attempting to update with +1e308 causes delta = z - mean to overflow to +Inf
  EXPECT_FALSE(stats.tryUpdate(1e308));

  // Verify state is completely preserved
  EXPECT_EQ(stats.count(), 1u);
  EXPECT_DOUBLE_EQ(stats.mean(), -1e308);
  EXPECT_DOUBLE_EQ(stats.m2(), 0.0);
  EXPECT_DOUBLE_EQ(stats.min(), -1e308);
  EXPECT_DOUBLE_EQ(stats.max(), -1e308);
  EXPECT_DOUBLE_EQ(stats.populationVariance(), 0.0);

  // Non-finite updates are rejected and leave state untouched
  EXPECT_FALSE(stats.tryUpdate(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(stats.tryUpdate(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(stats.tryUpdate(-std::numeric_limits<double>::infinity()));
  EXPECT_EQ(stats.count(), 1u);
  EXPECT_DOUBLE_EQ(stats.mean(), -1e308);
}

TEST(ElevationStatsTest, ResetClearsState) {
  ElevationStats stats;
  stats.update(10.0);
  stats.update(20.0);
  EXPECT_EQ(stats.count(), 2u);

  stats.reset();
  EXPECT_EQ(stats.count(), 0u);
  EXPECT_DOUBLE_EQ(stats.mean(), 0.0);
  EXPECT_DOUBLE_EQ(stats.m2(), 0.0);
  EXPECT_DOUBLE_EQ(stats.min(), 0.0);
  EXPECT_DOUBLE_EQ(stats.max(), 0.0);
  EXPECT_DOUBLE_EQ(stats.populationVariance(), 0.0);
  EXPECT_DOUBLE_EQ(stats.sampleVariance(), 0.0);
}

}  // namespace
