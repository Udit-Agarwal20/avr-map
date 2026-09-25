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

// Milestone 5: Parallel Welford tryMerge empty cases
TEST(ElevationStatsTest, TryMergeEmptyCases) {
  ElevationStats empty1;
  ElevationStats empty2;

  // empty + empty
  EXPECT_TRUE(empty1.tryMerge(empty2));
  EXPECT_EQ(empty1.count(), 0u);

  // non-empty + empty
  ElevationStats active(5, 10.0, 4.0, 8.0, 12.0);
  EXPECT_TRUE(active.tryMerge(empty1));
  EXPECT_EQ(active.count(), 5u);
  EXPECT_DOUBLE_EQ(active.mean(), 10.0);
  EXPECT_DOUBLE_EQ(active.m2(), 4.0);
  EXPECT_DOUBLE_EQ(active.min(), 8.0);
  EXPECT_DOUBLE_EQ(active.max(), 12.0);

  // empty + non-empty
  ElevationStats dest;
  EXPECT_TRUE(dest.tryMerge(active));
  EXPECT_EQ(dest.count(), 5u);
  EXPECT_DOUBLE_EQ(dest.mean(), 10.0);
  EXPECT_DOUBLE_EQ(dest.m2(), 4.0);
  EXPECT_DOUBLE_EQ(dest.min(), 8.0);
  EXPECT_DOUBLE_EQ(dest.max(), 12.0);
}

// Milestone 5: Parallel Welford merge of known sequence groups
TEST(ElevationStatsTest, TryMergeNormalSequence) {
  // Group 1: 2, 4, 4, 4
  ElevationStats g1;
  g1.update(2.0);
  g1.update(4.0);
  g1.update(4.0);
  g1.update(4.0);

  // Group 2: 5, 5, 7, 9
  ElevationStats g2;
  g2.update(5.0);
  g2.update(5.0);
  g2.update(7.0);
  g2.update(9.0);

  EXPECT_TRUE(g1.tryMerge(g2));
  EXPECT_EQ(g1.count(), 8u);
  EXPECT_DOUBLE_EQ(g1.mean(), 5.0);
  EXPECT_DOUBLE_EQ(g1.m2(), 32.0);
  EXPECT_DOUBLE_EQ(g1.populationVariance(), 4.0);
  EXPECT_NEAR(g1.sampleVariance(), 32.0 / 7.0, 1e-12);
  EXPECT_DOUBLE_EQ(g1.min(), 2.0);
  EXPECT_DOUBLE_EQ(g1.max(), 9.0);
}

// Milestone 5: tryMerge count overflow safety at uint64_t max
TEST(ElevationStatsTest, TryMergeCountOverflowSafety) {
  ElevationStats a(std::numeric_limits<uint64_t>::max() - 5, 10.0, 2.0, 5.0, 15.0);
  ElevationStats b(10, 12.0, 3.0, 6.0, 18.0);

  // Merging b into a would wrap uint64 -> must fail transactionally
  EXPECT_FALSE(a.tryMerge(b));

  // a must be completely unchanged
  EXPECT_EQ(a.count(), std::numeric_limits<uint64_t>::max() - 5);
  EXPECT_DOUBLE_EQ(a.mean(), 10.0);
  EXPECT_DOUBLE_EQ(a.m2(), 2.0);
  EXPECT_DOUBLE_EQ(a.min(), 5.0);
  EXPECT_DOUBLE_EQ(a.max(), 15.0);
}

// Milestone 5: tryMerge transaction rollback safety on extreme failure + subsequent valid merge
TEST(ElevationStatsTest, TryMergeTransactionSafety) {
  ElevationStats a;
  a.update(-1e300);

  ElevationStats b;
  b.update(1e300);

  // delta = 2e300 -> delta^2 overflows to +Inf -> tryMerge must fail
  EXPECT_FALSE(a.tryMerge(b));

  // Verify accumulator a remains bit-for-bit unmodified
  EXPECT_EQ(a.count(), 1u);
  EXPECT_DOUBLE_EQ(a.mean(), -1e300);
  EXPECT_DOUBLE_EQ(a.m2(), 0.0);
  EXPECT_DOUBLE_EQ(a.min(), -1e300);
  EXPECT_DOUBLE_EQ(a.max(), -1e300);

  // Verify a can still participate in a valid merge afterward
  ElevationStats c;
  c.update(-1e300);
  EXPECT_TRUE(a.tryMerge(c));
  EXPECT_EQ(a.count(), 2u);
  EXPECT_DOUBLE_EQ(a.mean(), -1e300);
  EXPECT_DOUBLE_EQ(a.m2(), 0.0);
}

}  // namespace
