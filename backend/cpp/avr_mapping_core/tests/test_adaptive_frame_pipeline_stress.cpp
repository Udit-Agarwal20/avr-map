#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "avr_mapping/adaptive_frame_processor.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

// Section 59, 60, 61: 250,000-Point Stress Pipeline & Invariant Test
TEST(AdaptiveFramePipelineStressTest, Synthetic250kObservationsStressAndConsistency) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  constexpr size_t TOTAL_POINTS = 250000;
  constexpr size_t NON_FINITE_POINTS = 30;
  constexpr size_t OUT_OF_RANGE_POINTS = 20;
  constexpr size_t VALID_POINTS = TOTAL_POINTS - NON_FINITE_POINTS - OUT_OF_RANGE_POINTS;

  std::vector<PointXYZ> points;
  points.reserve(TOTAL_POINTS);

  std::mt19937_64 rng(1337);
  // Spread points across [-40, 40] meters in x and y
  std::uniform_real_distribution<double> dist_xy(-40.0, 40.0);
  std::uniform_real_distribution<double> dist_z(-1.0, 3.0);

  for (size_t i = 0; i < VALID_POINTS; ++i) {
    points.push_back(PointXYZ{
        .x = dist_xy(rng),
        .y = dist_xy(rng),
        .z = dist_z(rng),
    });
  }

  // Non-finite points
  for (size_t i = 0; i < NON_FINITE_POINTS; ++i) {
    if (i % 2 == 0) {
      points.push_back(PointXYZ{
          .x = std::numeric_limits<double>::quiet_NaN(),
          .y = dist_xy(rng),
          .z = 0.0,
      });
    } else {
      points.push_back(PointXYZ{
          .x = dist_xy(rng),
          .y = std::numeric_limits<double>::infinity(),
          .z = 0.0,
      });
    }
  }

  // Out of range points (beyond 100,000 km)
  for (size_t i = 0; i < OUT_OF_RANGE_POINTS; ++i) {
    points.push_back(PointXYZ{
        .x = 2e9,
        .y = dist_xy(rng),
        .z = 0.0,
    });
  }

  ASSERT_EQ(points.size(), TOTAL_POINTS);

  const avr::spatial::WorldPoint2D sensor_origin{0.0, 0.0};
  auto stats_opt = processor->processFrame(points, 1000, sensor_origin);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  // Section 59: Frame counter partition
  EXPECT_EQ(stats.input_points, TOTAL_POINTS);
  EXPECT_EQ(stats.rejected_non_finite, NON_FINITE_POINTS);
  EXPECT_EQ(stats.rejected_out_of_range, OUT_OF_RANGE_POINTS);
  EXPECT_TRUE(stats.isInputPartitionConsistent());

  // Section 61: Leaf and node count consistency
  EXPECT_TRUE(stats.isHierarchyConsistent());
  const auto h_stats = processor->hierarchy().statistics();
  EXPECT_EQ(stats.final_node_count, h_stats.node_count);
  EXPECT_EQ(stats.final_leaf_count, h_stats.leaf_count);
  EXPECT_EQ(stats.final_internal_count, h_stats.internal_count);
  EXPECT_EQ(stats.leaf_count_by_level, h_stats.leaf_count_by_level);

  // Topology validation
  const auto validation = processor->hierarchy().validateInvariants();
  EXPECT_TRUE(validation.valid);

  // Section 60: Stress Decision Consistency for every RegionDecisionRecord
  const auto records = processor->decisionSnapshot();
  EXPECT_GT(records.size(), 0u);
  EXPECT_EQ(records.size(), stats.regions_evaluated);

  for (const auto& rec : records) {
    // 1. Effective target never coarser than baseline
    EXPECT_LE(rec.effective_target, rec.local_decision.baseline_level);

    // 2. Monotonic target rule: effective = min(requested, inherited)
    const uint8_t expected_effective =
        std::min(rec.local_decision.requested_level, rec.inherited_target);
    EXPECT_EQ(rec.effective_target, expected_effective);

    // 3. Reason propagation: effective = local | inherited
    const auto expected_reasons = rec.local_reasons | rec.inherited_reasons;
    EXPECT_EQ(rec.effective_reasons, expected_reasons);

    // 4. Region has observations
    EXPECT_GT(rec.observation_count, 0u);
    EXPECT_GT(rec.occupied_base_cells, 0u);
    EXPECT_GT(rec.density_score, 0.0);
  }
}

}  // namespace
}  // namespace avr::mapping
