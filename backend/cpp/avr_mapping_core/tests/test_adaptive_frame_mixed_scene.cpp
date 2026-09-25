#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_frame_processor.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

// Section 44: Mixed-Scene Test with simultaneous L0, L1, L2, L3 active leaves
TEST(AdaptiveFrameMixedSceneTest, MixedL0L1L2L3SimultaneousLeaves) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  std::vector<PointXYZ> points;
  const avr::spatial::WorldPoint2D sensor_origin{0.0, 0.0};

  // Region A: Near (< 10m), single cell -> refines to L0
  points.push_back(PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0});

  // Region B: Medium (10 <= d < 30m), flat -> refines to L1
  points.push_back(PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0});

  // Region C: Far (>= 60m), flat -> stays at L3
  points.push_back(PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0});

  // Region D: Far (>= 60m), but distributed vertical complexity -> refines to at least L2
  // We place 16 distinct base cells in root at cx = 250, cy = 0: [100.0, 100.40) x [0.0, 0.40).
  // 8 base cells at z = 0.0, 8 base cells at z = 0.25 (span = 0.25m, density = 16/64 = 0.25).
  // Geometry complexity: 0.25 / 0.40 = 0.625 >= 0.50 -> refines to L2.
  for (int cy = 0; cy < 4; ++cy) {
    for (int cx = 0; cx < 4; ++cx) {
      double px = 100.0 + cx * 0.05 + 0.025;
      double py = 0.0 + cy * 0.05 + 0.025;
      double pz = (cx < 2) ? 0.0 : 0.25;
      points.push_back(PointXYZ{.x = px, .y = py, .z = pz});
    }
  }

  auto stats_opt = processor->processFrame(points, 500, sensor_origin);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  // Verify all four levels have active leaves simultaneously!
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L0], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L1], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L2], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L3], 0u);

  // Invariants pass
  const auto validation = processor->hierarchy().validateInvariants();
  EXPECT_TRUE(validation.valid);
}

// Section 45: Multi-Root Independence across quadrants
TEST(AdaptiveFrameMixedSceneTest, MultiRootIndependenceAcrossQuadrants) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  const avr::spatial::WorldPoint2D sensor_origin{0.0, 0.0};
  std::vector<PointXYZ> points = {
      // Quadrant 1 (+x, +y): near -> L0
      PointXYZ{.x = 2.025, .y = 2.025, .z = 0.0},
      // Quadrant 2 (-x, +y): medium -> L1
      PointXYZ{.x = -20.025, .y = 2.025, .z = 0.0},
      // Quadrant 3 (-x, -y): far flat -> L3
      PointXYZ{.x = -80.025, .y = -2.025, .z = 0.0},
      // Quadrant 4 (+x, -y): L2 distance band (30 <= d < 60m) -> L2
      PointXYZ{.x = 40.025, .y = -2.025, .z = 0.0},
  };

  auto stats_opt = processor->processFrame(points, 600, sensor_origin);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 4u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L0], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L1], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L2], 0u);
  EXPECT_GT(stats.leaf_count_by_level[avr::spatial::L3], 0u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 46: Negative Coordinate End-to-End Test
TEST(AdaptiveFrameMixedSceneTest, NegativeCoordinateEndToEnd) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Negative coordinates in Quadrant 3
  const avr::spatial::WorldPoint2D sensor_origin{-50.0, -50.0};
  // Point near sensor: (-48.0, -50.0) -> distance 2.0m < 10m -> baseline L0
  std::vector<PointXYZ> points = {
      PointXYZ{.x = -48.025, .y = -49.975, .z = 1.5},
  };

  auto stats_opt = processor->processFrame(points, 700, sensor_origin);
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.final_leaf_count, 10u);  // 3 L2 + 3 L1 + 4 L0
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 4u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);

  // Verify findLeafContainingBase at negative grid coordinate
  const auto cell_opt = avr::spatial::worldToGridCoord(
      avr::spatial::WorldPoint2D{-48.025, -49.975});
  ASSERT_TRUE(cell_opt.has_value());

  const auto leaf_opt = processor->hierarchy().findLeafContainingBase(*cell_opt);
  ASSERT_TRUE(leaf_opt.has_value());
  EXPECT_EQ(leaf_opt->level, avr::spatial::L0);
}

}  // namespace
}  // namespace avr::mapping
