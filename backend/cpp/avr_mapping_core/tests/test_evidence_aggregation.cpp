#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/elevation_stats.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

using namespace avr::mapping;
using namespace avr::spatial;

TEST(EvidenceAggregationTest, L0ThroughL3SummaryFootprints) {
  FrameEvidence fe;

  // Populate an entire L3 region at (0, 0).
  // L3 region spans base cells gx in [0, 7], gy in [0, 7] (64 base cells total).
  // Metric coordinates: [0.0, 0.40) in both X and Y.
  std::vector<PointXYZ> points;
  for (int gy = 0; gy < 8; ++gy) {
    for (int gx = 0; gx < 8; ++gx) {
      // Cell center in meters: gx * 0.05 + 0.025
      const double x = gx * 0.05 + 0.025;
      const double y = gy * 0.05 + 0.025;
      points.push_back(PointXYZ{x, y, 1.0});
    }
  }

  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  // Query L0 at (0,0)
  const auto l0_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(l0_key.has_value());
  const auto l0_sum = fe.summarize(*l0_key);
  EXPECT_EQ(l0_sum.base_cell_capacity, 1);
  EXPECT_EQ(l0_sum.occupied_base_cells, 1);
  EXPECT_EQ(l0_sum.observation_count, 1);
  EXPECT_DOUBLE_EQ(l0_sum.density_score, 1.0);

  // Query L1 at (0,0): covers base gx in [0, 1], gy in [0, 1] (4 cells)
  const auto l1_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L1});
  ASSERT_TRUE(l1_key.has_value());
  const auto l1_sum = fe.summarize(*l1_key);
  EXPECT_EQ(l1_sum.base_cell_capacity, 4);
  EXPECT_EQ(l1_sum.occupied_base_cells, 4);
  EXPECT_EQ(l1_sum.observation_count, 4);
  EXPECT_DOUBLE_EQ(l1_sum.density_score, 1.0);

  // Query L2 at (0,0): covers base gx in [0, 3], gy in [0, 3] (16 cells)
  const auto l2_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L2});
  ASSERT_TRUE(l2_key.has_value());
  const auto l2_sum = fe.summarize(*l2_key);
  EXPECT_EQ(l2_sum.base_cell_capacity, 16);
  EXPECT_EQ(l2_sum.occupied_base_cells, 16);
  EXPECT_EQ(l2_sum.observation_count, 16);
  EXPECT_DOUBLE_EQ(l2_sum.density_score, 1.0);

  // Query L3 at (0,0): covers base gx in [0, 7], gy in [0, 7] (64 cells)
  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto l3_sum = fe.summarize(*l3_key);
  EXPECT_EQ(l3_sum.base_cell_capacity, 64);
  EXPECT_EQ(l3_sum.occupied_base_cells, 64);
  EXPECT_EQ(l3_sum.observation_count, 64);
  EXPECT_DOUBLE_EQ(l3_sum.density_score, 1.0);
}

TEST(EvidenceAggregationTest, EmptyRegionSummary) {
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{100.0, 100.0, 1.0}};
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  // Query L3 region at (0, 0) which is empty
  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto summary = fe.summarize(*l3_key);

  EXPECT_TRUE(summary.isEmpty());
  EXPECT_EQ(summary.observation_count, 0);
  EXPECT_EQ(summary.occupied_base_cells, 0);
  EXPECT_EQ(summary.base_cell_capacity, 64);
  EXPECT_DOUBLE_EQ(summary.density_score, 0.0);
  EXPECT_FALSE(summary.elevation_mean.has_value());
  EXPECT_FALSE(summary.elevation_variance.has_value());
  EXPECT_FALSE(summary.z_min.has_value());
  EXPECT_FALSE(summary.z_max.has_value());
  EXPECT_FALSE(summary.geometry_complexity.has_value());
}

TEST(EvidenceAggregationTest, RepeatedPointsAndDensityExactness) {
  FrameEvidence fe;

  // Insert 100 observations into the exact same L0 base cell
  std::vector<PointXYZ> points;
  for (int i = 0; i < 100; ++i) {
    points.push_back(PointXYZ{0.02, 0.02, 0.5 + 0.01 * (i % 5)});
  }
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  // L0 summary
  const auto l0_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(l0_key.has_value());
  const auto l0_sum = fe.summarize(*l0_key);
  EXPECT_EQ(l0_sum.observation_count, 100);
  EXPECT_EQ(l0_sum.occupied_base_cells, 1);
  EXPECT_DOUBLE_EQ(l0_sum.density_score, 1.0);

  // Containing L3 summary
  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto l3_sum = fe.summarize(*l3_key);
  EXPECT_EQ(l3_sum.observation_count, 100);
  EXPECT_EQ(l3_sum.occupied_base_cells, 1);
  EXPECT_DOUBLE_EQ(l3_sum.density_score, 1.0 / 64.0);
}

TEST(EvidenceAggregationTest, ParentAggregationConsistency) {
  FrameEvidence fe;

  // Populate arbitrary points across an L3 region
  std::vector<PointXYZ> points = {
      // Child L2 (0, 0): base [0, 3] x [0, 3]
      PointXYZ{0.02, 0.02, 1.0},
      PointXYZ{0.07, 0.07, 1.1},
      PointXYZ{0.07, 0.07, 1.2},  // duplicate in same base cell
      // Child L2 (1, 0): base [4, 7] x [0, 3]
      PointXYZ{0.22, 0.02, 1.3},
      PointXYZ{0.27, 0.12, 1.4},
      // Child L2 (0, 1): base [0, 3] x [4, 7]
      PointXYZ{0.02, 0.22, 1.5},
      // Child L2 (1, 1): base [4, 7] x [4, 7]
      PointXYZ{0.32, 0.32, 1.6},
      PointXYZ{0.37, 0.37, 1.7},
  };

  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto l3_sum = fe.summarize(*l3_key);

  const auto children = childrenOf(*l3_key);
  ASSERT_TRUE(children.has_value());

  uint64_t child_obs_sum = 0;
  uint32_t child_occupied_sum = 0;

  for (const auto& child_key : *children) {
    const auto child_sum = fe.summarize(child_key);
    child_obs_sum += child_sum.observation_count;
    child_occupied_sum += child_sum.occupied_base_cells;
  }

  EXPECT_EQ(child_obs_sum, l3_sum.observation_count);
  EXPECT_EQ(child_occupied_sum, l3_sum.occupied_base_cells);
  EXPECT_EQ(l3_sum.observation_count, 8);
  EXPECT_EQ(l3_sum.occupied_base_cells, 7);
}

TEST(EvidenceAggregationTest, NegativeCoordinatesAcrossTiles) {
  FrameEvidence fe;

  // Base coordinates around -1, -8, -9, -16, -17
  // -1 base is in tile -1 (lx = 15)
  // -16 base is in tile -1 (lx = 0)
  // -17 base is in tile -2 (lx = 15)
  std::vector<PointXYZ> points = {
      PointXYZ{-1 * 0.05 + 0.025, -1 * 0.05 + 0.025, 2.0},    // base (-1, -1)
      PointXYZ{-8 * 0.05 + 0.025, -8 * 0.05 + 0.025, 2.1},    // base (-8, -8)
      PointXYZ{-9 * 0.05 + 0.025, -9 * 0.05 + 0.025, 2.2},    // base (-9, -9)
      PointXYZ{-16 * 0.05 + 0.025, -16 * 0.05 + 0.025, 2.3},  // base (-16, -16)
      PointXYZ{-17 * 0.05 + 0.025, -17 * 0.05 + 0.025, 2.4},  // base (-17, -17)
  };

  const auto stats = fe.buildFrame(points, 5, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->accepted_points, 5);
  EXPECT_EQ(stats->active_base_cells, 5);

  // Verify L0 summary for base (-1, -1)
  const auto k_neg1 = CellKey::fromCellCoord(CellCoord{.x = -1, .y = -1, .level = L0});
  ASSERT_TRUE(k_neg1.has_value());
  const auto sum_neg1 = fe.summarize(*k_neg1);
  EXPECT_EQ(sum_neg1.occupied_base_cells, 1);
  EXPECT_DOUBLE_EQ(*sum_neg1.elevation_mean, 2.0);

  // Verify L0 summary for base (-17, -17)
  const auto k_neg17 = CellKey::fromCellCoord(CellCoord{.x = -17, .y = -17, .level = L0});
  ASSERT_TRUE(k_neg17.has_value());
  const auto sum_neg17 = fe.summarize(*k_neg17);
  EXPECT_EQ(sum_neg17.occupied_base_cells, 1);
  EXPECT_DOUBLE_EQ(*sum_neg17.elevation_mean, 2.4);
}

TEST(EvidenceAggregationTest, FlatSurfaceGeometry) {
  FrameEvidence fe;

  // Multiple distinct L0 cells in L3 region (0, 0) with identical z = 3.50
  std::vector<PointXYZ> points = {
      PointXYZ{0.02, 0.02, 3.50},
      PointXYZ{0.07, 0.02, 3.50},
      PointXYZ{0.12, 0.02, 3.50},
      PointXYZ{0.17, 0.02, 3.50},
  };
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto sum = fe.summarize(*l3_key);

  EXPECT_EQ(sum.occupied_base_cells, 4);
  EXPECT_EQ(sum.observation_count, 4);
  ASSERT_TRUE(sum.z_min.has_value());
  ASSERT_TRUE(sum.z_max.has_value());
  EXPECT_DOUBLE_EQ(*sum.z_min, 3.50);
  EXPECT_DOUBLE_EQ(*sum.z_max, 3.50);
  ASSERT_TRUE(sum.geometry_complexity.has_value());
  EXPECT_DOUBLE_EQ(*sum.geometry_complexity, 0.0);
}

TEST(EvidenceAggregationTest, VerticalComplexityRelief) {
  FrameEvidence fe;

  // For L3: region width = 0.40 m.
  // 1. z span = 0.20 m -> geometry = 0.20 / 0.40 = 0.50
  std::vector<PointXYZ> points_05 = {
      PointXYZ{0.02, 0.02, 0.00},
      PointXYZ{0.07, 0.02, 0.20},
  };
  fe.buildFrame(points_05, 1, WorldPoint2D{0.0, 0.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto sum_05 = fe.summarize(*l3_key);
  ASSERT_TRUE(sum_05.geometry_complexity.has_value());
  EXPECT_NEAR(*sum_05.geometry_complexity, 0.50, 1e-9);

  // 2. z span = 0.40 m or greater -> clamped to 1.0
  std::vector<PointXYZ> points_10 = {
      PointXYZ{0.02, 0.02, 0.00},
      PointXYZ{0.07, 0.02, 0.45},
  };
  fe.buildFrame(points_10, 2, WorldPoint2D{0.0, 0.0});
  const auto sum_10 = fe.summarize(*l3_key);
  ASSERT_TRUE(sum_10.geometry_complexity.has_value());
  EXPECT_DOUBLE_EQ(*sum_10.geometry_complexity, 1.0);

  // 3. Level 2 test: region width = 0.20 m.
  // z span = 0.10 m -> geometry = 0.10 / 0.20 = 0.50
  std::vector<PointXYZ> points_l2 = {
      PointXYZ{0.02, 0.02, 1.00},
      PointXYZ{0.07, 0.02, 1.10},
  };
  fe.buildFrame(points_l2, 3, WorldPoint2D{0.0, 0.0});
  const auto l2_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L2});
  ASSERT_TRUE(l2_key.has_value());
  const auto sum_l2 = fe.summarize(*l2_key);
  ASSERT_TRUE(sum_l2.geometry_complexity.has_value());
  EXPECT_NEAR(*sum_l2.geometry_complexity, 0.50, 1e-9);
}

TEST(EvidenceAggregationTest, InsufficientGeometrySupport) {
  FrameEvidence fe;

  // Case A: 1 point -> geometry nullopt
  std::vector<PointXYZ> p1 = {PointXYZ{0.02, 0.02, 1.0}};
  fe.buildFrame(p1, 1, WorldPoint2D{0.0, 0.0});
  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto sum_a = fe.summarize(*l3_key);
  EXPECT_EQ(sum_a.observation_count, 1);
  EXPECT_EQ(sum_a.occupied_base_cells, 1);
  EXPECT_FALSE(sum_a.geometry_complexity.has_value());

  // Case B: multiple repeated points in ONE L0 bin -> geometry nullopt
  std::vector<PointXYZ> p_repeat = {
      PointXYZ{0.02, 0.02, 1.0},
      PointXYZ{0.02, 0.02, 1.5},
      PointXYZ{0.02, 0.02, 2.0},
  };
  fe.buildFrame(p_repeat, 2, WorldPoint2D{0.0, 0.0});
  const auto sum_b = fe.summarize(*l3_key);
  EXPECT_EQ(sum_b.observation_count, 3);
  EXPECT_EQ(sum_b.occupied_base_cells, 1);
  EXPECT_FALSE(sum_b.geometry_complexity.has_value());

  // Case C: >= 2 points in >= 2 different L0 bins -> geometry populated
  std::vector<PointXYZ> p_multi = {
      PointXYZ{0.02, 0.02, 1.0},
      PointXYZ{0.07, 0.02, 1.2},
  };
  fe.buildFrame(p_multi, 3, WorldPoint2D{0.0, 0.0});
  const auto sum_c = fe.summarize(*l3_key);
  EXPECT_EQ(sum_c.observation_count, 2);
  EXPECT_EQ(sum_c.occupied_base_cells, 2);
  EXPECT_TRUE(sum_c.geometry_complexity.has_value());
}

TEST(EvidenceAggregationTest, ElevationMergeKnownSequence) {
  // Known sequence: 2, 4, 4, 4, 5, 5, 7, 9 across several evidence cells in one region
  // Expected:
  // count = 8
  // mean = 5.0
  // population variance = 4.0
  // sample variance = 32 / 7
  // min = 2.0
  // max = 9.0
  FrameEvidence fe;
  std::vector<PointXYZ> points = {
      PointXYZ{0.02, 0.02, 2.0},  // cell 0
      PointXYZ{0.07, 0.02, 4.0},  // cell 1
      PointXYZ{0.07, 0.02, 4.0},  // cell 1
      PointXYZ{0.07, 0.02, 4.0},  // cell 1
      PointXYZ{0.12, 0.02, 5.0},  // cell 2
      PointXYZ{0.12, 0.02, 5.0},  // cell 2
      PointXYZ{0.17, 0.02, 7.0},  // cell 3
      PointXYZ{0.22, 0.02, 9.0},  // cell 4
  };

  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());
  const auto sum = fe.summarize(*l3_key);

  EXPECT_EQ(sum.observation_count, 8);
  EXPECT_EQ(sum.occupied_base_cells, 5);
  ASSERT_TRUE(sum.elevation_mean.has_value());
  ASSERT_TRUE(sum.elevation_variance.has_value());
  ASSERT_TRUE(sum.z_min.has_value());
  ASSERT_TRUE(sum.z_max.has_value());

  EXPECT_NEAR(*sum.elevation_mean, 5.0, 1e-12);
  EXPECT_NEAR(*sum.elevation_variance, 4.0, 1e-12);  // Population variance M2 / n = 32 / 8 = 4.0
  EXPECT_DOUBLE_EQ(*sum.z_min, 2.0);
  EXPECT_DOUBLE_EQ(*sum.z_max, 9.0);
}

// Milestone 5: At L0, occupied base cells is at most 1, so geometry_complexity is strictly nullopt
TEST(EvidenceAggregationTest, Level0GeometryComplexityIsAlwaysNullopt) {
  FrameEvidence fe;
  // Multiple observations with vertical relief in one L0 cell
  std::vector<PointXYZ> points = {
      PointXYZ{0.02, 0.02, 1.0},
      PointXYZ{0.02, 0.02, 2.0},
      PointXYZ{0.02, 0.02, 3.0},
  };
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto l0_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(l0_key.has_value());
  const auto sum = fe.summarize(*l0_key);

  EXPECT_EQ(sum.observation_count, 3);
  EXPECT_EQ(sum.occupied_base_cells, 1);
  EXPECT_DOUBLE_EQ(sum.density_score, 1.0);
  ASSERT_TRUE(sum.elevation_mean.has_value());
  EXPECT_DOUBLE_EQ(*sum.elevation_mean, 2.0);

  // Even with vertical relief, geometry_complexity requires >= 2 distinct base cells.
  // At L0, capacity is 1, so geometry_complexity is always nullopt.
  EXPECT_FALSE(sum.geometry_complexity.has_value());
}
