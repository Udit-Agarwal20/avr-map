#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/constants.hpp"

using namespace avr::mapping;
using namespace avr::spatial;

TEST(FrameEvidenceTest, ValidFrameAndStatsConsistency) {
  FrameEvidence fe;
  EXPECT_FALSE(fe.hasFrame());
  EXPECT_EQ(fe.tileCount(), 0);
  EXPECT_EQ(fe.activeBaseCellCount(), 0);

  std::vector<PointXYZ> points = {
      PointXYZ{1.02, 2.03, 0.5},   // valid
      PointXYZ{1.03, 2.04, 0.6},   // same L0 cell
      PointXYZ{5.10, -3.20, 1.2},  // different tile
      PointXYZ{std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0},  // non-finite
      PointXYZ{1.0, std::numeric_limits<double>::infinity(), 0.0},   // non-finite
      PointXYZ{1e15, 1e15, 0.0},   // out of range XY
  };

  const auto stats_opt = fe.buildFrame(points, 101, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 101);
  EXPECT_EQ(stats.frame_seq, 101);
  EXPECT_EQ(stats.input_points, 6);
  EXPECT_EQ(stats.accepted_points, 3);
  EXPECT_EQ(stats.rejected_non_finite, 2);
  EXPECT_EQ(stats.rejected_out_of_range, 1);
  EXPECT_EQ(stats.rejected_numeric, 0);
  EXPECT_TRUE(stats.isConsistent());

  EXPECT_EQ(stats.active_base_cells, 2);
  EXPECT_EQ(stats.tile_count, 2);
  EXPECT_EQ(fe.activeBaseCellCount(), 2);
  EXPECT_EQ(fe.tileCount(), 2);
}

TEST(FrameEvidenceTest, InvalidSensorOriginTransaction) {
  FrameEvidence fe;

  // 1. Build valid frame A
  std::vector<PointXYZ> points_a = {
      PointXYZ{0.1, 0.1, 1.0},
      PointXYZ{0.2, 0.2, 2.0},
  };
  const auto stats_a = fe.buildFrame(points_a, 10, WorldPoint2D{5.0, 5.0});
  ASSERT_TRUE(stats_a.has_value());
  EXPECT_EQ(fe.frameSeq(), 10);
  EXPECT_EQ(fe.activeBaseCellCount(), 2);

  // 2. Attempt frame B with NaN sensor x
  std::vector<PointXYZ> points_b = {
      PointXYZ{10.0, 10.0, 3.0},
  };
  const auto stats_b_nan = fe.buildFrame(
      points_b, 11, WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0});
  EXPECT_FALSE(stats_b_nan.has_value());

  // Invariant: frame A evidence remains completely intact and uncorrupted
  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 10);
  EXPECT_EQ(fe.activeBaseCellCount(), 2);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().x, 5.0);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().y, 5.0);

  // 3. Attempt frame C with Inf sensor y
  const auto stats_c_inf = fe.buildFrame(
      points_b, 12, WorldPoint2D{0.0, std::numeric_limits<double>::infinity()});
  EXPECT_FALSE(stats_c_inf.has_value());

  // Invariant: frame A evidence still intact
  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 10);
  EXPECT_EQ(fe.activeBaseCellCount(), 2);
}

TEST(FrameEvidenceTest, FrameReplacement) {
  FrameEvidence fe;

  // Build frame 10
  std::vector<PointXYZ> points_10 = {
      PointXYZ{1.0, 1.0, 5.0},
      PointXYZ{2.0, 2.0, 6.0},
  };
  const auto stats_10 = fe.buildFrame(points_10, 10, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_10.has_value());
  EXPECT_EQ(fe.frameSeq(), 10);
  EXPECT_EQ(fe.activeBaseCellCount(), 2);

  // Build frame 42 with completely different points in different region
  std::vector<PointXYZ> points_42 = {
      PointXYZ{-10.0, -10.0, 1.0},
  };
  const auto stats_42 = fe.buildFrame(points_42, 42, WorldPoint2D{1.0, 2.0});
  ASSERT_TRUE(stats_42.has_value());
  EXPECT_EQ(fe.frameSeq(), 42);
  EXPECT_EQ(fe.activeBaseCellCount(), 1);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().x, 1.0);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().y, 2.0);

  // Query location of frame 10: must be empty
  const auto old_cell_key = CellKey::fromWorldPoint(WorldPoint2D{1.0, 1.0}, L0);
  ASSERT_TRUE(old_cell_key.has_value());
  const auto old_summary = fe.summarize(*old_cell_key);
  EXPECT_TRUE(old_summary.isEmpty());
  EXPECT_EQ(old_summary.observation_count, 0);
  EXPECT_EQ(old_summary.occupied_base_cells, 0);

  // Query location of frame 42: must be present
  const auto new_cell_key = CellKey::fromWorldPoint(WorldPoint2D{-10.0, -10.0}, L0);
  ASSERT_TRUE(new_cell_key.has_value());
  const auto new_summary = fe.summarize(*new_cell_key);
  EXPECT_FALSE(new_summary.isEmpty());
  EXPECT_EQ(new_summary.observation_count, 1);
  EXPECT_EQ(new_summary.occupied_base_cells, 1);
}

TEST(FrameEvidenceTest, ExplicitReset) {
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{0.5, 0.5, 1.0}};
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.activeBaseCellCount(), 1);

  fe.reset();
  EXPECT_FALSE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 0);
  EXPECT_EQ(fe.tileCount(), 0);
  EXPECT_EQ(fe.activeBaseCellCount(), 0);
  EXPECT_TRUE(fe.stats().isConsistent());
}

TEST(FrameEvidenceTest, BaseCellSnapshotDeterministicOrder) {
  FrameEvidence fe;

  // Insert points in non-sorted order
  std::vector<PointXYZ> points = {
      PointXYZ{10.0, 10.0, 1.0},
      PointXYZ{-5.0, -5.0, 2.0},
      PointXYZ{0.0, 0.0, 3.0},
      PointXYZ{2.0, -1.0, 4.0},
  };
  fe.buildFrame(points, 7, WorldPoint2D{0.0, 0.0});

  const auto snapshot = fe.baseCellSnapshot();
  EXPECT_EQ(snapshot.size(), 4);

  // Verify strictly sorted by CellKey (Morton order)
  for (size_t i = 1; i < snapshot.size(); ++i) {
    EXPECT_LT(snapshot[i - 1].key, snapshot[i].key);
  }
}

TEST(FrameEvidenceTest, ExtremeElevationSafety) {
  FrameEvidence fe;

  // An initial valid point followed by extreme finite values that overflow Welford M2
  std::vector<PointXYZ> points = {
      PointXYZ{0.02, 0.02, 1e200},
      PointXYZ{0.02, 0.02, -1e200},  // delta = -2e200 -> delta^2 overflows IEEE 754 double (max ~1.8e308)
  };

  const auto stats_opt = fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.input_points, 2);
  EXPECT_EQ(stats.accepted_points, 1);
  EXPECT_EQ(stats.rejected_numeric, 1);
  EXPECT_TRUE(stats.isConsistent());

  // Cell state must remain valid and uncorrupted
  const auto key = CellKey::fromWorldPoint(WorldPoint2D{0.02, 0.02}, L0);
  ASSERT_TRUE(key.has_value());
  const auto summary = fe.summarize(*key);
  EXPECT_EQ(summary.observation_count, 1);
  ASSERT_TRUE(summary.elevation_mean.has_value());
  EXPECT_DOUBLE_EQ(*summary.elevation_mean, 1e200);
}

// Milestone 5: Valid empty frame lifecycle and representation
TEST(FrameEvidenceTest, ValidEmptyFrameLifecycle) {
  FrameEvidence fe;
  EXPECT_FALSE(fe.hasFrame());

  // Empty span with valid finite sensor origin
  std::span<const PointXYZ> empty_points{};
  const auto stats_opt = fe.buildFrame(empty_points, 42, WorldPoint2D{10.0, 20.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 42);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().x, 10.0);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().y, 20.0);
  EXPECT_EQ(stats.input_points, 0);
  EXPECT_EQ(stats.accepted_points, 0);
  EXPECT_EQ(stats.rejected_non_finite, 0);
  EXPECT_EQ(stats.rejected_out_of_range, 0);
  EXPECT_EQ(stats.rejected_numeric, 0);
  EXPECT_EQ(stats.active_base_cells, 0);
  EXPECT_EQ(stats.tile_count, 0);
  EXPECT_TRUE(stats.isConsistent());
}

// Milestone 5: Valid all-invalid frame replaces previous valid evidence
TEST(FrameEvidenceTest, ValidAllInvalidFrameReplacement) {
  FrameEvidence fe;

  // 1. Build Frame A with valid evidence
  std::vector<PointXYZ> points_a = {PointXYZ{1.0, 1.0, 5.0}};
  const auto stats_a = fe.buildFrame(points_a, 10, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_a.has_value());
  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.activeBaseCellCount(), 1);

  // 2. Build Frame B with valid origin but 10 non-finite invalid points
  std::vector<PointXYZ> points_b;
  for (int i = 0; i < 10; ++i) {
    points_b.push_back(PointXYZ{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  }

  const auto stats_b = fe.buildFrame(points_b, 20, WorldPoint2D{5.0, 5.0});
  ASSERT_TRUE(stats_b.has_value());
  EXPECT_TRUE(fe.hasFrame());
  EXPECT_EQ(fe.frameSeq(), 20);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().x, 5.0);
  EXPECT_DOUBLE_EQ(fe.sensorOrigin().y, 5.0);

  // Frame B stats must reflect 10 rejections and 0 active cells
  EXPECT_EQ(stats_b->input_points, 10);
  EXPECT_EQ(stats_b->rejected_non_finite, 10);
  EXPECT_EQ(stats_b->accepted_points, 0);
  EXPECT_EQ(stats_b->active_base_cells, 0);
  EXPECT_EQ(stats_b->tile_count, 0);
  EXPECT_TRUE(stats_b->isConsistent());

  // Frame A evidence must be completely gone
  const auto key_a = CellKey::fromWorldPoint(WorldPoint2D{1.0, 1.0}, L0);
  ASSERT_TRUE(key_a.has_value());
  const auto sum_a = fe.summarize(*key_a);
  EXPECT_TRUE(sum_a.isEmpty());
  EXPECT_EQ(sum_a.observation_count, 0);
}

// Milestone 5: Numeric rejection on existing cell leaves cell, tile, and active counts unchanged
TEST(FrameEvidenceTest, NumericRejectionExistingCellLeavesCellAndTileUnchanged) {
  FrameEvidence fe;

  // Ingest valid point in cell
  std::vector<PointXYZ> points = {
      PointXYZ{0.02, 0.02, 1e200},
  };
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(fe.activeBaseCellCount(), 1);
  EXPECT_EQ(fe.tileCount(), 1);

  // Ingest frame with the same cell plus a point that causes numeric rejection in that same cell
  std::vector<PointXYZ> points_with_rej = {
      PointXYZ{0.02, 0.02, 1e200},
      PointXYZ{0.02, 0.02, -1e200},  // rejected numeric
  };
  const auto stats = fe.buildFrame(points_with_rej, 2, WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->input_points, 2);
  EXPECT_EQ(stats->accepted_points, 1);
  EXPECT_EQ(stats->rejected_numeric, 1);
  EXPECT_EQ(stats->active_base_cells, 1);
  EXPECT_EQ(stats->tile_count, 1);
  EXPECT_TRUE(stats->isConsistent());

  const auto key = CellKey::fromWorldPoint(WorldPoint2D{0.02, 0.02}, L0);
  ASSERT_TRUE(key.has_value());
  const auto summary = fe.summarize(*key);
  EXPECT_EQ(summary.observation_count, 1);
  ASSERT_TRUE(summary.elevation_mean.has_value());
  EXPECT_DOUBLE_EQ(*summary.elevation_mean, 1e200);
}
