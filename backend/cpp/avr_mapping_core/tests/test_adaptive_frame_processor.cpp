#include "avr_mapping/adaptive_frame_processor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "avr_mapping/point_xyz.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(AdaptiveFrameProcessorTest, FactoryRejectsInvalidConfig) {
  ResolutionPolicyConfig bad_config;
  bad_config.distance_threshold_1 = -5.0;  // Invalid threshold
  EXPECT_FALSE(bad_config.validate());

  auto processor = AdaptiveFrameProcessor::create(bad_config);
  EXPECT_EQ(processor, nullptr);

  ResolutionPolicyConfig good_config;
  EXPECT_TRUE(good_config.validate());
  auto valid_processor = AdaptiveFrameProcessor::create(good_config);
  EXPECT_NE(valid_processor, nullptr);
}

// Section 34: Valid Empty Frame
TEST(AdaptiveFrameProcessorTest, ValidEmptyFrameLifecycle) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  std::vector<PointXYZ> empty_points;
  const avr::spatial::WorldPoint2D sensor_origin{5.0, 10.0};

  auto stats_opt = processor->processFrame(empty_points, 101, sensor_origin);
  ASSERT_TRUE(stats_opt.has_value());

  const auto& stats = *stats_opt;
  EXPECT_EQ(stats.frame_seq, 101u);
  EXPECT_EQ(stats.input_points, 0u);
  EXPECT_EQ(stats.accepted_points, 0u);
  EXPECT_EQ(stats.active_base_cells, 0u);
  EXPECT_EQ(stats.seed_root_count, 0u);
  EXPECT_EQ(stats.regions_evaluated, 0u);
  EXPECT_EQ(stats.regions_split, 0u);
  EXPECT_EQ(stats.empty_children_held, 0u);
  EXPECT_EQ(stats.final_node_count, 0u);
  EXPECT_EQ(stats.final_leaf_count, 0u);
  EXPECT_EQ(stats.final_internal_count, 0u);

  EXPECT_TRUE(processor->frameEvidence().hasFrame());
  EXPECT_EQ(processor->hierarchy().nodeCount(), 0u);
  EXPECT_EQ(processor->hierarchy().leafCount(), 0u);
  EXPECT_TRUE(processor->decisionSnapshot().empty());
}

// Section 35: Invalid Sensor Origin Transaction Test
TEST(AdaptiveFrameProcessorTest, InvalidSensorOriginPreservesPreviousFrameTransaction) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Frame A: near single cell producing mixed topology
  std::vector<PointXYZ> frame_a_pts = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 1.0},
  };
  const avr::spatial::WorldPoint2D sensor_origin_a{0.0, 0.0};

  auto stats_a_opt = processor->processFrame(frame_a_pts, 1, sensor_origin_a);
  ASSERT_TRUE(stats_a_opt.has_value());
  EXPECT_GT(stats_a_opt->final_leaf_count, 0u);

  const auto leaf_snapshot_a = processor->hierarchy().leafSnapshot();
  const auto decision_snapshot_a = processor->decisionSnapshot();
  const auto last_stats_a = processor->lastStats();

  // Frame B: Invalid sensor origin (NaN)
  std::vector<PointXYZ> frame_b_pts = {
      PointXYZ{.x = 5.0, .y = 5.0, .z = 0.0},
  };
  const avr::spatial::WorldPoint2D invalid_origin{std::numeric_limits<double>::quiet_NaN(), 0.0};

  auto stats_b_opt = processor->processFrame(frame_b_pts, 2, invalid_origin);
  EXPECT_FALSE(stats_b_opt.has_value());

  // Verify Frame A state is preserved unmodified
  EXPECT_EQ(processor->hierarchy().leafSnapshot(), leaf_snapshot_a);
  EXPECT_EQ(processor->decisionSnapshot(), decision_snapshot_a);
  EXPECT_EQ(processor->lastStats(), last_stats_a);
  EXPECT_EQ(processor->frameEvidence().frameSeq(), 1u);
}

// Section 36: Valid All-Invalid Replacement Test
TEST(AdaptiveFrameProcessorTest, ValidAllInvalidFrameReplacement) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Populate Frame A
  std::vector<PointXYZ> frame_a_pts = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 1.0},
  };
  auto stats_a = processor->processFrame(frame_a_pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_a.has_value());
  EXPECT_GT(processor->hierarchy().nodeCount(), 0u);

  // Frame B: valid sensor origin, but all points non-finite
  std::vector<PointXYZ> frame_b_pts = {
      PointXYZ{.x = std::numeric_limits<double>::quiet_NaN(), .y = 0.0, .z = 0.0},
      PointXYZ{.x = 0.0, .y = std::numeric_limits<double>::infinity(), .z = 0.0},
  };
  auto stats_b = processor->processFrame(frame_b_pts, 2, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_b.has_value());

  EXPECT_EQ(stats_b->frame_seq, 2u);
  EXPECT_EQ(stats_b->input_points, 2u);
  EXPECT_EQ(stats_b->accepted_points, 0u);
  EXPECT_EQ(stats_b->rejected_non_finite, 2u);
  EXPECT_EQ(stats_b->active_base_cells, 0u);
  EXPECT_EQ(stats_b->final_node_count, 0u);
  EXPECT_EQ(stats_b->final_leaf_count, 0u);

  EXPECT_TRUE(processor->hierarchy().leafSnapshot().empty());
  EXPECT_TRUE(processor->decisionSnapshot().empty());
}

// Section 37: Far Flat Sparse Test (d >= 60m, flat, stays L3)
TEST(AdaptiveFrameProcessorTest, FarFlatSparseRemainsL3Leaf) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // L3 root cell at cx = 200, cy = 0: bounds [80.0, 80.40) x [0.0, 0.40). Center (80.20, 0.20).
  // Distance from (0, 0) is ~80.2m >= 60m -> distance baseline is L3.
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 80.05, .y = 0.05, .z = 0.0},
      PointXYZ{.x = 80.15, .y = 0.15, .z = 0.0},
  };

  auto stats_opt = processor->processFrame(points, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.regions_evaluated, 1u);
  EXPECT_EQ(stats.regions_split, 0u);
  EXPECT_EQ(stats.final_leaf_count, 1u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L3], 1u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 0u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 38, 47, 49: Far Distributed Complex Test (d >= 60m, vertical relief, splits to L2, target & reason inheritance)
TEST(AdaptiveFrameProcessorTest, FarDistributedComplexSplitsAndInheritsTarget) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // In L3 cell [80.0, 80.40) x [0.0, 0.40) at distance ~80m >= 60m.
  // We populate 16 distinct base cells in child 0 (SW child: [80.0, 80.20) x [0.0, 0.20)).
  // An L2 child has 4x4 = 16 base cells.
  // 8 cells at z = 0.0, 8 cells at z = 0.25 (span = 0.25m).
  // For the L3 root: occupied = 16, capacity = 64 -> density = 0.25 >= 0.25.
  // Geometry complexity: 0.25 / 0.40 = 0.625 >= 0.50 (high geometry threshold).
  // L3 root requests L2 via GEOMETRY_COMPLEXITY!
  std::vector<PointXYZ> points;
  for (int cy = 0; cy < 4; ++cy) {
    for (int cx = 0; cx < 4; ++cx) {
      double px = 80.0 + cx * 0.05 + 0.025;
      double py = 0.0 + cy * 0.05 + 0.025;
      double pz = (cx < 2) ? 0.0 : 0.25;
      points.push_back(PointXYZ{.x = px, .y = py, .z = pz});
    }
  }

  auto stats_opt = processor->processFrame(points, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_GE(stats.regions_split, 1u);
  EXPECT_GT(stats.empty_children_held, 0u);

  const auto records = processor->decisionSnapshot();
  ASSERT_GE(records.size(), 2u);

  // Root record: requested L2 due to GEOMETRY_COMPLEXITY
  const auto& root_rec = records[0];
  EXPECT_EQ(root_rec.region.level, avr::spatial::L3);
  EXPECT_EQ(root_rec.local_decision.requested_level, avr::spatial::L2);
  EXPECT_TRUE(root_rec.local_reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
  EXPECT_TRUE(root_rec.split_applied);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 39: Far Sparse Complex-Looking Test (large z, but only 1 occupied cell -> no refinement)
TEST(AdaptiveFrameProcessorTest, FarSparseComplexLookingNoRefinementWithoutSupport) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // In L3 root [80.0, 80.40) x [0.0, 0.40) at distance >= 60m.
  // Multiple points in the EXACT SAME single L0 base cell with extreme z difference:
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 5.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 10.0},
  };

  auto stats_opt = processor->processFrame(points, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  // Single base cell occupied -> geometry support is nullopt -> no geometry refinement!
  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.regions_split, 0u);
  EXPECT_EQ(stats.final_leaf_count, 1u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L3], 1u);
}

// Section 40: Near Single-Cell Test (d < 10m -> L0 quadtree path = 3 L2 + 3 L1 + 4 L0 = 10 leaves)
TEST(AdaptiveFrameProcessorTest, NearSingleCellRefinesL0QuadtreePath) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Single point near sensor (2.025, 0.025, 0.0) -> distance ~2.0m < 10m -> baseline L0
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
  };

  auto stats_opt = processor->processFrame(points, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.regions_split, 3u);  // L3 -> L2 -> L1 splits
  EXPECT_EQ(stats.empty_children_held, 9u);  // 3 L2 + 3 L1 + 3 L0 empty siblings

  // Final leaf structure: 3 L2 + 3 L1 + 4 L0 = 10 leaves
  EXPECT_EQ(stats.final_leaf_count, 10u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L3], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 3u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 3u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 4u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 41: Mid-Range Test (10 <= d < 30m -> L1 baseline -> 3 L2 + 4 L1 = 7 leaves)
TEST(AdaptiveFrameProcessorTest, MidRangeDistanceBandRefinesToL1) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Point at (20.025, 0.025) -> distance ~20.0m in [10.0, 30.0) -> baseline L1
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0},
  };

  auto stats_opt = processor->processFrame(points, 50, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.regions_split, 2u);  // L3 -> L2 splits
  EXPECT_EQ(stats.final_leaf_count, 7u);  // 3 L2 + 4 L1 = 7 leaves
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 4u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 0u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 42: L2 Distance Band Test (30 <= d < 60m -> L2 baseline -> 4 L2 leaves)
TEST(AdaptiveFrameProcessorTest, L2DistanceBandRefinesToL2) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Point at (40.025, 0.025) -> distance ~40.0m in [30.0, 60.0) -> baseline L2
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 40.025, .y = 0.025, .z = 0.0},
  };

  auto stats_opt = processor->processFrame(points, 60, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  const auto& stats = *stats_opt;

  EXPECT_EQ(stats.seed_root_count, 1u);
  EXPECT_EQ(stats.regions_split, 1u);  // L3 splits to L2
  EXPECT_EQ(stats.final_leaf_count, 4u);  // 4 L2 leaves
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L2], 4u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L1], 0u);
  EXPECT_EQ(stats.leaf_count_by_level[avr::spatial::L0], 0u);

  EXPECT_TRUE(processor->hierarchy().validateInvariants().valid);
}

// Section 43: Distance Boundaries 10m, 30m, 60m
TEST(AdaptiveFrameProcessorTest, DistanceBoundaries10m30m60m) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Boundary 10m: Just inside [0, 10) vs just inside [10, 30)
  // L3 cell at (9.8, 0.0) center (9.8+0.2=10.0). Distance = 9.8m vs 10.2m.
  // Using sensor origin shift: point at (0.025, 0.025).
  // Origin at (-9.9, 0) -> dist = 9.925 < 10.0 -> L0
  {
    std::vector<PointXYZ> pts = {PointXYZ{.x = 0.025, .y = 0.025, .z = 0.0}};
    auto st = processor->processFrame(pts, 1, avr::spatial::WorldPoint2D{-9.7, 0.0});
    ASSERT_TRUE(st.has_value());
    EXPECT_GT(st->leaf_count_by_level[avr::spatial::L0], 0u);
  }

  // Origin at (-10.5, 0) -> dist ~10.7m in [10, 30) -> finest leaf is L1
  {
    std::vector<PointXYZ> pts = {PointXYZ{.x = 0.025, .y = 0.025, .z = 0.0}};
    auto st = processor->processFrame(pts, 2, avr::spatial::WorldPoint2D{-10.5, 0.0});
    ASSERT_TRUE(st.has_value());
    EXPECT_EQ(st->leaf_count_by_level[avr::spatial::L0], 0u);
    EXPECT_GT(st->leaf_count_by_level[avr::spatial::L1], 0u);
  }

  // Origin at (-35.0, 0) -> dist ~35.2m in [30, 60) -> finest leaf is L2
  {
    std::vector<PointXYZ> pts = {PointXYZ{.x = 0.025, .y = 0.025, .z = 0.0}};
    auto st = processor->processFrame(pts, 3, avr::spatial::WorldPoint2D{-35.0, 0.0});
    ASSERT_TRUE(st.has_value());
    EXPECT_EQ(st->leaf_count_by_level[avr::spatial::L0], 0u);
    EXPECT_EQ(st->leaf_count_by_level[avr::spatial::L1], 0u);
    EXPECT_GT(st->leaf_count_by_level[avr::spatial::L2], 0u);
  }

  // Origin at (-65.0, 0) -> dist ~65.2m >= 60 -> stays L3
  {
    std::vector<PointXYZ> pts = {PointXYZ{.x = 0.025, .y = 0.025, .z = 0.0}};
    auto st = processor->processFrame(pts, 4, avr::spatial::WorldPoint2D{-65.0, 0.0});
    ASSERT_TRUE(st.has_value());
    EXPECT_EQ(st->leaf_count_by_level[avr::spatial::L3], 1u);
  }
}

// Section 48: Child Finer Override Test
TEST(AdaptiveFrameProcessorTest, ChildFinerOverride) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Region at medium distance (~20m -> parent baseline L1)
  // But inside child region, vertical complexity or cues request L0.
  // Then effective target = min(L0, L1) = L0. Child refines to L0!
  // Near distance (d < 10m): root requests L0, all descendants refine to L0.
  std::vector<PointXYZ> pts = {
      PointXYZ{.x = 1.025, .y = 1.025, .z = 0.0},
  };
  auto st = processor->processFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st.has_value());

  const auto records = processor->decisionSnapshot();
  for (const auto& rec : records) {
    EXPECT_LE(rec.effective_target, rec.inherited_target);
  }
}

// Section 50: Empty Child No-Decision Test
TEST(AdaptiveFrameProcessorTest, EmptyChildNoDecisionAndRemainsLeaf) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Single point in SW corner of L3 root [0, 0.40) x [0, 0.40) near sensor
  std::vector<PointXYZ> pts = {
      PointXYZ{.x = 0.025, .y = 0.025, .z = 0.0},
  };
  auto st = processor->processFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st.has_value());

  const auto records = processor->decisionSnapshot();
  // None of the records should have observation_count == 0
  for (const auto& rec : records) {
    EXPECT_GT(rec.observation_count, 0u);
  }

  // Hierarchy has leaves at L2, L1, L0
  const auto leaves = processor->hierarchy().leafSnapshot();
  for (const auto& leaf_key : leaves) {
    // If leaf is empty, it must be LEAF (verified by being in leafSnapshot)
    auto summary = processor->frameEvidence().summarize(leaf_key);
    if (summary.observation_count == 0) {
      EXPECT_TRUE(processor->hierarchy().isLeaf(leaf_key));
    }
  }
}

// Section 52: Frame Rebuild Coarsening Demonstration
TEST(AdaptiveFrameProcessorTest, FrameRebuildCoarseningDemonstration) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Frame A: Near single cell -> refines to L0 (10 leaves)
  std::vector<PointXYZ> pts_a = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
  };
  auto st_a = processor->processFrame(pts_a, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st_a.has_value());
  EXPECT_EQ(st_a->final_leaf_count, 10u);

  // Frame B: Same world region, but sensor is now far away (origin at -100, 0)
  // Distance ~102m >= 60m -> coarsens immediately to 1 coarse L3 leaf!
  auto st_b = processor->processFrame(pts_a, 2, avr::spatial::WorldPoint2D{-100.0, 0.0});
  ASSERT_TRUE(st_b.has_value());
  EXPECT_EQ(st_b->final_leaf_count, 1u);
  EXPECT_EQ(st_b->leaf_count_by_level[avr::spatial::L3], 1u);
}

// Section 53: Same Frame Repeatability
TEST(AdaptiveFrameProcessorTest, SameFrameRepeatability) {
  auto proc1 = AdaptiveFrameProcessor::create();
  auto proc2 = AdaptiveFrameProcessor::create();
  ASSERT_NE(proc1, nullptr);
  ASSERT_NE(proc2, nullptr);

  std::vector<PointXYZ> pts = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
  };
  const avr::spatial::WorldPoint2D origin{0.0, 0.0};

  auto st1 = proc1->processFrame(pts, 10, origin);
  auto st2 = proc2->processFrame(pts, 10, origin);
  ASSERT_TRUE(st1.has_value());
  ASSERT_TRUE(st2.has_value());

  EXPECT_EQ(*st1, *st2);
  EXPECT_EQ(proc1->decisionSnapshot(), proc2->decisionSnapshot());
  EXPECT_EQ(proc1->hierarchy().leafSnapshot(), proc2->hierarchy().leafSnapshot());
}

// Section 3: Point-Order Robustness (Strengthened)
TEST(AdaptiveFrameProcessorTest, PointOrderRobustness) {
  auto proc1 = AdaptiveFrameProcessor::create();
  auto proc2 = AdaptiveFrameProcessor::create();
  ASSERT_NE(proc1, nullptr);
  ASSERT_NE(proc2, nullptr);

  std::vector<PointXYZ> pts1 = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 40.025, .y = 0.025, .z = 0.0},
  };
  std::vector<PointXYZ> pts2 = pts1;
  std::reverse(pts2.begin(), pts2.end());

  const avr::spatial::WorldPoint2D origin{0.0, 0.0};

  auto st1 = proc1->processFrame(pts1, 100, origin);
  auto st2 = proc2->processFrame(pts2, 100, origin);
  ASSERT_TRUE(st1.has_value());
  ASSERT_TRUE(st2.has_value());

  EXPECT_EQ(st1->seed_root_count, st2->seed_root_count);
  EXPECT_EQ(st1->regions_evaluated, st2->regions_evaluated);
  EXPECT_EQ(st1->regions_split, st2->regions_split);
  EXPECT_EQ(st1->empty_children_held, st2->empty_children_held);
  EXPECT_EQ(st1->final_leaf_count, st2->final_leaf_count);
  EXPECT_EQ(st1->leaf_count_by_level, st2->leaf_count_by_level);

  EXPECT_EQ(proc1->hierarchy().leafSnapshot(), proc2->hierarchy().leafSnapshot());
  EXPECT_EQ(proc1->hierarchy().topologySnapshot(), proc2->hierarchy().topologySnapshot());
  EXPECT_EQ(proc1->decisionSnapshot(), proc2->decisionSnapshot());
}

// Section 2 & 4: Decision Snapshot Order Contract and Scrambled Input Order
TEST(AdaptiveFrameProcessorTest, DecisionSnapshotStrictlySortedByCellKey) {
  auto proc1 = AdaptiveFrameProcessor::create();
  auto proc2 = AdaptiveFrameProcessor::create();
  ASSERT_NE(proc1, nullptr);
  ASSERT_NE(proc2, nullptr);

  // Scrambled multi-root points
  std::vector<PointXYZ> scrambled_pts = {
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = -40.025, .y = -20.025, .z = 0.0},
      PointXYZ{.x = 2.025, .y = 2.025, .z = 0.0},
      PointXYZ{.x = -10.025, .y = 30.025, .z = 0.0},
  };

  auto st1 = proc1->processFrame(scrambled_pts, 101, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st1.has_value());

  const auto snapshot1 = proc1->decisionSnapshot();
  ASSERT_GT(snapshot1.size(), 1u);

  // Contract: decisionSnapshot()[i].region < decisionSnapshot()[i+1].region
  for (size_t i = 0; i + 1 < snapshot1.size(); ++i) {
    EXPECT_LT(snapshot1[i].region, snapshot1[i + 1].region);
  }

  // Reversed points must produce identical strictly sorted decision snapshot
  auto reversed_pts = scrambled_pts;
  std::reverse(reversed_pts.begin(), reversed_pts.end());

  auto st2 = proc2->processFrame(reversed_pts, 101, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st2.has_value());

  EXPECT_EQ(proc1->decisionSnapshot(), proc2->decisionSnapshot());
}

// Section 5, 6, 7: Adaptive Frame Stats and Counter Consistency
TEST(AdaptiveFrameProcessorTest, AdaptiveFrameStatsAndCounterConsistency) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  std::vector<PointXYZ> points = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = std::numeric_limits<double>::quiet_NaN(), .y = 0.0, .z = 0.0},
  };

  auto st_opt = processor->processFrame(points, 200, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st_opt.has_value());
  const auto& stats = *st_opt;
  const auto h_stats = processor->hierarchy().statistics();
  const auto& ev_stats = processor->frameEvidence().stats();
  const auto records = processor->decisionSnapshot();

  // Section 5: Stats vs Hierarchy consistency
  EXPECT_EQ(stats.final_hierarchy_tile_count, h_stats.tile_count);
  EXPECT_EQ(stats.final_node_count, h_stats.node_count);
  EXPECT_EQ(stats.final_leaf_count, h_stats.leaf_count);
  EXPECT_EQ(stats.final_internal_count, h_stats.internal_count);
  EXPECT_EQ(stats.leaf_count_by_level, h_stats.leaf_count_by_level);

  size_t leaf_sum = 0;
  for (size_t c : stats.leaf_count_by_level) {
    leaf_sum += c;
  }
  EXPECT_EQ(leaf_sum, stats.final_leaf_count);
  EXPECT_EQ(stats.final_node_count, stats.final_leaf_count + stats.final_internal_count);

  // Section 6: Processing counter consistency
  EXPECT_EQ(stats.regions_evaluated, records.size());
  size_t split_count = 0;
  for (const auto& rec : records) {
    if (rec.split_applied) {
      ++split_count;
    }
  }
  EXPECT_EQ(stats.regions_split, split_count);

  // Section 7: Frame point counter consistency with FrameEvidence
  EXPECT_EQ(stats.input_points, ev_stats.input_points);
  EXPECT_EQ(stats.accepted_points, ev_stats.accepted_points);
  EXPECT_EQ(stats.rejected_non_finite, ev_stats.rejected_non_finite);
  EXPECT_EQ(stats.rejected_out_of_range, ev_stats.rejected_out_of_range);
  EXPECT_EQ(stats.rejected_numeric, ev_stats.rejected_numeric);
  EXPECT_EQ(stats.evidence_tile_count, ev_stats.tile_count);
  EXPECT_EQ(stats.active_base_cells, ev_stats.active_base_cells);
}

// Section 8 & 9: Split and Effective Target Contract
TEST(AdaptiveFrameProcessorTest, SplitAndEffectiveTargetContract) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Near point -> L0 quadtree refinement
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 1.025, .y = 1.025, .z = 0.0},
  };

  auto st_opt = processor->processFrame(points, 300, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st_opt.has_value());

  const auto records = processor->decisionSnapshot();
  for (const auto& rec : records) {
    // Section 8: effective_target = min(requested, inherited)
    EXPECT_EQ(rec.effective_target,
              std::min(rec.local_decision.requested_level, rec.inherited_target));
    EXPECT_LE(rec.effective_target, avr::spatial::L3);
    EXPECT_LE(rec.effective_target, rec.local_decision.requested_level);

    // Section 9: Split contract
    if (rec.split_applied) {
      EXPECT_GT(rec.region.level, rec.effective_target);
      EXPECT_GT(rec.region.level, avr::spatial::L0);
      EXPECT_EQ(processor->hierarchy().nodeState(rec.region), NodeState::INTERNAL);
    }
  }
}

// Section 10 & 11: Reason Lineage Union Across a 3-Level Refinement Path
TEST(AdaptiveFrameProcessorTest, ReasonLineageUnionThreeLevelPath) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  // Near single cell -> L3 -> L2 -> L1 -> L0 path
  std::vector<PointXYZ> points = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
  };

  auto st_opt = processor->processFrame(points, 400, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st_opt.has_value());

  const auto records = processor->decisionSnapshot();
  ASSERT_GE(records.size(), 3u);

  for (const auto& rec : records) {
    // Section 11: Reason union contract
    EXPECT_EQ(rec.effective_reasons, rec.local_reasons | rec.inherited_reasons);

    if (rec.region.level == avr::spatial::L3) {
      // Root record must have empty inherited reasons
      EXPECT_TRUE(rec.inherited_reasons.empty());
    }
  }
}

// Section 20: Root Seed Consistency With Spatial Helpers
TEST(AdaptiveFrameProcessorTest, RootSeedConsistencyWithSpatialHelpers) {
  auto processor = AdaptiveFrameProcessor::create();
  ASSERT_NE(processor, nullptr);

  std::vector<PointXYZ> points = {
      PointXYZ{.x = 2.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 20.025, .y = 0.025, .z = 0.0},
      PointXYZ{.x = 80.025, .y = 0.025, .z = 0.0},
  };

  auto st_opt = processor->processFrame(points, 500, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(st_opt.has_value());
  const auto& stats = *st_opt;

  // Independently discover unique L3 roots from base cell snapshot
  const auto snapshot = processor->frameEvidence().baseCellSnapshot();
  std::vector<avr::spatial::CellKey> expected_roots;
  for (const auto& entry : snapshot) {
    auto p1 = avr::spatial::parentOf(entry.key);
    auto p2 = p1 ? avr::spatial::parentOf(*p1) : std::nullopt;
    auto p3 = p2 ? avr::spatial::parentOf(*p2) : std::nullopt;
    ASSERT_TRUE(p3.has_value());
    expected_roots.push_back(*p3);
  }
  std::sort(expected_roots.begin(), expected_roots.end());
  expected_roots.erase(std::unique(expected_roots.begin(), expected_roots.end()),
                       expected_roots.end());

  EXPECT_EQ(stats.seed_root_count, expected_roots.size());

  // Verify every expected root is allocated in final hierarchy
  for (const auto& root_key : expected_roots) {
    EXPECT_TRUE(processor->hierarchy().isAllocated(root_key));
  }
}

}  // namespace
}  // namespace avr::mapping
