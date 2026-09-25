#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(PersistentPayloadFusionTest, MultiFrameCoarseFusion) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  // Frame 10: 2 points in L3 root
  const std::vector<PointXYZ> frame1_pts = {
      {0.1, 0.1, 1.0},
      {0.2, 0.2, 3.0},
  };
  ASSERT_TRUE(evidence.buildFrame(frame1_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f1_stats = map.fuseFrameEvidence(evidence);
  ASSERT_TRUE(f1_stats.has_value());
  EXPECT_EQ(f1_stats->new_payload_cells, 1ULL);
  EXPECT_EQ(f1_stats->observations_fused, 2ULL);

  // Frame 20: 2 more points in same root
  const std::vector<PointXYZ> frame2_pts = {
      {0.1, 0.2, 5.0},
      {0.2, 0.1, 7.0},
  };
  ASSERT_TRUE(evidence.buildFrame(frame2_pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f2_stats = map.fuseFrameEvidence(evidence);
  ASSERT_TRUE(f2_stats.has_value());
  EXPECT_EQ(f2_stats->updated_payload_cells, 1ULL);
  EXPECT_EQ(f2_stats->observations_fused, 2ULL);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto* payload = map.findActivePayload(root_key);
  ASSERT_NE(payload, nullptr);

  EXPECT_EQ(payload->observationCount(), 4ULL);
  EXPECT_EQ(payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(payload->lastSeenFrame(), 20ULL);
  EXPECT_EQ(payload->observedFrameCount(), 2ULL);
  EXPECT_DOUBLE_EQ(payload->elevation().mean(), 4.0);
  EXPECT_DOUBLE_EQ(payload->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(payload->elevation().max(), 7.0);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, KnownMultiFrameWelford) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  // Frame A: 2, 4, 4, 4 (z coordinates at (0.05, 0.05))
  const std::vector<PointXYZ> frame_a = {
      {0.05, 0.05, 2.0},
      {0.05, 0.05, 4.0},
      {0.05, 0.05, 4.0},
      {0.05, 0.05, 4.0},
  };
  ASSERT_TRUE(evidence.buildFrame(frame_a, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(evidence).has_value());

  // Frame B: 5, 5, 7, 9 (z coordinates at (0.05, 0.05))
  const std::vector<PointXYZ> frame_b = {
      {0.05, 0.05, 5.0},
      {0.05, 0.05, 5.0},
      {0.05, 0.05, 7.0},
      {0.05, 0.05, 9.0},
  };
  ASSERT_TRUE(evidence.buildFrame(frame_b, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(evidence).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  const auto* payload = map.findActivePayload(root_key);
  ASSERT_NE(payload, nullptr);

  EXPECT_EQ(payload->observationCount(), 8ULL);
  EXPECT_DOUBLE_EQ(payload->elevation().mean(), 5.0);
  EXPECT_DOUBLE_EQ(payload->elevation().populationVariance(), 4.0);
  EXPECT_NEAR(payload->elevation().sampleVariance(), 32.0 / 7.0, 1e-12);
  EXPECT_DOUBLE_EQ(payload->elevation().min(), 2.0);
  EXPECT_DOUBLE_EQ(payload->elevation().max(), 9.0);
}

TEST(PersistentPayloadFusionTest, ObservationConservationAcrossLeaves) {
  PersistentAdaptiveMap map{};

  // Manually refine base (0,0) to L1 and base (10, 10) remains L3
  ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1));

  FrameEvidence evidence{};
  std::vector<PointXYZ> pts;
  // Put 4 points in each of the four L1 children of the refined branch
  pts.push_back({0.02, 0.02, 1.0});
  pts.push_back({0.12, 0.02, 2.0});
  pts.push_back({0.02, 0.12, 3.0});
  pts.push_back({0.12, 0.12, 4.0});

  // Put 3 points in another unrefined L3 root (around (2.0, 2.0))
  pts.push_back({2.05, 2.05, 5.0});
  pts.push_back({2.15, 2.15, 6.0});
  pts.push_back({2.25, 2.25, 7.0});

  ASSERT_TRUE(evidence.buildFrame(pts, 100, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto fusion_stats = map.fuseFrameEvidence(evidence);
  ASSERT_TRUE(fusion_stats.has_value());

  EXPECT_EQ(fusion_stats->observations_fused, 7ULL);
  EXPECT_EQ(fusion_stats->observations_fused, evidence.statistics().accepted_points);
  EXPECT_EQ(fusion_stats->unique_leaves_fused, 5ULL);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, NegativeCoordinatesAndTileBoundaries) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  const std::vector<PointXYZ> pts = {
      {-0.85, -0.85, 10.0},
      {-5.25, -3.15, 20.0},
      {-12.45, 8.15, 30.0},
  };
  ASSERT_TRUE(evidence.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto stats = map.fuseFrameEvidence(evidence);
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->observations_fused, 3ULL);
  EXPECT_EQ(stats->accepted_points, 3ULL);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, ReplayRequiredLeafCanReceiveNewFusion) {
  PersistentAdaptiveMap map{};

  // Create root and fuse frame 1
  FrameEvidence ev1{};
  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(ev1.buildFrame(pts1, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  // Split root to L2: parent becomes retired cache, children become REPLAY_REQUIRED
  ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L2));

  const auto leaf_opt = map.hierarchy().findLeafContainingBase(avr::spatial::GridCoord2D{1, 1});
  ASSERT_TRUE(leaf_opt.has_value());
  EXPECT_TRUE(map.requiresReplay(*leaf_opt));
  EXPECT_EQ(map.findActivePayload(*leaf_opt), nullptr);

  // Fuse frame 2 into the replay-required child
  FrameEvidence ev2{};
  const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 6.0}};
  ASSERT_TRUE(ev2.buildFrame(pts2, 2, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f2_stats = map.fuseFrameEvidence(ev2);
  ASSERT_TRUE(f2_stats.has_value());
  EXPECT_EQ(f2_stats->replay_required_leaves_touched, 1ULL);

  // Leaf now has active payload with new frame observation
  const auto* child_payload = map.findActivePayload(*leaf_opt);
  ASSERT_NE(child_payload, nullptr);
  EXPECT_EQ(child_payload->observationCount(), 1ULL);
  EXPECT_DOUBLE_EQ(child_payload->elevation().mean(), 6.0);
  EXPECT_EQ(child_payload->firstSeenFrame(), 2ULL);

  // AND it remains marked as REPLAY_REQUIRED!
  EXPECT_TRUE(map.requiresReplay(*leaf_opt));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, NumericFailureAndMissingRootRegression) {
  PersistentAdaptiveMap map{};
  FrameEvidence ev1{};

  // Existing root B at (0.05, 0.05) with valid extreme elevation +1e200
  const std::vector<PointXYZ> b_pts = {{0.05, 0.05, 1e200}};
  ASSERT_TRUE(ev1.buildFrame(b_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f1_stats = map.fuseFrameEvidence(ev1);
  ASSERT_TRUE(f1_stats.has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 10ULL);
  EXPECT_EQ(map.statistics().total_frames_fused, 1ULL);
  EXPECT_EQ(map.statistics().total_observations_fused, 1ULL);
  EXPECT_TRUE(map.validateInvariants().is_valid);

  // Take deterministic snapshot of persistent map state before the failed fusion
  const auto topo_before = map.hierarchy().topologySnapshot();
  const auto leaves_before = map.hierarchy().leafSnapshot();
  const auto active_before = map.activePayloadSnapshot();
  const auto retired_before = map.retiredCacheSnapshot();
  const auto replay_before = map.replayRequiredSnapshot();
  const auto incomplete_before = map.retiredCacheIncompleteSnapshot();
  const auto stats_before = map.statistics();
  const auto last_seq_before = map.lastFusedFrameSeq();

  // Frame 20 contains:
  // A. Point in completely unseen root A at (10.05, 10.05) with safe elevation 0.0
  // B. Point in existing root B at (0.05, 0.05) with extreme elevation -1e200
  FrameEvidence ev2{};
  const std::vector<PointXYZ> ev2_pts = {
      {10.05, 10.05, 0.0},
      {0.05, 0.05, -1e200},
  };
  ASSERT_TRUE(ev2.buildFrame(ev2_pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(ev2.statistics().accepted_points, 2ULL);

  // Fusing ev2 must fail safely in PREPARE due to non-finite Welford variance arithmetic
  const auto f2_stats = map.fuseFrameEvidence(ev2);
  EXPECT_FALSE(f2_stats.has_value());

  // PREPARE must be strictly side-effect free: map state must be identical to before
  EXPECT_EQ(map.hierarchy().topologySnapshot(), topo_before);
  EXPECT_EQ(map.hierarchy().leafSnapshot(), leaves_before);
  EXPECT_EQ(map.activePayloadSnapshot(), active_before);
  EXPECT_EQ(map.retiredCacheSnapshot(), retired_before);
  EXPECT_EQ(map.replayRequiredSnapshot(), replay_before);
  EXPECT_EQ(map.retiredCacheIncompleteSnapshot(), incomplete_before);
  EXPECT_EQ(map.statistics(), stats_before);
  EXPECT_EQ(map.lastFusedFrameSeq(), last_seq_before);
  EXPECT_EQ(map.statistics().total_frames_fused, 1ULL);
  EXPECT_EQ(map.statistics().total_observations_fused, 1ULL);

  // Crucial check: root A must NOT exist in the hierarchy
  EXPECT_FALSE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{201, 201}));
  const auto root_a_key = AdaptiveHierarchy::rootKeyForBase(avr::spatial::GridCoord2D{201, 201});
  ASSERT_TRUE(root_a_key.has_value());
  EXPECT_FALSE(map.hierarchy().isLeaf(*root_a_key));

  // Map remains healthy and passes invariant validation
  EXPECT_TRUE(map.validateInvariants().is_valid);

  // Subsequent valid frame 30 succeeds and proves the map remains functional
  FrameEvidence ev3{};
  const std::vector<PointXYZ> ev3_pts = {
      {10.05, 10.05, 5.0},
  };
  ASSERT_TRUE(ev3.buildFrame(ev3_pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f3_stats = map.fuseFrameEvidence(ev3);
  ASSERT_TRUE(f3_stats.has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 30ULL);
  EXPECT_EQ(map.statistics().total_frames_fused, 2ULL);
  EXPECT_EQ(map.statistics().total_observations_fused, 2ULL);
  EXPECT_TRUE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{201, 201}));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, MultipleNewRootsInSingleFrame) {
  PersistentAdaptiveMap map{};
  FrameEvidence ev{};

  // Frame spanning 3 distinct, previously unallocated L3 roots
  // L3 span is 0.4m (8 cells of 5cm)
  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},    // Root 1: (0, 0)
      {5.05, 5.05, 2.0},    // Root 2: (101, 101)
      {-5.05, -5.05, 3.0},  // Root 3: (-101, -101)
  };
  ASSERT_TRUE(ev.buildFrame(pts, 100, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(ev.statistics().accepted_points, 3ULL);

  const auto stats = map.fuseFrameEvidence(ev);
  ASSERT_TRUE(stats.has_value());
  EXPECT_EQ(stats->roots_created, 3ULL);
  EXPECT_EQ(stats->unique_leaves_fused, 3ULL);
  EXPECT_EQ(stats->new_payload_cells, 3ULL);
  EXPECT_EQ(stats->observations_fused, 3ULL);
  EXPECT_EQ(stats->accepted_points, 3ULL);

  EXPECT_TRUE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{1, 1}));
  EXPECT_TRUE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{101, 101}));
  EXPECT_TRUE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{-101, -101}));
  EXPECT_EQ(map.hierarchy().statistics().leaf_count, 3ULL);
  EXPECT_EQ(map.hierarchy().statistics().tile_count, 2ULL);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, MixedExistingLeavesAndNewRoots) {
  PersistentAdaptiveMap map{};

  // Frame 10: populate root at (0.05, 0.05)
  FrameEvidence ev1{};
  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(ev1.buildFrame(pts1, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  // Refine existing root to L1
  ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L1));
  const auto leaf1_opt = map.hierarchy().findLeafContainingBase(avr::spatial::GridCoord2D{1, 1});
  ASSERT_TRUE(leaf1_opt.has_value());
  EXPECT_EQ(leaf1_opt->level, avr::spatial::L1);

  // Frame 20: populate the replay-required L1 child leaf with initial observation
  FrameEvidence ev2{};
  const std::vector<PointXYZ> pts2 = {{0.05, 0.05, 4.0}};
  ASSERT_TRUE(ev2.buildFrame(pts2, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto f2_stats = map.fuseFrameEvidence(ev2);
  ASSERT_TRUE(f2_stats.has_value());
  EXPECT_EQ(f2_stats->new_payload_cells, 1ULL);
  EXPECT_EQ(f2_stats->replay_required_leaves_touched, 1ULL);

  // Frame 30: point in existing leaf (updates payload) + point in new unseen root (new root & payload)
  FrameEvidence ev3{};
  const std::vector<PointXYZ> pts3 = {
      {0.05, 0.05, 5.0},   // updates existing L1 leaf payload
      {-8.05, 8.05, 6.0},  // in new unseen L3 root
  };
  ASSERT_TRUE(ev3.buildFrame(pts3, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(ev3.statistics().accepted_points, 2ULL);

  const auto f3_stats = map.fuseFrameEvidence(ev3);
  ASSERT_TRUE(f3_stats.has_value());
  EXPECT_EQ(f3_stats->roots_created, 1ULL);
  EXPECT_EQ(f3_stats->unique_leaves_fused, 2ULL);
  EXPECT_EQ(f3_stats->updated_payload_cells, 1ULL);
  EXPECT_EQ(f3_stats->new_payload_cells, 1ULL);
  EXPECT_EQ(f3_stats->observations_fused, 2ULL);
  EXPECT_EQ(map.lastFusedFrameSeq(), 30ULL);
  EXPECT_EQ(map.statistics().total_frames_fused, 3ULL);
  EXPECT_EQ(map.statistics().total_observations_fused, 4ULL);

  // The new root exists
  EXPECT_TRUE(map.hierarchy().hasRootForBase(avr::spatial::GridCoord2D{-161, 161}));
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentPayloadFusionTest, OrdinaryFailedFusionZeroMutationMatrix) {
  PersistentAdaptiveMap map{};

  // Frame 10: initial valid state
  FrameEvidence ev1{};
  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(ev1.buildFrame(pts1, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  const auto topo_baseline = map.hierarchy().topologySnapshot();
  const auto active_baseline = map.activePayloadSnapshot();
  const auto stats_baseline = map.statistics();

  // Test Case 1: Duplicate frame sequence
  FrameEvidence ev_dup{};
  const std::vector<PointXYZ> pts_dup = {{0.05, 0.05, 4.0}};
  ASSERT_TRUE(ev_dup.buildFrame(pts_dup, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(map.fuseFrameEvidence(ev_dup).has_value());
  EXPECT_EQ(map.hierarchy().topologySnapshot(), topo_baseline);
  EXPECT_EQ(map.activePayloadSnapshot(), active_baseline);
  EXPECT_EQ(map.statistics(), stats_baseline);

  // Test Case 2: Out-of-order frame sequence
  FrameEvidence ev_ooo{};
  const std::vector<PointXYZ> pts_ooo = {{0.05, 0.05, 4.0}};
  ASSERT_TRUE(ev_ooo.buildFrame(pts_ooo, 5, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(map.fuseFrameEvidence(ev_ooo).has_value());
  EXPECT_EQ(map.hierarchy().topologySnapshot(), topo_baseline);
  EXPECT_EQ(map.activePayloadSnapshot(), active_baseline);
  EXPECT_EQ(map.statistics(), stats_baseline);

  // Test Case 3: Empty evidence without built frame
  FrameEvidence ev_empty{};
  EXPECT_FALSE(map.fuseFrameEvidence(ev_empty).has_value());
  EXPECT_EQ(map.hierarchy().topologySnapshot(), topo_baseline);
  EXPECT_EQ(map.activePayloadSnapshot(), active_baseline);
  EXPECT_EQ(map.statistics(), stats_baseline);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace
}  // namespace avr::mapping
