#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_mapping/point_xyz.hpp"

namespace avr::mapping {

struct ResolutionHysteresisTestAccess {
  static void setRevision(ResolutionHysteresisController& ctrl, uint64_t rev) noexcept {
    ctrl.state_revision_ = rev;
  }
};

struct PersistentAdaptivePipelineTestAccess {
  static void setSimulateFusionFailure(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_fusion_failure_ = v;
  }
  static void setSimulateCacheCommitBadAlloc(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_cache_commit_bad_alloc_ = v;
  }
  static void setSimulateCacheCommitFailure(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_cache_commit_failure_ = v;
  }
  static void setSimulatePolicyCommitRevisionMismatch(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_policy_commit_revision_mismatch_ = v;
  }
  static void setSimulatePolicyCommitBadAlloc(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_policy_commit_bad_alloc_ = v;
  }
  static void setSimulateInvariantFailure(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_invariant_failure_ = v;
  }
  static void setFatalLatch(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.fatal_error_latched_ = v;
  }
  static ResolutionHysteresisController& getHysteresis(PersistentAdaptiveFramePipeline& p) noexcept {
    return p.hysteresis_;
  }
};

namespace {

// Helper to generate dense points in a specific local meter bounding box
std::vector<PointXYZ> makePointsInBox(
    double min_x, double max_x,
    double min_y, double max_y,
    double z_min, double z_max,
    size_t count_x = 4, size_t count_y = 4) {
  std::vector<PointXYZ> pts;
  pts.reserve(count_x * count_y);
  const double dx = (count_x > 1) ? (max_x - min_x) / static_cast<double>(count_x - 1) : 0.0;
  const double dy = (count_y > 1) ? (max_y - min_y) / static_cast<double>(count_y - 1) : 0.0;
  const double dz = (count_x * count_y > 1) ? (z_max - z_min) / static_cast<double>(count_x * count_y - 1) : 0.0;

  size_t idx = 0;
  for (size_t ix = 0; ix < count_x; ++ix) {
    for (size_t iy = 0; iy < count_y; ++iy) {
      const double x = min_x + static_cast<double>(ix) * dx;
      const double y = min_y + static_cast<double>(iy) * dy;
      const double z = z_min + static_cast<double>(idx++) * dz;
      pts.push_back(PointXYZ{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    }
  }
  return pts;
}

// ============================================================================
// Section 1: Type Traits & Construction
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, TypeTraitsAndFactory) {
  EXPECT_FALSE(std::is_copy_constructible_v<PersistentAdaptiveFramePipeline>);
  EXPECT_FALSE(std::is_copy_assignable_v<PersistentAdaptiveFramePipeline>);
  EXPECT_FALSE(std::is_move_constructible_v<PersistentAdaptiveFramePipeline>);
  EXPECT_FALSE(std::is_move_assignable_v<PersistentAdaptiveFramePipeline>);

  PersistentAdaptivePipelineConfig cfg{};
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
  EXPECT_FALSE(pipeline->fatalErrorLatched());

  // String conversions
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::SUCCESS), "SUCCESS");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING), "SUCCESS_REPLAY_PENDING");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::FRAME_REJECTED_INPUT), "FRAME_REJECTED_INPUT");
  EXPECT_STREQ(structuralActionTypeToString(StructuralActionType::ROOT_CREATE), "ROOT_CREATE");
  EXPECT_STREQ(structuralActionTypeToString(StructuralActionType::SPLIT), "SPLIT");
}

// ============================================================================
// Section 2: First-Frame Unseen Localized Refinement & Anti-Uniform Structure
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, FirstFrameUnseenLocalizedRefineAndAntiUniform) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Sensor origin near origin.
  // Root L3 at (0, 0) covers [0, 0.40m] x [0, 0.40m].
  // Put dense high-complexity points strictly in one sub-quadrant: [0.01, 0.04] x [0.01, 0.04].
  // The rest of the L3 footprint [0.20, 0.40] x [0.20, 0.40] has 0 observations.
  const auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.10, 4, 4);
  const avr::spatial::WorldPoint2D sensor_origin{0.0, 0.0};

  const auto res = pipeline->processFrame(pts, 0, sensor_origin);
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res.frame_fused);
  EXPECT_TRUE(res.structure_changed);
  EXPECT_TRUE(res.cache_committed);
  EXPECT_EQ(res.processing_generation_before, 0U);
  EXPECT_EQ(res.processing_generation_after, 1U);
  EXPECT_EQ(pipeline->processingGeneration(), 1U);

  // Invariants check
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);

  // Topology verification: Anti-Uniform Localized Refinement!
  // It must NOT be a uniform full tree of 64 L0 leaves.
  const auto stats = pipeline->map().statistics().hierarchy_stats;
  const size_t root_count = stats.leaf_count_by_level[avr::spatial::L3] + stats.internal_count_by_level[avr::spatial::L3];
  EXPECT_EQ(root_count, 1U);
  EXPECT_LT(stats.leaf_count, 64U);  // Substantially fewer than a full 64-leaf uniform tree!
  EXPECT_GT(stats.leaf_count, 4U);   // At least one branch refined deeper!

  // Replay cache retains frame 0
  EXPECT_EQ(pipeline->replayCache().frameCount(), 1U);
  EXPECT_TRUE(pipeline->replayCache().containsFrame(0));
}

// ============================================================================
// Section 3: Multiple Evidence-Bearing Children Refine Independently
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, MultipleEvidenceBearingChildrenIndependentRefinement) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Generate points in two distinct quadrants of root L3 [0, 0.4m] x [0, 0.4m]:
  // Quadrant 0: [0.01, 0.08] x [0.01, 0.08]
  // Quadrant 3: [0.25, 0.35] x [0.25, 0.35]
  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.08, 3, 3);
  auto pts2 = makePointsInBox(0.25, 0.35, 0.25, 0.35, 0.0, 0.08, 3, 3);
  pts.insert(pts.end(), pts2.begin(), pts2.end());

  const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res.frame_fused);
  EXPECT_GT(res.splits_applied, 1U);

  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 4: Existing Payload Cascaded Replay
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, ExistingPayloadCascadedReplayComplete) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Frame 1: Coarse distance baseline or flat terrain at L2/L3
  // Sensor at (45.0, 0.0) -> distance to (0,0) is 45m -> distance baseline is L2 (20cm)
  auto pts1 = makePointsInBox(0.02, 0.38, 0.02, 0.38, 0.0, 0.01, 4, 4);
  const auto res1 = pipeline->processFrame(pts1, 100, avr::spatial::WorldPoint2D{45.0, 0.0});
  EXPECT_EQ(res1.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res1.frame_fused);
  EXPECT_EQ(pipeline->processingGeneration(), 1U);

  // Frame 2: Vehicle moves close: sensor at (0.0, 0.0) -> distance 0m -> requests fine L0!
  // Raw cache retains Frame 1 (100).
  // Deep localized refinement triggers: L3/L2 split -> direct replay of frame 100 succeeds!
  auto pts2 = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, 0.05, 4, 4);
  const auto res2 = pipeline->processFrame(pts2, 101, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res2.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res2.frame_fused);
  EXPECT_GT(res2.replay_attempts, 0U);
  EXPECT_EQ(res2.replay_complete, res2.replay_attempts);
  EXPECT_EQ(res2.replay_insufficient, 0U);
  EXPECT_FALSE(res2.replay_pending);

  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 5: Replay-Insufficient Cascade & Manual M9B3C Reversibility
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, ReplayInsufficientCascadeAndSubtreeCollapseRecovery) {
  // Use cache with capacity 2
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Ingest 3 frames far away to populate L3 root and push Frame 10 out of cache
  auto pts = makePointsInBox(0.02, 0.38, 0.02, 0.38, 0.0, 0.01, 4, 4);
  EXPECT_EQ(pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 30, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);

  // Cache now holds {20, 30}. Frame 10 has been evicted!
  EXPECT_FALSE(pipeline->replayCache().containsFrame(10));

  // Frame 40: Moves close to (0,0) -> triggers refinement to L0!
  // Historical replay for earliest observations is missing from cache -> REPLAY_PENDING!
  auto fine_pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, 0.05, 4, 4);
  const auto res40 = pipeline->processFrame(fine_pts, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res40.status, AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING);
  EXPECT_TRUE(res40.frame_fused);
  EXPECT_TRUE(res40.replay_pending);
  EXPECT_GT(res40.replay_insufficient, 0U);

  // Verify map invariants still hold
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);

  // Requirement 69: M9B3C Reversibility Integration Test
  // Manually invoke frozen collapseSubtreeFromCompleteShadow on root L3!
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  auto collapse_res = const_cast<PersistentAdaptiveMap&>(pipeline->map()).collapseSubtreeFromCompleteShadow(root_key);
  EXPECT_EQ(collapse_res.status, SubtreeCollapseStatus::APPLIED_GAP_CLOSED);
  EXPECT_TRUE(collapse_res.isSuccess());

  // After collapse, root is once again an active leaf with complete history!
  EXPECT_TRUE(pipeline->map().hierarchy().isLeaf(root_key));
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 6: HOLD, COARSEN_PENDING, COARSEN_READY Deferred Coarsening
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, HoldAndDeferredCoarseningNoAutoMerge) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Ingest Frame 1 at distance 0m (refines to fine)
  auto pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, 0.02, 4, 4);
  EXPECT_EQ(pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  const size_t leaves_after_f1 = pipeline->map().statistics().hierarchy_stats.leaf_count;

  // Frame 2: Sensor stays at distance 0m -> HOLD_CURRENT
  const auto res2 = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res2.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_FALSE(res2.structure_changed);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, leaves_after_f1);

  // Frame 3-7: Sensor moves far away (e.g. 70m) -> Requests coarser resolution!
  // Hysteresis counts stable coarsen streaks -> reaches COARSEN_PENDING, then COARSEN_READY.
  // M9B3F1 MUST NOT auto-merge!
  for (uint64_t f = 30; f <= 80; f += 10) {
    const auto res = pipeline->processFrame(pts, f, avr::spatial::WorldPoint2D{70.0, 0.0});
    EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
    EXPECT_FALSE(res.structure_changed);
    EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, leaves_after_f1);  // Leaves remain fine!
  }
}

// ============================================================================
// Section 7: Hysteresis Survivor Subset & Cleanup (Req 73)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, HysteresisSurvivorSubsetAndCleanup) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // 4 Roots:
  // Root B: [0.0, 0.4] x [0.0, 0.4]
  // Root D: [0.4, 0.8] x [0.0, 0.4]
  // Root A: [70.0, 70.4] x [0.0, 0.4]
  // Root C: [70.4, 70.8] x [0.0, 0.4]
  auto pts_b = makePointsInBox(0.02, 0.38, 0.02, 0.38, 0.0, 0.01, 2, 2);
  auto pts_d = makePointsInBox(0.42, 0.78, 0.02, 0.38, 0.0, 0.01, 2, 2);
  auto pts_a = makePointsInBox(70.02, 70.38, 0.02, 0.38, 0.0, 0.01, 2, 2);
  auto pts_c = makePointsInBox(70.42, 70.78, 0.02, 0.38, 0.0, 0.01, 2, 2);

  std::vector<PointXYZ> all_pts = pts_b;
  all_pts.insert(all_pts.end(), pts_d.begin(), pts_d.end());
  all_pts.insert(all_pts.end(), pts_a.begin(), pts_a.end());
  all_pts.insert(all_pts.end(), pts_c.begin(), pts_c.end());

  // Frame 1: Sensor at (0.0, 70.0) -> distance to all 4 roots >= 70m (> 60m).
  // All 4 prospective roots created at L3 and NOT refined.
  const auto res1 = pipeline->processFrame(all_pts, 10, avr::spatial::WorldPoint2D{0.0, 70.0});
  EXPECT_EQ(res1.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res1.prospective_roots, 4U);

  // Frame 2: Same points and sensor origin.
  // All 4 roots are now pre-existing leaves, evaluated at distance >= 70m -> HOLD_CURRENT.
  // All 4 are committed into hysteresis!
  const auto res2 = pipeline->processFrame(all_pts, 20, avr::spatial::WorldPoint2D{0.0, 70.0});
  EXPECT_EQ(res2.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res2.hysteresis_committed, 4U);
  EXPECT_EQ(pipeline->hysteresis().stateSnapshot().size(), 4U);

  // Frame 3: Sensor moves to (0.0, 0.0).
  // Roots B & D are close (< 1m, < 10m threshold) -> REFINE_NOW into INTERNAL!
  // Roots A & C are far (~70m, >= 60m threshold) -> HOLD_CURRENT at L3!
  const auto res3 = pipeline->processFrame(all_pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res3.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res3.hysteresis_previews, 4U);      // Pre-existing leaves A, B, C, D all previewed
  EXPECT_EQ(res3.hysteresis_committed, 2U);     // Surviving batch contains A and C only!
  EXPECT_EQ(res3.hysteresis_erased, 2U);        // Old states for B and D erased!

  const auto root_b = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto root_d = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{8, 0}, avr::spatial::L3);
  const auto root_a = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1400, 0}, avr::spatial::L3);
  const auto root_c = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1408, 0}, avr::spatial::L3);

  // B and D were converted to INTERNAL, so their old states were erased
  EXPECT_FALSE(pipeline->hysteresis().getState(root_b).has_value());
  EXPECT_FALSE(pipeline->hysteresis().getState(root_d).has_value());
  // A and C remained active leaves, so their states were committed in the surviving batch
  EXPECT_TRUE(pipeline->hysteresis().getState(root_a).has_value());
  EXPECT_TRUE(pipeline->hysteresis().getState(root_c).has_value());
}

// ============================================================================
// Section 8: Fusion Failure Handling (Conservative Structural Commit vs No-Structure)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, FusionFailureWithAndWithoutStructure) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Case A: Fusion failure with structural change
  PersistentAdaptivePipelineTestAccess::setSimulateFusionFailure(*pipeline, true);

  // First frame contains points that would create root & split
  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
  const auto res_fail = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_fail.status, AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE);
  EXPECT_TRUE(res_fail.structure_changed);
  EXPECT_FALSE(res_fail.frame_fused);
  EXPECT_FALSE(res_fail.cache_committed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);

  // Map topology is conservatively retained
  EXPECT_GT(pipeline->map().statistics().hierarchy_stats.leaf_count, 0U);
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);

  // Raw cache does NOT contain frame 10
  EXPECT_FALSE(pipeline->replayCache().containsFrame(10));

  // Case B: Fusion failure without structural change
  // Second call with same failure flag, but frame points are in already-created topology with no refinement
  const auto res_fail_nostruct = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{70.0, 0.0});
  EXPECT_EQ(res_fail_nostruct.status, AdaptivePipelineStatus::FUSION_FAILED);
  EXPECT_FALSE(res_fail_nostruct.structure_changed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
}

// ============================================================================
// Section 9: Pre-Flight Sequence & Out-of-Order Rejection
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, SequencePreflightRejectionZeroMutation) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
  EXPECT_EQ(pipeline->processFrame(pts, 100, avr::spatial::WorldPoint2D{0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->map().lastFusedFrameSeq(), 100U);
  const auto leaves_snap = pipeline->map().statistics().hierarchy_stats.leaf_count;

  // Duplicate frame_seq = 100
  const auto res_dup = pipeline->processFrame(pts, 100, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_dup.status, AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE);
  EXPECT_FALSE(res_dup.frame_fused);
  EXPECT_FALSE(res_dup.structure_changed);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, leaves_snap);

  // Out-of-order frame_seq = 99
  const auto res_ooo = pipeline->processFrame(pts, 99, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_ooo.status, AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE);
  EXPECT_FALSE(res_ooo.structure_changed);

  // Non-consecutive frames (105, 200, 1000) strictly accepted
  EXPECT_EQ(pipeline->processFrame(pts, 105, avr::spatial::WorldPoint2D{0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 200, avr::spatial::WorldPoint2D{0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 1000, avr::spatial::WorldPoint2D{0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processingGeneration(), 4U);
}

// ============================================================================
// Section 10: Negative Coordinates & Multi-Root Frame
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, NegativeCoordinatesAndMultiRoot) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Points spanning negative and positive roots
  auto pts_neg = makePointsInBox(-0.35, -0.05, -0.35, -0.05, 0.0, 0.05, 4, 4);
  auto pts_pos = makePointsInBox(0.05, 0.35, 0.05, 0.35, 0.0, 0.05, 4, 4);
  pts_neg.insert(pts_neg.end(), pts_pos.begin(), pts_pos.end());

  const auto res = pipeline->processFrame(pts_neg, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_GE(res.prospective_roots, 2U);
  const auto h_stats = pipeline->map().statistics().hierarchy_stats;
  const size_t total_roots = h_stats.leaf_count_by_level[avr::spatial::L3] + h_stats.internal_count_by_level[avr::spatial::L3];
  EXPECT_EQ(total_roots, res.prospective_roots);
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 11: Cache Window Protection in Pipeline
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, CacheWindowProtectionDuringReplay) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 3;
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.02, 0.38, 0.02, 0.38, 0.0, 0.01, 4, 4);
  EXPECT_EQ(pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processFrame(pts, 30, avr::spatial::WorldPoint2D{70.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS);

  EXPECT_EQ(pipeline->replayCache().frameCount(), 3U);
  EXPECT_TRUE(pipeline->replayCache().containsFrame(10));
  EXPECT_TRUE(pipeline->replayCache().containsFrame(20));
  EXPECT_TRUE(pipeline->replayCache().containsFrame(30));

  // Frame 40 causes replay of {10, 20, 30}.
  // During replay, frame 10 must NOT have been evicted yet!
  auto fine_pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, 0.05, 4, 4);
  const auto res40 = pipeline->processFrame(fine_pts, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res40.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res40.cache_committed);

  // Only after Frame 40 commit is frame 10 evicted
  EXPECT_FALSE(pipeline->replayCache().containsFrame(10));
  EXPECT_TRUE(pipeline->replayCache().containsFrame(40));
}

// ============================================================================
// Section 12: Fault Injections (Cache Degraded, Policy Failure, Fatal Latch)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, FaultInjectionsAndFatalLatch) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);

  // 1. Cache Degraded (non-fatal)
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, true);
  const auto res_degraded = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_degraded.status, AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED);
  EXPECT_TRUE(res_degraded.frame_fused);
  EXPECT_FALSE(res_degraded.cache_committed);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, false);

  // 2. Policy Commit Failure (fatal latch)
  PersistentAdaptivePipelineTestAccess::setSimulatePolicyCommitRevisionMismatch(*pipeline, true);
  const auto res_policy_fail = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_policy_fail.status, AdaptivePipelineStatus::POLICY_COMMIT_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());

  // Subsequent call is blocked by fatal latch
  const auto res_latched = pipeline->processFrame(pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res_latched.status, AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED);
  EXPECT_FALSE(res_latched.frame_fused);

  // Fatal latch is terminal for this pipeline instance
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

// ============================================================================
// Section 13: Mixed Raw Point Pipeline Golden & Component Accounting
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, MixedRawPointPipelineGolden) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> mixed_pts{
      PointXYZ{0.01f, 0.01f, 0.01f},                                  // valid A
      PointXYZ{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f},   // NaN
      PointXYZ{0.02f, 0.02f, 0.02f},                                  // valid B
      PointXYZ{std::numeric_limits<float>::infinity(), 0.0f, 0.0f},    // +Inf
      PointXYZ{1e10f, 1e10f, 0.0f},                                   // out-of-range finite
      PointXYZ{0.03f, 0.03f, 0.03f},                                  // valid C
  };

  const auto res = pipeline->processFrame(mixed_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res.input_points, 6U);
  EXPECT_EQ(res.accepted_points, 3U);
  EXPECT_TRUE(res.frame_fused);
  EXPECT_TRUE(res.cache_committed);

  // Evidence scratch accounting
  const auto& ev_stats = pipeline->frameEvidence().statistics();
  EXPECT_EQ(ev_stats.input_points, 6U);
  EXPECT_EQ(ev_stats.accepted_points, 3U);
  EXPECT_EQ(ev_stats.rejected_non_finite, 2U);
  EXPECT_EQ(ev_stats.rejected_out_of_range, 1U);

  // Replay cache raw retention and partitioning
  ASSERT_EQ(pipeline->replayCache().frameCount(), 1U);
  const auto* cached = pipeline->replayCache().findFrame(10);
  ASSERT_NE(cached, nullptr);
  EXPECT_EQ(cached->rawPoints().size(), 6U);
  const auto& cache_stats = cached->statistics();
  EXPECT_EQ(cache_stats.input_points, 6U);
  EXPECT_EQ(cache_stats.non_finite_unindexed, 2U);
  EXPECT_EQ(cache_stats.out_of_range_unindexed, 1U);
  EXPECT_EQ(cache_stats.replay_candidates, 3U);

  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 14: Non-Finite Sensor Origin Rejection
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, SensorOriginNonFiniteRejectionZeroMutation) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);

  // NaN in sensor origin
  const auto res_nan = pipeline->processFrame(
      pts, 10, avr::spatial::WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0});
  EXPECT_EQ(res_nan.status, AdaptivePipelineStatus::FRAME_REJECTED_INPUT);
  EXPECT_FALSE(res_nan.frame_fused);
  EXPECT_FALSE(res_nan.structure_changed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, 0U);
  EXPECT_EQ(pipeline->replayCache().frameCount(), 0U);
  EXPECT_EQ(pipeline->hysteresis().stateRevision(), 0U);

  // +Inf in sensor origin
  const auto res_inf = pipeline->processFrame(
      pts, 10, avr::spatial::WorldPoint2D{0.0, std::numeric_limits<double>::infinity()});
  EXPECT_EQ(res_inf.status, AdaptivePipelineStatus::FRAME_REJECTED_INPUT);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, 0U);
  EXPECT_EQ(pipeline->replayCache().frameCount(), 0U);
}

// ============================================================================
// Section 15: Hysteresis Revision Exhaustion Preflight
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, HysteresisRevisionExhaustionPreflightZeroMutation) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Set hysteresis revision to terminal UINT64_MAX
  ResolutionHysteresisTestAccess::setRevision(
      PersistentAdaptivePipelineTestAccess::getHysteresis(*pipeline), UINT64_MAX);
  EXPECT_EQ(pipeline->hysteresis().stateRevision(), UINT64_MAX);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
  const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::POLICY_REVISION_EXHAUSTED);
  EXPECT_FALSE(res.frame_fused);
  EXPECT_FALSE(res.structure_changed);
  EXPECT_FALSE(res.cache_prepared);
  EXPECT_FALSE(res.cache_committed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, 0U);
  EXPECT_EQ(pipeline->replayCache().frameCount(), 0U);
  EXPECT_EQ(pipeline->hysteresis().stateRevision(), UINT64_MAX);
}

// ============================================================================
// Section 16: Fatal Latch Precedence & Zero Mutation
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, FatalLatchPrecedenceZeroMutation) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  PersistentAdaptivePipelineTestAccess::setFatalLatch(*pipeline, true);
  EXPECT_TRUE(pipeline->fatalErrorLatched());

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
  const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED);
  EXPECT_FALSE(res.frame_fused);
  EXPECT_FALSE(res.structure_changed);
  EXPECT_FALSE(res.cache_prepared);
  EXPECT_FALSE(res.cache_committed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, 0U);
  EXPECT_EQ(pipeline->replayCache().frameCount(), 0U);
}

// ============================================================================
// Section 17: Root Creation Counts as Structural Change on Fusion Failure
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, RootOnlyFusionFailureConservativeCommit) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Far-range point so M4 requests L3 (no split)
  std::vector<PointXYZ> far_pts{PointXYZ{70.0f, 0.0f, 0.0f}};
  PersistentAdaptivePipelineTestAccess::setSimulateFusionFailure(*pipeline, true);

  const auto res = pipeline->processFrame(far_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE);
  EXPECT_TRUE(res.structure_changed);
  EXPECT_EQ(res.prospective_roots, 1U);
  EXPECT_EQ(res.splits_applied, 0U);
  EXPECT_FALSE(res.frame_fused);
  EXPECT_FALSE(res.cache_committed);
  EXPECT_EQ(pipeline->processingGeneration(), 0U);

  // Newly allocated empty L3 root remains under conservative structural commit
  const auto h_stats = pipeline->map().statistics().hierarchy_stats;
  EXPECT_EQ(h_stats.leaf_count_by_level[avr::spatial::L3], 1U);
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 18: Policy Batch std::bad_alloc OOM Path
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, PolicyBatchBadAllocFatalPath) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
  PersistentAdaptivePipelineTestAccess::setSimulatePolicyCommitBadAlloc(*pipeline, true);

  const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::POLICY_COMMIT_FATAL);
  EXPECT_TRUE(res.frame_fused);
  EXPECT_EQ(res.processing_generation_after, 1U);
  EXPECT_EQ(pipeline->processingGeneration(), 1U);
  EXPECT_FALSE(res.cache_committed);
  EXPECT_TRUE(pipeline->fatalErrorLatched());

  // Subsequent frame fails due to fatal latch
  const auto res2 = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res2.status, AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED);
}

// ============================================================================
// Section 19: Cache Commit Logical Failure & Post-Fusion Invariant Failure
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, CacheCommitLogicalFailureAndInvariantFailure) {
  // Test Cache Commit Logical Failure
  {
    auto pipeline = PersistentAdaptiveFramePipeline::create();
    ASSERT_NE(pipeline, nullptr);

    auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
    PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitFailure(*pipeline, true);

    const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
    EXPECT_EQ(res.status, AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR);
    EXPECT_TRUE(res.frame_fused);
    EXPECT_EQ(pipeline->processingGeneration(), 1U);
    EXPECT_FALSE(res.cache_committed);
    EXPECT_TRUE(pipeline->fatalErrorLatched());

    const auto res2 = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
    EXPECT_EQ(res2.status, AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED);
  }

  // Test Post-Processing Invariant Failure
  {
    auto pipeline = PersistentAdaptiveFramePipeline::create();
    ASSERT_NE(pipeline, nullptr);

    auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);
    PersistentAdaptivePipelineTestAccess::setSimulateInvariantFailure(*pipeline, true);

    const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
    EXPECT_EQ(res.status, AdaptivePipelineStatus::INVARIANT_FAILURE);
    EXPECT_TRUE(pipeline->fatalErrorLatched());

    const auto res2 = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
    EXPECT_EQ(res2.status, AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED);
  }
}

// ============================================================================
// Section 20: Cache Degradation Continuation
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, CacheDegradationContinuation) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.05, 4, 4);

  // Frame 10: Cache commit bad_alloc (non-fatal degradation)
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, true);
  const auto res10 = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res10.status, AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED);
  EXPECT_TRUE(res10.frame_fused);
  EXPECT_FALSE(res10.cache_committed);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  EXPECT_EQ(pipeline->processingGeneration(), 1U);

  // Frame 20: Recovery with cache allocation working
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, false);
  const auto res20 = pipeline->processFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res20.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_TRUE(res20.frame_fused);
  EXPECT_TRUE(res20.cache_committed);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  EXPECT_EQ(pipeline->processingGeneration(), 2U);
  EXPECT_TRUE(pipeline->replayCache().containsFrame(20));
}

// ============================================================================
// Section 21: Valid Empty Raw Frame
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, ValidEmptyRawFrame) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> empty_pts{};
  const auto res = pipeline->processFrame(empty_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res.input_points, 0U);
  EXPECT_EQ(res.accepted_points, 0U);
  EXPECT_TRUE(res.frame_fused);
  EXPECT_FALSE(res.structure_changed);
  EXPECT_TRUE(res.cache_committed);
  EXPECT_EQ(res.hysteresis_previews, 0U);
  EXPECT_EQ(res.hysteresis_committed, 0U);
  EXPECT_EQ(pipeline->processingGeneration(), 1U);
  EXPECT_EQ(pipeline->map().lastFusedFrameSeq(), 10U);
  EXPECT_TRUE(pipeline->replayCache().containsFrame(10));
}

// ============================================================================
// Section 22: Failed Fusion Obsolete Leaf Hysteresis Cleanup
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, FailedFusionObsoleteLeafCleanup) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Root B at [0.0, 0.4] x [0.0, 0.4]
  auto pts_b = makePointsInBox(0.02, 0.38, 0.02, 0.38, 0.0, 0.01, 2, 2);

  // Frame 10: Sensor at (0.0, 70.0) -> distance >= 70m -> prospective root B created at L3
  EXPECT_EQ(pipeline->processFrame(pts_b, 10, avr::spatial::WorldPoint2D{0.0, 70.0}).status,
            AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processingGeneration(), 1U);

  const auto root_b = *avr::spatial::CellKey::fromGridCoord(
      avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 20: Pre-existing root B evaluated at distance >= 70m -> HOLD_CURRENT, committed to hysteresis
  EXPECT_EQ(pipeline->processFrame(pts_b, 20, avr::spatial::WorldPoint2D{0.0, 70.0}).status,
            AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(pipeline->processingGeneration(), 2U);
  EXPECT_TRUE(pipeline->hysteresis().containsRegion(root_b));
  const auto rev_tracked = pipeline->hysteresis().stateRevision();
  EXPECT_GE(rev_tracked, 1U);

  // Frame 30: High-density points close to sensor (0.0, 0.0) in root B -> REFINE_NOW and splits root B,
  // but force persistent map fusion failure
  auto fine_pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, 0.05, 4, 4);
  PersistentAdaptivePipelineTestAccess::setSimulateFusionFailure(*pipeline, true);

  const auto res30 = pipeline->processFrame(fine_pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res30.status, AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE);
  EXPECT_TRUE(res30.structure_changed);
  EXPECT_GT(res30.splits_applied, 0U);
  EXPECT_FALSE(res30.frame_fused);
  EXPECT_EQ(pipeline->processingGeneration(), 2U);  // Generation does not advance!

  // Topology cleanup: obsolete leaf hysteresis state erased, advancing controller revision
  EXPECT_FALSE(pipeline->hysteresis().containsRegion(root_b));
  EXPECT_GT(pipeline->hysteresis().stateRevision(), rev_tracked);

  // Section 56: Structural root-only failure has no policy cleanup
  PersistentAdaptivePipelineTestAccess::setSimulateFusionFailure(*pipeline, true);
  const auto rev_before_root_only = pipeline->hysteresis().stateRevision();
  std::vector<PointXYZ> new_root_pts{PointXYZ{70.0f, 0.0f, 0.0f}};
  const auto res40 = pipeline->processFrame(new_root_pts, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res40.status, AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE);
  EXPECT_TRUE(res40.structure_changed);
  EXPECT_EQ(res40.splits_applied, 0U);
  EXPECT_FALSE(res40.frame_fused);
  EXPECT_EQ(pipeline->hysteresis().stateRevision(), rev_before_root_only);  // Hysteresis untouched!
}

// ============================================================================
// Section 23: Child Decision Explainability
// ============================================================================

TEST(PersistentAdaptiveFramePipelineTest, ChildDecisionExplainability) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  // Frame that causes multi-level split (L3 -> L2 -> L1 -> L0)
  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  const auto res = pipeline->processFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::SUCCESS);

  // Verify child decision records
  bool found_child_decision = false;
  for (const auto& dec : res.decision_records) {
    if (!dec.existed_pre_frame && dec.current_level < avr::spatial::L3) {
      found_child_decision = true;
      EXPECT_FALSE(dec.used_hysteresis);
      EXPECT_EQ(dec.hysteresis_action, HysteresisAction::INVALID_OBSERVATION);
      EXPECT_TRUE(dec.had_current_evidence);
    }
  }
  EXPECT_TRUE(found_child_decision);
}

}  // namespace
}  // namespace avr::mapping
