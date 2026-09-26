#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_mapping/point_xyz.hpp"

namespace avr::mapping {

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
  static void setSimulateMetadataBadAlloc(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_metadata_bad_alloc_ = v;
  }
  static void setSimulateAckFailure(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_ack_failure_ = v;
  }
  static void setSimulateCoarsenInternalError(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_coarsen_internal_error_ = v;
  }
  static void setFatalLatch(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.fatal_error_latched_ = v;
  }
  static ResolutionHysteresisController& getHysteresis(PersistentAdaptiveFramePipeline& p) noexcept {
    return p.hysteresis_;
  }
  static PersistentAdaptiveMap& getMap(PersistentAdaptiveFramePipeline& p) noexcept {
    return p.persistent_map_;
  }
  static std::unordered_map<avr::spatial::CellKey, LeafPolicyMetadata>& getLeafPolicyMetadata(
      PersistentAdaptiveFramePipeline& p) noexcept {
    return p.leaf_policy_metadata_;
  }
};

struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
  static std::unordered_map<avr::spatial::CellKey, InternalCoarseningState>& internalCoarseningStates(
      PersistentAdaptiveMap& map) noexcept {
    return map.internal_coarsening_states_;
  }
  static AdaptiveHierarchy& hierarchy(PersistentAdaptiveMap& map) noexcept {
    return map.hierarchy_;
  }
};

namespace {

// Helper to generate dense points in a local bounding box
std::vector<PointXYZ> makePointsInBox(
    double min_x, double max_x,
    double min_y, double max_y,
    double z_min, double z_max,
    size_t count_x = 4, size_t count_y = 4) {
  std::vector<PointXYZ> pts;
  pts.reserve(count_x * count_y);
  const double dx = (count_x > 1) ? (max_x - min_x) / (count_x - 1) : 0.0;
  const double dy = (count_y > 1) ? (max_y - min_y) / (count_y - 1) : 0.0;
  for (size_t ix = 0; ix < count_x; ++ix) {
    for (size_t iy = 0; iy < count_y; ++iy) {
      pts.push_back(PointXYZ{
          .x = min_x + ix * dx,
          .y = min_y + iy * dy,
          .z = (ix % 2 == 0) ? z_min : z_max,
      });
    }
  }
  return pts;
}

// ============================================================================
// Section 1: Freshness Window & Stale Expiration Tests (Req 58, 59, 60)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, W0Expiry) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.coarsen_group_max_age_generations = 0; // W = 0
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Frame 1: distance 0 -> refines to L0
  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.09, 0.0, 0.01, 3, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());

  // Frame 2 & 3: distance 70 -> coarse preference, threshold=2 reached!
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  const auto res3 = pipeline->processFrame(pts, 30, {70.0, 0.0});
  EXPECT_TRUE(res3.isSuccess());

  // Under W=0, after an unobserved empty frame, stale readiness expires immediately
  const auto res4 = pipeline->processFrame(std::span<const PointXYZ>{}, 40, {0.0, 0.0});
  EXPECT_TRUE(res4.isSuccess());
  EXPECT_GT(res4.stale_ready_cancelled, 0U);

  // Verify all ready generations cleared
  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap) {
    EXPECT_EQ(meta.latest_ready_generation, 0U);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, W2StaleExpiry) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.coarsen_group_max_age_generations = 2; // W = 2
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Ingest: frame 10 (refines), frames 20 & 30 (reach ready)
  auto pts_3_siblings = makePointsInBox(0.01, 0.04, 0.01, 0.09, 0.0, 0.01, 3, 4);
  EXPECT_TRUE(pipeline->processFrame(pts_3_siblings, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts_3_siblings, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts_3_siblings, 30, {70.0, 0.0}).isSuccess()); // Generation 3

  // At generation 3: ready generation is 3
  auto snap3 = pipeline->leafPolicyMetadataSnapshot();
  bool found_ready = false;
  for (const auto& [key, meta] : snap3) {
    if (meta.latest_ready_generation == 3) {
      found_ready = true;
    }
  }
  EXPECT_TRUE(found_ready);

  // Generation 4 (age 1 <= 2): unobserved empty frame -> no expiry
  const auto res4 = pipeline->processFrame(std::span<const PointXYZ>{}, 40, {0.0, 0.0});
  EXPECT_TRUE(res4.isSuccess());
  EXPECT_EQ(res4.stale_ready_cancelled, 0U);

  // Generation 5 (age 2 <= 2): unobserved empty frame -> no expiry
  const auto res5 = pipeline->processFrame(std::span<const PointXYZ>{}, 50, {0.0, 0.0});
  EXPECT_TRUE(res5.isSuccess());
  EXPECT_EQ(res5.stale_ready_cancelled, 0U);

  // Generation 6 (age 3 > 2): unobserved empty frame -> expired!
  const auto res6 = pipeline->processFrame(std::span<const PointXYZ>{}, 60, {0.0, 0.0});
  EXPECT_TRUE(res6.isSuccess());
  EXPECT_GT(res6.stale_ready_cancelled, 0U);

  // Verify ready generation is cleared
  const auto snap6 = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap6) {
    EXPECT_EQ(meta.latest_ready_generation, 0U);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, FutureGenerationCorruption) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {0.0, 0.0}).isSuccess());

  // Inject future generation into metadata
  auto& meta_map = PersistentAdaptivePipelineTestAccess::getLeafPolicyMetadata(*pipeline);
  ASSERT_FALSE(meta_map.empty());
  meta_map.begin()->second.latest_ready_generation = pipeline->processingGeneration() + 10;

  // Next frame should detect corruption and latch fatal error
  const auto res = pipeline->processFrame(std::span<const PointXYZ>{}, 30, {0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

// ============================================================================
// Section 2: Direct Merge Evidence & Readiness Tests (Req 61, 62, 63, 64)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, NoEvidenceGolden) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // 1. Refine parent [0, 0.1) x [0, 0.1) at L1 to 4 L0 leaves:
  // Child 0: [0, 0.05) x [0, 0.05)
  // Child 1: [0.05, 0.1) x [0, 0.05)
  // Child 2: [0, 0.05) x [0.05, 0.1)
  // Child 3: [0.05, 0.1) x [0.05, 0.1)
  auto all_4 = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(all_4, 10, {0.0, 0.0}).isSuccess());

  // 2. Feed 2 frames from 70m to all 4 siblings so all 4 become COARSEN_READY
  EXPECT_TRUE(pipeline->processFrame(all_4, 20, {70.0, 0.0}).isSuccess());

  // In frame 30, feed points ONLY to child 0, 1, 2 (leave child 3 unobserved!)
  // Child 3 is in [0.05, 0.10) x [0.05, 0.10). Points in [0.01, 0.04] x [0.01, 0.09] miss child 3.
  auto only_3 = makePointsInBox(0.01, 0.04, 0.01, 0.09, 0.0, 0.01, 3, 4);
  const auto res30 = pipeline->processFrame(only_3, 30, {70.0, 0.0});

  EXPECT_TRUE(res30.isSuccess());
  // CRITICAL MODEL A SAFETY CONTRACT:
  // Child 3 lacks current evidence -> NO merge occurs!
  EXPECT_EQ(res30.direct_merges_applied, 0U);
  EXPECT_EQ(res30.direct_merges_blocked, 0U); // Not attempted -> zero blocked ACK
  EXPECT_FALSE(res30.structure_changed);
}

TEST(PersistentAdaptivePipelineCoarseningTest, AllFourCurrentReadyGolden) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto all_4 = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(all_4, 10, {0.0, 0.0}).isSuccess());

  // Frames 20 & 30: all 4 receive coarse preference -> threshold reached on frame 30!
  EXPECT_TRUE(pipeline->processFrame(all_4, 20, {70.0, 0.0}).isSuccess());
  const auto res30 = pipeline->processFrame(all_4, 30, {70.0, 0.0});

  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.direct_merges_applied, 1U);
  EXPECT_TRUE(res30.structure_changed);

  // Check structural action record
  bool found_action = false;
  for (const auto& sa : res30.structural_action_records) {
    if (sa.action_type == StructuralActionType::DIRECT_MERGE) {
      found_action = true;
      EXPECT_EQ(sa.from_level, avr::spatial::L0);
      EXPECT_EQ(sa.to_level, avr::spatial::L1);
    }
  }
  EXPECT_TRUE(found_action);
}

TEST(PersistentAdaptivePipelineCoarseningTest, RawTargetFartherCoarseGolden) {
  // CRITICAL CORRECTION TEST:
  // Four L0 siblings request L3 (sensor 70m away).
  // M9B1 pending targets L1 (single-level coarsening).
  // Target check: latest_requested_level (L3) >= parent.level (L1).
  // L0 -> L1 merge MUST succeed and must not jump to L2 in the same frame!
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto all_4 = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(all_4, 10, {0.0, 0.0}).isSuccess());

  EXPECT_TRUE(pipeline->processFrame(all_4, 20, {70.0, 0.0}).isSuccess());
  const auto res30 = pipeline->processFrame(all_4, 30, {70.0, 0.0});

  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.direct_merges_applied, 1U);

  // Restored parent is at L1
  const auto parent_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L1);
  EXPECT_TRUE(pipeline->map().hierarchy().isLeaf(parent_key));

  // Restored parent starts fresh: zero metadata, zero hysteresis
  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap) {
    EXPECT_NE(key, parent_key);
  }
  EXPECT_FALSE(pipeline->hysteresis().containsRegion(parent_key));
}

TEST(PersistentAdaptivePipelineCoarseningTest, TenVotePipelineGolden) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 10; // Default 10 threshold

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto all_4 = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(all_4, 10, {0.0, 0.0}).isSuccess());

  // Frames 1 to 9: no merge
  for (uint64_t f = 20; f <= 100; f += 10) {
    const auto res = pipeline->processFrame(all_4, f, {70.0, 0.0});
    EXPECT_TRUE(res.isSuccess());
    EXPECT_EQ(res.direct_merges_applied, 0U);
    EXPECT_FALSE(res.structure_changed);
  }

  // Frame 11 (10th coarse observation): direct merge triggers!
  const auto res110 = pipeline->processFrame(all_4, 110, {70.0, 0.0});
  EXPECT_TRUE(res110.isSuccess());
  EXPECT_EQ(res110.direct_merges_applied, 1U);
  EXPECT_TRUE(res110.structure_changed);
}

// ============================================================================
// Section 3: Hysteresis Action Interactions (Req 65, 66, 67)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, ReadyPendingAckRefresh) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Ingest points covering 1 leaf
  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.01, 2, 2);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());

  // Frame 20 & 30: reach COARSEN_READY at generation 3
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 30, {70.0, 0.0}).isSuccess());

  // Frame 40: compatible coarse observation -> COARSEN_READY_PENDING_ACK at generation 4
  const auto res40 = pipeline->processFrame(pts, 40, {70.0, 0.0});
  EXPECT_TRUE(res40.isSuccess());

  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap) {
    EXPECT_EQ(meta.latest_ready_generation, 4U);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, HoldReset) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.01, 2, 2);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 30, {70.0, 0.0}).isSuccess());

  // Frame 40: Sensor moves to distance 0m -> triggers HOLD_CURRENT!
  const auto res40 = pipeline->processFrame(pts, 40, {0.0, 0.0});
  EXPECT_TRUE(res40.isSuccess());

  // Hysteresis pending cleared, latest_ready_generation = 0
  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap) {
    EXPECT_EQ(meta.latest_ready_generation, 0U);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, RefineVeto) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Start with L1 root refined to 4 L0 leaves
  auto all_4 = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(all_4, 10, {0.0, 0.0}).isSuccess());

  // Feed 2 coarse frames to build readiness
  EXPECT_TRUE(pipeline->processFrame(all_4, 20, {70.0, 0.0}).isSuccess());

  // In frame 30: sensor at 0.0m on child 0 -> child 0 evaluates REFINE/HOLD, vetoes coarsening!
  const auto res30 = pipeline->processFrame(all_4, 30, {0.0, 0.0});
  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.direct_merges_applied, 0U);
}

// ============================================================================
// Section 4: Direct Merge Correctness & Fault Injection (Req 68, 69, 70, 71)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, DirectMergeExactness) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipelineA = PersistentAdaptiveFramePipeline::create(cfg);
  auto pipelineB = PersistentAdaptiveFramePipeline::create(cfg);

  // Pipeline A: refines to L0, then coarsens back to L1
  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipelineA->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipelineA->processFrame(pts, 20, {20.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipelineA->processFrame(pts, 30, {20.0, 0.0}).isSuccess());

  // Pipeline B: stays at L1 (sensor at distance 20m targets L1)
  EXPECT_TRUE(pipelineB->processFrame(pts, 10, {20.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipelineB->processFrame(pts, 20, {20.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipelineB->processFrame(pts, 30, {20.0, 0.0}).isSuccess());

  const auto parent_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L1);
  const auto* pA = pipelineA->map().findActivePayload(parent_key);
  const auto* pB = pipelineB->map().findActivePayload(parent_key);

  ASSERT_NE(pA, nullptr);
  ASSERT_NE(pB, nullptr);
  EXPECT_EQ(pA->observationCount(), pB->observationCount());
  EXPECT_EQ(pA->firstSeenFrame(), pB->firstSeenFrame());
  EXPECT_EQ(pA->lastSeenFrame(), pB->lastSeenFrame());
  EXPECT_EQ(pA->observedFrameCount(), pB->observedFrameCount());
  EXPECT_EQ(pA->elevation().count(), pB->elevation().count());
  EXPECT_DOUBLE_EQ(pA->elevation().mean(), pB->elevation().mean());
  EXPECT_DOUBLE_EQ(pA->elevation().m2(), pB->elevation().m2());
  EXPECT_DOUBLE_EQ(pA->elevation().min(), pB->elevation().min());
  EXPECT_DOUBLE_EQ(pA->elevation().max(), pB->elevation().max());
}

TEST(PersistentAdaptivePipelineCoarseningTest, DirectMergeCleanup) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  const auto res30 = pipeline->processFrame(pts, 30, {70.0, 0.0});

  EXPECT_EQ(res30.direct_merges_applied, 1U);

  // 4 children absent from hierarchy
  const auto parent_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L1);
  auto children = *avr::spatial::childrenOf(parent_key);
  for (const auto& c : children) {
    EXPECT_FALSE(pipeline->map().hierarchy().isAllocated(c));
    EXPECT_FALSE(pipeline->hysteresis().containsRegion(c));
  }

  // Parent is LEAF, has no hysteresis or metadata
  EXPECT_TRUE(pipeline->map().hierarchy().isLeaf(parent_key));
  EXPECT_FALSE(pipeline->hysteresis().containsRegion(parent_key));

  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

TEST(PersistentAdaptivePipelineCoarseningTest, DirectSafeBlock) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());

  // Manually mark one child as requiring replay in payload store to trigger safe block REPLAY_INCOMPLETE
  const auto parent_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L1);
  auto children = *avr::spatial::childrenOf(parent_key);
  PersistentAdaptiveMapTestAccess::payloadStore(
      PersistentAdaptivePipelineTestAccess::getMap(*pipeline)).setReplayRequired(children[0], true);

  // Frame 30: candidate evaluation runs, but child[0] requires replay -> prefiltered or blocked!
  const auto res30 = pipeline->processFrame(pts, 30, {70.0, 0.0});
  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.direct_merges_applied, 0U);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
}

TEST(PersistentAdaptivePipelineCoarseningTest, DirectCorruptionFailure) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());

  // Inject internal error on direct merge
  PersistentAdaptivePipelineTestAccess::setSimulateCoarsenInternalError(*pipeline, true);

  const auto res30 = pipeline->processFrame(pts, 30, {70.0, 0.0});
  EXPECT_EQ(res30.status, AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

// ============================================================================
// Section 5: Complete-Shadow Subtree Recovery Tests (Req 72-81)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, LongLivedGapRecovery) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Ingest frame 1 far away -> root allocated
  auto root_pts = makePointsInBox(0.01, 0.39, 0.01, 0.39, 0.0, 0.01, 16, 16);
  EXPECT_TRUE(pipeline->processFrame(root_pts, 10, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 30, {70.0, 0.0}).isSuccess());

  // Frame 40: Close distance -> refines to L0, creates REPLAY_PENDING!
  auto fine_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  const auto res40 = pipeline->processFrame(fine_pts, 40, {0.0, 0.0});
  EXPECT_EQ(res40.status, AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING);

  // Ingest 50 frames to evict all old history completely
  for (uint64_t f = 50; f <= 150; f += 10) {
    EXPECT_TRUE(pipeline->processFrame(fine_pts, f, {0.0, 0.0}).isSuccess());
  }

  // Now move sensor far away (70m) and feed all frontier leaves coarse preference
  EXPECT_TRUE(pipeline->processFrame(root_pts, 160, {70.0, 0.0}).isSuccess());
  const auto res170 = pipeline->processFrame(root_pts, 170, {70.0, 0.0});

  EXPECT_TRUE(res170.isSuccess());
  EXPECT_EQ(res170.recovery_applied, 1U);
  EXPECT_TRUE(res170.structure_changed);

  // Root L3 restored as LEAF
  const auto root_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L3);
  EXPECT_TRUE(pipeline->map().hierarchy().isLeaf(root_key));
  EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
}

TEST(PersistentAdaptivePipelineCoarseningTest, RecoveryRequiresRealGap) {
  // Deep topology that is fully replay-complete must NOT trigger recovery collapse!
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 100; // Large cache -> replay complete!
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  const auto res30 = pipeline->processFrame(pts, 30, {70.0, 0.0});

  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.recovery_applied, 0U); // No recovery collapse
  EXPECT_EQ(res30.direct_merges_applied, 1U); // Ordinary direct merge instead!
}

TEST(PersistentAdaptivePipelineCoarseningTest, TargetIncompatibleRecovery) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto root_pts = makePointsInBox(0.01, 0.39, 0.01, 0.39, 0.0, 0.01, 16, 16);
  EXPECT_TRUE(pipeline->processFrame(root_pts, 10, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 30, {70.0, 0.0}).isSuccess());

  auto fine_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  EXPECT_EQ(pipeline->processFrame(fine_pts, 40, {0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING);

  // If frontier leaves only request L1 (e.g. sensor at distance 15m), recovery to L3 is forbidden!
  EXPECT_TRUE(pipeline->processFrame(root_pts, 50, {15.0, 0.0}).isSuccess());
  const auto res60 = pipeline->processFrame(root_pts, 60, {15.0, 0.0});

  EXPECT_TRUE(res60.isSuccess());
  EXPECT_EQ(res60.recovery_applied, 0U);
}

TEST(PersistentAdaptivePipelineCoarseningTest, RecoveryCleanup) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto root_pts = makePointsInBox(0.01, 0.39, 0.01, 0.39, 0.0, 0.01, 16, 16);
  EXPECT_TRUE(pipeline->processFrame(root_pts, 10, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 30, {70.0, 0.0}).isSuccess());

  auto fine_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  EXPECT_EQ(pipeline->processFrame(fine_pts, 40, {0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING);

  EXPECT_TRUE(pipeline->processFrame(root_pts, 50, {70.0, 0.0}).isSuccess());
  const auto res60 = pipeline->processFrame(root_pts, 60, {70.0, 0.0});

  EXPECT_TRUE(res60.isSuccess());
  EXPECT_EQ(res60.recovery_applied, 1U);

  // Restored ancestor has NO hysteresis and NO metadata
  const auto root_key = *avr::spatial::CellKey::fromGridCoord({0, 0}, avr::spatial::L3);
  EXPECT_FALSE(pipeline->hysteresis().containsRegion(root_key));
  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  for (const auto& [key, meta] : snap) {
    EXPECT_NE(key, root_key);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, RecoveryFatalFailure) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto root_pts = makePointsInBox(0.01, 0.39, 0.01, 0.39, 0.0, 0.01, 16, 16);
  EXPECT_TRUE(pipeline->processFrame(root_pts, 10, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(root_pts, 30, {70.0, 0.0}).isSuccess());

  auto fine_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  EXPECT_EQ(pipeline->processFrame(fine_pts, 40, {0.0, 0.0}).status, AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING);

  EXPECT_TRUE(pipeline->processFrame(root_pts, 50, {70.0, 0.0}).isSuccess());

  // Inject failure during collapse
  PersistentAdaptivePipelineTestAccess::setSimulateCoarsenInternalError(*pipeline, true);

  const auto res60 = pipeline->processFrame(root_pts, 60, {70.0, 0.0});
  EXPECT_EQ(res60.status, AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

// ============================================================================
// Section 6: Candidate Conflict & Disjoint Execution (Req 82, 83)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, DisjointOperations) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  // Two spatially disjoint sibling groups:
  // Group A in [0.01, 0.09] x [0.01, 0.09]
  // Group B in [0.41, 0.49] x [0.01, 0.09]
  auto pts_a = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  auto pts_b = makePointsInBox(0.41, 0.49, 0.01, 0.09, 0.0, 0.01, 4, 4);
  std::vector<PointXYZ> both = pts_a;
  both.insert(both.end(), pts_b.begin(), pts_b.end());

  EXPECT_TRUE(pipeline->processFrame(both, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(both, 20, {70.0, 0.0}).isSuccess());
  const auto res30 = pipeline->processFrame(both, 30, {70.0, 0.0});

  EXPECT_TRUE(res30.isSuccess());
  EXPECT_EQ(res30.direct_merges_applied, 2U); // Both execute in same frame!
}

// ============================================================================
// Section 7: Cache Degradation & Fault Boundaries (Req 84, 85, 86, 87)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, CacheDegradedMetadataGolden) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());

  // Frame 30: simulate cache commit bad_alloc
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, true);

  const auto res30 = pipeline->processFrame(pts, 30, {70.0, 0.0});
  EXPECT_EQ(res30.status, AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED);
  EXPECT_TRUE(res30.frame_fused);
  EXPECT_FALSE(res30.cache_committed);

  // CRITICAL M9B3F2B CONTRACT:
  // Metadata MUST reflect frame 30!
  const auto snap = pipeline->leafPolicyMetadataSnapshot();
  ASSERT_FALSE(snap.empty());
  for (const auto& [key, meta] : snap) {
    EXPECT_EQ(meta.latest_decision_generation, 3U);
  }

  // But coarsening was SKIPPED during degraded frame!
  EXPECT_EQ(res30.direct_merges_applied, 0U);
  EXPECT_EQ(res30.recovery_applied, 0U);

  // Next frame (cache restored) resumes normal coarsening!
  PersistentAdaptivePipelineTestAccess::setSimulateCacheCommitBadAlloc(*pipeline, false);
  const auto res40 = pipeline->processFrame(pts, 40, {70.0, 0.0});
  EXPECT_EQ(res40.status, AdaptivePipelineStatus::SUCCESS);
  EXPECT_EQ(res40.direct_merges_applied, 1U);
}

TEST(PersistentAdaptivePipelineCoarseningTest, EmptyFrameStaleExpiry) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.coarsen_group_max_age_generations = 1;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.01, 2, 2);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 30, {70.0, 0.0}).isSuccess()); // Generation 3, READY

  // Empty frame at 40 (Gen 4, age 1 <= 1) -> no expiry
  EXPECT_TRUE(pipeline->processFrame(std::span<const PointXYZ>{}, 40, {0.0, 0.0}).isSuccess());

  // Empty frame at 50 (Gen 5, age 2 > 1) -> expiry!
  const auto res50 = pipeline->processFrame(std::span<const PointXYZ>{}, 50, {0.0, 0.0});
  EXPECT_TRUE(res50.isSuccess());
  EXPECT_GT(res50.stale_ready_cancelled, 0U);
}

TEST(PersistentAdaptivePipelineCoarseningTest, MetadataOomTest) {
  auto pipeline = PersistentAdaptiveFramePipeline::create();
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());

  // Simulate bad_alloc during metadata insertion
  PersistentAdaptivePipelineTestAccess::setSimulateMetadataBadAlloc(*pipeline, true);

  const auto res = pipeline->processFrame(pts, 20, {70.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

TEST(PersistentAdaptivePipelineCoarseningTest, AckFailureTest) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());

  // Simulate ACK failure on merge
  PersistentAdaptivePipelineTestAccess::setSimulateAckFailure(*pipeline, true);

  const auto res = pipeline->processFrame(pts, 30, {70.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR);
  EXPECT_TRUE(pipeline->fatalErrorLatched());
}

// ============================================================================
// Section 8: Counters, Determinism & Adaptation Stress (Req 88, 89, 90, 91, 92)
// ============================================================================

TEST(PersistentAdaptivePipelineCoarseningTest, CounterConservation) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());
  EXPECT_TRUE(pipeline->processFrame(pts, 20, {70.0, 0.0}).isSuccess());
  const auto res = pipeline->processFrame(pts, 30, {70.0, 0.0});

  EXPECT_EQ(res.direct_merge_attempts, res.direct_merges_applied + res.direct_merges_blocked);
  size_t dm_records = 0;
  for (const auto& sa : res.structural_action_records) {
    if (sa.action_type == StructuralActionType::DIRECT_MERGE) {
      ++dm_records;
    }
  }
  EXPECT_EQ(dm_records, res.direct_merges_applied);
}

TEST(PersistentAdaptivePipelineCoarseningTest, Determinism) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pA = PersistentAdaptiveFramePipeline::create(cfg);
  auto pB = PersistentAdaptiveFramePipeline::create(cfg);

  for (uint64_t f = 1; f <= 50; ++f) {
    const double dist = (f % 5 == 0) ? 0.0 : 70.0;
    auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);

    const auto resA = pA->processFrame(pts, f * 10, {dist, 0.0});
    const auto resB = pB->processFrame(pts, f * 10, {dist, 0.0});

    EXPECT_EQ(resA.status, resB.status);
    EXPECT_EQ(resA.direct_merges_applied, resB.direct_merges_applied);
    EXPECT_EQ(resA.recovery_applied, resB.recovery_applied);
    EXPECT_EQ(resA.decision_records, resB.decision_records);
    EXPECT_EQ(resA.structural_action_records, resB.structural_action_records);
  }

  EXPECT_EQ(pA->leafPolicyMetadataSnapshot(), pB->leafPolicyMetadataSnapshot());
  EXPECT_EQ(pA->map().hierarchy().topologySnapshot(), pB->map().hierarchy().topologySnapshot());
}

TEST(PersistentAdaptivePipelineCoarseningTest, OrdinaryAdaptationCycleStress) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.01, 0.09, 0.01, 0.09, 0.0, 0.01, 4, 4);
  uint64_t seq = 100;

  // Run 500 adaptation cycles (Refine to L0 -> Coarsen to L1 -> Re-refine)
  for (int cycle = 0; cycle < 500; ++cycle) {
    // 1. Refine (close sensor targets L0)
    seq += 10;
    const auto r_ref = pipeline->processFrame(pts, seq, {0.0, 0.0});
    ASSERT_TRUE(r_ref.isSuccess());

    // 2. Coarsen (sensor at 20m targets L1, 2 frames)
    seq += 10;
    ASSERT_TRUE(pipeline->processFrame(pts, seq, {20.0, 0.0}).isSuccess());
    seq += 10;
    const auto r_coarsen = pipeline->processFrame(pts, seq, {20.0, 0.0});
    ASSERT_TRUE(r_coarsen.isSuccess());
    ASSERT_TRUE(r_coarsen.direct_merges_applied > 0 || r_coarsen.recovery_applied > 0);
    ASSERT_TRUE(r_coarsen.structure_changed);

    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }
}

TEST(PersistentAdaptivePipelineCoarseningTest, GapRecoveryCycleStress) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 2;

  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  auto root_pts = makePointsInBox(0.01, 0.39, 0.01, 0.39, 0.0, 0.01, 16, 16);
  auto fine_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  uint64_t seq = 10;

  // Run 100 gap recovery cycles
  for (int cycle = 0; cycle < 100; ++cycle) {
    // 1. Ingest 3 frames far away to populate L3 root and push earliest out of cache
    seq += 10;
    ASSERT_TRUE(pipeline->processFrame(root_pts, seq, {70.0, 0.0}).isSuccess());
    seq += 10;
    ASSERT_TRUE(pipeline->processFrame(root_pts, seq, {70.0, 0.0}).isSuccess());
    seq += 10;
    ASSERT_TRUE(pipeline->processFrame(root_pts, seq, {70.0, 0.0}).isSuccess());

    // 2. Refine close -> creates replay gap
    seq += 10;
    const auto r_ref = pipeline->processFrame(fine_pts, seq, {0.0, 0.0});
    ASSERT_TRUE(r_ref.isSuccess());

    // 3. Sustained coarse preference -> complete shadow recovery collapse!
    seq += 10;
    ASSERT_TRUE(pipeline->processFrame(root_pts, seq, {70.0, 0.0}).isSuccess());
    seq += 10;
    const auto r_rec = pipeline->processFrame(root_pts, seq, {70.0, 0.0});
    ASSERT_TRUE(r_rec.isSuccess());
    ASSERT_GT(r_rec.recovery_applied, 0U);

    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }
}

}  // namespace
}  // namespace avr::mapping
