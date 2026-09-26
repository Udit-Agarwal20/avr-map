#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/internal_coarsening_state.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_merge_result.hpp"
#include "avr_mapping/point_routing.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_reconstruction_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

void verifyExact8FieldEquivalence(
    const AdaptiveCellPayload& actual,
    const AdaptiveCellPayload& expected) {
  EXPECT_EQ(actual.observationCount(), expected.observationCount());
  EXPECT_EQ(actual.firstSeenFrame(), expected.firstSeenFrame());
  EXPECT_EQ(actual.lastSeenFrame(), expected.lastSeenFrame());
  EXPECT_EQ(actual.observedFrameCount(), expected.observedFrameCount());
  EXPECT_EQ(actual.elevation().count(), expected.elevation().count());
  EXPECT_DOUBLE_EQ(actual.elevation().mean(), expected.elevation().mean());
  EXPECT_DOUBLE_EQ(actual.elevation().m2(), expected.elevation().m2());
  EXPECT_DOUBLE_EQ(actual.elevation().min(), expected.elevation().min());
  EXPECT_DOUBLE_EQ(actual.elevation().max(), expected.elevation().max());
}

// ============================================================================
// Prompt 38 & 39: Critical Replay Golden Setup & Execution
// ============================================================================

TEST(PersistentReplayRoutingTest, CriticalReplayGoldenStaticFusionOnly) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Base root L3 covers [0, 0.40) x [0, 0.40).
  // Children at L2:
  // c0: [0.0, 0.20) x [0.0, 0.20)
  // c1: [0.20, 0.40) x [0.0, 0.20)
  // c2: [0.0, 0.20) x [0.20, 0.40)
  // c3: [0.20, 0.40) x [0.20, 0.40)
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // Raw historical frame containing static points S and dynamic points D:
  // S0 in c0, D0 in c1, S1 in c0, D1 in c3
  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},   // S0: static in child 0
      {0.25, 0.05, 10.0},  // D0: dynamic in child 1
      {0.08, 0.08, 1.5},   // S1: static in child 0
      {0.25, 0.25, 20.0},  // D1: dynamic in child 3
  };

  const std::vector<PointRouting> routing = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  // 1. Static subset S fused into map
  const std::vector<PointXYZ> static_subset = {
      raw_points[0],
      raw_points[2],
  };
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(static_subset, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Parent payload contains only S (2 points, mean 1.25, min 1.0, max 1.5)
  const auto* parent_before = map.findActivePayload(root_key);
  ASSERT_NE(parent_before, nullptr);
  EXPECT_EQ(parent_before->observationCount(), 2ULL);
  EXPECT_DOUBLE_EQ(parent_before->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(parent_before->elevation().max(), 1.5);
  EXPECT_DOUBLE_EQ(parent_before->elevation().mean(), 1.25);

  // 2. Full raw frame cached with routing mask
  ASSERT_TRUE(cache.pushFrame(raw_points, routing, 10, origin).has_value());

  // 3. Split parent
  ASSERT_TRUE(map.splitLeaf(root_key));

  // 4. Replay direct children using routed cache
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(result.parent_replayed_observations, 2ULL);
  EXPECT_EQ(result.historical_observations_distributed, 2ULL);

  // 5. Verification:
  // Child 0 contains S0 and S1
  const auto* child0 = map.findActivePayload(children[0]);
  ASSERT_NE(child0, nullptr);
  EXPECT_EQ(child0->observationCount(), 2ULL);
  EXPECT_DOUBLE_EQ(child0->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(child0->elevation().max(), 1.5);
  EXPECT_DOUBLE_EQ(child0->elevation().mean(), 1.25);

  // Children 1, 2, 3 have NO active payload (D0 in c1 and D1 in c3 were excluded)
  EXPECT_EQ(map.findActivePayload(children[1]), nullptr);
  EXPECT_EQ(map.findActivePayload(children[2]), nullptr);
  EXPECT_EQ(map.findActivePayload(children[3]), nullptr);

  // Replay flags are cleared for all children
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Prompt 36: Parent Reconstruction Mismatch Proof Without Routing
// ============================================================================

TEST(PersistentReplayRoutingTest, ParentReconstructionMismatchProofWithoutRouting) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Raw frame with S0, D0, S1, D1
  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},   // S0: static
      {0.25, 0.05, 10.0},  // D0: dynamic
      {0.08, 0.08, 1.5},   // S1: static
      {0.25, 0.25, 20.0},  // D1: dynamic
  };

  // Map originally fused with static points only (2 points)
  const std::vector<PointXYZ> static_subset = {raw_points[0], raw_points[2]};
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(static_subset, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Cache stores all 4 points with STATIC_FUSABLE (simulating unrouted/legacy behavior where dynamic points leak)
  const std::vector<PointRouting> unrouted = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
  };
  ASSERT_TRUE(cache.pushFrame(raw_points, unrouted, 10, origin).has_value());

  // Split parent
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Replay fails verification because replayed candidate parent (4 points) does not match retired parent (2 points)
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH);
}

// ============================================================================
// Prompt 40: Dynamic Child Empty Golden
// ============================================================================

TEST(PersistentReplayRoutingTest, DynamicChildEmptyGolden) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // S in child 0, D in child 2 (where no static points exist)
  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 2.0},  // child 0 (static)
      {0.05, 0.25, 9.9},  // child 2 (dynamic)
  };
  const std::vector<PointRouting> routing = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(std::vector<PointXYZ>{raw_points[0]}, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(cache.pushFrame(raw_points, routing, 10, origin).has_value());
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  // Child 2 receives zero payload, no ghost obstacle
  EXPECT_EQ(map.findActivePayload(children[2]), nullptr);
  EXPECT_FALSE(map.requiresReplay(children[2]));
  EXPECT_NE(map.findActivePayload(children[0]), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Prompt 41: Mixed Same-Child Golden
// ============================================================================

TEST(PersistentReplayRoutingTest, MixedSameChildGolden) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // Both S and D in child 0
  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},   // S
      {0.08, 0.08, 100.0}, // D (dynamic obstacle)
  };
  const std::vector<PointRouting> routing = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(std::vector<PointXYZ>{raw_points[0]}, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  ASSERT_TRUE(cache.pushFrame(raw_points, routing, 10, origin).has_value());
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  const auto* child0 = map.findActivePayload(children[0]);
  ASSERT_NE(child0, nullptr);
  // Refers to S only, D contributed zero elevation statistics
  EXPECT_EQ(child0->observationCount(), 1ULL);
  EXPECT_DOUBLE_EQ(child0->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(child0->elevation().max(), 1.0);
  EXPECT_DOUBLE_EQ(child0->elevation().mean(), 1.0);
}

// ============================================================================
// Prompt 42, 44, 45: All-Dynamic Historical Frame & Temporal Invariants
// ============================================================================

TEST(PersistentReplayRoutingTest, AllDynamicHistoricalFrameTemporalInvariants) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // Frame 10: Static point S in child 0
  const std::vector<PointXYZ> f10_pts = {{0.05, 0.05, 2.0}};
  const std::vector<PointRouting> f10_routing = {PointRouting::STATIC_FUSABLE};
  ASSERT_TRUE(cache.pushFrame(f10_pts, f10_routing, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10_pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Frame 20: Entire frame in this cell is dynamic excluded D
  const std::vector<PointXYZ> f20_pts = {{0.06, 0.06, 50.0}, {0.25, 0.25, 80.0}};
  const std::vector<PointRouting> f20_routing = {
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
  };
  ASSERT_TRUE(cache.pushFrame(f20_pts, f20_routing, 20, origin).has_value());
  // Frame 20 produces NO static evidence, so nothing is fused into map for frame 20

  const auto* parent_before = map.findActivePayload(root_key);
  ASSERT_NE(parent_before, nullptr);
  EXPECT_EQ(parent_before->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_before->lastSeenFrame(), 10ULL);
  EXPECT_EQ(parent_before->observedFrameCount(), 1ULL);

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  const auto* child0 = map.findActivePayload(children[0]);
  ASSERT_NE(child0, nullptr);
  // Temporal metadata preserved: frame 20 did not advance lastSeen or observedFrameCount
  EXPECT_EQ(child0->firstSeenFrame(), 10ULL);
  EXPECT_EQ(child0->lastSeenFrame(), 10ULL);
  EXPECT_EQ(child0->observedFrameCount(), 1ULL);
  EXPECT_EQ(child0->observationCount(), 1ULL);

  EXPECT_EQ(map.findActivePayload(children[3]), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Prompt 43: Multi-Frame Routed Replay
// ============================================================================

TEST(PersistentReplayRoutingTest, MultiFrameRoutedReplay) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // F10: static in c0 + dynamic in c1
  const std::vector<PointXYZ> f10_pts = {{0.05, 0.05, 1.0}, {0.25, 0.05, 99.0}};
  const std::vector<PointRouting> f10_r = {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED};
  ASSERT_TRUE(cache.pushFrame(f10_pts, f10_r, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(std::vector<PointXYZ>{f10_pts[0]}, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // F20: static only in c0
  const std::vector<PointXYZ> f20_pts = {{0.06, 0.06, 1.5}};
  const std::vector<PointRouting> f20_r = {PointRouting::STATIC_FUSABLE};
  ASSERT_TRUE(cache.pushFrame(f20_pts, f20_r, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20_pts, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // F30: dynamic in c0 + static in c1
  const std::vector<PointXYZ> f30_pts = {{0.07, 0.07, 77.0}, {0.26, 0.06, 2.0}};
  const std::vector<PointRouting> f30_r = {PointRouting::DYNAMIC_EXCLUDED, PointRouting::STATIC_FUSABLE};
  ASSERT_TRUE(cache.pushFrame(f30_pts, f30_r, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(std::vector<PointXYZ>{f30_pts[1]}, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Pre-split parent state: 3 static observations across frames 10, 20, 30
  const auto* parent_before = map.findActivePayload(root_key);
  ASSERT_NE(parent_before, nullptr);
  EXPECT_EQ(parent_before->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_before->lastSeenFrame(), 30ULL);
  EXPECT_EQ(parent_before->observedFrameCount(), 3ULL);
  EXPECT_EQ(parent_before->observationCount(), 3ULL);

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(result.parent_replayed_observations, 3ULL);
  EXPECT_EQ(result.historical_observations_distributed, 3ULL);

  // Child 0: static from F10 and F20
  const auto* c0 = map.findActivePayload(children[0]);
  ASSERT_NE(c0, nullptr);
  EXPECT_EQ(c0->observationCount(), 2ULL);
  EXPECT_EQ(c0->firstSeenFrame(), 10ULL);
  EXPECT_EQ(c0->lastSeenFrame(), 20ULL);
  EXPECT_EQ(c0->observedFrameCount(), 2ULL);

  // Child 1: static from F30
  const auto* c1 = map.findActivePayload(children[1]);
  ASSERT_NE(c1, nullptr);
  EXPECT_EQ(c1->observationCount(), 1ULL);
  EXPECT_EQ(c1->firstSeenFrame(), 30ULL);
  EXPECT_EQ(c1->lastSeenFrame(), 30ULL);
  EXPECT_EQ(c1->observedFrameCount(), 1ULL);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Prompt 46: Internal Shadow Compatibility
// ============================================================================

TEST(PersistentReplayRoutingTest, InternalCoarseningShadowCompatibility) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // F10: static + dynamic
  const std::vector<PointXYZ> f10_pts = {{0.05, 0.05, 1.0}, {0.25, 0.05, 50.0}};
  const std::vector<PointRouting> f10_r = {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED};
  ASSERT_TRUE(cache.pushFrame(f10_pts, f10_r, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(std::vector<PointXYZ>{f10_pts[0]}, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Shadow check
  const auto* shadow_state = map.findInternalCoarseningState(root_key);
  ASSERT_NE(shadow_state, nullptr);
  EXPECT_EQ(shadow_state->base_state, CoarseningBaseState::COMPLETE_RETIRED_CACHE);

  // F20: new static point in child 0
  const std::vector<PointXYZ> f20_pts = {{0.08, 0.08, 2.0}};
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20_pts, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  const auto* shadow_updated = map.findInternalCoarseningState(root_key);
  ASSERT_NE(shadow_updated, nullptr);
  EXPECT_TRUE(shadow_updated->has_current_shadow);
  EXPECT_EQ(shadow_updated->current_shadow.observationCount(), 2ULL);
  EXPECT_EQ(shadow_updated->current_shadow.firstSeenFrame(), 10ULL);
  EXPECT_EQ(shadow_updated->current_shadow.lastSeenFrame(), 20ULL);
  EXPECT_EQ(shadow_updated->current_shadow.observedFrameCount(), 2ULL);
  EXPECT_DOUBLE_EQ(shadow_updated->current_shadow.elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(shadow_updated->current_shadow.elevation().max(), 2.0);
  EXPECT_DOUBLE_EQ(shadow_updated->current_shadow.elevation().mean(), 1.5);
}

// ============================================================================
// Prompt 47 & 48: Merge After Routed Replay & Re-Split After Merge
// ============================================================================

TEST(PersistentReplayRoutingTest, MergeAfterRoutedReplayAndResplit) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto children = *avr::spatial::childrenOf(root_key);

  // F10: S in c0, D in c1
  const std::vector<PointXYZ> f10_pts = {{0.05, 0.05, 1.0}, {0.25, 0.05, 50.0}};
  const std::vector<PointRouting> f10_r = {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED};
  ASSERT_TRUE(cache.pushFrame(f10_pts, f10_r, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(std::vector<PointXYZ>{f10_pts[0]}, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto orig_parent = *map.findActivePayload(root_key);

  // 1. Split & replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // 2. Merge direct children
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());

  // Merged parent matches exact original static authoritative payload
  const auto* merged_parent = map.findActivePayload(root_key);
  ASSERT_NE(merged_parent, nullptr);
  verifyExact8FieldEquivalence(*merged_parent, orig_parent);

  // 3. Re-split after merge
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto resplit_res = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(resplit_res.status, ReplayStatus::APPLIED_COMPLETE);

  // Child 0 has exact original S payload again
  const auto* c0 = map.findActivePayload(children[0]);
  ASSERT_NE(c0, nullptr);
  EXPECT_EQ(c0->observationCount(), 1ULL);
  EXPECT_DOUBLE_EQ(c0->elevation().mean(), 1.0);
  EXPECT_EQ(map.findActivePayload(children[1]), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Prompt 49: Cache Eviction Limitation
// ============================================================================

TEST(PersistentReplayRoutingTest, CacheEvictionLimitation) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 2});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // F10, F20, F30
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  const std::vector<PointRouting> r = {PointRouting::STATIC_FUSABLE};

  ASSERT_TRUE(cache.pushFrame(pts, r, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(cache.pushFrame(pts, r, 20, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, r, 30, origin).has_value());
  // Frame 10 is now evicted from cache (capacity 2: 20, 30 remain)

  ASSERT_TRUE(map.splitLeaf(root_key));
  // Replay needs frame 10 (since parent firstSeenFrame is 10)
  const auto res = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
}

// ============================================================================
// Prompt 60: Repeatability
// ============================================================================

TEST(PersistentReplayRoutingTest, RepeatabilityAcrossInstances) {
  const auto run_pipeline = [](const std::vector<std::vector<PointXYZ>>& frames_pts,
                               const std::vector<std::vector<PointRouting>>& frames_r) {
    PersistentAdaptiveMap map{};
    auto cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

    for (size_t i = 0; i < frames_pts.size(); ++i) {
      const uint64_t seq = (i + 1) * 10;
      EXPECT_TRUE(cache.pushFrame(frames_pts[i], frames_r[i], seq, origin).has_value());

      // Filter static subset for fusion
      std::vector<PointXYZ> static_subset;
      for (size_t p = 0; p < frames_pts[i].size(); ++p) {
        if (frames_r[i][p] == PointRouting::STATIC_FUSABLE) {
          static_subset.push_back(frames_pts[i][p]);
        }
      }
      if (!static_subset.empty()) {
        FrameEvidence ev;
        EXPECT_TRUE(ev.buildFrame(static_subset, seq, origin).has_value());
        EXPECT_TRUE(map.fuseFrameEvidence(ev).has_value());
      }
    }

    EXPECT_TRUE(map.splitLeaf(root_key));
    const auto replay_res = map.replayDirectChildren(root_key, cache);
    EXPECT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

    return std::make_pair(cache.statistics(), map.activePayloadSnapshot());
  };

  const std::vector<std::vector<PointXYZ>> frames = {
      {{0.05, 0.05, 1.0}, {0.25, 0.05, 99.0}},
      {{0.08, 0.08, 2.0}},
      {{0.25, 0.25, 88.0}, {0.05, 0.25, 3.0}},
  };
  const std::vector<std::vector<PointRouting>> routings = {
      {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED},
      {PointRouting::STATIC_FUSABLE},
      {PointRouting::DYNAMIC_EXCLUDED, PointRouting::STATIC_FUSABLE},
  };

  auto [stats1, payloads1] = run_pipeline(frames, routings);
  auto [stats2, payloads2] = run_pipeline(frames, routings);

  EXPECT_EQ(stats1, stats2);
  EXPECT_EQ(payloads1, payloads2);
}

// ============================================================================
// Prompt 59: Deterministic Routed-Cache / Replay Stress (>= 10,000 frames)
// ============================================================================

TEST(PersistentReplayRoutingTest, DeterministicRoutedStress10000Frames) {
  constexpr size_t kTotalFrames = 10000;
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 20});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  PersistentAdaptiveMap map{};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  size_t split_replay_merge_cycles = 0;

  for (size_t f = 1; f <= kTotalFrames; ++f) {
    const uint64_t seq = static_cast<uint64_t>(f);
    const auto origin = avr::spatial::WorldPoint2D{
        (f % 2 == 0) ? -2.0 : 0.0,
        (f % 2 == 0) ? -2.0 : 0.0,
    };

    // Deterministic mix of static, dynamic, and invalid points
    std::vector<PointXYZ> raw_pts;
    std::vector<PointRouting> routing;

    if (f % 4 == 0) {
      // All static
      raw_pts = {{0.05, 0.05, 1.0 + (f % 5) * 0.1}, {0.15, 0.15, 2.0}};
      routing = {PointRouting::STATIC_FUSABLE, PointRouting::STATIC_FUSABLE};
    } else if (f % 4 == 1) {
      // All dynamic
      raw_pts = {{0.25, 0.25, 10.0}, {0.35, 0.35, 20.0}};
      routing = {PointRouting::DYNAMIC_EXCLUDED, PointRouting::DYNAMIC_EXCLUDED};
    } else if (f % 4 == 2) {
      // Mixed with invalid (NaN) point
      raw_pts = {
          {0.05, 0.05, 1.2},
          {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
          {0.25, 0.05, 30.0},
      };
      routing = {
          PointRouting::STATIC_FUSABLE,
          PointRouting::STATIC_FUSABLE,
          PointRouting::DYNAMIC_EXCLUDED,
      };
    } else {
      // Negative coordinates
      raw_pts = {{-1.95, -1.95, 0.5}, {-1.85, -1.85, 5.0}};
      routing = {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED};
    }

    auto prep_opt = cache.prepareFrame(raw_pts, routing, seq, origin);
    ASSERT_TRUE(prep_opt.has_value());
    ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());

    // Periodically (every 1,000 frames) do a test split/replay/merge on a localized root
    if (f % 1000 == 0) {
      PersistentAdaptiveMap local_map{};
      // Prepare 2 frames in local cache
      auto local_cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
      const std::vector<PointXYZ> l_pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 99.0}};
      const std::vector<PointRouting> l_r = {PointRouting::STATIC_FUSABLE, PointRouting::DYNAMIC_EXCLUDED};
      EXPECT_TRUE(local_cache.pushFrame(l_pts, l_r, seq - 1, origin).has_value());

      FrameEvidence ev;
      EXPECT_TRUE(ev.buildFrame(std::vector<PointXYZ>{l_pts[0]}, seq - 1, origin).has_value());
      EXPECT_TRUE(local_map.fuseFrameEvidence(ev).has_value());

      EXPECT_TRUE(local_map.splitLeaf(root_key));
      const auto rep = local_map.replayDirectChildren(root_key, local_cache);
      EXPECT_EQ(rep.status, ReplayStatus::APPLIED_COMPLETE);

      const auto mer = local_map.mergeDirectChildren(root_key, local_cache);
      EXPECT_EQ(mer.status, MergeStatus::APPLIED);
      ++split_replay_merge_cycles;
    }
  }

  EXPECT_EQ(cache.totalFramesPushed(), kTotalFrames);
  EXPECT_EQ(cache.frameCount(), 20ULL);
  EXPECT_EQ(split_replay_merge_cycles, 10ULL);
}

}  // namespace
}  // namespace avr::mapping
