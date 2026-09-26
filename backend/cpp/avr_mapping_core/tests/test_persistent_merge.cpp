#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_merge_result.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

using namespace avr::mapping;

namespace avr::mapping {

struct PersistentAdaptiveMapTestAccess {
  static void injectActivePayload(PersistentAdaptiveMap& map, avr::spatial::CellKey key, const AdaptiveCellPayload& p) {
    map.payload_store_.setActivePayload(key, p);
  }
};

}  // namespace avr::mapping

// ============================================================================
// Section 30: Basic Empty Merge Test (Cache not required)
// ============================================================================

TEST(PersistentMergeTest, BasicEmptyMergeTest) {
  PersistentAdaptiveMap map{};
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Base root L3 covers [0.0, 0.40) x [0.0, 0.40)
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Split empty L3 leaf to four L2 children
  // Need root to exist in hierarchy first
  ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L2));
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  // Coarsen without cache
  const auto result = map.mergeDirectChildren(root_key);
  EXPECT_EQ(result.status, MergeStatus::APPLIED);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.parent, root_key);
  EXPECT_FALSE(result.parent_payload_created);
  EXPECT_EQ(result.child_observations_verified, 0ULL);
  EXPECT_EQ(result.parent_observations_reconstructed, 0ULL);

  // Hierarchy validation
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_FALSE(map.requiresReplay(root_key));

  for (const auto& child : children) {
    EXPECT_EQ(map.hierarchy().nodeState(child), NodeState::ABSENT);
    EXPECT_EQ(map.findActivePayload(child), nullptr);
    EXPECT_FALSE(map.requiresReplay(child));
  }

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 31: Basic Payload Merge Test
// ============================================================================

TEST(PersistentMergeTest, BasicPayloadMergeTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10: 2 points in child 0
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.08, 0.08, 1.2}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Frame 20: 2 points in child 1
  const std::vector<PointXYZ> f20 = {{0.25, 0.05, 2.0}, {0.28, 0.08, 2.4}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto replay_res = map.replayDirectChildren(root_key, cache);
  ASSERT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

  // Now merge direct children
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());
  EXPECT_TRUE(merge_res.had_parent_retired_cache);
  EXPECT_TRUE(merge_res.parent_payload_created);
  EXPECT_EQ(merge_res.child_observations_verified, 4ULL);
  EXPECT_EQ(merge_res.parent_observations_reconstructed, 4ULL);

  // Parent must be LEAF
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_FALSE(map.requiresReplay(root_key));

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 4ULL);
  EXPECT_EQ(parent_payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_payload->lastSeenFrame(), 20ULL);
  EXPECT_EQ(parent_payload->observedFrameCount(), 2ULL);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 32: Multi-Child Same-Frame Test (Critical: observed_frame_count == 1)
// ============================================================================

TEST(PersistentMergeTest, MultiChildSameFrameTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10: 1 point in each of the 4 children
  const std::vector<PointXYZ> f10 = {
      {0.05, 0.05, 1.0},  // Child 0
      {0.25, 0.05, 2.0},  // Child 1
      {0.05, 0.25, 3.0},  // Child 2
      {0.25, 0.25, 4.0},  // Child 3
  };
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto replay_res = map.replayDirectChildren(root_key, cache);
  ASSERT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    const auto* cp = map.findActivePayload(child);
    ASSERT_NE(cp, nullptr);
    EXPECT_EQ(cp->observedFrameCount(), 1ULL);
  }

  // Merge
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 4ULL);
  // CRITICAL REQUIREMENT: observed_frame_count must be 1, NOT 4!
  EXPECT_EQ(parent_payload->observedFrameCount(), 1ULL);
  EXPECT_NE(parent_payload->observedFrameCount(), 4ULL);
  EXPECT_EQ(parent_payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_payload->lastSeenFrame(), 10ULL);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 33: Overlapping Frame Set Test (Distinct Frame Set Union)
// ============================================================================

TEST(PersistentMergeTest, OverlappingFrameSetTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Child A: 10, 20, 30
  // Child B: 20, 30, 40
  // Child C: 10, 40
  // Child D: 30

  // Frame 10: Child A (0.05, 0.05) and Child C (0.05, 0.25)
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.05, 0.25, 1.1}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Frame 20: Child A and Child B (0.25, 0.05)
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}, {0.25, 0.05, 2.1}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Frame 30: Child A, Child B, Child D (0.25, 0.25)
  const std::vector<PointXYZ> f30 = {{0.05, 0.05, 3.0}, {0.25, 0.05, 3.1}, {0.25, 0.25, 3.2}};
  ASSERT_TRUE(cache.pushFrame(f30, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Frame 40: Child B and Child C
  const std::vector<PointXYZ> f40 = {{0.25, 0.05, 4.0}, {0.05, 0.25, 4.1}};
  ASSERT_TRUE(cache.pushFrame(f40, 40, origin).has_value());
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev40).has_value());

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto replay_res = map.replayDirectChildren(root_key, cache);
  ASSERT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

  // Merge
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 9ULL);
  // CRITICAL: Distinct contributing frames are {10, 20, 30, 40} -> observed_frame_count must be 4, NOT 3+3+2+1=9!
  EXPECT_EQ(parent_payload->observedFrameCount(), 4ULL);
  EXPECT_NE(parent_payload->observedFrameCount(), 9ULL);
  EXPECT_EQ(parent_payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_payload->lastSeenFrame(), 40ULL);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 34: Child Temporal Difference Test
// ============================================================================

TEST(PersistentMergeTest, ChildTemporalDifferenceTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10: only in child 0
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Frame 25: only in child 2
  const std::vector<PointXYZ> f25 = {{0.05, 0.25, 2.5}};
  ASSERT_TRUE(cache.pushFrame(f25, 25, origin).has_value());
  FrameEvidence ev25;
  ASSERT_TRUE(ev25.buildFrame(f25, 25, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev25).has_value());

  // Frame 40: only in child 1
  const std::vector<PointXYZ> f40 = {{0.25, 0.05, 4.0}};
  ASSERT_TRUE(cache.pushFrame(f40, 40, origin).has_value());
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev40).has_value());

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_payload->lastSeenFrame(), 40ULL);
  EXPECT_EQ(parent_payload->observedFrameCount(), 3ULL);
  EXPECT_EQ(parent_payload->observationCount(), 3ULL);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 35: Cache-Evicted Merge Failure
// ============================================================================

TEST(PersistentMergeTest, CacheEvictedMergeFailure) {
  PersistentAdaptiveMap map{};
  // Cache capacity 2
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 2});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split now before frame 10 is evicted
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Now push frame 20 and frame 30 to evict frame 10 from cache
  const std::vector<PointXYZ> f20 = {{0.25, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  const std::vector<PointXYZ> f30 = {{0.25, 0.25, 3.0}};
  ASSERT_TRUE(cache.pushFrame(f30, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Frame 10 is evicted! Child 0 has firstSeenFrame = 10, but cache oldest is 20.
  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation verification
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 36: Same-Sequence Different-Cache Failure
// ============================================================================

TEST(PersistentMergeTest, SameSequenceDifferentCacheFailure) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Create different cache with same frame 10 but different elevation (z = 999.0)
  auto fake_cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(fake_cache_opt.has_value());
  auto& fake_cache = *fake_cache_opt;
  const std::vector<PointXYZ> fake_f10 = {{0.05, 0.05, 999.0}};
  ASSERT_TRUE(fake_cache.pushFrame(fake_f10, 10, origin).has_value());

  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_replay = map.replayRequiredSnapshot();
  const auto pre_stats = map.statistics();
  const auto children = *avr::spatial::childrenOf(root_key);

  const auto merge_res = map.mergeDirectChildren(root_key, fake_cache);
  EXPECT_EQ(merge_res.status, MergeStatus::CHILD_RECONSTRUCTION_MISMATCH);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation across hierarchy, payloads, retired cache, replay required, and stats
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), pre_replay);
  EXPECT_EQ(map.statistics(), pre_stats);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 37: Replay-Required Child Failure
// ============================================================================

TEST(PersistentMergeTest, ReplayRequiredChildFailure) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  // DO NOT REPLAY! Children have REPLAY_REQUIRED = true

  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::REPLAY_INCOMPLETE);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 38: Child Internal Failure
// ============================================================================

TEST(PersistentMergeTest, ChildInternalFailure) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Split child 0 to L1 (making child 0 INTERNAL)
  const auto children = *avr::spatial::childrenOf(root_key);
  ASSERT_TRUE(map.splitLeaf(children[0]));

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::CHILD_NOT_LEAF);
  EXPECT_FALSE(merge_res.isSuccess());

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 39: Parent Active Payload Invalid State
// ============================================================================

TEST(PersistentMergeTest, ParentActivePayloadInvalidState) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Inject active payload onto INTERNAL parent
  ElevationStats bogus_elev{};
  bogus_elev.update(5.0);
  AdaptiveCellPayload bogus_payload(bogus_elev, 10, 10, 1, ResolutionReasonBits{});
  PersistentAdaptiveMapTestAccess::injectActivePayload(map, root_key, bogus_payload);

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::INCONSISTENT_PAYLOAD_STATE);
  EXPECT_FALSE(merge_res.isSuccess());
}

// ============================================================================
// Section 40: Retired Cache Superseded Test
// ============================================================================

TEST(PersistentMergeTest, RetiredCacheSuperseded) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  EXPECT_NE(map.findRetiredCache(root_key), nullptr);

  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_NE(map.findRetiredCache(root_key), nullptr);

  // Coarsen
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  // Retired cache must be GONE
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_NE(map.findActivePayload(root_key), nullptr);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 41: New Observations After Replay Test
// ============================================================================

TEST(PersistentMergeTest, NewObservationsAfterReplayTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // History: 10, 20, 30
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const std::vector<PointXYZ> f20 = {{0.25, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  const std::vector<PointXYZ> f30 = {{0.05, 0.25, 3.0}};
  ASSERT_TRUE(cache.pushFrame(f30, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  // Split + replay
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // New observation frames post-split:
  // Frame 40 into Child 0
  const std::vector<PointXYZ> f40 = {{0.05, 0.05, 1.5}};
  ASSERT_TRUE(cache.pushFrame(f40, 40, origin).has_value());
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev40).has_value());

  // Frame 50 into Child 1
  const std::vector<PointXYZ> f50 = {{0.25, 0.05, 2.5}};
  ASSERT_TRUE(cache.pushFrame(f50, 50, origin).has_value());
  FrameEvidence ev50;
  ASSERT_TRUE(ev50.buildFrame(f50, 50, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev50).has_value());

  // Merge
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 5ULL);
  EXPECT_EQ(parent_payload->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_payload->lastSeenFrame(), 50ULL);
  EXPECT_EQ(parent_payload->observedFrameCount(), 5ULL);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 42: Reason Policy Test
// ============================================================================

TEST(PersistentMergeTest, ReasonPolicyTest) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Supply explicit coarsening reason
  ResolutionReasonBits reasons{};
  reasons.set(ResolutionReason::UNCERTAINTY);

  const auto merge_res = map.mergeDirectChildren(root_key, cache, reasons);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->lastUpdateReasons(), reasons);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 43: Negative Coordinate Merge
// ============================================================================

TEST(PersistentMergeTest, NegativeCoordinateMerge) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{-1.0, -1.0};
  const auto root_key = *avr::spatial::CellKey::fromWorldPoint(avr::spatial::WorldPoint2D{-0.8, -0.8}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{-0.75, -0.75, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);

  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  EXPECT_NE(map.findActivePayload(root_key), nullptr);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 44: Merge-Then-Refine Again
// ============================================================================

TEST(PersistentMergeTest, MergeThenRefineAgain) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split, replay, merge
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);
  ASSERT_EQ(map.mergeDirectChildren(root_key, cache).status, MergeStatus::APPLIED);

  // Now split AGAIN
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  ASSERT_TRUE(map.splitLeaf(root_key));

  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_NE(map.findRetiredCache(root_key), nullptr);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_TRUE(map.requiresReplay(child));
  }

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 45: Second Replay After Re-Split (Split -> Replay -> Merge -> Split -> Replay)
// ============================================================================

TEST(PersistentMergeTest, SplitReplayMergeSplitReplayCycle) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.25, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // 1. Split
  ASSERT_TRUE(map.splitLeaf(root_key));
  // 2. Replay
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);
  // 3. Merge
  ASSERT_EQ(map.mergeDirectChildren(root_key, cache).status, MergeStatus::APPLIED);
  // 4. Split
  ASSERT_TRUE(map.splitLeaf(root_key));
  // 5. Replay again
  const auto second_replay = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(second_replay.status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 46: No Global Ingestion Counter Change
// ============================================================================

TEST(PersistentMergeTest, NoGlobalIngestionCounterChange) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Record cumulative stats before merge
  const auto stats_before = map.statistics();
  const uint64_t pre_last_seq = map.lastFusedFrameSeq();
  const uint64_t pre_frames_fused = map.totalFramesFused();
  const uint64_t pre_obs_fused = map.totalObservationsFused();
  const bool pre_has_fused = map.hasFusedFrames();

  // Merge
  ASSERT_EQ(map.mergeDirectChildren(root_key, cache).status, MergeStatus::APPLIED);

  // Verify all cumulative ingestion counters are completely unchanged via direct accessors
  EXPECT_EQ(map.lastFusedFrameSeq(), pre_last_seq);
  EXPECT_EQ(map.totalFramesFused(), pre_frames_fused);
  EXPECT_EQ(map.totalObservationsFused(), pre_obs_fused);
  EXPECT_EQ(map.hasFusedFrames(), pre_has_fused);

  // Verify all cumulative ingestion counters are completely unchanged via PersistentMapStats
  const auto stats_after = map.statistics();
  EXPECT_EQ(stats_after.last_fused_frame_seq, stats_before.last_fused_frame_seq);
  EXPECT_EQ(stats_after.has_fused_frames, stats_before.has_fused_frames);
  EXPECT_EQ(stats_after.total_frames_fused, stats_before.total_frames_fused);
  EXPECT_EQ(stats_after.total_observations_fused, stats_before.total_observations_fused);
}

// ============================================================================
// Section 47: Current Structural Stats
// ============================================================================

TEST(PersistentMergeTest, CurrentStructuralStats) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto stats_before = map.statistics();
  EXPECT_EQ(stats_before.hierarchy_stats.internal_count, 1ULL);
  EXPECT_EQ(stats_before.hierarchy_stats.leaf_count, 4ULL);
  EXPECT_EQ(stats_before.payload_stats.retired_cache_count, 1ULL);

  // Merge
  ASSERT_EQ(map.mergeDirectChildren(root_key, cache).status, MergeStatus::APPLIED);

  const auto stats_after = map.statistics();
  EXPECT_EQ(stats_after.hierarchy_stats.internal_count, 0ULL);
  EXPECT_EQ(stats_after.hierarchy_stats.leaf_count, 1ULL);
  EXPECT_EQ(stats_after.payload_stats.retired_cache_count, 0ULL);
  EXPECT_EQ(stats_after.payload_stats.active_payload_count, 1ULL);
  EXPECT_EQ(stats_after.payload_stats.replay_required_count, 0ULL);
}

// ============================================================================
// Section 49: Determinism Across Fresh Instances
// ============================================================================

TEST(PersistentMergeTest, DeterminismAcrossFreshInstances) {
  const auto run_pipeline = []() {
    PersistentAdaptiveMap map{};
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
    auto& cache = *cache_opt;
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

    const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.25, 0.05, 2.0}};
    cache.pushFrame(f10, 10, origin);
    FrameEvidence ev10;
    ev10.buildFrame(f10, 10, origin);
    map.fuseFrameEvidence(ev10);

    const std::vector<PointXYZ> f20 = {{0.05, 0.25, 3.0}, {0.25, 0.25, 4.0}};
    cache.pushFrame(f20, 20, origin);
    FrameEvidence ev20;
    ev20.buildFrame(f20, 20, origin);
    map.fuseFrameEvidence(ev20);

    map.splitLeaf(root_key);
    map.replayDirectChildren(root_key, cache);
    ResolutionReasonBits reasons{};
    reasons.set(ResolutionReason::UNCERTAINTY);
    const auto res = map.mergeDirectChildren(root_key, cache, reasons);

    return std::make_tuple(res, map.activePayloadSnapshot(), map.statistics());
  };

  const auto [res1, snap1, stats1] = run_pipeline();
  const auto [res2, snap2, stats2] = run_pipeline();

  EXPECT_EQ(res1.status, res2.status);
  EXPECT_EQ(res1.child_observations_verified, res2.child_observations_verified);
  EXPECT_EQ(res1.parent_observations_reconstructed, res2.parent_observations_reconstructed);
  EXPECT_EQ(snap1, snap2);
  EXPECT_EQ(stats1, stats2);
}

// ============================================================================
// Section 50: No-Cache Non-Empty Merge Rejected (Zero Mutation)
// ============================================================================

TEST(PersistentMergeTest, NoCacheNonEmptyMergeRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
  }
  // At least one child has ACTIVE_PAYLOAD
  EXPECT_NE(map.findActivePayload(children[0]), nullptr);

  // Capture snapshots before no-cache merge call
  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_replay = map.replayRequiredSnapshot();
  const auto pre_stats = map.statistics();

  // Call no-cache overload: MUST reject non-empty merge with zero mutation
  const auto merge_res = map.mergeDirectChildren(root_key);
  EXPECT_EQ(merge_res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation verification
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), pre_replay);
  EXPECT_EQ(map.statistics(), pre_stats);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 51: Long-Lived Region / Short Cache Eviction Test (HOLD-FINE)
// ============================================================================

TEST(PersistentMergeTest, LongLivedRegionShortCacheEviction) {
  PersistentAdaptiveMap map{};
  // Short cache: capacity 3 frames
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10: initial observation in child 0
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Split and replay while frame 10 is still in cache
  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Ingest frames 20, 30, 40, 50 to advance history beyond the 3-frame cache capacity
  const std::vector<PointXYZ> f20 = {{0.25, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  const std::vector<PointXYZ> f30 = {{0.05, 0.25, 3.0}};
  ASSERT_TRUE(cache.pushFrame(f30, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(f30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  const std::vector<PointXYZ> f40 = {{0.25, 0.25, 4.0}};
  ASSERT_TRUE(cache.pushFrame(f40, 40, origin).has_value());
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(f40, 40, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev40).has_value());

  const std::vector<PointXYZ> f50 = {{0.05, 0.05, 1.5}};
  ASSERT_TRUE(cache.pushFrame(f50, 50, origin).has_value());
  FrameEvidence ev50;
  ASSERT_TRUE(ev50.buildFrame(f50, 50, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev50).has_value());

  // The 3-frame cache now retains only {30, 40, 50}. Frames 10 and 20 are evicted.
  // Children remain valid, replay-complete LEAF nodes spanning history [10, 50].
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
  }

  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_replay = map.replayRequiredSnapshot();
  const auto pre_stats = map.statistics();

  // Attempt non-empty merge: must fail safely with CACHE_HISTORY_INSUFFICIENT
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation verification: safe failure (HOLD-FINE)
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), pre_replay);
  EXPECT_EQ(map.statistics(), pre_stats);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 52: Parent Payload Tile Allocated During Merge
// ============================================================================

TEST(PersistentMergeTest, ParentPayloadTileAllocatedDuringMerge) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // 1. Refine empty root L3 leaf to L2 children before any observations exist
  ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L2));
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);

  // 2. Ingest observations directly into the children
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}, {0.25, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto children = *avr::spatial::childrenOf(root_key);
  EXPECT_NE(map.findActivePayload(children[0]), nullptr);
  EXPECT_NE(map.findActivePayload(children[1]), nullptr);
  EXPECT_FALSE(map.requiresReplay(children[0]));
  EXPECT_FALSE(map.requiresReplay(children[1]));

  // Parent still has no active payload and no retired cache
  EXPECT_EQ(map.findActivePayload(root_key), nullptr);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);

  // 3. Merge: parent payload storage must be created/allocated to hold merged active payload
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());
  EXPECT_FALSE(merge_res.had_parent_retired_cache);
  EXPECT_TRUE(merge_res.parent_payload_created);
  EXPECT_EQ(merge_res.child_observations_verified, 2ULL);
  EXPECT_EQ(merge_res.parent_observations_reconstructed, 2ULL);

  // Parent is now LEAF with active payload
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 2ULL);
  EXPECT_EQ(parent_payload->observedFrameCount(), 1ULL);
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);
  EXPECT_FALSE(map.requiresReplay(root_key));

  // Children removed from hierarchy and payload store
  for (const auto& child : children) {
    EXPECT_EQ(map.hierarchy().nodeState(child), NodeState::ABSENT);
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 53: Empty Child + Non-Empty Child Mix Test
// ============================================================================

TEST(PersistentMergeTest, MixedEmptyAndNonEmptyChildMerge) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10: observations in Child 0 and Child 1 only. Child 2 and Child 3 receive NO points.
  const std::vector<PointXYZ> f10 = {
      {0.05, 0.05, 1.0},  // Child 0
      {0.25, 0.05, 2.0},  // Child 1
  };
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);
  EXPECT_NE(map.findActivePayload(children[0]), nullptr);
  EXPECT_NE(map.findActivePayload(children[1]), nullptr);
  EXPECT_EQ(map.findActivePayload(children[2]), nullptr);
  EXPECT_EQ(map.findActivePayload(children[3]), nullptr);
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }

  // Merge direct children
  const auto merge_res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());
  EXPECT_EQ(merge_res.child_observations_verified, 2ULL);
  EXPECT_EQ(merge_res.parent_observations_reconstructed, 2ULL);

  const auto* parent_payload = map.findActivePayload(root_key);
  ASSERT_NE(parent_payload, nullptr);
  EXPECT_EQ(parent_payload->observationCount(), 2ULL);
  EXPECT_EQ(parent_payload->observedFrameCount(), 1ULL);
  // Elevation mean must be (1.0 + 2.0) / 2 = 1.5 with NO fake zero observations from empty children
  EXPECT_DOUBLE_EQ(parent_payload->elevation().mean(), 1.5);
  EXPECT_DOUBLE_EQ(parent_payload->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(parent_payload->elevation().max(), 2.0);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

// ============================================================================
// Section 54: Numeric Reconstruction Failure Test (Welford Arithmetic Overflow)
// ============================================================================

TEST(PersistentMergeTest, NumericReconstructionFailure) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Normal frame 10
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(f10, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(f10, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  ASSERT_TRUE(map.splitLeaf(root_key));
  ASSERT_EQ(map.replayDirectChildren(root_key, cache).status, ReplayStatus::APPLIED_COMPLETE);

  // Normal frame 20 into child 0
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(f20, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(f20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Construct a corrupt cache with extreme z values (+1e200 and -1e200) across frames 10 and 20
  // When candidate parent merges frame 20 into frame 10, delta^2 overflows double to +Inf.
  auto fake_cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  const std::vector<PointXYZ> fake_pts10 = {{0.05, 0.05, 1e200}};
  fake_cache.pushFrame(fake_pts10, 10, origin);
  const std::vector<PointXYZ> fake_pts20 = {{0.05, 0.05, -1e200}};
  fake_cache.pushFrame(fake_pts20, 20, origin);

  const auto pre_active = map.activePayloadSnapshot();
  const auto pre_retired = map.retiredCacheSnapshot();
  const auto pre_replay = map.replayRequiredSnapshot();
  const auto pre_stats = map.statistics();

  // Attempt merge: must fail safely with PAYLOAD_MERGE_FAILED
  const auto merge_res = map.mergeDirectChildren(root_key, fake_cache);
  EXPECT_EQ(merge_res.status, MergeStatus::PAYLOAD_MERGE_FAILED);
  EXPECT_FALSE(merge_res.isSuccess());

  // Zero mutation verification
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
  }
  EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), pre_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), pre_replay);
  EXPECT_EQ(map.statistics(), pre_stats);

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

