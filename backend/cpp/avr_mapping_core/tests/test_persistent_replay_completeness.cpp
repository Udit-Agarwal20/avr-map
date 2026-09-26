#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_payload_store.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_reconstruction_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

// Helper access for testing edge failure injections
struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
};

namespace {

// ============================================================================
// Section 29: Cache Eviction / History Insufficient
// ============================================================================

TEST(PersistentReplayCompletenessTest, CacheEvictionInsufficientEarlyHistory) {
  PersistentAdaptiveMap map{};
  // Max frames = 3, but parent receives 4 frames (10, 20, 30, 40)
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  for (uint64_t seq : {10ULL, 20ULL, 30ULL, 40ULL}) {
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
    ASSERT_TRUE(cache.pushFrame(pts, seq, origin).has_value());
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Cache now holds [20, 30, 40]; frame 10 has been evicted
  EXPECT_EQ(cache.oldestFrameSeq(), 20ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_FALSE(result.isSuccess());

  // Invariant: zero child mutation, all REPLAY_REQUIRED remain
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentReplayCompletenessTest, CacheEmptyReturnsHistoryInsufficient) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Empty cache (no frames pushed)
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentReplayCompletenessTest, CacheMissingLateHistoryReturnsHistoryInsufficient) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Map receives 10 and 20
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  for (uint64_t seq : {10ULL, 20ULL}) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Cache only pushed frame 10, newest is 10 < parent.last_seen (20)
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 30: Cache Content Mismatch
// ============================================================================

TEST(PersistentReplayCompletenessTest, SameSequenceMismatchedCacheContent) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Map receives z = 2.0 at frame 10 and 20
  const std::vector<PointXYZ> map_pts = {{0.05, 0.05, 2.0}};
  for (uint64_t seq : {10ULL, 20ULL}) {
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(map_pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Cache receives DIFFERENT measurements (z = 99.0) at the SAME frame numbers
  const std::vector<PointXYZ> cache_pts = {{0.05, 0.05, 99.0}};
  for (uint64_t seq : {10ULL, 20ULL}) {
    ASSERT_TRUE(cache.pushFrame(cache_pts, seq, origin).has_value());
  }

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH);
  EXPECT_FALSE(result.isSuccess());

  // Zero mutation on mismatch
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 31: Incomplete Retired Cache
// ============================================================================

TEST(PersistentReplayCompletenessTest, IncompleteRetiredCacheRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Mark parent retired cache as INCOMPLETE via test access
  const auto* current_cache = map.findRetiredCache(root_key);
  ASSERT_NE(current_cache, nullptr);
  PersistentAdaptiveMapTestAccess::payloadStore(map).setRetiredCache(root_key, *current_cache, /*incomplete=*/true);
  EXPECT_TRUE(map.isRetiredCacheIncomplete(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_CACHE_INCOMPLETE);
  EXPECT_FALSE(result.isSuccess());

  // Descendant replay gaps remain uncleared
  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
  }
}

// ============================================================================
// Section 34: Child Already Internal (Unsupported Topology)
// ============================================================================

TEST(PersistentReplayCompletenessTest, ChildAlreadyInternalRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Refine child 0 further: L2 -> L1
  const auto children = *avr::spatial::childrenOf(root_key);
  ASSERT_TRUE(map.splitLeaf(children[0]));
  EXPECT_TRUE(map.hierarchy().isInternal(children[0]));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::UNSUPPORTED_TOPOLOGY);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 35: No Retired Parent Cache
// ============================================================================

TEST(PersistentReplayCompletenessTest, NoRetiredParentCacheRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto root_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
  // Split empty root: parent becomes INTERNAL but has NO retired cache
  ASSERT_TRUE(map.splitLeaf(root_key));
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  EXPECT_EQ(map.findRetiredCache(root_key), nullptr);

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::NO_RETIRED_PARENT_CACHE);
  EXPECT_FALSE(result.isSuccess());
}

// ============================================================================
// Invalid Parent & Topology
// ============================================================================

TEST(PersistentReplayCompletenessTest, InvalidParentRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // L0 cell cannot be a parent
  const auto l0_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto res_l0 = map.replayDirectChildren(l0_key, cache);
  EXPECT_EQ(res_l0.status, ReplayStatus::INVALID_PARENT);

  // Absent cell
  const auto absent_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{500, 500}, avr::spatial::L3);
  const auto res_absent = map.replayDirectChildren(absent_key, cache);
  EXPECT_EQ(res_absent.status, ReplayStatus::INVALID_PARENT);

  // Leaf cell is unsupported topology (not INTERNAL)
  const auto leaf_key = *map.ensureRootForBase(avr::spatial::GridCoord2D{0, 0});
  const auto res_leaf = map.replayDirectChildren(leaf_key, cache);
  EXPECT_EQ(res_leaf.status, ReplayStatus::UNSUPPORTED_TOPOLOGY);
}

// ============================================================================
// Temporal Overlap in Child Post-Split Payload
// ============================================================================

TEST(PersistentReplayCompletenessTest, ChildTemporalOverlapRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);

  // Inject child payload with first_seen_frame <= parent.last_seen_frame (10)
  AdaptiveCellPayload invalid_child{};
  ElevationStats el{};
  el.update(2.0);
  ASSERT_TRUE(invalid_child.tryFuse(el, 10, ResolutionReasonBits{}));
  PersistentAdaptiveMapTestAccess::payloadStore(map).setActivePayload(children[0], invalid_child);

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::TEMPORAL_OVERLAP);
  EXPECT_FALSE(result.isSuccess());
}

// ============================================================================
// Requirement 19: Parent Observed Frame Count Mismatch
// ============================================================================

TEST(PersistentReplayCompletenessTest, ParentReconstructionObservedFrameCountMismatch) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Map receives 1 point at frame 10 (z = 2.0) and 1 point at frame 20 (z = 2.0).
  // Total parent: count = 2, mean = 2.0, M2 = 0.0, min = 2.0, max = 2.0,
  // first_seen = 10, last_seen = 20, observed_frame_count = 2.
  const std::vector<PointXYZ> m10 = {{0.05, 0.05, 2.0}};
  const std::vector<PointXYZ> m20 = {{0.08, 0.08, 2.0}};
  FrameEvidence em10, em20;
  ASSERT_TRUE(em10.buildFrame(m10, 10, origin).has_value());
  ASSERT_TRUE(em20.buildFrame(m20, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(em10).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(em20).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Cache receives BOTH points in frame 10, and a dummy point outside parent in frame 20.
  // Cache window spans [10, 20], so early screening passes.
  const std::vector<PointXYZ> c10 = {{0.05, 0.05, 2.0}, {0.08, 0.08, 2.0}};
  const std::vector<PointXYZ> c20 = {{10.0, 10.0, 2.0}};  // In different root
  ASSERT_TRUE(cache.pushFrame(c10, 10, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(c20, 20, origin).has_value());

  // Reconstructed parent will have:
  // count = 2, mean = 2.0, M2 = 0.0, min = 2.0, max = 2.0 (ELEVATION STATS MATCH!)
  // BUT observed_frame_count = 1 (differs from parent's 2)
  // and last_seen = 10 (differs from parent's 20)
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Requirement 20: Parent First/Last Frame Mismatch
// ============================================================================

TEST(PersistentReplayCompletenessTest, ParentReconstructionFirstLastFrameMismatch) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Map receives 1 point at frame 10 (z = 2.0) and 1 point at frame 30 (z = 2.0).
  // Parent: count = 2, mean = 2.0, first_seen = 10, last_seen = 30, observed_frame_count = 2.
  const std::vector<PointXYZ> m10 = {{0.05, 0.05, 2.0}};
  const std::vector<PointXYZ> m30 = {{0.08, 0.08, 2.0}};
  FrameEvidence em10, em30;
  ASSERT_TRUE(em10.buildFrame(m10, 10, origin).has_value());
  ASSERT_TRUE(em30.buildFrame(m30, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(em10).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(em30).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Cache receives point at frame 10 and frame 20 in parent, and dummy at frame 30 outside parent.
  // Cache window spans [10, 30], early screening passes.
  const std::vector<PointXYZ> c10 = {{0.05, 0.05, 2.0}};
  const std::vector<PointXYZ> c20 = {{0.08, 0.08, 2.0}};
  const std::vector<PointXYZ> c30 = {{10.0, 10.0, 2.0}};
  ASSERT_TRUE(cache.pushFrame(c10, 10, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(c20, 20, origin).has_value());
  ASSERT_TRUE(cache.pushFrame(c30, 30, origin).has_value());

  // Reconstructed parent has count = 2, mean = 2.0, observed_frame_count = 2.
  // But last_seen is 20 vs 30!
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Requirement 5: Mixed Replay State Rejected
// ============================================================================

TEST(PersistentReplayCompletenessTest, MixedReplayStateRejected) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);

  // Clear replay required on only 1 child, leaving a mixed state (3 required, 1 clear)
  PersistentAdaptiveMapTestAccess::payloadStore(map).setReplayRequired(children[0], false);

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::INCONSISTENT_REPLAY_STATE);
  EXPECT_FALSE(result.isSuccess());
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace
}  // namespace avr::mapping
