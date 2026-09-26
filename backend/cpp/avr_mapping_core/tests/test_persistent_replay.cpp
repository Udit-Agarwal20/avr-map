#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_reconstruction_result.hpp"
#include "avr_mapping/replay_reconstruction_stats.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

// ============================================================================
// AdaptiveCellPayload::tryPrependHistory Unit Tests
// ============================================================================

TEST(PersistentReplayTest, TryPrependHistoryRejectsEmptyHistorical) {
  AdaptiveCellPayload current{};
  ElevationStats cur_elev{};
  cur_elev.update(10.0);
  ASSERT_TRUE(current.tryFuse(cur_elev, 20, ResolutionReasonBits{}));

  AdaptiveCellPayload empty_hist{};
  EXPECT_FALSE(current.tryPrependHistory(empty_hist));
  EXPECT_EQ(current.firstSeenFrame(), 20ULL);
  EXPECT_EQ(current.lastSeenFrame(), 20ULL);
  EXPECT_EQ(current.observedFrameCount(), 1ULL);
}

TEST(PersistentReplayTest, TryPrependHistoryRejectsEmptyCurrent) {
  AdaptiveCellPayload current{};
  AdaptiveCellPayload hist{};
  ElevationStats hist_elev{};
  hist_elev.update(5.0);
  ASSERT_TRUE(hist.tryFuse(hist_elev, 10, ResolutionReasonBits{}));

  EXPECT_FALSE(current.tryPrependHistory(hist));
}

TEST(PersistentReplayTest, TryPrependHistoryRejectsTemporalOverlap) {
  AdaptiveCellPayload current{};
  ElevationStats cur_elev{};
  cur_elev.update(10.0);
  ASSERT_TRUE(current.tryFuse(cur_elev, 20, ResolutionReasonBits{}));

  // Equal frame_seq (overlap)
  AdaptiveCellPayload hist_equal{};
  ElevationStats hist_elev{};
  hist_elev.update(5.0);
  ASSERT_TRUE(hist_equal.tryFuse(hist_elev, 20, ResolutionReasonBits{}));
  EXPECT_FALSE(current.tryPrependHistory(hist_equal));

  // Newer historical frame_seq (inversion)
  AdaptiveCellPayload hist_newer{};
  ASSERT_TRUE(hist_newer.tryFuse(hist_elev, 25, ResolutionReasonBits{}));
  EXPECT_FALSE(current.tryPrependHistory(hist_newer));

  // Current remains completely unchanged
  EXPECT_EQ(current.firstSeenFrame(), 20ULL);
  EXPECT_EQ(current.lastSeenFrame(), 20ULL);
  EXPECT_EQ(current.observedFrameCount(), 1ULL);
}

TEST(PersistentReplayTest, TryPrependHistorySuccessAndFieldInvariants) {
  AdaptiveCellPayload current{};
  ElevationStats cur_elev{};
  cur_elev.update(10.0);
  cur_elev.update(12.0);
  ResolutionReasonBits cur_reasons{};
  cur_reasons.set(ResolutionReason::UNCERTAINTY);
  ASSERT_TRUE(current.tryFuse(cur_elev, 30, cur_reasons));
  ASSERT_TRUE(current.tryFuse(cur_elev, 40, cur_reasons));

  AdaptiveCellPayload hist{};
  ElevationStats hist_elev{};
  hist_elev.update(4.0);
  ASSERT_TRUE(hist.tryFuse(hist_elev, 10, ResolutionReasonBits{}));
  ASSERT_TRUE(hist.tryFuse(hist_elev, 20, ResolutionReasonBits{}));

  ASSERT_TRUE(current.tryPrependHistory(hist));

  // Verification of prepended state
  EXPECT_EQ(current.firstSeenFrame(), 10ULL);
  EXPECT_EQ(current.lastSeenFrame(), 40ULL);
  EXPECT_EQ(current.observedFrameCount(), 4ULL);  // 2 historical + 2 current
  EXPECT_EQ(current.observationCount(), 6ULL);    // 2 historical + 4 current
  EXPECT_DOUBLE_EQ(current.elevation().min(), 4.0);
  EXPECT_DOUBLE_EQ(current.elevation().max(), 12.0);
  // Provenance reasons must remain current reasons
  EXPECT_EQ(current.lastUpdateReasons(), cur_reasons);
}

TEST(PersistentReplayTest, TryPrependHistoryRejectsObservedFrameCountOverflow) {
  ElevationStats cur_elev{};
  cur_elev.update(10.0);
  ResolutionReasonBits cur_reasons(ResolutionReason::UNCERTAINTY);
  AdaptiveCellPayload current(cur_elev, 20, 20, 5, cur_reasons);

  ElevationStats hist_elev{};
  hist_elev.update(5.0);
  // historical + current > UINT64_MAX
  AdaptiveCellPayload hist(hist_elev, 10, 10, std::numeric_limits<uint64_t>::max() - 2, ResolutionReasonBits{});

  EXPECT_FALSE(current.tryPrependHistory(hist));

  // Current payload remains completely unchanged
  EXPECT_EQ(current.firstSeenFrame(), 20ULL);
  EXPECT_EQ(current.lastSeenFrame(), 20ULL);
  EXPECT_EQ(current.observedFrameCount(), 5ULL);
  EXPECT_EQ(current.lastUpdateReasons(), cur_reasons);
  EXPECT_EQ(current.elevation().count(), cur_elev.count());
  EXPECT_DOUBLE_EQ(current.elevation().mean(), cur_elev.mean());
}

TEST(PersistentReplayTest, TryPrependHistoryRejectsNumericMergeFailure) {
  ElevationStats cur_elev{};
  cur_elev.update(-1e200);
  ResolutionReasonBits cur_reasons(ResolutionReason::GEOMETRY_COMPLEXITY);
  AdaptiveCellPayload current(cur_elev, 30, 30, 1, cur_reasons);

  ElevationStats hist_elev{};
  hist_elev.update(1e200);
  AdaptiveCellPayload hist(hist_elev, 10, 10, 1, ResolutionReasonBits{});

  EXPECT_FALSE(current.tryPrependHistory(hist));

  // Current payload remains completely unchanged
  EXPECT_EQ(current.firstSeenFrame(), 30ULL);
  EXPECT_EQ(current.lastSeenFrame(), 30ULL);
  EXPECT_EQ(current.observedFrameCount(), 1ULL);
  EXPECT_EQ(current.lastUpdateReasons(), cur_reasons);
  EXPECT_DOUBLE_EQ(current.elevation().mean(), -1e200);
}

TEST(PersistentReplayTest, TryPrependHistoryPreservesCurrentReasonsStrictly) {
  ElevationStats cur_elev{};
  cur_elev.update(2.0);
  ResolutionReasonBits reasons_b(ResolutionReason::GEOMETRY_COMPLEXITY);
  AdaptiveCellPayload current(cur_elev, 40, 40, 1, reasons_b);

  ElevationStats hist_elev{};
  hist_elev.update(1.0);
  ResolutionReasonBits reasons_a(ResolutionReason::UNCERTAINTY);
  AdaptiveCellPayload hist(hist_elev, 10, 10, 1, reasons_a);

  ASSERT_TRUE(current.tryPrependHistory(hist));

  // Requirement 10: Must equal B, NOT A, NOT A | B
  EXPECT_EQ(current.lastUpdateReasons(), reasons_b);
  EXPECT_NE(current.lastUpdateReasons(), reasons_a);
  ResolutionReasonBits combined = reasons_a;
  combined.set(ResolutionReason::GEOMETRY_COMPLEXITY);
  EXPECT_NE(current.lastUpdateReasons(), combined);
}

// ============================================================================
// Section 28: Standard Complete Replay Example
// ============================================================================

TEST(PersistentReplayTest, CompleteReplayStandardExample) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Base root L3 covers [0.0, 0.40) x [0.0, 0.40).
  // L2 children:
  // Child 0: [0.0, 0.20) x [0.0, 0.20)
  // Child 1: [0.20, 0.40) x [0.0, 0.20)
  // Child 2: [0.0, 0.20) x [0.20, 0.40)
  // Child 3: [0.20, 0.40) x [0.20, 0.40)

  // Frame 10: points in child 0 (A)
  const std::vector<PointXYZ> frame10_pts = {
      {0.05, 0.05, 1.0},
      {0.08, 0.08, 1.2},
  };
  ASSERT_TRUE(cache.pushFrame(frame10_pts, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(frame10_pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  // Frame 20: points in child 1 (B)
  const std::vector<PointXYZ> frame20_pts = {
      {0.25, 0.05, 2.0},
      {0.28, 0.08, 2.4},
  };
  ASSERT_TRUE(cache.pushFrame(frame20_pts, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(frame20_pts, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  // Frame 30: points across child 0 (A), child 2 (C), and child 3 (D)
  const std::vector<PointXYZ> frame30_pts = {
      {0.05, 0.05, 1.5},  // Child 0
      {0.05, 0.25, 3.0},  // Child 2
      {0.25, 0.25, 4.0},  // Child 3
  };
  ASSERT_TRUE(cache.pushFrame(frame30_pts, 30, origin).has_value());
  FrameEvidence ev30;
  ASSERT_TRUE(ev30.buildFrame(frame30_pts, 30, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev30).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto* parent_before = map.findActivePayload(root_key);
  ASSERT_NE(parent_before, nullptr);
  const uint64_t expected_total_obs = parent_before->observationCount();
  EXPECT_EQ(expected_total_obs, 7ULL);

  // Split parent L3 -> four L2 children
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Verify post-split state before replay
  EXPECT_TRUE(map.hierarchy().isInternal(root_key));
  const auto* retired_cache = map.findRetiredCache(root_key);
  ASSERT_NE(retired_cache, nullptr);
  EXPECT_FALSE(map.isRetiredCacheIncomplete(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.hierarchy().isLeaf(child));
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  // Execute Direct-Parent Replay
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.parent_key, root_key);
  EXPECT_EQ(result.cached_frames_considered, 3ULL);
  EXPECT_EQ(result.historical_frames_with_parent_observations, 3ULL);
  EXPECT_EQ(result.parent_replayed_observations, 7ULL);
  EXPECT_EQ(result.parent_expected_observations, 7ULL);
  EXPECT_EQ(result.children_with_historical_payload, 4ULL);
  EXPECT_EQ(result.children_with_existing_current_payload, 0ULL);
  EXPECT_EQ(result.historical_observations_distributed, 7ULL);
  EXPECT_EQ(result.children_replay_cleared, 4ULL);

  // Verify child payloads after commit
  uint64_t child_obs_sum = 0;
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
    const auto* child_payload = map.findActivePayload(child);
    ASSERT_NE(child_payload, nullptr);
    child_obs_sum += child_payload->observationCount();
    // Replay-only payload has empty reason bits
    EXPECT_EQ(child_payload->lastUpdateReasons(), ResolutionReasonBits{});
  }
  EXPECT_EQ(child_obs_sum, expected_total_obs);

  // Parent retired cache remains untouched
  EXPECT_EQ(map.findRetiredCache(root_key), retired_cache);
  EXPECT_EQ(retired_cache->observationCount(), expected_total_obs);

  // Invariant verification
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 21 & 33: Empty Historical Child
// ============================================================================

TEST(PersistentReplayTest, EmptyHistoricalChildReplay) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Only put points in Child 0 (A) and Child 1 (B).
  // Child 2 (C) and Child 3 (D) have NO observations in history.
  const std::vector<PointXYZ> frame10_pts = {
      {0.05, 0.05, 1.0},
      {0.25, 0.05, 2.0},
  };
  ASSERT_TRUE(cache.pushFrame(frame10_pts, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(frame10_pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  EXPECT_TRUE(map.requiresReplay(children[2]));
  EXPECT_TRUE(map.requiresReplay(children[3]));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(result.children_with_historical_payload, 2ULL);
  EXPECT_EQ(result.historical_observations_distributed, 2ULL);
  EXPECT_EQ(result.children_replay_cleared, 4ULL);

  // Child 0 and Child 1 have active payload
  EXPECT_NE(map.findActivePayload(children[0]), nullptr);
  EXPECT_NE(map.findActivePayload(children[1]), nullptr);

  // Child 2 and Child 3 have NO active payload, but REPLAY_REQUIRED is cleared
  EXPECT_EQ(map.findActivePayload(children[2]), nullptr);
  EXPECT_FALSE(map.requiresReplay(children[2]));
  EXPECT_EQ(map.findActivePayload(children[3]), nullptr);
  EXPECT_FALSE(map.requiresReplay(children[3]));

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 32 & 33: Existing Post-Split Child Payload Prepend
// ============================================================================

TEST(PersistentReplayTest, ExistingChildPostSplitPayloadPrepend) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Frames 10, 20: pre-split history
  const std::vector<PointXYZ> frame10_pts = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(frame10_pts, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(frame10_pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const std::vector<PointXYZ> frame20_pts = {{0.05, 0.05, 1.5}};
  ASSERT_TRUE(cache.pushFrame(frame20_pts, 20, origin).has_value());
  FrameEvidence ev20;
  ASSERT_TRUE(ev20.buildFrame(frame20_pts, 20, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev20).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto children = *avr::spatial::childrenOf(root_key);
  const auto child_a = children[0];
  const auto child_d = children[3];  // Empty historical child

  // Frame 40: post-split observation directly into child_a AND child_d
  const std::vector<PointXYZ> frame40_pts = {
      {0.05, 0.05, 3.0},  // in child_a
      {0.25, 0.25, 5.0},  // in child_d
  };
  ASSERT_TRUE(cache.pushFrame(frame40_pts, 40, origin).has_value());
  FrameEvidence ev40;
  ASSERT_TRUE(ev40.buildFrame(frame40_pts, 40, origin).has_value());
  ResolutionReasonBits split_reasons{};
  split_reasons.set(ResolutionReason::GEOMETRY_COMPLEXITY);
  ASSERT_TRUE(map.fuseFrameEvidence(ev40, split_reasons).has_value());

  // Prior to replay:
  // child_a has ACTIVE payload from frame 40, REPLAY_REQUIRED = true
  const auto* child_a_before = map.findActivePayload(child_a);
  ASSERT_NE(child_a_before, nullptr);
  EXPECT_EQ(child_a_before->firstSeenFrame(), 40ULL);
  EXPECT_EQ(child_a_before->lastSeenFrame(), 40ULL);
  EXPECT_EQ(child_a_before->observationCount(), 1ULL);
  EXPECT_TRUE(map.requiresReplay(child_a));

  // child_d has ACTIVE payload from frame 40, REPLAY_REQUIRED = true
  const auto* child_d_before = map.findActivePayload(child_d);
  ASSERT_NE(child_d_before, nullptr);
  EXPECT_EQ(child_d_before->firstSeenFrame(), 40ULL);
  EXPECT_TRUE(map.requiresReplay(child_d));

  // Execute Replay
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(result.children_with_existing_current_payload, 2ULL);
  EXPECT_EQ(result.children_replay_cleared, 4ULL);

  // child_a verification: historical (frames 10, 20) + post-split (frame 40)
  const auto* child_a_after = map.findActivePayload(child_a);
  ASSERT_NE(child_a_after, nullptr);
  EXPECT_FALSE(map.requiresReplay(child_a));
  EXPECT_EQ(child_a_after->firstSeenFrame(), 10ULL);
  EXPECT_EQ(child_a_after->lastSeenFrame(), 40ULL);
  EXPECT_EQ(child_a_after->observedFrameCount(), 3ULL);
  EXPECT_EQ(child_a_after->observationCount(), 3ULL);
  EXPECT_DOUBLE_EQ(child_a_after->elevation().min(), 1.0);
  EXPECT_DOUBLE_EQ(child_a_after->elevation().max(), 3.0);
  // Post-split update reasons preserved!
  EXPECT_EQ(child_a_after->lastUpdateReasons(), split_reasons);

  // child_d verification: had zero historical points, post-split payload is preserved as-is!
  const auto* child_d_after = map.findActivePayload(child_d);
  ASSERT_NE(child_d_after, nullptr);
  EXPECT_FALSE(map.requiresReplay(child_d));
  EXPECT_EQ(child_d_after->firstSeenFrame(), 40ULL);
  EXPECT_EQ(child_d_after->lastSeenFrame(), 40ULL);
  EXPECT_EQ(child_d_after->observationCount(), 1ULL);
  EXPECT_EQ(child_d_after->lastUpdateReasons(), split_reasons);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 36: Negative Coordinate Replay
// ============================================================================

TEST(PersistentReplayTest, NegativeCoordinatesReplay) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{-5.0, -5.0};

  // World coordinates (-2.0, -2.0):
  // Base grid: (-2.0 / 0.05) = -40. L3 grid: floor(-40 / 8) = -5.
  // L3 bounds: [-5 * 8 * 0.05, -4 * 8 * 0.05) = [-2.00, -1.60).
  const std::vector<PointXYZ> frame10_pts = {
      {-1.95, -1.95, -0.5},
      {-1.75, -1.75, 0.5},
  };
  ASSERT_TRUE(cache.pushFrame(frame10_pts, 10, origin).has_value());
  FrameEvidence ev10;
  ASSERT_TRUE(ev10.buildFrame(frame10_pts, 10, origin).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto root_key = *avr::spatial::CellKey::fromWorldPoint(avr::spatial::WorldPoint2D{-1.95, -1.95}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(result.parent_replayed_observations, 2ULL);
  EXPECT_EQ(result.historical_observations_distributed, 2ULL);

  const auto children = *avr::spatial::childrenOf(root_key);
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 37 & 38: Parent Cache and Cache Immutability
// ============================================================================

TEST(PersistentReplayTest, ParentCacheAndRecentObservationCacheImmutability) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}, {0.25, 0.25, 4.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, origin).has_value());
  FrameEvidence ev;
  ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
  ResolutionReasonBits reasons{};
  reasons.set(ResolutionReason::DYNAMIC_RELEVANCE);
  ASSERT_TRUE(map.fuseFrameEvidence(ev, reasons).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  ASSERT_TRUE(map.splitLeaf(root_key));

  // Snapshot parent retired cache before replay
  const auto* retired_before = map.findRetiredCache(root_key);
  ASSERT_NE(retired_before, nullptr);
  const AdaptiveCellPayload saved_parent_cache = *retired_before;
  const bool saved_incomplete = map.isRetiredCacheIncomplete(root_key);

  // Snapshot cache state before replay
  const auto cache_stats_before = cache.statistics();
  const auto frame_seqs_before = cache.frameSequenceSnapshot();

  // Run replay
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  // Section 37: Parent retired cache must remain bit-for-bit identical
  const auto* retired_after = map.findRetiredCache(root_key);
  ASSERT_NE(retired_after, nullptr);
  EXPECT_EQ(*retired_after, saved_parent_cache);
  EXPECT_EQ(retired_after->elevation().count(), saved_parent_cache.elevation().count());
  EXPECT_DOUBLE_EQ(retired_after->elevation().mean(), saved_parent_cache.elevation().mean());
  EXPECT_DOUBLE_EQ(retired_after->elevation().m2(), saved_parent_cache.elevation().m2());
  EXPECT_DOUBLE_EQ(retired_after->elevation().min(), saved_parent_cache.elevation().min());
  EXPECT_DOUBLE_EQ(retired_after->elevation().max(), saved_parent_cache.elevation().max());
  EXPECT_EQ(retired_after->firstSeenFrame(), saved_parent_cache.firstSeenFrame());
  EXPECT_EQ(retired_after->lastSeenFrame(), saved_parent_cache.lastSeenFrame());
  EXPECT_EQ(retired_after->observedFrameCount(), saved_parent_cache.observedFrameCount());
  EXPECT_EQ(retired_after->lastUpdateReasons(), saved_parent_cache.lastUpdateReasons());
  EXPECT_EQ(map.isRetiredCacheIncomplete(root_key), saved_incomplete);

  // Section 38: RecentObservationCache must remain completely unmodified
  const auto cache_stats_after = cache.statistics();
  const auto frame_seqs_after = cache.frameSequenceSnapshot();
  EXPECT_EQ(cache_stats_after, cache_stats_before);
  EXPECT_EQ(frame_seqs_after, frame_seqs_before);
}

// ============================================================================
// Section 49: Repeatability
// ============================================================================

TEST(PersistentReplayTest, RepeatabilityAcrossFreshInstances) {
  PersistentAdaptiveMap map1{};
  PersistentAdaptiveMap map2{};
  auto cache1_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  auto cache2_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache1_opt.has_value());
  ASSERT_TRUE(cache2_opt.has_value());
  auto& cache1 = *cache1_opt;
  auto& cache2 = *cache2_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> f1 = {{0.05, 0.05, 1.0}, {0.25, 0.25, 3.0}};
  const std::vector<PointXYZ> f2 = {{0.05, 0.05, 1.5}, {0.25, 0.05, 2.5}};
  cache1.pushFrame(f1, 10, origin);
  cache1.pushFrame(f2, 20, origin);
  cache2.pushFrame(f1, 10, origin);
  cache2.pushFrame(f2, 20, origin);

  FrameEvidence ev1_a, ev1_b, ev2_a, ev2_b;
  ev1_a.buildFrame(f1, 10, origin);
  ev1_b.buildFrame(f1, 10, origin);
  ev2_a.buildFrame(f2, 20, origin);
  ev2_b.buildFrame(f2, 20, origin);

  map1.fuseFrameEvidence(ev1_a);
  map1.fuseFrameEvidence(ev2_a);
  map2.fuseFrameEvidence(ev1_b);
  map2.fuseFrameEvidence(ev2_b);

  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  map1.splitLeaf(root);
  map2.splitLeaf(root);

  const auto res1 = map1.replayDirectChildren(root, cache1);
  const auto res2 = map2.replayDirectChildren(root, cache2);

  EXPECT_EQ(res1.status, res2.status);
  EXPECT_EQ(res1.parent_replayed_observations, res2.parent_replayed_observations);
  EXPECT_EQ(res1.historical_observations_distributed, res2.historical_observations_distributed);
  EXPECT_EQ(res1.children_replay_cleared, res2.children_replay_cleared);

  const auto children = *avr::spatial::childrenOf(root);
  for (const auto& child : children) {
    const auto* p1 = map1.findActivePayload(child);
    const auto* p2 = map2.findActivePayload(child);
    if (p1 == nullptr) {
      EXPECT_EQ(p2, nullptr);
    } else {
      ASSERT_NE(p2, nullptr);
      EXPECT_EQ(*p1, *p2);
    }
    EXPECT_EQ(map1.requiresReplay(child), map2.requiresReplay(child));
  }
}

TEST(PersistentReplayTest, DoubleReplayReturnsAlreadyCompleteAndNoOp) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache.pushFrame(pts, 10, origin);
  FrameEvidence ev;
  ev.buildFrame(pts, 10, origin);
  map.fuseFrameEvidence(ev);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  map.splitLeaf(root_key);

  // First replay: APPLIED_COMPLETE
  const auto res1 = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(res1.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_TRUE(res1.isSuccess());

  // Record snapshots
  const auto snap_after_first_active = map.activePayloadSnapshot();
  const auto snap_after_first_retired = map.retiredCacheSnapshot();
  const auto snap_after_first_replay = map.replayRequiredSnapshot();
  const auto stats_after_first = map.statistics();

  // Second replay: ALREADY_COMPLETE (Requirement 6)
  const auto res2 = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(res2.status, ReplayStatus::ALREADY_COMPLETE);
  EXPECT_FALSE(res2.isSuccess());

  // Snapshots must be completely identical
  EXPECT_EQ(map.activePayloadSnapshot(), snap_after_first_active);
  EXPECT_EQ(map.retiredCacheSnapshot(), snap_after_first_retired);
  EXPECT_EQ(map.replayRequiredSnapshot(), snap_after_first_replay);
  const auto stats_after_second = map.statistics();
  EXPECT_EQ(stats_after_second.payload_stats.active_payload_count, stats_after_first.payload_stats.active_payload_count);
  EXPECT_EQ(stats_after_second.payload_stats.active_observation_count, stats_after_first.payload_stats.active_observation_count);
  EXPECT_EQ(stats_after_second.payload_stats.retired_cache_count, stats_after_first.payload_stats.retired_cache_count);
  EXPECT_EQ(stats_after_second.payload_stats.retired_cache_observation_count, stats_after_first.payload_stats.retired_cache_observation_count);
  EXPECT_EQ(stats_after_second.payload_stats.replay_required_count, stats_after_first.payload_stats.replay_required_count);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentReplayTest, ReplayDoesNotAdvanceGlobalMapTimeOrIngestionCounters) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  const std::vector<PointXYZ> f20 = {{0.25, 0.25, 2.0}};
  cache.pushFrame(f10, 10, origin);
  cache.pushFrame(f20, 20, origin);
  FrameEvidence ev10, ev20;
  ev10.buildFrame(f10, 10, origin);
  ev20.buildFrame(f20, 20, origin);
  map.fuseFrameEvidence(ev10);
  map.fuseFrameEvidence(ev20);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  map.splitLeaf(root_key);

  // Record global map time and ingestion counters before replay
  const uint64_t last_fused_before = map.lastFusedFrameSeq();
  const size_t frames_fused_before = map.totalFramesFused();
  const auto stats_before = map.statistics();

  // Execute replay
  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  // Ingestion counters and map time must remain identical (Requirements 11 & 12)
  EXPECT_EQ(map.lastFusedFrameSeq(), last_fused_before);
  EXPECT_EQ(map.totalFramesFused(), frames_fused_before);
  const auto stats_after = map.statistics();
  EXPECT_EQ(stats_after.last_fused_frame_seq, stats_before.last_fused_frame_seq);
  EXPECT_EQ(stats_after.total_frames_fused, stats_before.total_frames_fused);
  EXPECT_EQ(stats_after.total_observations_fused, stats_before.total_observations_fused);

  // Payload store metrics reflect newly populated active leaves (Requirement 13)
  EXPECT_EQ(stats_after.payload_stats.active_payload_count, 2ULL);
  EXPECT_EQ(stats_after.payload_stats.active_observation_count, 2ULL);
  EXPECT_EQ(stats_after.payload_stats.retired_cache_count, 1ULL);
  EXPECT_EQ(stats_after.payload_stats.retired_cache_observation_count, 2ULL);
  EXPECT_EQ(stats_after.payload_stats.replay_required_count, 0ULL);
}

TEST(PersistentReplayTest, ChildHistoricalFrameMetadataNarrowerThanParent) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Parent spans frames 10, 20, 30.
  // Child 0 ONLY receives observations in frame 20.
  // Child 1 receives in frame 10.
  // Child 2 receives in frame 30.
  const std::vector<PointXYZ> f10 = {{0.25, 0.05, 1.0}};  // Child 1
  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};  // Child 0
  const std::vector<PointXYZ> f30 = {{0.05, 0.25, 3.0}};  // Child 2

  cache.pushFrame(f10, 10, origin);
  cache.pushFrame(f20, 20, origin);
  cache.pushFrame(f30, 30, origin);

  FrameEvidence e10, e20, e30;
  e10.buildFrame(f10, 10, origin);
  e20.buildFrame(f20, 20, origin);
  e30.buildFrame(f30, 30, origin);

  map.fuseFrameEvidence(e10);
  map.fuseFrameEvidence(e20);
  map.fuseFrameEvidence(e30);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  map.splitLeaf(root_key);

  const auto* parent_retired = map.findRetiredCache(root_key);
  ASSERT_NE(parent_retired, nullptr);
  EXPECT_EQ(parent_retired->firstSeenFrame(), 10ULL);
  EXPECT_EQ(parent_retired->lastSeenFrame(), 30ULL);
  EXPECT_EQ(parent_retired->observedFrameCount(), 3ULL);

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  const auto children = *avr::spatial::childrenOf(root_key);

  // Child 0: narrower range! (Requirement 15)
  const auto* child0 = map.findActivePayload(children[0]);
  ASSERT_NE(child0, nullptr);
  EXPECT_EQ(child0->firstSeenFrame(), 20ULL);
  EXPECT_EQ(child0->lastSeenFrame(), 20ULL);
  EXPECT_EQ(child0->observedFrameCount(), 1ULL);
  EXPECT_EQ(child0->observationCount(), 1ULL);

  // Child 1:
  const auto* child1 = map.findActivePayload(children[1]);
  ASSERT_NE(child1, nullptr);
  EXPECT_EQ(child1->firstSeenFrame(), 10ULL);
  EXPECT_EQ(child1->lastSeenFrame(), 10ULL);
  EXPECT_EQ(child1->observedFrameCount(), 1ULL);

  // Child 2:
  const auto* child2 = map.findActivePayload(children[2]);
  ASSERT_NE(child2, nullptr);
  EXPECT_EQ(child2->firstSeenFrame(), 30ULL);
  EXPECT_EQ(child2->lastSeenFrame(), 30ULL);
  EXPECT_EQ(child2->observedFrameCount(), 1ULL);
}

TEST(PersistentReplayTest, ChildWithOnlyPostSplitObservationsPreservedBitForBit) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // Historical frame 10: only in Child 0
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence e10;
  e10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(e10);

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  map.splitLeaf(root_key);

  const auto children = *avr::spatial::childrenOf(root_key);
  const auto child_d = children[3];  // Zero historical points

  // Post-split frame 40 into Child 3
  const std::vector<PointXYZ> f40 = {{0.25, 0.25, 7.5}};
  cache.pushFrame(f40, 40, origin);
  FrameEvidence e40;
  e40.buildFrame(f40, 40, origin);
  ResolutionReasonBits reasons{};
  reasons.set(ResolutionReason::UNCERTAINTY);
  map.fuseFrameEvidence(e40, reasons);

  const auto* child_d_before = map.findActivePayload(child_d);
  ASSERT_NE(child_d_before, nullptr);
  const AdaptiveCellPayload saved_child_d = *child_d_before;
  EXPECT_TRUE(map.requiresReplay(child_d));

  const auto result = map.replayDirectChildren(root_key, cache);
  EXPECT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE);

  const auto* child_d_after = map.findActivePayload(child_d);
  ASSERT_NE(child_d_after, nullptr);
  // Bit-for-bit identical to pre-replay post-split payload (Requirement 17)
  EXPECT_EQ(*child_d_after, saved_child_d);
  EXPECT_FALSE(map.requiresReplay(child_d));
}

}  // namespace
}  // namespace avr::mapping
