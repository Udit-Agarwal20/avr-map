#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_merge_result.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
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

namespace {

struct MapStateSnapshot {
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> active_payloads;
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> retired_caches;
  std::vector<avr::spatial::CellKey> replay_required;
  PersistentMapStats stats;

  static MapStateSnapshot capture(const PersistentAdaptiveMap& map) {
    return MapStateSnapshot{
        .active_payloads = map.activePayloadSnapshot(),
        .retired_caches = map.retiredCacheSnapshot(),
        .replay_required = map.replayRequiredSnapshot(),
        .stats = map.statistics(),
    };
  }

  void verifyIdentical(const PersistentAdaptiveMap& map) const {
    EXPECT_EQ(active_payloads, map.activePayloadSnapshot());
    EXPECT_EQ(retired_caches, map.retiredCacheSnapshot());
    EXPECT_EQ(replay_required, map.replayRequiredSnapshot());
    EXPECT_EQ(stats, map.statistics());
  }
};

}  // namespace

// ============================================================================
// Section 48: Transaction Failure Matrix
// ============================================================================

TEST(PersistentMergeTransactionTest, RollbackOnInvalidParent) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());

  const auto snap = MapStateSnapshot::capture(map);

  // Level 0 cannot be coarsened
  const auto l0_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L0);
  const auto res = map.mergeDirectChildren(l0_key, *cache_opt);
  EXPECT_EQ(res.status, MergeStatus::INVALID_PARENT);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnParentNotInternal) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Ingest frame to create LEAF root
  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache_opt->pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  // Parent is LEAF, not INTERNAL!
  EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, *cache_opt);
  EXPECT_EQ(res.status, MergeStatus::PARENT_NOT_INTERNAL);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnChildNotLeaf) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  // Split child 0 (making it INTERNAL)
  const auto children = *avr::spatial::childrenOf(root_key);
  map.splitLeaf(children[0]);

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, MergeStatus::CHILD_NOT_LEAF);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnReplayIncomplete) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  // Split without replay
  map.splitLeaf(root_key);

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, MergeStatus::REPLAY_INCOMPLETE);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnCacheHistoryInsufficient) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  // Reset cache so it's empty
  cache.reset();

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnChildReconstructionMismatch) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  // Create fake cache with modified point elevation
  auto fake_cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  const std::vector<PointXYZ> fake_pts = {{0.05, 0.05, 999.0}};
  fake_cache.pushFrame(fake_pts, 10, origin);

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, fake_cache);
  EXPECT_EQ(res.status, MergeStatus::CHILD_RECONSTRUCTION_MISMATCH);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnInconsistentPayloadState) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  // Inject active payload on internal parent
  ElevationStats bogus{};
  bogus.update(1.0);
  PersistentAdaptiveMapTestAccess::injectActivePayload(map, root_key, AdaptiveCellPayload(bogus, 10, 10, 1, {}));

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, MergeStatus::INCONSISTENT_PAYLOAD_STATE);

  snap.verifyIdentical(map);
}

TEST(PersistentMergeTransactionTest, RollbackOnNoCacheNonEmpty) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  const auto snap = MapStateSnapshot::capture(map);

  // Calling no-cache overload on non-empty children must fail safely with zero mutation
  const auto res = map.mergeDirectChildren(root_key);
  EXPECT_EQ(res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

TEST(PersistentMergeTransactionTest, RollbackOnTemporalMetadataInvalid) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  const auto children = *avr::spatial::childrenOf(root_key);
  // Inject child 0 with invalid temporal metadata: firstSeen (50) > lastSeen (10)
  ElevationStats bogus{};
  bogus.update(1.0);
  AdaptiveCellPayload bogus_payload(bogus, 50, 10, 1, {});
  PersistentAdaptiveMapTestAccess::injectActivePayload(map, children[0], bogus_payload);

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, cache);
  EXPECT_EQ(res.status, MergeStatus::TEMPORAL_METADATA_INVALID);

  snap.verifyIdentical(map);
}

TEST(PersistentMergeTransactionTest, RollbackOnPayloadMergeFailed) {
  PersistentAdaptiveMap map{};
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  const std::vector<PointXYZ> f10 = {{0.05, 0.05, 1.0}};
  cache.pushFrame(f10, 10, origin);
  FrameEvidence ev10;
  ev10.buildFrame(f10, 10, origin);
  map.fuseFrameEvidence(ev10);

  map.splitLeaf(root_key);
  map.replayDirectChildren(root_key, cache);

  const std::vector<PointXYZ> f20 = {{0.05, 0.05, 2.0}};
  cache.pushFrame(f20, 20, origin);
  FrameEvidence ev20;
  ev20.buildFrame(f20, 20, origin);
  map.fuseFrameEvidence(ev20);

  // Construct a corrupt cache with extreme z values (+1e200 and -1e200) across frames 10 and 20
  auto fake_cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  const std::vector<PointXYZ> fake_pts10 = {{0.05, 0.05, 1e200}};
  fake_cache.pushFrame(fake_pts10, 10, origin);
  const std::vector<PointXYZ> fake_pts20 = {{0.05, 0.05, -1e200}};
  fake_cache.pushFrame(fake_pts20, 20, origin);

  const auto snap = MapStateSnapshot::capture(map);

  const auto res = map.mergeDirectChildren(root_key, fake_cache);
  EXPECT_EQ(res.status, MergeStatus::PAYLOAD_MERGE_FAILED);

  snap.verifyIdentical(map);
  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid) << val.error_message;
}

