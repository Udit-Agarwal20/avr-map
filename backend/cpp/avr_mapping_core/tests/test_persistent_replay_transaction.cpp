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

struct PersistentAdaptiveMapTestAccess {
  static AdaptivePayloadStore& payloadStore(PersistentAdaptiveMap& map) noexcept {
    return map.payload_store_;
  }
};

namespace {

struct MapStateSnapshot {
  PersistentMapStats stats{};
  uint64_t last_fused_frame_seq{0};
  bool has_fused_frames{false};
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> active_payloads{};
  std::vector<std::pair<avr::spatial::CellKey, AdaptiveCellPayload>> retired_caches{};
  std::vector<avr::spatial::CellKey> replay_required{};
  std::vector<avr::spatial::CellKey> retired_incomplete{};

  static MapStateSnapshot capture(const PersistentAdaptiveMap& map) {
    MapStateSnapshot snap;
    snap.stats = map.statistics();
    snap.last_fused_frame_seq = map.lastFusedFrameSeq();
    snap.has_fused_frames = map.hasFusedFrames();
    snap.active_payloads = map.activePayloadSnapshot();
    snap.retired_caches = map.retiredCacheSnapshot();
    snap.replay_required = map.replayRequiredSnapshot();
    snap.retired_incomplete = map.retiredCacheIncompleteSnapshot();
    return snap;
  }

  bool operator==(const MapStateSnapshot& other) const noexcept = default;
};

// Fixture setting up standard parent with history
class PersistentReplayTransactionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, 10, origin).has_value());
    ASSERT_TRUE(map_.fuseFrameEvidence(ev).has_value());

    root_key_ = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
    ASSERT_TRUE(map_.splitLeaf(root_key_));

    children_ = *avr::spatial::childrenOf(root_key_);
  }

  PersistentAdaptiveMap map_{};
  avr::spatial::CellKey root_key_{};
  std::array<avr::spatial::CellKey, 4> children_{};
};

TEST_F(PersistentReplayTransactionTest, RollbackOnCacheInsufficient) {
  // Empty cache -> insufficient
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnParentIncomplete) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache_opt->pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  // Inject incomplete flag
  const auto* current_cache = map_.findRetiredCache(root_key_);
  ASSERT_NE(current_cache, nullptr);
  PersistentAdaptiveMapTestAccess::payloadStore(map_).setRetiredCache(root_key_, *current_cache, /*incomplete=*/true);

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_CACHE_INCOMPLETE);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnPayloadMismatch) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  // Mismatched measurement points
  const std::vector<PointXYZ> mismatch_pts = {{0.05, 0.05, 99.0}};
  cache_opt->pushFrame(mismatch_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnUnsupportedTopology) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache_opt->pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  // Refine child 0 further
  ASSERT_TRUE(map_.splitLeaf(children_[0]));

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::UNSUPPORTED_TOPOLOGY);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnNoRetiredCache) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());

  // Key with no retired cache
  const auto unallocated_root = *map_.ensureRootForBase(avr::spatial::GridCoord2D{50, 50});
  ASSERT_TRUE(map_.splitLeaf(unallocated_root));

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(unallocated_root, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::NO_RETIRED_PARENT_CACHE);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnTemporalOverlap) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache_opt->pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  // Inject overlapping child payload
  AdaptiveCellPayload overlapping{};
  ElevationStats el{};
  el.update(1.0);
  ASSERT_TRUE(overlapping.tryFuse(el, 10, ResolutionReasonBits{}));
  PersistentAdaptiveMapTestAccess::payloadStore(map_).setActivePayload(children_[0], overlapping);

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::TEMPORAL_OVERLAP);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnAlreadyComplete) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache_opt->pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  // First replay completes
  const auto res1 = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(res1.status, ReplayStatus::APPLIED_COMPLETE);

  // Snapshot after first replay
  const auto before = MapStateSnapshot::capture(map_);

  // Second replay is no-op
  const auto res2 = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(res2.status, ReplayStatus::ALREADY_COMPLETE);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

TEST_F(PersistentReplayTransactionTest, RollbackOnInconsistentReplayState) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  cache_opt->pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  // Inject inconsistent mixed replay state
  PersistentAdaptiveMapTestAccess::payloadStore(map_).setReplayRequired(children_[0], false);

  const auto before = MapStateSnapshot::capture(map_);
  const auto result = map_.replayDirectChildren(root_key_, *cache_opt);
  EXPECT_EQ(result.status, ReplayStatus::INCONSISTENT_REPLAY_STATE);

  const auto after = MapStateSnapshot::capture(map_);
  EXPECT_EQ(before, after);
  EXPECT_TRUE(map_.validateInvariants().is_valid);
}

}  // namespace
}  // namespace avr::mapping
