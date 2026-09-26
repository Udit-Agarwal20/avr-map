#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "avr_mapping/cached_observation_frame.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/prepared_cached_observation_frame.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_cache_stats.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {
namespace {

TEST(RecentObservationCacheStagedTest, PreparedFrameTraitsAndMoveOnly) {
  static_assert(!std::is_copy_constructible_v<PreparedCachedObservationFrame>);
  static_assert(!std::is_copy_assignable_v<PreparedCachedObservationFrame>);
  static_assert(std::is_move_constructible_v<PreparedCachedObservationFrame>);
  static_assert(std::is_move_assignable_v<PreparedCachedObservationFrame>);

  // Default-constructed prepared frame is invalid
  PreparedCachedObservationFrame default_prep;
  EXPECT_FALSE(default_prep.isValid());
  EXPECT_EQ(default_prep.frameSeq(), 0ULL);
  EXPECT_EQ(default_prep.sensorOrigin().x, 0.0);
  EXPECT_EQ(default_prep.sensorOrigin().y, 0.0);
  EXPECT_EQ(default_prep.statistics().input_points, 0ULL);

  // Construct from valid CachedObservationFrame
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  auto raw_frame_opt = CachedObservationFrame::create(
      pts, 10, avr::spatial::WorldPoint2D{1.5, 2.5});
  ASSERT_TRUE(raw_frame_opt.has_value());

  PreparedCachedObservationFrame prep(std::move(*raw_frame_opt));
  EXPECT_TRUE(prep.isValid());
  EXPECT_EQ(prep.frameSeq(), 10ULL);
  EXPECT_EQ(prep.sensorOrigin().x, 1.5);
  EXPECT_EQ(prep.sensorOrigin().y, 2.5);
  EXPECT_EQ(prep.statistics().input_points, 1ULL);
  EXPECT_EQ(prep.statistics().replay_candidates, 1ULL);

  // Move-construct
  PreparedCachedObservationFrame moved(std::move(prep));
  EXPECT_TRUE(moved.isValid());
  EXPECT_EQ(moved.frameSeq(), 10ULL);

  // Extract frame invalidates prepared wrapper
  auto extracted = std::move(moved).extractFrame();
  EXPECT_EQ(extracted.frameSeq(), 10ULL);
  EXPECT_FALSE(moved.isValid());
  EXPECT_EQ(moved.frameSeq(), 0ULL);
}

TEST(RecentObservationCacheStagedTest, PrepareZeroMutationOnCache) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto stats_before = cache.statistics();
  const auto seqs_before = cache.frameSequenceSnapshot();
  EXPECT_EQ(cache.frameCount(), 2ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 2ULL);

  // Prepare frame 30 with 5 valid points
  const std::vector<PointXYZ> pts30 = {
      {0.05, 0.05, 1.0},
      {0.10, 0.10, 2.0},
      {0.15, 0.15, 3.0},
      {0.20, 0.20, 4.0},
      {0.25, 0.25, 5.0},
  };
  auto prep_opt = cache.prepareFrame(pts30, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_TRUE(prep_opt->isValid());
  EXPECT_EQ(prep_opt->frameSeq(), 30ULL);
  EXPECT_EQ(prep_opt->statistics().input_points, 5ULL);

  // Cache MUST remain bit-for-bit unchanged
  EXPECT_EQ(cache.frameCount(), 2ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 2ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 20ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), seqs_before);
  EXPECT_EQ(cache.statistics(), stats_before);
}

TEST(RecentObservationCacheStagedTest, PrepareAtCapacityDoesNotEvict) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 2});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 2ULL);

  // Cache is at capacity (2/2). Prepare frame 30.
  auto prep_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());

  // Frame 10 MUST NOT be evicted by prepare
  EXPECT_EQ(cache.frameCount(), 2ULL);
  EXPECT_TRUE(cache.containsFrame(10));
  EXPECT_TRUE(cache.containsFrame(20));
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 20ULL);
}

TEST(RecentObservationCacheStagedTest, PreparePointerLifetimePreserved) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 2});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Borrow pointer to frame 10
  const auto* p10 = cache.findFrame(10);
  ASSERT_NE(p10, nullptr);
  EXPECT_EQ(p10->frameSeq(), 10ULL);

  // Prepare frame 30
  auto prep_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());

  // Borrowed pointer remains completely valid
  EXPECT_EQ(p10->frameSeq(), 10ULL);
  EXPECT_EQ(p10->rawPoints().size(), 1ULL);
}

TEST(RecentObservationCacheStagedTest, PreCommitQueryInvisibility) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  auto prep_opt = cache.prepareFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());

  // Prepared frame 20 is invisible to all public cache queries
  EXPECT_EQ(cache.findFrame(20), nullptr);
  EXPECT_FALSE(cache.containsFrame(20));

  const auto seqs = cache.frameSequenceSnapshot();
  EXPECT_EQ(seqs.size(), 1ULL);
  EXPECT_EQ(seqs[0], 10ULL);

  EXPECT_TRUE(cache.bucketKeySnapshot(20).empty());

  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  std::vector<uint32_t> indices = {99};
  EXPECT_FALSE(cache.collectObservationIndices(20, l3_key, indices));
  EXPECT_TRUE(indices.empty());
}

TEST(RecentObservationCacheStagedTest, DiscardPreparedFrameHasZeroEffect) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto stats_before = cache.statistics();

  // Create and discard prepared frame in nested block
  {
    auto prep_opt = cache.prepareFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(prep_opt.has_value());
  }

  // Destruction has zero side-effects
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);
  EXPECT_EQ(cache.statistics(), stats_before);
}

TEST(RecentObservationCacheStagedTest, CommitMovesFrameAndPerformsEviction) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 3ULL);

  // Prepare frame 40
  auto prep_opt = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());

  // Commit frame 40
  const auto commit_stats_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  ASSERT_TRUE(commit_stats_opt.has_value());
  EXPECT_EQ(commit_stats_opt->frame_seq, 40ULL);

  // Exactly one frame evicted (10); retained are 20, 30, 40
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 4ULL);
  EXPECT_FALSE(cache.containsFrame(10));
  EXPECT_TRUE(cache.containsFrame(20));
  EXPECT_TRUE(cache.containsFrame(30));
  EXPECT_TRUE(cache.containsFrame(40));
  EXPECT_EQ(cache.oldestFrameSeq(), 20ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);
}

TEST(RecentObservationCacheStagedTest, CapacityOneStagedCommit) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 1});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);

  // Prepare 20
  auto prep_opt = cache.prepareFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);

  // Commit 20
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 20ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 20ULL);
  EXPECT_FALSE(cache.containsFrame(10));
  EXPECT_TRUE(cache.containsFrame(20));
}

TEST(RecentObservationCacheStagedTest, StalePreparedFrameRejectedWithZeroMutation) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Prepare frame 30 against current state [10, 20]
  auto prep30_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep30_opt.has_value());

  // Now frame 40 is committed directly
  ASSERT_TRUE(cache.pushFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);
  const auto stats_after_40 = cache.statistics();

  // Attempting to commit prepared frame 30 MUST FAIL (30 <= 40)
  auto stale_commit_opt = cache.commitPreparedFrame(std::move(*prep30_opt));
  EXPECT_FALSE(stale_commit_opt.has_value());

  // Cache state and stats remain unchanged from state containing 40
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);
  EXPECT_EQ(cache.statistics(), stats_after_40);
  EXPECT_FALSE(cache.containsFrame(30));
}

TEST(RecentObservationCacheStagedTest, MultiplePreparedFramesFromSameSnapshot) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Prepare two candidates from the same snapshot
  auto prep30_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  auto prep40_opt = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep30_opt.has_value());
  ASSERT_TRUE(prep40_opt.has_value());

  // Commit in order: 30 then 40
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep30_opt)).has_value());
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40_opt)).has_value());

  const auto seqs = cache.frameSequenceSnapshot();
  EXPECT_EQ(seqs, (std::vector<uint64_t>{20, 30, 40}));
}

TEST(RecentObservationCacheStagedTest, ReverseCommitRejection) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  auto prep30_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  auto prep40_opt = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep30_opt.has_value());
  ASSERT_TRUE(prep40_opt.has_value());

  // Commit 40 first
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40_opt)).has_value());

  // Committing 30 afterward fails
  EXPECT_FALSE(cache.commitPreparedFrame(std::move(*prep30_opt)).has_value());
}

TEST(RecentObservationCacheStagedTest, CacheResetInteractionWithPreparedFrame) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  auto prep30_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep30_opt.has_value());

  // Reset clears cache
  cache.reset();
  EXPECT_EQ(cache.frameCount(), 0ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 0ULL);

  // Prepared 30 commits successfully against the reset (empty) cache
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep30_opt)).has_value());
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 30ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 30ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);
}

TEST(RecentObservationCacheStagedTest, ValidEmptyFrameStagedCommit) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> empty_pts{};
  auto prep_opt = cache.prepareFrame(empty_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_EQ(prep_opt->statistics().input_points, 0ULL);

  auto commit_stats_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  ASSERT_TRUE(commit_stats_opt.has_value());
  EXPECT_EQ(commit_stats_opt->input_points, 0ULL);
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.statistics().stored_raw_points, 0ULL);
}

TEST(RecentObservationCacheStagedTest, InvalidRawPartitionBehaviorPreserved) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> mixed_pts = {
      {0.05, 0.05, 1.0},                                                // valid
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},              // non-finite
      {1e20, 0.0, 0.0},                                                 // out of range
      {0.15, 0.15, 2.0},                                                // valid
  };

  auto prep_opt = cache.prepareFrame(mixed_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_EQ(prep_opt->statistics().input_points, 4ULL);
  EXPECT_EQ(prep_opt->statistics().replay_candidates, 2ULL);
  EXPECT_EQ(prep_opt->statistics().non_finite_unindexed, 1ULL);
  EXPECT_EQ(prep_opt->statistics().out_of_range_unindexed, 1ULL);

  auto commit_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  ASSERT_TRUE(commit_opt.has_value());
  EXPECT_EQ(cache.statistics().stored_raw_points, 4ULL);
  EXPECT_EQ(cache.statistics().stored_replay_candidates, 2ULL);
}

TEST(RecentObservationCacheStagedTest, PrepareCommitVsLegacyPushEquivalence) {
  auto cacheA_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  auto cacheB_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cacheA_opt.has_value());
  ASSERT_TRUE(cacheB_opt.has_value());
  auto& cacheA = *cacheA_opt;
  auto& cacheB = *cacheB_opt;

  const std::vector<PointXYZ> pts1 = {{0.05, 0.05, 1.0}, {-0.05, -0.05, 2.0}};
  const std::vector<PointXYZ> pts2 = {{0.15, 0.15, 3.0}};
  const std::vector<PointXYZ> pts3 = {{-0.25, 0.25, 4.0}, {0.35, -0.35, 5.0}};
  const std::vector<PointXYZ> pts4 = {{0.45, 0.45, 6.0}};

  // Cache A: legacy pushFrame
  ASSERT_TRUE(cacheA.pushFrame(pts1, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cacheA.pushFrame(pts2, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cacheA.pushFrame(pts3, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cacheA.pushFrame(pts4, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Cache B: prepareFrame + commitPreparedFrame
  auto p1 = cacheB.prepareFrame(pts1, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(p1.has_value());
  ASSERT_TRUE(cacheB.commitPreparedFrame(std::move(*p1)).has_value());

  auto p2 = cacheB.prepareFrame(pts2, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(p2.has_value());
  ASSERT_TRUE(cacheB.commitPreparedFrame(std::move(*p2)).has_value());

  auto p3 = cacheB.prepareFrame(pts3, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(p3.has_value());
  ASSERT_TRUE(cacheB.commitPreparedFrame(std::move(*p3)).has_value());

  auto p4 = cacheB.prepareFrame(pts4, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(p4.has_value());
  ASSERT_TRUE(cacheB.commitPreparedFrame(std::move(*p4)).has_value());

  // Bit-for-bit equivalence
  EXPECT_EQ(cacheA.frameSequenceSnapshot(), cacheB.frameSequenceSnapshot());
  EXPECT_EQ(cacheA.statistics(), cacheB.statistics());
  EXPECT_EQ(cacheA.totalFramesPushed(), cacheB.totalFramesPushed());

  for (const uint64_t seq : cacheA.frameSequenceSnapshot()) {
    EXPECT_EQ(cacheA.bucketKeySnapshot(seq), cacheB.bucketKeySnapshot(seq));
  }
}

TEST(RecentObservationCacheStagedTest, ReplayIsolationGolden) {
  // Capacity = 3
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Frame 40 arrives: prepare it without committing
  auto prep40_opt = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep40_opt.has_value());

  // Verify cache still holds [10, 20, 30]
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10, 20, 30}));
  EXPECT_TRUE(cache.containsFrame(10));

  // Run replay-style query against frame 10: succeeds!
  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  std::vector<uint32_t> indices;
  EXPECT_TRUE(cache.collectObservationIndices(10, l3_key, indices));
  EXPECT_EQ(indices.size(), 1ULL);

  // Now commit frame 40
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40_opt)).has_value());

  // Cache now holds [20, 30, 40]; frame 10 is safely evicted
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
  EXPECT_FALSE(cache.containsFrame(10));
}

TEST(RecentObservationCacheStagedTest, DirectM8BIntegrationTest) {
  // Construct a PersistentAdaptiveMap with an L3 root that receives frames 10, 20, 30
  PersistentAdaptiveMap map;
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_opt = map.ensureRootForBase(base_coord);
  ASSERT_TRUE(root_opt.has_value());
  const auto parent_key = *root_opt;

  // Fuse 3 frames into root leaf, and push into cache
  for (uint64_t f : {10ULL, 20ULL, 30ULL}) {
    const std::vector<PointXYZ> raw = {{0.05, 0.05, 1.0 + 0.1 * static_cast<double>(f)}};
    ASSERT_TRUE(cache.pushFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Frame 40 arrives: prepare it staged without committing
  const std::vector<PointXYZ> raw40 = {{0.05, 0.05, 5.0}};
  auto prep40_opt = cache.prepareFrame(raw40, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep40_opt.has_value());

  // Now split parent
  ASSERT_TRUE(map.splitLeaf(parent_key));

  // Perform M8B historical replay using cache: SUCCEEDS because frame 10 was not evicted!
  const auto replay_result = map.replayDirectChildren(parent_key, cache);
  EXPECT_EQ(replay_result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(replay_result.cached_frames_considered, 3ULL);

  // Now commit frame 40 to cache: eviction of frame 10 occurs safely AFTER replay!
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40_opt)).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
}

TEST(RecentObservationCacheStagedTest, LegacyEarlyPushContrastTest) {
  // Contrast: Legacy early-push causes historical replay failure
  PersistentAdaptiveMap map;
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_opt = map.ensureRootForBase(base_coord);
  ASSERT_TRUE(root_opt.has_value());
  const auto parent_key = *root_opt;

  for (uint64_t f : {10ULL, 20ULL, 30ULL}) {
    const std::vector<PointXYZ> raw = {{0.05, 0.05, 1.0 + 0.1 * static_cast<double>(f)}};
    ASSERT_TRUE(cache.pushFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Early-push: caller pushes frame 40 BEFORE replay
  const std::vector<PointXYZ> raw40 = {{0.05, 0.05, 5.0}};
  ASSERT_TRUE(cache.pushFrame(raw40, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Cache evicted frame 10!
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));

  // Now split parent and attempt replay
  ASSERT_TRUE(map.splitLeaf(parent_key));
  const auto replay_result = map.replayDirectChildren(parent_key, cache);

  // Replay FAILS because frame 10 was evicted prematurely!
  EXPECT_EQ(replay_result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
}

TEST(RecentObservationCacheStagedTest, MovedFromPreparedFrameRejected) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  auto prep_opt = cache.prepareFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_TRUE(prep_opt->isValid());

  // First commit succeeds
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());
  EXPECT_FALSE(prep_opt->isValid());

  // Second commit with moved-from prepared object is rejected
  auto retry_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  EXPECT_FALSE(retry_opt.has_value());
}

TEST(RecentObservationCacheStagedTest, DeterministicStress10000Operations) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  uint64_t current_seq = 100;
  size_t prepared_count = 0;
  size_t committed_count = 0;
  size_t discarded_count = 0;

  for (size_t i = 0; i < 10000; ++i) {
    const uint64_t f_seq = current_seq++;
    const std::vector<PointXYZ> pts = {
        {0.05 * static_cast<double>(i % 10), -0.05 * static_cast<double>(i % 10), 1.0},
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},  // invalid point
    };

    auto prep_opt = cache.prepareFrame(pts, f_seq, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(prep_opt.has_value());
    ++prepared_count;

    if (i % 3 == 0) {
      // Discard without commit
      ++discarded_count;
    } else {
      // Commit
      auto commit_opt = cache.commitPreparedFrame(std::move(*prep_opt));
      ASSERT_TRUE(commit_opt.has_value());
      ++committed_count;
    }

    EXPECT_LE(cache.frameCount(), 5ULL);
  }

  EXPECT_EQ(prepared_count, 10000ULL);
  EXPECT_GT(discarded_count, 0ULL);
  EXPECT_GT(committed_count, 0ULL);
  EXPECT_EQ(cache.totalFramesPushed(), committed_count);
  EXPECT_EQ(cache.frameCount(), 5ULL);
}

TEST(RecentObservationCacheStagedTest, MemoryBoundStressTest) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  for (uint64_t seq = 1; seq <= 2000; ++seq) {
    auto prep_opt = cache.prepareFrame(pt, seq, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(prep_opt.has_value());

    auto commit_opt = cache.commitPreparedFrame(std::move(*prep_opt));
    ASSERT_TRUE(commit_opt.has_value());

    EXPECT_LE(cache.frameCount(), 3ULL);
    EXPECT_LE(cache.statistics().stored_frames, 3ULL);
  }

  EXPECT_EQ(cache.totalFramesPushed(), 2000ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 1998ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 2000ULL);
}

TEST(RecentObservationCacheStagedTest, NonFiniteRawPointGolden) {
  // Construct raw frame containing in order:
  // valid point A, NaN point, valid point B, +Inf point, out-of-range finite point, valid point C.
  const PointXYZ ptA{0.05, 0.05, 1.0};
  const PointXYZ ptNaN{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  const PointXYZ ptB{0.15, 0.15, 2.0};
  const PointXYZ ptInf{0.0, std::numeric_limits<double>::infinity(), 0.0};
  const PointXYZ ptOutOfRange{1e20, 0.0, 0.0};
  const PointXYZ ptC{0.25, 0.25, 3.0};

  const std::vector<PointXYZ> raw_seq = {ptA, ptNaN, ptB, ptInf, ptOutOfRange, ptC};

  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Prepare with finite sensor origin MUST SUCCEED
  auto prep_opt = cache.prepareFrame(raw_seq, 10, avr::spatial::WorldPoint2D{1.0, 2.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_TRUE(prep_opt->isValid());

  // Check statistics partition
  const auto prep_stats = prep_opt->statistics();
  EXPECT_EQ(prep_stats.input_points, 6ULL);
  EXPECT_EQ(prep_stats.replay_candidates, 3ULL);
  EXPECT_EQ(prep_stats.non_finite_unindexed, 2ULL);
  EXPECT_EQ(prep_stats.out_of_range_unindexed, 1ULL);
  EXPECT_EQ(prep_stats.input_points,
            prep_stats.replay_candidates + prep_stats.non_finite_unindexed + prep_stats.out_of_range_unindexed);

  // Commit
  auto commit_stats_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  ASSERT_TRUE(commit_stats_opt.has_value());

  // Verify retained frame still contains all original raw points in exact arrival order
  const auto* retained = cache.findFrame(10);
  ASSERT_NE(retained, nullptr);
  const auto raw_retained = retained->rawPoints();
  ASSERT_EQ(raw_retained.size(), 6ULL);

  EXPECT_EQ(raw_retained[0].x, ptA.x);
  EXPECT_EQ(raw_retained[0].y, ptA.y);
  EXPECT_EQ(raw_retained[0].z, ptA.z);

  EXPECT_TRUE(std::isnan(raw_retained[1].x));

  EXPECT_EQ(raw_retained[2].x, ptB.x);
  EXPECT_EQ(raw_retained[2].y, ptB.y);
  EXPECT_EQ(raw_retained[2].z, ptB.z);

  EXPECT_TRUE(std::isinf(raw_retained[3].y));

  EXPECT_EQ(raw_retained[4].x, ptOutOfRange.x);

  EXPECT_EQ(raw_retained[5].x, ptC.x);
  EXPECT_EQ(raw_retained[5].y, ptC.y);
  EXPECT_EQ(raw_retained[5].z, ptC.z);
}

TEST(RecentObservationCacheStagedTest, SensorOriginNonFiniteFailure) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto stats_before = cache.statistics();

  // 1. NaN in sensor origin
  auto prep_nan = cache.prepareFrame(
      pt, 20, avr::spatial::WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0});
  EXPECT_FALSE(prep_nan.has_value());

  // 2. Inf in sensor origin
  auto prep_inf = cache.prepareFrame(
      pt, 20, avr::spatial::WorldPoint2D{0.0, std::numeric_limits<double>::infinity()});
  EXPECT_FALSE(prep_inf.has_value());

  // Cache completely unchanged
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);
  EXPECT_EQ(cache.statistics(), stats_before);
}

TEST(RecentObservationCacheStagedTest, PointCountLimitPreserved) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Synthetic span exceeding UINT32_MAX without memory allocation
  const size_t overflow_size = static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1ULL;
  const std::span<const PointXYZ> giant_span(static_cast<const PointXYZ*>(nullptr), overflow_size);

  auto prep_opt = cache.prepareFrame(giant_span, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_FALSE(prep_opt.has_value());
  EXPECT_EQ(cache.frameCount(), 0ULL);
}

TEST(RecentObservationCacheStagedTest, DoubleCommitRegressionZeroMutation) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  auto prep_opt = cache.prepareFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  EXPECT_TRUE(prep_opt->isValid());

  // First commit succeeds
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());
  EXPECT_FALSE(prep_opt->isValid());

  const auto stats_after_first = cache.statistics();
  const auto seqs_after_first = cache.frameSequenceSnapshot();

  // Second commit with moved-from prepared object is rejected with ZERO mutation
  auto retry_opt = cache.commitPreparedFrame(std::move(*prep_opt));
  EXPECT_FALSE(retry_opt.has_value());

  EXPECT_EQ(cache.statistics(), stats_after_first);
  EXPECT_EQ(cache.frameSequenceSnapshot(), seqs_after_first);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);
}

TEST(RecentObservationCacheStagedTest, PreparedFrameVsInterveningResetAndNewPush) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Prepare 30
  auto prep30_opt = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep30_opt.has_value());

  // Intervening reset
  cache.reset();
  EXPECT_EQ(cache.frameCount(), 0ULL);

  // Push 40 into reset cache
  ASSERT_TRUE(cache.pushFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);

  // Attempt to commit prepared 30 (30 <= 40 -> rejected as stale)
  auto commit_stale_opt = cache.commitPreparedFrame(std::move(*prep30_opt));
  EXPECT_FALSE(commit_stale_opt.has_value());

  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 1ULL);
  EXPECT_TRUE(cache.containsFrame(40));
  EXPECT_FALSE(cache.containsFrame(30));
}

TEST(RecentObservationCacheStagedTest, MultiplePreparedSequenceMatrix) {
  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  // Case A: Commit 30 then 40 -> both succeed
  {
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
    ASSERT_TRUE(cache_opt.has_value());
    auto& cache = *cache_opt;
    ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    auto prep30 = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
    auto prep40 = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(prep30.has_value());
    ASSERT_TRUE(prep40.has_value());

    ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep30)).has_value());
    ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40)).has_value());
    EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
    EXPECT_EQ(cache.totalFramesPushed(), 3ULL);
  }

  // Case B: Commit 40 then 30 -> 40 succeeds, 30 rejected stale
  {
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
    ASSERT_TRUE(cache_opt.has_value());
    auto& cache = *cache_opt;
    ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    auto prep30 = cache.prepareFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
    auto prep40 = cache.prepareFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(prep30.has_value());
    ASSERT_TRUE(prep40.has_value());

    ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40)).has_value());
    EXPECT_FALSE(cache.commitPreparedFrame(std::move(*prep30)).has_value());
    EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 40}));
    EXPECT_EQ(cache.totalFramesPushed(), 2ULL);
  }
}

TEST(RecentObservationCacheStagedTest, EarlyPushReplayFailureZeroMutationSnapshot) {
  PersistentAdaptiveMap map;
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_opt = map.ensureRootForBase(base_coord);
  ASSERT_TRUE(root_opt.has_value());
  const auto parent_key = *root_opt;

  for (uint64_t f : {10ULL, 20ULL, 30ULL}) {
    const std::vector<PointXYZ> raw = {{0.05, 0.05, 1.0 + 0.1 * static_cast<double>(f)}};
    ASSERT_TRUE(cache.pushFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Early-push frame 40: evicts frame 10 from cache
  const std::vector<PointXYZ> raw40 = {{0.05, 0.05, 5.0}};
  ASSERT_TRUE(cache.pushFrame(raw40, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));

  // Split parent
  ASSERT_TRUE(map.splitLeaf(parent_key));

  // Snapshot before replay attempt
  const auto map_stats_before = map.statistics();
  const auto children = *avr::spatial::childrenOf(parent_key);
  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }
  const auto* retired_cache_before = map.findRetiredCache(parent_key);
  ASSERT_NE(retired_cache_before, nullptr);
  const auto retired_payload_before = *retired_cache_before;

  const auto* coarsening_before = map.findInternalCoarseningState(parent_key);
  ASSERT_NE(coarsening_before, nullptr);
  const auto coarsening_state_before = *coarsening_before;

  // Replay attempt fails conservatively
  const auto replay_result = map.replayDirectChildren(parent_key, cache);
  EXPECT_EQ(replay_result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);

  // Snapshot after failure: ZERO mutation, identical state
  const auto map_stats_after = map.statistics();
  EXPECT_EQ(map_stats_after, map_stats_before);

  for (const auto& child : children) {
    EXPECT_TRUE(map.requiresReplay(child));
    EXPECT_EQ(map.findActivePayload(child), nullptr);
  }

  const auto* retired_cache_after = map.findRetiredCache(parent_key);
  ASSERT_NE(retired_cache_after, nullptr);
  EXPECT_EQ(*retired_cache_after, retired_payload_before);

  const auto* coarsening_after = map.findInternalCoarseningState(parent_key);
  ASSERT_NE(coarsening_after, nullptr);
  EXPECT_EQ(coarsening_after->base_state, coarsening_state_before.base_state);

  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(RecentObservationCacheStagedTest, StagedReplaySuccessStateChanges) {
  PersistentAdaptiveMap map;
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const avr::spatial::GridCoord2D base_coord{0, 0};
  const auto root_opt = map.ensureRootForBase(base_coord);
  ASSERT_TRUE(root_opt.has_value());
  const auto parent_key = *root_opt;

  for (uint64_t f : {10ULL, 20ULL, 30ULL}) {
    const std::vector<PointXYZ> raw = {{0.05, 0.05, 1.0 + 0.1 * static_cast<double>(f)}};
    ASSERT_TRUE(cache.pushFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(raw, f, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
  }

  // Frame 40 prepared ONLY
  const std::vector<PointXYZ> raw40 = {{0.05, 0.05, 5.0}};
  auto prep40_opt = cache.prepareFrame(raw40, 40, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep40_opt.has_value());

  // Split parent
  ASSERT_TRUE(map.splitLeaf(parent_key));

  const auto* retired_cache_before = map.findRetiredCache(parent_key);
  ASSERT_NE(retired_cache_before, nullptr);
  const auto retired_payload_before = *retired_cache_before;
  const auto cache_snapshot_before = cache.frameSequenceSnapshot();

  // Replay succeeds completely
  const auto replay_result = map.replayDirectChildren(parent_key, cache);
  EXPECT_EQ(replay_result.status, ReplayStatus::APPLIED_COMPLETE);
  EXPECT_EQ(replay_result.cached_frames_considered, 3ULL);

  // During/immediately after replay:
  // 1. Parent retired cache remains strictly identical
  const auto* retired_cache_after = map.findRetiredCache(parent_key);
  ASSERT_NE(retired_cache_after, nullptr);
  EXPECT_EQ(*retired_cache_after, retired_payload_before);

  // 2. Cache is completely unchanged during replay
  EXPECT_EQ(cache.frameSequenceSnapshot(), cache_snapshot_before);

  // 3. Child replay flags cleared and payloads active
  const auto children = *avr::spatial::childrenOf(parent_key);
  for (const auto& child : children) {
    EXPECT_FALSE(map.requiresReplay(child));
  }
  EXPECT_TRUE(map.validateInvariants().is_valid);

  // 4. Only committing frame 40 advances cache window
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep40_opt)).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
}

TEST(RecentObservationCacheStagedTest, PreparedFrameSizeAndAlignment) {
  constexpr size_t kSize = sizeof(PreparedCachedObservationFrame);
  constexpr size_t kAlign = alignof(PreparedCachedObservationFrame);

  EXPECT_EQ(kSize, 168ULL);
  EXPECT_EQ(kAlign, 8ULL);
}

}  // namespace
}  // namespace avr::mapping

