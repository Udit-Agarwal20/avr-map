#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

#include "avr_mapping/cached_observation_frame.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(RecentObservationCacheTest, ConfigurationValidationAndDefaults) {
  // 1. max_frames = 0 is rejected explicitly (not silently clamped)
  EXPECT_FALSE(RecentObservationCache::create(ReplayCacheConfig{.max_frames = 0}).has_value());

  // 2. max_frames = 1 is valid
  auto one_cache = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 1});
  ASSERT_TRUE(one_cache.has_value());
  EXPECT_EQ(one_cache->config().max_frames, 1ULL);

  // 3. Default configuration: valid default max_frames == 3
  auto default_cache = RecentObservationCache::create();
  ASSERT_TRUE(default_cache.has_value());
  EXPECT_EQ(default_cache->config().max_frames, 3ULL);
  EXPECT_TRUE(default_cache->config().isValid());

  // 4. max_frames = 5 is valid
  auto five_cache = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(five_cache.has_value());
  EXPECT_EQ(five_cache->config().max_frames, 5ULL);

  // 5. No arbitrary maximum enforced
  auto large_cache = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 1000});
  ASSERT_TRUE(large_cache.has_value());
  EXPECT_EQ(large_cache->config().max_frames, 1000ULL);
}

TEST(RecentObservationCacheTest, DefaultConfigGoldenTest) {
  // Lock: ReplayCacheConfig{}.max_frames == 3 (configurable prototype default; not claimed optimal)
  constexpr ReplayCacheConfig default_cfg{};
  EXPECT_EQ(default_cfg.max_frames, 3ULL);
  EXPECT_TRUE(default_cfg.isValid());
}

TEST(RecentObservationCacheTest, OneFrameRetentionTest) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 1});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL}));

  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20ULL}));
  EXPECT_FALSE(cache.containsFrame(10));

  ASSERT_TRUE(cache.pushFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{30ULL}));
  EXPECT_FALSE(cache.containsFrame(20));
  EXPECT_EQ(cache.frameCount(), 1ULL);
}

TEST(RecentObservationCacheTest, FiveFrameRetentionTest) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  for (uint64_t seq : {10ULL, 20ULL, 30ULL, 40ULL, 50ULL, 60ULL}) {
    ASSERT_TRUE(cache.pushFrame(pt, seq, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  }

  EXPECT_EQ(cache.frameCount(), 5ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20ULL, 30ULL, 40ULL, 50ULL, 60ULL}));
  EXPECT_FALSE(cache.containsFrame(10));
}

TEST(RecentObservationCacheTest, CompileTimeOwnershipContract) {
  // RecentObservationCache: non-copyable, movable
  static_assert(!std::is_copy_constructible_v<RecentObservationCache>);
  static_assert(!std::is_copy_assignable_v<RecentObservationCache>);
  static_assert(std::is_move_constructible_v<RecentObservationCache>);
  static_assert(std::is_move_assignable_v<RecentObservationCache>);

  // CachedObservationFrame: non-copyable, movable
  static_assert(!std::is_copy_constructible_v<CachedObservationFrame>);
  static_assert(!std::is_copy_assignable_v<CachedObservationFrame>);
  static_assert(std::is_move_constructible_v<CachedObservationFrame>);
  static_assert(std::is_move_assignable_v<CachedObservationFrame>);
}

TEST(RecentObservationCacheTest, LayoutContract) {
  static_assert(sizeof(PointXYZ) == 24);
  static_assert(sizeof(CachedObservationFrame::PointIndex) == 4);
  std::cout << "[LayoutContract] sizeof(PointXYZ) = " << sizeof(PointXYZ) << std::endl;
  std::cout << "[LayoutContract] sizeof(PointIndex) = " << sizeof(CachedObservationFrame::PointIndex) << std::endl;
  std::cout << "[LayoutContract] sizeof(CachedObservationFrame) = " << sizeof(CachedObservationFrame) << std::endl;
  std::cout << "[LayoutContract] alignof(CachedObservationFrame) = " << alignof(CachedObservationFrame) << std::endl;
  EXPECT_GT(sizeof(CachedObservationFrame), 0ULL);
}

TEST(RecentObservationCacheTest, InitialStateIsEmpty) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  EXPECT_EQ(cache.frameCount(), 0ULL);
  EXPECT_FALSE(cache.oldestFrameSeq().has_value());
  EXPECT_FALSE(cache.newestFrameSeq().has_value());
  EXPECT_TRUE(cache.frameSequenceSnapshot().empty());

  const auto stats = cache.statistics();
  EXPECT_EQ(stats.stored_frames, 0ULL);
  EXPECT_EQ(stats.stored_raw_points, 0ULL);
  EXPECT_EQ(stats.stored_replay_candidates, 0ULL);
  EXPECT_EQ(stats.stored_bucket_count, 0ULL);
  EXPECT_EQ(stats.stored_point_indices, 0ULL);
  EXPECT_FALSE(stats.oldest_frame_seq.has_value());
  EXPECT_FALSE(stats.newest_frame_seq.has_value());
  EXPECT_EQ(stats.raw_point_payload_bytes, 0ULL);
  EXPECT_EQ(stats.index_payload_bytes, 0ULL);
}

TEST(RecentObservationCacheTest, ValidEmptyFrameLifecycle) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> empty_pts{};

  const auto stats_opt = cache.pushFrame(empty_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  EXPECT_EQ(stats_opt->frame_seq, 10ULL);
  EXPECT_EQ(stats_opt->input_points, 0ULL);
  EXPECT_EQ(stats_opt->replay_candidates, 0ULL);
  EXPECT_EQ(stats_opt->bucket_count, 0ULL);
  EXPECT_EQ(stats_opt->point_indices_count, 0ULL);

  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 10ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL}));

  const auto* frame = cache.findFrame(10);
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->frameSeq(), 10ULL);
  EXPECT_EQ(frame->pointCount(), 0ULL);
  EXPECT_EQ(frame->bucketCount(), 0ULL);
}

TEST(RecentObservationCacheTest, MonotonicFrameOrderingContract) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.1, 0.1, 1.0}};

  EXPECT_TRUE(cache.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_TRUE(cache.pushFrame(pts, 42, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Duplicate frame 42 rejected without mutation
  EXPECT_FALSE(cache.pushFrame(pts, 42, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL, 42ULL}));

  // Out-of-order frame 20 rejected without mutation
  EXPECT_FALSE(cache.pushFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL, 42ULL}));

  // Monotonic increase succeeds
  EXPECT_TRUE(cache.pushFrame(pts, 105, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL, 42ULL, 105ULL}));
}

TEST(RecentObservationCacheTest, InvalidSensorOriginRejected) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.1, 0.1, 1.0}};

  EXPECT_FALSE(cache.pushFrame(
      pts, 1, avr::spatial::WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0}).has_value());
  EXPECT_FALSE(cache.pushFrame(
      pts, 1, avr::spatial::WorldPoint2D{0.0, std::numeric_limits<double>::infinity()}).has_value());
  EXPECT_FALSE(cache.pushFrame(
      pts, 1, avr::spatial::WorldPoint2D{-std::numeric_limits<double>::infinity(), 0.0}).has_value());

  EXPECT_EQ(cache.frameCount(), 0ULL);
}

TEST(RecentObservationCacheTest, RawPointsRetentionAndPartitionCounters) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> input_pts = {
      {0.05, 0.05, 1.0},                                           // valid candidate
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},        // non-finite x
      {0.10, 0.10, 2.0},                                           // valid candidate
      {0.0, std::numeric_limits<double>::infinity(), 3.0},         // non-finite y
      {1e20, 0.0, 4.0},                                            // out-of-range
      {0.15, 0.15, std::numeric_limits<double>::quiet_NaN()},      // non-finite z
      {0.20, 0.20, 5.0},                                           // valid candidate
  };

  const auto stats_opt = cache.pushFrame(input_pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());

  // Conservation of raw point input categories
  EXPECT_EQ(stats_opt->input_points, 7ULL);
  EXPECT_EQ(stats_opt->replay_candidates, 3ULL);
  EXPECT_EQ(stats_opt->non_finite_unindexed, 3ULL);
  EXPECT_EQ(stats_opt->out_of_range_unindexed, 1ULL);
  EXPECT_EQ(
      stats_opt->input_points,
      stats_opt->replay_candidates + stats_opt->non_finite_unindexed + stats_opt->out_of_range_unindexed);

  // All 7 elements remain stored in rawPoints in exact input order
  const auto* frame = cache.findFrame(1);
  ASSERT_NE(frame, nullptr);
  ASSERT_EQ(frame->rawPoints().size(), 7ULL);
  for (size_t i = 0; i < input_pts.size(); ++i) {
    if (std::isnan(input_pts[i].x)) {
      EXPECT_TRUE(std::isnan(frame->rawPoints()[i].x));
    } else {
      EXPECT_DOUBLE_EQ(frame->rawPoints()[i].x, input_pts[i].x);
    }
    if (std::isnan(input_pts[i].y)) {
      EXPECT_TRUE(std::isnan(frame->rawPoints()[i].y));
    } else if (std::isinf(input_pts[i].y)) {
      EXPECT_TRUE(std::isinf(frame->rawPoints()[i].y));
    } else {
      EXPECT_DOUBLE_EQ(frame->rawPoints()[i].y, input_pts[i].y);
    }
    if (std::isnan(input_pts[i].z)) {
      EXPECT_TRUE(std::isnan(frame->rawPoints()[i].z));
    } else {
      EXPECT_DOUBLE_EQ(frame->rawPoints()[i].z, input_pts[i].z);
    }
  }

  EXPECT_EQ(stats_opt->point_indices_count, 3ULL);
}

TEST(RecentObservationCacheTest, ExtremeFiniteZRemainsReplayCandidate) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1e200},
  };

  const auto stats_opt = cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(stats_opt.has_value());
  EXPECT_EQ(stats_opt->input_points, 1ULL);
  EXPECT_EQ(stats_opt->replay_candidates, 1ULL);
  EXPECT_EQ(stats_opt->non_finite_unindexed, 0ULL);

  const auto* frame = cache.findFrame(1);
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->bucketCount(), 1ULL);
}

TEST(RecentObservationCacheTest, FifoRetentionAndOldestEviction) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL, 20ULL, 30ULL}));
  EXPECT_EQ(cache.oldestFrameSeq(), 10ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 30ULL);

  // Push frame 40: evicts frame 10
  ASSERT_TRUE(cache.pushFrame(pt, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20ULL, 30ULL, 40ULL}));
  EXPECT_EQ(cache.oldestFrameSeq(), 20ULL);
  EXPECT_EQ(cache.newestFrameSeq(), 40ULL);

  // Frame 10 is no longer accessible
  EXPECT_FALSE(cache.containsFrame(10));
  EXPECT_EQ(cache.findFrame(10), nullptr);

  // Push frame 50: evicts frame 20
  ASSERT_TRUE(cache.pushFrame(pt, 50, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{30ULL, 40ULL, 50ULL}));
  EXPECT_FALSE(cache.containsFrame(20));
}

TEST(RecentObservationCacheTest, FailedPushPreservesExistingOldestFrame) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};

  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Borrow read-only pointer to frame 10
  const auto* borrowed_10 = cache.findFrame(10);
  ASSERT_NE(borrowed_10, nullptr);
  EXPECT_EQ(borrowed_10->frameSeq(), 10ULL);
  EXPECT_EQ(borrowed_10->pointCount(), 1ULL);

  // Attempt invalid push with non-finite sensor origin (fails in PREPARE phase)
  EXPECT_FALSE(cache.pushFrame(
      pt, 40, avr::spatial::WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0}).has_value());

  // Cache state completely preserved: frame 10 NOT evicted
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{10ULL, 20ULL, 30ULL}));
  EXPECT_TRUE(cache.containsFrame(10));
  EXPECT_TRUE(cache.containsFrame(20));
  EXPECT_TRUE(cache.containsFrame(30));

  // Frame 10 logical content unchanged
  const auto* post_fail_10 = cache.findFrame(10);
  ASSERT_NE(post_fail_10, nullptr);
  EXPECT_EQ(post_fail_10->frameSeq(), 10ULL);
  EXPECT_EQ(post_fail_10->pointCount(), 1ULL);
}

TEST(RecentObservationCacheTest, QueryFailureContractAndSentinels) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto valid_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto absent_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{100, 100}, avr::spatial::L3);
  const avr::spatial::CellKey invalid_level_key{.morton = 0, .level = 4};

  // Case 1: Missing frame -> returns false, clears output
  std::vector<uint32_t> out1 = {999, 888};
  EXPECT_FALSE(cache.collectObservationIndices(99, valid_l3, out1));
  EXPECT_TRUE(out1.empty());

  // Case 2: Invalid region level -> returns false, clears output
  std::vector<uint32_t> out2 = {999, 888};
  EXPECT_FALSE(cache.collectObservationIndices(10, invalid_level_key, out2));
  EXPECT_TRUE(out2.empty());

  // Case 3: Valid region with no bucket -> returns true, clears output
  std::vector<uint32_t> out3 = {999, 888};
  EXPECT_TRUE(cache.collectObservationIndices(10, absent_l3, out3));
  EXPECT_TRUE(out3.empty());
}

TEST(RecentObservationCacheTest, EvictedFrameQuery) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto valid_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Frame 10 is evicted
  EXPECT_EQ(cache.findFrame(10), nullptr);
  std::vector<uint32_t> out = {42};
  EXPECT_FALSE(cache.collectObservationIndices(10, valid_l3, out));
  EXPECT_TRUE(out.empty());

  // Frames 20, 30, 40 remain functional
  for (uint64_t seq : {20ULL, 30ULL, 40ULL}) {
    EXPECT_NE(cache.findFrame(seq), nullptr);
    std::vector<uint32_t> indices;
    EXPECT_TRUE(cache.collectObservationIndices(seq, valid_l3, indices));
    EXPECT_EQ(indices.size(), 1ULL);
    EXPECT_EQ(indices[0], 0U);
  }
}

TEST(RecentObservationCacheTest, ResetClearsFramesAndPreservesConfig) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 5});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pt = {{0.05, 0.05, 1.0}};
  ASSERT_TRUE(cache.pushFrame(pt, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pt, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 2ULL);

  cache.reset();

  EXPECT_EQ(cache.frameCount(), 0ULL);
  EXPECT_EQ(cache.config().max_frames, 5ULL);  // preserved
  EXPECT_FALSE(cache.oldestFrameSeq().has_value());
  EXPECT_FALSE(cache.newestFrameSeq().has_value());
  EXPECT_TRUE(cache.frameSequenceSnapshot().empty());
  EXPECT_FALSE(cache.containsFrame(10));
  EXPECT_EQ(cache.findFrame(10), nullptr);

  const auto valid_l3 = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  std::vector<uint32_t> out = {77};
  EXPECT_FALSE(cache.collectObservationIndices(10, valid_l3, out));
  EXPECT_TRUE(out.empty());

  // Re-use after reset is completely clean
  ASSERT_TRUE(cache.pushFrame(pt, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.oldestFrameSeq(), 1ULL);
}

TEST(RecentObservationCacheTest, BucketAndRegionStrictAscendingOrdering) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Add 10 points in the same L3 bucket (at grid 0, 0)
  std::vector<PointXYZ> pts;
  for (int i = 0; i < 10; ++i) {
    pts.push_back(PointXYZ{0.05 + i * 0.01, 0.05, static_cast<double>(i)});
  }

  ASSERT_TRUE(cache.pushFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto* frame = cache.findFrame(1);
  ASSERT_NE(frame, nullptr);

  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto bucket_indices = frame->findBucketIndices(l3_key);
  ASSERT_EQ(bucket_indices.size(), 10ULL);

  // Invariant: bucket stored indices are strictly increasing
  for (size_t i = 1; i < bucket_indices.size(); ++i) {
    EXPECT_GT(bucket_indices[i], bucket_indices[i - 1]);
  }

  // Invariant: region query result is strictly increasing
  std::vector<uint32_t> region_out;
  EXPECT_TRUE(cache.collectObservationIndices(1, l3_key, region_out));
  ASSERT_EQ(region_out.size(), 10ULL);
  for (size_t i = 1; i < region_out.size(); ++i) {
    EXPECT_GT(region_out[i], region_out[i - 1]);
  }
}

TEST(RecentObservationCacheTest, FrameStatsPartitionAndAggregateDrop) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 2});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  // Frame 1: Normal frame with 3 valid, 1 NaN, 1 out of range
  const std::vector<PointXYZ> f1_pts = {
      {0.05, 0.05, 1.0},
      {0.15, 0.15, 2.0},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      {1e20, 1e20, 0.0},
      {0.25, 0.25, 3.0},
  };
  const auto f1_stats_opt = cache.pushFrame(f1_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(f1_stats_opt.has_value());
  EXPECT_EQ(f1_stats_opt->input_points, 5ULL);
  EXPECT_EQ(f1_stats_opt->replay_candidates, 3ULL);
  EXPECT_EQ(f1_stats_opt->non_finite_unindexed, 1ULL);
  EXPECT_EQ(f1_stats_opt->out_of_range_unindexed, 1ULL);
  EXPECT_EQ(f1_stats_opt->input_points,
            f1_stats_opt->replay_candidates + f1_stats_opt->non_finite_unindexed + f1_stats_opt->out_of_range_unindexed);

  // Frame 2: All-invalid frame
  const std::vector<PointXYZ> f2_pts = {
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      {0.0, std::numeric_limits<double>::infinity(), 0.0},
      {1e25, 0.0, 0.0},
  };
  const auto f2_stats_opt = cache.pushFrame(f2_pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(f2_stats_opt.has_value());
  EXPECT_EQ(f2_stats_opt->input_points, 3ULL);
  EXPECT_EQ(f2_stats_opt->replay_candidates, 0ULL);
  EXPECT_EQ(f2_stats_opt->non_finite_unindexed, 2ULL);
  EXPECT_EQ(f2_stats_opt->out_of_range_unindexed, 1ULL);
  EXPECT_EQ(f2_stats_opt->input_points,
            f2_stats_opt->replay_candidates + f2_stats_opt->non_finite_unindexed + f2_stats_opt->out_of_range_unindexed);

  // Aggregate stats across f1 and f2
  auto agg = cache.statistics();
  EXPECT_EQ(agg.stored_frames, 2ULL);
  EXPECT_EQ(agg.stored_raw_points, 8ULL);
  EXPECT_EQ(agg.stored_replay_candidates, 3ULL);
  EXPECT_EQ(agg.stored_point_indices, 3ULL);

  // Frame 3: Valid single point; evicts Frame 1
  const std::vector<PointXYZ> f3_pts = {{1.05, 1.05, 4.0}};
  const auto f3_stats_opt = cache.pushFrame(f3_pts, 30, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(f3_stats_opt.has_value());

  // Frame 1 counts must disappear immediately from aggregate telemetry
  agg = cache.statistics();
  EXPECT_EQ(agg.stored_frames, 2ULL);
  EXPECT_EQ(agg.stored_raw_points, f2_stats_opt->input_points + f3_stats_opt->input_points);
  EXPECT_EQ(agg.stored_replay_candidates, f2_stats_opt->replay_candidates + f3_stats_opt->replay_candidates);
  EXPECT_EQ(agg.stored_point_indices, agg.stored_replay_candidates);
  EXPECT_EQ(agg.oldest_frame_seq, 20ULL);
  EXPECT_EQ(agg.newest_frame_seq, 30ULL);
}

TEST(RecentObservationCacheTest, SaturatingArithmeticHelpers) {
  // Test saturating multiplication on limits
  constexpr size_t MAX_SIZE = std::numeric_limits<size_t>::max();
  EXPECT_EQ(saturatingMultiply(0, MAX_SIZE), 0ULL);
  EXPECT_EQ(saturatingMultiply(MAX_SIZE, 0), 0ULL);
  EXPECT_EQ(saturatingMultiply(MAX_SIZE, 1), MAX_SIZE);
  EXPECT_EQ(saturatingMultiply(MAX_SIZE, 2), MAX_SIZE);
  EXPECT_EQ(saturatingMultiply(100, 200), 20000ULL);

  // Test saturating addition on limits
  EXPECT_EQ(saturatingAdd(MAX_SIZE - 5, 10), MAX_SIZE);
  EXPECT_EQ(saturatingAdd(100, 200), 300ULL);

  constexpr uint64_t MAX_U64 = std::numeric_limits<uint64_t>::max();
  EXPECT_EQ(saturatingAddU64(MAX_U64 - 5, 10), MAX_U64);
  EXPECT_EQ(saturatingAddU64(100, 200), 300ULL);
}

TEST(RecentObservationCacheTest, CachedObservationFrameCreationValidation) {
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};

  // 1. Non-finite sensor origin rejected
  EXPECT_FALSE(CachedObservationFrame::create(
      pts, 1, avr::spatial::WorldPoint2D{std::numeric_limits<double>::quiet_NaN(), 0.0}).has_value());
  EXPECT_FALSE(CachedObservationFrame::create(
      pts, 1, avr::spatial::WorldPoint2D{0.0, std::numeric_limits<double>::infinity()}).has_value());

  // 2. Point count > UINT32_MAX rejected without memory allocation
  // Construct a theoretical non-dereferenced span with size > UINT32_MAX
  const size_t overflow_size = static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1ULL;
  const std::span<const PointXYZ> giant_span(static_cast<const PointXYZ*>(nullptr), overflow_size);
  EXPECT_FALSE(CachedObservationFrame::create(
      giant_span, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
}

TEST(RecentObservationCacheTest, RepeatabilityAcrossIndependentInstances) {
  auto cache1_opt = RecentObservationCache::create();
  auto cache2_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache1_opt.has_value());
  ASSERT_TRUE(cache2_opt.has_value());

  auto& cache1 = *cache1_opt;
  auto& cache2 = *cache2_opt;

  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},
      {-1.05, 2.05, 3.0},
  };

  ASSERT_TRUE(cache1.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache2.pushFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  EXPECT_EQ(cache1.frameSequenceSnapshot(), cache2.frameSequenceSnapshot());
  EXPECT_EQ(cache1.statistics(), cache2.statistics());
  EXPECT_EQ(cache1.bucketKeySnapshot(10), cache2.bucketKeySnapshot(10));
}

}  // namespace
}  // namespace avr::mapping
