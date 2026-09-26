#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/cached_observation_frame.hpp"
#include "avr_mapping/point_routing.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(RecentObservationCacheRoutingTest, TraitsAndValidation) {
  static_assert(sizeof(PointRouting) == 1);
  static_assert(std::is_trivially_copyable_v<PointRouting>);

  EXPECT_TRUE(isValidPointRouting(PointRouting::STATIC_FUSABLE));
  EXPECT_TRUE(isValidPointRouting(PointRouting::DYNAMIC_EXCLUDED));
  EXPECT_FALSE(isValidPointRouting(static_cast<PointRouting>(2)));
  EXPECT_FALSE(isValidPointRouting(static_cast<PointRouting>(255)));
}

TEST(RecentObservationCacheRoutingTest, MixedRawFrame) {
  // 0: valid static A
  // 1: valid dynamic B
  // 2: NaN point
  // 3: valid static C
  // 4: out-of-range point
  // 5: valid dynamic D
  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},
      {0.10, 0.10, 2.0},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
      {0.15, 0.15, 3.0},
      {1e10, 0.0, 1.0},
      {0.20, 0.20, 4.0},
  };

  const std::vector<PointRouting> routing = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  auto frame_opt = CachedObservationFrame::create(
      raw_points, routing, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(frame_opt.has_value());
  const auto& frame = *frame_opt;

  // Raw points and routing are retained in exact arrival order
  EXPECT_EQ(frame.pointCount(), 6ULL);
  EXPECT_EQ(frame.rawPoints().size(), 6ULL);
  EXPECT_EQ(frame.routing().size(), 6ULL);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(frame.routingAt(static_cast<uint32_t>(i)), routing[i]);
  }
  EXPECT_FALSE(frame.routingAt(6).has_value());

  // Statistics
  const auto& stats = frame.statistics();
  EXPECT_EQ(stats.input_points, 6ULL);
  EXPECT_EQ(stats.replay_candidates, 4ULL);
  EXPECT_EQ(stats.non_finite_unindexed, 1ULL);
  EXPECT_EQ(stats.out_of_range_unindexed, 1ULL);
  EXPECT_EQ(stats.static_fusable_indexed, 2ULL);
  EXPECT_EQ(stats.dynamic_excluded_indexed, 2ULL);

  // Containing L3 region covers [0, 0.40)
  const avr::spatial::GridCoord2D grid_origin{0, 0};
  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(grid_origin, avr::spatial::L3);

  // Generic query returns all 4 geometrically valid candidates (0, 1, 3, 5)
  std::vector<RecentObservationCache::PointIndex> generic_indices;
  EXPECT_TRUE(frame.collectObservationIndices(l3_key, generic_indices));
  EXPECT_EQ(generic_indices, (std::vector<uint32_t>{0, 1, 3, 5}));

  // Static-fusion query returns only static candidates (0, 3)
  std::vector<RecentObservationCache::PointIndex> static_indices;
  EXPECT_TRUE(frame.collectStaticFusableObservationIndices(l3_key, static_indices));
  EXPECT_EQ(static_indices, (std::vector<uint32_t>{0, 3}));
}

TEST(RecentObservationCacheRoutingTest, RoutingSizeMismatch) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},
      {0.10, 0.10, 2.0},
  };
  const std::vector<PointRouting> routing_too_short = {
      PointRouting::STATIC_FUSABLE,
  };
  const std::vector<PointRouting> routing_too_long = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::STATIC_FUSABLE,
  };

  EXPECT_FALSE(cache.prepareFrame(raw_points, routing_too_short, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(cache.prepareFrame(raw_points, routing_too_long, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(cache.pushFrame(raw_points, routing_too_short, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  EXPECT_EQ(cache.frameCount(), 0ULL);
  EXPECT_EQ(cache.totalFramesPushed(), 0ULL);
}

TEST(RecentObservationCacheRoutingTest, InvalidRoutingEnum) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> raw_points = {{0.05, 0.05, 1.0}};
  const std::vector<PointRouting> invalid_routing = {static_cast<PointRouting>(42)};

  EXPECT_FALSE(cache.prepareFrame(raw_points, invalid_routing, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(cache.pushFrame(raw_points, invalid_routing, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_EQ(cache.frameCount(), 0ULL);
}

TEST(RecentObservationCacheRoutingTest, InputRoutingLifetime) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},
      {0.10, 0.10, 2.0},
  };
  std::vector<PointRouting> routing = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  auto prep_opt = cache.prepareFrame(raw_points, routing, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());

  // Mutate original caller routing vector after prepare
  routing[0] = PointRouting::DYNAMIC_EXCLUDED;
  routing[1] = PointRouting::STATIC_FUSABLE;

  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());

  const auto* frame = cache.findFrame(10);
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->routingAt(0), PointRouting::STATIC_FUSABLE);
  EXPECT_EQ(frame->routingAt(1), PointRouting::DYNAMIC_EXCLUDED);
}

TEST(RecentObservationCacheRoutingTest, DefaultOverloadEquivalence) {
  auto cache_legacy_opt = RecentObservationCache::create();
  auto cache_routed_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_legacy_opt.has_value());
  ASSERT_TRUE(cache_routed_opt.has_value());

  const std::vector<PointXYZ> raw_points = {
      {0.05, 0.05, 1.0},
      {0.15, 0.15, 2.0},
  };
  const std::vector<PointRouting> all_static = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
  };

  auto prep_legacy = cache_legacy_opt->prepareFrame(raw_points, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  auto prep_routed = cache_routed_opt->prepareFrame(raw_points, all_static, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_legacy.has_value());
  ASSERT_TRUE(prep_routed.has_value());

  ASSERT_TRUE(cache_legacy_opt->commitPreparedFrame(std::move(*prep_legacy)).has_value());
  ASSERT_TRUE(cache_routed_opt->commitPreparedFrame(std::move(*prep_routed)).has_value());

  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  std::vector<uint32_t> idx_leg_gen, idx_rou_gen, idx_leg_stat, idx_rou_stat;
  EXPECT_TRUE(cache_legacy_opt->collectObservationIndices(10, l3_key, idx_leg_gen));
  EXPECT_TRUE(cache_routed_opt->collectObservationIndices(10, l3_key, idx_rou_gen));
  EXPECT_TRUE(cache_legacy_opt->collectStaticFusableObservationIndices(10, l3_key, idx_leg_stat));
  EXPECT_TRUE(cache_routed_opt->collectStaticFusableObservationIndices(10, l3_key, idx_rou_stat));

  EXPECT_EQ(idx_leg_gen, idx_rou_gen);
  EXPECT_EQ(idx_leg_stat, idx_rou_stat);
  EXPECT_EQ(idx_rou_gen, idx_rou_stat);
}

TEST(RecentObservationCacheRoutingTest, DefaultPushFrameEquivalence) {
  auto cache1_opt = RecentObservationCache::create();
  auto cache2_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache1_opt.has_value());
  ASSERT_TRUE(cache2_opt.has_value());

  const std::vector<PointXYZ> raw_points = {{0.05, 0.05, 1.0}};
  const std::vector<PointRouting> all_static = {PointRouting::STATIC_FUSABLE};

  const auto s1 = cache1_opt->pushFrame(raw_points, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  const auto s2 = cache2_opt->pushFrame(raw_points, all_static, 10, avr::spatial::WorldPoint2D{0.0, 0.0});

  ASSERT_TRUE(s1.has_value());
  ASSERT_TRUE(s2.has_value());
  EXPECT_EQ(*s1, *s2);
}

TEST(RecentObservationCacheRoutingTest, FifoEviction) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  const std::vector<PointRouting> r_static = {PointRouting::STATIC_FUSABLE};
  const std::vector<PointRouting> r_dynamic = {PointRouting::DYNAMIC_EXCLUDED};

  ASSERT_TRUE(cache.pushFrame(pts, r_static, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, r_dynamic, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, r_static, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(cache.pushFrame(pts, r_dynamic, 40, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
  EXPECT_FALSE(cache.containsFrame(10));

  EXPECT_EQ(cache.findFrame(20)->routingAt(0), PointRouting::DYNAMIC_EXCLUDED);
  EXPECT_EQ(cache.findFrame(30)->routingAt(0), PointRouting::STATIC_FUSABLE);
  EXPECT_EQ(cache.findFrame(40)->routingAt(0), PointRouting::DYNAMIC_EXCLUDED);
}

TEST(RecentObservationCacheRoutingTest, FifoAccountingDifferingCounts) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};

  // F10: 2 static, 1 dynamic, 1 NaN (4 raw)
  const std::vector<PointXYZ> f10_pts = {
      {0.05, 0.05, 1.0},
      {0.08, 0.08, 1.2},
      {0.25, 0.25, 2.0},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
  };
  const std::vector<PointRouting> f10_r = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::STATIC_FUSABLE,
  };
  ASSERT_TRUE(cache.pushFrame(f10_pts, f10_r, 10, origin).has_value());

  // F20: 1 static, 2 dynamic (3 raw)
  const std::vector<PointXYZ> f20_pts = {
      {0.05, 0.05, 1.5},
      {0.25, 0.05, 3.0},
      {0.25, 0.25, 4.0},
  };
  const std::vector<PointRouting> f20_r = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
  };
  ASSERT_TRUE(cache.pushFrame(f20_pts, f20_r, 20, origin).has_value());

  // F30: 3 static, 0 dynamic, 1 out-of-range (4 raw)
  const std::vector<PointXYZ> f30_pts = {
      {0.05, 0.05, 1.6},
      {0.08, 0.08, 1.7},
      {0.09, 0.09, 1.8},
      {1e10, 0.0, 0.0},
  };
  const std::vector<PointRouting> f30_r = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
  };
  ASSERT_TRUE(cache.pushFrame(f30_pts, f30_r, 30, origin).has_value());

  // F40: 1 static, 3 dynamic, 1 NaN (5 raw) -> Evicts F10
  const std::vector<PointXYZ> f40_pts = {
      {0.05, 0.05, 2.0},
      {0.25, 0.05, 5.0},
      {0.25, 0.25, 6.0},
      {0.05, 0.25, 7.0},
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
  };
  const std::vector<PointRouting> f40_r = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::STATIC_FUSABLE,
  };
  ASSERT_TRUE(cache.pushFrame(f40_pts, f40_r, 40, origin).has_value());

  // Retained frames: 20, 30, 40
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{20, 30, 40}));
  EXPECT_FALSE(cache.containsFrame(10));

  const auto stats = cache.statistics();
  EXPECT_EQ(stats.stored_frames, 3ULL);
  EXPECT_EQ(stats.total_frames_pushed, 4ULL);
  // Stored static: F20(1) + F30(3) + F40(1) = 5
  EXPECT_EQ(stats.stored_static_fusable, 5ULL);
  // Stored dynamic: F20(2) + F30(0) + F40(3) = 5
  EXPECT_EQ(stats.stored_dynamic_excluded, 5ULL);
  // Stored raw points: F20(3) + F30(4) + F40(5) = 12
  EXPECT_EQ(stats.stored_raw_points, 12ULL);
  EXPECT_EQ(stats.routing_payload_bytes, 12ULL * sizeof(PointRouting));
}

TEST(RecentObservationCacheRoutingTest, StalePreparedCommit) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}};
  const std::vector<PointRouting> r = {PointRouting::STATIC_FUSABLE};

  auto prep20 = cache.prepareFrame(pts, r, 20, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep20.has_value());

  // Intervening commit of frame 30
  ASSERT_TRUE(cache.pushFrame(pts, r, 30, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  // Attempting to commit prepared frame 20 (now stale relative to newest frame 30) fails
  EXPECT_FALSE(cache.commitPreparedFrame(std::move(*prep20)).has_value());
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{30}));
}

TEST(RecentObservationCacheRoutingTest, EmptyFrame) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> empty_pts;
  const std::vector<PointRouting> empty_routing;

  auto prep_opt = cache.prepareFrame(empty_pts, empty_routing, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(prep_opt.has_value());
  ASSERT_TRUE(cache.commitPreparedFrame(std::move(*prep_opt)).has_value());

  EXPECT_EQ(cache.frameCount(), 1ULL);
  EXPECT_EQ(cache.findFrame(10)->pointCount(), 0ULL);
}

TEST(RecentObservationCacheRoutingTest, AllDynamicFrame) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},
      {0.15, 0.15, 2.0},
  };
  const std::vector<PointRouting> all_dynamic = {
      PointRouting::DYNAMIC_EXCLUDED,
      PointRouting::DYNAMIC_EXCLUDED,
  };

  ASSERT_TRUE(cache.pushFrame(pts, all_dynamic, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  std::vector<uint32_t> generic_indices;
  EXPECT_TRUE(cache.collectObservationIndices(10, l3_key, generic_indices));
  EXPECT_EQ(generic_indices.size(), 2ULL);

  std::vector<uint32_t> static_indices;
  EXPECT_TRUE(cache.collectStaticFusableObservationIndices(10, l3_key, static_indices));
  EXPECT_TRUE(static_indices.empty());
}

TEST(RecentObservationCacheRoutingTest, AllStaticFrame) {
  auto cache_opt = RecentObservationCache::create();
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;

  const std::vector<PointXYZ> pts = {
      {0.05, 0.05, 1.0},
      {0.15, 0.15, 2.0},
  };
  const std::vector<PointRouting> all_static = {
      PointRouting::STATIC_FUSABLE,
      PointRouting::STATIC_FUSABLE,
  };

  ASSERT_TRUE(cache.pushFrame(pts, all_static, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto l3_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  std::vector<uint32_t> generic_indices, static_indices;
  EXPECT_TRUE(cache.collectObservationIndices(10, l3_key, generic_indices));
  EXPECT_TRUE(cache.collectStaticFusableObservationIndices(10, l3_key, static_indices));
  EXPECT_EQ(generic_indices, static_indices);
  EXPECT_EQ(static_indices.size(), 2ULL);
}

}  // namespace
}  // namespace avr::mapping
