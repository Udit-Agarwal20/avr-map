#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/internal_coarsening_state.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

// ============================================================================
// Section 71 & 72: Deterministic Stress (>= 100 scenarios, >= 1,000 frames)
// ============================================================================

TEST(InternalCoarseningStressTest, OverHundredScenariosAndThousandFrames) {
  constexpr size_t NUM_SCENARIOS = 105;
  constexpr size_t FRAMES_PER_SCENARIO = 12;  // Total frames = 105 * 12 = 1260 frames (> 1000)

  std::mt19937_64 rng(1337);
  std::uniform_int_distribution<int32_t> grid_coord_dist(-20, 20);
  std::uniform_real_distribution<double> local_dist(0.005, 0.040);
  std::uniform_real_distribution<double> elev_dist(-100.0, 100.0);

  size_t total_frames_processed = 0;

  for (size_t scenario = 0; scenario < NUM_SCENARIOS; ++scenario) {
    PersistentAdaptiveMap map{};
    auto cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 4});

    const int32_t gx = grid_coord_dist(rng);
    const int32_t gy = grid_coord_dist(rng);
    const avr::spatial::GridCoord2D grid_base{gx, gy};
    const double world_base_x = static_cast<double>(gx) * avr::spatial::BASE_RESOLUTION_METERS;
    const double world_base_y = static_cast<double>(gy) * avr::spatial::BASE_RESOLUTION_METERS;
    const auto origin = avr::spatial::WorldPoint2D{world_base_x, world_base_y};

    // Ensure root exists
    const auto root_opt = map.ensureRootForBase(grid_base);
    ASSERT_TRUE(root_opt.has_value());
    const auto root = *root_opt;
    ASSERT_TRUE(map.validateInvariants().is_valid);

    // Initial frames into root leaf
    for (uint64_t seq = 1; seq <= 3; ++seq) {
      const std::vector<PointXYZ> pts = {
          {world_base_x + local_dist(rng), world_base_y + local_dist(rng), elev_dist(rng)},
          {world_base_x + local_dist(rng), world_base_y + local_dist(rng), elev_dist(rng)},
      };
      cache.pushFrame(pts, seq, origin);
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
      ASSERT_TRUE(map.validateInvariants().is_valid);
      ++total_frames_processed;
    }

    // Split root (complete retired cache split)
    ASSERT_TRUE(map.splitLeaf(root));
    ASSERT_TRUE(map.validateInvariants().is_valid);

    // Replay direct children from cache
    ASSERT_EQ(map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);
    ASSERT_TRUE(map.validateInvariants().is_valid);

    // Further refine the child containing grid_base down to L1 to test deep ancestors
    const auto child_leaf = *map.hierarchy().findLeafContainingBase(grid_base);
    ASSERT_TRUE(map.splitLeaf(child_leaf));
    ASSERT_TRUE(map.validateInvariants().is_valid);

    // Run post-split frames (seq 4 through 12)
    for (uint64_t seq = 4; seq <= FRAMES_PER_SCENARIO; ++seq) {
      std::vector<PointXYZ> pts;
      const size_t pt_count = 1 + (seq % 5);
      for (size_t p = 0; p < pt_count; ++p) {
        pts.push_back({world_base_x + local_dist(rng), world_base_y + local_dist(rng), elev_dist(rng)});
      }
      cache.pushFrame(pts, seq, origin);
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
      ASSERT_TRUE(map.validateInvariants().is_valid);
      ++total_frames_processed;
    }

    // Replay child_leaf's children if possible so coarsening is eligible
    if (cache.frameCount() >= 3) {
      map.replayDirectChildren(child_leaf, cache);
      ASSERT_TRUE(map.validateInvariants().is_valid);
    }
  }

  EXPECT_GE(total_frames_processed, 1000);
}

// ============================================================================
// Section 72: Long-Run Cache-Independence Stress (>= 100 Frames, Capacity = 3)
// ============================================================================

TEST(InternalCoarseningStressTest, LongRunCacheIndependenceStress) {
  PersistentAdaptiveMap map{};
  auto cache = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  // Initial 2 frames
  for (uint64_t seq = 1; seq <= 2; ++seq) {
    const std::vector<PointXYZ> pts = {{0.05, 0.05, 10.0 + static_cast<double>(seq)}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.validateInvariants().is_valid);
  }

  // Split and replay
  ASSERT_TRUE(map.splitLeaf(root));
  ASSERT_EQ(map.replayDirectChildren(root, cache).status, ReplayStatus::APPLIED_COMPLETE);
  ASSERT_TRUE(map.validateInvariants().is_valid);

  // Run 120 sensor frames while parent remains split
  for (uint64_t seq = 3; seq <= 122; ++seq) {
    const double q = (seq % 2 == 0) ? 0.15 : -0.15;
    const double r = (seq % 4 == 0) ? 0.15 : -0.15;
    const std::vector<PointXYZ> pts = {{q, r, 20.0 + std::cos(static_cast<double>(seq))}};
    cache.pushFrame(pts, seq, origin);
    FrameEvidence ev;
    ASSERT_TRUE(ev.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    ASSERT_TRUE(map.validateInvariants().is_valid);
  }

  // Raw cache only has frames 120, 121, 122
  EXPECT_EQ(cache.frameCount(), 3);
  EXPECT_EQ(cache.oldestFrameSeq().value_or(0), 120);

  // Strict cache merge MUST fail
  const auto strict_res = map.mergeDirectChildren(root, cache);
  EXPECT_EQ(strict_res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);
  ASSERT_TRUE(map.validateInvariants().is_valid);

  // Operational no-cache merge MUST SUCCEED
  const auto merge_res = map.mergeDirectChildren(root);
  EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
  EXPECT_TRUE(merge_res.isSuccess());
  EXPECT_TRUE(map.hierarchy().isLeaf(root));
  EXPECT_NE(map.findActivePayload(root), nullptr);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

// ============================================================================
// Section 73: Deterministic Repeatability Across Independent Maps
// ============================================================================

TEST(InternalCoarseningStressTest, RepeatabilityAcrossIndependentSystems) {
  PersistentAdaptiveMap map1{};
  PersistentAdaptiveMap map2{};

  auto cache1 = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});
  auto cache2 = *RecentObservationCache::create(ReplayCacheConfig{.max_frames = 10});

  const auto origin = avr::spatial::WorldPoint2D{0.0, 0.0};
  const auto root = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);

  for (uint64_t seq = 1; seq <= 5; ++seq) {
    const std::vector<PointXYZ> pts = {{0.05 * static_cast<double>(seq), 0.05, 1.0 + static_cast<double>(seq)}};
    cache1.pushFrame(pts, seq, origin);
    cache2.pushFrame(pts, seq, origin);

    FrameEvidence ev1, ev2;
    ASSERT_TRUE(ev1.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(ev2.buildFrame(pts, seq, origin).has_value());

    ASSERT_TRUE(map1.fuseFrameEvidence(ev1).has_value());
    ASSERT_TRUE(map2.fuseFrameEvidence(ev2).has_value());
  }

  ASSERT_TRUE(map1.splitLeaf(root));
  ASSERT_TRUE(map2.splitLeaf(root));

  ASSERT_EQ(map1.replayDirectChildren(root, cache1).status, ReplayStatus::APPLIED_COMPLETE);
  ASSERT_EQ(map2.replayDirectChildren(root, cache2).status, ReplayStatus::APPLIED_COMPLETE);

  for (uint64_t seq = 6; seq <= 25; ++seq) {
    const std::vector<PointXYZ> pts = {{-0.10, 0.10, 10.0 + static_cast<double>(seq)}};
    cache1.pushFrame(pts, seq, origin);
    cache2.pushFrame(pts, seq, origin);

    FrameEvidence ev1, ev2;
    ASSERT_TRUE(ev1.buildFrame(pts, seq, origin).has_value());
    ASSERT_TRUE(ev2.buildFrame(pts, seq, origin).has_value());

    ASSERT_TRUE(map1.fuseFrameEvidence(ev1).has_value());
    ASSERT_TRUE(map2.fuseFrameEvidence(ev2).has_value());
  }

  // Verify internal coarsening snapshot bit-for-bit equality
  EXPECT_EQ(map1.internalCoarseningSnapshot(), map2.internalCoarseningSnapshot());
  EXPECT_EQ(map1.activePayloadSnapshot(), map2.activePayloadSnapshot());
  EXPECT_EQ(map1.retiredCacheSnapshot(), map2.retiredCacheSnapshot());
  EXPECT_EQ(map1.hierarchy().topologySnapshot(), map2.hierarchy().topologySnapshot());
  EXPECT_EQ(map1.statistics(), map2.statistics());

  // Merge both maps
  ASSERT_EQ(map1.mergeDirectChildren(root).status, MergeStatus::APPLIED);
  ASSERT_EQ(map2.mergeDirectChildren(root).status, MergeStatus::APPLIED);

  EXPECT_EQ(map1.internalCoarseningSnapshot(), map2.internalCoarseningSnapshot());
  EXPECT_EQ(map1.activePayloadSnapshot(), map2.activePayloadSnapshot());
  EXPECT_EQ(map1.hierarchy().topologySnapshot(), map2.hierarchy().topologySnapshot());
  EXPECT_EQ(map1.statistics(), map2.statistics());
}

}  // namespace avr::mapping
