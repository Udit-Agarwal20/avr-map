#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_merge_result.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

using namespace avr::mapping;

TEST(PersistentMergeStressTest, OverHundredScenariosConservationAndInvariants) {
  constexpr size_t NUM_SCENARIOS = 120;

  for (size_t s = 0; s < NUM_SCENARIOS; ++s) {
    PersistentAdaptiveMap map{};
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 20});
    ASSERT_TRUE(cache_opt.has_value());
    auto& cache = *cache_opt;

    // Positive and negative roots
    const int32_t gx = static_cast<int32_t>((s % 11) - 5) * 8;
    const int32_t gy = static_cast<int32_t>(((s / 11) % 11) - 5) * 8;

    const double base_x = static_cast<double>(gx) * 0.05;
    const double base_y = static_cast<double>(gy) * 0.05;
    const auto origin = avr::spatial::WorldPoint2D{base_x, base_y};

    const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{gx, gy}, avr::spatial::L3);

    // Scenario parameterization:
    // empty children scenario:
    const bool is_empty_children = (s % 10 == 0);
    // cache eviction scenario:
    const bool is_cache_evicted = (s % 10 == 7);
    // post-replay observations:
    const bool has_post_replay = (s % 3 == 0 && !is_empty_children);

    if (is_empty_children) {
      // Empty merge case
      ASSERT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{gx, gy}, avr::spatial::L2));
      const auto res = map.mergeDirectChildren(root_key, cache);
      EXPECT_EQ(res.status, MergeStatus::APPLIED);
      EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
      EXPECT_EQ(map.findActivePayload(root_key), nullptr);
      const auto val = map.validateInvariants();
      ASSERT_TRUE(val.is_valid) << val.error_message;
      continue;
    }

    // Determine how many frames and which children get observations
    const size_t num_frames = 1 + (s % 4);
    uint64_t total_points = 0;

    for (size_t f = 0; f < num_frames; ++f) {
      const uint64_t frame_seq = 10ULL + static_cast<uint64_t>(f * 10);
      std::vector<PointXYZ> frame_pts;

      // Quadrant offsets:
      // Child 0: +0.05, +0.05
      // Child 1: +0.25, +0.05
      // Child 2: +0.05, +0.25
      // Child 3: +0.25, +0.25
      const double offsets[4][2] = {
          {0.05, 0.05},
          {0.25, 0.05},
          {0.05, 0.25},
          {0.25, 0.25},
      };

      for (size_t c = 0; c < 4; ++c) {
        // Bitmask pattern based on scenario and frame
        if (((s + f + c) % 3) != 0) {
          const double px = base_x + offsets[c][0];
          const double py = base_y + offsets[c][1];
          const double pz = 1.0 + static_cast<double>(s + f + c) * 0.1;
          frame_pts.push_back(PointXYZ{px, py, pz});
        }
      }

      if (frame_pts.empty()) {
        frame_pts.push_back(PointXYZ{base_x + 0.05, base_y + 0.05, 1.0});
      }

      total_points += frame_pts.size();
      ASSERT_TRUE(cache.pushFrame(frame_pts, frame_seq, origin).has_value());
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(frame_pts, frame_seq, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
    }

    // Split parent
    ASSERT_TRUE(map.splitLeaf(root_key));
    const auto replay_res = map.replayDirectChildren(root_key, cache);
    ASSERT_EQ(replay_res.status, ReplayStatus::APPLIED_COMPLETE);

    if (has_post_replay) {
      const uint64_t post_seq = 10ULL + static_cast<uint64_t>(num_frames * 10);
      const std::vector<PointXYZ> post_pts = {{base_x + 0.05, base_y + 0.05, 5.0}};
      ASSERT_TRUE(cache.pushFrame(post_pts, post_seq, origin).has_value());
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(post_pts, post_seq, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
      total_points += 1;
    }

    if (is_cache_evicted) {
      // Evict history by pushing dummy frames elsewhere
      auto tiny_cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 1});
      auto& tiny_cache = *tiny_cache_opt;
      const std::vector<PointXYZ> dummy = {{base_x + 0.05, base_y + 0.05, 1.0}};
      tiny_cache.pushFrame(dummy, 999, origin);

      const auto pre_active = map.activePayloadSnapshot();
      const auto res = map.mergeDirectChildren(root_key, tiny_cache);
      EXPECT_EQ(res.status, MergeStatus::CACHE_HISTORY_INSUFFICIENT);
      EXPECT_EQ(map.activePayloadSnapshot(), pre_active);
      const auto val = map.validateInvariants();
      ASSERT_TRUE(val.is_valid) << val.error_message;
      continue;
    }

    // Execute Merge
    const auto merge_res = map.mergeDirectChildren(root_key, cache);
    EXPECT_EQ(merge_res.status, MergeStatus::APPLIED);
    EXPECT_TRUE(merge_res.isSuccess());

    EXPECT_TRUE(map.hierarchy().isLeaf(root_key));
    EXPECT_EQ(map.findRetiredCache(root_key), nullptr);

    const auto* parent_payload = map.findActivePayload(root_key);
    ASSERT_NE(parent_payload, nullptr);
    EXPECT_EQ(parent_payload->observationCount(), total_points);

    const auto val = map.validateInvariants();
    ASSERT_TRUE(val.is_valid) << val.error_message;
  }
}
