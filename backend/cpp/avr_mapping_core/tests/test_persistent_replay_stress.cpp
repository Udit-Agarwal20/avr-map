#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_reconstruction_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(PersistentReplayStressTest, OverHundredScenariosConservationAndInvariants) {
  constexpr size_t NUM_SCENARIOS = 120;
  size_t complete_applied_count = 0;
  size_t incomplete_tested_count = 0;

  for (size_t i = 0; i < NUM_SCENARIOS; ++i) {
    PersistentAdaptiveMap map{};
    const bool is_incomplete_case = (i % 6 == 5);

    // Large cache for complete cases; small cache for incomplete cases
    const size_t cache_capacity = is_incomplete_case ? 1 : 20;
    auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = cache_capacity});
    ASSERT_TRUE(cache_opt.has_value());
    auto& cache = *cache_opt;

    // Pick root coordinates: positive when i < 60, negative when i >= 60
    const int32_t rx = (i < 60) ? static_cast<int32_t>(i % 6) : -1 - static_cast<int32_t>(i % 6);
    const int32_t ry = (i < 60) ? static_cast<int32_t>((i / 6) % 6) : -1 - static_cast<int32_t>((i / 6) % 6);

    // Base origin for root in world coordinates
    // Root scale is 8 base cells, base cell is 0.05m -> root span is 0.40m
    const double root_x0 = static_cast<double>(rx) * 0.40;
    const double root_y0 = static_cast<double>(ry) * 0.40;
    const auto origin = avr::spatial::WorldPoint2D{root_x0, root_y0};

    const auto root_key = *avr::spatial::CellKey::fromWorldPoint(
        avr::spatial::WorldPoint2D{root_x0 + 0.1, root_y0 + 0.1}, avr::spatial::L3);

    // Subdivide into 4 child offsets (dx, dy):
    // Child 0: [0.00, 0.20) x [0.00, 0.20) -> center (0.05, 0.05)
    // Child 1: [0.20, 0.40) x [0.00, 0.20) -> center (0.25, 0.05)
    // Child 2: [0.00, 0.20) x [0.20, 0.40) -> center (0.05, 0.25)
    // Child 3: [0.20, 0.40) x [0.20, 0.40) -> center (0.25, 0.25)

    const size_t pattern = i % 6;
    std::vector<std::vector<PointXYZ>> historical_frames;

    // Frame 10 points
    std::vector<PointXYZ> f10;
    if (pattern == 1) {
      // Child 0 only
      f10.push_back(PointXYZ{root_x0 + 0.05, root_y0 + 0.05, 1.0 + 0.01 * static_cast<double>(i)});
      f10.push_back(PointXYZ{root_x0 + 0.08, root_y0 + 0.08, 1.1 + 0.01 * static_cast<double>(i)});
    } else if (pattern == 2) {
      // Children 0 and 2
      f10.push_back(PointXYZ{root_x0 + 0.05, root_y0 + 0.05, 1.0});
      f10.push_back(PointXYZ{root_x0 + 0.05, root_y0 + 0.25, 2.0});
    } else if (pattern == 3) {
      // Children 1 and 3
      f10.push_back(PointXYZ{root_x0 + 0.25, root_y0 + 0.05, 1.5});
      f10.push_back(PointXYZ{root_x0 + 0.25, root_y0 + 0.25, 2.5});
    } else {
      // All 4 children
      f10.push_back(PointXYZ{root_x0 + 0.05, root_y0 + 0.05, 1.0});
      f10.push_back(PointXYZ{root_x0 + 0.25, root_y0 + 0.05, 1.5});
      f10.push_back(PointXYZ{root_x0 + 0.05, root_y0 + 0.25, 2.0});
      f10.push_back(PointXYZ{root_x0 + 0.25, root_y0 + 0.25, 2.5});
    }
    historical_frames.push_back(f10);

    // Frame 20 points
    std::vector<PointXYZ> f20;
    f20.push_back(PointXYZ{root_x0 + 0.06, root_y0 + 0.06, 1.2});
    historical_frames.push_back(f20);

    // Frame 30 points
    std::vector<PointXYZ> f30;
    f30.push_back(PointXYZ{root_x0 + 0.26, root_y0 + 0.06, 1.8});
    historical_frames.push_back(f30);

    // Ingest historical frames into cache and map
    uint64_t seq = 10;
    for (const auto& frame_pts : historical_frames) {
      ASSERT_TRUE(cache.pushFrame(frame_pts, seq, origin).has_value());
      FrameEvidence ev;
      ASSERT_TRUE(ev.buildFrame(frame_pts, seq, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(ev).has_value());
      seq += 10;
    }

    // Split parent L3 -> 4 L2 children
    ASSERT_TRUE(map.splitLeaf(root_key));

    // For pattern == 4, add post-split frame into child 0 and child 2
    if (pattern == 4) {
      const std::vector<PointXYZ> post_split_pts = {
          {root_x0 + 0.05, root_y0 + 0.05, 3.5},
          {root_x0 + 0.05, root_y0 + 0.25, 4.5},
      };
      ASSERT_TRUE(cache.pushFrame(post_split_pts, 50, origin).has_value());
      FrameEvidence post_ev;
      ASSERT_TRUE(post_ev.buildFrame(post_split_pts, 50, origin).has_value());
      ASSERT_TRUE(map.fuseFrameEvidence(post_ev).has_value());
    }

    // Execute direct-parent replay
    const auto result = map.replayDirectChildren(root_key, cache);

    if (is_incomplete_case) {
      EXPECT_EQ(result.status, ReplayStatus::CACHE_HISTORY_INSUFFICIENT);
      EXPECT_FALSE(result.isSuccess());
      ++incomplete_tested_count;
    } else {
      ASSERT_EQ(result.status, ReplayStatus::APPLIED_COMPLETE) << "Failed on scenario " << i;
      EXPECT_TRUE(result.isSuccess());

      // Conservation verification (Section 51)
      EXPECT_EQ(result.historical_observations_distributed, result.parent_expected_observations);

      // Verify all 4 children are cleared of REPLAY_REQUIRED
      const auto children = *avr::spatial::childrenOf(root_key);
      uint64_t child_hist_obs_sum = 0;
      for (const auto& child : children) {
        EXPECT_FALSE(map.requiresReplay(child));
        const auto* cp = map.findActivePayload(child);
        if (cp != nullptr) {
          // If this child had post-split payload in pattern 4, discount frame 50's observation
          if (pattern == 4 && (child == children[0] || child == children[2])) {
            EXPECT_EQ(cp->lastSeenFrame(), 50ULL);
            child_hist_obs_sum += (cp->observationCount() - 1);
          } else {
            child_hist_obs_sum += cp->observationCount();
          }
        }
      }
      EXPECT_EQ(child_hist_obs_sum, result.parent_expected_observations);
      ++complete_applied_count;

      // Requirement 31: Stress Repeated Calls
      // Capture snapshots after first replay
      const auto active_snap1 = map.activePayloadSnapshot();
      const auto retired_snap1 = map.retiredCacheSnapshot();
      const auto replay_snap1 = map.replayRequiredSnapshot();
      const auto stats1 = map.statistics();

      // Invoke second replay: must return ALREADY_COMPLETE with zero mutation
      const auto res2 = map.replayDirectChildren(root_key, cache);
      EXPECT_EQ(res2.status, ReplayStatus::ALREADY_COMPLETE) << "Double replay failed on scenario " << i;
      EXPECT_FALSE(res2.isSuccess());

      EXPECT_EQ(map.activePayloadSnapshot(), active_snap1);
      EXPECT_EQ(map.retiredCacheSnapshot(), retired_snap1);
      EXPECT_EQ(map.replayRequiredSnapshot(), replay_snap1);
      const auto stats2 = map.statistics();
      EXPECT_EQ(stats2.payload_stats.active_payload_count, stats1.payload_stats.active_payload_count);
      EXPECT_EQ(stats2.payload_stats.active_observation_count, stats1.payload_stats.active_observation_count);
      EXPECT_EQ(stats2.payload_stats.retired_cache_count, stats1.payload_stats.retired_cache_count);
      EXPECT_EQ(stats2.payload_stats.retired_cache_observation_count, stats1.payload_stats.retired_cache_observation_count);
      EXPECT_EQ(stats2.payload_stats.replay_required_count, stats1.payload_stats.replay_required_count);
    }

    // Verify cross-store invariants after every single scenario
    const auto val = map.validateInvariants();
    ASSERT_TRUE(val.is_valid) << "Invariant failed on scenario " << i << ": " << val.error_message;
  }

  // Ensure minimum 100 complete scenarios and diverse coverage
  EXPECT_GE(complete_applied_count, 100ULL);
  EXPECT_GT(incomplete_tested_count, 15ULL);
}

}  // namespace
}  // namespace avr::mapping
