#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {
namespace {

// Deterministic 64-bit Linear Congruential Generator
class StressRng {
 public:
  explicit StressRng(uint64_t seed) : state_(seed) {}

  uint64_t next() noexcept {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return state_;
  }

  double uniform(double min_val, double max_val) noexcept {
    const double normalized = static_cast<double>(next() >> 11) / 9007199254740992.0;
    return min_val + normalized * (max_val - min_val);
  }

 private:
  uint64_t state_;
};

TEST(RecentObservationCacheStressTest, TenFramesOneMillionPointsBoundedMemory) {
  auto cache_opt = RecentObservationCache::create(ReplayCacheConfig{.max_frames = 3});
  ASSERT_TRUE(cache_opt.has_value());
  auto& cache = *cache_opt;
  StressRng rng(0x12345678DEADBEEFULL);

  constexpr size_t TOTAL_FRAMES = 11;
  constexpr size_t POINTS_PER_FRAME = 100000;
  uint64_t grand_total_input = 0;

  for (size_t f = 1; f <= TOTAL_FRAMES; ++f) {
    const uint64_t frame_seq = f * 10;
    std::vector<PointXYZ> points;

    if (f == 5) {
      // Empty frame in the sequence to test valid empty frame handling
      // points remains empty
    } else {
      points.reserve(POINTS_PER_FRAME);
      for (size_t p = 0; p < POINTS_PER_FRAME; ++p) {
        // Periodically inject invalid points
        if (p % 1000 == 0) {
          // NaN
          points.push_back(PointXYZ{
              std::numeric_limits<double>::quiet_NaN(),
              rng.uniform(-10.0, 10.0),
              0.0,
          });
        } else if (p % 2000 == 1) {
          // Infinity
          points.push_back(PointXYZ{
              rng.uniform(-10.0, 10.0),
              std::numeric_limits<double>::infinity(),
              0.0,
          });
        } else if (p % 5000 == 2) {
          // Extreme out of range
          points.push_back(PointXYZ{1e25, -1e25, 0.0});
        } else {
          // Valid points spanning positive and negative space [-12.0, +12.0] m
          const double x = rng.uniform(-12.0, 12.0);
          const double y = rng.uniform(-12.0, 12.0);
          const double z = rng.uniform(-3.0, 5.0);
          points.push_back(PointXYZ{x, y, z});
        }
      }
    }

    grand_total_input += points.size();

    const auto stats_opt = cache.pushFrame(points, frame_seq, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(stats_opt.has_value());
    EXPECT_EQ(stats_opt->input_points,
              stats_opt->replay_candidates + stats_opt->non_finite_unindexed + stats_opt->out_of_range_unindexed);

    // Invariant: At any time, stored frames <= max_frames (3)
    EXPECT_LE(cache.frameCount(), 3ULL);
    EXPECT_EQ(cache.newestFrameSeq(), frame_seq);

    const auto cache_stats = cache.statistics();
    EXPECT_LE(cache_stats.stored_frames, 3ULL);
    EXPECT_EQ(cache_stats.stored_point_indices, cache_stats.stored_replay_candidates);

    // Evicted frame reachability verification
    if (f > 3) {
      const uint64_t oldest_possible_seq = (f - 2) * 10;
      EXPECT_GE(*cache.oldestFrameSeq(), oldest_possible_seq);
      // Frame older than window must not exist
      const uint64_t evicted_seq = (f - 3) * 10;
      EXPECT_FALSE(cache.containsFrame(evicted_seq));
      EXPECT_EQ(cache.findFrame(evicted_seq), nullptr);
    }
  }

  // Grand total points processed >= 1,000,000
  EXPECT_GE(grand_total_input, 1000000ULL);

  // Exactly 3 frames retained at the end
  EXPECT_EQ(cache.frameCount(), 3ULL);
  EXPECT_EQ(cache.frameSequenceSnapshot(), (std::vector<uint64_t>{90ULL, 100ULL, 110ULL}));

  // Perform deterministic region query validations against the newest retained frame (110)
  const auto* newest_frame = cache.findFrame(110);
  ASSERT_NE(newest_frame, nullptr);

  const auto bucket_keys = newest_frame->bucketKeySnapshot();
  ASSERT_FALSE(bucket_keys.empty());

  for (size_t k = 0; k < std::min<size_t>(bucket_keys.size(), 5ULL); ++k) {
    const auto& l3_key = bucket_keys[k];
    std::vector<uint32_t> indices;
    EXPECT_TRUE(cache.collectObservationIndices(110, l3_key, indices));

    std::set<uint32_t> unique_indices;
    for (const auto idx : indices) {
      // 1. Index must be in valid range
      EXPECT_LT(idx, newest_frame->pointCount());

      // 2. Point must be finite and within range
      const auto& pt = newest_frame->rawPoints()[idx];
      EXPECT_TRUE(pt.isFinite());

      // 3. No duplicates
      EXPECT_EQ(unique_indices.count(idx), 0ULL);
      unique_indices.insert(idx);
    }
  }
}

}  // namespace
}  // namespace avr::mapping
