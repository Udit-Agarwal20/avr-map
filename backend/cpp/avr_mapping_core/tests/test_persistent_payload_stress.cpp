#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

// Deterministic 64-bit Linear Congruential Generator for reproducible synthetic points
class DeterministicRng {
 public:
  explicit DeterministicRng(uint64_t seed) : state_(seed) {}

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

TEST(PersistentPayloadStressTest, MultiFrameStressAndConservation) {
  PersistentAdaptiveMap map{};
  DeterministicRng rng(0xCAFEF00D12345678ULL);

  constexpr size_t FRAME_COUNT = 25;
  constexpr size_t POINTS_PER_FRAME = 25000;
  uint64_t grand_total_accepted = 0;

  for (size_t f = 1; f <= FRAME_COUNT; ++f) {
    const uint64_t frame_seq = f * 10;
    std::vector<PointXYZ> points;
    points.reserve(POINTS_PER_FRAME);

    for (size_t p = 0; p < POINTS_PER_FRAME; ++p) {
      // Span positive and negative coordinates [-8.0, +8.0] m
      const double x = rng.uniform(-8.0, 8.0);
      const double y = rng.uniform(-8.0, 8.0);
      const double z = rng.uniform(-2.0, 4.0);
      points.push_back(PointXYZ{x, y, z});
    }

    // Interleave manual structural refinements at specific frames
    if (f == 5) {
      EXPECT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L1));
    }
    if (f == 10) {
      EXPECT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{-10, -10}, avr::spatial::L0));
    }
    if (f == 15) {
      EXPECT_TRUE(map.refineToLevel(avr::spatial::GridCoord2D{12, -8}, avr::spatial::L2));
    }

    FrameEvidence evidence{};
    const auto frame_stats = evidence.buildFrame(points, frame_seq, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(frame_stats.has_value());

    const auto fusion_stats = map.fuseFrameEvidence(evidence);
    ASSERT_TRUE(fusion_stats.has_value());

    // Observation Conservation (Section 64)
    EXPECT_EQ(fusion_stats->observations_fused, evidence.statistics().accepted_points);
    EXPECT_EQ(fusion_stats->frame_seq, frame_seq);
    grand_total_accepted += fusion_stats->observations_fused;

    // Periodic invariant validation
    if (f % 5 == 0) {
      const auto val = map.validateInvariants();
      EXPECT_TRUE(val.is_valid) << "Invariant failed at frame " << f << ": " << val.error_message;
    }
  }

  // Ensure total points stress requirement (>= 500,000 points) is satisfied
  EXPECT_GE(grand_total_accepted, 500000ULL);
  EXPECT_EQ(grand_total_accepted, FRAME_COUNT * POINTS_PER_FRAME);

  const auto final_val = map.validateInvariants();
  EXPECT_TRUE(final_val.is_valid) << "Final validation failed: " << final_val.error_message;

  const auto stats = map.statistics();
  EXPECT_GT(stats.hierarchy_stats.node_count, 0ULL);
  EXPECT_GT(stats.payload_stats.active_payload_count, 0ULL);
  EXPECT_GT(stats.payload_stats.payload_tile_count, 0ULL);
}

TEST(PersistentPayloadStressTest, MultiInstanceRepeatability) {
  PersistentAdaptiveMap map1{};
  PersistentAdaptiveMap map2{};

  DeterministicRng rng(0x1337BEEFCAFEULL);
  constexpr size_t REPEAT_FRAMES = 5;
  constexpr size_t REPEAT_POINTS = 5000;

  for (size_t f = 1; f <= REPEAT_FRAMES; ++f) {
    const uint64_t frame_seq = f * 5;
    std::vector<PointXYZ> points;
    points.reserve(REPEAT_POINTS);

    for (size_t p = 0; p < REPEAT_POINTS; ++p) {
      const double x = rng.uniform(-4.0, 4.0);
      const double y = rng.uniform(-4.0, 4.0);
      const double z = rng.uniform(0.0, 3.0);
      points.push_back(PointXYZ{x, y, z});
    }

    if (f == 2) {
      EXPECT_TRUE(map1.refineToLevel(avr::spatial::GridCoord2D{5, 5}, avr::spatial::L1));
      EXPECT_TRUE(map2.refineToLevel(avr::spatial::GridCoord2D{5, 5}, avr::spatial::L1));
    }

    FrameEvidence ev1{};
    FrameEvidence ev2{};
    ASSERT_TRUE(ev1.buildFrame(points, frame_seq, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
    ASSERT_TRUE(ev2.buildFrame(points, frame_seq, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

    const auto s1 = map1.fuseFrameEvidence(ev1);
    const auto s2 = map2.fuseFrameEvidence(ev2);
    ASSERT_TRUE(s1.has_value());
    ASSERT_TRUE(s2.has_value());
    EXPECT_EQ(*s1, *s2);
  }

  // Exact snapshot repeatability
  const auto active_snap1 = map1.activePayloadSnapshot();
  const auto active_snap2 = map2.activePayloadSnapshot();
  EXPECT_EQ(active_snap1, active_snap2);

  const auto retired_snap1 = map1.retiredCacheSnapshot();
  const auto retired_snap2 = map2.retiredCacheSnapshot();
  EXPECT_EQ(retired_snap1, retired_snap2);

  const auto topo1 = map1.hierarchy().topologySnapshot();
  const auto topo2 = map2.hierarchy().topologySnapshot();
  EXPECT_EQ(topo1.size(), topo2.size());
  for (size_t i = 0; i < topo1.size(); ++i) {
    EXPECT_EQ(topo1[i].key, topo2[i].key);
    EXPECT_EQ(topo1[i].state, topo2[i].state);
  }

  EXPECT_EQ(map1.statistics(), map2.statistics());
}

}  // namespace
}  // namespace avr::mapping
