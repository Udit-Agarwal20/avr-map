#include <gtest/gtest.h>

#include <type_traits>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(PersistentAdaptiveMapTest, NonCopyableNonMovable) {
  static_assert(!std::is_copy_constructible_v<PersistentAdaptiveMap>);
  static_assert(!std::is_copy_assignable_v<PersistentAdaptiveMap>);
  static_assert(!std::is_move_constructible_v<PersistentAdaptiveMap>);
  static_assert(!std::is_move_assignable_v<PersistentAdaptiveMap>);
}

TEST(PersistentAdaptiveMapTest, InitialStateAndInspection) {
  const PersistentAdaptiveMap map{};
  EXPECT_FALSE(map.hasFusedFrames());
  EXPECT_EQ(map.lastFusedFrameSeq(), 0ULL);
  EXPECT_EQ(map.hierarchy().nodeCount(), 0ULL);
  EXPECT_EQ(map.payloadStore().tileCount(), 0ULL);
  EXPECT_TRUE(map.activePayloadSnapshot().empty());
  EXPECT_TRUE(map.retiredCacheSnapshot().empty());

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid);
}

TEST(PersistentAdaptiveMapTest, EnsureRootForBase) {
  PersistentAdaptiveMap map{};
  const avr::spatial::GridCoord2D base{0, 0};

  const auto root_key_opt = map.ensureRootForBase(base);
  ASSERT_TRUE(root_key_opt.has_value());

  EXPECT_EQ(root_key_opt->level, avr::spatial::L3);
  EXPECT_TRUE(map.hierarchy().isLeaf(*root_key_opt));

  // No payload fabricated upon root creation
  EXPECT_EQ(map.findActivePayload(*root_key_opt), nullptr);
  EXPECT_EQ(map.findRetiredCache(*root_key_opt), nullptr);
  EXPECT_FALSE(map.requiresReplay(*root_key_opt));

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid);
}

TEST(PersistentAdaptiveMapTest, ValidEmptyFrameLifecycle) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  const std::vector<PointXYZ> empty_pts{};
  const auto frame_stats = evidence.buildFrame(empty_pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  ASSERT_TRUE(frame_stats.has_value());

  const auto fusion_stats = map.fuseFrameEvidence(evidence);
  ASSERT_TRUE(fusion_stats.has_value());

  EXPECT_EQ(fusion_stats->frame_seq, 10ULL);
  EXPECT_EQ(fusion_stats->accepted_points, 0ULL);
  EXPECT_EQ(fusion_stats->unique_leaves_fused, 0ULL);
  EXPECT_EQ(map.lastFusedFrameSeq(), 10ULL);
  EXPECT_TRUE(map.hasFusedFrames());

  // No roots or payload were created
  EXPECT_EQ(map.hierarchy().nodeCount(), 0ULL);
  EXPECT_TRUE(map.payloadStore().isEmpty());

  const auto val = map.validateInvariants();
  EXPECT_TRUE(val.is_valid);
}

TEST(PersistentAdaptiveMapTest, DuplicateAndOutOfOrderFramesRejected) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  const std::vector<PointXYZ> pts = {{1.0, 1.0, 2.0}};
  ASSERT_TRUE(evidence.buildFrame(pts, 42, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  ASSERT_TRUE(map.fuseFrameEvidence(evidence).has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 42ULL);

  // Attempt same frame_seq 42 again: must be rejected
  EXPECT_FALSE(map.fuseFrameEvidence(evidence).has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 42ULL);

  // Attempt older frame_seq 20: must be rejected
  FrameEvidence older_evidence{};
  ASSERT_TRUE(older_evidence.buildFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_FALSE(map.fuseFrameEvidence(older_evidence).has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 42ULL);

  // Future valid frame 105: must succeed
  FrameEvidence newer_evidence{};
  ASSERT_TRUE(newer_evidence.buildFrame(pts, 105, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  EXPECT_TRUE(map.fuseFrameEvidence(newer_evidence).has_value());
  EXPECT_EQ(map.lastFusedFrameSeq(), 105ULL);
}

TEST(PersistentAdaptiveMapTest, ResetClearsAllState) {
  PersistentAdaptiveMap map{};
  FrameEvidence evidence{};

  const std::vector<PointXYZ> pts = {{1.0, 1.0, 2.0}};
  ASSERT_TRUE(evidence.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(evidence).has_value());

  EXPECT_GT(map.hierarchy().nodeCount(), 0ULL);
  EXPECT_GT(map.payloadStore().tileCount(), 0ULL);
  EXPECT_TRUE(map.hasFusedFrames());

  map.reset();

  EXPECT_EQ(map.hierarchy().nodeCount(), 0ULL);
  EXPECT_TRUE(map.payloadStore().isEmpty());
  EXPECT_FALSE(map.hasFusedFrames());
  EXPECT_EQ(map.lastFusedFrameSeq(), 0ULL);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

TEST(PersistentAdaptiveMapTest, FrameEvidenceElevationAggregationContract) {
  FrameEvidence evidence{};
  const std::vector<PointXYZ> pts = {
      {0.02, 0.02, 1.0},
      {0.02, 0.02, 3.0},
      {0.08, 0.08, 5.0},
  };
  ASSERT_TRUE(evidence.buildFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{0, 0}, avr::spatial::L3);
  const auto agg_opt = evidence.aggregateElevationStats(root_key);
  ASSERT_TRUE(agg_opt.has_value());

  const auto summary = evidence.summarize(root_key);
  EXPECT_EQ(agg_opt->count(), summary.observation_count);
  EXPECT_DOUBLE_EQ(agg_opt->mean(), *summary.elevation_mean);
  EXPECT_DOUBLE_EQ(agg_opt->min(), *summary.z_min);
  EXPECT_DOUBLE_EQ(agg_opt->max(), *summary.z_max);
}

TEST(PersistentAdaptiveMapTest, LastUpdateReasonsContract) {
  PersistentAdaptiveMap map{};

  const std::vector<PointXYZ> pts = {{0.05, 0.05, 2.0}};
  FrameEvidence ev1{};
  ASSERT_TRUE(ev1.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());

  const ResolutionReasonBits reasons_a(ResolutionReason::DISTANCE_BASELINE);
  ASSERT_TRUE(map.fuseFrameEvidence(ev1, reasons_a).has_value());

  const auto root_key = *avr::spatial::CellKey::fromGridCoord(avr::spatial::GridCoord2D{1, 1}, avr::spatial::L3);
  const auto* p1 = map.findActivePayload(root_key);
  ASSERT_NE(p1, nullptr);
  EXPECT_TRUE(p1->lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));
  EXPECT_FALSE(p1->lastUpdateReasons().has(ResolutionReason::UNCERTAINTY));

  // Frame 20 with reason B (UNCERTAINTY)
  FrameEvidence ev2{};
  ASSERT_TRUE(ev2.buildFrame(pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const ResolutionReasonBits reasons_b(ResolutionReason::UNCERTAINTY);
  ASSERT_TRUE(map.fuseFrameEvidence(ev2, reasons_b).has_value());

  const auto* p2 = map.findActivePayload(root_key);
  ASSERT_NE(p2, nullptr);
  // Must be strictly reasons_b, NOT reasons_a | reasons_b
  EXPECT_TRUE(p2->lastUpdateReasons().has(ResolutionReason::UNCERTAINTY));
  EXPECT_FALSE(p2->lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));

  // Split: retired parent cache must preserve reason B exactly
  ASSERT_TRUE(map.splitLeaf(root_key));
  const auto* cache = map.findRetiredCache(root_key);
  ASSERT_NE(cache, nullptr);
  EXPECT_TRUE(cache->lastUpdateReasons().has(ResolutionReason::UNCERTAINTY));
  EXPECT_FALSE(cache->lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));
}

TEST(PersistentAdaptiveMapTest, EmptyFrameSequencing) {
  PersistentAdaptiveMap map{};

  // Populated frame 10
  FrameEvidence ev10{};
  const std::vector<PointXYZ> pts = {{1.0, 1.0, 2.0}};
  ASSERT_TRUE(ev10.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev10).has_value());

  const auto stats_before = map.statistics();
  EXPECT_EQ(stats_before.last_fused_frame_seq, 10ULL);
  EXPECT_EQ(stats_before.total_frames_fused, 1ULL);
  EXPECT_EQ(stats_before.total_observations_fused, 1ULL);
  EXPECT_EQ(stats_before.payload_stats.active_payload_count, 1ULL);

  // Valid empty frame 20
  FrameEvidence ev20{};
  const std::vector<PointXYZ> empty_pts{};
  ASSERT_TRUE(ev20.buildFrame(empty_pts, 20, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  const auto fusion_stats = map.fuseFrameEvidence(ev20);
  ASSERT_TRUE(fusion_stats.has_value());

  const auto stats_after = map.statistics();
  // Sequencing advances
  EXPECT_EQ(stats_after.last_fused_frame_seq, 20ULL);
  EXPECT_EQ(stats_after.total_frames_fused, 2ULL);
  // Observations and map state remain unchanged
  EXPECT_EQ(stats_after.total_observations_fused, 1ULL);
  EXPECT_EQ(stats_after.payload_stats.active_payload_count, 1ULL);
  EXPECT_EQ(stats_after.hierarchy_stats.node_count, stats_before.hierarchy_stats.node_count);
}

TEST(PersistentAdaptiveMapTest, ResetAndReuse) {
  PersistentAdaptiveMap map{};

  // Create populated complex hierarchy: L3 -> L2 -> L1
  FrameEvidence ev1{};
  const std::vector<PointXYZ> pts = {{0.05, 0.05, 1.0}, {0.25, 0.25, 2.0}};
  ASSERT_TRUE(ev1.buildFrame(pts, 10, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev1).has_value());

  const auto base = avr::spatial::GridCoord2D{1, 1};
  ASSERT_TRUE(map.refineToLevel(base, avr::spatial::L1));

  EXPECT_GT(map.hierarchy().nodeCount(), 1ULL);
  EXPECT_GT(map.payloadStore().statistics().retired_cache_count, 0ULL);
  EXPECT_GT(map.replayRequiredSnapshot().size(), 0ULL);

  // Reset
  map.reset();
  EXPECT_TRUE(map.validateInvariants().is_valid);
  EXPECT_EQ(map.hierarchy().nodeCount(), 0ULL);
  EXPECT_TRUE(map.payloadStore().isEmpty());
  EXPECT_EQ(map.lastFusedFrameSeq(), 0ULL);
  EXPECT_FALSE(map.hasFusedFrames());
  EXPECT_TRUE(map.replayRequiredSnapshot().empty());
  EXPECT_TRUE(map.retiredCacheSnapshot().empty());

  // Rebuild afresh: frame 1 at negative coordinates
  FrameEvidence ev_fresh{};
  const std::vector<PointXYZ> fresh_pts = {{-5.0, -5.0, 10.0}};
  ASSERT_TRUE(ev_fresh.buildFrame(fresh_pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0}).has_value());
  ASSERT_TRUE(map.fuseFrameEvidence(ev_fresh).has_value());

  EXPECT_EQ(map.lastFusedFrameSeq(), 1ULL);
  EXPECT_EQ(map.statistics().total_frames_fused, 1ULL);
  EXPECT_EQ(map.statistics().total_observations_fused, 1ULL);
  EXPECT_EQ(map.payloadStore().statistics().active_payload_count, 1ULL);
  EXPECT_EQ(map.payloadStore().statistics().retired_cache_count, 0ULL);
  EXPECT_TRUE(map.validateInvariants().is_valid);
}

}  // namespace
}  // namespace avr::mapping
