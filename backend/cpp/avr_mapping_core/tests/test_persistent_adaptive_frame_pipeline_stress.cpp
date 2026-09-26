#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_mapping/point_xyz.hpp"

namespace avr::mapping {
namespace {

// Helper to generate dense points in a specific local meter bounding box
std::vector<PointXYZ> makePointsInBox(
    double min_x, double max_x,
    double min_y, double max_y,
    double z_min, double z_max,
    size_t count_x = 4, size_t count_y = 4) {
  std::vector<PointXYZ> pts;
  pts.reserve(count_x * count_y);
  const double dx = (count_x > 1) ? (max_x - min_x) / static_cast<double>(count_x - 1) : 0.0;
  const double dy = (count_y > 1) ? (max_y - min_y) / static_cast<double>(count_y - 1) : 0.0;
  const double dz = (count_x * count_y > 1) ? (z_max - z_min) / static_cast<double>(count_x * count_y - 1) : 0.0;

  size_t idx = 0;
  for (size_t ix = 0; ix < count_x; ++ix) {
    for (size_t iy = 0; iy < count_y; ++iy) {
      const double x = min_x + static_cast<double>(ix) * dx;
      const double y = min_y + static_cast<double>(iy) * dy;
      const double z = z_min + static_cast<double>(idx++) * dz;
      pts.push_back(PointXYZ{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    }
  }
  return pts;
}

// ============================================================================
// Section 1: 10,000 Successfully Fused Frames Deterministic Stress (Req 91)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineStressTest, TenThousandFusedFramesDeterministicStress) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 10;
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipeline, nullptr);

  uint64_t total_fused_frames = 0;
  uint64_t current_seq = 10;

  // --------------------------------------------------------------------------
  // Scenario 1: New roots & existing roots in positive and negative coords (2,500 frames)
  // --------------------------------------------------------------------------
  constexpr uint64_t SCENARIO_1_FRAMES = 2500;
  for (uint64_t i = 0; i < SCENARIO_1_FRAMES; ++i) {
    current_seq += 2;  // Non-consecutive frame_seq jumps
    // Alternate between 4 different roots (2 positive, 2 negative)
    const int32_t quadrant = static_cast<int32_t>(i % 4);
    const double base_x = (quadrant == 0 || quadrant == 1) ? 0.05 : -0.35;
    const double base_y = (quadrant == 0 || quadrant == 2) ? 0.05 : -0.35;

    auto pts = makePointsInBox(base_x, base_x + 0.04, base_y, base_y + 0.04, 0.0, 0.02, 2, 2);
    const avr::spatial::WorldPoint2D origin{base_x, base_y};

    const auto res = pipeline->processFrame(pts, current_seq, origin);
    ASSERT_TRUE(res.isSuccess()) << "Scenario 1 frame " << i << " failed with "
                                 << adaptivePipelineStatusToString(res.status);
    ASSERT_TRUE(res.frame_fused);
    ++total_fused_frames;

    // Validate map invariants after every frame
    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }

  // --------------------------------------------------------------------------
  // Scenario 2: Localized refinement with cascaded replay complete & insufficient (2,500 frames)
  // --------------------------------------------------------------------------
  constexpr uint64_t SCENARIO_2_FRAMES = 2500;
  for (uint64_t i = 0; i < SCENARIO_2_FRAMES; ++i) {
    current_seq += 3;
    // Localized high-complexity evidence in sub-quadrant of a new root
    const double root_offset = static_cast<double>((i % 5) + 1) * 0.40;
    auto pts = makePointsInBox(root_offset + 0.01, root_offset + 0.03, 0.01, 0.03, 0.0, 0.05, 3, 3);
    const avr::spatial::WorldPoint2D origin{root_offset, 0.0};

    const auto res = pipeline->processFrame(pts, current_seq, origin);
    ASSERT_TRUE(res.isSuccess()) << "Scenario 2 frame " << i << " failed with "
                                 << adaptivePipelineStatusToString(res.status);
    ASSERT_TRUE(res.frame_fused);
    ++total_fused_frames;

    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }

  // --------------------------------------------------------------------------
  // Scenario 3: Non-consecutive frame_seq jumps with multi-root frames (2,500 frames)
  // --------------------------------------------------------------------------
  constexpr uint64_t SCENARIO_3_FRAMES = 2500;
  for (uint64_t i = 0; i < SCENARIO_3_FRAMES; ++i) {
    current_seq += 5 + (i % 7);  // Variable non-consecutive jumps
    // Multi-root frame: points in positive root [0, 0.4] and negative root [-0.4, 0]
    auto pts1 = makePointsInBox(0.02, 0.06, 0.02, 0.06, 0.0, 0.02, 2, 2);
    auto pts2 = makePointsInBox(-0.35, -0.30, -0.35, -0.30, 0.0, 0.02, 2, 2);
    pts1.insert(pts1.end(), pts2.begin(), pts2.end());

    const avr::spatial::WorldPoint2D origin{0.0, 0.0};
    const auto res = pipeline->processFrame(pts1, current_seq, origin);
    ASSERT_TRUE(res.isSuccess()) << "Scenario 3 frame " << i << " failed with "
                                 << adaptivePipelineStatusToString(res.status);
    ASSERT_TRUE(res.frame_fused);
    ++total_fused_frames;

    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }

  // --------------------------------------------------------------------------
  // Scenario 4: HOLD, COARSEN_PENDING, COARSEN_READY cycles without auto merge (2,500 frames)
  // --------------------------------------------------------------------------
  constexpr uint64_t SCENARIO_4_FRAMES = 2500;
  const size_t leaves_before_coarsen_cycles = pipeline->map().statistics().hierarchy_stats.leaf_count;
  for (uint64_t i = 0; i < SCENARIO_4_FRAMES; ++i) {
    current_seq += 10;
    // Sensor far away (70m) repeatedly requesting coarse resolution
    auto pts = makePointsInBox(0.02, 0.06, 0.02, 0.06, 0.0, 0.01, 2, 2);
    const avr::spatial::WorldPoint2D origin{70.0, 0.0};

    const auto res = pipeline->processFrame(pts, current_seq, origin);
    ASSERT_TRUE(res.isSuccess()) << "Scenario 4 frame " << i << " failed with "
                                 << adaptivePipelineStatusToString(res.status);
    ASSERT_TRUE(res.frame_fused);
    // Crucial M9B3F1 invariant: no automatic merge occurs even when COARSEN_READY!
    ASSERT_FALSE(res.structure_changed);
    ++total_fused_frames;

    ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  }

  // Verify leaves were NOT merged
  EXPECT_EQ(pipeline->map().statistics().hierarchy_stats.leaf_count, leaves_before_coarsen_cycles);

  // Total fused frames verification
  EXPECT_GE(total_fused_frames, 10000U);
  EXPECT_EQ(pipeline->processingGeneration(), total_fused_frames);
}

// ============================================================================
// Section 2: Topology Size & Bounded Localized Refinement (Req 92)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineStressTest, TopologySizeBoundedLocalizedRefinement) {
  // 1. Localized Evidence Case
  {
    auto pipeline = PersistentAdaptiveFramePipeline::create();
    ASSERT_NE(pipeline, nullptr);

    // Root L3 covers [0, 0.4m] x [0, 0.4m].
    // High complexity points strictly restricted to one 3cm x 3cm corner.
    auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.10, 4, 4);
    const auto res = pipeline->processFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(res.isSuccess());

    const auto stats = pipeline->map().statistics().hierarchy_stats;
    const size_t root_count = stats.leaf_count_by_level[avr::spatial::L3] + stats.internal_count_by_level[avr::spatial::L3];
    std::cout << "[TopologySizeReport: Localized Case]"
              << " roots=" << root_count
              << " internal_nodes=" << stats.internal_count
              << " leaf_nodes=" << stats.leaf_count << std::endl;

    // A uniform full 4-level quadtree has 1 root + 4 L2 + 16 L1 + 64 L0 = 85 nodes (64 leaves).
    // Localized refinement MUST stay strictly bounded: only one branch splits.
    EXPECT_EQ(root_count, 1U);
    EXPECT_LT(stats.leaf_count, 20U);
    EXPECT_LT(stats.internal_count, 5U);
    EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
  }

  // 2. Widespread Evidence Case
  {
    auto pipeline = PersistentAdaptiveFramePipeline::create();
    ASSERT_NE(pipeline, nullptr);

    // Evidence uniformly distributed across all 4 quadrants of L3
    std::vector<PointXYZ> pts;
    for (int qx = 0; qx < 2; ++qx) {
      for (int qy = 0; qy < 2; ++qy) {
        double bx = 0.02 + static_cast<double>(qx) * 0.20;
        double by = 0.02 + static_cast<double>(qy) * 0.20;
        auto q_pts = makePointsInBox(bx, bx + 0.08, by, by + 0.08, 0.0, 0.10, 3, 3);
        pts.insert(pts.end(), q_pts.begin(), q_pts.end());
      }
    }

    const auto res = pipeline->processFrame(pts, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
    ASSERT_TRUE(res.isSuccess());

    const auto stats = pipeline->map().statistics().hierarchy_stats;
    const size_t root_count = stats.leaf_count_by_level[avr::spatial::L3] + stats.internal_count_by_level[avr::spatial::L3];
    std::cout << "[TopologySizeReport: Widespread Case]"
              << " roots=" << root_count
              << " internal_nodes=" << stats.internal_count
              << " leaf_nodes=" << stats.leaf_count << std::endl;

    EXPECT_EQ(root_count, 1U);
    EXPECT_GT(stats.leaf_count, 12U);
    EXPECT_TRUE(pipeline->map().validateInvariants().is_valid);
  }
}

// ============================================================================
// Section 3: Exact Bit-for-Bit Deterministic Repeatability (Req 93)
// ============================================================================

TEST(PersistentAdaptiveFramePipelineStressTest, DeterministicRepeatabilityTwoPipelines) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 5;

  auto pipelineA = PersistentAdaptiveFramePipeline::create(cfg);
  auto pipelineB = PersistentAdaptiveFramePipeline::create(cfg);
  ASSERT_NE(pipelineA, nullptr);
  ASSERT_NE(pipelineB, nullptr);

  constexpr size_t NUM_FRAMES = 120;
  uint64_t seq = 100;

  for (size_t f = 0; f < NUM_FRAMES; ++f) {
    seq += (f % 5) + 1;

    // Generate diverse frame configurations
    const double bx = (f % 3 == 0) ? -0.30 : 0.05 * static_cast<double>(f % 4);
    const double by = (f % 2 == 0) ? 0.05 : -0.25;
    const double origin_dist = (f % 6 == 0) ? 70.0 : 0.0;

    auto pts = makePointsInBox(bx, bx + 0.05, by, by + 0.05, 0.0, 0.05, 3, 3);
    const avr::spatial::WorldPoint2D origin{origin_dist, 0.0};

    const auto resA = pipelineA->processFrame(pts, seq, origin);
    const auto resB = pipelineB->processFrame(pts, seq, origin);

    // 1. Verify semantic result fields equivalence
    EXPECT_EQ(resA.frame_seq, resB.frame_seq);
    EXPECT_EQ(resA.processing_generation_before, resB.processing_generation_before);
    EXPECT_EQ(resA.processing_generation_after, resB.processing_generation_after);
    EXPECT_EQ(resA.input_points, resB.input_points);
    EXPECT_EQ(resA.accepted_points, resB.accepted_points);
    EXPECT_EQ(resA.frame_fused, resB.frame_fused);
    EXPECT_EQ(resA.structure_changed, resB.structure_changed);
    EXPECT_EQ(resA.cache_prepared, resB.cache_prepared);
    EXPECT_EQ(resA.cache_committed, resB.cache_committed);
    EXPECT_EQ(resA.replay_pending, resB.replay_pending);
    EXPECT_EQ(resA.regions_evaluated, resB.regions_evaluated);
    EXPECT_EQ(resA.prospective_roots, resB.prospective_roots);
    EXPECT_EQ(resA.refine_requests, resB.refine_requests);
    EXPECT_EQ(resA.splits_applied, resB.splits_applied);
    EXPECT_EQ(resA.replay_attempts, resB.replay_attempts);
    EXPECT_EQ(resA.replay_complete, resB.replay_complete);
    EXPECT_EQ(resA.replay_insufficient, resB.replay_insufficient);
    EXPECT_EQ(resA.hysteresis_previews, resB.hysteresis_previews);
    EXPECT_EQ(resA.hysteresis_committed, resB.hysteresis_committed);
    EXPECT_EQ(resA.hysteresis_erased, resB.hysteresis_erased);
    EXPECT_EQ(resA.status, resB.status);

    // 2. Decision records & Structural action records equivalence
    EXPECT_EQ(resA.decision_records, resB.decision_records);
    EXPECT_EQ(resA.structural_action_records, resB.structural_action_records);
  }

  // 3. Processing generation equivalence
  EXPECT_EQ(pipelineA->processingGeneration(), pipelineB->processingGeneration());

  // 4. Hierarchy topology snapshot equivalence
  EXPECT_EQ(pipelineA->map().hierarchy().topologySnapshot(),
            pipelineB->map().hierarchy().topologySnapshot());

  // 5. Payload snapshot equivalence
  EXPECT_EQ(pipelineA->map().payloadStore().activePayloadSnapshot(),
            pipelineB->map().payloadStore().activePayloadSnapshot());

  // 6. Internal shadow snapshot equivalence
  EXPECT_EQ(pipelineA->map().internalCoarseningSnapshot(),
            pipelineB->map().internalCoarseningSnapshot());

  // 7. Hysteresis snapshot equivalence
  EXPECT_EQ(pipelineA->hysteresis().stateSnapshot(),
            pipelineB->hysteresis().stateSnapshot());

  // 8. Cache sequence snapshot equivalence
  EXPECT_EQ(pipelineA->replayCache().frameSequenceSnapshot(),
            pipelineB->replayCache().frameSequenceSnapshot());
}

}  // namespace
}  // namespace avr::mapping
