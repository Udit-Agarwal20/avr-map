#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/semantic_provider.hpp"
#include "avr_mapping/semantic_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {
namespace {

class StressSemanticProvider : public SemanticProvider {
 public:
  std::optional<SemanticFrameResult> infer(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D) override {
    // Every 50 frames, simulate temporary provider degradation (nullopt)
    if (frame_seq % 50 == 0) {
      return std::nullopt;
    }

    SemanticFrameResult res{.frame_seq = frame_seq, .observations = {}};
    res.observations.reserve(raw_points.size());

    // Deterministic cyclical classification based on frame_seq and point index
    const uint64_t mode = frame_seq % 7;
    for (uint32_t i = 0; i < raw_points.size(); ++i) {
      SemanticObservation obs{
          .source_point_index = i,
          .semantic_class = SemanticClass::UNKNOWN,
          .semantic_confidence = 0.0f,
          .motion_state = MotionClassification::UNKNOWN,
          .motion_confidence = 0.0f,
      };

      switch (mode) {
        case 0:
          // All unknown
          break;
        case 1:
          // Static vehicles
          obs.semantic_class = SemanticClass::VEHICLE;
          obs.semantic_confidence = 0.90f;
          obs.motion_state = MotionClassification::STATIC;
          obs.motion_confidence = 0.90f;
          break;
        case 2:
          // Moving vehicles
          obs.semantic_class = SemanticClass::VEHICLE;
          obs.semantic_confidence = 0.95f;
          obs.motion_state = MotionClassification::CONFIRMED_DYNAMIC;
          obs.motion_confidence = 0.95f;
          break;
        case 3:
          // Pedestrians
          obs.semantic_class = SemanticClass::PEDESTRIAN;
          obs.semantic_confidence = 0.98f;
          obs.motion_state = (i % 2 == 0) ? MotionClassification::CONFIRMED_DYNAMIC : MotionClassification::STATIC;
          obs.motion_confidence = 0.95f;
          break;
        case 4:
          // Candidate motion
          obs.semantic_class = SemanticClass::OBSTACLE_OTHER;
          obs.semantic_confidence = 0.80f;
          obs.motion_state = MotionClassification::CANDIDATE;
          obs.motion_confidence = 0.85f;
          break;
        case 5:
          // Buildings and vegetation
          obs.semantic_class = (i % 2 == 0) ? SemanticClass::BUILDING : SemanticClass::VEGETATION;
          obs.semantic_confidence = 0.88f;
          obs.motion_state = MotionClassification::STATIC;
          obs.motion_confidence = 0.99f;
          break;
        case 6:
          // Mixed: sparse labeling
          if (i % 3 == 0) {
            obs.semantic_class = SemanticClass::PEDESTRIAN;
            obs.semantic_confidence = 0.92f;
            obs.motion_state = MotionClassification::CONFIRMED_DYNAMIC;
            obs.motion_confidence = 0.92f;
          }
          break;
      }
      res.observations.push_back(obs);
    }
    return res;
  }
};

std::vector<PointXYZ> makePointsInBox(
    double min_x, double max_x,
    double min_y, double max_y,
    double z_min, double z_max,
    size_t count_x = 3, size_t count_y = 3) {
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
// 10,000 Semantic-Enabled Processed Frames Deterministic Stress (Section 95)
// ============================================================================

TEST(PersistentAdaptivePipelineSemanticStressTest, TenThousandSemanticFramesDeterministicStress) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 10;
  auto provider = std::make_unique<StressSemanticProvider>();
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  constexpr uint64_t TOTAL_STRESS_FRAMES = 10000;
  uint64_t current_seq = 10;

  for (uint64_t i = 0; i < TOTAL_STRESS_FRAMES; ++i) {
    current_seq += (i % 3) + 1;

    // Alternate between 4 quadrants including negative coordinates
    const int32_t quadrant = static_cast<int32_t>(i % 4);
    const double base_x = (quadrant == 0 || quadrant == 1) ? 0.05 : -0.35;
    const double base_y = (quadrant == 0 || quadrant == 2) ? 0.05 : -0.35;

    // Cyclic height variation to exercise refinement and coarsening cycles
    const double z_span = (i % 20 < 10) ? 0.02 : 0.40;

    auto pts = makePointsInBox(base_x, base_x + 0.04, base_y, base_y + 0.04, 0.0, z_span, 3, 3);
    const avr::spatial::WorldPoint2D origin{base_x, base_y};

    const auto res = pipeline->processFrame(pts, current_seq, origin);
    ASSERT_TRUE(res.isSuccess()) << "Stress frame " << i << " failed with "
                                 << adaptivePipelineStatusToString(res.status);
    ASSERT_TRUE(res.frame_fused);

    if (i % 1000 == 0) {
      ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
    }
  }

  ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
  EXPECT_EQ(pipeline->processingGeneration(), TOTAL_STRESS_FRAMES);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
}

}  // namespace
}  // namespace avr::mapping
