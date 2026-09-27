#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "avr_mapping/dynamic_clusterer.hpp"
#include "avr_mapping/dynamic_types.hpp"
#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include "avr_mapping/persistent_adaptive_pipeline_types.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/semantic_policy_config.hpp"
#include "avr_mapping/semantic_provider.hpp"
#include "avr_mapping/semantic_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {
namespace {

class MockDynamicSemanticProvider : public SemanticProvider {
 public:
  using FrameCallback = std::function<std::optional<SemanticFrameResult>(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin)>;

  void setCallback(FrameCallback cb) {
    callback_ = std::move(cb);
  }

  void setPredefinedResult(uint64_t frame_seq, std::optional<SemanticFrameResult> res) {
    predefined_results_[frame_seq] = std::move(res);
  }

  std::optional<SemanticFrameResult> infer(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin) override {
    if (callback_) {
      return callback_(raw_points, frame_seq, sensor_origin);
    }
    auto it = predefined_results_.find(frame_seq);
    if (it != predefined_results_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

 private:
  FrameCallback callback_{nullptr};
  std::unordered_map<uint64_t, std::optional<SemanticFrameResult>> predefined_results_{};
};

PersistentAdaptivePipelineConfig makeTestConfig() {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.semantic_config.min_semantic_confidence = 0.50f;
  cfg.semantic_config.min_motion_confidence = 0.70f;
  return cfg;
}

}  // namespace

TEST(DynamicExtractionTest, ExtractionGolden) {
  // Requirement 57:
  // Raw semantic frame contains:
  // 0: static vehicle
  // 1: confirmed dynamic vehicle
  // 2: candidate pedestrian
  // 3: confirmed dynamic UNKNOWN
  // 4: invalid NaN confirmed dynamic
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();

  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},  // 0: static vehicle
      PointXYZ{.x = 2.0, .y = 2.0, .z = 0.0},  // 1: confirmed dynamic vehicle
      PointXYZ{.x = 3.0, .y = 3.0, .z = 0.0},  // 2: candidate pedestrian
      PointXYZ{.x = 4.0, .y = 4.0, .z = 0.0},  // 3: confirmed dynamic UNKNOWN
      PointXYZ{.x = std::numeric_limits<double>::quiet_NaN(), .y = 5.0, .z = 0.0},  // 4: invalid NaN dynamic
  };

  SemanticFrameResult s_res{};
  s_res.frame_seq = 1;
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 0,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::STATIC,
      .motion_confidence = 0.95f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 1,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.85f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 2,
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.8f,
      .motion_state = MotionClassification::CANDIDATE,
      .motion_confidence = 0.75f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 3,
      .semantic_class = SemanticClass::UNKNOWN,
      .semantic_confidence = 0.2f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.95f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 4,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.95f,
  });

  prov_ptr->setPredefinedResult(1, std::move(s_res));

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  // Requirement 57 & 6:
  // Points 1 and 3 are valid and confirmed dynamic -> extracted.
  // Point 4 is NaN -> must not enter DynamicPointSet.
  ASSERT_EQ(res.dynamic_points.points.size(), 2u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 2u);
  EXPECT_EQ(res.dynamic_points.frame_seq, 1u);

  EXPECT_EQ(res.dynamic_points.points[0].source_point_index, 1u);
  EXPECT_EQ(res.dynamic_points.points[0].semantic_class, SemanticClass::VEHICLE);
  EXPECT_EQ(res.dynamic_points.points[0].position, raw_points[1]);

  EXPECT_EQ(res.dynamic_points.points[1].source_point_index, 3u);
  EXPECT_EQ(res.dynamic_points.points[1].semantic_class, SemanticClass::UNKNOWN);
  EXPECT_EQ(res.dynamic_points.points[1].position, raw_points[3]);

  // Points 1, 3, 4 are routed DYNAMIC_EXCLUDED
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 3u);
}

TEST(DynamicExtractionTest, RoutingDynamicConservation) {
  // Requirement 6:
  // For geometrically valid raw points:
  // DYNAMIC_EXCLUDED count == DynamicPointSet.points.size().
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 2.0, .y = 2.0, .z = 0.0},
      PointXYZ{.x = 3.0, .y = 3.0, .z = 0.0},
      PointXYZ{.x = 4.0, .y = 4.0, .z = 0.0},
  };

  SemanticFrameResult s_res{};
  s_res.frame_seq = 1;
  // Two points confirmed dynamic
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 0,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.85f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 2,
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.85f,
  });

  prov_ptr->setPredefinedResult(1, std::move(s_res));

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, res.dynamic_points.points.size());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, res.dynamic_points.points.size());
  EXPECT_EQ(res.dynamic_points.points.size(), 2u);
}

TEST(DynamicExtractionTest, DynamicOnlyPipelineGolden) {
  // Requirement 58:
  // All valid points confirmed dynamic.
  // DynamicPointSet contains all valid points.
  // Static map remains empty (no active cells).
  // Raw cache retains all points as DYNAMIC_EXCLUDED.
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 1.1, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 1.2, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 1.3, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 1.4, .y = 1.0, .z = 0.0},
  };

  SemanticFrameResult s_res{};
  s_res.frame_seq = 1;
  for (uint32_t i = 0; i < 5; ++i) {
    s_res.observations.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  prov_ptr->setPredefinedResult(1, std::move(s_res));

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  EXPECT_EQ(res.dynamic_points.points.size(), 5u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 5u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 5u);

  // Static map has 0 active payloads
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);

  // Cache retained all 5 points as DYNAMIC_EXCLUDED
  EXPECT_EQ(pipeline->replayCache().frameCount(), 1u);
  const auto* cached_frame = pipeline->replayCache().findFrame(1);
  ASSERT_NE(cached_frame, nullptr);
  EXPECT_EQ(cached_frame->pointCount(), 5u);
  ASSERT_EQ(cached_frame->routing().size(), 5u);
  for (auto r : cached_frame->routing()) {
    EXPECT_EQ(r, PointRouting::DYNAMIC_EXCLUDED);
  }

  // Downstream clustering can consume DynamicPointSet independently
  DynamicClusterConfig cluster_config{
      .cluster_radius_xy_m = 0.50,
      .max_vertical_gap_m = 0.80,
      .min_cluster_points = 5,
  };
  auto c_res = DynamicClusterer::cluster(res.dynamic_points, cluster_config);
  EXPECT_TRUE(c_res.isSuccess());
  EXPECT_EQ(c_res.clusters.size(), 1u);
  EXPECT_EQ(c_res.clusters[0].member_source_indices.size(), 5u);
}

TEST(DynamicExtractionTest, NoProviderRegression) {
  // Requirement 60:
  // No provider configured -> dynamic_points empty, behavior bitwise identical to frozen baseline.
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), nullptr);
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
      PointXYZ{.x = 2.0, .y = 2.0, .z = 0.0},
  };

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_FALSE(res.semantic_diagnostics.provider_configured);
  EXPECT_FALSE(res.semantic_diagnostics.provider_invoked);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 0u);
  EXPECT_TRUE(res.dynamic_points.points.empty());
  EXPECT_GT(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(DynamicExtractionTest, ProviderUnavailableRegression) {
  // Requirement 61:
  // Provider returns nullopt -> dynamic_points empty, provider_degraded = true.
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
  };

  prov_ptr->setPredefinedResult(1, std::nullopt);

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.semantic_diagnostics.provider_degraded);
  EXPECT_EQ(res.semantic_diagnostics.provider_status, SemanticProviderStatus::UNAVAILABLE);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 0u);
  EXPECT_TRUE(res.dynamic_points.points.empty());
  EXPECT_GT(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(DynamicExtractionTest, MalformedSemanticFrameRegression) {
  // Requirement 62:
  // Malformed semantic frame -> no published dynamic points, map unmodified.
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
  };

  SemanticFrameResult malformed{};
  malformed.frame_seq = 999;  // Frame seq mismatch!
  prov_ptr->setPredefinedResult(1, std::move(malformed));

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_TRUE(res.dynamic_points.points.empty());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 0u);
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(DynamicExtractionTest, MapFailureOutputContract) {
  // Requirement 63:
  // Frame sequence violation or fatal mapping failure -> dynamic_points empty.
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},
  };

  SemanticFrameResult s1{};
  s1.frame_seq = 10;
  s1.observations.push_back(SemanticObservation{
      .source_point_index = 0,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.95f,
  });
  prov_ptr->setPredefinedResult(10, std::move(s1));

  auto res1 = pipeline->processFrame(raw_points, 10, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(res1.isSuccess());
  EXPECT_EQ(res1.dynamic_points.points.size(), 1u);

  // Frame 2 with non-monotonic sequence (5 <= 10)
  SemanticFrameResult s2{};
  s2.frame_seq = 5;
  prov_ptr->setPredefinedResult(5, std::move(s2));

  auto res2 = pipeline->processFrame(raw_points, 5, avr::spatial::WorldPoint2D{0.0, 0.0});
  EXPECT_EQ(res2.status, AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE);
  EXPECT_FALSE(res2.isSuccess());
  EXPECT_TRUE(res2.dynamic_points.points.empty());
  EXPECT_EQ(res2.semantic_diagnostics.dynamic_points_extracted, 0u);
}

struct PersistentAdaptivePipelineTestAccess {
  static void setSimulateFusionFailure(PersistentAdaptiveFramePipeline& p, bool v) noexcept {
    p.test_simulate_fusion_failure_ = v;
  }
};

TEST(DynamicExtractionTest, PostExtractionMapFailureGolden) {
  // Requirements 19, 20, 22:
  // Existing test seam (test_simulate_fusion_failure_) injects mapping failure AFTER
  // semantic validation and internal staging of confirmed-dynamic points.
  // Proves that on mapping failure:
  // - result.dynamic_points is empty (suppressed)
  // - result.semantic_diagnostics.dynamic_points_extracted is 0
  // - result.isSuccess() is false
  // - static map is not mutated
  // - no accepted dynamic frame is published downstream.
  auto provider = std::make_unique<MockDynamicSemanticProvider>();
  auto* prov_ptr = provider.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(makeTestConfig(), std::move(provider));
  ASSERT_NE(pipeline, nullptr);

  // Mixed frame with 1 confirmed dynamic point and 1 static point to exercise mapping fusion path
  std::vector<PointXYZ> raw_points = {
      PointXYZ{.x = 1.0, .y = 1.0, .z = 0.0},  // dynamic
      PointXYZ{.x = 5.0, .y = 5.0, .z = 0.0},  // static
  };

  SemanticFrameResult s_res{};
  s_res.frame_seq = 1;
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 0,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.95f,
  });
  s_res.observations.push_back(SemanticObservation{
      .source_point_index = 1,
      .semantic_class = SemanticClass::BUILDING,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::STATIC,
      .motion_confidence = 0.95f,
  });
  prov_ptr->setPredefinedResult(1, std::move(s_res));

  // Enable post-extraction fusion failure injection seam
  PersistentAdaptivePipelineTestAccess::setSimulateFusionFailure(*pipeline, true);

  auto res = pipeline->processFrame(raw_points, 1, avr::spatial::WorldPoint2D{0.0, 0.0});

  EXPECT_EQ(res.status, AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE);
  EXPECT_FALSE(res.isSuccess());
  EXPECT_FALSE(res.frame_fused);

  // Crucial contract: dynamic_points must be empty and dynamic_points_extracted must be 0!
  EXPECT_TRUE(res.dynamic_points.points.empty());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_points_extracted, 0u);

  // Static map remains untouched
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

}  // namespace avr::mapping

