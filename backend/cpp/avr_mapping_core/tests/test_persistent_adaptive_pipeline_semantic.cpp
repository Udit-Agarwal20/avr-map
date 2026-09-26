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

class MockSemanticProvider : public SemanticProvider {
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

  void setThrowRuntimeError(bool should_throw) {
    throw_runtime_error_ = should_throw;
  }

  void setThrowBadAlloc(bool should_throw) {
    throw_bad_alloc_ = should_throw;
  }

  [[nodiscard]] size_t inferCallCount() const noexcept {
    return infer_call_count_;
  }

  std::optional<SemanticFrameResult> infer(
      std::span<const PointXYZ> raw_points,
      uint64_t frame_seq,
      avr::spatial::WorldPoint2D sensor_origin) override {
    ++infer_call_count_;
    if (throw_bad_alloc_) {
      throw std::bad_alloc();
    }
    if (throw_runtime_error_) {
      throw std::runtime_error("Simulated provider exception");
    }
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
  bool throw_runtime_error_{false};
  bool throw_bad_alloc_{false};
  size_t infer_call_count_{0};
};

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
// Section 1: Types & Configuration Validation
// ============================================================================

TEST(SemanticPipelineTest, SemanticTypesAndPolicyValidation) {
  EXPECT_TRUE(isValidSemanticClass(SemanticClass::UNKNOWN));
  EXPECT_TRUE(isValidSemanticClass(SemanticClass::PEDESTRIAN));
  EXPECT_TRUE(isValidSemanticClass(SemanticClass::OBSTACLE_OTHER));
  EXPECT_FALSE(isValidSemanticClass(static_cast<SemanticClass>(8)));
  EXPECT_FALSE(isValidSemanticClass(static_cast<SemanticClass>(255)));

  EXPECT_TRUE(isValidMotionClassification(MotionClassification::UNKNOWN));
  EXPECT_TRUE(isValidMotionClassification(MotionClassification::CONFIRMED_DYNAMIC));
  EXPECT_FALSE(isValidMotionClassification(static_cast<MotionClassification>(4)));
  EXPECT_FALSE(isValidMotionClassification(static_cast<MotionClassification>(200)));

  SemanticPolicyConfig valid_cfg{};
  EXPECT_TRUE(valid_cfg.isValid());

  SemanticPolicyConfig bad_cfg = valid_cfg;
  bad_cfg.min_semantic_confidence = -0.1f;
  EXPECT_FALSE(bad_cfg.isValid());

  bad_cfg = valid_cfg;
  bad_cfg.min_semantic_confidence = 1.1f;
  EXPECT_FALSE(bad_cfg.isValid());

  bad_cfg = valid_cfg;
  bad_cfg.min_motion_confidence = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(bad_cfg.isValid());

  PersistentAdaptivePipelineConfig pipe_cfg{};
  pipe_cfg.semantic_config = bad_cfg;
  EXPECT_FALSE(pipe_cfg.isValid());
  auto p = PersistentAdaptiveFramePipeline::create(pipe_cfg);
  EXPECT_EQ(p, nullptr);
}

// ============================================================================
// Section 2: Provider Unavailable Fallback (Section 56)
// ============================================================================

TEST(SemanticPipelineTest, ProviderUnavailableFallback) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  // Default mock returns nullopt for any sequence
  PersistentAdaptivePipelineConfig cfg{};
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 3, 3);
  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});

  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.frame_fused);
  EXPECT_TRUE(res.semantic_diagnostics.provider_configured);
  EXPECT_TRUE(res.semantic_diagnostics.provider_invoked);
  EXPECT_TRUE(res.semantic_diagnostics.provider_degraded);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 0u);
}

// ============================================================================
// Section 3: Provider Exception Handling (Sections 57, 58)
// ============================================================================

TEST(SemanticPipelineTest, ProviderRuntimeErrorNonFatal) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  mock_ptr->setThrowRuntimeError(true);
  auto* mock_raw = mock_ptr.get();

  PersistentAdaptivePipelineConfig cfg{};
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 3, 3);
  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});

  EXPECT_EQ(res.status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_ERROR);
  EXPECT_FALSE(res.frame_fused);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  EXPECT_EQ(pipeline->processingGeneration(), 0u);
  EXPECT_EQ(pipeline->map().hierarchy().nodeCount(), 0u);

  // Next valid frame must succeed
  mock_raw->setThrowRuntimeError(false);
  mock_raw->setPredefinedResult(2, SemanticFrameResult{.frame_seq = 2, .observations = {}});
  const auto res2 = pipeline->processFrame(pts, 2, {0.0, 0.0});
  EXPECT_TRUE(res2.isSuccess());
  EXPECT_TRUE(res2.frame_fused);
  EXPECT_EQ(pipeline->processingGeneration(), 1u);
}

TEST(SemanticPipelineTest, ProviderBadAllocNonFatal) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  mock_ptr->setThrowBadAlloc(true);
  auto* mock_raw = mock_ptr.get();

  PersistentAdaptivePipelineConfig cfg{};
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 3, 3);
  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});

  EXPECT_EQ(res.status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE);
  EXPECT_FALSE(res.frame_fused);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  EXPECT_EQ(pipeline->processingGeneration(), 0u);

  // Recovery on next frame
  mock_raw->setThrowBadAlloc(false);
  mock_raw->setPredefinedResult(2, SemanticFrameResult{.frame_seq = 2, .observations = {}});
  const auto res2 = pipeline->processFrame(pts, 2, {0.0, 0.0});
  EXPECT_TRUE(res2.isSuccess());
  EXPECT_TRUE(res2.frame_fused);
}

// ============================================================================
// Section 4: Malformed Provider Output (Sections 59-62)
// ============================================================================

TEST(SemanticPipelineTest, MalformedDuplicateIndexRejected) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 3, 3);

  SemanticFrameResult malformed{};
  malformed.frame_seq = 1;
  malformed.observations.push_back(SemanticObservation{
      .source_point_index = 2,
      .semantic_class = SemanticClass::VEHICLE,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::STATIC,
      .motion_confidence = 0.9f,
  });
  malformed.observations.push_back(SemanticObservation{
      .source_point_index = 2,  // Duplicate!
      .semantic_class = SemanticClass::PEDESTRIAN,
      .semantic_confidence = 0.8f,
      .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
      .motion_confidence = 0.8f,
  });
  mock_raw->setPredefinedResult(1, std::move(malformed));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
  EXPECT_EQ(pipeline->processingGeneration(), 0u);
}

TEST(SemanticPipelineTest, MalformedIndexOutOfBoundsRejected) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 2, 2);  // 4 points (indices 0..3)

  SemanticFrameResult malformed{};
  malformed.frame_seq = 1;
  malformed.observations.push_back(SemanticObservation{
      .source_point_index = 4,  // Index == raw_points.size() -> OOB!
      .semantic_class = SemanticClass::BUILDING,
      .semantic_confidence = 0.9f,
      .motion_state = MotionClassification::STATIC,
      .motion_confidence = 0.9f,
  });
  mock_raw->setPredefinedResult(1, std::move(malformed));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_EQ(res.status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  EXPECT_FALSE(pipeline->fatalErrorLatched());
}

TEST(SemanticPipelineTest, MalformedConfidenceRejected) {
  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 2, 2);

  // NaN semantic confidence
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = std::numeric_limits<float>::quiet_NaN(),
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = 0.9f,
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }

  // > 1.0 semantic confidence
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 1.05f,
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = 0.9f,
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }

  // Inf motion confidence
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = std::numeric_limits<float>::infinity(),
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }

  // Negative motion confidence
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = -0.1f,
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }
}

TEST(SemanticPipelineTest, MalformedEnumRejected) {
  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 2, 2);

  // Invalid SemanticClass
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = static_cast<SemanticClass>(42),
        .semantic_confidence = 0.9f,
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = 0.9f,
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }

  // Invalid MotionClassification
  {
    auto mock_ptr = std::make_unique<MockSemanticProvider>();
    auto* mock_raw = mock_ptr.get();
    auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
    SemanticFrameResult malformed{.frame_seq = 1, .observations = {SemanticObservation{
        .source_point_index = 0,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.9f,
        .motion_state = static_cast<MotionClassification>(99),
        .motion_confidence = 0.9f,
    }}};
    mock_raw->setPredefinedResult(1, std::move(malformed));
    EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
  }
}

TEST(SemanticPipelineTest, MalformedFrameSeqMismatch) {
  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 2, 2);
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  SemanticFrameResult malformed{.frame_seq = 999, .observations = {}};  // Seq mismatch
  mock_raw->setPredefinedResult(1, std::move(malformed));
  EXPECT_EQ(pipeline->processFrame(pts, 1, {0.0, 0.0}).status, AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED);
}

// ============================================================================
// Section 5: Invalid Geometry Semantic Handling (Section 63)
// ============================================================================

TEST(SemanticPipelineTest, InvalidGeometrySemanticHandling) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // 1 invalid point (NaN) and 3 valid points
  std::vector<PointXYZ> pts = {
      PointXYZ{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f},
      PointXYZ{0.05f, 0.05f, 0.01f},
      PointXYZ{0.06f, 0.06f, 0.01f},
      PointXYZ{0.07f, 0.07f, 0.01f},
  };

  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 0,  // NaN point labeled PEDESTRIAN + CONFIRMED_DYNAMIC
          .semantic_class = SemanticClass::PEDESTRIAN,
          .semantic_confidence = 0.99f,
          .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
          .motion_confidence = 0.99f,
      },
      SemanticObservation{
          .source_point_index = 1,  // Valid point labeled GROUND
          .semantic_class = SemanticClass::GROUND,
          .semantic_confidence = 0.90f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.90f,
      },
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.frame_fused);
  // Observations returned is 2, but semantic observations used is only 1 (geometrically valid)
  EXPECT_EQ(res.semantic_diagnostics.observations_returned, 2u);
  EXPECT_EQ(res.semantic_diagnostics.semantic_observations_used, 1u);

  // Spatial evidence does not have the NaN observation
  EXPECT_EQ(pipeline->semanticFrameEvidence().observationCount(), 1u);
}

// ============================================================================
// Section 6: Sparse Output & Empty Frame (Sections 64, 65)
// ============================================================================

TEST(SemanticPipelineTest, SparseOutputGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.05, 0.15, 0.05, 0.15, 0.0, 0.02, 3, 3);  // 9 points
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 3,
          .semantic_class = SemanticClass::VEHICLE,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.observations_returned, 1u);
  EXPECT_EQ(res.semantic_diagnostics.semantic_observations_used, 1u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 0u);
}

TEST(SemanticPipelineTest, EmptyFrameHandling) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  std::vector<PointXYZ> empty_pts;
  const auto res = pipeline->processFrame(empty_pts, 1, {0.0, 0.0});

  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.semantic_diagnostics.provider_configured);
  EXPECT_FALSE(res.semantic_diagnostics.provider_invoked);
  EXPECT_EQ(mock_raw->inferCallCount(), 0u);
}

// ============================================================================
// Section 7: Local Semantic & Dynamic Golden (Sections 46, 47)
// ============================================================================

TEST(SemanticPipelineTest, LocalSemanticGoldenPedestrianForcesFineResolution) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  // Construct flat, low-complexity points across two different L1 children within root L3(0,0)
  // Child 0: [0.01, 0.04] (in child (0,0))
  // Child 1: [0.11, 0.14] (in child (1,0))
  auto pts_c0 = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  auto pts_c1 = makePointsInBox(0.11, 0.14, 0.01, 0.04, 0.0, 0.0, 2, 2);

  std::vector<PointXYZ> all_pts;
  all_pts.insert(all_pts.end(), pts_c0.begin(), pts_c0.end());
  all_pts.insert(all_pts.end(), pts_c1.begin(), pts_c1.end());

  // Mark child 0 point with PEDESTRIAN (SAFETY_CRITICAL)
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::PEDESTRIAN,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(all_pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  // Check decision records: at least one region must have SEMANTIC_RELEVANCE reason
  bool found_semantic_reason = false;
  for (const auto& dec : res.decision_records) {
    if (dec.reasons.has(ResolutionReason::SEMANTIC_RELEVANCE)) {
      found_semantic_reason = true;
      EXPECT_EQ(dec.requested_level, avr::spatial::L0);
    }
  }
  EXPECT_TRUE(found_semantic_reason);
}

TEST(SemanticPipelineTest, LocalDynamicGoldenConfirmedForcesFineResolution) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::UNKNOWN,
          .semantic_confidence = 0.0f,
          .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.confirmed_dynamic_points, 1u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 1u);

  bool found_dynamic_reason = false;
  for (const auto& dec : res.decision_records) {
    if (dec.reasons.has(ResolutionReason::DYNAMIC_RELEVANCE)) {
      found_dynamic_reason = true;
      EXPECT_EQ(dec.requested_level, avr::spatial::L0);
    }
  }
  EXPECT_TRUE(found_dynamic_reason);
}

// ============================================================================
// Section 8: Classification vs Motion Independence (Sections 48-51)
// ============================================================================

TEST(SemanticPipelineTest, StaticVehicleGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::VEHICLE,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 0u);

  // Points fused into map!
  EXPECT_GT(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(SemanticPipelineTest, MovingUnknownClassGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  // All 4 points are CONFIRMED_DYNAMIC with UNKNOWN semantic class
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{0, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CONFIRMED_DYNAMIC, 0.95f},
      SemanticObservation{1, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CONFIRMED_DYNAMIC, 0.95f},
      SemanticObservation{2, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CONFIRMED_DYNAMIC, 0.95f},
      SemanticObservation{3, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CONFIRMED_DYNAMIC, 0.95f},
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 4u);

  // Dynamic points are excluded from static map fusion -> 0 static active payload cells!
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(SemanticPipelineTest, LowMotionConfidenceGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  // Motion is CONFIRMED_DYNAMIC but confidence is 0.50f (below default 0.70f threshold)
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{0, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CONFIRMED_DYNAMIC, 0.50f},
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.confirmed_dynamic_points, 0u);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 0u);
  // Remains STATIC_FUSABLE and enters map
  EXPECT_GT(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

TEST(SemanticPipelineTest, CandidateMotionGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 2, 2);
  // CANDIDATE motion with high confidence
  SemanticFrameResult sem{.frame_seq = 1, .observations = {
      SemanticObservation{0, SemanticClass::UNKNOWN, 0.0f, MotionClassification::CANDIDATE, 0.95f},
  }};
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 0u);
  // Remains STATIC_FUSABLE
  EXPECT_GT(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

// ============================================================================
// Section 9: Static Map Exclusion & Dynamic-Only Frame (Sections 52, 53)
// ============================================================================

TEST(SemanticPipelineTest, StaticMapExclusionGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  // 15 points total: 10 static, 5 dynamic
  auto pts_static = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 5, 2);   // 10 pts
  auto pts_dynamic = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 5, 1);  // 5 pts

  std::vector<PointXYZ> all_pts;
  all_pts.insert(all_pts.end(), pts_static.begin(), pts_static.end());
  all_pts.insert(all_pts.end(), pts_dynamic.begin(), pts_dynamic.end());

  SemanticFrameResult sem{.frame_seq = 1, .observations = {}};
  for (uint32_t i = 10; i < 15; ++i) {
    sem.observations.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::UNKNOWN,
        .semantic_confidence = 0.0f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(all_pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 5u);

  // Decision geometry sees all 15 points
  EXPECT_EQ(pipeline->frameEvidence().statistics().accepted_points, 15u);

  // Check persistent map payload count: sum across all active cells is exactly 10!
  uint64_t total_static_payload_points = 0;
  for (const auto& [leaf_key, payload] : pipeline->map().activePayloadSnapshot()) {
    total_static_payload_points += payload.observationCount();
  }
  EXPECT_EQ(total_static_payload_points, 10u);
}

TEST(SemanticPipelineTest, DynamicOnlyFrameGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 3, 3);  // 9 pts
  SemanticFrameResult sem{.frame_seq = 1, .observations = {}};
  for (uint32_t i = 0; i < 9; ++i) {
    sem.observations.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::UNKNOWN,
        .semantic_confidence = 0.0f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(1, std::move(sem));

  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_TRUE(res.frame_fused);
  EXPECT_EQ(res.semantic_diagnostics.dynamic_excluded_points, 9u);
  EXPECT_EQ(pipeline->processingGeneration(), 1u);

  // Persistent map has 0 static active payload cells
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);
}

// ============================================================================
// Section 10: Dynamic Historical Replay Golden (Section 54)
// ============================================================================

TEST(SemanticPipelineTest, DynamicHistoricalReplayGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  PersistentAdaptivePipelineConfig cfg{};
  cfg.replay_cache_config.max_frames = 10;
  cfg.policy_config.confirmed_dynamic_target = avr::spatial::L3;
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // Frame 1: Coarse flat observation at 70m distance with 10 static points and 5 dynamic points
  auto pts_static = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 5, 2);   // 10 pts
  auto pts_dynamic = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.0, 5, 1);  // 5 pts
  std::vector<PointXYZ> f1_pts;
  f1_pts.insert(f1_pts.end(), pts_static.begin(), pts_static.end());
  f1_pts.insert(f1_pts.end(), pts_dynamic.begin(), pts_dynamic.end());

  SemanticFrameResult sem1{.frame_seq = 1, .observations = {}};
  for (uint32_t i = 10; i < 15; ++i) {
    sem1.observations.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::UNKNOWN,
        .semantic_confidence = 0.0f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(1, std::move(sem1));

  const auto res1 = pipeline->processFrame(f1_pts, 1, {0.0, 70.0});
  ASSERT_TRUE(res1.isSuccess());
  ASSERT_EQ(res1.semantic_diagnostics.dynamic_excluded_points, 5u);

  // Frame 2: Safety-critical pedestrian close to sensor forcing immediate REFINE_NOW and direct parent replay
  auto f2_pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.05, 4, 4);
  SemanticFrameResult sem2{.frame_seq = 2, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::PEDESTRIAN,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(2, std::move(sem2));

  const auto res2 = pipeline->processFrame(f2_pts, 2, {0.0, 0.0});
  ASSERT_TRUE(res2.isSuccess());
  EXPECT_GT(res2.splits_applied, 0u);
  EXPECT_GT(res2.replay_attempts, 0u);
  EXPECT_GT(res2.replay_complete, 0u);

  // Validate that no dynamic points entered the reconstructed historical payload
  ASSERT_TRUE(pipeline->map().validateInvariants().is_valid);
}

// ============================================================================
// Section 11: Geometry-Only Bitwise Regression & All-Unknown Equivalence (Sections 73, 74)
// ============================================================================

TEST(SemanticPipelineTest, GeometryOnlyBitwiseRegression) {
  // Pipeline A: No provider configured
  PersistentAdaptivePipelineConfig cfg{};
  auto pipelineA = PersistentAdaptiveFramePipeline::create(cfg);

  // Pipeline B: Provider returns all UNKNOWN with zero confidences
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipelineB = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));

  mock_raw->setCallback([](std::span<const PointXYZ> pts, uint64_t seq, avr::spatial::WorldPoint2D) {
    SemanticFrameResult sem{.frame_seq = seq, .observations = {}};
    for (uint32_t i = 0; i < pts.size(); ++i) {
      sem.observations.push_back(SemanticObservation{
          .source_point_index = i,
          .semantic_class = SemanticClass::UNKNOWN,
          .semantic_confidence = 0.0f,
          .motion_state = MotionClassification::UNKNOWN,
          .motion_confidence = 0.0f,
      });
    }
    return sem;
  });

  auto pts1 = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.02, 3, 3);
  auto pts2 = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.50, 4, 4);

  const auto resA1 = pipelineA->processFrame(pts1, 1, {0.0, 0.0});
  const auto resB1 = pipelineB->processFrame(pts1, 1, {0.0, 0.0});

  EXPECT_EQ(resA1.status, resB1.status);
  EXPECT_EQ(resA1.decision_records.size(), resB1.decision_records.size());
  EXPECT_EQ(resA1.structural_action_records.size(), resB1.structural_action_records.size());
  EXPECT_EQ(pipelineA->map().statistics().payload_stats.active_payload_count,
            pipelineB->map().statistics().payload_stats.active_payload_count);

  const auto resA2 = pipelineA->processFrame(pts2, 2, {0.0, 0.0});
  const auto resB2 = pipelineB->processFrame(pts2, 2, {0.0, 0.0});

  EXPECT_EQ(resA2.status, resB2.status);
  EXPECT_EQ(resA2.splits_applied, resB2.splits_applied);
  EXPECT_EQ(pipelineA->map().statistics().payload_stats.active_payload_count,
            pipelineB->map().statistics().payload_stats.active_payload_count);
}

// ============================================================================
// Section 12: Determinism (Section 96)
// ============================================================================

TEST(SemanticPipelineTest, DeterminismTwoPipelines) {
  auto mockA = std::make_unique<MockSemanticProvider>();
  auto mockB = std::make_unique<MockSemanticProvider>();

  auto callback = [](std::span<const PointXYZ> pts, uint64_t seq, avr::spatial::WorldPoint2D) {
    SemanticFrameResult sem{.frame_seq = seq, .observations = {}};
    for (uint32_t i = 0; i < pts.size(); ++i) {
      if (i % 3 == 0) {
        sem.observations.push_back(SemanticObservation{
            .source_point_index = i,
            .semantic_class = SemanticClass::PEDESTRIAN,
            .semantic_confidence = 0.95f,
            .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
            .motion_confidence = 0.95f,
        });
      }
    }
    return sem;
  };

  mockA->setCallback(callback);
  mockB->setCallback(callback);

  PersistentAdaptivePipelineConfig cfg{};
  auto pipeA = PersistentAdaptiveFramePipeline::create(cfg, std::move(mockA));
  auto pipeB = PersistentAdaptiveFramePipeline::create(cfg, std::move(mockB));

  for (uint64_t seq = 1; seq <= 20; ++seq) {
    auto pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 0.0, static_cast<double>(seq) * 0.01, 3, 3);
    const auto resA = pipeA->processFrame(pts, seq, {0.0, 0.0});
    const auto resB = pipeB->processFrame(pts, seq, {0.0, 0.0});

    ASSERT_EQ(resA.status, resB.status);
    ASSERT_EQ(resA.decision_records.size(), resB.decision_records.size());
    ASSERT_EQ(resA.structural_action_records.size(), resB.structural_action_records.size());
    ASSERT_EQ(pipeA->map().statistics(), pipeB->map().statistics());
  }
}

// ============================================================================
// Section 13: Provider Call Count & Coarsening Interaction (Sections 71, 78, 79)
// ============================================================================

TEST(SemanticPipelineTest, ProviderCalledAtMostOncePerFrame) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  // Complex scene that triggers prospective root creation and multiple splits
  auto pts = makePointsInBox(0.01, 0.08, 0.01, 0.08, 0.0, 0.50, 4, 4);
  const auto res = pipeline->processFrame(pts, 1, {0.0, 0.0});

  EXPECT_TRUE(res.isSuccess());
  EXPECT_GT(res.splits_applied, 1u);
  // Provider must be called exactly ONCE; recursive child M4 queries semantic evidence without reinvoking provider
  EXPECT_EQ(mock_raw->inferCallCount(), 1u);
}

TEST(SemanticPipelineTest, ConfirmedHazardPreventsCoarsening) {
  PersistentAdaptivePipelineConfig cfg{};
  cfg.coarsen_group_max_age_generations = 2;
  cfg.hysteresis_config.coarsen_stable_observations = 1;  // 1 stable observation to reach ready

  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // Frame 1: Refines to fine level
  auto pts = makePointsInBox(0.01, 0.04, 0.01, 0.04, 0.0, 0.01, 4, 4);
  mock_raw->setPredefinedResult(10, SemanticFrameResult{.frame_seq = 10, .observations = {}});
  EXPECT_TRUE(pipeline->processFrame(pts, 10, {0.0, 0.0}).isSuccess());

  // Frame 2: Distance 70m would normally prefer coarse resolution and merge.
  // But point 0 receives a safety-critical PEDESTRIAN observation!
  SemanticFrameResult sem20{.frame_seq = 20, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::PEDESTRIAN,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(20, std::move(sem20));

  const auto res2 = pipeline->processFrame(pts, 20, {70.0, 0.0});
  EXPECT_TRUE(res2.isSuccess());
  // The safety-critical pedestrian prevents direct merge coarsening
  EXPECT_EQ(res2.direct_merges_applied, 0u);
}

// ============================================================================
// M10B2-R1: Contract Repair & Comprehensive Localization Verification Tests
// ============================================================================

TEST(SemanticPipelineTest, SourceIndexWidthGolden) {
  static_assert(std::is_same_v<decltype(SemanticObservation::source_point_index), uint32_t>,
                "source_point_index must be uint32_t");
  static_assert(sizeof(SemanticObservation::source_point_index) == 4,
                "source_point_index must be 4 bytes");
  static_assert(sizeof(RecentObservationCache::PointIndex) == 4,
                "PointIndex in RecentObservationCache must be 4 bytes");
  EXPECT_EQ(sizeof(SemanticObservation::source_point_index), 4u);
}

TEST(SemanticPipelineTest, PipelineCacheRetentionGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(
      PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // Raw frame: static A, dynamic B, NaN C, static D, dynamic E (all in L3 cell [0.0, 0.40))
  const std::vector<PointXYZ> raw_pts = {
      PointXYZ{0.05, 0.05, 0.5},                                     // A (index 0)
      PointXYZ{0.10, 0.10, 0.5},                                     // B (index 1)
      PointXYZ{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},  // C (index 2)
      PointXYZ{0.20, 0.20, 0.5},                                     // D (index 3)
      PointXYZ{0.30, 0.30, 0.5},                                     // E (index 4)
  };

  SemanticFrameResult sem{.frame_seq = 10, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::ROAD,
          .semantic_confidence = 0.90f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.90f,
      },
      SemanticObservation{
          .source_point_index = 1,
          .semantic_class = SemanticClass::VEHICLE,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
          .motion_confidence = 0.95f,
      },
      SemanticObservation{
          .source_point_index = 3,
          .semantic_class = SemanticClass::ROAD,
          .semantic_confidence = 0.90f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.90f,
      },
      SemanticObservation{
          .source_point_index = 4,
          .semantic_class = SemanticClass::VEHICLE,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
          .motion_confidence = 0.95f,
      },
  }};
  mock_raw->setPredefinedResult(10, std::move(sem));

  const auto res = pipeline->processFrame(raw_pts, 10, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  const auto* cached = pipeline->replayCache().findFrame(10);
  ASSERT_NE(cached, nullptr);
  EXPECT_EQ(cached->rawPoints().size(), 5u);
  EXPECT_EQ(cached->rawPoints()[0], raw_pts[0]);
  EXPECT_EQ(cached->rawPoints()[1], raw_pts[1]);
  EXPECT_TRUE(std::isnan(cached->rawPoints()[2].x));
  EXPECT_EQ(cached->rawPoints()[3], raw_pts[3]);
  EXPECT_EQ(cached->rawPoints()[4], raw_pts[4]);

  // Routing validation
  EXPECT_EQ(cached->routingAt(0), PointRouting::STATIC_FUSABLE);
  EXPECT_EQ(cached->routingAt(1), PointRouting::DYNAMIC_EXCLUDED);
  EXPECT_EQ(cached->routingAt(2), PointRouting::STATIC_FUSABLE);
  EXPECT_EQ(cached->routingAt(3), PointRouting::STATIC_FUSABLE);
  EXPECT_EQ(cached->routingAt(4), PointRouting::DYNAMIC_EXCLUDED);

  // Region queries
  const auto l3_key = avr::spatial::CellKey::fromWorldPoint({0.05, 0.05}, avr::spatial::L3);
  ASSERT_TRUE(l3_key.has_value());

  std::vector<RecentObservationCache::PointIndex> all_indices;
  EXPECT_TRUE(cached->collectObservationIndices(*l3_key, all_indices));
  EXPECT_EQ(all_indices.size(), 4u);  // A, B, D, E (geometrically valid points)

  std::vector<RecentObservationCache::PointIndex> static_indices;
  EXPECT_TRUE(cached->collectStaticFusableObservationIndices(*l3_key, static_indices));
  EXPECT_EQ(static_indices.size(), 2u);  // A, D only
  EXPECT_EQ(static_indices[0], 0u);
  EXPECT_EQ(static_indices[1], 3u);
}

TEST(SemanticPipelineTest, SemanticLocalizationAdversarial_L2) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;

  // L3 Cell at (0, 0): [0.0, 0.40) x [0.0, 0.40)
  // Four L2 children:
  // Child 0: [0.0, 0.20) x [0.0, 0.20) -> pedestrian
  // Child 1: [0.20, 0.40) x [0.0, 0.20) -> ground
  // Child 2: [0.0, 0.20) x [0.20, 0.40) -> ground
  // Child 3: [0.20, 0.40) x [0.20, 0.40) -> ground
  const auto c0_opt = avr::spatial::worldToGridCoord({0.05, 0.05});
  const auto c1_opt = avr::spatial::worldToGridCoord({0.25, 0.05});
  const auto c2_opt = avr::spatial::worldToGridCoord({0.05, 0.25});
  const auto c3_opt = avr::spatial::worldToGridCoord({0.25, 0.25});
  ASSERT_TRUE(c0_opt && c1_opt && c2_opt && c3_opt);

  evidence.addObservation(0, *c0_opt, SemanticClass::PEDESTRIAN, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(1, *c1_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(2, *c2_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(3, *c3_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);

  const auto k0 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L2);
  const auto k1 = avr::spatial::CellKey::fromGridCoord(*c1_opt, avr::spatial::L2);
  const auto k2 = avr::spatial::CellKey::fromGridCoord(*c2_opt, avr::spatial::L2);
  const auto k3 = avr::spatial::CellKey::fromGridCoord(*c3_opt, avr::spatial::L2);
  ASSERT_TRUE(k0 && k1 && k2 && k3);

  // Distinct L2 keys
  EXPECT_NE(*k0, *k1);
  EXPECT_NE(*k0, *k2);
  EXPECT_NE(*k0, *k3);

  // Child 0 has safety-critical hazard; siblings 1, 2, 3 must be NONE!
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k0), SemanticRelevance::SAFETY_CRITICAL);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k1), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k2), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k3), SemanticRelevance::NONE);

  // L3 parent covers all 4 children and reports SAFETY_CRITICAL
  const auto parent_l3 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L3);
  ASSERT_TRUE(parent_l3);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*parent_l3), SemanticRelevance::SAFETY_CRITICAL);
}

TEST(SemanticPipelineTest, SemanticLocalizationAdversarial_L1) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;

  // Inside L2 child [0.0, 0.20) x [0.0, 0.20):
  // L1 Child 0: [0.0, 0.10) x [0.0, 0.10) -> pedestrian
  // L1 Child 1: [0.10, 0.20) x [0.0, 0.10) -> ground
  // L1 Child 2: [0.0, 0.10) x [0.10, 0.20) -> ground
  // L1 Child 3: [0.10, 0.20) x [0.10, 0.20) -> ground
  const auto c0_opt = avr::spatial::worldToGridCoord({0.05, 0.05});
  const auto c1_opt = avr::spatial::worldToGridCoord({0.15, 0.05});
  const auto c2_opt = avr::spatial::worldToGridCoord({0.05, 0.15});
  const auto c3_opt = avr::spatial::worldToGridCoord({0.15, 0.15});
  ASSERT_TRUE(c0_opt && c1_opt && c2_opt && c3_opt);

  evidence.addObservation(0, *c0_opt, SemanticClass::PEDESTRIAN, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(1, *c1_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(2, *c2_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(3, *c3_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);

  const auto k0 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L1);
  const auto k1 = avr::spatial::CellKey::fromGridCoord(*c1_opt, avr::spatial::L1);
  const auto k2 = avr::spatial::CellKey::fromGridCoord(*c2_opt, avr::spatial::L1);
  const auto k3 = avr::spatial::CellKey::fromGridCoord(*c3_opt, avr::spatial::L1);
  ASSERT_TRUE(k0 && k1 && k2 && k3);

  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k0), SemanticRelevance::SAFETY_CRITICAL);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k1), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k2), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k3), SemanticRelevance::NONE);
}

TEST(SemanticPipelineTest, SemanticLocalizationAdversarial_L0) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;

  // Inside L1 child [0.0, 0.10) x [0.0, 0.10):
  // L0 Cell 0: [0.0, 0.05) x [0.0, 0.05) -> pedestrian
  // L0 Cell 1: [0.05, 0.10) x [0.0, 0.05) -> ground
  const auto c0_opt = avr::spatial::worldToGridCoord({0.02, 0.02});
  const auto c1_opt = avr::spatial::worldToGridCoord({0.07, 0.02});
  ASSERT_TRUE(c0_opt && c1_opt);

  evidence.addObservation(0, *c0_opt, SemanticClass::PEDESTRIAN, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(1, *c1_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);

  const auto k0 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L0);
  const auto k1 = avr::spatial::CellKey::fromGridCoord(*c1_opt, avr::spatial::L0);
  ASSERT_TRUE(k0 && k1);

  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k0), SemanticRelevance::SAFETY_CRITICAL);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k1), SemanticRelevance::NONE);
}

TEST(SemanticPipelineTest, SemanticLocalizationNegativeCoordinates) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;

  // Negative coordinates: L3 parent at [-0.40, 0.0) x [-0.40, 0.0)
  const auto c0_opt = avr::spatial::worldToGridCoord({-0.35, -0.35});
  const auto c1_opt = avr::spatial::worldToGridCoord({-0.15, -0.35});
  const auto c2_opt = avr::spatial::worldToGridCoord({-0.35, -0.15});
  const auto c3_opt = avr::spatial::worldToGridCoord({-0.15, -0.15});
  ASSERT_TRUE(c0_opt && c1_opt && c2_opt && c3_opt);

  evidence.addObservation(0, *c0_opt, SemanticClass::PEDESTRIAN, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(1, *c1_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(2, *c2_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(3, *c3_opt, SemanticClass::GROUND, 0.95f,
                          MotionClassification::STATIC, 0.95f, config);

  const auto k0 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L2);
  const auto k1 = avr::spatial::CellKey::fromGridCoord(*c1_opt, avr::spatial::L2);
  const auto k2 = avr::spatial::CellKey::fromGridCoord(*c2_opt, avr::spatial::L2);
  const auto k3 = avr::spatial::CellKey::fromGridCoord(*c3_opt, avr::spatial::L2);
  ASSERT_TRUE(k0 && k1 && k2 && k3);

  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k0), SemanticRelevance::SAFETY_CRITICAL);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k1), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k2), SemanticRelevance::NONE);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*k3), SemanticRelevance::NONE);

  const auto parent_l3 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L3);
  ASSERT_TRUE(parent_l3);
  EXPECT_EQ(evidence.evaluateSemanticRelevance(*parent_l3), SemanticRelevance::SAFETY_CRITICAL);
}

TEST(SemanticPipelineTest, DynamicLocalizationAdversarial) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;

  const auto c0_opt = avr::spatial::worldToGridCoord({0.05, 0.05});
  const auto c1_opt = avr::spatial::worldToGridCoord({0.25, 0.05});
  const auto c2_opt = avr::spatial::worldToGridCoord({0.05, 0.25});
  const auto c3_opt = avr::spatial::worldToGridCoord({0.25, 0.25});
  ASSERT_TRUE(c0_opt && c1_opt && c2_opt && c3_opt);

  evidence.addObservation(0, *c0_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::CONFIRMED_DYNAMIC, 0.95f, config);
  evidence.addObservation(1, *c1_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(2, *c2_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::STATIC, 0.95f, config);
  evidence.addObservation(3, *c3_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::STATIC, 0.95f, config);

  const auto k0 = avr::spatial::CellKey::fromGridCoord(*c0_opt, avr::spatial::L2);
  const auto k1 = avr::spatial::CellKey::fromGridCoord(*c1_opt, avr::spatial::L2);
  const auto k2 = avr::spatial::CellKey::fromGridCoord(*c2_opt, avr::spatial::L2);
  const auto k3 = avr::spatial::CellKey::fromGridCoord(*c3_opt, avr::spatial::L2);
  ASSERT_TRUE(k0 && k1 && k2 && k3);

  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k0), DynamicRelevance::CONFIRMED);
  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k1), DynamicRelevance::NONE);
  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k2), DynamicRelevance::NONE);
  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k3), DynamicRelevance::NONE);
}

TEST(SemanticPipelineTest, MotionConfidenceGate_LowConfidenceCandidate) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;
  config.min_motion_confidence = 0.70f;

  const auto c_opt = avr::spatial::worldToGridCoord({1.0, 1.0});
  ASSERT_TRUE(c_opt);

  // CANDIDATE with low confidence (0.50f < 0.70f) -> DynamicRelevance::NONE
  evidence.addObservation(0, *c_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::CANDIDATE, 0.50f, config);

  const auto k = avr::spatial::CellKey::fromGridCoord(*c_opt, avr::spatial::L0);
  ASSERT_TRUE(k);
  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k), DynamicRelevance::NONE);
}

TEST(SemanticPipelineTest, MotionConfidenceGate_HighConfidenceCandidate) {
  SemanticFrameEvidence evidence;
  SemanticPolicyConfig config;
  config.min_motion_confidence = 0.70f;

  const auto c_opt = avr::spatial::worldToGridCoord({1.0, 1.0});
  ASSERT_TRUE(c_opt);

  // CANDIDATE with high confidence (0.85f >= 0.70f) -> DynamicRelevance::CANDIDATE
  evidence.addObservation(0, *c_opt, SemanticClass::UNKNOWN, 0.0f,
                          MotionClassification::CANDIDATE, 0.85f, config);

  const auto k = avr::spatial::CellKey::fromGridCoord(*c_opt, avr::spatial::L0);
  ASSERT_TRUE(k);
  EXPECT_EQ(evidence.evaluateDynamicRelevance(*k), DynamicRelevance::CANDIDATE);
}

TEST(SemanticPipelineTest, SparseSafetyGolden) {
  const auto controller_opt = ResolutionController::create(ResolutionPolicyConfig{});
  ASSERT_TRUE(controller_opt.has_value());
  ResolutionSignals signals{};
  signals.distance_m = 5.0;
  signals.observation_count = 1;
  signals.density_score = 0.01;  // Below normal continuous density gate
  signals.semantic_relevance = SemanticRelevance::SAFETY_CRITICAL;
  signals.dynamic_relevance = DynamicRelevance::NONE;
  ASSERT_TRUE(signals.isValid());

  const auto decision = controller_opt->decide(signals);
  ASSERT_TRUE(decision.has_value());
  EXPECT_TRUE(decision->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
  EXPECT_EQ(decision->requested_level, avr::spatial::L0);
}

TEST(SemanticPipelineTest, ConfirmedDynamicSparseGolden) {
  const auto controller_opt = ResolutionController::create(ResolutionPolicyConfig{});
  ASSERT_TRUE(controller_opt.has_value());
  ResolutionSignals signals{};
  signals.distance_m = 5.0;
  signals.observation_count = 1;
  signals.density_score = 0.01;  // Below normal continuous density gate
  signals.semantic_relevance = SemanticRelevance::NONE;
  signals.dynamic_relevance = DynamicRelevance::CONFIRMED;
  ASSERT_TRUE(signals.isValid());

  const auto decision = controller_opt->decide(signals);
  ASSERT_TRUE(decision.has_value());
  EXPECT_TRUE(decision->reasons.has(ResolutionReason::DYNAMIC_RELEVANCE));
  EXPECT_EQ(decision->requested_level, avr::spatial::L0);
}

TEST(SemanticPipelineTest, CacheStatsEndToEnd) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(
      PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // 10 static, 5 dynamic, 2 invalid = 17 points
  std::vector<PointXYZ> raw_pts;
  raw_pts.reserve(17);
  for (size_t i = 0; i < 10; ++i) {
    raw_pts.push_back(PointXYZ{1.0 + 0.01 * static_cast<double>(i), 1.0, 0.5});
  }
  for (size_t i = 0; i < 5; ++i) {
    raw_pts.push_back(PointXYZ{1.2 + 0.01 * static_cast<double>(i), 1.2, 0.5});
  }
  raw_pts.push_back(PointXYZ{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  raw_pts.push_back(PointXYZ{0.0, std::numeric_limits<double>::infinity(), 0.0});

  std::vector<SemanticObservation> obs;
  for (uint32_t i = 0; i < 10; ++i) {
    obs.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::ROAD,
        .semantic_confidence = 0.90f,
        .motion_state = MotionClassification::STATIC,
        .motion_confidence = 0.90f,
    });
  }
  for (uint32_t i = 10; i < 15; ++i) {
    obs.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.95f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(1, SemanticFrameResult{.frame_seq = 1, .observations = std::move(obs)});

  const auto res = pipeline->processFrame(raw_pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());

  const auto* cached = pipeline->replayCache().findFrame(1);
  ASSERT_NE(cached, nullptr);
  const auto& stats = cached->statistics();
  EXPECT_EQ(stats.input_points, 17u);
  EXPECT_EQ(stats.static_fusable_indexed, 10u);
  EXPECT_EQ(stats.dynamic_excluded_indexed, 5u);
  EXPECT_EQ(stats.non_finite_unindexed, 2u);
  EXPECT_EQ(stats.replay_candidates, 15u);
  EXPECT_EQ(cached->routingVector().size(), 17u);
}

TEST(SemanticPipelineTest, DynamicOnlyCacheGolden) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(
      PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  std::vector<PointXYZ> raw_pts;
  std::vector<SemanticObservation> obs;
  for (uint32_t i = 0; i < 8; ++i) {
    raw_pts.push_back(PointXYZ{0.05 * static_cast<double>(i), 0.05, 0.5});
    obs.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.95f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(1, SemanticFrameResult{.frame_seq = 1, .observations = std::move(obs)});

  const auto res = pipeline->processFrame(raw_pts, 1, {0.0, 0.0});
  EXPECT_TRUE(res.isSuccess());
  EXPECT_EQ(pipeline->map().statistics().payload_stats.active_payload_count, 0u);

  const auto* cached = pipeline->replayCache().findFrame(1);
  ASSERT_NE(cached, nullptr);
  EXPECT_EQ(cached->rawPoints().size(), 8u);
  for (uint32_t i = 0; i < 8; ++i) {
    EXPECT_EQ(cached->routingAt(i), PointRouting::DYNAMIC_EXCLUDED);
  }

  const auto l3_key = avr::spatial::CellKey::fromWorldPoint({0.05, 0.05}, avr::spatial::L3);
  ASSERT_TRUE(l3_key);
  std::vector<RecentObservationCache::PointIndex> all_idx;
  EXPECT_TRUE(cached->collectObservationIndices(*l3_key, all_idx));
  EXPECT_EQ(all_idx.size(), 8u);

  std::vector<RecentObservationCache::PointIndex> static_idx;
  EXPECT_TRUE(cached->collectStaticFusableObservationIndices(*l3_key, static_idx));
  EXPECT_EQ(static_idx.size(), 0u);
}

TEST(SemanticPipelineTest, SemanticFrameEvidenceCurrentFrameReset) {
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(
      PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  const std::vector<PointXYZ> pts = {{1.0, 1.0, 0.5}};
  mock_raw->setPredefinedResult(1, SemanticFrameResult{.frame_seq = 1, .observations = {
      SemanticObservation{
          .source_point_index = 0,
          .semantic_class = SemanticClass::PEDESTRIAN,
          .semantic_confidence = 0.95f,
          .motion_state = MotionClassification::STATIC,
          .motion_confidence = 0.95f,
      },
  }});

  EXPECT_TRUE(pipeline->processFrame(pts, 1, {0.0, 0.0}).isSuccess());
  EXPECT_EQ(pipeline->semanticFrameEvidence().observationCount(), 1u);

  // Frame 2: Provider returns empty observations
  mock_raw->setPredefinedResult(2, SemanticFrameResult{.frame_seq = 2, .observations = {}});
  EXPECT_TRUE(pipeline->processFrame(pts, 2, {0.0, 0.0}).isSuccess());

  // Previous evidence must be fully cleared; zero semantic ghost state
  EXPECT_EQ(pipeline->semanticFrameEvidence().observationCount(), 0u);
  const auto l0_key = avr::spatial::CellKey::fromWorldPoint({1.0, 1.0}, avr::spatial::L0);
  ASSERT_TRUE(l0_key);
  EXPECT_EQ(pipeline->semanticFrameEvidence().evaluateSemanticRelevance(*l0_key),
            SemanticRelevance::NONE);
}

TEST(SemanticPipelineTest, StrongAllUnknownEquivalence) {
  // Pipeline A: No provider
  auto pipeA = PersistentAdaptiveFramePipeline::create(PersistentAdaptivePipelineConfig{});

  // Pipeline B: Provider returns all UNKNOWN with 0.0 confidence
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeB = PersistentAdaptiveFramePipeline::create(
      PersistentAdaptivePipelineConfig{}, std::move(mock_ptr));

  for (uint64_t seq = 1; seq <= 5; ++seq) {
    const auto pts = makePointsInBox(0.01 * static_cast<double>(seq), 0.05,
                                     0.01, 0.05, 0.0, 0.20, 3, 3);
    std::vector<SemanticObservation> obs;
    for (uint32_t i = 0; i < pts.size(); ++i) {
      obs.push_back(SemanticObservation{
          .source_point_index = i,
          .semantic_class = SemanticClass::UNKNOWN,
          .semantic_confidence = 0.0f,
          .motion_state = MotionClassification::UNKNOWN,
          .motion_confidence = 0.0f,
      });
    }
    mock_raw->setPredefinedResult(seq, SemanticFrameResult{.frame_seq = seq, .observations = std::move(obs)});

    const auto resA = pipeA->processFrame(pts, seq, {0.0, 0.0});
    const auto resB = pipeB->processFrame(pts, seq, {0.0, 0.0});

    EXPECT_TRUE(resA.isSuccess());
    EXPECT_TRUE(resB.isSuccess());
    EXPECT_EQ(resA.status, resB.status);
    EXPECT_EQ(resA.frame_fused, resB.frame_fused);
    EXPECT_EQ(resA.structure_changed, resB.structure_changed);
    EXPECT_EQ(resA.decision_records.size(), resB.decision_records.size());
    EXPECT_EQ(resA.structural_action_records.size(), resB.structural_action_records.size());
    EXPECT_EQ(pipeA->map().statistics(), pipeB->map().statistics());
    EXPECT_EQ(pipeA->processingGeneration(), pipeB->processingGeneration());

    const auto* cachedA = pipeA->replayCache().findFrame(seq);
    const auto* cachedB = pipeB->replayCache().findFrame(seq);
    ASSERT_NE(cachedA, nullptr);
    ASSERT_NE(cachedB, nullptr);
    EXPECT_EQ(cachedA->rawPoints().size(), cachedB->rawPoints().size());
    EXPECT_EQ(cachedA->routingVector(), cachedB->routingVector());
  }
}

TEST(SemanticPipelineTest, DynamicPointDoesNotTouchStaticShadow) {
  // Setup pipeline with coarsening hysteresis
  PersistentAdaptivePipelineConfig cfg{};
  cfg.coarsen_group_max_age_generations = 10;
  auto mock_ptr = std::make_unique<MockSemanticProvider>();
  auto* mock_raw = mock_ptr.get();
  auto pipeline = PersistentAdaptiveFramePipeline::create(cfg, std::move(mock_ptr));
  ASSERT_NE(pipeline, nullptr);

  // Frame 1: static points refine region
  auto pts = makePointsInBox(0.01, 0.05, 0.01, 0.05, 1.0, 1.1, 3, 3);
  mock_raw->setPredefinedResult(1, SemanticFrameResult{.frame_seq = 1, .observations = {}});
  EXPECT_TRUE(pipeline->processFrame(pts, 1, {0.0, 0.0}).isSuccess());

  const auto map_stats1 = pipeline->map().statistics();

  // Frame 2: Process ONLY confirmed dynamic points in the same region
  std::vector<SemanticObservation> dynamic_obs;
  for (uint32_t i = 0; i < pts.size(); ++i) {
    dynamic_obs.push_back(SemanticObservation{
        .source_point_index = i,
        .semantic_class = SemanticClass::VEHICLE,
        .semantic_confidence = 0.95f,
        .motion_state = MotionClassification::CONFIRMED_DYNAMIC,
        .motion_confidence = 0.95f,
    });
  }
  mock_raw->setPredefinedResult(2, SemanticFrameResult{.frame_seq = 2, .observations = std::move(dynamic_obs)});

  const auto res2 = pipeline->processFrame(pts, 2, {0.0, 0.0});
  EXPECT_TRUE(res2.isSuccess());

  // Static payload & shadow statistics must remain identical
  const auto map_stats2 = pipeline->map().statistics();
  EXPECT_EQ(map_stats1.payload_stats, map_stats2.payload_stats);
}

TEST(SemanticPipelineTest, AdaptivePipelineStatusFrozenValuesLocked) {
  // Frozen 93d4eb7 statuses:
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::SUCCESS) == 0);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::SUCCESS_REPLAY_PENDING) == 1);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::SUCCESS_CACHE_DEGRADED) == 2);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_INPUT) == 3);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE) == 4);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_DECISION) == 5);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_POLICY) == 6);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::REFINEMENT_INTERNAL_ERROR) == 7);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::REPLAY_INTERNAL_ERROR) == 8);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FUSION_FAILED) == 9);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FUSION_FAILED_AFTER_STRUCTURAL_CHANGE) == 10);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::POLICY_COMMIT_INTERNAL_ERROR) == 11);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::POLICY_COMMIT_FATAL) == 12);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::CACHE_COMMIT_INTERNAL_ERROR) == 13);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::GENERATION_EXHAUSTED) == 14);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::POLICY_REVISION_EXHAUSTED) == 15);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::INVARIANT_FAILURE) == 16);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::PIPELINE_FATAL_LATCHED) == 17);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR) == 18);

  // Appended M10B2 semantic statuses:
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED) == 19);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_ERROR) == 20);
  static_assert(static_cast<uint8_t>(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE) == 21);

  // Verify strings
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::SUCCESS), "SUCCESS");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::COARSENING_INTERNAL_ERROR), "COARSENING_INTERNAL_ERROR");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_MALFORMED), "FRAME_REJECTED_SEMANTIC_MALFORMED");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_ERROR), "FRAME_REJECTED_SEMANTIC_ERROR");
  EXPECT_STREQ(adaptivePipelineStatusToString(AdaptivePipelineStatus::FRAME_REJECTED_SEMANTIC_RESOURCE), "FRAME_REJECTED_SEMANTIC_RESOURCE");
}

TEST(SemanticPipelineTest, ResolutionReasonFrozenBitsLocked) {
  // Frozen 93d4eb7 ResolutionReason bits:
  static_assert(static_cast<uint16_t>(ResolutionReason::NONE) == 0);
  static_assert(static_cast<uint16_t>(ResolutionReason::DISTANCE_BASELINE) == (1 << 0));
  static_assert(static_cast<uint16_t>(ResolutionReason::SEMANTIC_RELEVANCE) == (1 << 1));
  static_assert(static_cast<uint16_t>(ResolutionReason::DYNAMIC_RELEVANCE) == (1 << 2));
  static_assert(static_cast<uint16_t>(ResolutionReason::UNCERTAINTY) == (1 << 3));
  static_assert(static_cast<uint16_t>(ResolutionReason::GEOMETRY_COMPLEXITY) == (1 << 4));
  static_assert(static_cast<uint16_t>(ResolutionReason::INSUFFICIENT_EVIDENCE) == (1 << 5));
  static_assert(static_cast<uint16_t>(ResolutionReason::DENSITY_GATE_BLOCKED) == (1 << 6));
}

TEST(SemanticPipelineTest, SemanticClassAndMotionValidation) {
  // Canonical SemanticClass taxonomy (0..7)
  static_assert(static_cast<uint8_t>(SemanticClass::UNKNOWN) == 0);
  static_assert(static_cast<uint8_t>(SemanticClass::GROUND) == 1);
  static_assert(static_cast<uint8_t>(SemanticClass::ROAD) == 2);
  static_assert(static_cast<uint8_t>(SemanticClass::VEGETATION) == 3);
  static_assert(static_cast<uint8_t>(SemanticClass::BUILDING) == 4);
  static_assert(static_cast<uint8_t>(SemanticClass::VEHICLE) == 5);
  static_assert(static_cast<uint8_t>(SemanticClass::PEDESTRIAN) == 6);
  static_assert(static_cast<uint8_t>(SemanticClass::OBSTACLE_OTHER) == 7);

  for (uint8_t i = 0; i <= 7; ++i) {
    EXPECT_TRUE(isValidSemanticClass(static_cast<SemanticClass>(i)));
  }
  EXPECT_FALSE(isValidSemanticClass(static_cast<SemanticClass>(8)));
  EXPECT_FALSE(isValidSemanticClass(static_cast<SemanticClass>(9)));
  EXPECT_FALSE(isValidSemanticClass(static_cast<SemanticClass>(255)));

  // Canonical MotionClassification taxonomy (0..3)
  static_assert(static_cast<uint8_t>(MotionClassification::UNKNOWN) == 0);
  static_assert(static_cast<uint8_t>(MotionClassification::STATIC) == 1);
  static_assert(static_cast<uint8_t>(MotionClassification::CANDIDATE) == 2);
  static_assert(static_cast<uint8_t>(MotionClassification::CONFIRMED_DYNAMIC) == 3);

  for (uint8_t i = 0; i <= 3; ++i) {
    EXPECT_TRUE(isValidMotionClassification(static_cast<MotionClassification>(i)));
  }
  EXPECT_FALSE(isValidMotionClassification(static_cast<MotionClassification>(4)));
  EXPECT_FALSE(isValidMotionClassification(static_cast<MotionClassification>(5)));
  EXPECT_FALSE(isValidMotionClassification(static_cast<MotionClassification>(255)));
}

TEST(SemanticPipelineTest, PointRoutingValuesLocked) {
  static_assert(static_cast<uint8_t>(PointRouting::STATIC_FUSABLE) == 0);
  static_assert(static_cast<uint8_t>(PointRouting::DYNAMIC_EXCLUDED) == 1);
  EXPECT_TRUE(isValidPointRouting(PointRouting::STATIC_FUSABLE));
  EXPECT_TRUE(isValidPointRouting(PointRouting::DYNAMIC_EXCLUDED));
  EXPECT_FALSE(isValidPointRouting(static_cast<PointRouting>(2)));
  EXPECT_FALSE(isValidPointRouting(static_cast<PointRouting>(255)));
}

}  // namespace
}  // namespace avr::mapping
