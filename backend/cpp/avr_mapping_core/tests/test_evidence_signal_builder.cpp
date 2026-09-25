#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "avr_mapping/evidence_signal_builder.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/point_xyz.hpp"
#include "avr_mapping/resolution_controller.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

using namespace avr::mapping;
using namespace avr::spatial;

TEST(EvidenceSignalBuilderTest, DistanceCalculationAndNumericContract) {
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{1.0, 1.0, 0.0}};
  // Sensor origin at (0.0, 0.0)
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  // Cell at (10, 0) at L0 (0.05m width). Center is at (10 * 0.05 + 0.025, 0.025) = (0.525, 0.025)
  const auto key = CellKey::fromCellCoord(CellCoord{.x = 10, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());

  const auto signals_opt = makeResolutionSignals(fe, *key);
  ASSERT_TRUE(signals_opt.has_value());
  const double expected_dist = std::hypot(0.525 - 0.0, 0.025 - 0.0);
  EXPECT_NEAR(signals_opt->distance_m, expected_dist, 1e-9);

  // Region at extreme coordinates where distance hypot would overflow
  const auto far_key = CellKey::fromCellCoord(
      CellCoord{.x = std::numeric_limits<int32_t>::max() / 2,
                .y = std::numeric_limits<int32_t>::max() / 2,
                .level = L0});
  ASSERT_TRUE(far_key.has_value());
  // Place sensor origin far away such that dx, dy = max(), overflowing distance hypot to Inf
  FrameEvidence fe_far;
  fe_far.buildFrame(points, 2,
                    WorldPoint2D{-std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max()});
  const auto far_signals = makeResolutionSignals(fe_far, *far_key);
  // hypot overflows to Inf -> must fail explicitly
  EXPECT_FALSE(far_signals.has_value());
}

TEST(EvidenceSignalBuilderTest, ObservationCountSaturation) {
  // Verify that count saturation correctly clamps to UINT32_MAX
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{0.02, 0.02, 1.0}};
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());
  const auto sig = makeResolutionSignals(fe, *key);
  ASSERT_TRUE(sig.has_value());
  EXPECT_EQ(sig->observation_count, 1);
}

TEST(EvidenceSignalBuilderTest, ExternalCueValidation) {
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{0.02, 0.02, 1.0}};
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());

  // 1. Invalid semantic enum
  const auto sig_bad_sem = makeResolutionSignals(
      fe, *key, static_cast<SemanticRelevance>(99));
  EXPECT_FALSE(sig_bad_sem.has_value());

  // 2. Invalid dynamic enum
  const auto sig_bad_dyn = makeResolutionSignals(
      fe, *key, SemanticRelevance::NONE, static_cast<DynamicRelevance>(99));
  EXPECT_FALSE(sig_bad_dyn.has_value());

  // 3. Out-of-range uncertainty
  const auto sig_bad_unc_high = makeResolutionSignals(
      fe, *key, SemanticRelevance::NONE, DynamicRelevance::NONE, 1.05);
  EXPECT_FALSE(sig_bad_unc_high.has_value());

  const auto sig_bad_unc_low = makeResolutionSignals(
      fe, *key, SemanticRelevance::NONE, DynamicRelevance::NONE, -0.01);
  EXPECT_FALSE(sig_bad_unc_low.has_value());

  const auto sig_bad_unc_nan = makeResolutionSignals(
      fe, *key, SemanticRelevance::NONE, DynamicRelevance::NONE,
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(sig_bad_unc_nan.has_value());
}

TEST(EvidenceSignalBuilderTest, NoDetailFabrication) {
  FrameEvidence fe;
  std::vector<PointXYZ> points = {PointXYZ{0.02, 0.02, 1.0}};
  fe.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});

  const auto key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());

  const auto sig = makeResolutionSignals(fe, *key);
  ASSERT_TRUE(sig.has_value());

  // Defaults must be unopinionated
  EXPECT_EQ(sig->semantic_relevance, SemanticRelevance::NONE);
  EXPECT_EQ(sig->dynamic_relevance, DynamicRelevance::NONE);
  EXPECT_FALSE(sig->uncertainty.has_value());
}

TEST(EvidenceSignalBuilderTest, SignalToControllerIntegration) {
  // Construct L3 evidence region with:
  // 16 occupied L0 cells
  // >= 4 observations
  // vertical relief 0.20m (z_min = 0.0, z_max = 0.20)
  // Region width for L3 is 0.40m -> geometry_complexity = 0.20 / 0.40 = 0.50
  // Density score = 16 / 64 = 0.25
  FrameEvidence fe;
  std::vector<PointXYZ> points;
  for (int i = 0; i < 16; ++i) {
    const int gx = i % 4;
    const int gy = i / 4;
    const double x = gx * 0.05 + 0.025;
    const double y = gy * 0.05 + 0.025;
    const double z = (i == 0) ? 0.0 : ((i == 15) ? 0.20 : 0.10);
    points.push_back(PointXYZ{x, y, z});
  }

  // Sensor origin placed at (0.0, -40.0), so distance to region center (0.2, 0.2) is ~ 40.2 m.
  // Distance 40.2 m gives M4 baseline level L2 (30m - 60m).
  fe.buildFrame(points, 1, WorldPoint2D{0.0, -40.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());

  const auto sig_opt = makeResolutionSignals(fe, *l3_key);
  ASSERT_TRUE(sig_opt.has_value());
  const auto& sig = *sig_opt;

  EXPECT_EQ(sig.observation_count, 16);
  EXPECT_DOUBLE_EQ(sig.density_score, 0.25);
  ASSERT_TRUE(sig.geometry_complexity.has_value());
  EXPECT_NEAR(*sig.geometry_complexity, 0.50, 1e-9);

  // Evaluate with default M4 ResolutionController
  const auto controller_opt = ResolutionController::create();
  ASSERT_TRUE(controller_opt.has_value());
  const auto decision_opt = controller_opt->decide(sig);
  ASSERT_TRUE(decision_opt.has_value());
  const auto& decision = *decision_opt;

  // With distance ~40.2m, baseline is L2 (0.20m).
  // Evidence is SUFFICIENT (obs=16 >= 4, density=0.25 >= 0.25).
  // Geometry threshold is 0.50 -> met! Refinement requested: L2 -> L1 (0.10m).
  EXPECT_EQ(decision.baseline_level, L2);
  EXPECT_EQ(decision.evidence_state, EvidenceState::SUFFICIENT);
  EXPECT_EQ(decision.requested_level, L1);
  EXPECT_TRUE(decision.refinement_requested());
  EXPECT_TRUE(decision.reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
}

TEST(EvidenceSignalBuilderTest, SparseEvidenceIntegration) {
  // Construct an L3 region with:
  // 100 observations in ONE base cell
  FrameEvidence fe;
  std::vector<PointXYZ> points;
  for (int i = 0; i < 100; ++i) {
    points.push_back(PointXYZ{0.02, 0.02, 1.0});
  }

  // Sensor origin at (0.0, -40.0) -> distance ~ 40m (baseline L2)
  fe.buildFrame(points, 1, WorldPoint2D{0.0, -40.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());

  // Supply high upstream uncertainty = 0.80 (> 0.60 threshold)
  const auto sig_opt = makeResolutionSignals(
      fe, *l3_key, SemanticRelevance::NONE, DynamicRelevance::NONE, 0.80);
  ASSERT_TRUE(sig_opt.has_value());
  const auto& sig = *sig_opt;

  EXPECT_EQ(sig.observation_count, 100);
  EXPECT_DOUBLE_EQ(sig.density_score, 1.0 / 64.0);
  EXPECT_FALSE(sig.geometry_complexity.has_value());  // only 1 base cell -> no geometry support

  // Evaluate with M4 controller
  const auto controller_opt = ResolutionController::create();
  ASSERT_TRUE(controller_opt.has_value());
  const auto decision_opt = controller_opt->decide(sig);
  ASSERT_TRUE(decision_opt.has_value());
  const auto& decision = *decision_opt;

  // Even though observation_count = 100 and uncertainty = 0.80 is high:
  // density_score = 1/64 < 0.25 -> evidence is INSUFFICIENT!
  // Refinement must be BLOCKED by the density gate!
  EXPECT_EQ(decision.baseline_level, L2);
  EXPECT_EQ(decision.evidence_state, EvidenceState::INSUFFICIENT);
  EXPECT_EQ(decision.requested_level, L2);
  EXPECT_FALSE(decision.refinement_requested());
  EXPECT_TRUE(decision.reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
}

TEST(EvidenceSignalBuilderTest, DistributedEvidenceIntegration) {
  // Construct an L3 region with:
  // >= 16 occupied L0 cells, low flat vertical relief (0.0)
  FrameEvidence fe;
  std::vector<PointXYZ> points;
  for (int i = 0; i < 16; ++i) {
    const int gx = i % 4;
    const int gy = i / 4;
    points.push_back(PointXYZ{gx * 0.05 + 0.025, gy * 0.05 + 0.025, 1.0});
  }

  fe.buildFrame(points, 1, WorldPoint2D{0.0, -40.0});

  const auto l3_key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L3});
  ASSERT_TRUE(l3_key.has_value());

  // Supply high upstream uncertainty = 0.80
  const auto sig_opt = makeResolutionSignals(
      fe, *l3_key, SemanticRelevance::NONE, DynamicRelevance::NONE, 0.80);
  ASSERT_TRUE(sig_opt.has_value());
  const auto& sig = *sig_opt;

  EXPECT_EQ(sig.observation_count, 16);
  EXPECT_DOUBLE_EQ(sig.density_score, 0.25);

  const auto controller_opt = ResolutionController::create();
  ASSERT_TRUE(controller_opt.has_value());
  const auto decision_opt = controller_opt->decide(sig);
  ASSERT_TRUE(decision_opt.has_value());
  const auto& decision = *decision_opt;

  // Now evidence is SUFFICIENT -> uncertainty refinement allowed!
  EXPECT_EQ(decision.baseline_level, L2);
  EXPECT_EQ(decision.evidence_state, EvidenceState::SUFFICIENT);
  EXPECT_EQ(decision.requested_level, L1);
  EXPECT_TRUE(decision.refinement_requested());
  EXPECT_TRUE(decision.reasons.has(ResolutionReason::UNCERTAINTY));
}

// Milestone 5: makeResolutionSignals fails without an active frame
TEST(EvidenceSignalBuilderTest, MakeResolutionSignalsFailsWithoutActiveFrame) {
  const auto key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());

  // 1. Fresh FrameEvidence object (no frame)
  FrameEvidence fe_fresh;
  EXPECT_FALSE(fe_fresh.hasFrame());
  EXPECT_FALSE(makeResolutionSignals(fe_fresh, *key).has_value());

  // 2. After reset() (frame cleared)
  std::vector<PointXYZ> points = {PointXYZ{0.02, 0.02, 1.0}};
  fe_fresh.buildFrame(points, 1, WorldPoint2D{0.0, 0.0});
  EXPECT_TRUE(fe_fresh.hasFrame());
  EXPECT_TRUE(makeResolutionSignals(fe_fresh, *key).has_value());

  fe_fresh.reset();
  EXPECT_FALSE(fe_fresh.hasFrame());
  EXPECT_FALSE(makeResolutionSignals(fe_fresh, *key).has_value());
}

// Milestone 5: Valid empty frame produces ResolutionSignals with correct non-zero origin distance
TEST(EvidenceSignalBuilderTest, ValidEmptyFrameDistanceCalculationWithNonZeroOrigin) {
  FrameEvidence fe;
  std::span<const PointXYZ> empty_points{};
  // Non-zero sensor origin at (10.0, 20.0)
  const auto stats_opt = fe.buildFrame(empty_points, 99, WorldPoint2D{10.0, 20.0});
  ASSERT_TRUE(stats_opt.has_value());
  EXPECT_TRUE(fe.hasFrame());

  // Target cell at (0, 0) at L0 (width 0.05m). Center is at (0.025, 0.025)
  const auto key = CellKey::fromCellCoord(CellCoord{.x = 0, .y = 0, .level = L0});
  ASSERT_TRUE(key.has_value());

  const auto sig_opt = makeResolutionSignals(fe, *key);
  ASSERT_TRUE(sig_opt.has_value());
  const auto& sig = *sig_opt;

  const double expected_dist = std::hypot(0.025 - 10.0, 0.025 - 20.0);
  EXPECT_NEAR(sig.distance_m, expected_dist, 1e-9);
  EXPECT_EQ(sig.observation_count, 0);
  EXPECT_DOUBLE_EQ(sig.density_score, 0.0);
  EXPECT_FALSE(sig.geometry_complexity.has_value());
  EXPECT_FALSE(sig.uncertainty.has_value());
  EXPECT_TRUE(sig.isValid());
}
