#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <iostream>
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

TEST(ResolutionControllerLayout, ReportMemoryLayout) {
  std::cout << "\n=== ResolutionController Memory Layout ===" << std::endl;
  std::cout << "sizeof(ResolutionPolicyConfig): " << sizeof(ResolutionPolicyConfig)
            << " bytes, alignof: " << alignof(ResolutionPolicyConfig) << std::endl;
  std::cout << "sizeof(ResolutionSignals): " << sizeof(ResolutionSignals)
            << " bytes, alignof: " << alignof(ResolutionSignals) << std::endl;
  std::cout << "sizeof(ResolutionDecision): " << sizeof(ResolutionDecision)
            << " bytes, alignof: " << alignof(ResolutionDecision) << std::endl;
  std::cout << "sizeof(ResolutionReasonBits): " << sizeof(ResolutionReasonBits)
            << " bytes, alignof: " << alignof(ResolutionReasonBits) << std::endl;
  std::cout << "==========================================\n" << std::endl;

  EXPECT_LE(sizeof(ResolutionDecision), 8);
  EXPECT_TRUE(sizeof(ResolutionReasonBits) == 2);
}

TEST(ResolutionControllerBoundary, DistanceBoundaries) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  // Helper with sufficient evidence and no optional signals
  auto make_signals = [](double d) {
    ResolutionSignals s;
    s.distance_m = d;
    s.observation_count = 10;
    s.density_score = 0.5;
    return s;
  };

  // 0 m -> L0
  auto dec = controller->decide(make_signals(0.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L0);
  EXPECT_EQ(dec->requested_level, avr::spatial::L0);
  EXPECT_EQ(dec->reasons, ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE));

  // nextafter(10, -inf) -> L0
  double below_10 = std::nextafter(10.0, -std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(below_10));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L0);
  EXPECT_EQ(dec->requested_level, avr::spatial::L0);

  // 10 m -> L1
  dec = controller->decide(make_signals(10.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L1);
  EXPECT_EQ(dec->requested_level, avr::spatial::L1);

  // nextafter(10, +inf) -> L1
  double above_10 = std::nextafter(10.0, std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(above_10));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L1);
  EXPECT_EQ(dec->requested_level, avr::spatial::L1);

  // nextafter(30, -inf) -> L1
  double below_30 = std::nextafter(30.0, -std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(below_30));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L1);
  EXPECT_EQ(dec->requested_level, avr::spatial::L1);

  // 30 m -> L2
  dec = controller->decide(make_signals(30.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L2);
  EXPECT_EQ(dec->requested_level, avr::spatial::L2);

  // nextafter(30, +inf) -> L2
  double above_30 = std::nextafter(30.0, std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(above_30));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L2);
  EXPECT_EQ(dec->requested_level, avr::spatial::L2);

  // nextafter(60, -inf) -> L2
  double below_60 = std::nextafter(60.0, -std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(below_60));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L2);
  EXPECT_EQ(dec->requested_level, avr::spatial::L2);

  // 60 m -> L3
  dec = controller->decide(make_signals(60.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);

  // nextafter(60, +inf) -> L3
  double above_60 = std::nextafter(60.0, std::numeric_limits<double>::infinity());
  dec = controller->decide(make_signals(above_60));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);

  // 100 m -> L3
  dec = controller->decide(make_signals(100.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);

  // 1000 m -> L3
  dec = controller->decide(make_signals(1000.0));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);

  // DBL_MAX -> L3
  dec = controller->decide(make_signals(std::numeric_limits<double>::max()));
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);
}

TEST(ResolutionControllerValidInputs, StandardDiscreteDistanceSequence) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  const std::vector<std::pair<double, uint8_t>> test_cases = {
      {0.0, avr::spatial::L0},
      {1.0, avr::spatial::L0},
      {5.0, avr::spatial::L0},
      {9.0, avr::spatial::L0},
      {10.0, avr::spatial::L1},
      {20.0, avr::spatial::L1},
      {29.0, avr::spatial::L1},
      {30.0, avr::spatial::L2},
      {40.0, avr::spatial::L2},
      {59.0, avr::spatial::L2},
      {60.0, avr::spatial::L3},
      {80.0, avr::spatial::L3},
      {100.0, avr::spatial::L3},
      {1000.0, avr::spatial::L3},
  };

  for (const auto& [dist, expected_level] : test_cases) {
    ResolutionSignals s;
    s.distance_m = dist;
    s.observation_count = 10;
    s.density_score = 0.5;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value()) << "Failed for distance: " << dist;
    EXPECT_EQ(dec->baseline_level, expected_level);
    EXPECT_EQ(dec->requested_level, expected_level);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->reasons, ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE));
  }
}

TEST(ResolutionControllerInvalidInputs, RejectionOfMalformedNumericInputs) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  auto base_valid = []() {
    ResolutionSignals s;
    s.distance_m = 20.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    return s;
  };

  // Invalid distances
  {
    auto s = base_valid();
    s.distance_m = -1.0;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.distance_m = -0.0001;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.distance_m = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller->decide(s).has_value());

    s.distance_m = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller->decide(s).has_value());

    s.distance_m = -std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller->decide(s).has_value());
  }

  // Invalid density scores
  {
    auto s = base_valid();
    s.density_score = -0.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.density_score = 1.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.density_score = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller->decide(s).has_value());

    s.density_score = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller->decide(s).has_value());
  }

  // Invalid uncertainty when present
  {
    auto s = base_valid();
    s.uncertainty = -0.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.uncertainty = 1.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.uncertainty = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller->decide(s).has_value());

    s.uncertainty = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller->decide(s).has_value());
  }

  // Invalid geometry complexity when present
  {
    auto s = base_valid();
    s.geometry_complexity = -0.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.geometry_complexity = 1.1;
    EXPECT_FALSE(controller->decide(s).has_value());

    s.geometry_complexity = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller->decide(s).has_value());

    s.geometry_complexity = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller->decide(s).has_value());
  }
}

TEST(ResolutionControllerEvidenceGate, UncertaintyRefinementMatrix) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  // 1. High uncertainty + sufficient count + sufficient density -> refinement
  {
    ResolutionSignals s;
    s.distance_m = 80.0;  // baseline L3
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  }

  // 2. High uncertainty + low count + high density -> NO uncertainty refinement + INSUFFICIENT
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 2;  // < 4
    s.density_score = 0.9;
    s.uncertainty = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  }

  // 3. High uncertainty + high count + low density -> NO uncertainty refinement + INSUFFICIENT
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 50;
    s.density_score = 0.1;  // < 0.25
    s.uncertainty = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  }

  // 4. High uncertainty + zero observations -> NO uncertainty refinement + INSUFFICIENT
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 0;
    s.density_score = 0.5;
    s.uncertainty = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  }

  // 5. Low uncertainty + sufficient evidence -> no uncertainty refinement
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.3;  // < 0.60

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->reasons, ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE));
  }
}

TEST(ResolutionControllerEvidenceGate, GeometryRefinementMatrix) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  // High geometry + sufficient evidence -> refinement
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.geometry_complexity = 0.7;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
  }

  // High geometry + insufficient evidence -> blocked
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 1;
    s.density_score = 0.05;
    s.geometry_complexity = 0.7;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->refinement_requested());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  }
}

TEST(ResolutionControllerThresholds, ThresholdEqualityAndNextAfter) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  auto base_sufficient = []() {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    return s;
  };

  // Uncertainty threshold: 0.60
  {
    auto s = base_sufficient();
    s.uncertainty = 0.60;  // exactly threshold -> triggers
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));

    s.uncertainty = std::nextafter(0.60, -std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));

    s.uncertainty = std::nextafter(0.60, std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
  }

  // Geometry threshold: 0.50
  {
    auto s = base_sufficient();
    s.geometry_complexity = 0.50;  // exactly threshold -> triggers
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));

    s.geometry_complexity = std::nextafter(0.50, -std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));

    s.geometry_complexity = std::nextafter(0.50, std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
  }

  // Density score threshold: 0.25
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 4;
    s.density_score = 0.25;  // exactly threshold -> sufficient
    s.uncertainty = 0.8;
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);

    s.density_score = std::nextafter(0.25, -std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);

    s.density_score = std::nextafter(0.25, std::numeric_limits<double>::infinity());
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);
  }

  // Observation count threshold: 4
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 4;  // exactly threshold
    s.density_score = 0.5;
    s.uncertainty = 0.8;
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);

    s.observation_count = 3;  // below threshold
    dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    EXPECT_EQ(dec->requested_level, avr::spatial::L3);
  }
}

TEST(ResolutionControllerSemantic, SemanticCapsAcrossDistances) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  auto test_semantic = [&](double dist, SemanticRelevance sem, uint8_t expected_req, bool has_sem_reason) {
    ResolutionSignals s;
    s.distance_m = dist;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.semantic_relevance = sem;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, expected_req);
    EXPECT_EQ(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE), has_sem_reason);
  };

  // At 80 m (baseline L3)
  test_semantic(80.0, SemanticRelevance::NONE, avr::spatial::L3, false);
  test_semantic(80.0, SemanticRelevance::ELEVATED, avr::spatial::L1, true);
  test_semantic(80.0, SemanticRelevance::SAFETY_CRITICAL, avr::spatial::L0, true);

  // At 20 m (baseline L1)
  test_semantic(20.0, SemanticRelevance::NONE, avr::spatial::L1, false);
  test_semantic(20.0, SemanticRelevance::ELEVATED, avr::spatial::L1, true);
  test_semantic(20.0, SemanticRelevance::SAFETY_CRITICAL, avr::spatial::L0, true);

  // At 5 m (baseline L0)
  test_semantic(5.0, SemanticRelevance::NONE, avr::spatial::L0, false);
  test_semantic(5.0, SemanticRelevance::ELEVATED, avr::spatial::L0, true);
  test_semantic(5.0, SemanticRelevance::SAFETY_CRITICAL, avr::spatial::L0, true);
}

TEST(ResolutionControllerDynamic, DynamicCuesAcrossDistances) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  auto test_dyn = [&](double dist, DynamicRelevance dyn, uint8_t expected_req, bool has_dyn_reason) {
    ResolutionSignals s;
    s.distance_m = dist;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.dynamic_relevance = dyn;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, expected_req);
    EXPECT_EQ(dec->reasons.has(ResolutionReason::DYNAMIC_RELEVANCE), has_dyn_reason);
  };

  // At 80 m (baseline L3)
  test_dyn(80.0, DynamicRelevance::NONE, avr::spatial::L3, false);
  test_dyn(80.0, DynamicRelevance::CANDIDATE, avr::spatial::L3, false);
  test_dyn(80.0, DynamicRelevance::CONFIRMED, avr::spatial::L0, true);

  // At 5 m (baseline L0)
  test_dyn(5.0, DynamicRelevance::CONFIRMED, avr::spatial::L0, true);
}

TEST(ResolutionControllerCombined, CumulativeSignalsAndSaturation) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  // Baseline L3 + ELEVATED (target L1) + high uncertainty (+1) + high geometry (+1)
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.semantic_relevance = SemanticRelevance::ELEVATED;
    s.uncertainty = 0.8;
    s.geometry_complexity = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    // Baseline 3 - (1+1) = 1 (L1). Capped by ELEVATED (L1) -> L1.
    EXPECT_EQ(dec->requested_level, avr::spatial::L1);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
  }

  // SAFETY_CRITICAL + CONFIRMED + high uncertainty + high geometry -> saturates at L0 safely
  {
    ResolutionSignals s;
    s.distance_m = 80.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.semantic_relevance = SemanticRelevance::SAFETY_CRITICAL;
    s.dynamic_relevance = DynamicRelevance::CONFIRMED;
    s.uncertainty = 0.9;
    s.geometry_complexity = 0.9;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L0);
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DYNAMIC_RELEVANCE));
  }
}

TEST(ResolutionControllerSparse, SparseSafetyTest) {
  // Section 41:
  // distance = 80 m, semantic = SAFETY_CRITICAL, dynamic = NONE, obs = 1, density = very low, uncertainty = high
  // Expected: baseline = L3, requested = L0, evidence = INSUFFICIENT
  // Uncertainty blocked by density gate, but safety semantic refinement remains.
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  ResolutionSignals s;
  s.distance_m = 80.0;
  s.semantic_relevance = SemanticRelevance::SAFETY_CRITICAL;
  s.dynamic_relevance = DynamicRelevance::NONE;
  s.observation_count = 1;
  s.density_score = 0.01;
  s.uncertainty = 0.85;

  auto dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L0);
  EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
  EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
}

TEST(ResolutionControllerSparse, SparseUncertaintyTest) {
  // Section 42:
  // distance = 80 m, semantic = NONE, dynamic = NONE, obs = 1, density = 0.01, uncertainty = high, geometry = low/none
  // Expected: baseline = L3, requested = L3, evidence = INSUFFICIENT
  // No fabricated refinement.
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  ResolutionSignals s;
  s.distance_m = 80.0;
  s.semantic_relevance = SemanticRelevance::NONE;
  s.dynamic_relevance = DynamicRelevance::NONE;
  s.observation_count = 1;
  s.density_score = 0.01;
  s.uncertainty = 0.85;

  auto dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);
  EXPECT_FALSE(dec->refinement_requested());
  EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
  EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
  EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
}

TEST(ResolutionControllerConfig, RejectInvalidConfigurations) {
  // Valid baseline
  ResolutionPolicyConfig cfg;
  EXPECT_TRUE(cfg.validate());

  // Equal distance thresholds
  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_2 = cfg.distance_threshold_1;
  EXPECT_FALSE(cfg.validate());
  EXPECT_FALSE(ResolutionController::create(cfg).has_value());

  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_3 = cfg.distance_threshold_2;
  EXPECT_FALSE(cfg.validate());

  // Descending distance thresholds
  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_1 = 30.0;
  cfg.distance_threshold_2 = 10.0;
  EXPECT_FALSE(cfg.validate());

  // Negative distance threshold
  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_1 = -5.0;
  EXPECT_FALSE(cfg.validate());

  // NaN / Inf distance threshold
  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_1 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.distance_threshold_3 = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(cfg.validate());

  // Density score threshold out of range
  cfg = ResolutionPolicyConfig{};
  cfg.min_density_score = -0.01;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.min_density_score = 1.01;
  EXPECT_FALSE(cfg.validate());

  // Uncertainty threshold out of range
  cfg = ResolutionPolicyConfig{};
  cfg.high_uncertainty_threshold = -0.1;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.high_uncertainty_threshold = 1.1;
  EXPECT_FALSE(cfg.validate());

  // Geometry threshold out of range
  cfg = ResolutionPolicyConfig{};
  cfg.high_geometry_threshold = -0.1;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.high_geometry_threshold = 1.1;
  EXPECT_FALSE(cfg.validate());

  // Invalid level targets
  cfg = ResolutionPolicyConfig{};
  cfg.semantic_elevated_target = 4;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.semantic_critical_target = 5;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.confirmed_dynamic_target = 10;
  EXPECT_FALSE(cfg.validate());

  // Invalid refinement step settings
  cfg = ResolutionPolicyConfig{};
  cfg.uncertainty_refinement_steps = 4;
  EXPECT_FALSE(cfg.validate());

  cfg = ResolutionPolicyConfig{};
  cfg.geometry_refinement_steps = 4;
  EXPECT_FALSE(cfg.validate());
}

TEST(ResolutionControllerConfig, AlternateConfigurationObeysCustomParameters) {
  ResolutionPolicyConfig custom_cfg;
  custom_cfg.distance_threshold_1 = 5.0;
  custom_cfg.distance_threshold_2 = 20.0;
  custom_cfg.distance_threshold_3 = 50.0;
  custom_cfg.min_observations = 10;
  custom_cfg.min_density_score = 0.40;
  custom_cfg.high_uncertainty_threshold = 0.70;
  custom_cfg.high_geometry_threshold = 0.65;
  custom_cfg.semantic_elevated_target = avr::spatial::L2;
  custom_cfg.semantic_critical_target = avr::spatial::L1;
  custom_cfg.confirmed_dynamic_target = avr::spatial::L1;
  custom_cfg.uncertainty_refinement_steps = 2;
  custom_cfg.geometry_refinement_steps = 2;

  ASSERT_TRUE(custom_cfg.validate());
  auto controller = ResolutionController::create(custom_cfg);
  ASSERT_TRUE(controller.has_value());
  EXPECT_EQ(controller->config().distance_threshold_1, 5.0);

  // At 7 m with custom cfg: between 5 and 20 -> baseline is L1 (whereas default config is L0)
  ResolutionSignals s;
  s.distance_m = 7.0;
  s.observation_count = 15;
  s.density_score = 0.5;
  auto dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L1);

  // At 55 m (baseline L3) + ELEVATED -> target is L2 under custom cfg (L1 under default)
  s.distance_m = 55.0;
  s.semantic_relevance = SemanticRelevance::ELEVATED;
  dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
  EXPECT_EQ(dec->requested_level, avr::spatial::L2);

  // Uncertainty threshold 0.70 with 2 steps refinement:
  s.semantic_relevance = SemanticRelevance::NONE;
  s.uncertainty = 0.65;  // < 0.70 -> no refinement
  dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->requested_level, avr::spatial::L3);

  s.uncertainty = 0.75;  // >= 0.70 -> 2 steps refinement (L3 -> L1)
  dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->requested_level, avr::spatial::L1);
}

TEST(ResolutionControllerStatelessness, IdenticalOutputsAcrossConsecutiveCalls) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  ResolutionSignals sig_a;
  sig_a.distance_m = 45.0;
  sig_a.observation_count = 12;
  sig_a.density_score = 0.6;
  sig_a.semantic_relevance = SemanticRelevance::ELEVATED;

  ResolutionSignals sig_b;
  sig_b.distance_m = 15.0;
  sig_b.observation_count = 2;
  sig_b.density_score = 0.1;
  sig_b.uncertainty = 0.9;

  // Call sequence: A -> B -> A
  auto dec_a1 = controller->decide(sig_a);
  auto dec_b = controller->decide(sig_b);
  auto dec_a2 = controller->decide(sig_a);

  ASSERT_TRUE(dec_a1.has_value());
  ASSERT_TRUE(dec_b.has_value());
  ASSERT_TRUE(dec_a2.has_value());

  EXPECT_EQ(dec_a1.value(), dec_a2.value());
}

TEST(ResolutionControllerCombinationAlgebra, IndependentTargetRegressionCases) {
  // CASE A:
  // baseline L3
  // ELEVATED semantic target L1
  // high uncertainty, sufficient evidence, 1 step
  // geometry low
  // Expected: continuous target L2, semantic target L1, final L1
  {
    auto controller = ResolutionController::create();
    ASSERT_TRUE(controller.has_value());

    ResolutionSignals s;
    s.distance_m = 80.0;  // baseline L3
    s.semantic_relevance = SemanticRelevance::ELEVATED;  // target L1
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;  // 1 step -> continuous target L2

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L1);  // min(L3, L2, L1) = L1
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
  }

  // CASE B:
  // baseline L3
  // ELEVATED semantic L1
  // high uncertainty 1 step
  // high geometry 1 step
  // Expected: continuous target L1, semantic target L1, final L1
  {
    auto controller = ResolutionController::create();
    ASSERT_TRUE(controller.has_value());

    ResolutionSignals s;
    s.distance_m = 80.0;
    s.semantic_relevance = SemanticRelevance::ELEVATED;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;
    s.geometry_complexity = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L1);  // min(L3, L1, L1) = L1
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
  }

  // CASE C:
  // baseline L3
  // ELEVATED semantic L2 in an alternate valid config
  // high uncertainty 1 step
  // Expected: continuous target L2, semantic target L2, final L2
  {
    ResolutionPolicyConfig cfg;
    cfg.semantic_elevated_target = avr::spatial::L2;
    cfg.semantic_critical_target = avr::spatial::L1;
    auto controller = ResolutionController::create(cfg);
    ASSERT_TRUE(controller.has_value());

    ResolutionSignals s;
    s.distance_m = 80.0;
    s.semantic_relevance = SemanticRelevance::ELEVATED;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L3);
    EXPECT_EQ(dec->requested_level, avr::spatial::L2);  // min(L3, L2, L2) = L2
  }

  // CASE D:
  // baseline L3
  // SAFETY_CRITICAL target L0
  // high uncertainty + high geometry
  // Expected: L0
  {
    auto controller = ResolutionController::create();
    ASSERT_TRUE(controller.has_value());

    ResolutionSignals s;
    s.distance_m = 80.0;
    s.semantic_relevance = SemanticRelevance::SAFETY_CRITICAL;
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;
    s.geometry_complexity = 0.8;

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->requested_level, avr::spatial::L0);
  }

  // CASE E:
  // baseline L2
  // confirmed dynamic target L1 in alternate config
  // high uncertainty one step
  // Expected: dynamic target L1, continuous target L1, final L1
  {
    ResolutionPolicyConfig cfg;
    cfg.confirmed_dynamic_target = avr::spatial::L1;
    auto controller = ResolutionController::create(cfg);
    ASSERT_TRUE(controller.has_value());

    ResolutionSignals s;
    s.distance_m = 40.0;  // baseline L2
    s.dynamic_relevance = DynamicRelevance::CONFIRMED;  // target L1
    s.observation_count = 10;
    s.density_score = 0.5;
    s.uncertainty = 0.8;  // 1 step -> continuous target L1

    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->baseline_level, avr::spatial::L2);
    EXPECT_EQ(dec->requested_level, avr::spatial::L1);  // min(L2, L1, L1) = L1
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::DYNAMIC_RELEVANCE));
    EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
  }
}

TEST(ResolutionControllerConfig, MinObservationsContract) {
  ResolutionPolicyConfig cfg;
  cfg.min_observations = 0;
  EXPECT_FALSE(cfg.validate());
  EXPECT_FALSE(ResolutionController::create(cfg).has_value());

  cfg.min_observations = 1;
  EXPECT_TRUE(cfg.validate());
  EXPECT_TRUE(ResolutionController::create(cfg).has_value());

  cfg.min_observations = 4;
  EXPECT_TRUE(cfg.validate());

  cfg.min_observations = std::numeric_limits<uint32_t>::max();
  EXPECT_TRUE(cfg.validate());
  EXPECT_TRUE(ResolutionController::create(cfg).has_value());
}

TEST(ResolutionControllerConfig, SemanticTargetOrderingContract) {
  ResolutionPolicyConfig cfg;

  // Valid: critical L0, elevated L1
  cfg.semantic_critical_target = avr::spatial::L0;
  cfg.semantic_elevated_target = avr::spatial::L1;
  EXPECT_TRUE(cfg.validate());

  // Valid: critical L0, elevated L0
  cfg.semantic_critical_target = avr::spatial::L0;
  cfg.semantic_elevated_target = avr::spatial::L0;
  EXPECT_TRUE(cfg.validate());

  // Valid: critical L1, elevated L2
  cfg.semantic_critical_target = avr::spatial::L1;
  cfg.semantic_elevated_target = avr::spatial::L2;
  EXPECT_TRUE(cfg.validate());

  // Invalid: critical L2, elevated L1 (critical is coarser than elevated)
  cfg.semantic_critical_target = avr::spatial::L2;
  cfg.semantic_elevated_target = avr::spatial::L1;
  EXPECT_FALSE(cfg.validate());
  EXPECT_FALSE(ResolutionController::create(cfg).has_value());

  // Invalid: critical L3, elevated L0
  cfg.semantic_critical_target = avr::spatial::L3;
  cfg.semantic_elevated_target = avr::spatial::L0;
  EXPECT_FALSE(cfg.validate());
  EXPECT_FALSE(ResolutionController::create(cfg).has_value());
}

TEST(ResolutionControllerReasons, BaselineOnlyReasonTests) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  // A. Plain baseline with SUFFICIENT evidence:
  // Expected reason: DISTANCE_BASELINE only.
  {
    ResolutionSignals s;
    s.distance_m = 20.0;
    s.observation_count = 10;
    s.density_score = 0.5;
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::SUFFICIENT);
    EXPECT_EQ(dec->reasons, ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE));
  }

  // B. Plain baseline with INSUFFICIENT evidence but no high blocked uncertainty/geometry:
  // Expected reasons: DISTANCE_BASELINE, INSUFFICIENT_EVIDENCE.
  // DENSITY_GATE_BLOCKED must NOT be set unless an actual high uncertainty/geometry was blocked.
  {
    ResolutionSignals s;
    s.distance_m = 20.0;
    s.observation_count = 2;  // insufficient
    s.density_score = 0.1;
    auto dec = controller->decide(s);
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(dec->evidence_state, EvidenceState::INSUFFICIENT);
    ResolutionReasonBits expected = ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE) |
                                   ResolutionReason::INSUFFICIENT_EVIDENCE;
    EXPECT_EQ(dec->reasons, expected);
    EXPECT_FALSE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
  }
}

TEST(ResolutionControllerInvalidInputs, EnumValidationRegression) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  ResolutionSignals s;
  s.distance_m = 20.0;
  s.observation_count = 10;
  s.density_score = 0.5;

  // Invalid SemanticRelevance
  s.semantic_relevance = static_cast<SemanticRelevance>(3);
  EXPECT_FALSE(controller->decide(s).has_value());

  s.semantic_relevance = static_cast<SemanticRelevance>(255);
  EXPECT_FALSE(controller->decide(s).has_value());

  // Restore valid semantic
  s.semantic_relevance = SemanticRelevance::NONE;

  // Invalid DynamicRelevance
  s.dynamic_relevance = static_cast<DynamicRelevance>(3);
  EXPECT_FALSE(controller->decide(s).has_value());

  s.dynamic_relevance = static_cast<DynamicRelevance>(128);
  EXPECT_FALSE(controller->decide(s).has_value());
}

TEST(ResolutionControllerImmutability, CallerConfigMutationDoesNotAlterController) {
  ResolutionPolicyConfig cfg;
  cfg.distance_threshold_1 = 10.0;

  auto controller = ResolutionController::create(cfg);
  ASSERT_TRUE(controller.has_value());

  // Mutate caller's config
  cfg.distance_threshold_1 = 50.0;

  // Controller must still use original 10.0
  ResolutionSignals s;
  s.distance_m = 15.0;  // > 10.0 -> L1 under controller's config (would be L0 if mutated)
  s.observation_count = 10;
  s.density_score = 0.5;

  auto dec = controller->decide(s);
  ASSERT_TRUE(dec.has_value());
  EXPECT_EQ(dec->baseline_level, avr::spatial::L1);
  EXPECT_EQ(controller->config().distance_threshold_1, 10.0);
}

TEST(ResolutionControllerConfig, DefaultPolicyGoldenValues) {
  ResolutionPolicyConfig cfg;
  EXPECT_EQ(cfg.distance_threshold_1, 10.0);
  EXPECT_EQ(cfg.distance_threshold_2, 30.0);
  EXPECT_EQ(cfg.distance_threshold_3, 60.0);

  EXPECT_EQ(cfg.min_observations, 4U);
  EXPECT_EQ(cfg.min_density_score, 0.25);

  EXPECT_EQ(cfg.high_uncertainty_threshold, 0.60);
  EXPECT_EQ(cfg.high_geometry_threshold, 0.50);

  EXPECT_EQ(cfg.semantic_elevated_target, avr::spatial::L1);
  EXPECT_EQ(cfg.semantic_critical_target, avr::spatial::L0);
  EXPECT_EQ(cfg.confirmed_dynamic_target, avr::spatial::L0);

  EXPECT_EQ(cfg.uncertainty_refinement_steps, 1U);
  EXPECT_EQ(cfg.geometry_refinement_steps, 1U);
}

}  // namespace
}  // namespace avr::mapping
