#include <gtest/gtest.h>
#include <random>
#include <vector>
#include "avr_mapping/resolution_controller.hpp"
#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {
namespace {

uint8_t reference_independent_targets(const ResolutionPolicyConfig& cfg,
                                      const ResolutionSignals& signals) {
  uint8_t baseline_target = avr::spatial::L3;
  if (signals.distance_m < cfg.distance_threshold_1) {
    baseline_target = avr::spatial::L0;
  } else if (signals.distance_m < cfg.distance_threshold_2) {
    baseline_target = avr::spatial::L1;
  } else if (signals.distance_m < cfg.distance_threshold_3) {
    baseline_target = avr::spatial::L2;
  } else {
    baseline_target = avr::spatial::L3;
  }

  const bool sufficient = (signals.observation_count > 0) &&
                          (signals.observation_count >= cfg.min_observations) &&
                          (signals.density_score >= cfg.min_density_score);

  uint8_t continuous_steps = 0;
  if (signals.uncertainty.has_value() &&
      *signals.uncertainty >= cfg.high_uncertainty_threshold && sufficient) {
    continuous_steps = static_cast<uint8_t>(continuous_steps + cfg.uncertainty_refinement_steps);
  }
  if (signals.geometry_complexity.has_value() &&
      *signals.geometry_complexity >= cfg.high_geometry_threshold && sufficient) {
    continuous_steps = static_cast<uint8_t>(continuous_steps + cfg.geometry_refinement_steps);
  }

  int cont_cand = static_cast<int>(baseline_target) - static_cast<int>(continuous_steps);
  uint8_t continuous_target = static_cast<uint8_t>(std::max(0, cont_cand));

  std::optional<uint8_t> semantic_target;
  if (signals.semantic_relevance == SemanticRelevance::ELEVATED) {
    semantic_target = cfg.semantic_elevated_target;
  } else if (signals.semantic_relevance == SemanticRelevance::SAFETY_CRITICAL) {
    semantic_target = cfg.semantic_critical_target;
  }

  std::optional<uint8_t> dynamic_target;
  if (signals.dynamic_relevance == DynamicRelevance::CONFIRMED) {
    dynamic_target = cfg.confirmed_dynamic_target;
  }

  uint8_t expected = std::min(baseline_target, continuous_target);
  if (semantic_target.has_value()) {
    expected = std::min(expected, *semantic_target);
  }
  if (dynamic_target.has_value()) {
    expected = std::min(expected, *dynamic_target);
  }

  expected = std::min(expected, baseline_target);
  expected = std::max(expected, avr::spatial::L0);

  return expected;
}

TEST(ResolutionPolicyMatrix, Full288CombinationPropertyMatrix) {
  auto controller = ResolutionController::create();
  ASSERT_TRUE(controller.has_value());

  const std::vector<double> distances = {5.0, 20.0, 40.0, 80.0};  // L0, L1, L2, L3
  const std::vector<SemanticRelevance> semantics = {
      SemanticRelevance::NONE, SemanticRelevance::ELEVATED, SemanticRelevance::SAFETY_CRITICAL};
  const std::vector<DynamicRelevance> dynamics = {
      DynamicRelevance::NONE, DynamicRelevance::CANDIDATE, DynamicRelevance::CONFIRMED};
  const std::vector<std::pair<uint32_t, double>> evidence_categories = {
      {10, 0.50},  // SUFFICIENT
      {1, 0.01}    // INSUFFICIENT
  };
  const std::vector<std::optional<double>> uncertainty_categories = {
      std::nullopt,  // LOW / None
      0.80           // HIGH
  };
  const std::vector<std::optional<double>> geometry_categories = {
      std::nullopt,  // LOW / None
      0.80           // HIGH
  };

  size_t combinations_tested = 0;

  for (double dist : distances) {
    for (auto sem : semantics) {
      for (auto dyn : dynamics) {
        for (const auto& [obs, dens] : evidence_categories) {
          for (const auto& unc : uncertainty_categories) {
            for (const auto& geom : geometry_categories) {
              ResolutionSignals s;
              s.distance_m = dist;
              s.semantic_relevance = sem;
              s.dynamic_relevance = dyn;
              s.observation_count = obs;
              s.density_score = dens;
              s.uncertainty = unc;
              s.geometry_complexity = geom;

              auto dec = controller->decide(s);
              ASSERT_TRUE(dec.has_value());

              // 1. Valid levels
              EXPECT_GE(dec->requested_level, avr::spatial::L0);
              EXPECT_LE(dec->requested_level, avr::spatial::L3);
              EXPECT_GE(dec->baseline_level, avr::spatial::L0);
              EXPECT_LE(dec->baseline_level, avr::spatial::L3);

              // 2. Fundamental invariant: NO COARSENING relative to baseline
              EXPECT_LE(dec->requested_level, dec->baseline_level);

              // 3. Exact equivalence to independent-target mathematical formulation
              EXPECT_EQ(dec->requested_level,
                        reference_independent_targets(controller->config(), s));

              // 4. Evidence state strictly adheres to formula
              const bool is_sufficient = (obs >= 4) && (dens >= 0.25) && (obs > 0);
              const EvidenceState expected_evidence =
                  is_sufficient ? EvidenceState::SUFFICIENT : EvidenceState::INSUFFICIENT;
              EXPECT_EQ(dec->evidence_state, expected_evidence);

              // 5. Baseline reason bit always present
              EXPECT_TRUE(dec->reasons.has(ResolutionReason::DISTANCE_BASELINE));

              // 6. Evidence reason bits
              if (!is_sufficient) {
                EXPECT_TRUE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
                if ((unc.has_value() && *unc >= 0.60) || (geom.has_value() && *geom >= 0.50)) {
                  EXPECT_TRUE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
                }
              } else {
                EXPECT_FALSE(dec->reasons.has(ResolutionReason::INSUFFICIENT_EVIDENCE));
                EXPECT_FALSE(dec->reasons.has(ResolutionReason::DENSITY_GATE_BLOCKED));
              }

              // 7. Continuous signals refinement requires sufficiency
              if (unc.has_value() && *unc >= 0.60 && is_sufficient) {
                EXPECT_TRUE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
              } else {
                EXPECT_FALSE(dec->reasons.has(ResolutionReason::UNCERTAINTY));
              }

              if (geom.has_value() && *geom >= 0.50 && is_sufficient) {
                EXPECT_TRUE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
              } else {
                EXPECT_FALSE(dec->reasons.has(ResolutionReason::GEOMETRY_COMPLEXITY));
              }

              // 8. Safety critical and confirmed dynamic caps
              if (sem == SemanticRelevance::SAFETY_CRITICAL) {
                EXPECT_LE(dec->requested_level, avr::spatial::L0);
                EXPECT_TRUE(dec->reasons.has(ResolutionReason::SEMANTIC_RELEVANCE));
              }
              if (dyn == DynamicRelevance::CONFIRMED) {
                EXPECT_LE(dec->requested_level, avr::spatial::L0);
                EXPECT_TRUE(dec->reasons.has(ResolutionReason::DYNAMIC_RELEVANCE));
              }

              // 9. If evidence is insufficient, step-refinements cannot occur
              if (!is_sufficient) {
                if (sem == SemanticRelevance::NONE && dyn != DynamicRelevance::CONFIRMED) {
                  EXPECT_EQ(dec->requested_level, dec->baseline_level);
                }
              }

              ++combinations_tested;
            }
          }
        }
      }
    }
  }

  EXPECT_EQ(combinations_tested, 288);
}

TEST(ResolutionPolicyMatrix, Randomized100kSignalsAndRepeatability) {
  auto controller_a = ResolutionController::create();
  auto controller_b = ResolutionController::create();
  ASSERT_TRUE(controller_a.has_value());
  ASSERT_TRUE(controller_b.has_value());

  constexpr uint32_t kSeed = 26053;
  std::mt19937_64 rng(kSeed);

  std::uniform_real_distribution<double> dist_dist(0.0, 150.0);
  std::uniform_int_distribution<uint32_t> sem_dist(0, 2);
  std::uniform_int_distribution<uint32_t> dyn_dist(0, 2);
  std::uniform_int_distribution<uint32_t> obs_dist(0, 20);
  std::uniform_real_distribution<double> dens_dist(0.0, 1.0);
  std::uniform_real_distribution<double> norm_dist(0.0, 1.0);
  std::bernoulli_distribution has_opt_dist(0.7);

  constexpr size_t kNumIterations = 100000;

  for (size_t i = 0; i < kNumIterations; ++i) {
    ResolutionSignals s;
    s.distance_m = dist_dist(rng);
    s.semantic_relevance = static_cast<SemanticRelevance>(sem_dist(rng));
    s.dynamic_relevance = static_cast<DynamicRelevance>(dyn_dist(rng));
    s.observation_count = obs_dist(rng);
    s.density_score = dens_dist(rng);

    if (has_opt_dist(rng)) {
      s.uncertainty = norm_dist(rng);
    }
    if (has_opt_dist(rng)) {
      s.geometry_complexity = norm_dist(rng);
    }

    auto dec_a = controller_a->decide(s);
    auto dec_b = controller_b->decide(s);

    ASSERT_TRUE(dec_a.has_value());
    ASSERT_TRUE(dec_b.has_value());

    // Repeatability across identical controllers
    EXPECT_EQ(dec_a.value(), dec_b.value());

    // Invariants
    EXPECT_GE(dec_a->requested_level, avr::spatial::L0);
    EXPECT_LE(dec_a->requested_level, avr::spatial::L3);
    EXPECT_GE(dec_a->baseline_level, avr::spatial::L0);
    EXPECT_LE(dec_a->baseline_level, avr::spatial::L3);

    // No coarsening
    EXPECT_LE(dec_a->requested_level, dec_a->baseline_level);

    // Exact equivalence to independent-target mathematical formulation
    EXPECT_EQ(dec_a->requested_level,
              reference_independent_targets(controller_a->config(), s));

    // Evidence formula check
    const bool is_sufficient =
        (s.observation_count >= 4) && (s.density_score >= 0.25) && (s.observation_count > 0);
    EXPECT_EQ(dec_a->evidence_state,
              is_sufficient ? EvidenceState::SUFFICIENT : EvidenceState::INSUFFICIENT);

    EXPECT_TRUE(dec_a->reasons.has(ResolutionReason::DISTANCE_BASELINE));
  }
}

}  // namespace
}  // namespace avr::mapping
