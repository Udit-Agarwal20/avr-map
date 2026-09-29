#include "avr_mapping/resolution_controller.hpp"
#include <algorithm>

namespace avr::mapping {

std::optional<ResolutionController> ResolutionController::create(
    const ResolutionPolicyConfig& config) noexcept {
  if (!config.validate()) {
    return std::nullopt;
  }
  return ResolutionController(config);
}

std::optional<ResolutionDecision> ResolutionController::decide(
    const ResolutionSignals& signals) const noexcept {
  if (!signals.isValid()) {
    return std::nullopt;
  }

  // 1. Determine distance baseline level
  uint8_t baseline_level = avr::spatial::L3;
  if (signals.distance_m < config_.distance_threshold_1) {
    baseline_level = avr::spatial::L0;
  } else if (signals.distance_m < config_.distance_threshold_2) {
    baseline_level = avr::spatial::L1;
  } else if (signals.distance_m < config_.distance_threshold_3) {
    baseline_level = avr::spatial::L2;
  } else {
    baseline_level = avr::spatial::L3;
  }

  ResolutionReasonBits reasons;
  reasons.set(ResolutionReason::DISTANCE_BASELINE);

  // 2. Evaluate evidence sufficiency (observation_count == 0 is always insufficient)
  const bool evidence_sufficient =
      (signals.observation_count > 0) &&
      (signals.observation_count >= config_.min_observations) &&
      (signals.density_score >= config_.min_density_score);

  const EvidenceState evidence_state =
      evidence_sufficient ? EvidenceState::SUFFICIENT : EvidenceState::INSUFFICIENT;

  if (!evidence_sufficient) {
    reasons.set(ResolutionReason::INSUFFICIENT_EVIDENCE);
  }

  // Benchmark alternatives change allocation only. The pipeline still constructs
  // identical evidence and routing, and applies normal hysteresis/replay/coarsening.
  // Empty regions retain the baseline; supported uniform regions request L0.
  if (config_.selection_strategy != ResolutionSelectionStrategy::FULL_AVR) {
    ResolutionDecision decision;
    decision.baseline_level = baseline_level;
    decision.requested_level =
        config_.selection_strategy == ResolutionSelectionStrategy::UNIFORM_L0_OBSERVED &&
                signals.observation_count > 0
            ? avr::spatial::L0 : baseline_level;
    decision.evidence_state = evidence_state;
    decision.reasons = reasons;
    return decision;
  }

  // 3. Evaluate continuous signals (uncertainty and geometry complexity) gated by evidence sufficiency
  uint8_t total_refinement_steps = 0;

  if (signals.uncertainty.has_value() &&
      *signals.uncertainty >= config_.high_uncertainty_threshold) {
    if (evidence_sufficient) {
      total_refinement_steps = static_cast<uint8_t>(
          total_refinement_steps + config_.uncertainty_refinement_steps);
      reasons.set(ResolutionReason::UNCERTAINTY);
    } else {
      reasons.set(ResolutionReason::DENSITY_GATE_BLOCKED);
    }
  }

  if (signals.geometry_complexity.has_value() &&
      *signals.geometry_complexity >= config_.high_geometry_threshold) {
    if (evidence_sufficient) {
      total_refinement_steps = static_cast<uint8_t>(
          total_refinement_steps + config_.geometry_refinement_steps);
      reasons.set(ResolutionReason::GEOMETRY_COMPLEXITY);
    } else {
      reasons.set(ResolutionReason::DENSITY_GATE_BLOCKED);
    }
  }

  // Independent Combination Algebra:
  // Each signal produces an independent resolution target:
  // - baseline_target: distance baseline
  // - continuous_target: baseline refined by the SUM of evidence-approved uncertainty + geometry steps
  // - semantic_target: configured semantic maximum-coarseness cap if semantic signal active
  // - dynamic_target: configured dynamic maximum-coarseness cap if confirmed dynamic signal active
  // Final requested_level is the minimum (fine-most) of all active targets.
  const uint8_t baseline_target = baseline_level;

  // Compute continuous target
  int continuous_candidate =
      static_cast<int>(baseline_target) - static_cast<int>(total_refinement_steps);
  if (continuous_candidate < static_cast<int>(avr::spatial::L0)) {
    continuous_candidate = static_cast<int>(avr::spatial::L0);
  }
  const uint8_t continuous_target = static_cast<uint8_t>(continuous_candidate);

  uint8_t requested_level = std::min(baseline_target, continuous_target);

  // Apply semantic target cap (if active)
  if (signals.semantic_relevance == SemanticRelevance::ELEVATED) {
    reasons.set(ResolutionReason::SEMANTIC_RELEVANCE);
    requested_level = std::min(requested_level, config_.semantic_elevated_target);
  } else if (signals.semantic_relevance == SemanticRelevance::SAFETY_CRITICAL) {
    reasons.set(ResolutionReason::SEMANTIC_RELEVANCE);
    requested_level = std::min(requested_level, config_.semantic_critical_target);
  }

  // Apply confirmed dynamic target cap (if active)
  if (signals.dynamic_relevance == DynamicRelevance::CONFIRMED) {
    reasons.set(ResolutionReason::DYNAMIC_RELEVANCE);
    requested_level = std::min(requested_level, config_.confirmed_dynamic_target);
  }

  // Final invariant guarantees: never coarsen beyond baseline, bounded within [L0, baseline_level]
  if (requested_level > baseline_level) {
    requested_level = baseline_level;
  }
  if (requested_level < avr::spatial::L0) {
    requested_level = avr::spatial::L0;
  }

  ResolutionDecision decision;
  decision.baseline_level = baseline_level;
  decision.requested_level = requested_level;
  decision.evidence_state = evidence_state;
  decision.reasons = reasons;

  return decision;
}

}  // namespace avr::mapping
