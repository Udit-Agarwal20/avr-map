#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Input signals for resolution decision evaluation.
 *
 * All numeric signals are strictly validated; NaN, Inf, negative distance,
 * or out-of-range normalized indicators cause the decision to fail.
 */
struct ResolutionSignals {
  double distance_m{0.0};

  SemanticRelevance semantic_relevance{SemanticRelevance::NONE};
  DynamicRelevance dynamic_relevance{DynamicRelevance::NONE};

  uint32_t observation_count{0};
  double density_score{0.0};

  std::optional<double> uncertainty{std::nullopt};
  std::optional<double> geometry_complexity{std::nullopt};

  [[nodiscard]] bool isValid() const noexcept {
    if (!std::isfinite(distance_m) || distance_m < 0.0) return false;
    if (!std::isfinite(density_score) || density_score < 0.0 || density_score > 1.0) return false;

    if (uncertainty.has_value()) {
      if (!std::isfinite(*uncertainty) || *uncertainty < 0.0 || *uncertainty > 1.0) return false;
    }

    if (geometry_complexity.has_value()) {
      if (!std::isfinite(*geometry_complexity) || *geometry_complexity < 0.0 || *geometry_complexity > 1.0) return false;
    }

    if (static_cast<uint8_t>(semantic_relevance) > static_cast<uint8_t>(SemanticRelevance::SAFETY_CRITICAL)) {
      return false;
    }
    if (static_cast<uint8_t>(dynamic_relevance) > static_cast<uint8_t>(DynamicRelevance::CONFIRMED)) {
      return false;
    }

    return true;
  }

  constexpr bool operator==(const ResolutionSignals& other) const noexcept = default;
};

/**
 * @brief Output result of a resolution decision evaluation.
 */
struct ResolutionDecision {
  uint8_t baseline_level{avr::spatial::L3};
  uint8_t requested_level{avr::spatial::L3};
  EvidenceState evidence_state{EvidenceState::INSUFFICIENT};
  ResolutionReasonBits reasons{};

  [[nodiscard]] constexpr bool refinement_requested() const noexcept {
    return requested_level < baseline_level;
  }

  constexpr bool operator==(const ResolutionDecision& other) const noexcept = default;
};

}  // namespace avr::mapping
