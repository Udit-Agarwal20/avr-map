#pragma once

#include <cmath>

namespace avr::mapping {

/**
 * @brief Policy configuration parameters for semantic perception and dynamic filtering.
 */
struct SemanticPolicyConfig {
  float min_semantic_confidence{0.50f};
  float min_motion_confidence{0.70f};

  bool vehicle_safety_critical{true};
  bool pedestrian_safety_critical{true};

  [[nodiscard]] bool isValid() const noexcept {
    return std::isfinite(min_semantic_confidence) &&
           min_semantic_confidence >= 0.0f && min_semantic_confidence <= 1.0f &&
           std::isfinite(min_motion_confidence) &&
           min_motion_confidence >= 0.0f && min_motion_confidence <= 1.0f;
  }

  constexpr bool operator==(const SemanticPolicyConfig&) const noexcept = default;
};

}  // namespace avr::mapping
