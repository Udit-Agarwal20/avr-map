#pragma once

#include <optional>
#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"

namespace avr::mapping {

/**
 * @brief Deterministic, pure C++ resolution decision engine and information-density gate.
 *
 * Maps continuous distance, normalized evidence density, uncertainty, geometry complexity,
 * and categorical semantic and dynamic cues to target hierarchy resolution levels.
 *
 * Invariants:
 * - Deterministic and stateless: identical inputs yield identical decisions; no temporal state.
 * - Map-mutation-independent: does not mutate or reference AdaptiveHierarchy.
 * - Thread-safe: const methods perform no internal mutation; concurrent decide() calls
 *   on the same controller instance are thread-safe.
 * - No coarsening: requested_level <= baseline_level for all valid decisions.
 */
class ResolutionController {
 public:
  /**
   * @brief Factory creating a validated ResolutionController.
   *
   * @param config Policy configuration.
   * @return Validated controller instance, or std::nullopt if config validation fails.
   */
  [[nodiscard]] static std::optional<ResolutionController> create(
      const ResolutionPolicyConfig& config = ResolutionPolicyConfig{}) noexcept;

  /**
   * @brief Evaluates resolution signals and produces a deterministic decision.
   *
   * @param signals Input signals to evaluate.
   * @return Computed decision, or std::nullopt if signals are numerically invalid.
   */
  [[nodiscard]] std::optional<ResolutionDecision> decide(
      const ResolutionSignals& signals) const noexcept;

  /**
   * @brief Accesses the immutable policy configuration.
   */
  [[nodiscard]] const ResolutionPolicyConfig& config() const noexcept {
    return config_;
  }

 private:
  explicit ResolutionController(const ResolutionPolicyConfig& config) noexcept
      : config_(config) {}

  ResolutionPolicyConfig config_;
};

}  // namespace avr::mapping
