#pragma once

#include <cmath>
#include <cstdint>
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

/**
 * @brief Abstract model-agnostic semantic importance category.
 */
enum class SemanticRelevance : uint8_t {
  NONE = 0,
  ELEVATED = 1,
  SAFETY_CRITICAL = 2,
};

/**
 * @brief Abstract motion relevance category.
 *
 * Distinguishes dynamic-capable semantic classes from confirmed moving objects.
 */
enum class DynamicRelevance : uint8_t {
  NONE = 0,
  CANDIDATE = 1,
  CONFIRMED = 2,
};

/**
 * @brief Information-density evidence sufficiency state.
 */
enum class EvidenceState : uint8_t {
  SUFFICIENT = 0,
  INSUFFICIENT = 1,
};

/**
 * @brief Categorical reason flags participating in resolution decisions.
 */
enum class ResolutionReason : uint16_t {
  NONE = 0,
  DISTANCE_BASELINE = 1 << 0,
  SEMANTIC_RELEVANCE = 1 << 1,
  DYNAMIC_RELEVANCE = 1 << 2,
  UNCERTAINTY = 1 << 3,
  GEOMETRY_COMPLEXITY = 1 << 4,
  INSUFFICIENT_EVIDENCE = 1 << 5,
  DENSITY_GATE_BLOCKED = 1 << 6,
};

/**
 * @brief Compact, strongly-typed bitmask representing active decision reasons.
 */
struct ResolutionReasonBits {
  uint16_t mask{0};

  constexpr ResolutionReasonBits() noexcept = default;
  constexpr explicit ResolutionReasonBits(uint16_t m) noexcept : mask(m) {}
  constexpr ResolutionReasonBits(ResolutionReason r) noexcept : mask(static_cast<uint16_t>(r)) {}

  [[nodiscard]] constexpr bool has(ResolutionReason r) const noexcept {
    return (mask & static_cast<uint16_t>(r)) != 0;
  }

  constexpr void set(ResolutionReason r) noexcept {
    mask |= static_cast<uint16_t>(r);
  }

  constexpr void clear(ResolutionReason r) noexcept {
    mask &= ~static_cast<uint16_t>(r);
  }

  [[nodiscard]] constexpr bool empty() const noexcept {
    return mask == 0;
  }

  constexpr bool operator==(const ResolutionReasonBits& other) const noexcept = default;

  constexpr ResolutionReasonBits operator|(ResolutionReason r) const noexcept {
    return ResolutionReasonBits(static_cast<uint16_t>(mask | static_cast<uint16_t>(r)));
  }

  constexpr ResolutionReasonBits operator|(ResolutionReasonBits other) const noexcept {
    return ResolutionReasonBits(static_cast<uint16_t>(mask | other.mask));
  }

  constexpr ResolutionReasonBits& operator|=(ResolutionReason r) noexcept {
    mask |= static_cast<uint16_t>(r);
    return *this;
  }

  constexpr ResolutionReasonBits& operator|=(ResolutionReasonBits other) noexcept {
    mask |= other.mask;
    return *this;
  }
};

/**
 * @brief Immutable configuration defining the adaptive resolution policy and density gate.
 *
 * All thresholds represent configurable prototype defaults and will be calibrated on real datasets.
 */
struct ResolutionPolicyConfig {
  double distance_threshold_1{10.0};  // [0, d1) -> L0 (5 cm)
  double distance_threshold_2{30.0};  // [d1, d2) -> L1 (10 cm)
  double distance_threshold_3{60.0};  // [d2, d3) -> L2 (20 cm); >= d3 -> L3 (40 cm)

  uint32_t min_observations{4};
  double min_density_score{0.25};

  double high_uncertainty_threshold{0.60};
  double high_geometry_threshold{0.50};

  uint8_t semantic_elevated_target{avr::spatial::L1};
  uint8_t semantic_critical_target{avr::spatial::L0};
  uint8_t confirmed_dynamic_target{avr::spatial::L0};

  uint8_t uncertainty_refinement_steps{1};
  uint8_t geometry_refinement_steps{1};

  [[nodiscard]] bool validate() const noexcept {
    if (!std::isfinite(distance_threshold_1) || distance_threshold_1 < 0.0) return false;
    if (!std::isfinite(distance_threshold_2) || distance_threshold_2 < 0.0) return false;
    if (!std::isfinite(distance_threshold_3) || distance_threshold_3 < 0.0) return false;
    if (!(distance_threshold_1 < distance_threshold_2 && distance_threshold_2 < distance_threshold_3)) return false;

    if (!std::isfinite(min_density_score) || min_density_score < 0.0 || min_density_score > 1.0) return false;
    if (!std::isfinite(high_uncertainty_threshold) || high_uncertainty_threshold < 0.0 || high_uncertainty_threshold > 1.0) return false;
    if (!std::isfinite(high_geometry_threshold) || high_geometry_threshold < 0.0 || high_geometry_threshold > 1.0) return false;

    if (min_observations < 1) return false;

    if (!avr::spatial::isValidLevel(semantic_elevated_target)) return false;
    if (!avr::spatial::isValidLevel(semantic_critical_target)) return false;
    if (!avr::spatial::isValidLevel(confirmed_dynamic_target)) return false;

    // Safety critical must never be coarser than elevated (lower level number is finer)
    if (semantic_critical_target > semantic_elevated_target) return false;

    if (uncertainty_refinement_steps > 3 || geometry_refinement_steps > 3) return false;

    return true;
  }
};

}  // namespace avr::mapping
