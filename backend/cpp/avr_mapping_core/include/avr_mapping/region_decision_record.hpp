#pragma once

#include <cstdint>
#include <optional>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Compact diagnostic record capturing resolution evaluation and target propagation.
 *
 * Reason Lineage Semantics:
 * - local_reasons: Reasons evaluated by ResolutionController for this region footprint.
 * - inherited_reasons: Cumulative reason lineage passed down from ancestor evaluations (empty for root).
 * - effective_reasons: Diagnostic union (local_reasons | inherited_reasons).
 *   IMPORTANT: effective_reasons represents an ancestral diagnostic context, NOT a minimal mathematical
 *   causal set. It explains why this region or its ancestors required refinement without asserting that
 *   every inherited bit individually dictated the numeric level.
 */
struct RegionDecisionRecord {
  avr::spatial::CellKey region{};

  ResolutionDecision local_decision{};

  uint8_t inherited_target{avr::spatial::L3};
  uint8_t effective_target{avr::spatial::L3};

  ResolutionReasonBits local_reasons{};
  ResolutionReasonBits inherited_reasons{};
  ResolutionReasonBits effective_reasons{};

  uint64_t observation_count{0};
  uint32_t occupied_base_cells{0};
  double density_score{0.0};
  std::optional<double> geometry_complexity{std::nullopt};

  bool split_applied{false};

  constexpr bool operator==(const RegionDecisionRecord&) const noexcept = default;

  [[nodiscard]] constexpr bool operator<(const RegionDecisionRecord& other) const noexcept {
    return region < other.region;
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}  // namespace AVR
