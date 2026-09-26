#pragma once

#include <cstdint>

#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Per-region tracking state in ResolutionHysteresisController.
 */
struct RegionHysteresisState {
  uint64_t last_frame_seq{0};
  uint32_t stable_coarsen_count{0};
  ResolutionReasonBits latest_reasons{};
  uint8_t pending_target_level{0};
  bool has_last_frame{false};
  bool coarsen_action_pending{false};

  constexpr bool operator==(const RegionHysteresisState&) const noexcept = default;
};

/**
 * @brief Deterministic, sortable snapshot entry of tracked region hysteresis state.
 */
struct RegionHysteresisSnapshot {
  avr::spatial::CellKey region{};
  uint64_t last_frame_seq{0};
  uint32_t stable_coarsen_count{0};
  ResolutionReasonBits latest_reasons{};
  uint8_t pending_target_level{0};
  bool has_last_frame{false};
  bool coarsen_action_pending{false};

  constexpr bool operator==(const RegionHysteresisSnapshot&) const noexcept = default;
  constexpr auto operator<=>(const RegionHysteresisSnapshot& other) const noexcept {
    return region <=> other.region;
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
