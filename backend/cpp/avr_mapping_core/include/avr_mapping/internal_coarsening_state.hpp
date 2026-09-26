#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Categorical historical completeness of an internal node at coarsening base establishment.
 */
enum class CoarseningBaseState : uint8_t {
  /// Parent had no active elevation payload when split and had no replay gap.
  COMPLETE_EMPTY = 0,
  /// Parent had complete active payload when split; immutable retired cache is canonical historical base.
  COMPLETE_RETIRED_CACHE = 1,
  /// Parent was already REPLAY_REQUIRED when split; historical completeness is not known.
  INCOMPLETE = 2,
};

/**
 * @brief Persistent internal-node coarsening-history state.
 *
 * Implements M9B2A exact internal coarsening shadow foundation:
 * - current_shadow represents the full currently known measurement/temporal state the INTERNAL
 *   node would possess if it had remained an active LEAF and received the same sequence of
 *   real sensor-frame observations.
 * - It is NOT only post-split data, an aggregated delta, a replay cache, or a second retired cache.
 * - Maintained sequentially frame-by-frame on real sensor observations.
 * - Preserves bit-for-bit equivalence with unsplit active leaf payloads under identical frame inputs.
 * - Decouples routine coarsening from bounded raw observation cache retention.
 */
struct InternalCoarseningState {
  CoarseningBaseState base_state{CoarseningBaseState::COMPLETE_EMPTY};
  bool has_current_shadow{false};
  AdaptiveCellPayload current_shadow{};

  bool operator==(const InternalCoarseningState&) const noexcept = default;
  bool operator!=(const InternalCoarseningState&) const noexcept = default;
};

/**
 * @brief Deterministic sorted snapshot entry for internal coarsening states.
 */
using InternalCoarseningSnapshot =
    std::vector<std::pair<avr::spatial::CellKey, InternalCoarseningState>>;

/**
 * @brief Derived summary statistics for internal coarsening shadow state.
 *
 * Note: shadow_observation_count is duplicated structural summary data across ancestors.
 * It must NOT be interpreted as additional sensor observations.
 */
struct InternalCoarseningStats {
  size_t internal_coarsening_state_count{0};
  size_t complete_empty_internal_count{0};
  size_t complete_retired_internal_count{0};
  size_t incomplete_internal_count{0};
  size_t materialized_shadow_count{0};
  uint64_t shadow_observation_count{0};
  size_t sizeof_base_state{sizeof(CoarseningBaseState)};
  size_t sizeof_internal_coarsening_state{sizeof(InternalCoarseningState)};
  size_t alignof_internal_coarsening_state{alignof(InternalCoarseningState)};
  size_t map_bucket_count{0};

  bool operator==(const InternalCoarseningStats&) const noexcept = default;
  bool operator!=(const InternalCoarseningStats&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
