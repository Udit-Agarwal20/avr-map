#pragma once

#include <cstddef>
#include <cstdint>

#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Categorical status of a direct-parent raw observation replay attempt.
 */
enum class ReplayStatus : uint8_t {
  /// Complete replay verified against the complete retired-parent measurement/temporal payload contract.
  /// (Does NOT claim mathematical proof that the retained raw point multiset is identical to the original input history).
  APPLIED_COMPLETE = 0,

  /// Cache window does not span parent history interval [first_seen, last_seen].
  CACHE_HISTORY_INSUFFICIENT = 1,

  /// Parent retired cache has incomplete history; cannot clear descendant gaps.
  PARENT_CACHE_INCOMPLETE = 2,

  /// Reconstructed parent does not match immutable retired parent measurement or temporal history.
  PARENT_RECONSTRUCTION_MISMATCH = 3,

  /// Parent has no retired cache (e.g. split before receiving observations, or gap propagated from ancestor).
  NO_RETIRED_PARENT_CACHE = 4,

  /// Parent is not INTERNAL, or direct children are not all LEAF nodes.
  UNSUPPORTED_TOPOLOGY = 5,

  /// Parent key is invalid level (<= L0) or absent from hierarchy.
  INVALID_PARENT = 6,

  /// Reconstructed child historical observations do not sum to parent retired observations.
  CONSERVATION_FAILURE = 7,

  /// Existing child active payload has first_seen <= parent last_seen.
  TEMPORAL_OVERLAP = 8,

  /// Numeric or container invariant error.
  INTERNAL_ERROR = 9,

  /// All four direct children are already clear of REPLAY_REQUIRED; no-op with zero mutation.
  ALREADY_COMPLETE = 10,

  /// Direct children have mixed replay-required states (some true, some false); inconsistent topology, zero mutation.
  INCONSISTENT_REPLAY_STATE = 11,
};

[[nodiscard]] constexpr const char* replayStatusToString(ReplayStatus status) noexcept {
  switch (status) {
    case ReplayStatus::APPLIED_COMPLETE:
      return "APPLIED_COMPLETE";
    case ReplayStatus::CACHE_HISTORY_INSUFFICIENT:
      return "CACHE_HISTORY_INSUFFICIENT";
    case ReplayStatus::PARENT_CACHE_INCOMPLETE:
      return "PARENT_CACHE_INCOMPLETE";
    case ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH:
      return "PARENT_RECONSTRUCTION_MISMATCH";
    case ReplayStatus::NO_RETIRED_PARENT_CACHE:
      return "NO_RETIRED_PARENT_CACHE";
    case ReplayStatus::UNSUPPORTED_TOPOLOGY:
      return "UNSUPPORTED_TOPOLOGY";
    case ReplayStatus::INVALID_PARENT:
      return "INVALID_PARENT";
    case ReplayStatus::CONSERVATION_FAILURE:
      return "CONSERVATION_FAILURE";
    case ReplayStatus::TEMPORAL_OVERLAP:
      return "TEMPORAL_OVERLAP";
    case ReplayStatus::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case ReplayStatus::ALREADY_COMPLETE:
      return "ALREADY_COMPLETE";
    case ReplayStatus::INCONSISTENT_REPLAY_STATE:
      return "INCONSISTENT_REPLAY_STATE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Detailed diagnostic and accounting metrics for a direct-parent replay attempt.
 *
 * NOTE: parent_replayed_observations, historical_observations_distributed, and
 * children_replay_cleared are diagnostics local to this replay operation. They must
 * not be confused with PersistentAdaptiveMap cumulative ingestion counters.
 */
struct ReplayReconstructionResult {
  avr::spatial::CellKey parent_key{};
  ReplayStatus status{ReplayStatus::INVALID_PARENT};
  size_t cached_frames_considered{0};
  size_t historical_frames_with_parent_observations{0};
  uint64_t parent_replayed_observations{0};
  uint64_t parent_expected_observations{0};
  size_t children_with_historical_payload{0};
  size_t children_with_existing_current_payload{0};
  uint64_t historical_observations_distributed{0};
  size_t children_replay_cleared{0};

  [[nodiscard]] constexpr bool isSuccess() const noexcept {
    return status == ReplayStatus::APPLIED_COMPLETE;
  }

  constexpr bool operator==(const ReplayReconstructionResult&) const noexcept = default;
  constexpr bool operator!=(const ReplayReconstructionResult&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
