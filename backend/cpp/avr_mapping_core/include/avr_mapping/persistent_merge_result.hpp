#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Categorical outcome of a PersistentAdaptiveMap::mergeDirectChildren operation.
 */
enum class MergeStatus : uint8_t {
  APPLIED = 0,
  INVALID_PARENT = 1,
  PARENT_NOT_INTERNAL = 2,
  CHILD_NOT_LEAF = 3,
  REPLAY_INCOMPLETE = 4,
  CACHE_HISTORY_INSUFFICIENT = 5,
  CHILD_RECONSTRUCTION_MISMATCH = 6,
  CONSERVATION_FAILURE = 7,
  TEMPORAL_METADATA_INVALID = 8,
  PAYLOAD_MERGE_FAILED = 9,
  INCONSISTENT_PAYLOAD_STATE = 10,
  INTERNAL_ERROR = 11,
  PARENT_RECONSTRUCTION_MISMATCH = 12,
};

[[nodiscard]] constexpr std::string_view mergeStatusToString(MergeStatus status) noexcept {
  switch (status) {
    case MergeStatus::APPLIED:
      return "APPLIED";
    case MergeStatus::INVALID_PARENT:
      return "INVALID_PARENT";
    case MergeStatus::PARENT_NOT_INTERNAL:
      return "PARENT_NOT_INTERNAL";
    case MergeStatus::CHILD_NOT_LEAF:
      return "CHILD_NOT_LEAF";
    case MergeStatus::REPLAY_INCOMPLETE:
      return "REPLAY_INCOMPLETE";
    case MergeStatus::CACHE_HISTORY_INSUFFICIENT:
      return "CACHE_HISTORY_INSUFFICIENT";
    case MergeStatus::CHILD_RECONSTRUCTION_MISMATCH:
      return "CHILD_RECONSTRUCTION_MISMATCH";
    case MergeStatus::CONSERVATION_FAILURE:
      return "CONSERVATION_FAILURE";
    case MergeStatus::TEMPORAL_METADATA_INVALID:
      return "TEMPORAL_METADATA_INVALID";
    case MergeStatus::PAYLOAD_MERGE_FAILED:
      return "PAYLOAD_MERGE_FAILED";
    case MergeStatus::INCONSISTENT_PAYLOAD_STATE:
      return "INCONSISTENT_PAYLOAD_STATE";
    case MergeStatus::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
    case MergeStatus::PARENT_RECONSTRUCTION_MISMATCH:
      return "PARENT_RECONSTRUCTION_MISMATCH";
  }
  return "UNKNOWN";
}

/**
 * @brief Detailed operation diagnostics returned by mergeDirectChildren.
 */
struct PersistentMergeResult {
  avr::spatial::CellKey parent{};
  MergeStatus status{MergeStatus::INVALID_PARENT};
  size_t cached_frames_considered{0};
  size_t children_reconstructed{0};
  uint64_t child_observations_verified{0};
  uint64_t parent_observations_reconstructed{0};
  bool had_parent_retired_cache{false};
  bool parent_payload_created{false};

  [[nodiscard]] bool isSuccess() const noexcept {
    return status == MergeStatus::APPLIED;
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
