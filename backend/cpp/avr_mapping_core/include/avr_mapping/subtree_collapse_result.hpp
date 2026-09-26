#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Categorical outcome of PersistentAdaptiveMap::collapseSubtreeFromCompleteShadow.
 */
enum class SubtreeCollapseStatus : uint8_t {
  APPLIED_GAP_CLOSED = 0,
  INVALID_ANCESTOR = 1,
  ANCESTOR_NOT_INTERNAL = 2,
  ANCESTOR_BASE_INCOMPLETE = 3,
  INVALID_SUBTREE_STATE = 4,
  DESCENDANT_COVERAGE_INCONSISTENT = 5,
  PAYLOAD_STATE_INVALID = 6,
  INTERNAL_ERROR = 7,
};

[[nodiscard]] constexpr std::string_view subtreeCollapseStatusToString(
    SubtreeCollapseStatus status) noexcept {
  switch (status) {
    case SubtreeCollapseStatus::APPLIED_GAP_CLOSED:
      return "APPLIED_GAP_CLOSED";
    case SubtreeCollapseStatus::INVALID_ANCESTOR:
      return "INVALID_ANCESTOR";
    case SubtreeCollapseStatus::ANCESTOR_NOT_INTERNAL:
      return "ANCESTOR_NOT_INTERNAL";
    case SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE:
      return "ANCESTOR_BASE_INCOMPLETE";
    case SubtreeCollapseStatus::INVALID_SUBTREE_STATE:
      return "INVALID_SUBTREE_STATE";
    case SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT:
      return "DESCENDANT_COVERAGE_INCONSISTENT";
    case SubtreeCollapseStatus::PAYLOAD_STATE_INVALID:
      return "PAYLOAD_STATE_INVALID";
    case SubtreeCollapseStatus::INTERNAL_ERROR:
      return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

/**
 * @brief Detailed structural and diagnostic result of collapseSubtreeFromCompleteShadow.
 */
struct SubtreeCollapseResult {
  avr::spatial::CellKey ancestor{};
  SubtreeCollapseStatus status{SubtreeCollapseStatus::INVALID_ANCESTOR};
  uint8_t ancestor_level{0};
  uint8_t finest_removed_level{0};
  size_t descendant_nodes_removed{0};
  size_t descendant_internal_nodes_removed{0};
  size_t descendant_leaves_removed{0};
  size_t replay_required_leaves_closed{0};
  size_t active_descendant_payloads_removed{0};
  uint64_t active_descendant_observations_removed{0};
  uint64_t candidate_parent_observations{0};
  bool had_materialized_shadow{false};
  bool had_retired_cache{false};
  uint8_t levels_collapsed{0};

  /**
   * @brief Canonical list of all removed hierarchy descendant keys (leaves and internal).
   *
   * Excludes the restored ancestor. Guaranteed to be sorted canonically, unique, and empty on failure.
   * Enables downstream coordinators (e.g. M9B1 hysteresis) to purge associated region states.
   */
  std::vector<avr::spatial::CellKey> removed_descendant_keys{};

  [[nodiscard]] bool isSuccess() const noexcept {
    return status == SubtreeCollapseStatus::APPLIED_GAP_CLOSED;
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
