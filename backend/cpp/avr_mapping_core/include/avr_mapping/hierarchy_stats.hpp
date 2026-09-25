#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "avr_mapping/adaptive_tile.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Exact structural statistics for AdaptiveHierarchy and TilePool.
 *
 * Note: Payload bytes (tile_payload_bytes, pool_reserved_payload_bytes) reflect exact
 * sizeof() structural payload data. They are explicitly distinguished from and do NOT claim
 * to represent total process memory, allocator overhead, unordered_map node memory, or process RSS.
 */
struct HierarchyStats {
  size_t tile_count{0};
  size_t node_count{0};
  size_t leaf_count{0};
  size_t internal_count{0};

  std::array<size_t, 4> leaf_count_by_level{0, 0, 0, 0};
  std::array<size_t, 4> internal_count_by_level{0, 0, 0, 0};

  size_t pool_capacity{0};
  size_t pool_in_use{0};
  size_t pool_free{0};
  size_t pool_block_count{0};

  size_t sizeof_adaptive_tile{0};
  size_t tile_payload_bytes{0};
  size_t pool_reserved_payload_bytes{0};
};

/**
 * @brief Categorization of structural topology invariant violations.
 */
enum class TopologyValidationCategory : uint8_t {
  VALID = 0,
  LEAF_HAS_CHILDREN,
  INTERNAL_MISSING_CHILDREN,
  INTERNAL_CHILD_NOT_PRESENT,
  ORPHAN_NON_ROOT,
  ROOT_HAS_PARENT,
  DUAL_OWNERSHIP,
  INVALID_LEVEL,
  MASK_COUNT_MISMATCH,
  TILE_STORE_POINTER_INVALID,
  OTHER,
};

/**
 * @brief Detailed report returned by invariant validation.
 */
struct TopologyValidationResult {
  bool valid{true};
  TopologyValidationCategory category{TopologyValidationCategory::VALID};
  std::string error_message{};

  [[nodiscard]] explicit operator bool() const noexcept {
    return valid;
  }
};

/**
 * @brief Deterministic record of an allocated hierarchy node for snapshots.
 */
struct TopologyRecord {
  avr::spatial::CellKey key{};
  NodeState state{NodeState::ABSENT};

  constexpr bool operator==(const TopologyRecord&) const noexcept = default;
  constexpr auto operator<=>(const TopologyRecord&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
