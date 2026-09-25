#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace avr::mapping {

/**
 * @brief Complete statistics and counters for an adaptive frame processing cycle.
 */
struct AdaptiveFrameStats {
  uint64_t frame_seq{0};

  uint64_t input_points{0};
  uint64_t accepted_points{0};
  uint64_t rejected_non_finite{0};
  uint64_t rejected_out_of_range{0};
  uint64_t rejected_numeric{0};

  uint32_t evidence_tile_count{0};
  uint32_t active_base_cells{0};

  uint32_t seed_root_count{0};

  uint32_t regions_evaluated{0};
  uint32_t regions_split{0};
  uint32_t empty_children_held{0};

  size_t final_hierarchy_tile_count{0};
  size_t final_node_count{0};
  size_t final_leaf_count{0};
  size_t final_internal_count{0};

  std::array<size_t, 4> leaf_count_by_level{0, 0, 0, 0};

  uint32_t regions_with_insufficient_evidence{0};
  uint32_t regions_with_geometry_reason{0};
  uint32_t regions_with_distance_only{0};

  [[nodiscard]] constexpr bool isInputPartitionConsistent() const noexcept {
    return input_points ==
           (accepted_points + rejected_non_finite + rejected_out_of_range + rejected_numeric);
  }

  [[nodiscard]] constexpr bool isHierarchyConsistent() const noexcept {
    return final_node_count == (final_leaf_count + final_internal_count) &&
           (leaf_count_by_level[0] + leaf_count_by_level[1] + leaf_count_by_level[2] +
            leaf_count_by_level[3]) == final_leaf_count;
  }

  constexpr bool operator==(const AdaptiveFrameStats&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}  // namespace AVR
