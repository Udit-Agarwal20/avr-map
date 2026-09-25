#pragma once

#include <cstddef>
#include <cstdint>

namespace avr::mapping {

/**
 * @brief Exact structural and memory payload statistics of UniformMap.
 *
 * Note: Payload bytes (tile_payload_bytes, active_cell_payload_bytes) reflect exact
 * sizeof() structural payload data. They are explicitly distinguished from and do NOT claim
 * to represent total process memory, allocator overhead, unordered_map node memory, or process RSS.
 */
struct MapStats {
  size_t tile_count{0};
  size_t active_cell_count{0};
  size_t allocated_cell_slots{0};
  size_t total_accepted_observations{0};
  size_t bucket_count{0};
  float load_factor{0.0f};
  size_t sizeof_uniform_cell{0};
  size_t sizeof_uniform_tile{0};
  size_t tile_payload_bytes{0};
  size_t active_cell_payload_bytes{0};
};

/**
 * @brief Deterministic statistics for a single integrateFrame execution.
 */
struct FrameUpdateStats {
  size_t input_points{0};
  size_t accepted_points{0};
  size_t rejected_non_finite{0};
  size_t rejected_out_of_range{0};
  size_t rejected_outside_extent{0};
  size_t rejected_numeric{0};
  size_t tiles_created{0};
  size_t cells_created{0};
  size_t cells_updated{0};
  size_t unique_cells_touched{0};
  uint64_t frame_seq{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
