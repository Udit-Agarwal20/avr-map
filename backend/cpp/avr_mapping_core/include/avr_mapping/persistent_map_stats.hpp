#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/adaptive_payload_tile.hpp"
#include "avr_mapping/hierarchy_stats.hpp"
#include "avr_mapping/internal_coarsening_state.hpp"

namespace avr::mapping {

/**
 * @brief Structural statistics and memory payload facts for AdaptivePayloadStore.
 */
struct PayloadStoreStats {
  size_t payload_tile_count{0};
  size_t active_payload_count{0};
  size_t retired_cache_count{0};
  size_t replay_required_count{0};
  size_t retired_cache_incomplete_count{0};
  uint64_t active_observation_count{0};
  uint64_t retired_cache_observation_count{0};
  size_t sizeof_payload{sizeof(AdaptiveCellPayload)};
  size_t sizeof_payload_tile{sizeof(AdaptivePayloadTile)};
  size_t payload_tile_bytes{0};

  bool operator==(const PayloadStoreStats&) const noexcept = default;
  bool operator!=(const PayloadStoreStats&) const noexcept = default;
};

/**
 * @brief Diagnostic statistics returned upon a successful frame fusion event.
 */
struct PersistentFusionStats {
  uint64_t frame_seq{0};
  uint64_t input_points{0};
  uint64_t accepted_points{0};
  size_t roots_created{0};
  size_t unique_leaves_fused{0};
  size_t new_payload_cells{0};
  size_t updated_payload_cells{0};
  uint64_t observations_fused{0};
  size_t replay_required_leaves_touched{0};
  size_t payload_tile_count{0};
  size_t active_payload_cells{0};
  size_t retired_cache_cells{0};
  size_t replay_required_leaf_count{0};
  uint64_t last_fused_frame_seq{0};

  // M9B2A ancestor shadow fusion diagnostics (duplicate structural summaries)
  size_t internal_ancestors_considered{0};
  size_t internal_shadows_created{0};
  size_t internal_shadows_updated{0};
  uint64_t internal_shadow_observations_summarized{0};

  bool operator==(const PersistentFusionStats&) const noexcept = default;
  bool operator!=(const PersistentFusionStats&) const noexcept = default;
};

/**
 * @brief Unified cross-subsystem statistics for PersistentAdaptiveMap.
 */
struct PersistentMapStats {
  HierarchyStats hierarchy_stats{};
  PayloadStoreStats payload_stats{};
  InternalCoarseningStats coarsening_stats{};
  uint64_t last_fused_frame_seq{0};
  bool has_fused_frames{false};

  uint64_t total_frames_fused{0};
  uint64_t total_observations_fused{0};

  bool operator==(const PersistentMapStats& other) const noexcept {
    return last_fused_frame_seq == other.last_fused_frame_seq &&
           has_fused_frames == other.has_fused_frames &&
           total_frames_fused == other.total_frames_fused &&
           total_observations_fused == other.total_observations_fused &&
           payload_stats == other.payload_stats &&
           coarsening_stats == other.coarsening_stats &&
           hierarchy_stats.tile_count == other.hierarchy_stats.tile_count &&
           hierarchy_stats.node_count == other.hierarchy_stats.node_count &&
           hierarchy_stats.leaf_count == other.hierarchy_stats.leaf_count &&
           hierarchy_stats.internal_count == other.hierarchy_stats.internal_count &&
           hierarchy_stats.leaf_count_by_level == other.hierarchy_stats.leaf_count_by_level &&
           hierarchy_stats.internal_count_by_level == other.hierarchy_stats.internal_count_by_level;
  }
  bool operator!=(const PersistentMapStats& other) const noexcept {
    return !(*this == other);
  }
};

/**
 * @brief Categorical failure code returned by PersistentAdaptiveMap::validateInvariants().
 */
enum class PersistentMapValidationCode : uint8_t {
  VALID = 0,
  HIERARCHY_INVALID = 1,
  EMPTY_PAYLOAD_TILE_STORED = 2,
  ACTIVE_AND_RETIRED_MASK_INTERSECT = 3,
  INCOMPLETE_BIT_WITHOUT_RETIRED_CACHE = 4,
  ACTIVE_COUNT_MASK_MISMATCH = 5,
  RETIRED_CACHE_COUNT_MASK_MISMATCH = 6,
  REPLAY_REQUIRED_COUNT_MASK_MISMATCH = 7,
  METADATA_ON_ABSENT_NODE = 8,
  ACTIVE_PAYLOAD_ON_NON_LEAF = 9,
  ACTIVE_PAYLOAD_ZERO_OBSERVATIONS = 10,
  ACTIVE_PAYLOAD_FRAME_ORDER_INVALID = 11,
  ACTIVE_PAYLOAD_FRAME_COUNT_INVALID = 12,
  ACTIVE_PAYLOAD_POINT_COUNT_LESS_THAN_FRAMES = 13,
  RETIRED_CACHE_ON_NON_INTERNAL = 14,
  RETIRED_CACHE_ZERO_OBSERVATIONS = 15,
  RETIRED_CACHE_FRAME_ORDER_INVALID = 16,
  RETIRED_CACHE_FRAME_COUNT_INVALID = 17,
  RETIRED_CACHE_POINT_COUNT_LESS_THAN_FRAMES = 18,
  REPLAY_REQUIRED_ON_NON_LEAF = 19,
  GLOBAL_FRAME_SEQUENCING_INCONSISTENT = 20,
  INVALID_LOCAL_INDEX = 21,
  INTERNAL_NODE_MISSING_COARSENING_STATE = 22,
  COARSENING_STATE_ON_NON_INTERNAL = 23,
  COARSENING_BASE_RETIRED_CACHE_MISSING = 24,
  COARSENING_BASE_EMPTY_HAS_RETIRED_CACHE = 25,
  COARSENING_SHADOW_OBSERVATIONS_INVALID = 26,
  COARSENING_SHADOW_FRAME_ORDER_INVALID = 27,
  COARSENING_SHADOW_FRAME_COUNT_INVALID = 28,
  COARSENING_SHADOW_POINT_COUNT_INVALID = 29,
  COARSENING_SHADOW_TEMPORALLY_PRECEDES_BASE = 30,
};

/**
 * @brief Result struct returned by PersistentAdaptiveMap::validateInvariants().
 */
struct PersistentMapValidationResult {
  bool is_valid{true};
  PersistentMapValidationCode code{PersistentMapValidationCode::VALID};
  std::string error_message{};

  [[nodiscard]] explicit operator bool() const noexcept {
    return is_valid;
  }
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
