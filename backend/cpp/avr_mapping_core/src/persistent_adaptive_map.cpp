#include "avr_mapping/persistent_adaptive_map.hpp"

#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace avr::mapping {

PersistentAdaptiveMap::PersistentAdaptiveMap(size_t pool_block_capacity)
    : hierarchy_(pool_block_capacity) {}

std::optional<avr::spatial::CellKey> PersistentAdaptiveMap::ensureRootForBase(
    const avr::spatial::GridCoord2D& base) {
  return hierarchy_.ensureRootForBase(base);
}

bool PersistentAdaptiveMap::splitLeaf(
    const avr::spatial::CellKey& parent_key,
    ResolutionReasonBits /*reasons*/) {
  // Prevalidation:
  // 1. Parent must be a LEAF in hierarchy, and level > L0
  if (!hierarchy_.isLeaf(parent_key) ||
      parent_key.level == avr::spatial::L0) {
    return false;
  }

  // 2. Derive children keys
  const auto children_opt = avr::spatial::childrenOf(parent_key);
  if (!children_opt.has_value()) {
    return false;
  }

  // 3. Under valid invariants, a leaf MUST NOT already have a retired cache.
  if (payload_store_.findRetiredCache(parent_key) != nullptr) {
    return false;
  }

  // 4. Under valid invariants, children must be currently ABSENT in hierarchy,
  //    and must NOT have active payload, retired cache, or replay-required bits.
  for (const auto& child : *children_opt) {
    if (hierarchy_.nodeState(child) != NodeState::ABSENT) {
      return false;
    }
    if (payload_store_.findActivePayload(child) != nullptr ||
        payload_store_.findRetiredCache(child) != nullptr ||
        payload_store_.requiresReplay(child)) {
      return false;
    }
  }

  // Check parent payload state prior to structural split
  const auto* active_payload = payload_store_.findActivePayload(parent_key);
  const bool has_active_payload = (active_payload != nullptr && active_payload->hasObservations());
  const AdaptiveCellPayload parent_payload_copy =
      has_active_payload ? *active_payload : AdaptiveCellPayload{};
  const bool parent_was_replay_required = payload_store_.requiresReplay(parent_key);

  // Perform structural hierarchy split
  if (!hierarchy_.splitLeaf(parent_key)) {
    return false;
  }

  // Parent is now INTERNAL.
  // Coordinate payload transitions:
  if (has_active_payload) {
    // 1. Parent active payload becomes immutable RETIRED_PARENT_CACHE
    // 2. If parent was replay-required, cache is marked incomplete
    payload_store_.setRetiredCache(
        parent_key, parent_payload_copy, /*incomplete=*/parent_was_replay_required);

    // 3. Parent cannot have replay-required flag while internal
    payload_store_.setReplayRequired(parent_key, false);

    // 4. All four children are marked REPLAY_REQUIRED with NO fabricated aggregate payload
    for (const auto& child : *children_opt) {
      payload_store_.setReplayRequired(child, true);
    }
  } else {
    // Parent had no active payload
    if (parent_was_replay_required) {
      // Clear replay-required on parent (internal nodes cannot be replay required)
      payload_store_.setReplayRequired(parent_key, false);

      // History gap propagates downward: all four children become REPLAY_REQUIRED
      for (const auto& child : *children_opt) {
        payload_store_.setReplayRequired(child, true);
      }
    } else {
      // Parent had no payload and was not replay-required.
      // Children have no payload and are not replay-required.
    }
  }

  payload_store_.pruneEmptyTiles();
  return true;
}

bool PersistentAdaptiveMap::refineToLevel(
    const avr::spatial::GridCoord2D& base,
    uint8_t target_level,
    ResolutionReasonBits reasons) {
  if (!avr::spatial::isValidLevel(target_level)) {
    return false;
  }

  auto root_opt = hierarchy_.ensureRootForBase(base);
  if (!root_opt.has_value()) {
    return false;
  }

  while (true) {
    auto leaf_opt = hierarchy_.findLeafContainingBase(base);
    if (!leaf_opt.has_value()) {
      return false;
    }
    const auto leaf_key = *leaf_opt;
    // Lower numeric value = finer resolution (L0=0, L1=1, L2=2, L3=3)
    if (leaf_key.level <= target_level) {
      break;
    }
    if (leaf_key.level == avr::spatial::L0) {
      break;
    }
    if (!splitLeaf(leaf_key, reasons)) {
      return false;
    }
  }

  return true;
}

std::optional<PersistentFusionStats> PersistentAdaptiveMap::fuseFrameEvidence(
    const FrameEvidence& evidence,
    ResolutionReasonBits reasons) {
  if (!evidence.hasFrame()) {
    return std::nullopt;
  }

  const uint64_t frame_seq = evidence.frameSeq();

  // Strict monotonic frame sequencing check (reject duplicate and out-of-order)
  if (has_fused_frames_ && frame_seq <= last_fused_frame_seq_) {
    return std::nullopt;
  }

  // Handle valid empty frame (accepted_points == 0)
  if (evidence.statistics().accepted_points == 0) {
    last_fused_frame_seq_ = frame_seq;
    has_fused_frames_ = true;
    ++total_frames_fused_;

    PersistentFusionStats stats{};
    stats.frame_seq = frame_seq;
    stats.last_fused_frame_seq = frame_seq;
    stats.input_points = evidence.statistics().input_points;
    stats.accepted_points = 0;
    stats.payload_tile_count = payload_store_.tileCount();
    const auto store_stats = payload_store_.statistics();
    stats.active_payload_cells = store_stats.active_payload_count;
    stats.retired_cache_cells = store_stats.retired_cache_count;
    stats.replay_required_leaf_count = store_stats.replay_required_count;
    return stats;
  }

  // ==========================================
  // PREPARE PHASE (Strictly Side-Effect Free)
  // No mutations to AdaptiveHierarchy, AdaptivePayloadStore, frame sequencing, or persistent stats.
  // ==========================================
  const auto base_cells = evidence.baseCellSnapshot();
  if (base_cells.empty()) {
    return std::nullopt;
  }

  // Missing L3 roots are PLANNED in deterministic CellKey order, NOT yet created in AdaptiveHierarchy.
  std::map<avr::spatial::CellKey, avr::spatial::GridCoord2D> planned_roots;
  std::vector<avr::spatial::CellKey> owning_leaves;
  owning_leaves.reserve(base_cells.size());

  for (const auto& entry : base_cells) {
    const auto coord = entry.key.decode();
    const avr::spatial::GridCoord2D base_grid{coord.x, coord.y};

    if (hierarchy_.hasRootForBase(base_grid)) {
      const auto leaf_opt = hierarchy_.findLeafContainingBase(base_grid);
      if (!leaf_opt.has_value() || !hierarchy_.isLeaf(*leaf_opt)) {
        return std::nullopt;
      }
      owning_leaves.push_back(*leaf_opt);
    } else {
      const auto root_key_opt = AdaptiveHierarchy::rootKeyForBase(base_grid);
      if (!root_key_opt.has_value()) {
        return std::nullopt;
      }
      planned_roots.try_emplace(*root_key_opt, base_grid);
      owning_leaves.push_back(*root_key_opt);
    }
  }

  // Deduplicate owning leaf CellKeys deterministically
  std::sort(owning_leaves.begin(), owning_leaves.end());
  owning_leaves.erase(std::unique(owning_leaves.begin(), owning_leaves.end()), owning_leaves.end());

  struct CandidateUpdate {
    avr::spatial::CellKey key;
    AdaptiveCellPayload payload;
    bool is_new{false};
    bool is_replay_required{false};
    uint64_t observations{0};
  };

  std::vector<CandidateUpdate> candidates;
  candidates.reserve(owning_leaves.size());
  uint64_t total_observations_fused = 0;

  for (const auto& leaf_key : owning_leaves) {
    const auto frame_elev_opt = evidence.aggregateElevationStats(leaf_key);
    if (!frame_elev_opt.has_value() || frame_elev_opt->count() == 0) {
      continue;
    }

    const auto& frame_elev = *frame_elev_opt;
    total_observations_fused += frame_elev.count();

    const auto* existing = payload_store_.findActivePayload(leaf_key);
    const bool is_new = (existing == nullptr || !existing->hasObservations());
    AdaptiveCellPayload candidate = existing ? *existing : AdaptiveCellPayload{};

    if (!candidate.tryFuse(frame_elev, frame_seq, reasons)) {
      // Numerical failure, overflow, or frame ordering violation
      return std::nullopt;
    }

    const bool is_replay = payload_store_.requiresReplay(leaf_key);
    candidates.push_back(CandidateUpdate{
        .key = leaf_key,
        .payload = candidate,
        .is_new = is_new,
        .is_replay_required = is_replay,
        .observations = frame_elev.count(),
    });
  }

  // Observation Conservation verification (Section 33)
  if (total_observations_fused != evidence.statistics().accepted_points) {
    return std::nullopt;
  }

  // ==========================================
  // COMMIT PHASE (Executed only after all candidate checks succeed)
  // ==========================================
  // 1. Ensure planned missing roots in hierarchy in deterministic CellKey order
  for (const auto& [root_key, base_coord] : planned_roots) {
    if (!hierarchy_.ensureRootForBase(base_coord).has_value()) {
      return std::nullopt;
    }
  }

  // 2. Commit candidate payload updates
  size_t new_payload_cells = 0;
  size_t updated_payload_cells = 0;
  size_t replay_leaves_touched = 0;

  for (const auto& cand : candidates) {
    payload_store_.setActivePayload(cand.key, cand.payload);
    if (cand.is_new) {
      ++new_payload_cells;
    } else {
      ++updated_payload_cells;
    }
    if (cand.is_replay_required) {
      ++replay_leaves_touched;
    }
  }

  // 3. Publish frame sequencing state and cumulative counters only AFTER commits succeed
  last_fused_frame_seq_ = frame_seq;
  has_fused_frames_ = true;
  ++total_frames_fused_;
  total_observations_fused_ += total_observations_fused;

  // 4. Assemble statistics
  const auto store_stats = payload_store_.statistics();
  PersistentFusionStats stats{
      .frame_seq = frame_seq,
      .input_points = evidence.statistics().input_points,
      .accepted_points = evidence.statistics().accepted_points,
      .roots_created = planned_roots.size(),
      .unique_leaves_fused = candidates.size(),
      .new_payload_cells = new_payload_cells,
      .updated_payload_cells = updated_payload_cells,
      .observations_fused = total_observations_fused,
      .replay_required_leaves_touched = replay_leaves_touched,
      .payload_tile_count = payload_store_.tileCount(),
      .active_payload_cells = store_stats.active_payload_count,
      .retired_cache_cells = store_stats.retired_cache_count,
      .replay_required_leaf_count = store_stats.replay_required_count,
      .last_fused_frame_seq = last_fused_frame_seq_,
  };

  return stats;
}

PersistentMapValidationResult PersistentAdaptiveMap::validateInvariants() const {
  const auto topo_res = hierarchy_.validateInvariants();
  if (!topo_res.valid) {
    return PersistentMapValidationResult{
        .is_valid = false,
        .code = PersistentMapValidationCode::HIERARCHY_INVALID,
        .error_message = "AdaptiveHierarchy topology invalid: " + topo_res.error_message,
    };
  }

  for (const auto& [tile_key, tile] : payload_store_.tiles()) {
    if (tile.isEmpty()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::EMPTY_PAYLOAD_TILE_STORED,
          .error_message = "Empty payload tile stored in AdaptivePayloadStore",
      };
    }

    if ((tile.activeMask() & tile.retiredCacheMask()).any()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::ACTIVE_AND_RETIRED_MASK_INTERSECT,
          .error_message = "active_payload_mask and retired_cache_mask intersect",
      };
    }

    if ((tile.retiredCacheIncompleteMask() & ~tile.retiredCacheMask()).any()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::INCOMPLETE_BIT_WITHOUT_RETIRED_CACHE,
          .error_message = "retired_cache_incomplete_mask has bits not in retired_cache_mask",
      };
    }

    if (tile.activeCount() != tile.activeMask().count()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::ACTIVE_COUNT_MASK_MISMATCH,
          .error_message = "activeCount() mismatch with activeMask()",
      };
    }

    if (tile.retiredCacheCount() != tile.retiredCacheMask().count()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::RETIRED_CACHE_COUNT_MASK_MISMATCH,
          .error_message = "retiredCacheCount() mismatch with retiredCacheMask()",
      };
    }

    if (tile.replayRequiredCount() != tile.replayRequiredMask().count()) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::REPLAY_REQUIRED_COUNT_MASK_MISMATCH,
          .error_message = "replayRequiredCount() mismatch with replayRequiredMask()",
      };
    }

    for (size_t i = 0; i < AdaptivePayloadTile::CELL_COUNT; ++i) {
      const uint16_t local_idx = static_cast<uint16_t>(i);
      const bool has_meta = tile.isActive(local_idx) ||
                            tile.isRetiredCache(local_idx) ||
                            tile.isReplayRequired(local_idx);
      if (!has_meta) {
        continue;
      }

      const auto cell_key_opt = avr::spatial::cellKeyFromTileAndLocal(tile_key, local_idx);
      if (!cell_key_opt.has_value()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::INVALID_LOCAL_INDEX,
            .error_message = "Invalid local index in tile",
        };
      }
      const auto cell_key = *cell_key_opt;
      const auto state = hierarchy_.nodeState(cell_key);

      if (state == NodeState::ABSENT) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::METADATA_ON_ABSENT_NODE,
            .error_message = "Payload metadata exists for ABSENT hierarchy node",
        };
      }

      if (tile.isActive(local_idx)) {
        if (state != NodeState::LEAF) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::ACTIVE_PAYLOAD_ON_NON_LEAF,
              .error_message = "ACTIVE_PAYLOAD exists on non-LEAF node",
          };
        }
        const auto& p = tile.cell(local_idx);
        if (!p.hasObservations()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::ACTIVE_PAYLOAD_ZERO_OBSERVATIONS,
              .error_message = "ACTIVE_PAYLOAD has 0 observations",
          };
        }
        if (p.firstSeenFrame() > p.lastSeenFrame()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::ACTIVE_PAYLOAD_FRAME_ORDER_INVALID,
              .error_message = "first_seen_frame > last_seen_frame",
          };
        }
        if (p.observedFrameCount() < 1) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::ACTIVE_PAYLOAD_FRAME_COUNT_INVALID,
              .error_message = "observed_frame_count < 1",
          };
        }
        if (p.elevation().count() < p.observedFrameCount()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::ACTIVE_PAYLOAD_POINT_COUNT_LESS_THAN_FRAMES,
              .error_message = "elevation.count() < observed_frame_count",
          };
        }
      }

      if (tile.isRetiredCache(local_idx)) {
        if (state != NodeState::INTERNAL) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::RETIRED_CACHE_ON_NON_INTERNAL,
              .error_message = "RETIRED_PARENT_CACHE exists on non-INTERNAL node",
          };
        }
        const auto& p = tile.cell(local_idx);
        if (!p.hasObservations()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::RETIRED_CACHE_ZERO_OBSERVATIONS,
              .error_message = "RETIRED_PARENT_CACHE has 0 observations",
          };
        }
        if (p.firstSeenFrame() > p.lastSeenFrame()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::RETIRED_CACHE_FRAME_ORDER_INVALID,
              .error_message = "retired cache first_seen_frame > last_seen_frame",
          };
        }
        if (p.observedFrameCount() < 1) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::RETIRED_CACHE_FRAME_COUNT_INVALID,
              .error_message = "retired cache observed_frame_count < 1",
          };
        }
        if (p.elevation().count() < p.observedFrameCount()) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::RETIRED_CACHE_POINT_COUNT_LESS_THAN_FRAMES,
              .error_message = "retired cache elevation.count() < observed_frame_count",
          };
        }
      }

      if (tile.isReplayRequired(local_idx)) {
        if (state != NodeState::LEAF) {
          return PersistentMapValidationResult{
              .is_valid = false,
              .code = PersistentMapValidationCode::REPLAY_REQUIRED_ON_NON_LEAF,
              .error_message = "REPLAY_REQUIRED exists on non-LEAF node",
          };
        }
      }
    }
  }

  if (!has_fused_frames_ && last_fused_frame_seq_ != 0) {
    return PersistentMapValidationResult{
        .is_valid = false,
        .code = PersistentMapValidationCode::GLOBAL_FRAME_SEQUENCING_INCONSISTENT,
        .error_message = "has_fused_frames is false but last_fused_frame_seq is non-zero",
    };
  }

  return PersistentMapValidationResult{
      .is_valid = true,
      .code = PersistentMapValidationCode::VALID,
      .error_message = "",
  };
}

PersistentMapStats PersistentAdaptiveMap::statistics() const noexcept {
  return PersistentMapStats{
      .hierarchy_stats = hierarchy_.statistics(),
      .payload_stats = payload_store_.statistics(),
      .last_fused_frame_seq = last_fused_frame_seq_,
      .has_fused_frames = has_fused_frames_,
      .total_frames_fused = total_frames_fused_,
      .total_observations_fused = total_observations_fused_,
  };
}

void PersistentAdaptiveMap::reset() noexcept {
  hierarchy_.reset();
  payload_store_.reset();
  last_fused_frame_seq_ = 0;
  has_fused_frames_ = false;
  total_frames_fused_ = 0;
  total_observations_fused_ = 0;
}

}  // namespace avr::mapping
