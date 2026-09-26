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

  // 3. Under valid invariants, a leaf MUST NOT already have a retired cache or coarsening state.
  if (payload_store_.findRetiredCache(parent_key) != nullptr ||
      internal_coarsening_states_.find(parent_key) != internal_coarsening_states_.end()) {
    return false;
  }

  // 4. Under valid invariants, children must be currently ABSENT in hierarchy,
  //    and must NOT have active payload, retired cache, replay-required bits, or coarsening state.
  for (const auto& child : *children_opt) {
    if (hierarchy_.nodeState(child) != NodeState::ABSENT) {
      return false;
    }
    if (payload_store_.findActivePayload(child) != nullptr ||
        payload_store_.findRetiredCache(child) != nullptr ||
        payload_store_.requiresReplay(child) ||
        internal_coarsening_states_.find(child) != internal_coarsening_states_.end()) {
      return false;
    }
  }

  // Check parent payload and coarsening state prior to structural split
  if (internal_coarsening_states_.find(parent_key) != internal_coarsening_states_.end()) {
    return false;
  }
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

  // Coordinate internal coarsening state creation (Sections 6, 12, 13, 14, 15)
  CoarseningBaseState base_state = CoarseningBaseState::COMPLETE_EMPTY;
  if (has_active_payload) {
    base_state = parent_was_replay_required ? CoarseningBaseState::INCOMPLETE
                                            : CoarseningBaseState::COMPLETE_RETIRED_CACHE;
  } else if (parent_was_replay_required) {
    base_state = CoarseningBaseState::INCOMPLETE;
  } else {
    base_state = CoarseningBaseState::COMPLETE_EMPTY;
  }

  internal_coarsening_states_[parent_key] = InternalCoarseningState{
      .base_state = base_state,
      .has_current_shadow = false,
      .current_shadow = AdaptiveCellPayload{},
  };

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

  // Section 19 & 20: Ancestor Discovery and Canonical Ancestor Order
  // Walk parentOf(...) up through hierarchy from each touched active leaf.
  // Collect only nodes currently NodeState::INTERNAL.
  // Deduplicate all ancestor CellKeys deterministically and sort canonically.
  std::vector<avr::spatial::CellKey> ancestor_keys;
  for (const auto& leaf_key : owning_leaves) {
    auto current = leaf_key;
    while (current.level < avr::spatial::L3) {
      const auto p_opt = avr::spatial::parentOf(current);
      if (!p_opt.has_value()) {
        break;
      }
      const auto parent_key = *p_opt;
      if (hierarchy_.nodeState(parent_key) == NodeState::INTERNAL) {
        ancestor_keys.push_back(parent_key);
      }
      current = parent_key;
    }
  }
  std::sort(ancestor_keys.begin(), ancestor_keys.end());
  ancestor_keys.erase(std::unique(ancestor_keys.begin(), ancestor_keys.end()), ancestor_keys.end());

  struct CandidateShadowUpdate {
    avr::spatial::CellKey key;
    AdaptiveCellPayload shadow;
    bool is_created{false};
    uint64_t observations{0};
  };

  std::vector<CandidateShadowUpdate> shadow_candidates;
  shadow_candidates.reserve(ancestor_keys.size());
  const size_t ancestors_considered = ancestor_keys.size();

  // Section 16, 17, 18, 21: Candidate Shadow Preparation
  for (const auto& ancestor_key : ancestor_keys) {
    const auto frame_elev_opt = evidence.aggregateElevationStats(ancestor_key);
    if (!frame_elev_opt.has_value() || frame_elev_opt->count() == 0) {
      // Section 18: No update for zero-contribution frame
      continue;
    }

    const auto& frame_elev = *frame_elev_opt;
    const auto it = internal_coarsening_states_.find(ancestor_key);
    if (it == internal_coarsening_states_.end()) {
      return std::nullopt;
    }

    AdaptiveCellPayload cand_shadow;
    bool is_created = false;
    if (!it->second.has_current_shadow) {
      is_created = true;
      if (it->second.base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE) {
        const auto* retired = payload_store_.findRetiredCache(ancestor_key);
        if (retired == nullptr || !retired->hasObservations()) {
          return std::nullopt;
        }
        cand_shadow = *retired;
      } else if (it->second.base_state == CoarseningBaseState::COMPLETE_EMPTY) {
        cand_shadow = AdaptiveCellPayload{};
      } else if (it->second.base_state == CoarseningBaseState::INCOMPLETE) {
        const auto* retired = payload_store_.findRetiredCache(ancestor_key);
        if (retired != nullptr && retired->hasObservations()) {
          cand_shadow = *retired;
        } else {
          cand_shadow = AdaptiveCellPayload{};
        }
      }
    } else {
      cand_shadow = it->second.current_shadow;
    }

    // Shadow last_update_reasons is NOT authoritative (Section 9)
    if (!cand_shadow.tryFuse(frame_elev, frame_seq, ResolutionReasonBits{})) {
      // Numerical failure, overflow, or frame ordering violation
      return std::nullopt;
    }

    shadow_candidates.push_back(CandidateShadowUpdate{
        .key = ancestor_key,
        .shadow = cand_shadow,
        .is_created = is_created,
        .observations = frame_elev.count(),
    });
  }

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

  // 3. Commit internal shadow candidates (Section 27)
  size_t shadows_created = 0;
  size_t shadows_updated = 0;
  uint64_t shadow_observations_summarized = 0;

  for (const auto& sc : shadow_candidates) {
    auto it = internal_coarsening_states_.find(sc.key);
    if (it != internal_coarsening_states_.end()) {
      it->second.current_shadow = sc.shadow;
      it->second.has_current_shadow = true;
    }
    if (sc.is_created) {
      ++shadows_created;
    } else {
      ++shadows_updated;
    }
    shadow_observations_summarized += sc.observations;
  }

  // 4. Publish frame sequencing state and cumulative counters only AFTER commits succeed
  // Note: Shadow updates do NOT increment total_observations_fused_ (Section 28)
  last_fused_frame_seq_ = frame_seq;
  has_fused_frames_ = true;
  ++total_frames_fused_;
  total_observations_fused_ += total_observations_fused;

  // 5. Assemble statistics
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
      .internal_ancestors_considered = ancestors_considered,
      .internal_shadows_created = shadows_created,
      .internal_shadows_updated = shadows_updated,
      .internal_shadow_observations_summarized = shadow_observations_summarized,
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

  // A. Every INTERNAL hierarchy node has exactly one InternalCoarseningState
  const auto topo_nodes = hierarchy_.topologySnapshot();
  for (const auto& node : topo_nodes) {
    if (node.state == NodeState::INTERNAL) {
      if (internal_coarsening_states_.find(node.key) == internal_coarsening_states_.end()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::INTERNAL_NODE_MISSING_COARSENING_STATE,
            .error_message = "INTERNAL hierarchy node missing InternalCoarseningState",
        };
      }
    }
  }

  // B & C. No LEAF or ABSENT node owns InternalCoarseningState, plus D-H base and shadow constraints
  for (const auto& [key, state] : internal_coarsening_states_) {
    const auto node_st = hierarchy_.nodeState(key);
    if (node_st != NodeState::INTERNAL) {
      return PersistentMapValidationResult{
          .is_valid = false,
          .code = PersistentMapValidationCode::COARSENING_STATE_ON_NON_INTERNAL,
          .error_message = "InternalCoarseningState exists for non-INTERNAL node",
      };
    }

    if (state.base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE) {
      const auto* ret = payload_store_.findRetiredCache(key);
      if (ret == nullptr || payload_store_.isRetiredCacheIncomplete(key)) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_BASE_RETIRED_CACHE_MISSING,
            .error_message = "COMPLETE_RETIRED_CACHE requires complete retired cache",
        };
      }
    } else if (state.base_state == CoarseningBaseState::COMPLETE_EMPTY) {
      if (payload_store_.findRetiredCache(key) != nullptr ||
          payload_store_.isRetiredCacheIncomplete(key)) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_BASE_EMPTY_HAS_RETIRED_CACHE,
            .error_message = "COMPLETE_EMPTY internal node must not have retired cache or incomplete bit",
        };
      }
    } else if (state.base_state == CoarseningBaseState::INCOMPLETE) {
      const auto* ret = payload_store_.findRetiredCache(key);
      if (ret != nullptr && !payload_store_.isRetiredCacheIncomplete(key)) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_BASE_RETIRED_CACHE_MISSING,
            .error_message = "INCOMPLETE node with retired cache must have retired_cache_incomplete flag set",
        };
      }
    }

    if (!state.has_current_shadow) {
      if (state.current_shadow.hasObservations()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_SHADOW_OBSERVATIONS_INVALID,
            .error_message = "Non-materialized shadow (!has_current_shadow) contains observations",
        };
      }
    } else {
      if (!state.current_shadow.hasObservations()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_SHADOW_OBSERVATIONS_INVALID,
            .error_message = "Materialized shadow has 0 observations",
        };
      }
      if (state.current_shadow.firstSeenFrame() > state.current_shadow.lastSeenFrame()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_SHADOW_FRAME_ORDER_INVALID,
            .error_message = "Materialized shadow firstSeenFrame > lastSeenFrame",
        };
      }
      if (state.current_shadow.observedFrameCount() < 1) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_SHADOW_FRAME_COUNT_INVALID,
            .error_message = "Materialized shadow observedFrameCount < 1",
        };
      }
      if (state.current_shadow.elevation().count() < state.current_shadow.observedFrameCount()) {
        return PersistentMapValidationResult{
            .is_valid = false,
            .code = PersistentMapValidationCode::COARSENING_SHADOW_POINT_COUNT_INVALID,
            .error_message = "Materialized shadow elevation.count < observedFrameCount",
        };
      }
      if (state.base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE) {
        const auto* ret = payload_store_.findRetiredCache(key);
        if (ret != nullptr) {
          if (state.current_shadow.firstSeenFrame() != ret->firstSeenFrame() ||
              state.current_shadow.lastSeenFrame() < ret->lastSeenFrame() ||
              state.current_shadow.observedFrameCount() < ret->observedFrameCount() ||
              state.current_shadow.elevation().count() < ret->elevation().count()) {
            return PersistentMapValidationResult{
                .is_valid = false,
                .code = PersistentMapValidationCode::COARSENING_SHADOW_TEMPORALLY_PRECEDES_BASE,
                .error_message = "Materialized shadow temporally precedes or contradicts retired base",
            };
          }
        }
      }
    }
  }

  if (internal_coarsening_states_.size() != hierarchy_.internalCount()) {
    return PersistentMapValidationResult{
        .is_valid = false,
        .code = PersistentMapValidationCode::INTERNAL_NODE_MISSING_COARSENING_STATE,
        .error_message = "internal_coarsening_states_.size() != hierarchy_.internalCount()",
    };
  }

  return PersistentMapValidationResult{
      .is_valid = true,
      .code = PersistentMapValidationCode::VALID,
      .error_message = "",
  };
}

std::vector<std::pair<avr::spatial::CellKey, InternalCoarseningState>>
PersistentAdaptiveMap::internalCoarseningSnapshot() const {
  std::vector<std::pair<avr::spatial::CellKey, InternalCoarseningState>> snapshot;
  snapshot.reserve(internal_coarsening_states_.size());
  for (const auto& [k, v] : internal_coarsening_states_) {
    snapshot.emplace_back(k, v);
  }
  std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });
  return snapshot;
}

InternalCoarseningStats PersistentAdaptiveMap::coarseningStats() const noexcept {
  InternalCoarseningStats stats{};
  stats.internal_coarsening_state_count = internal_coarsening_states_.size();
  stats.map_bucket_count = internal_coarsening_states_.bucket_count();
  for (const auto& [k, s] : internal_coarsening_states_) {
    switch (s.base_state) {
      case CoarseningBaseState::COMPLETE_EMPTY:
        ++stats.complete_empty_internal_count;
        break;
      case CoarseningBaseState::COMPLETE_RETIRED_CACHE:
        ++stats.complete_retired_internal_count;
        break;
      case CoarseningBaseState::INCOMPLETE:
        ++stats.incomplete_internal_count;
        break;
    }
    if (s.has_current_shadow) {
      ++stats.materialized_shadow_count;
      stats.shadow_observation_count += s.current_shadow.observationCount();
    }
  }
  return stats;
}

PersistentMapStats PersistentAdaptiveMap::statistics() const noexcept {
  return PersistentMapStats{
      .hierarchy_stats = hierarchy_.statistics(),
      .payload_stats = payload_store_.statistics(),
      .coarsening_stats = coarseningStats(),
      .last_fused_frame_seq = last_fused_frame_seq_,
      .has_fused_frames = has_fused_frames_,
      .total_frames_fused = total_frames_fused_,
      .total_observations_fused = total_observations_fused_,
  };
}

void PersistentAdaptiveMap::reset() noexcept {
  hierarchy_.reset();
  payload_store_.reset();
  internal_coarsening_states_.clear();
  last_fused_frame_seq_ = 0;
  has_fused_frames_ = false;
  total_frames_fused_ = 0;
  total_observations_fused_ = 0;
}

}  // namespace avr::mapping
