#include <algorithm>
#include <limits>
#include <optional>
#include <unordered_set>
#include <vector>

#include "avr_mapping/persistent_adaptive_map.hpp"

namespace avr::mapping {

SubtreeCollapseResult PersistentAdaptiveMap::collapseSubtreeFromCompleteShadow(
    const avr::spatial::CellKey& ancestor,
    ResolutionReasonBits reasons) {
  SubtreeCollapseResult result{};
  result.ancestor = ancestor;

  // 1. Validate ancestor existence and topology state
  if (!avr::spatial::isValidLevel(ancestor.level) || ancestor.level <= avr::spatial::L0) {
    result.status = SubtreeCollapseStatus::INVALID_ANCESTOR;
    return result;
  }

  const auto ancestor_state = hierarchy_.nodeState(ancestor);
  if (ancestor_state == NodeState::ABSENT) {
    result.status = SubtreeCollapseStatus::INVALID_ANCESTOR;
    return result;
  }

  if (ancestor_state != NodeState::INTERNAL) {
    result.status = SubtreeCollapseStatus::ANCESTOR_NOT_INTERNAL;
    return result;
  }

  // INTERNAL ancestor must NOT have active payload
  if (payload_store_.findActivePayload(ancestor) != nullptr) {
    result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
    return result;
  }

  // 2. Validate ancestor coarsening state
  const auto coarsen_it = internal_coarsening_states_.find(ancestor);
  if (coarsen_it == internal_coarsening_states_.end()) {
    result.status = SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE;
    return result;
  }

  const auto& coarsen_state = coarsen_it->second;
  if (coarsen_state.base_state == CoarseningBaseState::INCOMPLETE) {
    result.status = SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE;
    return result;
  }

  // 3. Derive canonical current-parent candidate
  const auto* parent_retired = payload_store_.findRetiredCache(ancestor);
  std::optional<AdaptiveCellPayload> candidate_payload;

  if (coarsen_state.has_current_shadow) {
    candidate_payload = coarsen_state.current_shadow;
  } else if (coarsen_state.base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE) {
    if (parent_retired == nullptr || !parent_retired->hasObservations()) {
      result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
      return result;
    }
    candidate_payload = *parent_retired;
  } else if (coarsen_state.base_state == CoarseningBaseState::COMPLETE_EMPTY) {
    if (parent_retired != nullptr) {
      result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
      return result;
    }
    candidate_payload = std::nullopt;
  } else {
    result.status = SubtreeCollapseStatus::ANCESTOR_BASE_INCOMPLETE;
    return result;
  }

  if (candidate_payload.has_value()) {
    candidate_payload->setLastUpdateReasons(reasons);
  }

  // Record structural pre-conditions
  result.ancestor_level = ancestor.level;
  result.had_materialized_shadow = coarsen_state.has_current_shadow;
  result.had_retired_cache = (parent_retired != nullptr);
  result.candidate_parent_observations =
      (candidate_payload.has_value() && candidate_payload->hasObservations())
          ? candidate_payload->observationCount()
          : 0ULL;

  // 4. Enumerate entire descendant subtree and validate structural integrity
  std::vector<avr::spatial::CellKey> internal_descendants;
  std::vector<avr::spatial::CellKey> leaf_descendants;
  std::vector<avr::spatial::CellKey> all_descendants;
  std::unordered_set<avr::spatial::CellKey> visited;

  std::vector<avr::spatial::CellKey> to_expand = {ancestor};
  while (!to_expand.empty()) {
    const auto current = to_expand.back();
    to_expand.pop_back();

    const auto children_opt = avr::spatial::childrenOf(current);
    if (!children_opt.has_value()) {
      result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
      return result;
    }

    for (const auto& child : *children_opt) {
      if (!avr::spatial::isValidLevel(child.level) || !hierarchy_.isAllocated(child)) {
        result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
        return result;
      }
      if (child.level != current.level - 1) {
        result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
        return result;
      }
      if (!visited.insert(child).second) {
        result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
        return result;
      }

      all_descendants.push_back(child);

      if (hierarchy_.isInternal(child)) {
        if (internal_coarsening_states_.find(child) == internal_coarsening_states_.end()) {
          result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
          return result;
        }
        internal_descendants.push_back(child);
        to_expand.push_back(child);
      } else if (hierarchy_.isLeaf(child)) {
        if (internal_coarsening_states_.find(child) != internal_coarsening_states_.end()) {
          result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
          return result;
        }
        leaf_descendants.push_back(child);
      } else {
        result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
        return result;
      }
    }
  }

  if (all_descendants.empty()) {
    result.status = SubtreeCollapseStatus::INVALID_SUBTREE_STATE;
    return result;
  }

  // Determine finest removed level
  uint8_t finest_level = ancestor.level;
  for (const auto& d : all_descendants) {
    if (d.level < finest_level) {
      finest_level = d.level;
    }
  }
  result.finest_removed_level = finest_level;
  result.levels_collapsed = ancestor.level - finest_level;

  // 5. Validate internal descendants: must not have active payloads
  for (const auto& internal_node : internal_descendants) {
    if (payload_store_.findActivePayload(internal_node) != nullptr) {
      result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
      return result;
    }
  }

  // 6. Validate active descendant leaf measurements and coverage
  uint64_t total_active_descendant_observations = 0;
  size_t active_descendant_payloads_count = 0;
  size_t replay_required_count = 0;

  for (const auto& leaf : leaf_descendants) {
    if (payload_store_.findRetiredCache(leaf) != nullptr) {
      result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
      return result;
    }
    if (payload_store_.requiresReplay(leaf)) {
      ++replay_required_count;
    }

    const auto* p = payload_store_.findActivePayload(leaf);
    if (p != nullptr && p->hasObservations()) {
      ++active_descendant_payloads_count;
      const uint64_t obs = p->observationCount();

      // Checked arithmetic for observation count
      if (std::numeric_limits<uint64_t>::max() - total_active_descendant_observations < obs) {
        result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
        return result;
      }
      total_active_descendant_observations += obs;

      // Leaf payload internal validity
      if (p->elevation().count() < p->observedFrameCount()) {
        result.status = SubtreeCollapseStatus::PAYLOAD_STATE_INVALID;
        return result;
      }

      // If canonical ancestor candidate has no elevation payload, no descendant can have observations
      if (!candidate_payload.has_value() || !candidate_payload->hasObservations()) {
        result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
        return result;
      }

      // Range consistency check (Section 17):
      // descendant min >= parent min, descendant max <= parent max
      if (p->elevation().min() < candidate_payload->elevation().min() ||
          p->elevation().max() > candidate_payload->elevation().max()) {
        result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
        return result;
      }

      // Temporal consistency check (Section 18 & 19):
      // Necessary consistency checks: descendant first_seen >= parent first_seen,
      // descendant last_seen <= parent last_seen, descendant observed_frame_count <= parent observed_frame_count.
      // (Actual completeness guarantee derives from M9B2A online shadow fusion provenance).
      if (p->firstSeenFrame() < candidate_payload->firstSeenFrame() ||
          p->lastSeenFrame() > candidate_payload->lastSeenFrame() ||
          p->observedFrameCount() > candidate_payload->observedFrameCount()) {
        result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
        return result;
      }
    }
  }

  // Count subset check (Section 15 & 16):
  if (candidate_payload.has_value() && candidate_payload->hasObservations()) {
    if (total_active_descendant_observations > candidate_payload->observationCount()) {
      result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
      return result;
    }
  } else {
    if (total_active_descendant_observations > 0) {
      result.status = SubtreeCollapseStatus::DESCENDANT_COVERAGE_INCONSISTENT;
      return result;
    }
  }

  // 7. Prepare bottom-up deterministic hierarchy merge plan (Section 28 & 29)
  // Deepest levels first (L1 before L2, L2 before L3), then canonical CellKey order.
  std::vector<avr::spatial::CellKey> internal_nodes_to_merge = internal_descendants;
  internal_nodes_to_merge.push_back(ancestor);

  std::sort(internal_nodes_to_merge.begin(), internal_nodes_to_merge.end(),
            [](const avr::spatial::CellKey& a, const avr::spatial::CellKey& b) {
              if (a.level != b.level) {
                return a.level < b.level;
              }
              return a < b;
            });

  // 8. Execute atomic commit (Section 31 & 32)
  for (const auto& node : internal_nodes_to_merge) {
    const bool merged = hierarchy_.mergeChildren(node);
    if (!merged) {
      result.status = SubtreeCollapseStatus::INTERNAL_ERROR;
      return result;
    }
  }

  // Sort all removed descendants in canonical CellKey order for deterministic cleanup
  // and reporting to downstream coordinators (e.g. M9B1 hysteresis).
  std::sort(all_descendants.begin(), all_descendants.end());

  // Clear all payload metadata and internal coarsening state for removed descendants
  for (const auto& desc : all_descendants) {
    payload_store_.clearCell(desc);
    internal_coarsening_states_.erase(desc);
  }

  // Update ancestor node: clear retired cache and coarsening state, install active payload
  payload_store_.clearCell(ancestor);
  internal_coarsening_states_.erase(ancestor);
  payload_store_.setReplayRequired(ancestor, false);

  if (candidate_payload.has_value() && candidate_payload->hasObservations()) {
    payload_store_.setActivePayload(ancestor, *candidate_payload);
  }

  payload_store_.pruneEmptyTiles();

  // Populate successful result
  result.status = SubtreeCollapseStatus::APPLIED_GAP_CLOSED;
  result.descendant_nodes_removed = all_descendants.size();
  result.descendant_internal_nodes_removed = internal_descendants.size();
  result.descendant_leaves_removed = leaf_descendants.size();
  result.replay_required_leaves_closed = replay_required_count;
  result.active_descendant_payloads_removed = active_descendant_payloads_count;
  result.active_descendant_observations_removed = total_active_descendant_observations;
  result.removed_descendant_keys = std::move(all_descendants);

  return result;
}

}  // namespace avr::mapping
