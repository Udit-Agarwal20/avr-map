#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/adaptive_hierarchy.hpp"
#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/persistent_merge_result.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

PersistentMergeResult PersistentAdaptiveMap::mergeDirectChildren(
    const avr::spatial::CellKey& parent,
    const RecentObservationCache& cache,
    ResolutionReasonBits reasons) {
  return mergeDirectChildrenImpl(parent, &cache, reasons);
}

PersistentMergeResult PersistentAdaptiveMap::mergeDirectChildren(
    const avr::spatial::CellKey& parent,
    ResolutionReasonBits reasons) {
  return mergeDirectChildrenImpl(parent, nullptr, reasons);
}

PersistentMergeResult PersistentAdaptiveMap::mergeDirectChildrenImpl(
    const avr::spatial::CellKey& parent,
    const RecentObservationCache* cache,
    ResolutionReasonBits reasons) {
  PersistentMergeResult result;
  result.parent = parent;

  // =========================================================================
  // 1. Structural PREPARE: Parent hierarchy validation
  // =========================================================================
  if (!avr::spatial::isValidLevel(parent.level) || parent.level <= avr::spatial::L0) {
    result.status = MergeStatus::INVALID_PARENT;
    return result;
  }

  const auto parent_state = hierarchy_.nodeState(parent);
  if (parent_state == NodeState::ABSENT) {
    result.status = MergeStatus::INVALID_PARENT;
    return result;
  }
  if (parent_state == NodeState::LEAF) {
    result.status = MergeStatus::PARENT_NOT_INTERNAL;
    return result;
  }
  if (parent_state != NodeState::INTERNAL) {
    result.status = MergeStatus::PARENT_NOT_INTERNAL;
    return result;
  }

  // INTERNAL parent must NOT have active payload
  if (payload_store_.findActivePayload(parent) != nullptr) {
    result.status = MergeStatus::INCONSISTENT_PAYLOAD_STATE;
    return result;
  }

  const auto children_opt = avr::spatial::childrenOf(parent);
  if (!children_opt.has_value()) {
    result.status = MergeStatus::INVALID_PARENT;
    return result;
  }
  const auto& children = *children_opt;

  // =========================================================================
  // 2. Structural & Replay PREPARE: Direct children validation
  // =========================================================================
  for (size_t c = 0; c < 4; ++c) {
    const auto child_state = hierarchy_.nodeState(children[c]);
    if (child_state == NodeState::INTERNAL) {
      result.status = MergeStatus::CHILD_NOT_LEAF;
      return result;
    }
    if (child_state != NodeState::LEAF) {
      result.status = MergeStatus::CHILD_NOT_LEAF;
      return result;
    }

    // Children must have resolved replay
    if (payload_store_.requiresReplay(children[c])) {
      result.status = MergeStatus::REPLAY_INCOMPLETE;
      return result;
    }

    // Children are LEAF, must NOT have retired cache
    if (payload_store_.findRetiredCache(children[c]) != nullptr ||
        payload_store_.isRetiredCacheIncomplete(children[c])) {
      result.status = MergeStatus::INCONSISTENT_PAYLOAD_STATE;
      return result;
    }
  }

  // Record if parent currently has retired cache
  const auto* parent_retired = payload_store_.findRetiredCache(parent);
  result.had_parent_retired_cache = (parent_retired != nullptr);

  // Check internal coarsening state (Section 15, 33, 34)
  const auto it_coarsen = internal_coarsening_states_.find(parent);
  if (it_coarsen == internal_coarsening_states_.end()) {
    result.status = MergeStatus::INCONSISTENT_PAYLOAD_STATE;
    return result;
  }
  const auto& coarsen_state = it_coarsen->second;

  // Section 34: Incomplete Base Merge
  // An INCOMPLETE internal node must NEVER produce a supposedly complete coarsened LEAF.
  if (coarsen_state.base_state == CoarseningBaseState::INCOMPLETE) {
    result.status = MergeStatus::REPLAY_INCOMPLETE;
    return result;
  }

  // =========================================================================
  // 3. Child Active Payload Inspection
  // =========================================================================
  uint64_t min_first_seen = UINT64_MAX;
  uint64_t max_last_seen = 0;
  double min_child_z = std::numeric_limits<double>::infinity();
  double max_child_z = -std::numeric_limits<double>::infinity();
  uint64_t total_child_active_observations = 0;
  std::array<bool, 4> child_has_payload{false, false, false, false};

  for (size_t c = 0; c < 4; ++c) {
    const auto* p = payload_store_.findActivePayload(children[c]);
    if (p != nullptr && p->hasObservations()) {
      child_has_payload[c] = true;
      min_first_seen = std::min(min_first_seen, p->firstSeenFrame());
      max_last_seen = std::max(max_last_seen, p->lastSeenFrame());
      min_child_z = std::min(min_child_z, p->elevation().min());
      max_child_z = std::max(max_child_z, p->elevation().max());
      if (UINT64_MAX - total_child_active_observations < p->observationCount()) {
        result.status = MergeStatus::CONSERVATION_FAILURE;
        return result;
      }
      total_child_active_observations += p->observationCount();
    }
  }

  // =========================================================================
  // 4. Derive Canonical Current-Parent Candidate (Section 33)
  // =========================================================================
  AdaptiveCellPayload canonical_parent{};
  bool has_canonical_parent = false;

  if (coarsen_state.has_current_shadow) {
    canonical_parent = coarsen_state.current_shadow;
    has_canonical_parent = true;
  } else if (coarsen_state.base_state == CoarseningBaseState::COMPLETE_RETIRED_CACHE) {
    if (parent_retired == nullptr || !parent_retired->hasObservations()) {
      result.status = MergeStatus::INCONSISTENT_PAYLOAD_STATE;
      return result;
    }
    canonical_parent = *parent_retired;
    has_canonical_parent = true;
  } else if (coarsen_state.base_state == CoarseningBaseState::COMPLETE_EMPTY) {
    canonical_parent = AdaptiveCellPayload{};
    has_canonical_parent = false;
  }

  // Branch A: Payload-Free Child & Candidate Merge (Section 42)
  if (total_child_active_observations == 0 && !has_canonical_parent) {
    const bool merged = hierarchy_.mergeChildren(parent);
    if (!merged) {
      result.status = MergeStatus::INTERNAL_ERROR;
      return result;
    }

    for (size_t c = 0; c < 4; ++c) {
      payload_store_.clearCell(children[c]);
    }

    payload_store_.clearCell(parent);
    payload_store_.setReplayRequired(parent, false);
    internal_coarsening_states_.erase(parent);

    result.status = MergeStatus::APPLIED;
    result.children_reconstructed = 4;
    result.child_observations_verified = 0;
    result.parent_observations_reconstructed = 0;
    result.cached_frames_considered = 0;
    result.parent_payload_created = false;
    return result;
  }

  // If one side is empty and the other is non-empty: conservation failure
  if (!has_canonical_parent || total_child_active_observations == 0) {
    result.status = MergeStatus::CONSERVATION_FAILURE;
    return result;
  }

  // Current Child Conservation Checks (Section 56)
  if (total_child_active_observations != canonical_parent.observationCount()) {
    result.status = MergeStatus::CONSERVATION_FAILURE;
    return result;
  }
  if (min_first_seen != canonical_parent.firstSeenFrame() ||
      max_last_seen != canonical_parent.lastSeenFrame()) {
    result.status = MergeStatus::TEMPORAL_METADATA_INVALID;
    return result;
  }
  if (min_child_z != canonical_parent.elevation().min() ||
      max_child_z != canonical_parent.elevation().max()) {
    result.status = MergeStatus::CONSERVATION_FAILURE;
    return result;
  }

  // =========================================================================
  // 5. Strict Optional Cache Verification Mode (Section 36)
  // =========================================================================
  if (cache != nullptr) {
    if (min_first_seen > max_last_seen) {
      result.status = MergeStatus::TEMPORAL_METADATA_INVALID;
      return result;
    }

    if (cache->frameCount() == 0 ||
        !cache->oldestFrameSeq().has_value() ||
        !cache->newestFrameSeq().has_value()) {
      result.status = MergeStatus::CACHE_HISTORY_INSUFFICIENT;
      return result;
    }

    if (*cache->oldestFrameSeq() > min_first_seen ||
        *cache->newestFrameSeq() < max_last_seen) {
      result.status = MergeStatus::CACHE_HISTORY_INSUFFICIENT;
      return result;
    }

    const auto frame_seqs = cache->frameSequenceSnapshot();
    AdaptiveCellPayload candidate_parent{};
    std::array<AdaptiveCellPayload, 4> candidate_children{};
    FrameEvidence scratch_evidence;

    for (const uint64_t frame_seq : frame_seqs) {
      if (frame_seq < min_first_seen || frame_seq > max_last_seen) {
        continue;
      }
      ++result.cached_frames_considered;

      const auto* cached_frame = cache->findFrame(frame_seq);
      if (!cached_frame) {
        result.status = MergeStatus::CACHE_HISTORY_INSUFFICIENT;
        return result;
      }

      // Reconstruct 4 direct children for this frame
      for (size_t c = 0; c < 4; ++c) {
        std::vector<RecentObservationCache::PointIndex> c_indices;
        if (!cache->collectObservationIndices(frame_seq, children[c], c_indices)) {
          result.status = MergeStatus::INTERNAL_ERROR;
          return result;
        }
        if (c_indices.empty()) {
          continue;
        }

        std::vector<PointXYZ> c_points;
        c_points.reserve(c_indices.size());
        for (const auto idx : c_indices) {
          c_points.push_back(cached_frame->rawPoints()[idx]);
        }

        if (!scratch_evidence.buildFrame(c_points, frame_seq, cached_frame->sensorOrigin()).has_value()) {
          result.status = MergeStatus::INTERNAL_ERROR;
          return result;
        }

        auto c_elev_opt = scratch_evidence.aggregateElevationStats(children[c]);
        if (c_elev_opt.has_value() && c_elev_opt->count() > 0) {
          if (!candidate_children[c].tryFuse(*c_elev_opt, frame_seq, ResolutionReasonBits{})) {
            result.status = MergeStatus::PAYLOAD_MERGE_FAILED;
            return result;
          }
        }
      }

      // Reconstruct candidate parent for this frame
      std::vector<RecentObservationCache::PointIndex> p_indices;
      if (!cache->collectObservationIndices(frame_seq, parent, p_indices)) {
        result.status = MergeStatus::INTERNAL_ERROR;
        return result;
      }
      if (p_indices.empty()) {
        continue;
      }

      std::vector<PointXYZ> p_points;
      p_points.reserve(p_indices.size());
      for (const auto idx : p_indices) {
        p_points.push_back(cached_frame->rawPoints()[idx]);
      }

      if (!scratch_evidence.buildFrame(p_points, frame_seq, cached_frame->sensorOrigin()).has_value()) {
        result.status = MergeStatus::INTERNAL_ERROR;
        return result;
      }

      auto p_elev_opt = scratch_evidence.aggregateElevationStats(parent);
      if (p_elev_opt.has_value() && p_elev_opt->count() > 0) {
        if (!candidate_parent.tryFuse(*p_elev_opt, frame_seq, ResolutionReasonBits{})) {
          result.status = MergeStatus::PAYLOAD_MERGE_FAILED;
          return result;
        }
      }
    }

    // Verify Reconstructed Children Against Current Active Payloads
    uint64_t total_child_reconstructed_observations = 0;
    for (size_t c = 0; c < 4; ++c) {
      const auto* actual = payload_store_.findActivePayload(children[c]);
      if (!child_has_payload[c] || actual == nullptr || !actual->hasObservations()) {
        if (candidate_children[c].observationCount() > 0) {
          result.status = MergeStatus::CHILD_RECONSTRUCTION_MISMATCH;
          return result;
        }
      } else {
        if (candidate_children[c].observationCount() < actual->observationCount()) {
          result.status = MergeStatus::CACHE_HISTORY_INSUFFICIENT;
          return result;
        }
        if (candidate_children[c].observationCount() != actual->observationCount() ||
            candidate_children[c].firstSeenFrame() != actual->firstSeenFrame() ||
            candidate_children[c].lastSeenFrame() != actual->lastSeenFrame() ||
            candidate_children[c].observedFrameCount() != actual->observedFrameCount() ||
            candidate_children[c].elevation().count() != actual->elevation().count() ||
            candidate_children[c].elevation().mean() != actual->elevation().mean() ||
            candidate_children[c].elevation().m2() != actual->elevation().m2() ||
            candidate_children[c].elevation().min() != actual->elevation().min() ||
            candidate_children[c].elevation().max() != actual->elevation().max()) {
          result.status = MergeStatus::CHILD_RECONSTRUCTION_MISMATCH;
          return result;
        }
        total_child_reconstructed_observations += candidate_children[c].observationCount();
      }
    }
    result.child_observations_verified = total_child_reconstructed_observations;

    // Verify Reconstructed Parent Against Canonical Shadow Parent
    result.parent_observations_reconstructed = candidate_parent.observationCount();
    if (candidate_parent.observationCount() != canonical_parent.observationCount() ||
        candidate_parent.firstSeenFrame() != canonical_parent.firstSeenFrame() ||
        candidate_parent.lastSeenFrame() != canonical_parent.lastSeenFrame() ||
        candidate_parent.observedFrameCount() != canonical_parent.observedFrameCount() ||
        candidate_parent.elevation().count() != canonical_parent.elevation().count() ||
        candidate_parent.elevation().mean() != canonical_parent.elevation().mean() ||
        candidate_parent.elevation().m2() != canonical_parent.elevation().m2() ||
        candidate_parent.elevation().min() != canonical_parent.elevation().min() ||
        candidate_parent.elevation().max() != canonical_parent.elevation().max()) {
      result.status = MergeStatus::PARENT_RECONSTRUCTION_MISMATCH;
      return result;
    }
  }

  // Set explicit coarsening reasons on parent candidate (Section 9)
  canonical_parent.setLastUpdateReasons(reasons);

  // =========================================================================
  // 6. COMMIT: Atomic Hierarchy, Payload Store, and Coarsening State Mutation
  // =========================================================================
  const bool merged = hierarchy_.mergeChildren(parent);
  if (!merged) {
    result.status = MergeStatus::INTERNAL_ERROR;
    return result;
  }

  // Clear children from payload store
  for (size_t c = 0; c < 4; ++c) {
    payload_store_.clearCell(children[c]);
  }

  // Install merged active payload on parent (removes retired cache)
  payload_store_.setActivePayload(parent, canonical_parent);
  payload_store_.setReplayRequired(parent, false);

  // Erase parent InternalCoarseningState (Section 50)
  internal_coarsening_states_.erase(parent);

  result.status = MergeStatus::APPLIED;
  result.children_reconstructed = 4;
  result.child_observations_verified = total_child_active_observations;
  result.parent_observations_reconstructed = canonical_parent.observationCount();
  result.parent_payload_created = true;
  return result;
}

}  // namespace avr::mapping
