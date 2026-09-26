#include <array>
#include <vector>

#include "avr_mapping/frame_evidence.hpp"
#include "avr_mapping/persistent_adaptive_map.hpp"
#include "avr_mapping/recent_observation_cache.hpp"
#include "avr_mapping/replay_reconstruction_result.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"

namespace avr::mapping {

ReplayReconstructionResult PersistentAdaptiveMap::replayDirectChildren(
    const avr::spatial::CellKey& parent,
    const RecentObservationCache& cache) {
  ReplayReconstructionResult result{};
  result.parent_key = parent;
  result.status = ReplayStatus::INVALID_PARENT;

  // 1. Validate parent coordinates and hierarchy level (parent must have level > L0)
  if (!avr::spatial::isValidLevel(parent.level) || parent.level == avr::spatial::L0) {
    result.status = ReplayStatus::INVALID_PARENT;
    return result;
  }

  // 2. Parent must exist in the hierarchy and be an INTERNAL node
  const auto parent_state = hierarchy_.nodeState(parent);
  if (parent_state == NodeState::ABSENT) {
    result.status = ReplayStatus::INVALID_PARENT;
    return result;
  }
  if (parent_state != NodeState::INTERNAL) {
    result.status = ReplayStatus::UNSUPPORTED_TOPOLOGY;
    return result;
  }

  // 3. Parent must possess a RETIRED_PARENT_CACHE
  const auto* parent_retired = payload_store_.findRetiredCache(parent);
  if (parent_retired == nullptr) {
    result.status = ReplayStatus::NO_RETIRED_PARENT_CACHE;
    return result;
  }

  // 4. Parent retired cache must be COMPLETE (cannot clear gaps from incomplete anchor)
  if (payload_store_.isRetiredCacheIncomplete(parent)) {
    result.status = ReplayStatus::PARENT_CACHE_INCOMPLETE;
    return result;
  }

  result.parent_expected_observations = parent_retired->observationCount();

  // 5. Parent must have exactly four direct children, and ALL four must currently be LEAF nodes
  const auto children_opt = avr::spatial::childrenOf(parent);
  if (!children_opt.has_value()) {
    result.status = ReplayStatus::INVALID_PARENT;
    return result;
  }

  const auto& children = *children_opt;
  size_t replay_required_child_count = 0;
  for (const auto& child : children) {
    const auto child_state = hierarchy_.nodeState(child);
    if (child_state != NodeState::LEAF) {
      // If any child is INTERNAL (split further) or ABSENT, direct replay is unsupported
      result.status = ReplayStatus::UNSUPPORTED_TOPOLOGY;
      return result;
    }
    if (payload_store_.requiresReplay(child)) {
      ++replay_required_child_count;
    }
  }

  // 5b. Replay entry state contract (Requirements 4 & 5)
  // If all four direct children are ALREADY not replay-required: explicit no-op
  if (replay_required_child_count == 0) {
    result.status = ReplayStatus::ALREADY_COMPLETE;
    return result;
  }

  // If mixed replay-required states exist across direct children: inconsistent state
  if (replay_required_child_count != 4) {
    result.status = ReplayStatus::INCONSISTENT_REPLAY_STATE;
    return result;
  }

  // 6. Cache window fast early screening
  const uint64_t parent_first_seen = parent_retired->firstSeenFrame();
  const uint64_t parent_last_seen = parent_retired->lastSeenFrame();

  if (cache.frameCount() == 0 ||
      !cache.oldestFrameSeq().has_value() ||
      !cache.newestFrameSeq().has_value()) {
    result.status = ReplayStatus::CACHE_HISTORY_INSUFFICIENT;
    return result;
  }

  if (*cache.oldestFrameSeq() > parent_first_seen ||
      *cache.newestFrameSeq() < parent_last_seen) {
    result.status = ReplayStatus::CACHE_HISTORY_INSUFFICIENT;
    return result;
  }

  // 7. Parent reconstruction from cached raw observations
  const auto frame_seqs = cache.frameSequenceSnapshot();
  AdaptiveCellPayload reconstructed_parent{};
  FrameEvidence scratch_evidence;

  for (const uint64_t frame_seq : frame_seqs) {
    if (frame_seq < parent_first_seen || frame_seq > parent_last_seen) {
      continue;
    }
    ++result.cached_frames_considered;

    std::vector<RecentObservationCache::PointIndex> parent_indices;
    if (!cache.collectObservationIndices(frame_seq, parent, parent_indices)) {
      result.status = ReplayStatus::INTERNAL_ERROR;
      return result;
    }
    if (parent_indices.empty()) {
      continue;
    }

    const auto* cached_frame = cache.findFrame(frame_seq);
    if (!cached_frame) {
      result.status = ReplayStatus::CACHE_HISTORY_INSUFFICIENT;
      return result;
    }

    std::vector<PointXYZ> scratch_points;
    scratch_points.reserve(parent_indices.size());
    for (const auto idx : parent_indices) {
      scratch_points.push_back(cached_frame->rawPoints()[idx]);
    }

    if (!scratch_evidence.buildFrame(scratch_points, frame_seq, cached_frame->sensorOrigin()).has_value()) {
      result.status = ReplayStatus::INTERNAL_ERROR;
      return result;
    }

    auto frame_elev_opt = scratch_evidence.aggregateElevationStats(parent);
    if (frame_elev_opt.has_value() && frame_elev_opt->count() > 0) {
      if (!reconstructed_parent.tryFuse(*frame_elev_opt, frame_seq, ResolutionReasonBits{})) {
        result.status = ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH;
        return result;
      }
      ++result.historical_frames_with_parent_observations;
    }
  }

  result.parent_replayed_observations = reconstructed_parent.observationCount();

  // 8. Verify reconstructed parent against immutable retired parent state across all 8 invariants
  if (reconstructed_parent.observationCount() != parent_retired->observationCount() ||
      reconstructed_parent.firstSeenFrame() != parent_retired->firstSeenFrame() ||
      reconstructed_parent.lastSeenFrame() != parent_retired->lastSeenFrame() ||
      reconstructed_parent.observedFrameCount() != parent_retired->observedFrameCount() ||
      reconstructed_parent.elevation().count() != parent_retired->elevation().count() ||
      reconstructed_parent.elevation().mean() != parent_retired->elevation().mean() ||
      reconstructed_parent.elevation().m2() != parent_retired->elevation().m2() ||
      reconstructed_parent.elevation().min() != parent_retired->elevation().min() ||
      reconstructed_parent.elevation().max() != parent_retired->elevation().max()) {
    result.status = ReplayStatus::PARENT_RECONSTRUCTION_MISMATCH;
    return result;
  }

  // 9. Reconstruct historical payload for each of the four direct children
  std::array<AdaptiveCellPayload, 4> reconstructed_children{};
  std::array<bool, 4> child_has_historical{false, false, false, false};
  uint64_t total_child_historical_observations = 0;

  for (size_t c = 0; c < 4; ++c) {
    const auto& child_key = children[c];
    FrameEvidence child_evidence;

    for (const uint64_t frame_seq : frame_seqs) {
      if (frame_seq < parent_first_seen || frame_seq > parent_last_seen) {
        continue;
      }

      std::vector<RecentObservationCache::PointIndex> child_indices;
      if (!cache.collectObservationIndices(frame_seq, child_key, child_indices)) {
        result.status = ReplayStatus::INTERNAL_ERROR;
        return result;
      }
      if (child_indices.empty()) {
        continue;
      }

      const auto* cached_frame = cache.findFrame(frame_seq);
      if (!cached_frame) {
        result.status = ReplayStatus::CACHE_HISTORY_INSUFFICIENT;
        return result;
      }

      std::vector<PointXYZ> child_scratch_points;
      child_scratch_points.reserve(child_indices.size());
      for (const auto idx : child_indices) {
        child_scratch_points.push_back(cached_frame->rawPoints()[idx]);
      }

      if (!child_evidence.buildFrame(child_scratch_points, frame_seq, cached_frame->sensorOrigin()).has_value()) {
        result.status = ReplayStatus::INTERNAL_ERROR;
        return result;
      }

      auto child_elev_opt = child_evidence.aggregateElevationStats(child_key);
      if (child_elev_opt.has_value() && child_elev_opt->count() > 0) {
        if (!reconstructed_children[c].tryFuse(*child_elev_opt, frame_seq, ResolutionReasonBits{})) {
          result.status = ReplayStatus::INTERNAL_ERROR;
          return result;
        }
        child_has_historical[c] = true;
      }
    }

    if (child_has_historical[c]) {
      ++result.children_with_historical_payload;
      total_child_historical_observations += reconstructed_children[c].observationCount();
    }
  }

  // 10. Observation conservation: sum of historical child observations == parent retired count
  if (total_child_historical_observations != parent_retired->observationCount()) {
    result.status = ReplayStatus::CONSERVATION_FAILURE;
    return result;
  }
  result.historical_observations_distributed = total_child_historical_observations;

  // 11. Prepare candidate child updates and validate post-split child fusion
  std::array<std::optional<AdaptiveCellPayload>, 4> final_child_payloads;

  for (size_t c = 0; c < 4; ++c) {
    const auto& child_key = children[c];
    const auto* existing_active = payload_store_.findActivePayload(child_key);

    if (existing_active != nullptr && existing_active->hasObservations()) {
      ++result.children_with_existing_current_payload;

      // Existing child active payload must strictly post-date parent retirement
      if (existing_active->firstSeenFrame() <= parent_last_seen) {
        result.status = ReplayStatus::TEMPORAL_OVERLAP;
        return result;
      }

      if (child_has_historical[c]) {
        AdaptiveCellPayload merged = *existing_active;
        if (!merged.tryPrependHistory(reconstructed_children[c])) {
          result.status = ReplayStatus::INTERNAL_ERROR;
          return result;
        }
        final_child_payloads[c] = merged;
      } else {
        // Child had no pre-split observations; preserve post-split active payload as-is
        final_child_payloads[c] = *existing_active;
      }
    } else {
      if (child_has_historical[c]) {
        // Pure historical reconstruction; reasons are 0/empty per Section 24
        final_child_payloads[c] = reconstructed_children[c];
      } else {
        // Empty historical child with no post-split observations
        final_child_payloads[c] = std::nullopt;
      }
    }
  }

  // 12. COMMIT PHASE (Atomic mutation only after all validation and preparation succeed)
  for (size_t c = 0; c < 4; ++c) {
    const auto& child_key = children[c];
    if (final_child_payloads[c].has_value()) {
      payload_store_.setActivePayload(child_key, *final_child_payloads[c]);
      payload_store_.setReplayRequired(child_key, false);
    } else {
      payload_store_.clearCell(child_key);
    }
  }

  result.children_replay_cleared = 4;
  result.status = ReplayStatus::APPLIED_COMPLETE;
  return result;
}

}  // namespace avr::mapping
