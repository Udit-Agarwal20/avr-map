#include "avr_mapping/resolution_hysteresis.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "avr_spatial/constants.hpp"

namespace avr::mapping {

std::optional<ResolutionHysteresisController> ResolutionHysteresisController::create(
    const ResolutionHysteresisConfig& config) {
  if (!config.isValid()) {
    return std::nullopt;
  }
  return ResolutionHysteresisController(config);
}

ResolutionHysteresisController::TransitionEvaluation
ResolutionHysteresisController::evaluateTransition(
    const RegionHysteresisState* prior,
    const HysteresisObservation& obs,
    const ResolutionHysteresisConfig& config) noexcept {
  TransitionEvaluation eval;
  eval.result.region = obs.region;
  eval.result.current_level = obs.region.level;
  eval.result.requested_level = obs.decision.requested_level;
  eval.result.required_coarsen_count = config.coarsen_stable_observations;
  eval.result.trigger_reasons = obs.decision.reasons;
  eval.result.action = HysteresisAction::INVALID_OBSERVATION;
  eval.result.target_level = 0;  // Documented non-actionable sentinel for invalid observations

  // 1. Strict spatial resolution validation on region, requested_level, and baseline_level
  if (!avr::spatial::isValidLevel(obs.region.level) ||
      !avr::spatial::isValidLevel(obs.decision.requested_level) ||
      !avr::spatial::isValidLevel(obs.decision.baseline_level)) {
    return eval;
  }

  // 2. Strict frame sequence validation per region
  // First accepted observation for any region can have any uint64 frame_seq, including 0.
  // Subsequent accepted observations must have frame_seq strictly greater than last_frame_seq.
  if (prior != nullptr && prior->has_last_frame) {
    if (obs.frame_seq <= prior->last_frame_seq) {
      // Duplicate or out-of-order frame sequences must be rejected with zero mutation
      return eval;
    }
  }

  // 3. Validated observation: initialize candidate state
  RegionHysteresisState candidate = (prior != nullptr) ? *prior : RegionHysteresisState{};
  candidate.has_last_frame = true;
  candidate.last_frame_seq = obs.frame_seq;
  candidate.latest_reasons = obs.decision.reasons;

  const uint8_t current = obs.region.level;
  const uint8_t requested = obs.decision.requested_level;

  // 4. Fast Refinement: requested finer than current level
  if (requested < current) {
    candidate.stable_coarsen_count = 0;
    candidate.coarsen_action_pending = false;
    candidate.pending_target_level = 0;

    eval.result.action = HysteresisAction::REFINE_NOW;
    eval.result.target_level = requested;
    eval.result.stable_coarsen_count = 0;
    eval.result.already_pending = false;
    eval.candidate = candidate;
    return eval;
  }

  // 5. Equal Level: requested equal to current level
  if (requested == current) {
    candidate.stable_coarsen_count = 0;
    candidate.coarsen_action_pending = false;
    candidate.pending_target_level = 0;

    eval.result.action = HysteresisAction::HOLD_CURRENT;
    eval.result.target_level = current;
    eval.result.stable_coarsen_count = 0;
    eval.result.already_pending = false;
    eval.candidate = candidate;
    return eval;
  }

  // 6. Coarser Request: requested > current level
  // Coarsening proceeds strictly one level at a time
  const uint8_t one_level_target = static_cast<uint8_t>(current + 1);
  eval.result.target_level = one_level_target;

  if (candidate.coarsen_action_pending) {
    // Already reached threshold and pending caller acknowledgement
    candidate.stable_coarsen_count = config.coarsen_stable_observations;

    eval.result.action = HysteresisAction::COARSEN_READY_PENDING_ACK;
    eval.result.stable_coarsen_count = candidate.stable_coarsen_count;
    eval.result.already_pending = true;
    eval.candidate = candidate;
    return eval;
  }

  // Increment stable coarsen streak
  ++candidate.stable_coarsen_count;

  if (candidate.stable_coarsen_count >= config.coarsen_stable_observations) {
    candidate.stable_coarsen_count = config.coarsen_stable_observations;
    candidate.coarsen_action_pending = true;
    candidate.pending_target_level = one_level_target;

    eval.result.action = HysteresisAction::COARSEN_READY;
    eval.result.stable_coarsen_count = candidate.stable_coarsen_count;
    eval.result.already_pending = false;
    eval.candidate = candidate;
    return eval;
  }

  eval.result.action = HysteresisAction::COARSEN_PENDING;
  eval.result.stable_coarsen_count = candidate.stable_coarsen_count;
  eval.result.already_pending = false;
  eval.candidate = candidate;
  return eval;
}

bool ResolutionHysteresisController::isValidCandidate(
    const HysteresisPreparedObservation& prepared,
    const ResolutionHysteresisConfig& config) noexcept {
  if (!prepared.has_candidate_state || !prepared.result.isSuccess()) {
    return false;
  }
  if (!avr::spatial::isValidLevel(prepared.region.level)) {
    return false;
  }
  const auto& cand = prepared.candidate_state;
  if (!cand.has_last_frame) {
    return false;
  }
  if (cand.last_frame_seq != prepared.frame_seq) {
    return false;
  }
  if (cand.stable_coarsen_count > config.coarsen_stable_observations) {
    return false;
  }
  if (cand.coarsen_action_pending) {
    if (cand.stable_coarsen_count != config.coarsen_stable_observations) {
      return false;
    }
    if (cand.pending_target_level != static_cast<uint8_t>(prepared.region.level + 1)) {
      return false;
    }
    if (!avr::spatial::isValidLevel(cand.pending_target_level)) {
      return false;
    }
  } else {
    if (cand.pending_target_level != 0) {
      return false;
    }
  }
  return true;
}

HysteresisPreparedObservation ResolutionHysteresisController::previewObservation(
    const HysteresisObservation& obs) const {
  const RegionHysteresisState* prior = findRegionState(obs.region);
  const auto eval = evaluateTransition(prior, obs, config_);

  HysteresisPreparedObservation prepared{};
  prepared.region = obs.region;
  prepared.frame_seq = obs.frame_seq;
  prepared.result = eval.result;
  prepared.expected_controller_revision = state_revision_;
  prepared.expected_prior_present = (prior != nullptr);
  if (prior != nullptr) {
    prepared.expected_prior_state = *prior;
  }
  if (eval.candidate.has_value()) {
    prepared.has_candidate_state = true;
    prepared.candidate_state = *eval.candidate;
  } else {
    prepared.has_candidate_state = false;
  }
  return prepared;
}

PreparedCommitStatus ResolutionHysteresisController::commitPreparedObservation(
    const HysteresisPreparedObservation& prepared) {
  if (!prepared.isCommittable() || !isValidCandidate(prepared, config_)) {
    return PreparedCommitStatus::NOT_COMMITTABLE;
  }

  if (state_revision_ == UINT64_MAX) {
    return PreparedCommitStatus::REVISION_EXHAUSTED;
  }

  if (prepared.expected_controller_revision != state_revision_) {
    return PreparedCommitStatus::STALE_CONTROLLER_REVISION;
  }

  auto it = regions_.find(prepared.region);
  if (!prepared.expected_prior_present) {
    if (it != regions_.end()) {
      return PreparedCommitStatus::PRIOR_STATE_MISMATCH;
    }
  } else {
    if (it == regions_.end()) {
      return PreparedCommitStatus::PRIOR_STATE_MISMATCH;
    }
    if (!(it->second == prepared.expected_prior_state)) {
      return PreparedCommitStatus::PRIOR_STATE_MISMATCH;
    }
  }

  regions_[prepared.region] = prepared.candidate_state;
  ++state_revision_;
  return PreparedCommitStatus::APPLIED;
}

PreparedBatchCommitResult ResolutionHysteresisController::commitPreparedBatch(
    std::span<const HysteresisPreparedObservation> prepared) {
  if (prepared.empty()) {
    return PreparedBatchCommitResult{
        .status = PreparedBatchCommitStatus::APPLIED,
        .committed_count = 0,
    };
  }

  if (state_revision_ == UINT64_MAX) {
    return PreparedBatchCommitResult{
        .status = PreparedBatchCommitStatus::REVISION_EXHAUSTED,
        .committed_count = 0,
    };
  }

  for (const auto& entry : prepared) {
    if (!entry.isCommittable() || !isValidCandidate(entry, config_)) {
      return PreparedBatchCommitResult{
          .status = PreparedBatchCommitStatus::NOT_COMMITTABLE,
          .committed_count = 0,
      };
    }
  }

  for (const auto& entry : prepared) {
    if (entry.expected_controller_revision != state_revision_) {
      return PreparedBatchCommitResult{
          .status = PreparedBatchCommitStatus::STALE_CONTROLLER_REVISION,
          .committed_count = 0,
      };
    }
  }

  std::vector<const HysteresisPreparedObservation*> sorted_entries;
  sorted_entries.reserve(prepared.size());
  for (const auto& entry : prepared) {
    sorted_entries.push_back(&entry);
  }
  std::sort(sorted_entries.begin(), sorted_entries.end(),
            [](const HysteresisPreparedObservation* a, const HysteresisPreparedObservation* b) {
              return a->region < b->region;
            });

  for (size_t i = 1; i < sorted_entries.size(); ++i) {
    if (sorted_entries[i]->region == sorted_entries[i - 1]->region) {
      return PreparedBatchCommitResult{
          .status = PreparedBatchCommitStatus::DUPLICATE_REGION,
          .committed_count = 0,
      };
    }
  }

  for (const auto* entry : sorted_entries) {
    auto it = regions_.find(entry->region);
    if (!entry->expected_prior_present) {
      if (it != regions_.end()) {
        return PreparedBatchCommitResult{
            .status = PreparedBatchCommitStatus::PRIOR_STATE_MISMATCH,
            .committed_count = 0,
        };
      }
    } else {
      if (it == regions_.end()) {
        return PreparedBatchCommitResult{
            .status = PreparedBatchCommitStatus::PRIOR_STATE_MISMATCH,
            .committed_count = 0,
        };
      }
      if (!(it->second == entry->expected_prior_state)) {
        return PreparedBatchCommitResult{
            .status = PreparedBatchCommitStatus::PRIOR_STATE_MISMATCH,
            .committed_count = 0,
        };
      }
    }
  }

  for (const auto* entry : sorted_entries) {
    regions_[entry->region] = entry->candidate_state;
  }
  ++state_revision_;

  return PreparedBatchCommitResult{
      .status = PreparedBatchCommitStatus::APPLIED,
      .committed_count = prepared.size(),
  };
}

HysteresisResult ResolutionHysteresisController::observe(const HysteresisObservation& obs) {
  if (state_revision_ == UINT64_MAX) {
    HysteresisResult result;
    result.region = obs.region;
    result.current_level = obs.region.level;
    result.requested_level = obs.decision.requested_level;
    result.required_coarsen_count = config_.coarsen_stable_observations;
    result.trigger_reasons = obs.decision.reasons;
    result.action = HysteresisAction::INVALID_OBSERVATION;
    result.target_level = 0;
    return result;
  }

  const auto prepared = previewObservation(obs);
  if (!prepared.isCommittable()) {
    return prepared.result;
  }

  const auto status = commitPreparedObservation(prepared);
  if (status != PreparedCommitStatus::APPLIED) {
    HysteresisResult result = prepared.result;
    result.action = HysteresisAction::INVALID_OBSERVATION;
    return result;
  }

  return prepared.result;
}

bool ResolutionHysteresisController::acknowledgeCoarsenResult(
    const avr::spatial::CellKey& region,
    CoarsenAttemptOutcome outcome) {
  if (state_revision_ == UINT64_MAX) {
    return false;
  }

  auto it = regions_.find(region);
  if (it == regions_.end()) {
    return false;
  }

  // Acknowledgement is strictly valid ONLY for an actually pending coarsening action
  if (!it->second.coarsen_action_pending) {
    return false;
  }

  switch (outcome) {
    case CoarsenAttemptOutcome::APPLIED:
      // Merged parent replaces child leaves; child state is removed completely
      regions_.erase(it);
      ++state_revision_;
      return true;

    case CoarsenAttemptOutcome::BLOCKED_HOLD_FINE:
    case CoarsenAttemptOutcome::CANCELLED:
      // Coarsening was blocked or cancelled: clear pending flag and reset streak to 0,
      // but PRESERVE accepted-frame ordering history (has_last_frame and last_frame_seq)
      it->second.coarsen_action_pending = false;
      it->second.stable_coarsen_count = 0;
      it->second.pending_target_level = 0;
      ++state_revision_;
      return true;
  }

  return false;
}

bool ResolutionHysteresisController::eraseRegion(const avr::spatial::CellKey& region) {
  if (state_revision_ == UINT64_MAX) {
    return false;
  }
  auto it = regions_.find(region);
  if (it == regions_.end()) {
    return false;
  }
  regions_.erase(it);
  ++state_revision_;
  return true;
}

void ResolutionHysteresisController::clear() noexcept {
  if (state_revision_ == UINT64_MAX) {
    return;
  }
  regions_.clear();
  ++state_revision_;
}

bool ResolutionHysteresisController::containsRegion(const avr::spatial::CellKey& region) const noexcept {
  return regions_.find(region) != regions_.end();
}

size_t ResolutionHysteresisController::trackedRegionCount() const noexcept {
  return regions_.size();
}

const RegionHysteresisState* ResolutionHysteresisController::findRegionState(
    const avr::spatial::CellKey& region) const noexcept {
  auto it = regions_.find(region);
  if (it == regions_.end()) {
    return nullptr;
  }
  return &it->second;
}

std::optional<RegionHysteresisSnapshot> ResolutionHysteresisController::regionState(
    const avr::spatial::CellKey& region) const {
  auto it = regions_.find(region);
  if (it == regions_.end()) {
    return std::nullopt;
  }
  return RegionHysteresisSnapshot{
      .region = it->first,
      .last_frame_seq = it->second.last_frame_seq,
      .stable_coarsen_count = it->second.stable_coarsen_count,
      .latest_reasons = it->second.latest_reasons,
      .pending_target_level = it->second.pending_target_level,
      .has_last_frame = it->second.has_last_frame,
      .coarsen_action_pending = it->second.coarsen_action_pending,
  };
}

std::optional<RegionHysteresisState> ResolutionHysteresisController::getState(
    const avr::spatial::CellKey& region) const noexcept {
  auto it = regions_.find(region);
  if (it == regions_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<RegionHysteresisSnapshot> ResolutionHysteresisController::stateSnapshot() const {
  std::vector<RegionHysteresisSnapshot> snapshots;
  snapshots.reserve(regions_.size());
  for (const auto& [key, state] : regions_) {
    snapshots.push_back(RegionHysteresisSnapshot{
        .region = key,
        .last_frame_seq = state.last_frame_seq,
        .stable_coarsen_count = state.stable_coarsen_count,
        .latest_reasons = state.latest_reasons,
        .pending_target_level = state.pending_target_level,
        .has_last_frame = state.has_last_frame,
        .coarsen_action_pending = state.coarsen_action_pending,
    });
  }
  std::sort(snapshots.begin(), snapshots.end());
  return snapshots;
}

}  // namespace avr::mapping
