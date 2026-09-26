#include "avr_mapping/resolution_hysteresis.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
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

HysteresisResult ResolutionHysteresisController::observe(const HysteresisObservation& obs) {
  HysteresisResult result;
  result.region = obs.region;
  result.current_level = obs.region.level;
  result.requested_level = obs.decision.requested_level;
  result.required_coarsen_count = config_.coarsen_stable_observations;
  result.trigger_reasons = obs.decision.reasons;
  result.action = HysteresisAction::INVALID_OBSERVATION;
  result.target_level = 0;  // Documented non-actionable sentinel for invalid observations

  // 1. Strict spatial resolution validation on region, requested_level, and baseline_level
  if (!avr::spatial::isValidLevel(obs.region.level) ||
      !avr::spatial::isValidLevel(obs.decision.requested_level) ||
      !avr::spatial::isValidLevel(obs.decision.baseline_level)) {
    return result;
  }

  // 2. Strict frame sequence validation per region
  // First accepted observation for any region can have any uint64 frame_seq, including 0.
  // Subsequent accepted observations must have frame_seq strictly greater than last_frame_seq.
  auto it = regions_.find(obs.region);
  if (it != regions_.end() && it->second.has_last_frame) {
    if (obs.frame_seq <= it->second.last_frame_seq) {
      // Duplicate or out-of-order frame sequences must be rejected with zero mutation
      return result;
    }
  }

  // 3. Validated observation: locate or insert region state
  auto& state = regions_[obs.region];
  state.has_last_frame = true;
  state.last_frame_seq = obs.frame_seq;
  state.latest_reasons = obs.decision.reasons;

  const uint8_t current = obs.region.level;
  const uint8_t requested = obs.decision.requested_level;

  // 4. Fast Refinement: requested finer than current level
  if (requested < current) {
    state.stable_coarsen_count = 0;
    state.coarsen_action_pending = false;
    state.pending_target_level = 0;

    result.action = HysteresisAction::REFINE_NOW;
    result.target_level = requested;
    result.stable_coarsen_count = 0;
    result.already_pending = false;
    return result;
  }

  // 5. Equal Level: requested equal to current level
  if (requested == current) {
    state.stable_coarsen_count = 0;
    state.coarsen_action_pending = false;
    state.pending_target_level = 0;

    result.action = HysteresisAction::HOLD_CURRENT;
    result.target_level = current;
    result.stable_coarsen_count = 0;
    result.already_pending = false;
    return result;
  }

  // 6. Coarser Request: requested > current level
  // Coarsening proceeds strictly one level at a time
  const uint8_t one_level_target = static_cast<uint8_t>(current + 1);
  result.target_level = one_level_target;

  if (state.coarsen_action_pending) {
    // Already reached threshold and pending caller acknowledgement
    state.stable_coarsen_count = config_.coarsen_stable_observations;

    result.action = HysteresisAction::COARSEN_READY_PENDING_ACK;
    result.stable_coarsen_count = state.stable_coarsen_count;
    result.already_pending = true;
    return result;
  }

  // Increment stable coarsen streak
  ++state.stable_coarsen_count;

  if (state.stable_coarsen_count >= config_.coarsen_stable_observations) {
    state.stable_coarsen_count = config_.coarsen_stable_observations;
    state.coarsen_action_pending = true;
    state.pending_target_level = one_level_target;

    result.action = HysteresisAction::COARSEN_READY;
    result.stable_coarsen_count = state.stable_coarsen_count;
    result.already_pending = false;
    return result;
  }

  result.action = HysteresisAction::COARSEN_PENDING;
  result.stable_coarsen_count = state.stable_coarsen_count;
  result.already_pending = false;
  return result;
}

bool ResolutionHysteresisController::acknowledgeCoarsenResult(
    const avr::spatial::CellKey& region,
    CoarsenAttemptOutcome outcome) {
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
      return true;

    case CoarsenAttemptOutcome::BLOCKED_HOLD_FINE:
    case CoarsenAttemptOutcome::CANCELLED:
      // Coarsening was blocked or cancelled: clear pending flag and reset streak to 0,
      // but PRESERVE accepted-frame ordering history (has_last_frame and last_frame_seq)
      it->second.coarsen_action_pending = false;
      it->second.stable_coarsen_count = 0;
      it->second.pending_target_level = 0;
      return true;
  }

  return false;
}

bool ResolutionHysteresisController::eraseRegion(const avr::spatial::CellKey& region) {
  return regions_.erase(region) > 0;
}

void ResolutionHysteresisController::clear() noexcept {
  regions_.clear();
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
