#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_mapping/resolution_hysteresis_state.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief An evidence-backed observation event submitted to ResolutionHysteresisController.
 */
struct HysteresisObservation {
  avr::spatial::CellKey region{};
  uint64_t frame_seq{0};
  ResolutionDecision decision{};
};

/**
 * @brief Value-owned staged observation candidate prepared against a specific controller revision.
 */
struct HysteresisPreparedObservation {
  avr::spatial::CellKey region{};
  uint64_t frame_seq{0};
  HysteresisResult result{};
  uint64_t expected_controller_revision{0};
  bool expected_prior_present{false};
  RegionHysteresisState expected_prior_state{};
  bool has_candidate_state{false};
  RegionHysteresisState candidate_state{};

  [[nodiscard]] constexpr bool isCommittable() const noexcept {
    return has_candidate_state && result.isSuccess();
  }

  constexpr bool operator==(const HysteresisPreparedObservation&) const noexcept = default;
};

/**
 * @brief Deterministic temporal resolution hysteresis controller.
 *
 * Implements fast multi-level refinement and delayed one-level coarsening.
 * Fully decoupled from persistent map mutation, hierarchy splits, and raw cache replay.
 * Single-writer component: no internal locking; concurrent mutation is not supported.
 */
class ResolutionHysteresisController {
 public:
  /**
   * @brief Factory constructor validating hysteresis configuration.
   *        Returns std::nullopt if configuration is invalid (e.g. threshold == 0).
   */
  [[nodiscard]] static std::optional<ResolutionHysteresisController> create(
      const ResolutionHysteresisConfig& config = {});

  // Non-copyable, movable
  ResolutionHysteresisController(const ResolutionHysteresisController&) = delete;
  ResolutionHysteresisController& operator=(const ResolutionHysteresisController&) = delete;
  ResolutionHysteresisController(ResolutionHysteresisController&&) noexcept = default;
  ResolutionHysteresisController& operator=(ResolutionHysteresisController&&) noexcept = default;
  ~ResolutionHysteresisController() = default;

  /**
   * @brief Access the active hysteresis configuration.
   */
  [[nodiscard]] const ResolutionHysteresisConfig& config() const noexcept {
    return config_;
  }

  /**
   * @brief Previews an observation event without mutating controller state.
   *
   * Zero persistent state mutation. Zero insertion into tracked regions.
   * Captured against current stateRevision().
   *
   * @param obs Observation event containing region CellKey, frame_seq, and decision.
   * @return Value-owned HysteresisPreparedObservation holding candidate state and result.
   */
  [[nodiscard]] HysteresisPreparedObservation previewObservation(
      const HysteresisObservation& obs) const;

  /**
   * @brief Convenience overload for previewObservation.
   */
  [[nodiscard]] HysteresisPreparedObservation previewObservation(
      const avr::spatial::CellKey& region,
      uint64_t frame_seq,
      const ResolutionDecision& decision) const {
    return previewObservation(HysteresisObservation{
        .region = region,
        .frame_seq = frame_seq,
        .decision = decision,
    });
  }

  /**
   * @brief Commits a previously prepared observation.
   *
   * Validates that controller revision and prior tracking state have not changed
   * since the observation was prepared. On success, applies the candidate state
   * and advances stateRevision() by 1.
   *
   * @param prepared Prepared observation to commit.
   * @return Commit status indicating outcome.
   */
  PreparedCommitStatus commitPreparedObservation(
      const HysteresisPreparedObservation& prepared);

  /**
   * @brief Atomically commits a batch of prepared observations for distinct regions.
   *
   * All entries must share the same expected controller revision matching current stateRevision().
   * Zero mutation occurs if ANY entry fails prevalidation (stale revision, prior mismatch,
   * duplicate region, non-committable).
   *
   * On success, applies all candidates in canonical CellKey order and advances stateRevision()
   * exactly once.
   *
   * An empty batch is a successful no-op that does not advance revision.
   *
   * @param prepared Span of prepared observations.
   * @return PreparedBatchCommitResult with status and committed count.
   */
  PreparedBatchCommitResult commitPreparedBatch(
      std::span<const HysteresisPreparedObservation> prepared);

  /**
   * @brief Ingests an evidence-backed resolution decision for a region.
   *
   * @param obs Observation event containing region CellKey, strictly increasing frame_seq,
   *            and instantaneous M4 ResolutionDecision.
   * @return HysteresisResult indicating the recommended action and current streak.
   */
  HysteresisResult observe(const HysteresisObservation& obs);

  /**
   * @brief Convenience overload for observe.
   */
  HysteresisResult observe(
      const avr::spatial::CellKey& region,
      uint64_t frame_seq,
      const ResolutionDecision& decision) {
    return observe(HysteresisObservation{
        .region = region,
        .frame_seq = frame_seq,
        .decision = decision,
    });
  }

  /**
   * @brief Acknowledges the outcome of an actually pending coarsening attempt.
   *        Returns false if region is not tracked or has no pending coarsening action.
   *
   * - APPLIED: Region state is erased; merged parent starts fresh.
   * - BLOCKED_HOLD_FINE: Clears pending state and resets coarsening streak to 0
   *                      while preserving frame ordering history (requires full window before retry).
   * - CANCELLED: Clears pending state and resets coarsening streak to 0
   *              while preserving frame ordering history.
   *
   * @param region Tracked cell key.
   * @param outcome Coarsening attempt outcome.
   * @return true if region was pending and acknowledged, false if region not found or not pending.
   */
  bool acknowledgeCoarsenResult(
      const avr::spatial::CellKey& region,
      CoarsenAttemptOutcome outcome);

  /**
   * @brief Erases tracking state for an explicit region (e.g., following topology mutation).
   * @return true if region was present and erased, false otherwise.
   */
  bool eraseRegion(const avr::spatial::CellKey& region);

  /**
   * @brief Clears all tracked region hysteresis state while preserving configuration.
   *        Always increments stateRevision() as a transaction barrier.
   */
  void clear() noexcept;

  /**
   * @brief Current controller mutation revision token.
   *
   * Increments on every successful state mutation (commit, legacy observe,
   * acknowledge, erase, clear). Used for stale prepared observation detection.
   */
  [[nodiscard]] uint64_t stateRevision() const noexcept {
    return state_revision_;
  }

  /**
   * @brief Checks if a region is currently tracked.
   */
  [[nodiscard]] bool containsRegion(const avr::spatial::CellKey& region) const noexcept;

  /**
   * @brief Number of active regions currently tracked.
   */
  [[nodiscard]] size_t trackedRegionCount() const noexcept;

  /**
   * @brief Fast read-only pointer lookup for a tracked region.
   *        Returns nullptr if region is not tracked.
   */
  [[nodiscard]] const RegionHysteresisState* findRegionState(
      const avr::spatial::CellKey& region) const noexcept;

  /**
   * @brief Snapshot lookup for a single region.
   *        Returns std::nullopt if region is not tracked.
   */
  [[nodiscard]] std::optional<RegionHysteresisSnapshot> regionState(
      const avr::spatial::CellKey& region) const;

  /**
   * @brief Inspects tracking state for a region.
   *        Returns std::nullopt if region is not tracked.
   */
  [[nodiscard]] std::optional<RegionHysteresisState> getState(
      const avr::spatial::CellKey& region) const noexcept;

  /**
   * @brief Returns a deterministic snapshot of all tracked regions sorted by CellKey.
   */
  [[nodiscard]] std::vector<RegionHysteresisSnapshot> stateSnapshot() const;

 private:
  struct TransitionEvaluation {
    HysteresisResult result{};
    std::optional<RegionHysteresisState> candidate{};
  };

  [[nodiscard]] static TransitionEvaluation evaluateTransition(
      const RegionHysteresisState* prior,
      const HysteresisObservation& obs,
      const ResolutionHysteresisConfig& config) noexcept;

  [[nodiscard]] static bool isValidCandidate(
      const HysteresisPreparedObservation& prepared,
      const ResolutionHysteresisConfig& config) noexcept;

  explicit ResolutionHysteresisController(const ResolutionHysteresisConfig& config) noexcept
      : config_(config) {}

  ResolutionHysteresisConfig config_{};
  std::unordered_map<avr::spatial::CellKey, RegionHysteresisState> regions_{};
  uint64_t state_revision_{0};

  friend struct ResolutionHysteresisTestAccess;
};

static_assert(!std::is_copy_constructible_v<ResolutionHysteresisController>);
static_assert(!std::is_copy_assignable_v<ResolutionHysteresisController>);
static_assert(std::is_move_constructible_v<ResolutionHysteresisController>);
static_assert(std::is_move_assignable_v<ResolutionHysteresisController>);

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
