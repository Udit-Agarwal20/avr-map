#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
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
   */
  void clear() noexcept;

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
  explicit ResolutionHysteresisController(const ResolutionHysteresisConfig& config) noexcept
      : config_(config) {}

  ResolutionHysteresisConfig config_{};
  std::unordered_map<avr::spatial::CellKey, RegionHysteresisState> regions_{};
};

static_assert(!std::is_copy_constructible_v<ResolutionHysteresisController>);
static_assert(!std::is_copy_assignable_v<ResolutionHysteresisController>);
static_assert(std::is_move_constructible_v<ResolutionHysteresisController>);
static_assert(std::is_move_assignable_v<ResolutionHysteresisController>);

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
