#pragma once

#include <cstdint>
#include <string_view>

#include "avr_mapping/resolution_policy.hpp"
#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Discrete temporal action emitted by ResolutionHysteresisController.
 */
enum class HysteresisAction : uint8_t {
  HOLD_CURRENT = 0,
  REFINE_NOW = 1,
  COARSEN_PENDING = 2,
  COARSEN_READY = 3,
  COARSEN_READY_PENDING_ACK = 4,
  INVALID_OBSERVATION = 5,
};

[[nodiscard]] constexpr std::string_view hysteresisActionToString(HysteresisAction action) noexcept {
  switch (action) {
    case HysteresisAction::HOLD_CURRENT:
      return "HOLD_CURRENT";
    case HysteresisAction::REFINE_NOW:
      return "REFINE_NOW";
    case HysteresisAction::COARSEN_PENDING:
      return "COARSEN_PENDING";
    case HysteresisAction::COARSEN_READY:
      return "COARSEN_READY";
    case HysteresisAction::COARSEN_READY_PENDING_ACK:
      return "COARSEN_READY_PENDING_ACK";
    case HysteresisAction::INVALID_OBSERVATION:
      return "INVALID_OBSERVATION";
  }
  return "UNKNOWN";
}

/**
 * @brief Outcome reported when acknowledging a coarsening attempt.
 */
enum class CoarsenAttemptOutcome : uint8_t {
  APPLIED = 0,
  BLOCKED_HOLD_FINE = 1,
  CANCELLED = 2,
};

[[nodiscard]] constexpr std::string_view coarsenAttemptOutcomeToString(CoarsenAttemptOutcome outcome) noexcept {
  switch (outcome) {
    case CoarsenAttemptOutcome::APPLIED:
      return "APPLIED";
    case CoarsenAttemptOutcome::BLOCKED_HOLD_FINE:
      return "BLOCKED_HOLD_FINE";
    case CoarsenAttemptOutcome::CANCELLED:
      return "CANCELLED";
  }
  return "UNKNOWN";
}

/**
 * @brief Detailed result of a temporal resolution hysteresis observation.
 */
struct HysteresisResult {
  avr::spatial::CellKey region{};
  uint8_t current_level{0};
  uint8_t requested_level{0};
  HysteresisAction action{HysteresisAction::INVALID_OBSERVATION};
  uint8_t target_level{0};
  uint32_t stable_coarsen_count{0};
  uint32_t required_coarsen_count{0};
  ResolutionReasonBits trigger_reasons{};
  bool already_pending{false};

  [[nodiscard]] constexpr bool isSuccess() const noexcept {
    return action != HysteresisAction::INVALID_OBSERVATION;
  }

  constexpr bool operator==(const HysteresisResult&) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
