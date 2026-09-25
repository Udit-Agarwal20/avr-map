#pragma once

#include <cstdint>
#include <limits>

#include "avr_mapping/elevation_stats.hpp"
#include "avr_mapping/resolution_decision.hpp"

namespace avr::mapping {

/**
 * @brief Persistent measurement payload for an active or cached adaptive hierarchy cell.
 *
 * Implements M7 persistent leaf payload foundation:
 * - Stores continuous elevation statistics (count, mean, M2, min, max).
 * - Tracks temporal observation frame metadata (first_seen, last_seen, observed_frame_count).
 * - Records the decision reason context associated with the most recent payload update.
 *
 * Transaction & Fusion Rules:
 * - An AdaptiveCellPayload is valid if it represents at least one accepted observation
 *   (hasObservations() == true, i.e., elevation.count() > 0).
 * - First fusion sets first_seen_frame = frame_seq, last_seen_frame = frame_seq,
 *   observed_frame_count = 1.
 * - Subsequent fusions require frame_seq > last_seen_frame (rejects duplicate and out-of-order frames).
 * - observed_frame_count increment is overflow-guarded against UINT64_MAX.
 * - Merging uses transactional parallel Welford (ElevationStats::tryMerge).
 *   Any numeric or ordering failure leaves all payload fields completely unmodified.
 * - last_update_reasons records the resolution-decision context of the most recent update,
 *   not a historical bitwise accumulation.
 */
class AdaptiveCellPayload {
 public:
  AdaptiveCellPayload() = default;

  constexpr AdaptiveCellPayload(
      const ElevationStats& elevation,
      uint64_t first_seen_frame,
      uint64_t last_seen_frame,
      uint64_t observed_frame_count,
      ResolutionReasonBits last_update_reasons) noexcept
      : elevation_(elevation),
        first_seen_frame_(first_seen_frame),
        last_seen_frame_(last_seen_frame),
        observed_frame_count_(observed_frame_count),
        last_update_reasons_(last_update_reasons) {}

  AdaptiveCellPayload(const AdaptiveCellPayload&) = default;
  AdaptiveCellPayload& operator=(const AdaptiveCellPayload&) = default;
  AdaptiveCellPayload(AdaptiveCellPayload&&) noexcept = default;
  AdaptiveCellPayload& operator=(AdaptiveCellPayload&&) noexcept = default;

  [[nodiscard]] const ElevationStats& elevation() const noexcept {
    return elevation_;
  }

  [[nodiscard]] uint64_t firstSeenFrame() const noexcept {
    return first_seen_frame_;
  }

  [[nodiscard]] uint64_t lastSeenFrame() const noexcept {
    return last_seen_frame_;
  }

  [[nodiscard]] uint64_t observedFrameCount() const noexcept {
    return observed_frame_count_;
  }

  [[nodiscard]] ResolutionReasonBits lastUpdateReasons() const noexcept {
    return last_update_reasons_;
  }

  [[nodiscard]] bool hasObservations() const noexcept {
    return elevation_.count() > 0;
  }

  [[nodiscard]] uint64_t observationCount() const noexcept {
    return elevation_.count();
  }

  /**
   * @brief Transactionally fuses a frame-local elevation summary into this persistent payload.
   *
   * @param frame_elevation Aggregated elevation statistics from current sensor frame.
   * @param frame_seq Global sequence number of current frame.
   * @param reasons Decision reasons associated with the current frame update.
   * @return true if fused successfully, false on invalid input, ordering violation, or numeric failure.
   */
  bool tryFuse(
      const ElevationStats& frame_elevation,
      uint64_t frame_seq,
      ResolutionReasonBits reasons = ResolutionReasonBits{}) noexcept {
    if (frame_elevation.count() == 0) {
      return false;
    }

    if (observed_frame_count_ == 0) {
      ElevationStats candidate = elevation_;
      if (!candidate.tryMerge(frame_elevation)) {
        return false;
      }
      elevation_ = candidate;
      first_seen_frame_ = frame_seq;
      last_seen_frame_ = frame_seq;
      observed_frame_count_ = 1;
      last_update_reasons_ = reasons;
      return true;
    }

    // Subsequent fusion: must be strictly ordered frame_seq > last_seen_frame_
    if (frame_seq <= last_seen_frame_) {
      return false;
    }

    // Check overflow of observed_frame_count_
    if (observed_frame_count_ == std::numeric_limits<uint64_t>::max()) {
      return false;
    }

    ElevationStats candidate = elevation_;
    if (!candidate.tryMerge(frame_elevation)) {
      return false;
    }

    elevation_ = candidate;
    last_seen_frame_ = frame_seq;
    observed_frame_count_ += 1;
    last_update_reasons_ = reasons;
    return true;
  }

  bool operator==(const AdaptiveCellPayload& other) const noexcept = default;
  bool operator!=(const AdaptiveCellPayload& other) const noexcept = default;

 private:
  ElevationStats elevation_{};
  uint64_t first_seen_frame_{0};
  uint64_t last_seen_frame_{0};
  uint64_t observed_frame_count_{0};
  ResolutionReasonBits last_update_reasons_{};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
