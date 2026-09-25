#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace avr::mapping {

/**
 * @brief Numerically stable online elevation statistics using Welford's algorithm.
 *
 * Maintains running sample count (n), running mean, and sum of squared differences (M2)
 * for elevation observations z.
 *
 * Guarantees:
 * - O(1) time update per observation.
 * - O(1) memory footprint (no historical observation storage).
 * - Safe handling of n = 0 and n = 1 (no division by zero).
 * - Finite and accurate variance even with large baseline offsets.
 * - Strict transactional safety: failed updates leave previous state unmodified.
 * - uint64_t observation counter with saturation/overflow guard.
 */
class ElevationStats {
 public:
  constexpr ElevationStats() noexcept = default;

  constexpr ElevationStats(uint64_t count, double mean, double m2, double min, double max) noexcept
      : mean_(mean), m2_(m2), min_(min), max_(max), count_(count) {}

  /**
   * @brief Transactionally attempts to ingest an elevation observation z using Welford's update.
   *
   * Validates all candidate values (mean, M2, min, max, count) are finite and valid before
   * committing. If an extreme observation would cause intermediate or final arithmetic overflow,
   * returns false and leaves the existing state completely unmodified.
   *
   * @return true if candidate values were valid and committed, false if rejected.
   */
  bool tryUpdate(double z) noexcept {
    if (!std::isfinite(z)) {
      return false;
    }
    if (count_ == std::numeric_limits<uint64_t>::max()) {
      return false;
    }
    if (count_ == 0) {
      mean_ = z;
      m2_ = 0.0;
      min_ = z;
      max_ = z;
      count_ = 1;
      return true;
    }

    const uint64_t cand_count = count_ + 1;
    const double delta = z - mean_;
    if (!std::isfinite(delta)) {
      return false;
    }

    const double cand_mean = mean_ + delta / static_cast<double>(cand_count);
    if (!std::isfinite(cand_mean)) {
      return false;
    }

    const double delta2 = z - cand_mean;
    if (!std::isfinite(delta2)) {
      return false;
    }

    const double term = delta * delta2;
    if (!std::isfinite(term)) {
      return false;
    }

    const double cand_m2 = m2_ + term;
    if (!std::isfinite(cand_m2) || cand_m2 < 0.0) {
      return false;
    }

    const double cand_min = (z < min_) ? z : min_;
    const double cand_max = (z > max_) ? z : max_;
    if (!std::isfinite(cand_min) || !std::isfinite(cand_max)) {
      return false;
    }

    // All candidate values validated as finite and safe -> commit transaction
    mean_ = cand_mean;
    m2_ = cand_m2;
    min_ = cand_min;
    max_ = cand_max;
    count_ = cand_count;
    return true;
  }

  /**
   * @brief Ingests an elevation observation z, ignoring numeric failures.
   */
  void update(double z) noexcept {
    tryUpdate(z);
  }

  [[nodiscard]] constexpr uint64_t count() const noexcept { return count_; }
  [[nodiscard]] constexpr double mean() const noexcept { return mean_; }
  [[nodiscard]] constexpr double m2() const noexcept { return m2_; }
  [[nodiscard]] constexpr double min() const noexcept { return min_; }
  [[nodiscard]] constexpr double max() const noexcept { return max_; }

  /**
   * @brief Population variance: M2 / n.
   *        Returns 0.0 for n = 0 or n = 1.
   */
  [[nodiscard]] double populationVariance() const noexcept {
    if (count_ == 0) {
      return 0.0;
    }
    return m2_ / static_cast<double>(count_);
  }

  /**
   * @brief Sample variance: M2 / (n - 1).
   *        Returns 0.0 for n < 2.
   */
  [[nodiscard]] double sampleVariance() const noexcept {
    if (count_ < 2) {
      return 0.0;
    }
    return m2_ / static_cast<double>(count_ - 1);
  }

  void reset() noexcept {
    mean_ = 0.0;
    m2_ = 0.0;
    min_ = 0.0;
    max_ = 0.0;
    count_ = 0;
  }

  constexpr bool operator==(const ElevationStats&) const noexcept = default;

 private:
  double mean_{0.0};
  double m2_{0.0};
  double min_{0.0};
  double max_{0.0};
  uint64_t count_{0};
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
}
