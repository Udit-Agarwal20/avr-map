#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "avr_spatial/cell_key.hpp"

namespace avr::mapping {

/**
 * @brief Aggregate evidence summary over an arbitrary valid CellKey footprint (L0 - L3).
 *
 * Provides a read-only aggregate of observations, spatial coverage, and vertical relief
 * over any hierarchical cell footprint:
 * - L0: 1 base cell (capacity = 1)
 * - L1: 2x2 = 4 base cells (capacity = 4)
 * - L2: 4x4 = 16 base cells (capacity = 16)
 * - L3: 8x8 = 64 base cells (capacity = 64)
 *
 * Semantics:
 * - density_score is the frame-local spatial coverage proxy: occupied_base_cells / base_cell_capacity.
 * - geometry_complexity is the vertical-relief proxy: clamp((z_max - z_min) / region_width, 0, 1)
 *   if supported by >= 2 observations and >= 2 distinct base cells; otherwise std::nullopt.
 * - Empty regions return observation_count = 0, occupied_base_cells = 0, density_score = 0.0,
 *   and all elevation/geometry optionals as std::nullopt.
 */
struct EvidenceSummary {
  avr::spatial::CellKey region{};

  uint64_t observation_count{0};
  uint32_t occupied_base_cells{0};
  uint32_t base_cell_capacity{0};
  double density_score{0.0};

  std::optional<double> elevation_mean{std::nullopt};
  /// Population variance (M2 / n) of observations inside this frame region (nullopt if empty).
  std::optional<double> elevation_variance{std::nullopt};
  std::optional<double> z_min{std::nullopt};
  std::optional<double> z_max{std::nullopt};
  std::optional<double> geometry_complexity{std::nullopt};

  [[nodiscard]] bool isEmpty() const noexcept {
    return observation_count == 0 && occupied_base_cells == 0;
  }

  constexpr bool operator==(const EvidenceSummary& other) const noexcept = default;
};

}  // namespace avr::mapping

namespace AVR {
namespace mapping = ::avr::mapping;
namespace mapping_core = ::avr::mapping;
}
