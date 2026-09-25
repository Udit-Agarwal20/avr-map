#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "avr_spatial/constants.hpp"
#include "avr_spatial/floor_div.hpp"

namespace avr::spatial {

/**
 * @brief Continuous 2D world coordinates in meters (typically in the 'odom' frame).
 */
struct WorldPoint2D {
  double x{0.0};
  double y{0.0};

  constexpr bool operator==(const WorldPoint2D&) const noexcept = default;
};

/**
 * @brief Integer discrete 2D coordinates on the finest base grid (Level 0, 5 cm).
 *
 * Supported coordinate range: int32_t [-2,147,483,648, +2,147,483,647].
 * At 0.05m per cell, this spans over +/-100,000 km, vastly exceeding the 100m rolling map radius.
 */
struct GridCoord2D {
  int32_t x{0};
  int32_t y{0};

  constexpr bool operator==(const GridCoord2D&) const noexcept = default;
};

/**
 * @brief Discrete 2D integer origin offset on the finest base grid (Level 0, 5 cm).
 *
 * Core Origin Contract (ADR-007, ADR-008):
 * - The canonical AVR map grid is permanently anchored to the 'odom' frame at (0, 0).
 * - A rolling local map moves by allocating and deallocating integer cells; the grid never shifts.
 * - Any logical origin offset is represented as integer base-cell offsets (x, y).
 * - Canonical origin is GridOrigin2D{0, 0}.
 */
struct GridOrigin2D {
  int32_t x{0};
  int32_t y{0};

  constexpr bool operator==(const GridOrigin2D&) const noexcept = default;
  constexpr auto operator<=>(const GridOrigin2D&) const noexcept = default;

  [[nodiscard]] static constexpr GridOrigin2D canonical() noexcept {
    return GridOrigin2D{0, 0};
  }
};

/**
 * @brief Subtracts an integer origin offset from a global base-cell coordinate with checked int32 arithmetic.
 *
 * Computes: local_cell = global_cell - origin_cell.
 * Returns std::nullopt if the result falls outside [INT32_MIN, INT32_MAX]. Never wraps.
 */
[[nodiscard]] constexpr std::optional<int32_t> checkedCellOffset(
    int32_t global_cell, int32_t origin_cell) noexcept {
  const int64_t diff = static_cast<int64_t>(global_cell) - static_cast<int64_t>(origin_cell);
  if (diff < static_cast<int64_t>(std::numeric_limits<int32_t>::min()) ||
      diff > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) [[unlikely]] {
    return std::nullopt;
  }
  return static_cast<int32_t>(diff);
}

/**
 * @brief Converts a 1D continuous metric origin in meters to an integer base-cell offset.
 *
 * Contract (ADR-008):
 * - Succeeds ONLY if the metric origin lies exactly on a canonical 5 cm base-grid boundary.
 * - Non-aligned origins (e.g., 0.01, 0.02, 0.033, 1.001, -0.014) fail explicitly (std::nullopt).
 * - Never silently rounds or shifts coordinate systems.
 */
[[nodiscard]] constexpr std::optional<int32_t> metricOriginToGridCell(double origin) noexcept {
  if (!std::isfinite(origin)) [[unlikely]] {
    return std::nullopt;
  }
  const double cell_d = std::round(origin * BASE_CELLS_PER_METER);
  if (cell_d < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      cell_d > static_cast<double>(std::numeric_limits<int32_t>::max())) [[unlikely]] {
    return std::nullopt;
  }
  const int32_t cell = static_cast<int32_t>(cell_d);
  const double recon = static_cast<double>(cell) / BASE_CELLS_PER_METER;
  if (recon != origin) [[unlikely]] {
    return std::nullopt;
  }
  return cell;
}

/**
 * @brief Converts a 2D continuous metric origin to an integer GridOrigin2D base-cell offset.
 *
 * Returns std::nullopt if either coordinate does not lie exactly on a canonical 5 cm base-grid boundary.
 */
[[nodiscard]] constexpr std::optional<GridOrigin2D> metricOriginToGridOrigin(
    const WorldPoint2D& origin) noexcept {
  const auto gx = metricOriginToGridCell(origin.x);
  const auto gy = metricOriginToGridCell(origin.y);
  if (!gx.has_value() || !gy.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return GridOrigin2D{.x = *gx, .y = *gy};
}

/**
 * @brief Integer coordinates of a cell at a specific dyadic hierarchy level.
 */
struct CellCoord {
  int32_t x{0};
  int32_t y{0};
  uint8_t level{0};

  constexpr bool operator==(const CellCoord&) const noexcept = default;
};

/**
 * @brief Integer coordinates of a 16x16 tile at a specific hierarchy level.
 */
struct TileCoord {
  int32_t x{0};
  int32_t y{0};
  uint8_t level{0};

  constexpr bool operator==(const TileCoord&) const noexcept = default;
};

/**
 * @brief Hardened quantization of a continuous 1D world coordinate to canonical integer base-grid units.
 *
 * Core Quantization Contract (ADR-007, ADR-008):
 * - Integer cell coordinates define spatial ownership.
 * - Continuous world coordinates are converted into discrete integer ownership via exact half-open interval [min, max).
 * - Physical metric bounds are derived geometric views, never ground-truth ownership definitions.
 * - The canonical map origin is permanently anchored to (0.0, 0.0) in the 'odom' frame.
 * - Map shifts and submap origins are represented as integer base-cell offsets on the canonical grid.
 * - Fuzzy snapping tolerances and silent rounding of non-aligned origins are forbidden.
 * - Non-finite inputs (NaN, +/-Inf) and out-of-range coordinates fail explicitly (std::nullopt).
 * - Never invokes float-to-int overflow undefined behavior; never throws exceptions.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGridSafe(double coord) noexcept {
  if (!std::isfinite(coord)) [[unlikely]] {
    return std::nullopt;
  }

  const double v = std::floor(coord * BASE_CELLS_PER_METER);

  if (!std::isfinite(v) ||
      v < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      v > static_cast<double>(std::numeric_limits<int32_t>::max())) [[unlikely]] {
    return std::nullopt;
  }

  return static_cast<int32_t>(v);
}

/**
 * @brief Quantizes a 1D world coordinate relative to an integer base-cell origin offset.
 *
 * Uses checked integer arithmetic to prevent signed overflow.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGridSafe(
    double coord, int32_t origin_cell) noexcept {
  const auto global_cell = quantizeToBaseGridSafe(coord);
  if (!global_cell.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return checkedCellOffset(*global_cell, origin_cell);
}

/**
 * @brief Quantizes a 1D world coordinate relative to a metric origin.
 *
 * Succeeds ONLY if the metric origin lies exactly on a canonical base-grid boundary.
 * Never silently rounds a non-aligned origin.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGridSafe(
    double coord, double metric_origin) noexcept {
  if (metric_origin == 0.0) {
    return quantizeToBaseGridSafe(coord);
  }
  const auto o_cell = metricOriginToGridCell(metric_origin);
  if (!o_cell.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return quantizeToBaseGridSafe(coord, *o_cell);
}

/**
 * @brief Public 1D base-grid quantization APIs returning std::optional<int32_t>.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGrid(double coord) noexcept {
  return quantizeToBaseGridSafe(coord);
}

[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGrid(
    double coord, int32_t origin_cell) noexcept {
  return quantizeToBaseGridSafe(coord, origin_cell);
}

[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGrid(
    double coord, double metric_origin) noexcept {
  return quantizeToBaseGridSafe(coord, metric_origin);
}

/**
 * @brief Quantizes a 2D continuous world point into finest base-grid integer coordinates
 *        relative to an integer GridOrigin2D (default: canonical GridOrigin2D{0, 0}).
 *
 * Returns std::nullopt if any coordinate is non-finite or integer subtraction overflows.
 */
[[nodiscard]] inline std::optional<GridCoord2D> worldToGridCoord(
    const WorldPoint2D& pt, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  const auto qx = quantizeToBaseGridSafe(pt.x, origin.x);
  const auto qy = quantizeToBaseGridSafe(pt.y, origin.y);
  if (!qx.has_value() || !qy.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return GridCoord2D{.x = *qx, .y = *qy};
}

/**
 * @brief Overload accepting continuous WorldPoint2D metric origin.
 *        Succeeds ONLY if metric origin is exactly grid-aligned.
 */
[[nodiscard]] inline std::optional<GridCoord2D> worldToGridCoord(
    const WorldPoint2D& pt, const WorldPoint2D& metric_origin) noexcept {
  const auto grid_origin = metricOriginToGridOrigin(metric_origin);
  if (!grid_origin.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return worldToGridCoord(pt, *grid_origin);
}

/**
 * @brief Maps a base-grid integer coordinate to a coarser level coordinate using floor division.
 *        Returns std::nullopt if level is invalid (>= LEVEL_COUNT).
 */
[[nodiscard]] constexpr std::optional<int32_t> baseToLevelCoordSafe(
    int32_t base_coord, uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  return detail::floorDivPositive(base_coord, static_cast<int32_t>(levelScale(level)));
}

/**
 * @brief Fast mapping of base-grid coordinate to level coordinate for validated levels.
 *
 * In debug mode asserts isValidLevel. In release mode returns 0 on invalid level without UB.
 */
[[nodiscard]] constexpr int32_t baseToLevelCoord(int32_t base_coord, uint8_t level) noexcept {
  assert(isValidLevel(level) && "baseToLevelCoord requires valid level [0, 3]");
  if (!isValidLevel(level)) [[unlikely]] {
    return 0;
  }
  return detail::floorDivPositive(base_coord, static_cast<int32_t>(levelScale(level)));
}

/**
 * @brief Converts a finest base-grid coordinate to a hierarchical cell coordinate.
 *        Returns std::nullopt if level is invalid.
 */
[[nodiscard]] constexpr std::optional<CellCoord> gridToCellCoordSafe(
    const GridCoord2D& base_grid, uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  return CellCoord{
      .x = baseToLevelCoord(base_grid.x, level),
      .y = baseToLevelCoord(base_grid.y, level),
      .level = level,
  };
}

/**
 * @brief Converts a finest base-grid coordinate to a hierarchical cell coordinate.
 */
[[nodiscard]] constexpr CellCoord gridToCellCoord(
    const GridCoord2D& base_grid, uint8_t level) noexcept {
  assert(isValidLevel(level) && "gridToCellCoord requires valid level [0, 3]");
  return CellCoord{
      .x = baseToLevelCoord(base_grid.x, level),
      .y = baseToLevelCoord(base_grid.y, level),
      .level = level,
  };
}

/**
 * @brief Maps a continuous world point to a hierarchical cell coordinate at the requested level
 *        relative to an integer GridOrigin2D.
 *
 * Returns std::nullopt if level is invalid, coordinates are non-finite, or subtraction overflows.
 */
[[nodiscard]] inline std::optional<CellCoord> worldToCellCoord(
    const WorldPoint2D& pt, uint8_t level, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  const auto base_grid = worldToGridCoord(pt, origin);
  if (!base_grid.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return gridToCellCoord(*base_grid, level);
}

/**
 * @brief Overload accepting continuous WorldPoint2D metric origin.
 *        Succeeds ONLY if metric origin is exactly grid-aligned.
 */
[[nodiscard]] inline std::optional<CellCoord> worldToCellCoord(
    const WorldPoint2D& pt, uint8_t level, const WorldPoint2D& metric_origin) noexcept {
  const auto grid_origin = metricOriginToGridOrigin(metric_origin);
  if (!grid_origin.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return worldToCellCoord(pt, level, *grid_origin);
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

