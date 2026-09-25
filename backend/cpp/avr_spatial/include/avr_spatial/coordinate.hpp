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
 * @brief Hardened quantization of a continuous 1D world coordinate to integer base-grid units.
 *
 * Core Quantization Contract (ADR-007):
 * - Integer cell coordinates define spatial ownership.
 * - Continuous world coordinates are converted into discrete integer ownership via exact half-open interval [min, max).
 * - Physical metric bounds are derived geometric views, never ground-truth ownership definitions.
 * - The canonical map origin is permanently anchored to (0.0, 0.0) in the 'odom' frame.
 * - Map shifts and submap origins are represented as integer base-cell offsets on the canonical grid.
 * - Fuzzy snapping tolerances and magic epsilon shifts are rejected to preserve strict half-open ownership.
 * - Non-finite inputs (NaN, +/-Inf) and out-of-range coordinates fail explicitly (std::nullopt).
 * - Never invokes float-to-int overflow undefined behavior; never throws exceptions.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGridSafe(
    double coord, double origin = 0.0) noexcept {
  if (!std::isfinite(coord) || !std::isfinite(origin)) [[unlikely]] {
    return std::nullopt;
  }

  // Exact half-open floor quantization relative to canonical grid.
  // When origin is non-zero, it is aligned to the canonical integer base-cell grid (ADR-007).
  const double c_cell = std::floor(coord * BASE_CELLS_PER_METER);
  const double o_cell = (origin == 0.0) ? 0.0 : std::round(origin * BASE_CELLS_PER_METER);
  const double v = c_cell - o_cell;

  if (!std::isfinite(v) ||
      v < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      v > static_cast<double>(std::numeric_limits<int32_t>::max())) [[unlikely]] {
    return std::nullopt;
  }

  return static_cast<int32_t>(v);
}

/**
 * @brief Public 1D base-grid quantization API returning std::optional<int32_t>.
 */
[[nodiscard]] inline std::optional<int32_t> quantizeToBaseGrid(
    double coord, double origin = 0.0) noexcept {
  return quantizeToBaseGridSafe(coord, origin);
}

/**
 * @brief Quantizes a 2D continuous world point into finest base-grid integer coordinates.
 *
 * Returns std::nullopt if any coordinate is non-finite (NaN, Inf) or out-of-range (> INT32_MAX, < INT32_MIN).
 */
[[nodiscard]] inline std::optional<GridCoord2D> worldToGridCoord(
    const WorldPoint2D& pt, const WorldPoint2D& origin = {0.0, 0.0}) noexcept {
  const auto qx = quantizeToBaseGridSafe(pt.x, origin.x);
  const auto qy = quantizeToBaseGridSafe(pt.y, origin.y);
  if (!qx.has_value() || !qy.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return GridCoord2D{.x = *qx, .y = *qy};
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
  return floorDivPositive(base_coord, static_cast<int32_t>(levelScale(level)));
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
  return floorDivPositive(base_coord, static_cast<int32_t>(levelScale(level)));
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
 * @brief Maps a continuous world point to a hierarchical cell coordinate at the requested level.
 *
 * Returns std::nullopt if level is invalid, coordinates are non-finite, or coordinates are out-of-range.
 */
[[nodiscard]] inline std::optional<CellCoord> worldToCellCoord(
    const WorldPoint2D& pt, uint8_t level, const WorldPoint2D& origin = {0.0, 0.0}) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  const auto base_grid = worldToGridCoord(pt, origin);
  if (!base_grid.has_value()) [[unlikely]] {
    return std::nullopt;
  }
  return gridToCellCoord(*base_grid, level);
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

