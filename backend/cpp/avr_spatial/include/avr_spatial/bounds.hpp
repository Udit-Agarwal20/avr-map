#pragma once

#include <cstdint>
#include <optional>

#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::spatial {

/**
 * @brief Half-open 2D physical bounding box: [min_x, max_x) x [min_y, max_y).
 *
 * IMPORTANT ARCHITECTURAL INVARIANT:
 * HIERARCHY OWNERSHIP IS DEFINED IN INTEGER SPACE.
 * Floating physical bounds are derived visualization/geometric views.
 * Floating-point bounds must NEVER be used to determine hierarchy cell ownership.
 */
struct Bounds2D {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};

  constexpr bool operator==(const Bounds2D&) const noexcept = default;

  /**
   * @brief Evaluates whether a continuous world point falls inside the half-open interval.
   */
  [[nodiscard]] constexpr bool contains(double x, double y) const noexcept {
    return (x >= min_x && x < max_x) && (y >= min_y && y < max_y);
  }

  [[nodiscard]] constexpr bool contains(const WorldPoint2D& pt) const noexcept {
    return contains(pt.x, pt.y);
  }

  /**
   * @brief Evaluates whether this bounding box fully encloses another bounding box.
   */
  [[nodiscard]] constexpr bool encloses(const Bounds2D& other, double tolerance = 1e-9) const noexcept {
    return (other.min_x >= min_x - tolerance && other.max_x <= max_x + tolerance) &&
           (other.min_y >= min_y - tolerance && other.max_y <= max_y + tolerance);
  }
};

/**
 * @brief Computes physical metric bounds for a hierarchical cell.
 *
 * Uses stable cellsPerMeter(level) division and int64_t intermediate coordinate arithmetic
 * to prevent signed integer overflow at INT32_MAX.
 */
[[nodiscard]] inline Bounds2D cellBounds(
    const CellCoord& cell, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  const double cpm = cellsPerMeter(cell.level);
  if (cpm <= 0.0) [[unlikely]] {
    const double ox = static_cast<double>(origin.x) / BASE_CELLS_PER_METER;
    const double oy = static_cast<double>(origin.y) / BASE_CELLS_PER_METER;
    return Bounds2D{.min_x = ox, .min_y = oy, .max_x = ox, .max_y = oy};
  }

  const int64_t scale = static_cast<int64_t>(levelScale(cell.level));
  const int64_t global_base_x = static_cast<int64_t>(cell.x) * scale + static_cast<int64_t>(origin.x);
  const int64_t global_base_y = static_cast<int64_t>(cell.y) * scale + static_cast<int64_t>(origin.y);

  const double min_x = static_cast<double>(global_base_x) / BASE_CELLS_PER_METER;
  const double min_y = static_cast<double>(global_base_y) / BASE_CELLS_PER_METER;
  const double max_x = static_cast<double>(global_base_x + scale) / BASE_CELLS_PER_METER;
  const double max_y = static_cast<double>(global_base_y + scale) / BASE_CELLS_PER_METER;

  return Bounds2D{
      .min_x = min_x,
      .min_y = min_y,
      .max_x = max_x,
      .max_y = max_y,
  };
}

[[nodiscard]] inline Bounds2D cellBounds(
    const CellCoord& cell, const WorldPoint2D& metric_origin) noexcept {
  const auto grid_origin = metricOriginToGridOrigin(metric_origin);
  if (grid_origin.has_value()) {
    return cellBounds(cell, *grid_origin);
  }
  const double cpm = cellsPerMeter(cell.level);
  if (cpm <= 0.0) [[unlikely]] {
    return Bounds2D{.min_x = metric_origin.x, .min_y = metric_origin.y, .max_x = metric_origin.x, .max_y = metric_origin.y};
  }
  const double min_x = metric_origin.x + static_cast<double>(cell.x) / cpm;
  const double min_y = metric_origin.y + static_cast<double>(cell.y) / cpm;
  const double max_x = metric_origin.x + static_cast<double>(static_cast<int64_t>(cell.x) + 1) / cpm;
  const double max_y = metric_origin.y + static_cast<double>(static_cast<int64_t>(cell.y) + 1) / cpm;
  return Bounds2D{.min_x = min_x, .min_y = min_y, .max_x = max_x, .max_y = max_y};
}

[[nodiscard]] inline std::optional<Bounds2D> cellBoundsSafe(
    const CellCoord& cell, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  if (!isValidLevel(cell.level)) [[unlikely]] {
    return std::nullopt;
  }
  return cellBounds(cell, origin);
}

[[nodiscard]] inline std::optional<Bounds2D> cellBoundsSafe(
    const CellCoord& cell, const WorldPoint2D& metric_origin) noexcept {
  if (!isValidLevel(cell.level)) [[unlikely]] {
    return std::nullopt;
  }
  return cellBounds(cell, metric_origin);
}

[[nodiscard]] inline Bounds2D tileBounds(
    const TileCoord& tile, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  const double cpm = cellsPerMeter(tile.level);
  if (cpm <= 0.0) [[unlikely]] {
    const double ox = static_cast<double>(origin.x) / BASE_CELLS_PER_METER;
    const double oy = static_cast<double>(origin.y) / BASE_CELLS_PER_METER;
    return Bounds2D{.min_x = ox, .min_y = oy, .max_x = ox, .max_y = oy};
  }

  const int64_t scale = static_cast<int64_t>(levelScale(tile.level));
  const int64_t tile_cells = static_cast<int64_t>(TILE_SIZE) * scale;
  const int64_t global_base_x = static_cast<int64_t>(tile.x) * tile_cells + static_cast<int64_t>(origin.x);
  const int64_t global_base_y = static_cast<int64_t>(tile.y) * tile_cells + static_cast<int64_t>(origin.y);

  const double min_x = static_cast<double>(global_base_x) / BASE_CELLS_PER_METER;
  const double min_y = static_cast<double>(global_base_y) / BASE_CELLS_PER_METER;
  const double max_x = static_cast<double>(global_base_x + tile_cells) / BASE_CELLS_PER_METER;
  const double max_y = static_cast<double>(global_base_y + tile_cells) / BASE_CELLS_PER_METER;

  return Bounds2D{
      .min_x = min_x,
      .min_y = min_y,
      .max_x = max_x,
      .max_y = max_y,
  };
}

[[nodiscard]] inline Bounds2D tileBounds(
    const TileCoord& tile, const WorldPoint2D& metric_origin) noexcept {
  const auto grid_origin = metricOriginToGridOrigin(metric_origin);
  if (grid_origin.has_value()) {
    return tileBounds(tile, *grid_origin);
  }
  const double cpm = cellsPerMeter(tile.level);
  if (cpm <= 0.0) [[unlikely]] {
    return Bounds2D{.min_x = metric_origin.x, .min_y = metric_origin.y, .max_x = metric_origin.x, .max_y = metric_origin.y};
  }
  const double tile_span = static_cast<double>(TILE_SIZE) / cpm;
  const double min_x = metric_origin.x + static_cast<double>(tile.x) * tile_span;
  const double min_y = metric_origin.y + static_cast<double>(tile.y) * tile_span;
  const double max_x = metric_origin.x + static_cast<double>(static_cast<int64_t>(tile.x) + 1) * tile_span;
  const double max_y = metric_origin.y + static_cast<double>(static_cast<int64_t>(tile.y) + 1) * tile_span;
  return Bounds2D{.min_x = min_x, .min_y = min_y, .max_x = max_x, .max_y = max_y};
}

[[nodiscard]] inline std::optional<Bounds2D> tileBoundsSafe(
    const TileCoord& tile, const GridOrigin2D& origin = GridOrigin2D{0, 0}) noexcept {
  if (!isValidLevel(tile.level)) [[unlikely]] {
    return std::nullopt;
  }
  return tileBounds(tile, origin);
}

[[nodiscard]] inline std::optional<Bounds2D> tileBoundsSafe(
    const TileCoord& tile, const WorldPoint2D& metric_origin) noexcept {
  if (!isValidLevel(tile.level)) [[unlikely]] {
    return std::nullopt;
  }
  return tileBounds(tile, metric_origin);
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

