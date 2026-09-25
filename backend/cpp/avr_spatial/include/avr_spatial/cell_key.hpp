#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/floor_div.hpp"
#include "avr_spatial/morton.hpp"

namespace avr::spatial {

/**
 * @brief Lightweight, unique spatial identifier for an adaptive map cell.
 *
 * Encodes cell position at a given hierarchy level via 64-bit Morton code and 8-bit level.
 */
struct CellKey {
  uint64_t morton{0};
  uint8_t level{0};

  constexpr bool operator==(const CellKey&) const noexcept = default;
  constexpr auto operator<=>(const CellKey&) const noexcept = default;

  /**
   * @brief Constructs a CellKey from explicit level cell coordinates (cx, cy, level).
   *        Returns std::nullopt if level is invalid (>= LEVEL_COUNT).
   */
  [[nodiscard]] static constexpr std::optional<CellKey> fromCellCoord(
      const CellCoord& coord) noexcept {
    if (!isValidLevel(coord.level)) [[unlikely]] {
      return std::nullopt;
    }
    return CellKey{
        .morton = morton2DEncode(coord.x, coord.y),
        .level = coord.level,
    };
  }

  /**
   * @brief Constructs a CellKey from finest base-grid coordinates and target hierarchy level.
   *        Returns std::nullopt if level is invalid.
   */
  [[nodiscard]] static constexpr std::optional<CellKey> fromGridCoord(
      const GridCoord2D& base_grid, uint8_t level) noexcept {
    if (!isValidLevel(level)) [[unlikely]] {
      return std::nullopt;
    }
    const CellCoord cell_coord = gridToCellCoord(base_grid, level);
    return fromCellCoord(cell_coord);
  }

  /**
   * @brief Constructs a CellKey from a continuous world point and target hierarchy level.
   *        Returns std::nullopt if level is invalid, point is non-finite, or point is out-of-range.
   */
  [[nodiscard]] static std::optional<CellKey> fromWorldPoint(
      const WorldPoint2D& pt, uint8_t level, const WorldPoint2D& origin = {0.0, 0.0}) noexcept {
    if (!isValidLevel(level)) [[unlikely]] {
      return std::nullopt;
    }
    const auto cell_coord = worldToCellCoord(pt, level, origin);
    if (!cell_coord.has_value()) [[unlikely]] {
      return std::nullopt;
    }
    return fromCellCoord(*cell_coord);
  }

  /**
   * @brief Decodes this key back into signed integer cell coordinates (cx, cy, level).
   */
  [[nodiscard]] constexpr CellCoord decode() const noexcept {
    int32_t cx = 0;
    int32_t cy = 0;
    morton2DDecode(morton, cx, cy);
    return CellCoord{.x = cx, .y = cy, .level = level};
  }

  /**
   * @brief Computes physical metric bounds [min_x, max_x) x [min_y, max_y) for this cell.
   */
  [[nodiscard]] Bounds2D bounds(const WorldPoint2D& origin = {0.0, 0.0}) const noexcept {
    return cellBounds(decode(), origin);
  }
};

/**
 * @brief Returns the parent CellKey at level L+1.
 *
 * Contract:
 * - If cell is at the coarsest level (L3) or has an invalid level, returns std::nullopt.
 * - Parent coordinates are derived via exact integer floor division:
 *     px = floorDivPositive(cx, 2)
 *     py = floorDivPositive(cy, 2)
 * - Never overflows signed 32-bit integer.
 */
[[nodiscard]] constexpr std::optional<CellKey> parentOf(const CellKey& key) noexcept {
  if (!isValidLevel(key.level) || key.level >= L3) [[unlikely]] {
    return std::nullopt;
  }
  const CellCoord coord = key.decode();
  const int32_t px = floorDivPositive(coord.x, 2);
  const int32_t py = floorDivPositive(coord.y, 2);
  return CellKey::fromCellCoord(CellCoord{
      .x = px,
      .y = py,
      .level = static_cast<uint8_t>(key.level + 1),
  });
}

/**
 * @brief Returns the four child CellKeys at level L-1 in deterministic Morton / Z-order:
 *   Index 0: (2cx,     2cy)     [South-West]
 *   Index 1: (2cx + 1, 2cy)     [South-East]
 *   Index 2: (2cx,     2cy + 1) [North-West]
 *   Index 3: (2cx + 1, 2cy + 1) [North-East]
 *
 * Contract:
 * - If cell is at the finest level (L0) or has an invalid level, returns std::nullopt.
 * - Uses 64-bit intermediate arithmetic to audit coordinate representability:
 *   If 2cx or 2cx + 1 overflows int32_t, fails safely by returning std::nullopt.
 * - Zero signed integer overflow undefined behavior.
 */
[[nodiscard]] constexpr std::optional<std::array<CellKey, 4>> childrenOf(
    const CellKey& key) noexcept {
  if (!isValidLevel(key.level) || key.level <= L0) [[unlikely]] {
    return std::nullopt;
  }
  const CellCoord coord = key.decode();

  // Audit 2*cx and 2*cx+1 for signed 32-bit overflow using 64-bit intermediates
  const int64_t bx64 = static_cast<int64_t>(coord.x) * 2;
  const int64_t by64 = static_cast<int64_t>(coord.y) * 2;

  if (bx64 < std::numeric_limits<int32_t>::min() ||
      bx64 + 1 > std::numeric_limits<int32_t>::max() ||
      by64 < std::numeric_limits<int32_t>::min() ||
      by64 + 1 > std::numeric_limits<int32_t>::max()) [[unlikely]] {
    return std::nullopt;
  }

  const uint8_t child_level = static_cast<uint8_t>(key.level - 1);
  const int32_t bx = static_cast<int32_t>(bx64);
  const int32_t by = static_cast<int32_t>(by64);

  return std::array<CellKey, 4>{
      *CellKey::fromCellCoord(CellCoord{.x = bx, .y = by, .level = child_level}),
      *CellKey::fromCellCoord(CellCoord{.x = bx + 1, .y = by, .level = child_level}),
      *CellKey::fromCellCoord(CellCoord{.x = bx, .y = by + 1, .level = child_level}),
      *CellKey::fromCellCoord(CellCoord{.x = bx + 1, .y = by + 1, .level = child_level}),
  };
}

}  // namespace avr::spatial

// Hash functor specialization for standard containers
template <>
struct std::hash<avr::spatial::CellKey> {
  [[nodiscard]] std::size_t operator()(const avr::spatial::CellKey& k) const noexcept {
    // 64-bit splitmix / hash combine with golden ratio
    std::size_t h = std::hash<uint64_t>{}(k.morton);
    h ^= std::hash<uint8_t>{}(k.level) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

namespace AVR {
namespace spatial = ::avr::spatial;
}

