#pragma once

#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>

#include "avr_spatial/bounds.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/floor_div.hpp"
#include "avr_spatial/morton.hpp"

namespace avr::spatial {

/**
 * @brief Computes 1D local tile index [0, 255] in row-major order: index = local_y * 16 + local_x.
 */
[[nodiscard]] constexpr uint16_t localCellIndex(int32_t cx, int32_t cy) noexcept {
  const int32_t tx = floorDivPositive(cx, static_cast<int32_t>(TILE_SIZE));
  const int32_t ty = floorDivPositive(cy, static_cast<int32_t>(TILE_SIZE));
  const int64_t lx = static_cast<int64_t>(cx) - static_cast<int64_t>(tx) * static_cast<int64_t>(TILE_SIZE);
  const int64_t ly = static_cast<int64_t>(cy) - static_cast<int64_t>(ty) * static_cast<int64_t>(TILE_SIZE);
  return static_cast<uint16_t>(ly * TILE_SIZE + lx);
}

/**
 * @brief Inverse of localCellIndex(): returns (local_x, local_y) in [0, 15] from a 1D index [0, 255].
 */
[[nodiscard]] constexpr std::pair<int32_t, int32_t> localCoordsFromIndex(
    uint16_t index) noexcept {
  const int32_t lx = static_cast<int32_t>(index % TILE_SIZE);
  const int32_t ly = static_cast<int32_t>(index / TILE_SIZE);
  return {lx, ly};
}

/**
 * @brief Computes 2D TileCoord (tx, ty, level) containing the given cell coordinate.
 *
 * Negative cell coordinates are correctly partitioned:
 *   cx = -1  -> tx = -1, lx = 15
 *   cx = -16 -> tx = -1, lx = 0
 *   cx = -17 -> tx = -2, lx = 15
 */
[[nodiscard]] constexpr TileCoord cellToTileCoord(const CellCoord& cell) noexcept {
  assert(isValidLevel(cell.level) && "cellToTileCoord requires valid level [0, 3]");
  return TileCoord{
      .x = floorDivPositive(cell.x, static_cast<int32_t>(TILE_SIZE)),
      .y = floorDivPositive(cell.y, static_cast<int32_t>(TILE_SIZE)),
      .level = cell.level,
  };
}

/**
 * @brief Computes 2D TileCoord with level validation, returning std::nullopt if level is invalid.
 */
[[nodiscard]] constexpr std::optional<TileCoord> cellToTileCoordSafe(
    const CellCoord& cell) noexcept {
  if (!isValidLevel(cell.level)) [[unlikely]] {
    return std::nullopt;
  }
  return cellToTileCoord(cell);
}

/**
 * @brief Lightweight, unique spatial identifier for a 16x16 cell Tile.
 */
struct TileKey {
  uint64_t morton{0};
  uint8_t level{0};

  constexpr bool operator==(const TileKey&) const noexcept = default;
  constexpr auto operator<=>(const TileKey&) const noexcept = default;

  /**
   * @brief Constructs TileKey from explicit TileCoord.
   *        Returns std::nullopt if level is invalid (>= LEVEL_COUNT).
   */
  [[nodiscard]] static constexpr std::optional<TileKey> fromTileCoord(
      const TileCoord& coord) noexcept {
    if (!isValidLevel(coord.level)) [[unlikely]] {
      return std::nullopt;
    }
    return TileKey{
        .morton = morton2DEncode(coord.x, coord.y),
        .level = coord.level,
    };
  }

  /**
   * @brief Constructs TileKey directly from a CellKey.
   *        Returns std::nullopt if level is invalid.
   */
  [[nodiscard]] static constexpr std::optional<TileKey> fromCellKey(
      const CellKey& cell_key) noexcept {
    if (!isValidLevel(cell_key.level)) [[unlikely]] {
      return std::nullopt;
    }
    const CellCoord cell_coord = cell_key.decode();
    const TileCoord tile_coord = cellToTileCoord(cell_coord);
    return fromTileCoord(tile_coord);
  }

  /**
   * @brief Decodes this TileKey back into signed tile coordinates (tx, ty, level).
   */
  [[nodiscard]] constexpr TileCoord decode() const noexcept {
    int32_t tx = 0;
    int32_t ty = 0;
    morton2DDecode(morton, tx, ty);
    return TileCoord{.x = tx, .y = ty, .level = level};
  }

  /**
   * @brief Returns physical metric bounds [min_x, max_x) x [min_y, max_y) for this entire 16x16 tile.
   */
  [[nodiscard]] Bounds2D bounds(const WorldPoint2D& origin = {0.0, 0.0}) const noexcept {
    return tileBounds(decode(), origin);
  }
};

/**
 * @brief Reconstructs the exact CellKey belonging to a TileKey at a given local slot index [0, 255].
 *
 * Contract:
 * - Returns std::nullopt if local_index >= 256 or level is invalid.
 * - Uses 64-bit intermediate arithmetic to prevent signed overflow near INT32_MIN / INT32_MAX.
 * - Returns std::nullopt if the reconstructed cell coordinates would exceed 32-bit representable range.
 */
[[nodiscard]] constexpr std::optional<CellKey> cellKeyFromTileAndLocal(
    const TileKey& tile_key, uint16_t local_index) noexcept {
  if (!isValidLevel(tile_key.level) || local_index >= TILE_CELL_COUNT) [[unlikely]] {
    return std::nullopt;
  }
  const TileCoord tile_coord = tile_key.decode();
  const auto [lx, ly] = localCoordsFromIndex(local_index);

  const int64_t cx64 = static_cast<int64_t>(tile_coord.x) * static_cast<int64_t>(TILE_SIZE) + lx;
  const int64_t cy64 = static_cast<int64_t>(tile_coord.y) * static_cast<int64_t>(TILE_SIZE) + ly;

  if (cx64 < std::numeric_limits<int32_t>::min() ||
      cx64 > std::numeric_limits<int32_t>::max() ||
      cy64 < std::numeric_limits<int32_t>::min() ||
      cy64 > std::numeric_limits<int32_t>::max()) [[unlikely]] {
    return std::nullopt;
  }

  return CellKey::fromCellCoord(CellCoord{
      .x = static_cast<int32_t>(cx64),
      .y = static_cast<int32_t>(cy64),
      .level = tile_key.level,
  });
}

}  // namespace avr::spatial

// Hash functor specialization for standard containers
template <>
struct std::hash<avr::spatial::TileKey> {
  [[nodiscard]] std::size_t operator()(const avr::spatial::TileKey& k) const noexcept {
    std::size_t h = std::hash<uint64_t>{}(k.morton);
    h ^= std::hash<uint8_t>{}(k.level) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

namespace AVR {
namespace spatial = ::avr::spatial;
}

