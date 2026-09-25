#pragma once

#include <cstdint>
#include <optional>

namespace avr::spatial {

/// Base spatial resolution for Level 0 in meters (5 cm)
inline constexpr double BASE_RESOLUTION_METERS = 0.05;

/// Number of finest base cells per meter (exact reciprocal of 0.05m = 20.0)
inline constexpr double BASE_CELLS_PER_METER = 20.0;

/// Total number of dyadic hierarchy levels (L0 to L3)
inline constexpr uint8_t LEVEL_COUNT = 4;

/// Dyadic hierarchy level identifiers
inline constexpr uint8_t L0 = 0;  // 0.05 m (5 cm)
inline constexpr uint8_t L1 = 1;  // 0.10 m (10 cm)
inline constexpr uint8_t L2 = 2;  // 0.20 m (20 cm)
inline constexpr uint8_t L3 = 3;  // 0.40 m (40 cm)

/// Spatial tile dimensions: 16 x 16 cells
inline constexpr uint32_t TILE_SIZE = 16;
inline constexpr uint32_t TILE_CELL_COUNT = TILE_SIZE * TILE_SIZE;  // 256 cells

/// Static invariants
static_assert(TILE_SIZE == 16, "AVR tile dimension must be 16");
static_assert(TILE_CELL_COUNT == 256, "AVR tile must contain 256 cells");
static_assert(LEVEL_COUNT == 4, "AVR V1 hierarchy must have exactly 4 levels");
static_assert(BASE_RESOLUTION_METERS * BASE_CELLS_PER_METER == 1.0,
              "Resolution and scale factor must be exact reciprocals");

static_assert(sizeof(int32_t) == 4, "int32_t must be exactly 4 bytes");
static_assert(sizeof(uint32_t) == 4, "uint32_t must be exactly 4 bytes");
static_assert(sizeof(int64_t) == 8, "int64_t must be exactly 8 bytes");
static_assert(sizeof(uint64_t) == 8, "uint64_t must be exactly 8 bytes");

/// Validates whether a hierarchy level is within [L0, L3]
[[nodiscard]] constexpr bool isValidLevel(uint8_t level) noexcept {
  return level < LEVEL_COUNT;
}

/// Returns the cell density (cells per meter) for a given hierarchy level.
/// L0 -> 20.0, L1 -> 10.0, L2 -> 5.0, L3 -> 2.5. Returns 0.0 for invalid levels.
[[nodiscard]] constexpr double cellsPerMeter(uint8_t level) noexcept {
  switch (level) {
    case 0:
      return 20.0;
    case 1:
      return 10.0;
    case 2:
      return 5.0;
    case 3:
      return 2.5;
    default:
      return 0.0;
  }
}

/// Returns the cell density wrapped in std::optional for fallible queries.
[[nodiscard]] constexpr std::optional<double> cellsPerMeterSafe(uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  return cellsPerMeter(level);
}

/// Returns the physical metric resolution for a given hierarchy level.
/// L0 -> 0.05m, L1 -> 0.10m, L2 -> 0.20m, L3 -> 0.40m.
/// Returns 0.0 for invalid levels (never shifts or generates fake resolutions).
[[nodiscard]] constexpr double resolutionMeters(uint8_t level) noexcept {
  switch (level) {
    case 0:
      return 0.05;
    case 1:
      return 0.10;
    case 2:
      return 0.20;
    case 3:
      return 0.40;
    default:
      return 0.0;
  }
}

/// Returns the physical metric resolution wrapped in std::optional for fallible queries.
[[nodiscard]] constexpr std::optional<double> resolutionMetersSafe(uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  return resolutionMeters(level);
}

/// Returns the scale factor in base cells (2^level) for valid levels [0, 3].
/// Returns 0 for invalid levels, preventing undefined shift behavior.
[[nodiscard]] constexpr uint32_t levelScale(uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return 0;
  }
  return 1U << level;
}

/// Returns the scale factor wrapped in std::optional for fallible queries.
[[nodiscard]] constexpr std::optional<uint32_t> levelScaleSafe(uint8_t level) noexcept {
  if (!isValidLevel(level)) [[unlikely]] {
    return std::nullopt;
  }
  return 1U << level;
}

}  // namespace avr::spatial

namespace AVR {
namespace spatial = ::avr::spatial;
}

