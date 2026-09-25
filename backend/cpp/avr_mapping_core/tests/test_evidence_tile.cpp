#include <gtest/gtest.h>

#include <iostream>

#include "avr_mapping/evidence_cell.hpp"
#include "avr_mapping/evidence_summary.hpp"
#include "avr_mapping/evidence_tile.hpp"
#include "avr_spatial/constants.hpp"
#include "avr_spatial/coordinate.hpp"
#include "avr_spatial/tile_key.hpp"

using namespace avr::mapping;
using namespace avr::spatial;

TEST(EvidenceTileLayout, ReportMemoryLayout) {
  std::cout << "\n==================================================\n";
  std::cout << "M5 LAYOUT REPORT:\n";
  std::cout << "sizeof(EvidenceCell):    " << sizeof(EvidenceCell)
            << " bytes, alignof: " << alignof(EvidenceCell) << "\n";
  std::cout << "sizeof(EvidenceTile):    " << sizeof(EvidenceTile)
            << " bytes, alignof: " << alignof(EvidenceTile) << "\n";
  std::cout << "sizeof(EvidenceSummary): " << sizeof(EvidenceSummary)
            << " bytes, alignof: " << alignof(EvidenceSummary) << "\n";
  std::cout << "==================================================\n";

  EXPECT_EQ(sizeof(EvidenceCell), 40);
  EXPECT_EQ(alignof(EvidenceCell), 8);
  EXPECT_EQ(sizeof(EvidenceTile), 10280);
  EXPECT_EQ(alignof(EvidenceTile), 8);
}

TEST(EvidenceTileTest, DefaultStateAndActivation) {
  EvidenceTile tile;
  EXPECT_EQ(tile.activeCellCount(), 0);
  EXPECT_FALSE(tile.activeMask().any());

  for (uint16_t i = 0; i < EvidenceTile::CELL_COUNT; ++i) {
    EXPECT_FALSE(tile.isCellActive(i));
    EXPECT_EQ(tile.cell(i).count(), 0);
  }

  // Update cell 42
  auto res1 = tile.updateCell(42, 1.25);
  EXPECT_TRUE(res1.success);
  EXPECT_TRUE(res1.became_active);
  EXPECT_EQ(tile.activeCellCount(), 1);
  EXPECT_TRUE(tile.isCellActive(42));
  EXPECT_EQ(tile.cell(42).count(), 1);
  EXPECT_DOUBLE_EQ(tile.cell(42).elevation().mean(), 1.25);

  // Update cell 42 again
  auto res2 = tile.updateCell(42, 1.75);
  EXPECT_TRUE(res2.success);
  EXPECT_FALSE(res2.became_active);
  EXPECT_EQ(tile.activeCellCount(), 1);
  EXPECT_EQ(tile.cell(42).count(), 2);
  EXPECT_DOUBLE_EQ(tile.cell(42).elevation().mean(), 1.50);

  // Reset tile
  tile.reset();
  EXPECT_EQ(tile.activeCellCount(), 0);
  EXPECT_FALSE(tile.isCellActive(42));
  EXPECT_EQ(tile.cell(42).count(), 0);
}

TEST(EvidenceTileTest, TileBoundariesAndCoordinates) {
  // Coordinates around critical L0 tile boundaries:
  // -17, -16, -15, -1, 0, 15, 16, 17
  const std::array<int32_t, 8> coords = {-17, -16, -15, -1, 0, 15, 16, 17};

  for (int32_t x : coords) {
    for (int32_t y : coords) {
      const CellCoord cell_coord{.x = x, .y = y, .level = L0};
      const TileCoord tile_coord = cellToTileCoord(cell_coord);
      const auto tile_key = TileKey::fromTileCoord(tile_coord);
      ASSERT_TRUE(tile_key.has_value());

      const uint16_t local_idx = localCellIndex(x, y);
      EXPECT_LT(local_idx, EvidenceTile::CELL_COUNT);

      // Reconstruct cell key from tile and local index
      const auto reconstructed_key = cellKeyFromTileAndLocal(*tile_key, local_idx);
      ASSERT_TRUE(reconstructed_key.has_value());
      const CellCoord reconstructed_coord = reconstructed_key->decode();
      EXPECT_EQ(reconstructed_coord.x, x);
      EXPECT_EQ(reconstructed_coord.y, y);
      EXPECT_EQ(reconstructed_coord.level, L0);
    }
  }
}

TEST(EvidenceTileTest, EvidenceTileStructuralConsistency) {
  EvidenceTile tile;
  EXPECT_EQ(tile.activeCellCount(), 0);
  EXPECT_EQ(tile.activeCellCount(), tile.activeMask().count());

  // Activate several cells
  for (uint16_t idx : {0, 15, 16, 127, 255}) {
    const auto res = tile.updateCell(idx, static_cast<double>(idx));
    EXPECT_TRUE(res.success);
    EXPECT_TRUE(res.became_active);
    EXPECT_EQ(tile.activeCellCount(), tile.activeMask().count());
  }

  // Update existing cells again (should not change active count or mask count)
  for (uint16_t idx : {0, 15, 16, 127, 255}) {
    const auto res = tile.updateCell(idx, static_cast<double>(idx + 1));
    EXPECT_TRUE(res.success);
    EXPECT_FALSE(res.became_active);
    EXPECT_EQ(tile.activeCellCount(), tile.activeMask().count());
  }

  EXPECT_EQ(tile.activeCellCount(), 5);
  EXPECT_EQ(tile.activeMask().count(), 5);

  // Reset verifies both counters return to zero
  tile.reset();
  EXPECT_EQ(tile.activeCellCount(), 0);
  EXPECT_EQ(tile.activeMask().count(), 0);
}
