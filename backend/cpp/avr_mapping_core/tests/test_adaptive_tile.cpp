#include <gtest/gtest.h>

#include <bitset>
#include <type_traits>

#include "avr_mapping/adaptive_tile.hpp"

namespace avr::mapping {

TEST(AdaptiveTileTest, SizeAndAlignment) {
  EXPECT_EQ(sizeof(AdaptiveTile), 72u);
  EXPECT_EQ(alignof(AdaptiveTile), 8u);
  EXPECT_TRUE(std::is_nothrow_default_constructible_v<AdaptiveTile>);
}

TEST(AdaptiveTileTest, InitialStateIsEmptyAndClean) {
  AdaptiveTile tile;
  EXPECT_TRUE(tile.empty());
  EXPECT_EQ(tile.presentCount(), 0u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 0u);
  EXPECT_TRUE(tile.presentMask().none());
  EXPECT_TRUE(tile.leafMask().none());
  EXPECT_TRUE(tile.validateInvariants());

  for (uint16_t i = 0; i < AdaptiveTile::CELL_COUNT; ++i) {
    EXPECT_EQ(tile.nodeState(i), NodeState::ABSENT);
    EXPECT_FALSE(tile.isPresent(i));
    EXPECT_FALSE(tile.isLeaf(i));
    EXPECT_FALSE(tile.isInternal(i));
  }
}

TEST(AdaptiveTileTest, SetLeafTransitionsAndCounts) {
  AdaptiveTile tile;
  tile.setLeaf(42);

  EXPECT_FALSE(tile.empty());
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 1u);
  EXPECT_EQ(tile.internalCount(), 0u);
  EXPECT_EQ(tile.nodeState(42), NodeState::LEAF);
  EXPECT_TRUE(tile.isPresent(42));
  EXPECT_TRUE(tile.isLeaf(42));
  EXPECT_FALSE(tile.isInternal(42));
  EXPECT_TRUE(tile.validateInvariants());

  // Idempotent setLeaf
  tile.setLeaf(42);
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 1u);
  EXPECT_EQ(tile.internalCount(), 0u);
}

TEST(AdaptiveTileTest, SetInternalTransitionsAndCounts) {
  AdaptiveTile tile;
  tile.setInternal(100);

  EXPECT_FALSE(tile.empty());
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 1u);
  EXPECT_EQ(tile.nodeState(100), NodeState::INTERNAL);
  EXPECT_TRUE(tile.isPresent(100));
  EXPECT_FALSE(tile.isLeaf(100));
  EXPECT_TRUE(tile.isInternal(100));
  EXPECT_TRUE(tile.validateInvariants());

  // Transition from INTERNAL to LEAF
  tile.setLeaf(100);
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 1u);
  EXPECT_EQ(tile.internalCount(), 0u);
  EXPECT_EQ(tile.nodeState(100), NodeState::LEAF);

  // Transition from LEAF back to INTERNAL
  tile.setInternal(100);
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 1u);
  EXPECT_EQ(tile.nodeState(100), NodeState::INTERNAL);
}

TEST(AdaptiveTileTest, EraseNodeClearsMasksAndDecrementsCounts) {
  AdaptiveTile tile;
  tile.setLeaf(10);
  tile.setInternal(20);
  EXPECT_EQ(tile.presentCount(), 2u);
  EXPECT_EQ(tile.leafCount(), 1u);
  EXPECT_EQ(tile.internalCount(), 1u);

  // Erase leaf node
  tile.eraseNode(10);
  EXPECT_EQ(tile.presentCount(), 1u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 1u);
  EXPECT_EQ(tile.nodeState(10), NodeState::ABSENT);
  EXPECT_FALSE(tile.empty());

  // Erase internal node
  tile.eraseNode(20);
  EXPECT_EQ(tile.presentCount(), 0u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 0u);
  EXPECT_EQ(tile.nodeState(20), NodeState::ABSENT);
  EXPECT_TRUE(tile.empty());
  EXPECT_TRUE(tile.validateInvariants());

  // Erase on absent node is a no-op
  tile.eraseNode(30);
  EXPECT_EQ(tile.presentCount(), 0u);
}

TEST(AdaptiveTileTest, ResetRestoresCleanState) {
  AdaptiveTile tile;
  for (uint16_t i = 0; i < 50; ++i) {
    if (i % 2 == 0) {
      tile.setLeaf(i);
    } else {
      tile.setInternal(i);
    }
  }
  EXPECT_EQ(tile.presentCount(), 50u);
  EXPECT_EQ(tile.leafCount(), 25u);
  EXPECT_EQ(tile.internalCount(), 25u);

  tile.reset();
  EXPECT_TRUE(tile.empty());
  EXPECT_EQ(tile.presentCount(), 0u);
  EXPECT_EQ(tile.leafCount(), 0u);
  EXPECT_EQ(tile.internalCount(), 0u);
  EXPECT_TRUE(tile.presentMask().none());
  EXPECT_TRUE(tile.leafMask().none());
  EXPECT_TRUE(tile.validateInvariants());
}

// Section 9: AdaptiveTile index boundary safety
TEST(AdaptiveTileTest, IndexSafetyBoundaries) {
  AdaptiveTile tile;

  // Boundary 0 and 255 are valid
  tile.setLeaf(0);
  tile.setInternal(255);
  EXPECT_EQ(tile.nodeState(0), NodeState::LEAF);
  EXPECT_EQ(tile.nodeState(255), NodeState::INTERNAL);
  EXPECT_TRUE(tile.isPresent(0));
  EXPECT_TRUE(tile.isPresent(255));
  EXPECT_TRUE(tile.isLeaf(0));
  EXPECT_TRUE(tile.isInternal(255));

  // Boundary 256 and out-of-range indices must not corrupt or throw
  const std::vector<uint16_t> invalid_indices = {256, 257, 500, 1000, 65535};
  for (uint16_t idx : invalid_indices) {
    EXPECT_EQ(tile.nodeState(idx), NodeState::ABSENT);
    EXPECT_FALSE(tile.isPresent(idx));
    EXPECT_FALSE(tile.isLeaf(idx));
    EXPECT_FALSE(tile.isInternal(idx));

    // Mutators with invalid index must safely no-op
    tile.setLeaf(idx);
    tile.setInternal(idx);
    tile.eraseNode(idx);
  }

  // Verify tile remains valid and untouched by invalid indices
  EXPECT_EQ(tile.presentCount(), 2u);
  EXPECT_EQ(tile.leafCount(), 1u);
  EXPECT_EQ(tile.internalCount(), 1u);
  EXPECT_TRUE(tile.validateInvariants());
}

// Section 10: Exhaustive state transition tests
TEST(AdaptiveTileTest, ExhaustiveStateTransitionsAndInvariants) {
  AdaptiveTile tile;

  auto verify_invariants = [&tile]() {
    EXPECT_EQ(tile.presentCount(), tile.presentMask().count());
    EXPECT_EQ(tile.leafCount(), tile.leafMask().count());
    EXPECT_EQ(tile.internalCount(), tile.presentCount() - tile.leafCount());
    EXPECT_TRUE((tile.leafMask() & ~tile.presentMask()).none());
    EXPECT_TRUE(tile.validateInvariants());
  };

  // 1. ABSENT -> LEAF
  tile.setLeaf(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::LEAF);
  verify_invariants();

  // 2. LEAF -> INTERNAL
  tile.setInternal(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::INTERNAL);
  verify_invariants();

  // 3. INTERNAL -> LEAF
  tile.setLeaf(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::LEAF);
  verify_invariants();

  // 4. LEAF -> ABSENT
  tile.eraseNode(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::ABSENT);
  verify_invariants();

  // 5. ABSENT -> INTERNAL -> ABSENT
  tile.setInternal(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::INTERNAL);
  verify_invariants();

  tile.eraseNode(42);
  EXPECT_EQ(tile.nodeState(42), NodeState::ABSENT);
  verify_invariants();
}

}  // namespace avr::mapping
