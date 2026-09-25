#include <gtest/gtest.h>

#include <vector>

#include "avr_mapping/tile_pool.hpp"

namespace avr::mapping {

TEST(TilePoolTest, FreshPoolInitialStats) {
  TilePool pool(64);
  EXPECT_EQ(pool.capacity(), 0u);
  EXPECT_EQ(pool.inUse(), 0u);
  EXPECT_EQ(pool.freeCount(), 0u);
  EXPECT_EQ(pool.blockCount(), 0u);
  EXPECT_EQ(pool.blockSize(), 64u);
}

TEST(TilePoolTest, FirstBlockAcquisitionAndStatsInvariant) {
  TilePool pool(64);
  AdaptiveTile* t1 = pool.acquireTile();
  ASSERT_NE(t1, nullptr);
  EXPECT_TRUE(t1->empty());
  EXPECT_EQ(pool.blockCount(), 1u);
  EXPECT_EQ(pool.capacity(), 64u);
  EXPECT_EQ(pool.inUse(), 1u);
  EXPECT_EQ(pool.freeCount(), 63u);
  EXPECT_EQ(pool.capacity(), pool.inUse() + pool.freeCount());
  EXPECT_TRUE(pool.containsPointer(t1));

  AdaptiveTile stack_tile;
  EXPECT_FALSE(pool.containsPointer(&stack_tile));
  EXPECT_FALSE(pool.containsPointer(nullptr));
}

TEST(TilePoolTest, ReleaseAndReacquireStateCleanliness) {
  TilePool pool(32);
  AdaptiveTile* t1 = pool.acquireTile();
  t1->setLeaf(5);
  t1->setInternal(10);
  EXPECT_EQ(t1->presentCount(), 2u);

  pool.releaseTile(t1);
  EXPECT_EQ(pool.inUse(), 0u);
  EXPECT_EQ(pool.freeCount(), 32u);
  EXPECT_EQ(pool.capacity(), 32u);

  // Reacquired tile must be clean/reset
  AdaptiveTile* t2 = pool.acquireTile();
  EXPECT_EQ(t2, t1);  // LIFO free list reuses last freed pointer
  EXPECT_TRUE(t2->empty());
  EXPECT_EQ(t2->presentCount(), 0u);
  EXPECT_EQ(pool.inUse(), 1u);
  EXPECT_EQ(pool.freeCount(), 31u);
}

TEST(TilePoolTest, PointerStabilityAcrossBlockGrowth) {
  TilePool pool(32);  // 32 tiles per block

  std::vector<AdaptiveTile*> acquired;
  acquired.reserve(100);

  // Acquire first block (32 tiles)
  for (size_t i = 0; i < 32; ++i) {
    AdaptiveTile* t = pool.acquireTile();
    t->setLeaf(static_cast<uint16_t>(i));
    acquired.push_back(t);
  }
  EXPECT_EQ(pool.blockCount(), 1u);
  EXPECT_EQ(pool.capacity(), 32u);
  EXPECT_EQ(pool.inUse(), 32u);
  EXPECT_EQ(pool.freeCount(), 0u);

  // Trigger allocation of block 2 (next 32 tiles) and block 3
  for (size_t i = 32; i < 70; ++i) {
    AdaptiveTile* t = pool.acquireTile();
    t->setLeaf(static_cast<uint16_t>(i % 256));
    acquired.push_back(t);
  }
  EXPECT_EQ(pool.blockCount(), 3u);
  EXPECT_EQ(pool.capacity(), 96u);
  EXPECT_EQ(pool.inUse(), 70u);
  EXPECT_EQ(pool.freeCount(), 26u);
  EXPECT_EQ(pool.capacity(), pool.inUse() + pool.freeCount());

  // Verify all previously acquired pointers remain 100% stable and intact
  for (size_t i = 0; i < 70; ++i) {
    EXPECT_TRUE(pool.containsPointer(acquired[i]));
    EXPECT_TRUE(acquired[i]->isLeaf(static_cast<uint16_t>(i % 256)));
  }
}

TEST(TilePoolTest, ResetReclaimsAllCapacityForReuse) {
  TilePool pool(32);
  for (size_t i = 0; i < 50; ++i) {
    AdaptiveTile* t = pool.acquireTile();
    t->setLeaf(1);
  }
  EXPECT_EQ(pool.blockCount(), 2u);
  EXPECT_EQ(pool.capacity(), 64u);
  EXPECT_EQ(pool.inUse(), 50u);

  pool.reset();
  EXPECT_EQ(pool.blockCount(), 2u);
  EXPECT_EQ(pool.capacity(), 64u);
  EXPECT_EQ(pool.inUse(), 0u);
  EXPECT_EQ(pool.freeCount(), 64u);

  // Next acquisition does not allocate a new block
  AdaptiveTile* t = pool.acquireTile();
  EXPECT_TRUE(t->empty());
  EXPECT_EQ(pool.blockCount(), 2u);
  EXPECT_EQ(pool.inUse(), 1u);
  EXPECT_EQ(pool.freeCount(), 63u);
}

// Section 4 & 7: TilePool ownership safety and double release prevention
TEST(TilePoolTest, OwnershipSafetyAndDoubleReleaseProtection) {
  TilePool pool(32);
  AdaptiveTile* t1 = pool.acquireTile();
  ASSERT_NE(t1, nullptr);
  EXPECT_TRUE(pool.owns(t1));
  EXPECT_TRUE(pool.isAcquired(t1));

  // Release valid acquired tile
  EXPECT_TRUE(pool.releaseTile(t1));
  EXPECT_TRUE(pool.owns(t1));
  EXPECT_FALSE(pool.isAcquired(t1));

  // Double release must be rejected safely
  EXPECT_FALSE(pool.releaseTile(t1));
  EXPECT_EQ(pool.inUse(), 0u);
  EXPECT_EQ(pool.freeCount(), 32u);

  // nullptr release must be rejected safely
  EXPECT_FALSE(pool.releaseTile(nullptr));

  // Foreign tile from another pool must be rejected safely
  TilePool other_pool(32);
  AdaptiveTile* foreign_tile = other_pool.acquireTile();
  EXPECT_FALSE(pool.owns(foreign_tile));
  EXPECT_FALSE(pool.releaseTile(foreign_tile));

  // Stack allocated tile must be rejected safely
  AdaptiveTile stack_tile;
  EXPECT_FALSE(pool.owns(&stack_tile));
  EXPECT_FALSE(pool.releaseTile(&stack_tile));
}

// Section 5: Zero block capacity normalization
TEST(TilePoolTest, ZeroBlockCapacityNormalization) {
  TilePool pool(0);  // 0 should be normalized to DEFAULT_BLOCK_CAPACITY (64)
  EXPECT_EQ(pool.blockSize(), TilePool::DEFAULT_BLOCK_CAPACITY);
  EXPECT_EQ(pool.capacity(), 0u);

  AdaptiveTile* t = pool.acquireTile();
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(pool.capacity(), TilePool::DEFAULT_BLOCK_CAPACITY);
  EXPECT_EQ(pool.inUse(), 1u);
  EXPECT_EQ(pool.freeCount(), TilePool::DEFAULT_BLOCK_CAPACITY - 1);
}

// Section 6: TilePool reuse guarantees clean state
TEST(TilePoolTest, ReuseGuaranteesCleanState) {
  TilePool pool(16);
  AdaptiveTile* t = pool.acquireTile();
  t->setLeaf(1);
  t->setInternal(2);
  t->setLeaf(3);
  EXPECT_FALSE(t->empty());

  pool.releaseTile(t);

  AdaptiveTile* t_reused = pool.acquireTile();
  EXPECT_EQ(t_reused, t);
  EXPECT_TRUE(t_reused->empty());
  EXPECT_EQ(t_reused->presentCount(), 0u);
  EXPECT_EQ(t_reused->leafCount(), 0u);
  EXPECT_EQ(t_reused->internalCount(), 0u);
  EXPECT_TRUE(t_reused->presentMask().none());
  EXPECT_TRUE(t_reused->leafMask().none());
  EXPECT_TRUE(t_reused->isAcquired());
}

// Section 8: Compile-time check for non-copyable and non-movable semantics
TEST(TilePoolTest, NonCopyableNonMovableContract) {
  static_assert(!std::is_copy_constructible_v<TilePool>);
  static_assert(!std::is_copy_assignable_v<TilePool>);
  static_assert(!std::is_move_constructible_v<TilePool>);
  static_assert(!std::is_move_assignable_v<TilePool>);
  SUCCEED();
}

// Milestone 3.1: Targeted portable foreign pointer ownership rejection
TEST(TilePoolTest, PortableForeignPointerRejection) {
  TilePool pool(16);
  AdaptiveTile* valid_tile = pool.acquireTile();
  ASSERT_NE(valid_tile, nullptr);
  EXPECT_TRUE(pool.owns(valid_tile));
  EXPECT_TRUE(pool.isAcquired(valid_tile));

  // 1. Separate TilePool instance tile
  TilePool other_pool(16);
  AdaptiveTile* foreign_tile = other_pool.acquireTile();
  ASSERT_NE(foreign_tile, nullptr);
  EXPECT_FALSE(pool.owns(foreign_tile));
  EXPECT_FALSE(pool.isAcquired(foreign_tile));
  EXPECT_FALSE(pool.releaseTile(foreign_tile));

  // 2. Independent heap-allocated tile
  auto heap_tile = std::make_unique<AdaptiveTile>();
  EXPECT_FALSE(pool.owns(heap_tile.get()));
  EXPECT_FALSE(pool.isAcquired(heap_tile.get()));
  EXPECT_FALSE(pool.releaseTile(heap_tile.get()));

  // 3. Stack-allocated tile
  AdaptiveTile stack_tile;
  EXPECT_FALSE(pool.owns(&stack_tile));
  EXPECT_FALSE(pool.isAcquired(&stack_tile));
  EXPECT_FALSE(pool.releaseTile(&stack_tile));

  // 4. nullptr
  EXPECT_FALSE(pool.owns(nullptr));
  EXPECT_FALSE(pool.isAcquired(nullptr));
  EXPECT_FALSE(pool.releaseTile(nullptr));

  // Verify pool state remains completely unaffected by rejected foreign releases
  EXPECT_EQ(pool.inUse(), 1u);
  EXPECT_EQ(pool.freeCount(), 15u);
  EXPECT_EQ(pool.capacity(), 16u);

  // 5. Valid release succeeds
  EXPECT_TRUE(pool.releaseTile(valid_tile));
  EXPECT_EQ(pool.inUse(), 0u);
  EXPECT_EQ(pool.freeCount(), 16u);
}

}  // namespace avr::mapping
