#include <gtest/gtest.h>

// Infrastructure sanity test verifying GoogleTest and C++20 toolchain compilation
TEST(SanityTest, BasicAssertions) {
  EXPECT_EQ(1 + 1, 2);
  EXPECT_TRUE(true);
}
