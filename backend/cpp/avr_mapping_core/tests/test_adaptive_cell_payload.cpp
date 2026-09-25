#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "avr_mapping/adaptive_cell_payload.hpp"
#include "avr_mapping/resolution_decision.hpp"

namespace avr::mapping {
namespace {

TEST(AdaptiveCellPayloadTest, DefaultPayloadIsEmpty) {
  const AdaptiveCellPayload payload{};
  EXPECT_FALSE(payload.hasObservations());
  EXPECT_EQ(payload.observationCount(), 0ULL);
  EXPECT_EQ(payload.firstSeenFrame(), 0ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 0ULL);
  EXPECT_EQ(payload.observedFrameCount(), 0ULL);
  EXPECT_TRUE(payload.lastUpdateReasons().empty());
}

TEST(AdaptiveCellPayloadTest, ValueConstructor) {
  ElevationStats elev{};
  elev.update(1.5);
  elev.update(2.5);

  const ResolutionReasonBits reasons(ResolutionReason::DISTANCE_BASELINE);
  const AdaptiveCellPayload payload(elev, 10, 20, 2, reasons);

  EXPECT_TRUE(payload.hasObservations());
  EXPECT_EQ(payload.observationCount(), 2ULL);
  EXPECT_EQ(payload.firstSeenFrame(), 10ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 20ULL);
  EXPECT_EQ(payload.observedFrameCount(), 2ULL);
  EXPECT_TRUE(payload.lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));
}

TEST(AdaptiveCellPayloadTest, FirstFusionSuccess) {
  AdaptiveCellPayload payload{};

  ElevationStats frame1{};
  frame1.update(2.0);
  frame1.update(4.0);

  const ResolutionReasonBits reasons(ResolutionReason::GEOMETRY_COMPLEXITY);
  const bool ok = payload.tryFuse(frame1, 10, reasons);
  EXPECT_TRUE(ok);

  EXPECT_TRUE(payload.hasObservations());
  EXPECT_EQ(payload.observationCount(), 2ULL);
  EXPECT_EQ(payload.firstSeenFrame(), 10ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 10ULL);
  EXPECT_EQ(payload.observedFrameCount(), 1ULL);
  EXPECT_TRUE(payload.lastUpdateReasons().has(ResolutionReason::GEOMETRY_COMPLEXITY));
  EXPECT_DOUBLE_EQ(payload.elevation().mean(), 3.0);
}

TEST(AdaptiveCellPayloadTest, SubsequentFusionMonotonic) {
  AdaptiveCellPayload payload{};

  ElevationStats frame1{};
  frame1.update(2.0);
  EXPECT_TRUE(payload.tryFuse(frame1, 10, ResolutionReasonBits(ResolutionReason::DISTANCE_BASELINE)));

  ElevationStats frame2{};
  frame2.update(4.0);
  EXPECT_TRUE(payload.tryFuse(frame2, 20, ResolutionReasonBits(ResolutionReason::UNCERTAINTY)));

  EXPECT_EQ(payload.observationCount(), 2ULL);
  EXPECT_EQ(payload.firstSeenFrame(), 10ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 20ULL);
  EXPECT_EQ(payload.observedFrameCount(), 2ULL);
  EXPECT_DOUBLE_EQ(payload.elevation().mean(), 3.0);

  // Reasons represent the most recent update context, not cumulative OR
  EXPECT_TRUE(payload.lastUpdateReasons().has(ResolutionReason::UNCERTAINTY));
  EXPECT_FALSE(payload.lastUpdateReasons().has(ResolutionReason::DISTANCE_BASELINE));
}

TEST(AdaptiveCellPayloadTest, DuplicateFrameRejected) {
  AdaptiveCellPayload payload{};

  ElevationStats frame1{};
  frame1.update(2.0);
  EXPECT_TRUE(payload.tryFuse(frame1, 42));

  // Same frame_seq again must be rejected
  ElevationStats frame1_dup{};
  frame1_dup.update(5.0);
  EXPECT_FALSE(payload.tryFuse(frame1_dup, 42));

  // State remains completely unmodified
  EXPECT_EQ(payload.observationCount(), 1ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 42ULL);
  EXPECT_EQ(payload.observedFrameCount(), 1ULL);
  EXPECT_DOUBLE_EQ(payload.elevation().mean(), 2.0);
}

TEST(AdaptiveCellPayloadTest, OutOfOrderFrameRejected) {
  AdaptiveCellPayload payload{};

  ElevationStats frame1{};
  frame1.update(2.0);
  EXPECT_TRUE(payload.tryFuse(frame1, 100));

  // Older frame_seq rejected
  ElevationStats frame_older{};
  frame_older.update(10.0);
  EXPECT_FALSE(payload.tryFuse(frame_older, 50));

  EXPECT_EQ(payload.observationCount(), 1ULL);
  EXPECT_EQ(payload.lastSeenFrame(), 100ULL);
  EXPECT_DOUBLE_EQ(payload.elevation().mean(), 2.0);
}

TEST(AdaptiveCellPayloadTest, EmptyFrameElevationRejected) {
  AdaptiveCellPayload payload{};

  const ElevationStats empty_frame{};
  EXPECT_FALSE(payload.tryFuse(empty_frame, 10));
  EXPECT_FALSE(payload.hasObservations());
}

TEST(AdaptiveCellPayloadTest, ObservedFrameCountOverflowGuarded) {
  ElevationStats elev{};
  elev.update(1.0);

  // Construct payload with observed_frame_count at UINT64_MAX
  AdaptiveCellPayload payload(
      elev, 1, 10, std::numeric_limits<uint64_t>::max(), ResolutionReasonBits{});

  ElevationStats frame2{};
  frame2.update(2.0);

  EXPECT_FALSE(payload.tryFuse(frame2, 20));
  EXPECT_EQ(payload.observedFrameCount(), std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(payload.lastSeenFrame(), 10ULL);
  EXPECT_EQ(payload.observationCount(), 1ULL);
  EXPECT_DOUBLE_EQ(payload.elevation().mean(), 1.0);
}

TEST(AdaptiveCellPayloadTest, PerLeafFrameOrderSequence) {
  AdaptiveCellPayload payload{};

  ElevationStats f10{};
  f10.update(1.0);
  EXPECT_TRUE(payload.tryFuse(f10, 10));
  EXPECT_EQ(payload.lastSeenFrame(), 10ULL);

  ElevationStats f42{};
  f42.update(2.0);
  EXPECT_TRUE(payload.tryFuse(f42, 42));
  EXPECT_EQ(payload.lastSeenFrame(), 42ULL);

  // Duplicate 42 rejected without mutation
  ElevationStats f42_dup{};
  f42_dup.update(99.0);
  EXPECT_FALSE(payload.tryFuse(f42_dup, 42));
  EXPECT_EQ(payload.lastSeenFrame(), 42ULL);
  EXPECT_EQ(payload.observationCount(), 2ULL);

  // Out of order 20 rejected without mutation
  ElevationStats f20{};
  f20.update(99.0);
  EXPECT_FALSE(payload.tryFuse(f20, 20));
  EXPECT_EQ(payload.lastSeenFrame(), 42ULL);
  EXPECT_EQ(payload.observationCount(), 2ULL);

  // Future frame 105 succeeds
  ElevationStats f105{};
  f105.update(3.0);
  EXPECT_TRUE(payload.tryFuse(f105, 105));
  EXPECT_EQ(payload.lastSeenFrame(), 105ULL);
  EXPECT_EQ(payload.observationCount(), 3ULL);
  EXPECT_EQ(payload.observedFrameCount(), 3ULL);
}

TEST(AdaptiveCellPayloadTest, FrameCountVsPointCount) {
  AdaptiveCellPayload payload{};

  ElevationStats frame1{};
  for (int i = 0; i < 100; ++i) {
    frame1.update(2.0);
  }
  EXPECT_TRUE(payload.tryFuse(frame1, 10));

  ElevationStats frame2{};
  frame2.update(4.0);
  EXPECT_TRUE(payload.tryFuse(frame2, 20));

  EXPECT_EQ(payload.elevation().count(), 101ULL);
  EXPECT_EQ(payload.observationCount(), 101ULL);
  EXPECT_EQ(payload.observedFrameCount(), 2ULL);
}

TEST(AdaptiveCellPayloadTest, LayoutContract) {
  EXPECT_EQ(sizeof(ResolutionReasonBits), 2);
  EXPECT_EQ(sizeof(AdaptiveCellPayload), 72);
  EXPECT_EQ(alignof(AdaptiveCellPayload), 8);
}

}  // namespace
}  // namespace avr::mapping
