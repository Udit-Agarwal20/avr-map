#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_set>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_hysteresis.hpp"
#include "avr_mapping/resolution_hysteresis_config.hpp"
#include "avr_mapping/resolution_hysteresis_result.hpp"
#include "avr_mapping/resolution_hysteresis_state.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

using namespace avr::mapping;

TEST(ResolutionHysteresisStressTest, HundredThousandObservationsDeterministicStressAndRepeatability) {
  constexpr size_t NUM_OBSERVATIONS = 120000;
  constexpr size_t NUM_REGIONS = 12000;  // Exceeds 10,000 distinct CellKeys
  constexpr uint32_t THRESHOLD = 5;

  // Generate distinct keys across negative and positive coordinates and all four levels
  std::vector<avr::spatial::CellKey> test_keys;
  test_keys.reserve(NUM_REGIONS);
  for (size_t i = 0; i < NUM_REGIONS; ++i) {
    const int32_t cx = static_cast<int32_t>(i % 200) - 100;
    const int32_t cy = static_cast<int32_t>(i / 200) - 100;
    const uint8_t level = static_cast<uint8_t>(i % 4);
    test_keys.push_back(*avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = cx, .y = cy, .level = level}));
  }

  std::unordered_set<avr::spatial::CellKey> unique_keys(test_keys.begin(), test_keys.end());
  ASSERT_GE(unique_keys.size(), 10000U);

  const auto unknown_dummy_key = *avr::spatial::CellKey::fromCellCoord(avr::spatial::CellCoord{.x = 99999, .y = 99999, .level = 0});

  // Deterministic PRNG (Linear Congruential Generator)
  uint64_t rng = 0xDEADBEEF12345678ULL;
  auto next_random = [&rng]() -> uint64_t {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng;
  };

  struct StreamResults {
    std::vector<HysteresisAction> actions{};
    std::vector<bool> acks{};
  };

  const auto run_stream = [&](ResolutionHysteresisController& ctrl) -> StreamResults {
    StreamResults sr;
    sr.actions.reserve(NUM_OBSERVATIONS);
    sr.acks.reserve(NUM_OBSERVATIONS / 4);

    // Track state in test driver
    std::vector<bool> region_initialized(NUM_REGIONS, false);
    std::vector<uint64_t> region_last_frame(NUM_REGIONS, 0);

    for (size_t step = 0; step < NUM_OBSERVATIONS; ++step) {
      const size_t region_idx = next_random() % NUM_REGIONS;
      const auto& region = test_keys[region_idx];
      const uint8_t cur_level = region.level;

      const uint64_t op_type = next_random() % 100;

      // 1. Invalid ack on unknown region periodically
      if (op_type == 0) {
        bool ack_un = ctrl.acknowledgeCoarsenResult(unknown_dummy_key, CoarsenAttemptOutcome::CANCELLED);
        EXPECT_FALSE(ack_un);
        sr.acks.push_back(ack_un);
      }

      // 2. First observation frame zero test or duplicate/out-of-order test
      uint64_t frame_seq = 0;
      if (!region_initialized[region_idx]) {
        // Some region lifecycles begin explicitly at frame_seq = 0
        if ((next_random() % 3) == 0) {
          frame_seq = 0;
        } else {
          frame_seq = next_random() % 20;
        }
        region_initialized[region_idx] = true;
        region_last_frame[region_idx] = frame_seq;
      } else {
        if (op_type == 1 && region_last_frame[region_idx] == 0) {
          // Attempt duplicate frame 0
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, 0, dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        if (op_type == 2 && region_last_frame[region_idx] > 0) {
          // Attempt duplicate non-zero frame
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, region_last_frame[region_idx], dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        if (op_type == 3 && region_last_frame[region_idx] > 10) {
          // Attempt out-of-order frame
          ResolutionDecision dec{.baseline_level = avr::spatial::L3, .requested_level = cur_level};
          auto res = ctrl.observe(region, region_last_frame[region_idx] - 5, dec);
          EXPECT_EQ(res.action, HysteresisAction::INVALID_OBSERVATION);
          sr.actions.push_back(res.action);
          continue;
        }

        // Strictly increasing valid frame sequence
        frame_seq = region_last_frame[region_idx] + 1 + (next_random() % 10);
        region_last_frame[region_idx] = frame_seq;
      }

      // 3. Invalid ack on known non-pending region periodically
      if (op_type == 4) {
        const auto* st = ctrl.findRegionState(region);
        if (st != nullptr && !st->coarsen_action_pending) {
          bool bad_ack = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          EXPECT_FALSE(bad_ack);
          sr.acks.push_back(bad_ack);
        }
      }

      // Select requested level (finer, equal, or coarser)
      uint8_t req_level = static_cast<uint8_t>(next_random() % 4);

      ResolutionReasonBits reasons{};
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::UNCERTAINTY);
      if ((next_random() % 2) != 0) reasons.set(ResolutionReason::GEOMETRY_COMPLEXITY);

      ResolutionDecision dec{
          .baseline_level = avr::spatial::L3,
          .requested_level = req_level,
          .evidence_state = EvidenceState::SUFFICIENT,
          .reasons = reasons,
      };

      auto res = ctrl.observe(region, frame_seq, dec);
      sr.actions.push_back(res.action);

      // Verify immediate invariant after observation
      if (res.action == HysteresisAction::COARSEN_READY) {
        EXPECT_EQ(res.stable_coarsen_count, THRESHOLD);
        const uint64_t ack_choice = next_random() % 3;
        if (ack_choice == 0) {
          // APPLIED
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::APPLIED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
          region_initialized[region_idx] = false;
          region_last_frame[region_idx] = 0;
        } else if (ack_choice == 1) {
          // BLOCKED_HOLD_FINE (frame ordering preserved!)
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::BLOCKED_HOLD_FINE);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        } else {
          // CANCELLED (frame ordering preserved!)
          bool ok = ctrl.acknowledgeCoarsenResult(region, CoarsenAttemptOutcome::CANCELLED);
          EXPECT_TRUE(ok);
          sr.acks.push_back(ok);
        }
      } else if (res.action == HysteresisAction::REFINE_NOW || res.action == HysteresisAction::HOLD_CURRENT) {
        EXPECT_EQ(res.stable_coarsen_count, 0U);
      } else if (res.action == HysteresisAction::COARSEN_PENDING) {
        EXPECT_LT(res.stable_coarsen_count, THRESHOLD);
      }

      // Random eraseRegion lifecycle test
      if (op_type == 50) {
        ctrl.eraseRegion(region);
        region_initialized[region_idx] = false;
        region_last_frame[region_idx] = 0;
      }
    }

    return sr;
  };

  // Run controller 1
  uint64_t initial_seed = rng;
  auto ctrl1 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results1 = run_stream(ctrl1);

  // Invariant verification on final state of ctrl1
  const auto snap1 = ctrl1.stateSnapshot();
  for (size_t i = 0; i < snap1.size(); ++i) {
    const auto& s = snap1[i];
    EXPECT_TRUE(s.has_last_frame);
    EXPECT_LE(s.stable_coarsen_count, THRESHOLD);
    if (s.coarsen_action_pending) {
      EXPECT_EQ(s.stable_coarsen_count, THRESHOLD);
      EXPECT_EQ(s.pending_target_level, s.region.level + 1);
    } else {
      EXPECT_EQ(s.pending_target_level, 0U);
    }
    if (i + 1 < snap1.size()) {
      EXPECT_TRUE(snap1[i].region < snap1[i + 1].region);
    }
  }

  // Run controller 2 with identical seed stream
  auto ctrl2 = *ResolutionHysteresisController::create(ResolutionHysteresisConfig{.coarsen_stable_observations = THRESHOLD});
  rng = initial_seed;
  const auto results2 = run_stream(ctrl2);

  // Repeatability verification: exact bit-for-bit equivalence
  EXPECT_EQ(results1.actions.size(), NUM_OBSERVATIONS);
  EXPECT_EQ(results1.actions, results2.actions);
  EXPECT_EQ(results1.acks, results2.acks);
  EXPECT_EQ(ctrl1.trackedRegionCount(), ctrl2.trackedRegionCount());
  EXPECT_EQ(ctrl1.stateSnapshot(), ctrl2.stateSnapshot());
}
