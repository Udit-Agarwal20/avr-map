#pragma once

#include <cstdint>
#include <vector>

namespace avr::mapping {

/**
 * @brief Canonical AVR semantic classification taxonomy for LiDAR point returns.
 */
enum class SemanticClass : uint8_t {
  UNKNOWN = 0,
  GROUND = 1,
  ROAD = 2,
  VEGETATION = 3,
  BUILDING = 4,
  VEHICLE = 5,
  PEDESTRIAN = 6,
  OBSTACLE_OTHER = 7,
};

[[nodiscard]] constexpr bool isValidSemanticClass(SemanticClass sc) noexcept {
  switch (sc) {
    case SemanticClass::UNKNOWN:
    case SemanticClass::GROUND:
    case SemanticClass::ROAD:
    case SemanticClass::VEGETATION:
    case SemanticClass::BUILDING:
    case SemanticClass::VEHICLE:
    case SemanticClass::PEDESTRIAN:
    case SemanticClass::OBSTACLE_OTHER:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr const char* semanticClassToString(SemanticClass sc) noexcept {
  switch (sc) {
    case SemanticClass::UNKNOWN:
      return "UNKNOWN";
    case SemanticClass::GROUND:
      return "GROUND";
    case SemanticClass::ROAD:
      return "ROAD";
    case SemanticClass::VEGETATION:
      return "VEGETATION";
    case SemanticClass::BUILDING:
      return "BUILDING";
    case SemanticClass::VEHICLE:
      return "VEHICLE";
    case SemanticClass::PEDESTRIAN:
      return "PEDESTRIAN";
    case SemanticClass::OBSTACLE_OTHER:
      return "OBSTACLE_OTHER";
  }
  return "UNKNOWN";
}

/**
 * @brief Independent motion state classification for LiDAR point returns.
 */
enum class MotionClassification : uint8_t {
  UNKNOWN = 0,
  STATIC = 1,
  CANDIDATE = 2,
  CONFIRMED_DYNAMIC = 3,
};

[[nodiscard]] constexpr bool isValidMotionClassification(MotionClassification mc) noexcept {
  switch (mc) {
    case MotionClassification::UNKNOWN:
    case MotionClassification::STATIC:
    case MotionClassification::CANDIDATE:
    case MotionClassification::CONFIRMED_DYNAMIC:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr const char* motionClassificationToString(MotionClassification mc) noexcept {
  switch (mc) {
    case MotionClassification::UNKNOWN:
      return "UNKNOWN";
    case MotionClassification::STATIC:
      return "STATIC";
    case MotionClassification::CANDIDATE:
      return "CANDIDATE";
    case MotionClassification::CONFIRMED_DYNAMIC:
      return "CONFIRMED_DYNAMIC";
  }
  return "UNKNOWN";
}

/**
 * @brief Point-level prediction from a semantic perception provider.
 *
 * Semantic class and motion classification carry independent confidence scores.
 * source_point_index refers to the original index in the raw_points span passed to processFrame.
 */
struct SemanticObservation {
  uint32_t source_point_index{0};

  SemanticClass semantic_class{SemanticClass::UNKNOWN};
  float semantic_confidence{0.0f};

  MotionClassification motion_state{MotionClassification::UNKNOWN};
  float motion_confidence{0.0f};

  constexpr bool operator==(const SemanticObservation&) const noexcept = default;
};

/**
 * @brief Result bundle emitted by a SemanticProvider for a single frame.
 */
struct SemanticFrameResult {
  uint64_t frame_seq{0};
  std::vector<SemanticObservation> observations{};
};

}  // namespace avr::mapping
