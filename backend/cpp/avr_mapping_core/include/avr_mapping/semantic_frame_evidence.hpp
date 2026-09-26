#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "avr_mapping/resolution_decision.hpp"
#include "avr_mapping/resolution_policy.hpp"
#include "avr_mapping/semantic_policy_config.hpp"
#include "avr_mapping/semantic_types.hpp"
#include "avr_spatial/cell_key.hpp"
#include "avr_spatial/coordinate.hpp"

namespace avr::mapping {

/**
 * @brief Ephemeral current-frame spatial aggregation structure for semantic and motion evidence.
 *
 * Exists only during a single processFrame cycle. Does NOT store persistent map data.
 * Aggregates point-level semantic classifications and motion predictions into dyadic
 * regions (L0-L3) to evaluate localized SemanticRelevance and DynamicRelevance for M4.
 */
class SemanticFrameEvidence {
 public:
  struct Entry {
    uint32_t source_point_index{0};
    avr::spatial::GridCoord2D base_coord{};
    SemanticClass semantic_class{SemanticClass::UNKNOWN};
    float semantic_confidence{0.0f};
    MotionClassification motion_state{MotionClassification::UNKNOWN};
    float motion_confidence{0.0f};
    SemanticRelevance semantic_relevance{SemanticRelevance::NONE};
    DynamicRelevance dynamic_relevance{DynamicRelevance::NONE};
  };

  SemanticFrameEvidence() = default;

  /**
   * @brief Resets and clears all internal spatial buckets.
   */
  void reset() noexcept {
    buckets_.clear();
    entry_count_ = 0;
  }

  /**
   * @brief Adds a validated, geometrically quantizable observation to the spatial index.
   *
   * Maps point-level semantic and motion states to qualifying SemanticRelevance and DynamicRelevance
   * using the provided policy configuration thresholds.
   */
  void addObservation(
      uint32_t source_point_index,
      const avr::spatial::GridCoord2D& base_coord,
      SemanticClass sc,
      float s_conf,
      MotionClassification mc,
      float m_conf,
      const SemanticPolicyConfig& config) {
    // 1. Semantic relevance mapping (after confidence gating)
    SemanticRelevance s_rel = SemanticRelevance::NONE;
    if (s_conf >= config.min_semantic_confidence) {
      switch (sc) {
        case SemanticClass::PEDESTRIAN:
          s_rel = config.pedestrian_safety_critical
                      ? SemanticRelevance::SAFETY_CRITICAL
                      : SemanticRelevance::ELEVATED;
          break;
        case SemanticClass::VEHICLE:
          s_rel = config.vehicle_safety_critical
                      ? SemanticRelevance::SAFETY_CRITICAL
                      : SemanticRelevance::ELEVATED;
          break;
        case SemanticClass::BUILDING:
        case SemanticClass::OBSTACLE_OTHER:
        case SemanticClass::VEGETATION:
          s_rel = SemanticRelevance::ELEVATED;
          break;
        case SemanticClass::GROUND:
        case SemanticClass::ROAD:
        case SemanticClass::UNKNOWN:
          s_rel = SemanticRelevance::NONE;
          break;
      }
    }

    // 2. Dynamic relevance mapping (after independent confidence gating)
    DynamicRelevance d_rel = DynamicRelevance::NONE;
    if (m_conf >= config.min_motion_confidence) {
      switch (mc) {
        case MotionClassification::CONFIRMED_DYNAMIC:
          d_rel = DynamicRelevance::CONFIRMED;
          break;
        case MotionClassification::CANDIDATE:
          d_rel = DynamicRelevance::CANDIDATE;
          break;
        case MotionClassification::STATIC:
        case MotionClassification::UNKNOWN:
          d_rel = DynamicRelevance::NONE;
          break;
      }
    }

    const auto l3_key_opt = avr::spatial::CellKey::fromGridCoord(base_coord, avr::spatial::L3);
    if (!l3_key_opt.has_value()) {
      return;
    }

    buckets_[*l3_key_opt].push_back(Entry{
        .source_point_index = source_point_index,
        .base_coord = base_coord,
        .semantic_class = sc,
        .semantic_confidence = s_conf,
        .motion_state = mc,
        .motion_confidence = m_conf,
        .semantic_relevance = s_rel,
        .dynamic_relevance = d_rel,
    });
    ++entry_count_;
  }

  /**
   * @brief Evaluates conservative localized SemanticRelevance across the footprint of region.
   *
   * Returns the maximum qualifying SemanticRelevance:
   * NONE < ELEVATED < SAFETY_CRITICAL.
   */
  [[nodiscard]] SemanticRelevance evaluateSemanticRelevance(
      const avr::spatial::CellKey& region) const noexcept {
    if (!avr::spatial::isValidLevel(region.level)) {
      return SemanticRelevance::NONE;
    }

    avr::spatial::CellKey l3_key = region;
    while (l3_key.level < avr::spatial::L3) {
      const auto parent_opt = avr::spatial::parentOf(l3_key);
      if (!parent_opt.has_value()) {
        return SemanticRelevance::NONE;
      }
      l3_key = *parent_opt;
    }

    const auto it = buckets_.find(l3_key);
    if (it == buckets_.end()) {
      return SemanticRelevance::NONE;
    }

    SemanticRelevance max_rel = SemanticRelevance::NONE;
    for (const auto& entry : it->second) {
      if (region.level < avr::spatial::L3) {
        const auto key_opt = avr::spatial::CellKey::fromGridCoord(entry.base_coord, region.level);
        if (!key_opt.has_value() || *key_opt != region) {
          continue;
        }
      }
      if (static_cast<uint8_t>(entry.semantic_relevance) > static_cast<uint8_t>(max_rel)) {
        max_rel = entry.semantic_relevance;
        if (max_rel == SemanticRelevance::SAFETY_CRITICAL) {
          return max_rel;  // Maximum possible value reached
        }
      }
    }
    return max_rel;
  }

  /**
   * @brief Evaluates conservative localized DynamicRelevance across the footprint of region.
   *
   * Returns the maximum qualifying DynamicRelevance:
   * NONE < CANDIDATE < CONFIRMED.
   */
  [[nodiscard]] DynamicRelevance evaluateDynamicRelevance(
      const avr::spatial::CellKey& region) const noexcept {
    if (!avr::spatial::isValidLevel(region.level)) {
      return DynamicRelevance::NONE;
    }

    avr::spatial::CellKey l3_key = region;
    while (l3_key.level < avr::spatial::L3) {
      const auto parent_opt = avr::spatial::parentOf(l3_key);
      if (!parent_opt.has_value()) {
        return DynamicRelevance::NONE;
      }
      l3_key = *parent_opt;
    }

    const auto it = buckets_.find(l3_key);
    if (it == buckets_.end()) {
      return DynamicRelevance::NONE;
    }

    DynamicRelevance max_rel = DynamicRelevance::NONE;
    for (const auto& entry : it->second) {
      if (region.level < avr::spatial::L3) {
        const auto key_opt = avr::spatial::CellKey::fromGridCoord(entry.base_coord, region.level);
        if (!key_opt.has_value() || *key_opt != region) {
          continue;
        }
      }
      if (static_cast<uint8_t>(entry.dynamic_relevance) > static_cast<uint8_t>(max_rel)) {
        max_rel = entry.dynamic_relevance;
        if (max_rel == DynamicRelevance::CONFIRMED) {
          return max_rel;  // Maximum possible value reached
        }
      }
    }
    return max_rel;
  }

  [[nodiscard]] size_t observationCount() const noexcept {
    return entry_count_;
  }

  [[nodiscard]] size_t bucketCount() const noexcept {
    return buckets_.size();
  }

 private:
  std::unordered_map<avr::spatial::CellKey, std::vector<Entry>> buckets_{};
  size_t entry_count_{0};
};

}  // namespace avr::mapping
