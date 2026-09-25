#include "avr_mapping/recent_observation_cache.hpp"

#include <algorithm>

namespace avr::mapping {

std::optional<RecentObservationCache> RecentObservationCache::create(
    ReplayCacheConfig config) noexcept {
  if (!config.isValid()) {
    return std::nullopt;
  }
  return RecentObservationCache(config);
}

RecentObservationCache::RecentObservationCache(ReplayCacheConfig config) noexcept
    : config_(config) {}

std::optional<ReplayCacheFrameStats> RecentObservationCache::pushFrame(
    std::span<const PointXYZ> raw_points,
    uint64_t frame_seq,
    avr::spatial::WorldPoint2D sensor_origin) {
  // 1. Frame order check: strictly increasing frame_seq
  if (!frames_.empty() && frame_seq <= frames_.back().frameSeq()) {
    return std::nullopt;
  }

  // 2. Prepare candidate frame in temporary object
  auto frame_opt = CachedObservationFrame::create(raw_points, frame_seq, sensor_origin);
  if (!frame_opt.has_value()) {
    // Non-finite sensor origin or point count > UINT32_MAX
    return std::nullopt;
  }

  // 3. Commit phase: push new frame
  const auto stats = frame_opt->statistics();
  frames_.push_back(std::move(*frame_opt));

  // 4. Eviction: evict oldest frame(s) until size <= max_frames
  while (frames_.size() > config_.max_frames) {
    frames_.pop_front();
  }

  return stats;
}

const CachedObservationFrame* RecentObservationCache::findFrame(uint64_t frame_seq) const noexcept {
  for (const auto& frame : frames_) {
    if (frame.frameSeq() == frame_seq) {
      return &frame;
    }
  }
  return nullptr;
}

bool RecentObservationCache::containsFrame(uint64_t frame_seq) const noexcept {
  return findFrame(frame_seq) != nullptr;
}

std::optional<uint64_t> RecentObservationCache::oldestFrameSeq() const noexcept {
  if (frames_.empty()) {
    return std::nullopt;
  }
  return frames_.front().frameSeq();
}

std::optional<uint64_t> RecentObservationCache::newestFrameSeq() const noexcept {
  if (frames_.empty()) {
    return std::nullopt;
  }
  return frames_.back().frameSeq();
}

std::vector<uint64_t> RecentObservationCache::frameSequenceSnapshot() const {
  std::vector<uint64_t> seqs;
  seqs.reserve(frames_.size());
  for (const auto& frame : frames_) {
    seqs.push_back(frame.frameSeq());
  }
  return seqs;
}

std::vector<avr::spatial::CellKey> RecentObservationCache::bucketKeySnapshot(uint64_t frame_seq) const {
  const auto* frame = findFrame(frame_seq);
  if (!frame) {
    return {};
  }
  return frame->bucketKeySnapshot();
}

bool RecentObservationCache::collectObservationIndices(
    uint64_t frame_seq,
    const avr::spatial::CellKey& region,
    std::vector<PointIndex>& output) const {
  const auto* frame = findFrame(frame_seq);
  if (!frame) {
    output.clear();
    return false;
  }
  return frame->collectObservationIndices(region, output);
}

ReplayCacheStats RecentObservationCache::statistics() const noexcept {
  ReplayCacheStats stats{};
  stats.max_frames = config_.max_frames;
  stats.stored_frames = frames_.size();
  if (!frames_.empty()) {
    stats.oldest_frame_seq = frames_.front().frameSeq();
    stats.newest_frame_seq = frames_.back().frameSeq();
  }
  for (const auto& frame : frames_) {
    const auto& fstats = frame.statistics();
    stats.stored_raw_points = saturatingAddU64(stats.stored_raw_points, fstats.input_points);
    stats.stored_replay_candidates = saturatingAddU64(stats.stored_replay_candidates, fstats.replay_candidates);
    stats.stored_bucket_count = saturatingAdd(stats.stored_bucket_count, fstats.bucket_count);
    stats.stored_point_indices = saturatingAddU64(stats.stored_point_indices, fstats.point_indices_count);
    stats.raw_point_payload_bytes = saturatingAdd(stats.raw_point_payload_bytes, fstats.raw_point_bytes);
    stats.index_payload_bytes = saturatingAdd(stats.index_payload_bytes, fstats.index_bytes);
  }
  return stats;
}

void RecentObservationCache::reset() noexcept {
  frames_.clear();
}

}  // namespace avr::mapping
