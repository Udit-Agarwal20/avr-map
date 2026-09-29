#pragma once
#include <array>
#include <filesystem>
#include <string>
#include <vector>
#include "avr_mapping/semantic_provider.hpp"

namespace avr::benchmark {
struct FrameIndex {
  uint64_t frame_seq{}, timestamp_ns{};
  mapping::PointXYZ sensor_origin{};
  std::string points_file, content_hash;
  size_t point_count{};
};
struct LabeledFrame {
  FrameIndex index;
  std::vector<mapping::PointXYZ> points;
  std::vector<double> intensities;
  mapping::SemanticFrameResult semantics;
};
struct Sequence {
  std::filesystem::path root;
  std::string name, kind, provenance, source_description, license, pose_provenance, hash;
  std::array<double,4> bounds{}; // min x, max x, min y, max y, half-open
  std::vector<FrameIndex> frames;
  uint64_t point_count{};
  [[nodiscard]] bool isFixture() const { return kind=="HARNESS_VALIDATION_FIXTURE"; }
  [[nodiscard]] std::string dataTruth() const;
};
// Throws on any malformed data, missing evidence, non-monotonic sequence/time,
// out-of-bounds/non-quantizable geometry or escaping paths. Never filters/reorders.
[[nodiscard]] Sequence loadSequence(const std::filesystem::path& directory);
[[nodiscard]] LabeledFrame loadFrame(const Sequence&, size_t ordinal);
[[nodiscard]] uint64_t parseUnsigned(const std::string& value);

class OfflineLabeledSemanticProvider final : public mapping::SemanticProvider {
 public:
  // Non-owning binding valid for the synchronous processFrame call only.
  void bind(const LabeledFrame& frame) { frame_=&frame; }
  std::optional<mapping::SemanticFrameResult> infer(
      std::span<const mapping::PointXYZ>, uint64_t, spatial::WorldPoint2D) override;
 private:
  const LabeledFrame* frame_{nullptr};
};
}
