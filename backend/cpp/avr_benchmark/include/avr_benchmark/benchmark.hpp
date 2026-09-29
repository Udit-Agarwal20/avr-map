#pragma once
#include "avr_benchmark/sequence.hpp"
#include "avr_mapping/persistent_adaptive_frame_pipeline.hpp"
#include <memory>

namespace avr::benchmark {
using Mode = mapping::ResolutionSelectionStrategy;
Mode parseMode(const std::string&);
std::string modeName(Mode);
std::string modeLabel(Mode);
uint64_t currentRssBytes();
uint64_t peakRssBytes();
struct Distribution {
  size_t count{};
  double mean{},p50{},p95{},p99{},max{};
};
Distribution distribution(std::vector<double> values);
struct FrameMetrics {
  uint64_t frame_seq{},timestamp_ns{};
  size_t raw_points{},static_points{},dynamic_points{},clusters{};
  double latency_us{};
  uint64_t rss_bytes{},generation{},fused_observations{};
  mapping::AdaptivePipelineFrameResult result;
  mapping::HierarchyStats hierarchy;
  mapping::PayloadStoreStats payload;
  bool invariants_valid{},routing_valid{},valid{};
  std::string input_hash,routing_hash,topology_hash;
};
class BenchmarkSession {
 public:
  explicit BenchmarkSession(Mode mode);
  FrameMetrics process(const LabeledFrame&);
  [[nodiscard]] const mapping::PersistentAdaptiveFramePipeline& pipeline() const {return *pipeline_;}
  [[nodiscard]] const mapping::PersistentAdaptivePipelineConfig& config() const {return config_;}
 private:
  mapping::PersistentAdaptivePipelineConfig config_;
  OfflineLabeledSemanticProvider* provider_{};
  std::unique_ptr<mapping::PersistentAdaptiveFramePipeline> pipeline_;
  bool stopped_{false};
};
// New output directory required. Failure artifacts preserve all attempted frames.
int runBenchmark(const Sequence&,Mode,const std::filesystem::path& output,size_t warmup);
}
