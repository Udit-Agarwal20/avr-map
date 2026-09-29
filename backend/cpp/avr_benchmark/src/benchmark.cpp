#include "avr_benchmark/benchmark.hpp"
#include "avr_benchmark/sha256.hpp"
#include "avr_mapping/dynamic_clusterer.hpp"
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace avr::benchmark {
Mode parseMode(const std::string& s) {
  if(s=="uniform-l0-observed") return Mode::UNIFORM_L0_OBSERVED;
  if(s=="distance-only") return Mode::DISTANCE_ONLY;
  if(s=="full-avr") return Mode::FULL_AVR;
  throw std::runtime_error("unknown benchmark mode: "+s);
}
std::string modeName(Mode m) {
  switch(m) {
    case Mode::UNIFORM_L0_OBSERVED:return "UNIFORM_L0_OBSERVED";
    case Mode::DISTANCE_ONLY:return "DISTANCE_ONLY";
    case Mode::FULL_AVR:return "FULL_AVR";
  }
  throw std::runtime_error("invalid mode");
}
std::string modeLabel(Mode m) {return m==Mode::UNIFORM_L0_OBSERVED?"Uniform 5 cm — observed support":modeName(m);}
Distribution distribution(std::vector<double> values) {
  Distribution d;d.count=values.size();if(values.empty()) return d;
  for(auto v:values) if(!std::isfinite(v)||v<0) throw std::runtime_error("invalid latency sample");
  std::sort(values.begin(),values.end());
  d.mean=std::accumulate(values.begin(),values.end(),0.0)/values.size();
  const auto percentile=[&](double q) {
    const auto index=q*(values.size()-1);const auto lo=static_cast<size_t>(index);
    const auto hi=std::min(lo+1,values.size()-1);
    return values[lo]+(values[hi]-values[lo])*(index-lo);
  };
  d.p50=percentile(.5);d.p95=percentile(.95);d.p99=percentile(.99);d.max=values.back();return d;
}
BenchmarkSession::BenchmarkSession(Mode mode) {
  config_.policy_config.selection_strategy=mode;
  auto provider=std::make_unique<OfflineLabeledSemanticProvider>();provider_=provider.get();
  pipeline_=mapping::PersistentAdaptiveFramePipeline::create(config_,std::move(provider));
  if(!pipeline_) throw std::runtime_error("invalid pipeline configuration");
}
FrameMetrics BenchmarkSession::process(const LabeledFrame& frame) {
  if(stopped_) throw std::runtime_error("benchmark stopped after failed frame");
  FrameMetrics row;row.frame_seq=frame.index.frame_seq;row.timestamp_ns=frame.index.timestamp_ns;
  row.raw_points=frame.points.size();row.input_hash=frame.index.content_hash;
  provider_->bind(frame);
  const auto fused_before=pipeline_->map().statistics().total_observations_fused;
  const auto start=std::chrono::steady_clock::now();
  row.result=pipeline_->processFrame(frame.points,frame.index.frame_seq,
      {frame.index.sensor_origin.x,frame.index.sensor_origin.y});
  const auto end=std::chrono::steady_clock::now();
  row.latency_us=std::chrono::duration<double,std::micro>(end-start).count();
  // Clustering, external validation, hashing, file I/O and RSS sampling are outside
  // processFrame timing. Pipeline-internal validation remains inside it.
  auto clustered=mapping::DynamicClusterer::cluster(row.result.dynamic_points,{});
  row.clusters=clustered.clusters.size();
  const auto stats=pipeline_->map().statistics();row.hierarchy=stats.hierarchy_stats;row.payload=stats.payload_stats;
  row.fused_observations=stats.total_observations_fused-fused_before;
  row.generation=pipeline_->processingGeneration();
  row.invariants_valid=static_cast<bool>(pipeline_->map().validateInvariants())&&
      row.result.status!=mapping::AdaptivePipelineStatus::INVARIANT_FAILURE;
  const auto* cached=pipeline_->replayCache().findFrame(frame.index.frame_seq);
  row.routing_valid=cached&&cached->rawPoints().size()==frame.points.size()&&
      cached->routing().size()==frame.points.size()&&cached->sensorOrigin().x==frame.index.sensor_origin.x&&
      cached->sensorOrigin().y==frame.index.sensor_origin.y&&
      std::equal(cached->rawPoints().begin(),cached->rawPoints().end(),frame.points.begin());
  Sha256 routing;hashField(routing,std::to_string(frame.index.frame_seq));
  hashField(routing,std::to_string(frame.index.timestamp_ns));
  if(row.routing_valid) {
    for(size_t i=0;i<frame.points.size();++i) {
      const auto& observation=frame.semantics.observations.at(i);
      const bool expected_dynamic=observation.motion_state==mapping::MotionClassification::CONFIRMED_DYNAMIC&&
          observation.motion_confidence>=config_.semantic_config.min_motion_confidence;
      const auto actual=cached->routing()[i];
      const bool dynamic=actual==mapping::PointRouting::DYNAMIC_EXCLUDED;
      row.routing_valid=row.routing_valid&&(observation.source_point_index==i)&&
          (actual==mapping::PointRouting::STATIC_FUSABLE||dynamic)&&(dynamic==expected_dynamic);
      if(dynamic) ++row.dynamic_points; else ++row.static_points;
      hashField(routing,std::to_string(i));
      const auto& p=cached->rawPoints()[i];
      for(auto v:{p.x,p.y,p.z}) hashField(routing,std::to_string(std::bit_cast<uint64_t>(v)));
      hashField(routing,std::to_string(static_cast<unsigned>(actual)));
    }
  }
  row.routing_hash=routing.hex();
  Sha256 topology;
  for(const auto& node:pipeline_->map().hierarchy().topologySnapshot()) {
    hashField(topology,std::to_string(node.key.morton));hashField(topology,std::to_string(node.key.level));
    hashField(topology,std::to_string(static_cast<unsigned>(node.state)));
  }
  row.topology_hash=topology.hex();row.rss_bytes=currentRssBytes();
  row.valid=row.result.isSuccess()&&row.invariants_valid&&row.routing_valid&&clustered.isSuccess()&&
      row.result.accepted_points==row.raw_points&&row.static_points+row.dynamic_points==row.raw_points&&
      row.result.semantic_diagnostics.dynamic_excluded_points==row.dynamic_points&&
      row.result.dynamic_points.points.size()==row.dynamic_points&&row.fused_observations==row.static_points;
  stopped_=!row.valid;return row;
}
}
