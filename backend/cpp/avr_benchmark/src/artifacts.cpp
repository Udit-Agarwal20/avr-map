#include "avr_benchmark/benchmark.hpp"
#include "avr_benchmark/sha256.hpp"
#include "build_metadata.hpp"
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace avr::benchmark {
namespace {
std::string quote(const std::string& s) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:s) {
    if(c=='"'||c=='\\') out<<'\\'<<c;
    else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}
std::ofstream outputFile(const std::filesystem::path& path) {
  std::ofstream out;out.exceptions(std::ios::badbit|std::ios::failbit);out.open(path);
  out.imbue(std::locale::classic());out<<std::setprecision(17)<<std::boolalpha;return out;
}
std::string utcNow() {
  const auto now=std::time(nullptr);std::tm utc{};
  if(!gmtime_r(&now,&utc)) throw std::runtime_error("UTC unavailable");
  std::ostringstream out;out<<std::put_time(&utc,"%Y-%m-%dT%H:%M:%SZ");return out.str();
}
void writeDistribution(std::ostream& out,const Distribution& d) {
  out<<"{\"count\":"<<d.count;
  if(d.count==0) out<<",\"mean\":null,\"p50\":null,\"p95\":null,\"p99\":null,\"max\":null}";
  else out<<",\"mean\":"<<d.mean<<",\"p50\":"<<d.p50<<",\"p95\":"<<d.p95<<",\"p99\":"<<d.p99<<",\"max\":"<<d.max<<'}';
}
void writeConfig(std::ostream& o,const mapping::PersistentAdaptivePipelineConfig& c) {
  const auto& p=c.policy_config;const auto& s=c.semantic_config;
  o<<"{\"distance_thresholds_m\":["<<p.distance_threshold_1<<','<<p.distance_threshold_2<<','<<p.distance_threshold_3
   <<"],\"min_observations\":"<<p.min_observations<<",\"min_density_score\":"<<p.min_density_score
   <<",\"high_uncertainty_threshold\":"<<p.high_uncertainty_threshold<<",\"high_geometry_threshold\":"<<p.high_geometry_threshold
   <<",\"semantic_elevated_target\":"<<unsigned(p.semantic_elevated_target)<<",\"semantic_critical_target\":"<<unsigned(p.semantic_critical_target)
   <<",\"confirmed_dynamic_target\":"<<unsigned(p.confirmed_dynamic_target)<<",\"uncertainty_refinement_steps\":"<<unsigned(p.uncertainty_refinement_steps)
   <<",\"geometry_refinement_steps\":"<<unsigned(p.geometry_refinement_steps)
   <<",\"coarsen_stable_observations\":"<<c.hysteresis_config.coarsen_stable_observations
   <<",\"replay_max_frames\":"<<c.replay_cache_config.max_frames<<",\"pool_block_capacity\":"<<c.pool_block_capacity
   <<",\"coarsen_group_max_age_generations\":"<<c.coarsen_group_max_age_generations
   <<",\"min_semantic_confidence\":"<<s.min_semantic_confidence<<",\"min_motion_confidence\":"<<s.min_motion_confidence
   <<",\"vehicles_safety_critical\":"<<s.vehicle_safety_critical<<",\"pedestrians_safety_critical\":"<<s.pedestrian_safety_critical<<'}';
}
void csvHeader(std::ostream& o) {
  o<<"data_truth,frame_seq,timestamp_ns,input_hash,raw_points,accepted_points,static_fusable_points,dynamic_excluded_points,dynamic_cluster_count,status_code,status_name,frame_latency_us,active_leaf_count,l0_count,l1_count,l2_count,l3_count,internal_node_count,allocated_tile_count,refinement_requests,splits_applied,direct_merges_applied,subtree_merges_applied,processing_generation,rss_bytes,invariants_valid,routing_valid,valid,fused_observations,routing_hash,topology_hash,tile_payload_bytes,pool_reserved_payload_bytes,payload_tile_bytes\n";
}
void csvRow(std::ostream& o,const std::string& truth,const FrameMetrics& r) {
  const auto& h=r.hierarchy;const auto& p=r.result;
  o<<truth<<','<<r.frame_seq<<','<<r.timestamp_ns<<','<<r.input_hash<<','<<r.raw_points<<','<<p.accepted_points<<','<<r.static_points<<','<<r.dynamic_points<<','<<r.clusters
   <<','<<unsigned(p.status)<<','<<mapping::adaptivePipelineStatusToString(p.status)<<','<<r.latency_us<<','<<h.leaf_count;
  for(auto n:h.leaf_count_by_level) o<<','<<n;
  o<<','<<h.internal_count<<','<<h.tile_count<<','<<p.refine_requests<<','<<p.splits_applied<<','<<p.direct_merges_applied<<','<<p.recovery_applied<<','<<r.generation
   <<','<<r.rss_bytes<<','<<r.invariants_valid<<','<<r.routing_valid<<','<<r.valid<<','<<r.fused_observations<<','<<r.routing_hash<<','<<r.topology_hash
   <<','<<h.tile_payload_bytes<<','<<h.pool_reserved_payload_bytes<<','<<r.payload.payload_tile_bytes<<'\n';
}
}
int runBenchmark(const Sequence& seq,Mode mode,const std::filesystem::path& output,size_t warmup) {
  if(warmup>=seq.frames.size()) throw std::runtime_error("warmup must leave at least one measured frame");
  if(std::filesystem::exists(output)) throw std::runtime_error("output already exists; use a fresh directory");
  std::filesystem::create_directories(output);
  const auto started=utcNow();
  auto csv=outputFile(output/"frames.csv");csvHeader(csv);
  const auto baseline=currentRssBytes();
  BenchmarkSession session(mode);
  std::vector<double> latencies;latencies.reserve(seq.frames.size());
  Sha256 routing;
  uint64_t static_total=0,dynamic_total=0,refinements=0,direct_merges=0,subtree_merges=0,invariant_failures=0;
  FrameMetrics last;std::string error;
  try {
    for(size_t i=0;i<seq.frames.size();++i) {
      auto frame=loadFrame(seq,i);last=session.process(frame);
      csvRow(csv,seq.dataTruth(),last);csv.flush();
      latencies.push_back(last.latency_us);hashField(routing,last.routing_hash);
      static_total+=last.static_points;dynamic_total+=last.dynamic_points;refinements+=last.result.refine_requests;
      direct_merges+=last.result.direct_merges_applied;subtree_merges+=last.result.recovery_applied;
      if(!last.invariants_valid) ++invariant_failures;
      if(!last.valid) {error=std::string("invalid frame: ")+mapping::adaptivePipelineStatusToString(last.result.status);break;}
    }
  }catch(const std::exception& e){error=e.what();}
  const auto final_rss=currentRssBytes();const auto peak=std::max({baseline,final_rss,peakRssBytes()});
  const bool valid=error.empty()&&latencies.size()==seq.frames.size();
  const auto post_start=std::min(warmup,latencies.size());
  const std::vector<double> measured(latencies.begin()+post_start,latencies.end());
  auto manifest=outputFile(output/"run_manifest.json");
  manifest<<"{\n\"schema_version\":1,\n\"valid\":"<<valid<<",\n\"data_truth\":"<<quote(seq.dataTruth())
   <<",\n\"git_commit\":"<<quote(AVR_BENCH_GIT_COMMIT)<<",\n\"git_dirty\":"<<AVR_BENCH_GIT_DIRTY
   <<",\n\"git_provenance_scope\":\"CMake configuration snapshot\",\n\"utc_run_timestamp\":"<<quote(started)
   <<",\n\"compiler\":"<<quote(AVR_BENCH_COMPILER)<<",\n\"build_type\":"<<quote(AVR_BENCH_BUILD_TYPE)
   <<",\n\"sanitizers\":"<<AVR_BENCH_SANITIZERS<<",\n\"operating_system\":"<<quote(AVR_BENCH_OS)
   <<",\n\"cpu_architecture\":"<<quote(AVR_BENCH_ARCH)<<",\n\"mode\":"<<quote(modeName(mode))<<",\n\"mode_label\":"<<quote(modeLabel(mode))
   <<",\n\"sequence_name\":"<<quote(seq.name)<<",\n\"sequence_hash\":"<<quote(seq.hash)
   <<",\n\"hash_algorithm\":\"SHA-256 length-prefixed manifest then index then each point file in frame order\""
   <<",\n\"semantic_provenance\":"<<quote(seq.provenance)<<",\n\"source_description\":"<<quote(seq.source_description)
   <<",\n\"license\":"<<quote(seq.license)<<",\n\"pose_provenance\":"<<quote(seq.pose_provenance)
   <<",\n\"coordinate_frame\":\"FIXED_MAP_FRAME\",\n\"grid_origin_m\":[0,0],\n\"bounds_xy_m\":[";
  for(size_t i=0;i<4;++i) manifest<<(i?",":"")<<seq.bounds[i];
  manifest<<"],\n\"frame_count\":"<<seq.frames.size()<<",\n\"point_count\":"<<seq.point_count
   <<",\n\"warmup_frames\":"<<warmup<<",\n\"attempted_frames\":"<<latencies.size()<<",\n\"measured_frame_count\":"<<measured.size()
   <<",\n\"config\":";writeConfig(manifest,session.config());
  const mapping::DynamicClusterConfig cluster_config{};
  manifest<<",\n\"clustering_config\":{\"cluster_radius_xy_m\":"<<cluster_config.cluster_radius_xy_m
   <<",\"max_vertical_gap_m\":"<<cluster_config.max_vertical_gap_m<<",\"min_cluster_points\":"<<cluster_config.min_cluster_points
   <<",\"min_semantic_confidence\":"<<cluster_config.min_semantic_confidence<<'}'
   <<",\n\"uncertainty_source\":\"not supplied by current persistent pipeline\""
   <<",\n\"latency_scope\":\"steady_clock around processFrame only; clustering and harness IO/validation excluded\""
   <<",\n\"rss_scope\":\"whole fresh process; baseline after preflight before pipeline; final before pipeline destruction; peak is OS lifetime high-water including preflight\""
   <<",\n\"rss_current_source\":\"mach task_info on macOS; /proc/self/statm on Linux\",\n\"rss_peak_source\":\"getrusage RUSAGE_SELF converted to bytes\"\n}\n";
  auto summary=outputFile(output/"summary.json");
  summary<<"{\n\"schema_version\":1,\n\"valid\":"<<valid<<",\n\"data_truth\":"<<quote(seq.dataTruth())<<",\n\"error\":"<<quote(error)
   <<",\n\"mode\":"<<quote(modeName(mode))<<",\n\"sequence_hash\":"<<quote(seq.hash)<<",\n\"routing_hash\":"<<quote(routing.hex())
   <<",\n\"attempted_frames\":"<<latencies.size()<<",\n\"static_fusable_points\":"<<static_total<<",\n\"dynamic_excluded_points\":"<<dynamic_total
   <<",\n\"refinement_requests\":"<<refinements<<",\n\"direct_merges_applied\":"<<direct_merges<<",\n\"subtree_merges_applied\":"<<subtree_merges
   <<",\n\"invariant_failures\":"<<invariant_failures
   <<",\n\"dynamic_contamination_count\":null,\n\"dynamic_contamination_note\":\"No per-observation terrain provenance; routing and fusion counts checked; replay/split/merge exclusion tested separately\""
   <<",\n\"latency_us_all_frames\":";writeDistribution(summary,distribution(latencies));
  summary<<",\n\"latency_us_post_warmup\":";writeDistribution(summary,distribution(measured));
  summary<<",\n\"percentile_method\":\"linear interpolation at q*(n-1)\",\n\"rss_bytes\":{\"baseline\":"<<baseline<<",\"peak\":"<<peak<<",\"final\":"<<final_rss
   <<",\"peak_delta\":"<<peak-baseline<<",\"final_delta\":"<<static_cast<int64_t>(final_rss)-static_cast<int64_t>(baseline)<<'}'
   <<",\n\"final_active_leaf_count\":"<<last.hierarchy.leaf_count<<",\n\"final_level_counts\":[";
  for(size_t i=0;i<4;++i) summary<<(i?",":"")<<last.hierarchy.leaf_count_by_level[i];
  summary<<"],\n\"final_topology_hash\":"<<quote(last.topology_hash)
   <<",\n\"structure_size_bytes\":{\"adaptive_tile\":"<<last.hierarchy.sizeof_adaptive_tile<<",\"adaptive_cell_payload\":"<<last.payload.sizeof_payload
   <<",\"adaptive_payload_tile\":"<<last.payload.sizeof_payload_tile<<'}'
   <<",\n\"final_allocated_structure_bytes\":{\"tile_payload\":"<<last.hierarchy.tile_payload_bytes<<",\"pool_reserved_payload\":"<<last.hierarchy.pool_reserved_payload_bytes
   <<",\"payload_tiles\":"<<last.payload.payload_tile_bytes<<'}'
   <<",\n\"structure_bytes_note\":\"sizeof-based components; exclude allocator/container overhead; overlapping categories must not be summed as RSS\""
   <<",\n\"modelled_logical_bytes\":null\n}\n";
  manifest.close();summary.close();csv.close();return valid?0:2;
}
}
