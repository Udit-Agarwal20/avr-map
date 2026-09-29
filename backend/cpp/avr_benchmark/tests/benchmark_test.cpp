#include "avr_benchmark/benchmark.hpp"
#include "avr_benchmark/sha256.hpp"
#include "avr_mapping/adaptive_cell_payload.hpp"
#include <gtest/gtest.h>
#include <array>
#include <chrono>
#include <fstream>
#include <iterator>

namespace avr::mapping {
struct PersistentAdaptivePipelineTestAccess {
  static void failInvariant(PersistentAdaptiveFramePipeline& p){p.test_simulate_invariant_failure_=true;}
};
}
namespace avr::benchmark {
namespace {
constexpr std::array<Mode,3> modes{Mode::UNIFORM_L0_OBSERVED,Mode::DISTANCE_ONLY,Mode::FULL_AVR};
class CopyFixture:public ::testing::Test {
 protected:
  std::filesystem::path root;
  void SetUp() override {
    root=std::filesystem::temp_directory_path()/("avr-benchmark-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::copy(AVR_BENCH_FIXTURE,root,std::filesystem::copy_options::recursive);
  }
  void TearDown() override {std::filesystem::remove_all(root);}
  void replace(const std::string& file,const std::string& from,const std::string& to) {
    std::ifstream in(root/file);std::string bytes{std::istreambuf_iterator<char>(in),{}};
    const auto pos=bytes.find(from);ASSERT_NE(pos,std::string::npos);bytes.replace(pos,from.size(),to);
    std::ofstream(root/file)<<bytes;
  }
};
int ownerLevel(const BenchmarkSession& s,const mapping::PointXYZ& p) {
  for(const auto key:s.pipeline().map().hierarchy().leafSnapshot()) {
    auto b=key.bounds();if(p.x>=b.min_x&&p.x<b.max_x&&p.y>=b.min_y&&p.y<b.max_y) return key.level;
  }
  return -1;
}
}
TEST(BenchmarkHash,KnownVectorsAndChunking) {
  EXPECT_EQ(sha256(""),"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256("abc"),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  Sha256 h;for(int i=0;i<1000;++i)h.update(std::string(1000,'a'));
  EXPECT_EQ(h.hex(),"cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  EXPECT_EQ(h.hex(),h.hex());
}
TEST(BenchmarkHash,FramingPreventsConcatenationAmbiguity) {
  Sha256 a,b;hashField(a,"ab");hashField(a,"c");hashField(b,"a");hashField(b,"bc");EXPECT_NE(a.hex(),b.hex());
}
TEST(BenchmarkParser,PreservesSourceIntensityOriginAndTime) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);auto f=loadFrame(seq,3);
  EXPECT_EQ(seq.frames.size(),20u);EXPECT_EQ(seq.point_count,280u);
  EXPECT_EQ(f.index.frame_seq,4u);EXPECT_EQ(f.index.timestamp_ns,400000000u);
  EXPECT_EQ(f.points.size(),14u);EXPECT_DOUBLE_EQ(f.intensities[10],20);
  for(size_t i=0;i<f.points.size();++i) EXPECT_EQ(f.semantics.observations[i].source_point_index,i);
  EXPECT_TRUE(seq.isFixture());EXPECT_NE(seq.dataTruth().find("NOT REAL PERFORMANCE DATA"),std::string::npos);
}
TEST_F(CopyFixture,MissingLabelsRejected) {
  replace("road.csv","semantic_class","absent_labels");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,DuplicateSourceIndexRejected) {
  replace("road.csv","1,20.025","0,20.025");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,ReorderedSourceIndexRejected) {
  replace("road.csv","0,1.025","1,1.025");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,InvalidEnumsRejected) {
  replace("road.csv",",2,0.95,1,",",8,0.95,1,");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,InvalidConfidenceRejected) {
  replace("road.csv",",0.95,1,",",1.01,1,");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,NonFiniteGeometryRejected) {
  replace("road.csv","1.025","nan");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,OutOfBoundsRejectedWithoutFiltering) {
  replace("road.csv","1.025","81.025");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,NonMonotonicFrameRejected) {
  replace("frames.csv","2,200000000","1,200000000");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,NonMonotonicTimeRejected) {
  replace("frames.csv","2,200000000","2,100000000");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,UnknownOrDuplicateManifestFieldRejected) {
  std::ofstream(root/"sequence.avr",std::ios::app)<<"name=duplicate\n";EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,PathsCannotEscapeSequence) {
  replace("frames.csv","road.csv","../road.csv");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,SyntheticLabelsCannotBeCalledReal) {
  replace("sequence.avr","kind=HARNESS_VALIDATION_FIXTURE","kind=REAL_RECORDED");EXPECT_THROW((void)loadSequence(root),std::runtime_error);
}
TEST_F(CopyFixture,DatasetHashStableAndSensitiveToIntensity) {
  auto a=loadSequence(root),b=loadSequence(root);EXPECT_EQ(a.hash,b.hash);
  replace("road.csv",",10,2,",",11,2,");EXPECT_NE(a.hash,loadSequence(root).hash);
  EXPECT_THROW((void)loadFrame(a,0),std::runtime_error);
}
TEST(BenchmarkProvider,DeterministicAndRejectsMisalignment) {
  const auto seq=loadSequence(AVR_BENCH_FIXTURE);auto f=loadFrame(seq,0);
  OfflineLabeledSemanticProvider p;p.bind(f);
  auto first=p.infer(f.points,1,{0,0}),second=p.infer(f.points,1,{0,0});
  ASSERT_TRUE(first);ASSERT_TRUE(second);EXPECT_EQ(first->frame_seq,second->frame_seq);EXPECT_EQ(first->observations,second->observations);
  EXPECT_THROW(p.infer(f.points,2,{0,0}),std::runtime_error);
  EXPECT_THROW(p.infer(f.points,1,{1,0}),std::runtime_error);
  auto points=f.points;std::swap(points[0],points[1]);EXPECT_THROW(p.infer(points,1,{0,0}),std::runtime_error);
}
TEST(BenchmarkPolicy,DistanceIgnoresSignalsButValidatesThem) {
  mapping::ResolutionPolicyConfig c;c.selection_strategy=Mode::DISTANCE_ONLY;
  auto controller=mapping::ResolutionController::create(c);
  mapping::ResolutionSignals signals;signals.distance_m=70;signals.observation_count=10;signals.density_score=1;
  signals.semantic_relevance=mapping::SemanticRelevance::SAFETY_CRITICAL;signals.dynamic_relevance=mapping::DynamicRelevance::CONFIRMED;
  signals.uncertainty=1;signals.geometry_complexity=1;
  auto d=controller->decide(signals);ASSERT_TRUE(d);EXPECT_EQ(d->requested_level,3);
  EXPECT_EQ(d->reasons.mask,static_cast<uint16_t>(mapping::ResolutionReason::DISTANCE_BASELINE));
  signals.distance_m=-1;EXPECT_FALSE(controller->decide(signals));
}
TEST(BenchmarkPolicy,UniformOnlyRequestsSupportedRegions) {
  mapping::ResolutionPolicyConfig c;c.selection_strategy=Mode::UNIFORM_L0_OBSERVED;
  auto controller=mapping::ResolutionController::create(c);mapping::ResolutionSignals s;s.distance_m=70;
  EXPECT_EQ(controller->decide(s)->requested_level,3);s.observation_count=1;
  EXPECT_EQ(controller->decide(s)->requested_level,0);
  c.selection_strategy=static_cast<Mode>(255);EXPECT_FALSE(c.validate());
}
TEST(BenchmarkIntegration,IdenticalRoutingAcrossAllModesAndIntendedAllocation) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);
  std::array<std::unique_ptr<BenchmarkSession>,3> sessions;
  for(size_t i=0;i<3;++i)sessions[i]=std::make_unique<BenchmarkSession>(modes[i]);
  for(size_t f=0;f<seq.frames.size();++f) {
    auto frame=loadFrame(seq,f);std::array<FrameMetrics,3> rows;
    for(size_t m=0;m<3;++m) {
      rows[m]=sessions[m]->process(frame);ASSERT_TRUE(rows[m].valid)<<f<<' '<<m;
      EXPECT_EQ(rows[m].static_points,8u);EXPECT_EQ(rows[m].dynamic_points,6u);EXPECT_EQ(rows[m].clusters,1u);
      EXPECT_EQ(rows[m].routing_hash,rows[0].routing_hash);
    }
    for(const auto& p:frame.points) EXPECT_EQ(ownerLevel(*sessions[0],p),0);
    if(f==3) {
      EXPECT_EQ(ownerLevel(*sessions[1],frame.points[4]),1);EXPECT_EQ(ownerLevel(*sessions[2],frame.points[4]),0);
      EXPECT_EQ(ownerLevel(*sessions[1],frame.points[8]),2);EXPECT_EQ(ownerLevel(*sessions[2],frame.points[8]),0);
    }
  }
  EXPECT_LT(sessions[0]->pipeline().map().statistics().hierarchy_stats.leaf_count,200u);
}
TEST(BenchmarkIntegration,DynamicReturnsNeverEnterActiveOrRetiredElevationDuringReplayAndCoarsening) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);
  for(auto mode:modes) {
    BenchmarkSession s(mode);size_t splits=0,merges=0;
    for(size_t f=0;f<seq.frames.size();++f) {
      auto row=s.process(loadFrame(seq,f));ASSERT_TRUE(row.valid);splits+=row.result.splits_applied;merges+=row.result.direct_merges_applied;
      for(const auto& [key,p]:s.pipeline().map().activePayloadSnapshot()) EXPECT_DOUBLE_EQ(p.elevation().max(),.1);
      for(const auto& [key,p]:s.pipeline().map().retiredCacheSnapshot()) EXPECT_DOUBLE_EQ(p.elevation().max(),.1);
    }
    EXPECT_GT(splits,0u);if(mode==Mode::FULL_AVR) EXPECT_GT(merges,0u);
  }
}
TEST(BenchmarkIntegration,TopologyDeterministicOnRepeatedRuns) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);
  for(auto mode:modes) {
    BenchmarkSession a(mode),b(mode);
    for(size_t i=0;i<seq.frames.size();++i) {
      auto f=loadFrame(seq,i);auto x=a.process(f),y=b.process(f);
      EXPECT_TRUE(x.valid);EXPECT_TRUE(y.valid);EXPECT_EQ(x.topology_hash,y.topology_hash);EXPECT_EQ(x.routing_hash,y.routing_hash);
    }
  }
}
TEST(BenchmarkFailure,NonMonotonicPipelineStopsSession) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);auto frame=loadFrame(seq,0);BenchmarkSession s(Mode::FULL_AVR);
  ASSERT_TRUE(s.process(frame).valid);auto failed=s.process(frame);EXPECT_FALSE(failed.valid);
  EXPECT_EQ(failed.result.status,mapping::AdaptivePipelineStatus::FRAME_REJECTED_SEQUENCE);
  EXPECT_THROW(s.process(loadFrame(seq,1)),std::runtime_error);
}
TEST(BenchmarkFailure,InvariantFailureStopsSession) {
  auto seq=loadSequence(AVR_BENCH_FIXTURE);BenchmarkSession s(Mode::FULL_AVR);
  mapping::PersistentAdaptivePipelineTestAccess::failInvariant(const_cast<mapping::PersistentAdaptiveFramePipeline&>(s.pipeline()));
  auto failed=s.process(loadFrame(seq,0));EXPECT_FALSE(failed.valid);EXPECT_FALSE(failed.invariants_valid);
  EXPECT_EQ(failed.result.status,mapping::AdaptivePipelineStatus::INVARIANT_FAILURE);
  EXPECT_THROW(s.process(loadFrame(seq,1)),std::runtime_error);
}
TEST(BenchmarkMetrics,PercentilesAndEmptyPopulation) {
  auto d=distribution({40,10,20,30});EXPECT_DOUBLE_EQ(d.mean,25);EXPECT_DOUBLE_EQ(d.p50,25);
  EXPECT_DOUBLE_EQ(d.p95,38.5);EXPECT_DOUBLE_EQ(d.p99,39.7);EXPECT_DOUBLE_EQ(d.max,40);
  EXPECT_EQ(distribution({}).count,0u);EXPECT_DOUBLE_EQ(distribution({7}).p99,7);
  EXPECT_THROW(distribution({-1}),std::runtime_error);
}
TEST(BenchmarkMetrics,StatusNamesRetainCanonicalCodes) {
  constexpr std::array<const char*,22> names{"SUCCESS","SUCCESS_REPLAY_PENDING","SUCCESS_CACHE_DEGRADED","FRAME_REJECTED_INPUT","FRAME_REJECTED_SEQUENCE","FRAME_REJECTED_DECISION","FRAME_REJECTED_POLICY","REFINEMENT_INTERNAL_ERROR","REPLAY_INTERNAL_ERROR","FUSION_FAILED","FUSION_FAILED_AFTER_STRUCTURAL_CHANGE","POLICY_COMMIT_INTERNAL_ERROR","POLICY_COMMIT_FATAL","CACHE_COMMIT_INTERNAL_ERROR","GENERATION_EXHAUSTED","POLICY_REVISION_EXHAUSTED","INVARIANT_FAILURE","PIPELINE_FATAL_LATCHED","COARSENING_INTERNAL_ERROR","FRAME_REJECTED_SEMANTIC_MALFORMED","FRAME_REJECTED_SEMANTIC_ERROR","FRAME_REJECTED_SEMANTIC_RESOURCE"};
  for(size_t i=0;i<names.size();++i)EXPECT_STREQ(mapping::adaptivePipelineStatusToString(static_cast<mapping::AdaptivePipelineStatus>(i)),names[i]);
}
TEST(BenchmarkMetrics,OsRssIsAvailableInBytes) {EXPECT_GT(currentRssBytes(),1024u);EXPECT_GT(peakRssBytes(),1024u);}
}
