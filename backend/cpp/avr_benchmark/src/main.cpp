#include "avr_benchmark/benchmark.hpp"
#include <iostream>
#include <map>
#include <stdexcept>
int main(int argc,char** argv) {
  try {
    std::map<std::string,std::string> args;
    for(int i=1;i<argc;i+=2) {
      const std::string key=argv[i];
      if(i+1>=argc||(key!="--mode"&&key!="--sequence"&&key!="--output"&&key!="--warmup-frames")||!args.emplace(key,argv[i+1]).second)
        throw std::runtime_error("usage: avr_benchmark_runner --mode MODE --sequence DIRECTORY --output NEW_DIRECTORY [--warmup-frames N]");
    }
    if(!args.contains("--mode")||!args.contains("--sequence")||!args.contains("--output")) throw std::runtime_error("mode, sequence and output are required");
    const auto mode=avr::benchmark::parseMode(args.at("--mode"));
    const auto sequence=avr::benchmark::loadSequence(args.at("--sequence"));
    std::cout<<sequence.dataTruth()<<'\n';
    const auto warmup=args.contains("--warmup-frames")?avr::benchmark::parseUnsigned(args.at("--warmup-frames")):0;
    return avr::benchmark::runBenchmark(sequence,mode,args.at("--output"),warmup);
  }catch(const std::exception& e){std::cerr<<"INVALID: "<<e.what()<<'\n';return 2;}
}
