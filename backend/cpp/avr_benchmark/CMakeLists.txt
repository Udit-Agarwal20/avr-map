# Git provenance is captured by the canonical configure-before-build scripts.
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE AVR_BENCH_GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND git status --porcelain WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE AVR_BENCH_GIT_STATUS OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
set(AVR_BENCH_GIT_DIRTY false)
if(AVR_BENCH_GIT_STATUS)
  set(AVR_BENCH_GIT_DIRTY true)
endif()
set(AVR_BENCH_SANITIZERS false)
if(AVR_ENABLE_SANITIZERS)
  set(AVR_BENCH_SANITIZERS true)
endif()
configure_file(src/build_metadata.hpp.in generated/build_metadata.hpp @ONLY)
add_library(avr_benchmark_support src/sha256.cpp src/sequence.cpp src/benchmark.cpp src/platform.cpp src/artifacts.cpp)
target_include_directories(avr_benchmark_support PUBLIC include PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(avr_benchmark_support PUBLIC AVR::mapping_core)
add_executable(avr_benchmark_runner src/main.cpp)
target_link_libraries(avr_benchmark_runner PRIVATE avr_benchmark_support)
if(AVR_BUILD_TESTS)
  add_executable(avr_benchmark_tests tests/benchmark_test.cpp)
  target_link_libraries(avr_benchmark_tests PRIVATE avr_benchmark_support GTest::gtest_main)
  target_compile_definitions(avr_benchmark_tests PRIVATE AVR_BENCH_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/validation")
  include(GoogleTest)
  gtest_discover_tests(avr_benchmark_tests)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME Benchmark.ArtifactAndProcessIntegration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration_test.py
      $<TARGET_FILE:avr_benchmark_runner> ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/validation
      ${CMAKE_SOURCE_DIR}/../../tools/benchmark_compare.py)
endif()
