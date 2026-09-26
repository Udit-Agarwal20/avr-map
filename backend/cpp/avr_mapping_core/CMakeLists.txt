add_library(avr_mapping_core
  src/uniform_map.cpp
  src/adaptive_hierarchy.cpp
  src/resolution_controller.cpp
  src/frame_evidence.cpp
  src/evidence_signal_builder.cpp
  src/adaptive_frame_processor.cpp
  src/adaptive_payload_store.cpp
  src/persistent_adaptive_map.cpp
  src/recent_observation_cache.cpp
  src/persistent_replay.cpp
  src/persistent_merge.cpp
  src/resolution_hysteresis.cpp
  src/persistent_subtree_collapse.cpp
  src/persistent_adaptive_frame_pipeline.cpp
)

add_library(AVR::mapping_core ALIAS avr_mapping_core)

target_include_directories(avr_mapping_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(avr_mapping_core
  PUBLIC
    AVR::spatial
    avr_build_flags
)

if(AVR_BUILD_TESTS)
  add_subdirectory(tests)
endif()
