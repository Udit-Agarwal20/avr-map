add_executable(avr_mapping_tests
  test_elevation_stats.cpp
  test_uniform_cell.cpp
  test_uniform_tile_boundaries.cpp
  test_uniform_map.cpp
  test_uniform_map_stress.cpp
  test_adaptive_tile.cpp
  test_tile_pool.cpp
  test_adaptive_hierarchy.cpp
  test_hierarchy_boundaries.cpp
  test_hierarchy_randomized.cpp
)

target_link_libraries(avr_mapping_tests
  PRIVATE
    AVR::mapping_core
    AVR::spatial
    avr_build_flags
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(avr_mapping_tests)
