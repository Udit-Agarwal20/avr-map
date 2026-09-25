add_executable(avr_spatial_tests
  test_floor_div.cpp
  test_resolution.cpp
  test_quantization.cpp
  test_zigzag.cpp
  test_morton.cpp
  test_cell_key.cpp
  test_hierarchy.cpp
  test_tile.cpp
  test_randomized.cpp
)

target_link_libraries(avr_spatial_tests
  PRIVATE
    AVR::spatial
    avr_build_flags
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(avr_spatial_tests)
