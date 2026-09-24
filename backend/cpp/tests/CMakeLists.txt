add_executable(avr_sanity_test
  sanity_test.cpp
)

target_link_libraries(avr_sanity_test
  PRIVATE
    avr_build_flags
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(avr_sanity_test)
