add_library(avr_spatial
  src/morton.cpp
)

add_library(AVR::spatial ALIAS avr_spatial)

target_include_directories(avr_spatial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(avr_spatial
  PUBLIC
    avr_build_flags
)

if(AVR_BUILD_TESTS)
  add_subdirectory(tests)
endif()
