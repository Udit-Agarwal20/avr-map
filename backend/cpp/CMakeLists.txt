cmake_minimum_required(VERSION 3.22)

project(avr_core
  VERSION 0.1.0
  DESCRIPTION "AVR-Map Core Pure C++ Spatial Mapping Engine"
  LANGUAGES CXX
)

# ------------------------------------------------------------------------------
# C++ Language Standard
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for language servers (clangd, ccls)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Expose compile commands to conventional root and backend locations (policy: dev preset canonical)
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  file(CREATE_LINK
    "${CMAKE_BINARY_DIR}/compile_commands.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json"
    SYMBOLIC
    COPY_ON_ERROR
  )
  if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../compile_commands.json")
    file(CREATE_LINK
      "${CMAKE_BINARY_DIR}/compile_commands.json"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../compile_commands.json"
      SYMBOLIC
      COPY_ON_ERROR
    )
  endif()
endif()

# ------------------------------------------------------------------------------
# Build Options
# ------------------------------------------------------------------------------
option(AVR_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer for debug builds" OFF)
option(AVR_BUILD_TESTS "Build test suite" ON)

# ------------------------------------------------------------------------------
# Compiler Warnings & Sanitizers Interface Target
# ------------------------------------------------------------------------------
add_library(avr_build_flags INTERFACE)

# Compiler Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
  target_compile_options(avr_build_flags INTERFACE
    -Wall
    -Wextra
    -Wpedantic
  )
elseif(MSVC)
  target_compile_options(avr_build_flags INTERFACE
    /W4
    /permissive-
  )
endif()

# Sanitizers (ASan + UBSan)
if(AVR_ENABLE_SANITIZERS)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
    message(STATUS "AVR-Map: Enabling AddressSanitizer and UndefinedBehaviorSanitizer")
    target_compile_options(avr_build_flags INTERFACE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(avr_build_flags INTERFACE
      -fsanitize=address,undefined
    )
  else()
    message(WARNING "AVR-Map: Sanitizers requested but not supported for compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()
endif()

# ------------------------------------------------------------------------------
# Testing Setup with GoogleTest (FetchContent pinned to v1.15.2)
# ------------------------------------------------------------------------------
if(AVR_BUILD_TESTS)
  enable_testing()

  include(FetchContent)
  
  # Prevent GoogleTest from installing rules
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

  message(STATUS "AVR-Map: Fetching GoogleTest (pinned release tag: v1.15.2)")
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(googletest)
endif()

# ------------------------------------------------------------------------------
# Core Pure C++ Libraries
# ------------------------------------------------------------------------------
add_subdirectory(avr_spatial)
add_subdirectory(avr_mapping_core)

# Include infrastructure sanity tests directory
if(AVR_BUILD_TESTS)
  add_subdirectory(tests)
endif()
