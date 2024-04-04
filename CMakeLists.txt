cmake_minimum_required(VERSION 3.26.4)

project(Pathfinder LANGUAGES CXX C)

# Leave C++ standard up to the root application, so set it only if this is the
# current top-level CMake project.
if(CMAKE_SOURCE_DIR STREQUAL Pathfinder_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_STANDARD VERSION_LESS 17)
  message(FATAL_ERROR "${PROJECT_NAME} requires C++17 or newer. Please set CMAKE_CXX_STANDARD accordingly.")
endif()

set(ABSL_PROPAGATE_CXX_STD ON)
add_subdirectory(abseil-cpp)

# Create an interface library for Triangle that will be used by Pathfinder
add_library(Triangle INTERFACE)

# Add the sources of Triangle
target_sources(Triangle INTERFACE
  triangle/acute.c
  triangle/eps_writer.c
  triangle/predicates.c
  triangle/triangle_api.c
  triangle/triangle_helper.c
  triangle/triangle_io.c
  triangle/triangle.c
)

# Create Pathfinder as a static library
add_library(${PROJECT_NAME} STATIC
  behaviorBuilder.cpp
  debuglogger.cpp
  funnel.cpp
  math_helpers.cpp
  path.cpp
  pathfinder.cpp
  triangle_lib_navmesh.cpp
  vector.cpp
)

# Link Triangle to Pathfinder
target_link_libraries(${PROJECT_NAME}
PRIVATE
  Triangle
PUBLIC
  absl::log
  absl::log_initialize
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Specify the source and destination paths for .poly files
set(POLY_FILES_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
set(POLY_FILES_DEST_DIR ${CMAKE_BINARY_DIR}/poly)

# Use file(COPY ...) with a wildcard (*) to copy all .poly files to the build directory
file(GLOB POLY_FILES "${POLY_FILES_SOURCE_DIR}/*.poly")
file(COPY ${POLY_FILES}
     DESTINATION ${POLY_FILES_DEST_DIR})

# ==============================================================================
# =================================== Tests ====================================
# ==============================================================================

find_package(GTest CONFIG)

if (NOT ${GTest_FOUND})

  # GoogleTest is not installed, pull a copy
  # TODO: Maybe this is a bad idea as it could result in different versions
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/703bd9caab50b139428cea1aaff9974ebee5742e.zip
  )

  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

endif()

enable_testing()

add_executable(pathfinder_test
  test/math.cpp
  test/pathfinding.cpp
)

target_link_libraries(pathfinder_test
PUBLIC
  absl::log
  absl::log_initialize
  ${PROJECT_NAME}
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(pathfinder_test)