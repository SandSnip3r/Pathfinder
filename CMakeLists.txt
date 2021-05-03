cmake_minimum_required(VERSION 3.16)

project(Pathfinder LANGUAGES CXX C)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)

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
  pathfinder.cpp
  vector.cpp
)

target_compile_features(Pathfinder PUBLIC cxx_std_17)

# Link Triangle to Pathfinder
target_link_libraries(${PROJECT_NAME}
  Triangle
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

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

add_executable(
  pathfinder_test
  test/math.cpp
)
target_link_libraries(
  pathfinder_test
  ${PROJECT_NAME}
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(pathfinder_test)