cmake_minimum_required(VERSION 3.16)

project(Pathfinder LANGUAGES CXX C)

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
  behaviorFactory.cpp
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