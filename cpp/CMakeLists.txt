cmake_minimum_required(VERSION 3.10)

# Project name
project(optimize)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the NLopt package
find_package(NLopt REQUIRED)
include_directories(include)

set(SOURCES
    # src/blended_trajectories.cpp
    src/blended_trajectories_with_blend.cpp
    src/main.cpp
    src/correct_values.cpp
)

set(HEADERS
    include/blended_trajectories_with_blend.hpp
    include/correct_values.hpp
)
# Include directories
include_directories(${NLOPT_INCLUDE_DIRS})

# Add executable

add_executable(optimize ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(optimize ${NLOPT_LIBRARIES})

