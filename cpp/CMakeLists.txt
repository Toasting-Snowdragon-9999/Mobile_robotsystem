cmake_minimum_required(VERSION 3.10)

# Project name
project(mobile_system)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the NLopt package
# find_package(NLopt REQUIRED)
include_directories(include)
find_package(SFML 2.5 COMPONENTS audio system REQUIRED) # for sine waves

set(SOURCES
    src/main.cpp
    src/hello_world.cpp
    src/read_shared_data.cpp
    src/wave_generator.cpp
)

set(HEADERS
    include/hello_world.h
    include/read_shared_data.h
    include/wave_generator.h
)
# Include directories
#include_directories(${NLOPT_INCLUDE_DIRS})

# Add executable

add_executable(mobile_system ${SOURCES} ${HEADERS})

# Link libraries
# target_link_libraries(mobile_system ${NLOPT_LIBRARIES})
target_link_libraries(mobile_system PRIVATE sfml-audio sfml-system) # for sine waves
