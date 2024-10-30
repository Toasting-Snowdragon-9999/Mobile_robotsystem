cmake_minimum_required(VERSION 3.10)

# Project name
project(robot_code)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Sources
set(SOURCES
    src/main.cpp
    src/dft.cpp
    src/goertzel.cpp
)

# Headers
set(HEADERS
    include/algorithms.h
    include/dft.h
    include/goertzel.h
)

# Add executable
add_executable(robot_code ${SOURCES} ${HEADERS})

