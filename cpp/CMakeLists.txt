cmake_minimum_required(VERSION 3.10)

# Project name
project(mobile_system)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Find SFML for sine waves
find_package(SFML 2.5 COMPONENTS audio system REQUIRED)

# Find sndfile
find_package(PkgConfig REQUIRED)
pkg_check_modules(SNDFILE REQUIRED sndfile)

# Sources
set(SOURCES
    src/main.cpp
    src/hello_world.cpp
    src/read_shared_data.cpp
    src/wave_generator.cpp
    src/com_protocol.cpp
    src/applicationlayer_conversion.cpp
)

# Headers
set(HEADERS
    include/hello_world.h
    include/read_shared_data.h
    include/wave_generator.h
    include/com_protocol.h
    include/applicationlayer_conversion.h
)

# Add executable
add_executable(mobile_system ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(mobile_system
    PRIVATE sfml-audio sfml-system
    ${SNDFILE_LIBRARIES}
)