cmake_minimum_required(VERSION 3.10)

# Project name
project(robot_code)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Find PortAudio explicitly
set(PORTAUDIO_LIB_DIR "/usr/lib/x86_64-linux-gnu/")
set(PORTAUDIO_INCLUDE_DIR "/usr/include")

find_library(PORTAUDIO_LIBRARIES NAMES portaudio PATHS ${PORTAUDIO_LIB_DIR} REQUIRED)
find_path(PORTAUDIO_INCLUDE_DIRS NAMES portaudio.h PATHS ${PORTAUDIO_INCLUDE_DIR} REQUIRED)

if (NOT PORTAUDIO_LIBRARIES OR NOT PORTAUDIO_INCLUDE_DIRS)
    message(FATAL_ERROR "PortAudio not found!")
else()
    message(STATUS "Found PortAudio: ${PORTAUDIO_LIBRARIES}")
    message(STATUS "Found PortAudio headers: ${PORTAUDIO_INCLUDE_DIRS}")
endif()

# Find SFML for sine waves
find_package(SFML 2.5 COMPONENTS audio system REQUIRED)

# Find sndfile
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(SNDFILE REQUIRED sndfile)

# Sources
set(SOURCES
    src/main.cpp
    src/audio_input.cpp
)

# Headers
set(HEADERS
    include/audio_input.h
    include/algorithms.h
)

# Add executable
add_executable(robot_code ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(robot_code
    PRIVATE sfml-audio sfml-system
    # ${SNDFILE_LIBRARIES}
    ${PORTAUDIO_LIBRARIES}  # Link PortAudio
)

# Include PortAudio headers
target_include_directories(robot_code PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
