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
find_package(PkgConfig REQUIRED)
pkg_check_modules(SNDFILE REQUIRED sndfile)

# Shared sources
set(SHARED_SOURCES
    src/algorithms/goertzel.cpp
    src/audio/audio_input.cpp
    src/audio/sfml_wave_generator.cpp
    src/communication_protocol/physical_layer.cpp
    src/communication_protocol/data_link_layer.cpp
    src/communication_protocol/application_layer.cpp
    src/communication_protocol/transport_layer.cpp 
    src/communication_protocol/crc.cpp
    src/interfaces/al_to_tl.cpp
    src/interfaces/tl_to_dll.cpp
    src/interfaces/dll_to_pl.cpp
    src/interfaces/signal_processing.cpp
)

# GUI executable sources
set(GUI_SOURCES
    src/main_gui_code.cpp
    src/read_shared_data.cpp
)

# Robot executable sources
set(ROBOT_SOURCES
    src/main.cpp
)

# Add executables
add_executable(robot_code ${ROBOT_SOURCES} ${SHARED_SOURCES})
add_executable(gui_code ${GUI_SOURCES} ${SHARED_SOURCES})

# Link libraries for robot_code
target_link_libraries(robot_code
    PRIVATE 
        sfml-audio 
        sfml-system
        ${SNDFILE_LIBRARIES}
        ${PORTAUDIO_LIBRARIES}  # Link PortAudio
)

# Link libraries for gui_code
target_link_libraries(gui_code
    PRIVATE 
        sfml-audio 
        sfml-system
        ${SNDFILE_LIBRARIES}
        ${PORTAUDIO_LIBRARIES}  # Link PortAudio
)

# Include PortAudio headers for both executables
target_include_directories(robot_code PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
target_include_directories(gui_code PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
