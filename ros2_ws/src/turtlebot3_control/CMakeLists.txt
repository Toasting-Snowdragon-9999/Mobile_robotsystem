cmake_minimum_required(VERSION 3.5)

# Project name
project(turtlebot3_control)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    ../../../cpp/include
    ../../../cpp/src
)

# For SFML (sine waves)
find_package(SFML 2.5 COMPONENTS audio system REQUIRED)

# For ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
link_directories(/usr/lib/aarch64-linux-gnu/)
find_package(SFML 2.5 REQUIRED COMPONENTS audio system)

if(SFML_FOUND)
    message(STATUS "SFML found: ${SFML_LIBRARIES}")
else()
    message(FATAL_ERROR "SFML not found!")
endif()

# Sources
set(SOURCES
    # Robot
    src/main.cpp
    src/move_turtlebot.cpp
    # Audio
    ../../../cpp/src/audio/audio_input.cpp
    ../../../cpp/src/audio/wave_generator.cpp
    # Signal Processing
    ../../../cpp/src/algorithms/goertzel.cpp
    # Communication Protocol
    ../../../cpp/src/communication_protocol/physical_layer.cpp
    ../../../cpp/src/communication_protocol/data_link_layer.cpp
    ../../../cpp/src/communication_protocol/application_layer.cpp
    ../../../cpp/src/communication_protocol/transport_layer.cpp
    ../../../cpp/src/communication_protocol/crc.cpp
    # Interfaces
    ../../../cpp/src/interfaces/al_to_tl.cpp
    ../../../cpp/src/interfaces/tl_to_dll.cpp
    ../../../cpp/src/interfaces/dll_to_pl.cpp
    ../../../cpp/src/interfaces/signal_processing.cpp
)

# Add executable
add_executable(move_turtlebot ${SOURCES})

ament_target_dependencies(move_turtlebot
    rclcpp
    geometry_msgs
    nav_msgs
    rosidl_default_runtime
)

target_link_libraries(move_turtlebot
        sfml-audio
        sfml-system
        ${PORTAUDIO_LIBRARIES}
        tf2::tf2
        tf2_geometry_msgs::tf2_geometry_msgs
)

# PortAudio headers
target_include_directories(move_turtlebot
    PRIVATE
        ${PORTAUDIO_INCLUDE_DIRS}
)

install(TARGETS
  move_turtlebot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
