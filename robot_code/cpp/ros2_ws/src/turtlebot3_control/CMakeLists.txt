cmake_minimum_required(VERSION 3.5)
project(turtlebot3_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GetYaw.srv"
# )

add_executable(move_turtlebot src/main.cpp src/move_turtlebot.cpp)
ament_target_dependencies(move_turtlebot rclcpp geometry_msgs nav_msgs rosidl_default_runtime)

target_link_libraries(move_turtlebot tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs)

install(TARGETS
  move_turtlebot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
