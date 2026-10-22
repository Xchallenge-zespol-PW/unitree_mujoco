cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  /usr/local/include/iceoryx/v2.0.2
  mujoco/include
  mujoco/simulate
  src/lodepng
)

link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

# -----------------------------------------------------------------------------
# RGB-D camera
# -----------------------------------------------------------------------------

add_library(xchallenge_realsense_camera STATIC
  src/realsense/realsense_camera_bridge.cpp
  src/realsense/mujoco_rgbd_renderer.cpp
  src/realsense/ros_camera_publisher.cpp
)

target_include_directories(xchallenge_realsense_camera
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include
)

target_link_libraries(xchallenge_realsense_camera
  mujoco
  glfw
  OpenGL::GL
)

ament_target_dependencies(xchallenge_realsense_camera
  rclcpp
  sensor_msgs
  builtin_interfaces
)

# -----------------------------------------------------------------------------
# 3D LiDAR
# -----------------------------------------------------------------------------

add_library(xchallenge_lidar STATIC
  src/lidar/mujoco_lidar3d.cpp
  src/lidar/ros_lidar_publisher.cpp
)

target_include_directories(xchallenge_lidar
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include
)

target_link_libraries(xchallenge_lidar
  mujoco
)

ament_target_dependencies(xchallenge_lidar
  rclcpp
  sensor_msgs
)

# -----------------------------------------------------------------------------
# Body odometry
# -----------------------------------------------------------------------------

add_library(xchallenge_odometry STATIC
  src/odometry/mujoco_odometry.cpp
  src/odometry/ros_odometry_publisher.cpp
)

target_include_directories(xchallenge_odometry
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include
)

target_link_libraries(xchallenge_odometry
  mujoco
)

ament_target_dependencies(xchallenge_odometry
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# -----------------------------------------------------------------------------
# Global link dependencies
# -----------------------------------------------------------------------------

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

# -----------------------------------------------------------------------------
# Main simulator
# -----------------------------------------------------------------------------

add_executable(unitree_mujoco
  ${SIM_SRC}
  src/main.cc
)

target_link_libraries(unitree_mujoco
  xchallenge_realsense_camera
  xchallenge_lidar
  xchallenge_odometry
)

ament_target_dependencies(unitree_mujoco
  rclcpp
  geometry_msgs
  rosgraph_msgs
  tf2_ros
)

# -----------------------------------------------------------------------------
# Diagnostic executables
# -----------------------------------------------------------------------------

add_executable(jstest
  src/joystick/jstest.cc
  src/joystick/joystick.cc
)

add_executable(g1_lowstate_test
  test/g1_lowstate_test.cpp
)

target_link_libraries(g1_lowstate_test
  unitree_sdk2
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
