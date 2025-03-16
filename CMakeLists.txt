cmake_minimum_required(VERSION 3.5)
project(project_arp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${airsim_interfaces_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(IMU src/IMU.cpp)
add_executable(Encoder src/Encoder.cpp)
add_executable(DriveTrain src/DriveTrain.cpp)
add_executable(PathPlanner src/PathPlanner.cpp)
add_executable(StateEstimator src/StateEstimator.cpp)
add_executable(Camera src/Camera.cpp)
ament_target_dependencies(IMU rclcpp sensor_msgs)
ament_target_dependencies(Encoder rclcpp sensor_msgs)
ament_target_dependencies(DriveTrain rclcpp airsim_interfaces geometry_msgs)
ament_target_dependencies(PathPlanner rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(StateEstimator rclcpp sensor_msgs nav_msgs)
ament_target_dependencies(Camera rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  IMU
  Encoder
  DriveTrain
  PathPlanner
  StateEstimator
  Camera
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()