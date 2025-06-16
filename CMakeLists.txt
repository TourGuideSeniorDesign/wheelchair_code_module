cmake_minimum_required(VERSION 3.8)
project(wheelchair_code_module)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wheelchair_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(rcl_interfaces REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

add_library(obstacle_publisher
  src/obstacle_publisher.cpp
)

ament_target_dependencies(obstacle_publisher
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  Eigen3
  PCL
)

target_include_directories(obstacle_publisher PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

target_link_libraries(obstacle_publisher
  ${PCL_LIBRARIES}
)

add_executable(wheelchair
  src/main.cpp
  src/fan_publisher.cpp
  src/light_publisher.cpp
  src/ref_speed_publisher.cpp
  src/fingerprint_subscriber.cpp
  src/sensors_subscriber.cpp
  src/obstacle_subscriber.cpp
  src/speed_publisher.cpp
  src/uwb_subscriber.cpp
)

add_executable(stopping
  src/main_turning.cpp
  src/fan_publisher.cpp
  src/light_publisher.cpp
  src/ref_speed_publisher.cpp
  src/fingerprint_subscriber.cpp
  src/sensors_subscriber.cpp
  src/obstacle_subscriber.cpp
  src/speed_publisher.cpp
 )

ament_target_dependencies(wheelchair
  rclcpp
  wheelchair_sensor_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
)

ament_target_dependencies(stopping
  rclcpp
  wheelchair_sensor_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_include_directories(wheelchair PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

target_include_directories(stopping PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

target_link_libraries(wheelchair
  obstacle_publisher
  Eigen3::Eigen
)

target_link_libraries(stopping
  obstacle_publisher
  Eigen3::Eigen
)

add_executable(gyro
  src/gyro_turn.cpp
  src/fan_publisher.cpp
  src/light_publisher.cpp
  src/ref_speed_publisher.cpp
  src/fingerprint_subscriber.cpp
  src/sensors_subscriber.cpp
  src/obstacle_subscriber.cpp
  src/speed_publisher.cpp
  src/heading.cpp
)

ament_target_dependencies(gyro
  rclcpp
  wheelchair_sensor_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_include_directories(gyro PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

target_link_libraries(gyro
  obstacle_publisher
  Eigen3::Eigen
)

add_executable(obstacle_publisher_node
  src/obstacle_publisher_node.cpp
)

ament_target_dependencies(obstacle_publisher_node rclcpp)

target_include_directories(obstacle_publisher_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

target_link_libraries(obstacle_publisher_node
  obstacle_publisher
)

# Adding the fan speed executable
add_executable(temp_monitor
        src/temp_monitor.cpp
        src/fan_publisher.cpp
)

# Declare ROS dependencies
ament_target_dependencies(
  temp_monitor
  rclcpp
  wheelchair_sensor_msgs
)

# Include directories (Eigen + your headers)
target_include_directories(
  temp_monitor
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
)

install(TARGETS
  wheelchair
  obstacle_publisher_node
  obstacle_publisher
  stopping
  gyro
  temp_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY headers/ DESTINATION include)


ament_package()
