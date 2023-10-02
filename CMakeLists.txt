cmake_minimum_required(VERSION 3.5)
project(ros_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-DCMAKE_BUILD_TYPE=Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(eigen_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(roslint REQUIRED)
# find_package(roslib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(tf_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(jsk_rviz_plugins REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})


## mesages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES std_msgs
# )
# ## BUILD ###
include_directories(include
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  scripts/risk-eval-cpp
)

## LIBRARY ROS_TOOLS
add_library(ros_tools
  src/ros_visuals.cpp
  src/helpers.cpp
  src/profiling.cpp
  src/data_saver.cpp
  # scripts/
)
# Adds headers based on the cpps added as sources
target_include_directories(ros_tools
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(ros_tools 
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(example
  src/example.cpp
)

target_link_libraries(example 
  ros_tools
  ${Boost_LIBRARIES}
)

## Install

install(TARGETS ros_tools
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS example
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

## Package
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_export_dependencies(geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros visualization_msgs roslib Eigen3)
ament_export_include_directories(include include/${PROJECT_NAME})
ament_export_libraries(ros_tools)

ament_package()