cmake_minimum_required(VERSION 3.5)
project(ros_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-DCMAKE_BUILD_TYPE=Release)

## PACKAGES ##
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED) # for python example
find_package(Python3 REQUIRED)
set(PYTHON_EXECUTABLE python3)
# find_package(jsk_rviz_plugins REQUIRED)

include(ConfigExtras.cmake)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

## BUILD ###
include_directories(include
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  scripts/risk-eval-cpp
)

## LIBRARY ROS_TOOLS
add_library(${PROJECT_NAME} SHARED
  src/ros_visuals.cpp
  src/helpers.cpp
  src/profiling.cpp
  src/data_saver.cpp
  # scripts/
)
# Adds headers based on the cpps added as sources
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME} 
  ${Boost_LIBRARIES}
)

# EXAMPLE
add_executable(example
  src/example.cpp
)

target_link_libraries(example 
  ros_tools
  ${Boost_LIBRARIES}
)

## INSTALL ##
# The library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# The example
install(TARGETS example
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  ros_tools/example_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME example_py
)

install(PROGRAMS
  ros_tools/ros_visuals_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ros_visuals_py
)

## FINALIZE PACKAGE ##
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(Boost)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(visualization_msgs)

ament_package(CONFIG_EXTRAS "ConfigExtras.cmake")
