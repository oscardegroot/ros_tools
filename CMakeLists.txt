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

find_package(eigen_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roslint REQUIRED)
find_package(roslib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsk_rviz_plugins REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem thread)

add_definitions(${EIGEN_DEFINITIONS})


## mesages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
# ## BUILD ###
include_directories(include
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  scripts/risk-eval-cpp
)

add_library(ros_tools
  src/ros_visuals.cpp
  src/helpers.cpp
  src/profiling.cpp
  src/data_saver.cpp
  scripts/
)
# Adds headers based on the cpps added as sources
target_include_directories(ros_tools
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# How to get dependencies in here properly?
ament_target_dependencies(ros_tools eigen_conversions)


add_dependencies(ros_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Test Case ####
install(TARGETS ros_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/ros_tools/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(risk_computation_node
  scripts/risk-eval-cpp/risk_computation_node.cpp
)

target_link_libraries(risk_computation_node
  ros_tools
  ${catkin_LIBRARIES}
)

add_dependencies(risk_computation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Package
ament_export_dependencies(eigen_conversions geometry_msgs roscpp sensor_msgs std_msgs tf tf_conversions visualization_msgs roslib)
ament_export_include_directories(include include/${PROJECT_NAME})
ament_export_libraries(ros_tools)

ament_package()