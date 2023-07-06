cmake_minimum_required(VERSION 2.8.3)
project(ros_tools)

add_compile_options(-std=c++11)
add_compile_options(-fstack-protector)
set(CMAKE_CXX_FLAGS "-O3 -Wall -DCMAKE_BUILD_TYPE=Release")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  roslint
  roslib
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem thread)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS eigen_conversions geometry_msgs roscpp sensor_msgs std_msgs tf tf_conversions visualization_msgs roslib
  DEPENDS Boost
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ros_tools
)

include_directories(include)

# ## BUILD ###
include_directories(include
  ${catkin_INCLUDE_DIRS}
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