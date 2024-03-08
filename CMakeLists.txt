cmake_minimum_required(VERSION 3.8)
project(ros_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# General requirements
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  eigen3_cmake_module
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# # BUILD ###
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/ros_visuals.cpp
  src/spline.cpp
  src/convertions.cpp
  src/data_saver.cpp
  src/math.cpp
  src/profiling.cpp
  src/random_generator.cpp
  src/third_party/tkspline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

install(DIRECTORY include/third_party/
  DESTINATION include/third_party/)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()