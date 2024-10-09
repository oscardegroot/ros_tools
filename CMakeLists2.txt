cmake_minimum_required(VERSION 3.8)
project(ros_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# General requirements
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

# # BUILD ###
include_directories(
  include
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
  src/third_party/clothoid.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
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

# ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()