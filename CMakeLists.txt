cmake_minimum_required(VERSION 3.5)
project(isc_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# path planner
add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(
  path_planner_node
  geometry_msgs
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(path_planner_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# pure pursuit
add_executable(pure_pursuit
  src/pure_pursuit.cpp
  src/pure_pursuit_node.cpp
)
ament_target_dependencies(
  pure_pursuit
  geometry_msgs
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(pure_pursuit  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS path_planner_node pure_pursuit
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_my_test test/test.cpp)

  # Link isc_nav files
  target_include_directories(${PROJECT_NAME}_my_test PUBLIC src/)

  target_include_directories(${PROJECT_NAME}_my_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  # We need to ament link because we use isc_nav files which need ros
  ament_target_dependencies(
  ${PROJECT_NAME}_my_test
  geometry_msgs
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
endif()

ament_package()
