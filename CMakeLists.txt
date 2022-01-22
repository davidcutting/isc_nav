cmake_minimum_required(VERSION 3.5)
project(isc_snav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# motion planner
add_executable(motion_planner_node src/motion_planner_node.cpp)

ament_target_dependencies(
  motion_planner_node
  geometry_msgs
  rclcpp
  nav_msgs
)

# path planner
add_executable(path_planner_node src/path_planner_node.cpp)

ament_target_dependencies(
  path_planner_node
  geometry_msgs
  rclcpp
  nav_msgs
)

# waypoint server
add_executable(waypoint_server_node src/waypoint_server_node.cpp)

ament_target_dependencies(
  waypoint_server_node
  geometry_msgs
  rclcpp
  nav_msgs
)


target_include_directories(isc_snav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
      
install(TARGETS isc_snav 
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
