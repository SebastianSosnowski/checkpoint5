cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

add_executable(pre_approach_executable src/pre_approach.cpp)
ament_target_dependencies(pre_approach_executable 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  rcl_interfaces
  rclcpp_lifecycle
  lifecycle_msgs
)

add_executable(approach_executable src/pre_approach_v2.cpp)
ament_target_dependencies(approach_executable 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  rcl_interfaces
  rclcpp_lifecycle
  lifecycle_msgs
)

add_executable(approach_srv_server_executable
  src/approach_service_server.cpp
)

ament_target_dependencies(approach_srv_server_executable
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

# =========================
# Link generated service
# =========================

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  approach_executable
  "${cpp_typesupport_target}"
)

target_link_libraries(
  approach_srv_server_executable
  "${cpp_typesupport_target}"
)


install(TARGETS 
	pre_approach_executable
	approach_srv_server_executable
	approach_executable
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
