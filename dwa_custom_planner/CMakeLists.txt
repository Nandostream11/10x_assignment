cmake_minimum_required(VERSION 3.8)
project(dwa_custom_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface Generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ToGoal.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

# Executables
add_executable(nav_goal_dwa src/nav_goal_dwa.cpp)

# Link dependencies
ament_target_dependencies(nav_goal_dwa
  rclcpp
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  tf2
)

# Link generated service interfaces to executables
rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(nav_goal_dwa ${typesupport_target})

# Installation
install(TARGETS
  nav_goal_dwa
  DESTINATION lib/${PROJECT_NAME})

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
