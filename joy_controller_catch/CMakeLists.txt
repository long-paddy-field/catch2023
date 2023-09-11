cmake_minimum_required(VERSION 3.8)
project(joy_controller_catch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(principal_interfaces REQUIRED)
find_package(backward_ros REQUIRED)

add_executable(joy_controller_catch src/joy_controller_catch.cpp src/button_manager.cpp)
ament_target_dependencies(joy_controller_catch rclcpp std_msgs principal_interfaces sensor_msgs geometry_msgs)

add_backward(joy_controller_catch)

install(TARGETS
  joy_controller_catch
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
