cmake_minimum_required(VERSION 3.8)
project(hexapod_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Configuration for the leg movement controller
add_executable(leg_movement_controller src/leg_movement_controller.cpp)
ament_target_dependencies(leg_movement_controller
  "rclcpp"
  "hexapod_interfaces")

# Configuration for the leg servo controller
add_executable(leg_servo_controller src/leg_servo_controller.cpp)
ament_target_dependencies(leg_servo_controller
  "rclcpp"
  "hexapod_interfaces")

# Configuration for the leg state broadcaster
add_executable(leg_state_broadcaster src/leg_state_broadcaster.cpp)
ament_target_dependencies(leg_state_broadcaster
  "rclcpp"
  "hexapod_interfaces"
  "sensor_msgs")

add_executable(main_movement_controller src/main_movement_controller.cpp)
ament_target_dependencies(main_movement_controller
  "rclcpp"
  "hexapod_interfaces")
  
install(TARGETS
  leg_movement_controller
  leg_servo_controller
  leg_state_broadcaster
  main_movement_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME})

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
