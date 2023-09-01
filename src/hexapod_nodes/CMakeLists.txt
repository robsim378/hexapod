cmake_minimum_required(VERSION 3.8)
project(hexapod_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action)
find_package(sensor_msgs REQUIRED)

# Configuration for the leg movement controller
add_executable(leg_motion_controller src/leg_motion_controller.cpp)
ament_target_dependencies(leg_motion_controller
    "rclcpp"
    "rclcpp_action"
    "hexapod_interfaces")

# Configuration for the leg servo controller
add_executable(leg_servo_controller src/leg_servo_controller.cpp)
ament_target_dependencies(leg_servo_controller
    "rclcpp"
    "hexapod_interfaces")

# Configuration for the leg state broadcaster
add_executable(leg_position_broadcaster src/leg_position_broadcaster.cpp)
ament_target_dependencies(leg_position_broadcaster
    "rclcpp"
    "hexapod_interfaces"
    "sensor_msgs")

# Configuration for the leg step controller
add_executable(leg_step_controller src/leg_step_controller.cpp)
ament_target_dependencies(leg_step_controller
    "rclcpp"
    "rclcpp_action"
    "hexapod_interfaces")

# Configuration for the gait controller
add_executable(gait_controller src/gait_controller.cpp)
ament_target_dependencies(gait_controller
    "rclcpp"
    "rclcpp_action"
    "hexapod_interfaces")

install(TARGETS
    leg_motion_controller
    leg_servo_controller
    leg_position_broadcaster
    leg_step_controller
    gait_controller
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
