cmake_minimum_required(VERSION 3.8)
project(aresdog_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(aresdog_controller_node
  src/aresdog_controller_node.cpp
  src/balance_controller.cpp
  src/leg_kinematics.cpp)

ament_target_dependencies(aresdog_controller_node rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS aresdog_controller_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(aresdog_balance_walk_node
  src/aresdog_balance_walk_node.cpp
  src/balance_controller.cpp
  src/leg_kinematics.cpp)

ament_target_dependencies(aresdog_balance_walk_node rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS aresdog_balance_walk_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_motor_cmd_node
  src/test_motor_cmd_node.cpp)
ament_target_dependencies(test_motor_cmd_node rclcpp sensor_msgs)
install(TARGETS test_motor_cmd_node DESTINATION lib/${PROJECT_NAME})

# Install Python modules and scripts
ament_python_install_package(aresdog_controller)
install(
  PROGRAMS aresdog_controller/jump_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
