cmake_minimum_required(VERSION 3.8)
project(rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

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

# Install Python modules and scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  rl_controller/rl_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the ONNX policy so it is accessible via get_package_share_directory
install(FILES
  policy.onnx
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
