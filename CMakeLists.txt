cmake_minimum_required(VERSION 3.8)
project(go2_inekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs)
find_package(sensor_msgs)
find_package(std_msgs)
find_package(geometry_msgs)


# for future C++ nodes
# install(TARGETS
#   <name>
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python executables
file(GLOB SCRIPTS_LIST 
    scripts/calc_bias_imu.py
    scripts/fake_imu.py
    scripts/*.py)

install(PROGRAMS
  ${SCRIPTS_LIST}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# # for later
# # Install config files.
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )


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
