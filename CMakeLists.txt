cmake_minimum_required(VERSION 3.8)
project(ros2_control_mecanum_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include )

# COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/mecanum_controller_velocity.cpp
  src/odometry.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(
  ${PROJECT_NAME}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  realtime_tools
  tf2
  tf2_msgs
)

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)


install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(ros2_control_odrive_hw REQUIRED)

  ament_add_gmock(
    test_load_mecanum_velocity_controller
    test/test_load_mecanum_velocity_controller.cpp
  )
  target_include_directories(test_load_mecanum_velocity_controller PRIVATE include)
  ament_target_dependencies(test_load_mecanum_velocity_controller
    controller_manager
    geometry_msgs
  )

endif()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)


ament_package()
