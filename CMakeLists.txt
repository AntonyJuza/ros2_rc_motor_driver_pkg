cmake_minimum_required(VERSION 3.8)
project(humanoid_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Try to find pigpiod_if2 / pigpiod_if / pigpio (optional). If present, build the Cytron controller node.
find_path(PIGPIO_INCLUDE pigpio.h)
find_library(PIGPIOD_LIB NAMES pigpiod_if2 pigpiod_if pigpio)
if(PIGPIO_INCLUDE AND PIGPIOD_LIB)
  set(PIGPIO_FOUND TRUE)
  set(PIGPIO_LIBRARY ${PIGPIOD_LIB})
else()
  set(PIGPIO_FOUND FALSE)
endif()

# Cytron MDDRC10 Motor Controller Node
if(PIGPIO_FOUND)
  add_executable(cytron_controller_node 
    src/cytron_controller_node.cpp
    src/cytron_controller.cpp
  )
  ament_target_dependencies(cytron_controller_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
  )

  target_include_directories(cytron_controller_node
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_include_directories(cytron_controller_node PRIVATE ${PIGPIO_INCLUDE})
  target_link_libraries(cytron_controller_node ${PIGPIO_LIBRARY})
else()
  message(WARNING "pigpio not found: cytron_controller_node will not be built. Install pigpio development files (libpigpio-dev) to enable it.")
endif()

# Encoder Reader Node
add_executable(encoder_node 
  src/encoder_node.cpp
)
ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
  sensor_msgs
)
if(PIGPIO_FOUND)
  target_include_directories(encoder_node PRIVATE ${PIGPIO_INCLUDE})
  target_link_libraries(encoder_node ${PIGPIO_LIBRARY})
else()
  message(WARNING "pigpio not found: encoder_node will be built without pigpio support. Install libpigpio-dev to enable it.")
endif()

# Differential Drive Controller
add_executable(diff_drive_controller 
  src/diff_drive_controller.cpp
)
ament_target_dependencies(diff_drive_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
set(_install_targets encoder_node diff_drive_controller)
if(TARGET cytron_controller_node)
  list(APPEND _install_targets cytron_controller_node)
endif()

install(TARGETS
  ${_install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()