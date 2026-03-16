cmake_minimum_required(VERSION 3.8)
project(bno055)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_msgs REQUIRED)

# ── wjwwood/serial library (built inline as a static library) ──
set(SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/serial)
add_library(serial_lib STATIC
  ${SERIAL_DIR}/src/serial.cc
  ${SERIAL_DIR}/src/impl/unix.cc
  ${SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc
)
target_include_directories(serial_lib PUBLIC
  ${SERIAL_DIR}/include
)
target_link_libraries(serial_lib rt pthread)

# Include directories
include_directories(
  include
  ${SERIAL_DIR}/include
)

# C++ Lifecycle Node executable
add_executable(bno055_lifecycle_node
  src/main.cpp
  src/bno055_node.cpp
  src/sensor_service.cpp
  src/i2c_connector.cpp
  src/uart_connector.cpp
)

ament_target_dependencies(bno055_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  example_interfaces
  robot_msgs
)
target_link_libraries(bno055_lifecycle_node serial_lib)

# Install C++ executables
install(TARGETS
  bno055_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package with metadata (for entry_point executable)
ament_python_install_package(${PROJECT_NAME})

# Install Python executable script
install(PROGRAMS scripts/bno055
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY bno055/params/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
