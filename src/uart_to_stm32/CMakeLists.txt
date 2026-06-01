cmake_minimum_required(VERSION 3.5)
project(uart_to_stm32)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(serial_comm REQUIRED)

add_library(uart_to_stm32_lib
  src/uart_to_stm32.cpp
)

target_include_directories(uart_to_stm32_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uart_to_stm32_lib
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  serial_comm
)

add_executable(uart_to_stm32_node
  src/uart_to_stm32_node.cpp
)

target_include_directories(uart_to_stm32_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(uart_to_stm32_node
  uart_to_stm32_lib
)

ament_target_dependencies(uart_to_stm32_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  serial_comm
)

install(TARGETS uart_to_stm32_lib
  EXPORT export_uart_to_stm32
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS uart_to_stm32_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_targets(export_uart_to_stm32 HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  serial_comm
)

ament_package()
