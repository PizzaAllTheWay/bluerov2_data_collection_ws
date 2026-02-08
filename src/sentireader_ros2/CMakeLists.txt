cmake_minimum_required(VERSION 3.5)
project(sentireader_ros2)

find_package(ament_cmake REQUIRED)

execute_process(
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

install(
  PROGRAMS target/release/sentireader_ros2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
