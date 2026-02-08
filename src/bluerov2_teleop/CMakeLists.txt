cmake_minimum_required(VERSION 3.5)
project(bluerov2_teleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Force C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-math-errno -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bluerov_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(Sophus REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    realtime_tools
    eigen3_cmake_module
    Eigen3
    sensor_msgs
    nav_msgs
    # Sophus
    bluerov_interfaces
    )

set(BLUEROV2_TELEOP_LIB bluerov2_teleop)

add_library(${BLUEROV2_TELEOP_LIB} SHARED src/bluerov2_teleop_node.cpp)

target_include_directories(${BLUEROV2_TELEOP_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${BLUEROV2_TELEOP_LIB} ${dependencies})
target_link_libraries(${BLUEROV2_TELEOP_LIB} Sophus::Sophus)


# rclcpp_components_register_nodes(${PROJECT_NAME}
  # "bluerov2_teleop::BlueROVTeleOp")
rclcpp_components_register_nodes(${BLUEROV2_TELEOP_LIB} "BlueROVTeleOpNode")


set(BLUEROV2_TELEOP_EXE "${PROJECT_NAME}_exe")
add_executable(${BLUEROV2_TELEOP_EXE} src/bluerov2_teleop_node_main.cpp)

target_link_libraries(${BLUEROV2_TELEOP_EXE} ${BLUEROV2_TELEOP_LIB})

ament_export_targets(export_${BLUEROV2_TELEOP_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

install(
    DIRECTORY include/
    DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${BLUEROV2_TELEOP_LIB} ${BLUEROV2_TELEOP_EXE}
    EXPORT export_${BLUEROV2_TELEOP_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
