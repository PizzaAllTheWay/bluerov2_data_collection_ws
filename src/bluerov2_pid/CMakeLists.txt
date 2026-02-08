cmake_minimum_required(VERSION 3.12)
project(bluerov2_pid)

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
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(realtime_tools REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
# find_package(manif REQUIRED)
find_package(Sophus REQUIRED)
# find_package(pendulum_utils REQUIRED)
find_package(bluerov_interfaces REQUIRED)
# find_package(cse_actuator_driver REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    # realtime_tools
    eigen3_cmake_module
    Eigen3
    # manif
    #Sophus
    # pendulum_utils
    bluerov_interfaces
    )

set(BLUEROV2_PID_LIB bluerov2_pid)
# add_library(${bluerov2_pid_LIB} SHARED
#         src/bluerov2_pid_node.cpp
#         src/bluerov2_pid.cpp)
add_library(${BLUEROV2_PID_LIB} SHARED
        src/bluerov2_pid_node.cpp
        src/bluerov2_pid.cpp)

target_include_directories(${BLUEROV2_PID_LIB}
  PUBLIC
    # include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    # include
    )

add_definitions(${EIGEN_DEFINITIONS})


ament_target_dependencies(${BLUEROV2_PID_LIB} ${dependencies})

target_link_libraries(${BLUEROV2_PID_LIB} Sophus::Sophus)

rclcpp_components_register_nodes(${BLUEROV2_PID_LIB} "Controller")

set(BLUEROV2_PID_EXE "${BLUEROV2_PID_LIB}_exe")
add_executable(${BLUEROV2_PID_EXE} src/bluerov2_pid_node_main.cpp)
target_link_libraries(${BLUEROV2_PID_EXE} ${BLUEROV2_PID_LIB})
# target_link_libraries(${BLUEROV2_PID_EXE} ${BLUEROV2_PID_LIB} 
#   # cse_actuator_driver::cse_actuator_driver
#   wiringPi)

ament_export_targets(export_${BLUEROV2_PID_LIB} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY include/
    DESTINATION include
)

# install(TARGETS
#   BLUEROV2_PID
#   DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include
# )

install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS ${BLUEROV2_PID_LIB} ${BLUEROV2_PID_EXE}
    EXPORT export_${BLUEROV2_PID_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()
