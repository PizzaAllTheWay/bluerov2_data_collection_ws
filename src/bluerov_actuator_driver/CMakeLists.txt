cmake_minimum_required(VERSION 3.5)
project(bluerov_actuator_driver)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Di2c -O3)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(pendulum_utils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(bluerov_interfaces REQUIRED)
# find_package(wiringPi)

# find_library(wiringPi_LIB wiringPi)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    lifecycle_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    eigen3_cmake_module
    Eigen3
    pendulum_utils
    realtime_tools
    bluerov_interfaces
)

set(BLUEROV_ACTUATOR_DRIVER_LIB bluerov_actuator_driver)
# add_library(${SHIP_OBSERVER_LIB} SHARED
#         src/ship_observer_node.cpp
#         src/ship_observer.cpp)
add_library(${BLUEROV_ACTUATOR_DRIVER_LIB} SHARED
        src/bluerov_actuator_driver_node.cpp
        src/bluerov_actuator_driver.cpp
        src/pixhawk.cpp)


# find_library(wiringPi_LIB wiringPi)
# find_library(i2c_LIB i2c)


target_include_directories(${BLUEROV_ACTUATOR_DRIVER_LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${BLUEROV_ACTUATOR_DRIVER_LIB} ${dependencies})

rclcpp_components_register_nodes(${BLUEROV_ACTUATOR_DRIVER_LIB} "BlueROVActuatorDriver")
# rclcpp_components_register_nodes(ship_observer "Observer")

set(BLUEROV_ACTUATOR_DRIVER_EXE "${BLUEROV_ACTUATOR_DRIVER_LIB}_exe")
add_executable(${BLUEROV_ACTUATOR_DRIVER_EXE} src/bluerov_actuator_driver_node_main.cpp)
target_link_libraries(${BLUEROV_ACTUATOR_DRIVER_EXE}
  ${BLUEROV_ACTUATOR_DRIVER_LIB}
  # wiringPi
  # -lwiringPi
)


ament_export_targets(export_${BLUEROV_ACTUATOR_DRIVER_LIB} HAS_LIBRARY_TARGET)
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

install(
    TARGETS ${BLUEROV_ACTUATOR_DRIVER_LIB} ${BLUEROV_ACTUATOR_DRIVER_EXE}
    EXPORT export_${BLUEROV_ACTUATOR_DRIVER_LIB}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

# ament_export_include_directories(include)
# ament_export_libraries(${BLUEROV_ACTUATOR_DRIVER_LIB})

ament_package()
