cmake_minimum_required(VERSION 3.5)
project(apos_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Sensor.msg"
    "msg/Transponder.msg"
    "msg/Vessel.msg"
    "msg/NED.msg"
    "srv/SetNedToCurrentPos.srv"
    "srv/SetNedToPos.srv"
    DEPENDENCIES std_msgs
    )

ament_package()
