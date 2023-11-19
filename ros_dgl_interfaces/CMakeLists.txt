cmake_minimum_required(VERSION 3.5)
project(ros_dgl_interfaces)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
    "srv/ObserveCloud.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
    DEPENDENCIES
    builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()