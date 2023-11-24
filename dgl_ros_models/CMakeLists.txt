cmake_minimum_required(VERSION 3.8)
project(dgl_ros_models)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dgl_ros REQUIRED)
find_package(dgl_ros_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)

if(GPD_LIB)
    message(STATUS "Library GPD found in ${GPD_LIB}")
else()
    message(FATAL_ERROR "Library GPD not found")
endif()

find_path(GPD_INCLUDE_DIRS NAMES gpd PATHS /usr/local/include NO_DEFAULT_PATH)

if(GPD_INCLUDE_DIRS)
    message(STATUS "Include directory GPD found in ${GPD_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Include directory GPD not found")
endif()

include_directories(
    include
    ${dgl_ros_INCLUDE_DIRS}
    SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GPD_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

add_executable(gpd
    src/gpd.cpp
)
ament_target_dependencies(gpd
    rclcpp
    dgl_ros_interfaces
    pcl_conversions
    dgl_ros
    sensor_msgs)
target_link_libraries(gpd
    ${Eigen3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GPD_LIB}
    ${dgl_ros_LIBRARIES})

# add_executable(contact_graspnet
# src/contact_graspnet.cpp
# )
# ament_target_dependencies(contact_graspnet
# rclcpp
# dgl_ros_interfaces
# pcl_conversions
# dgl_ros
# sensor_msgs)
# target_link_libraries(contact_graspnet
# ${Eigen3_LIBRARIES}
# ${PCL_LIBRARIES}
# ${GPD_LIB})
install(TARGETS
    gpd

    # contact_graspnet
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()