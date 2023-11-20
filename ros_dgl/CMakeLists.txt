cmake_minimum_required(VERSION 3.8)
project(ros_dgl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_dgl_interfaces REQUIRED)
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

add_library(observer SHARED src/observer.cpp)
ament_target_dependencies(observer
    rclcpp sensor_msgs ros_dgl_interfaces)

add_library(grasp_generator SHARED
    src/grasp_generator.cpp)
ament_target_dependencies(grasp_generator
    rclcpp
    rclcpp_action
    std_msgs
    ros_dgl_interfaces
    sensor_msgs)

add_library(cloud_util SHARED
    src/util/sensors/cloud.cpp)
target_link_libraries(cloud_util
    ${PCL_LIBRARIES}
)

add_library(pose_util SHARED
    src/util/pose.cpp)
target_link_libraries(pose_util
    ${Eigen3_LIBRARIES}
)
ament_target_dependencies(pose_util
    geometry_msgs)

add_executable(gpd_grasp_detection_server
    src/gpd_grasp_detection_server.cpp
)
ament_target_dependencies(gpd_grasp_detection_server rclcpp
    ros_dgl_interfaces pcl_conversions)
target_link_libraries(gpd_grasp_detection_server ${Eigen3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GPD_LIB} grasp_generator observer cloud_util pose_util)

install(TARGETS
gpd_grasp_detection_server
    cloud_util pose_util
    observer
    grasp_generator
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