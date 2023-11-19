cmake_minimum_required(VERSION 3.5)
project(ros_dgl)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# create ament index resource which references the libraries in the binary dir
# set(node_plugins "")

# Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# find_package(rclcpp_components REQUIRED)
find_package(deep_grasp_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros_dgl_interfaces REQUIRED)
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

include_directories(include)
include_directories(
    SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GPD_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

install(
    DIRECTORY include/
    DESTINATION include
)

add_library(grasp_generator SHARED
    src/components/grasp_generator.cpp)

# target_compile_definitions(grasp_generator_server
# PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(grasp_generator
    rclcpp
    rclcpp_action

    # rclcpp_components
    std_msgs
    deep_grasp_msgs
    sensor_msgs)

# rclcpp_components_register_nodes(grasp_generator_server "GraspGeneratorServer")
# set(node_plugins "${node_plugins}GraspGeneratorServer;$<TARGET_FILE:grasp_generator_server>\n")
add_library(observer SHARED src/observer.cpp)
ament_target_dependencies(observer
    rclcpp sensor_msgs ros_dgl_interfaces
)

add_library(sensor_listener SHARED
    src/components/sensor_listener.cpp)

# target_compile_definitions(sensor_listener
# PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(sensor_listener
    rclcpp

    # rclcpp_components
    std_msgs
    deep_grasp_msgs
    sensor_msgs)

# rclcpp_components_register_nodes(sensor_listener "SensorListener")

# set(node_plugins "${node_plugins}SensorListener;$<TARGET_FILE:sensor_listener>\n")
ament_target_dependencies(sensor_listener
    rclcpp

    # rclcpp_components
    std_msgs
    deep_grasp_msgs
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

add_executable(grasp_detection_server
    src/grasp_detection_server.cpp
)
ament_target_dependencies(grasp_detection_server rclcpp
    deep_grasp_msgs pcl_conversions)

target_link_libraries(grasp_detection_server ${Eigen3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GPD_LIB} grasp_generator sensor_listener cloud_util pose_util)

install(TARGETS
    grasp_detection_server
    cloud_util pose_util
    observer
    grasp_generator
    sensor_listener
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION bin)
# if(NOT WIN32)
# ament_environment_hooks(
# "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
# endif()
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()