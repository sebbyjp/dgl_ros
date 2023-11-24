cmake_minimum_required(VERSION 3.8)
project(dgl_ros)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find_package(dgl_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Util
add_library(cloud_util SHARED src/util/cloud.cpp)
target_link_libraries(cloud_util ${PCL_LIBRARIES})
target_include_directories(cloud_util PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_library(geometry_util SHARED src/util/geometry.cpp)
target_link_libraries(geometry_util Eigen3::Eigen)
ament_target_dependencies(geometry_util geometry_msgs)
target_include_directories(geometry_util PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# add_library(observer SHARED src/observer.cpp)
# ament_target_dependencies(observer rclcpp)

# target_include_directories(observer PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# add_library(actor SHARED src/actor.cpp)
# ament_target_dependencies(actor rclcpp rclcpp_action)
# target_include_directories(actor PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# add_library(agent SHARED src/agent.cpp)
# target_link_libraries(agent PUBLIC actor observer)
# ament_target_dependencies(agent PUBLIC rclcpp rclcpp_action)
# target_include_directories(agent PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} INTERFACE
    rclcpp
    rclcpp_action
)
install(TARGETS
    cloud_util
    geometry_util
    ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)
ament_export_dependencies(
    rclcpp rclcpp_action
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()