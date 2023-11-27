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
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

set(DEPENDENCIES
    rclcpp geometry_msgs rclcpp_action Eigen3 pcl_ros)
add_library(${PROJECT_NAME} SHARED
    src/agent.cpp src/actor.cpp src/observer.cpp
    src/util/cloud.cpp src/util/geometry.cpp)

#
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
    ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()