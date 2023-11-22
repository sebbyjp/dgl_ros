cmake_minimum_required(VERSION 3.8)
project(ros_dgl_moveit)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(ros_dgl_interfaces REQUIRED)

include_directories(include)

add_library(deep_grasp_pose src/stages/deep_grasp_pose.cpp)
ament_target_dependencies(deep_grasp_pose
  moveit_task_constructor_core moveit_visual_tools rclcpp ros_dgl_interfaces
)

install(
  TARGETS
  deep_grasp_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
