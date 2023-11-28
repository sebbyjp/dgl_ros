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

# add_compile_options(-fPIC)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPYTHON_INCLUDE_DIR=/usr/include/python3.10
# -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu -DPYTHON_EXECUTABLE=$(which python3)")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC $(python3 -m pybind11 --includes)")
# Use minimum-size optimization for pybind11 bindings
add_compile_options("-Os")

# configure pybind11 install for use by downstream packages in install space

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(PYBIND11_INSTALL ON CACHE INTERNAL "Install pybind11")
set(CMAKE_INSTALL_INCLUDEDIR include/${PROJECT_NAME})
set(PYBIND11_CMAKECONFIG_INSTALL_DIR share/${PROJECT_NAME}/cmake
    CACHE INTERNAL "install path for pybind11 cmake files")
add_subdirectory(pybind11)
find_package(Python REQUIRED COMPONENTS Interpreter Development)

# find_package(pybind11 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dgl_ros REQUIRED)
find_package(dgl_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find_package(pybind11 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)

# find_package(Python3 COMPONENTS Interpreter Development.Embecd)
set(DEPENDENCIES
    rclcpp geometry_msgs rclcpp_action Eigen3 pcl_ros tf2_ros tf2_eigen)

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
    ${Python_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

add_library(util SHARED
    src/util/tf.cpp
    src/util/geometry.cpp
    src/util/cloud.cpp)
ament_target_dependencies(util PUBLIC ${DEPENDENCIES})
target_include_directories(util PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/dgl_ros>")

add_executable(gpd
    src/gpd.cpp
)
ament_target_dependencies(gpd PUBLIC
    rclcpp
    dgl_ros_interfaces
    pcl_conversions
    dgl_ros
    sensor_msgs)
target_link_libraries(gpd PUBLIC
    util
    ${Eigen3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GPD_LIB})

target_include_directories(gpd PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(contact_graspnet
    src/contact_graspnet.cpp
)
ament_target_dependencies(contact_graspnet PUBLIC
    rclcpp
    dgl_ros_interfaces
    pcl_conversions
    PCL
    dgl_ros
    sensor_msgs)
target_link_libraries(contact_graspnet PUBLIC
    util

    # pybind11
    # Python3
    ${Python_LIBRARIES}
    ${pybind11_LIBRARIES})
target_include_directories(contact_graspnet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# Python_add_library(contact_graspnet_py SHARED src/contact_graspnet.cpp)
install(TARGETS
    gpd
    contact_graspnet
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)
install(TARGETS util
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()