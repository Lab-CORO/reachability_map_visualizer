cmake_minimum_required(VERSION 3.10)
project(reachability_map_visualizer)

# ------------------------------------------------------------------------------
# Settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

# Set CMake policy for Python finder modules
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(HDF5 REQUIRED)
find_package(MPI REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# ------------------------------------------------------------------------------
# Message generation
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/WsSphere.msg"
        "msg/WorkSpace.msg"
        DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
include_directories(
        include
        ${HDF5_INCLUDE_DIRS}
        ${MPI_CXX_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------

# Dataset library
add_library(hdf5_dataset SHARED
        src/hdf5_dataset.cpp
)
ament_target_dependencies(hdf5_dataset
        rclcpp
        std_msgs
        geometry_msgs
)
target_link_libraries(hdf5_dataset
        ${HDF5_LIBRARIES}
        ${MPI_CXX_LIBRARIES}
)

# RViz2 Plugin
add_library(reachability_map_plugin SHARED
        src/reachability_map_visual.cpp
        src/reachability_map_display.cpp
)
ament_target_dependencies(reachability_map_plugin
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        rviz_common
        rviz_rendering
)
target_link_libraries(reachability_map_plugin
        Qt5::Widgets
        ${HDF5_LIBRARIES}
        ${MPI_CXX_LIBRARIES}
        rviz_ogre_vendor::OgreMain
        "${cpp_typesupport_target}"
)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------

# Voxel map
add_executable(load_reachability_voxel src/load_reachability_voxel.cpp)
ament_target_dependencies(load_reachability_voxel
        rclcpp
        std_msgs
        visualization_msgs
        geometry_msgs
        sensor_msgs
)
target_link_libraries(load_reachability_voxel
        hdf5_dataset
        "${cpp_typesupport_target}"
)
# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------

# Install libraries
install(
        TARGETS
        hdf5_dataset
        reachability_map_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install executable
install(
        TARGETS
        load_reachability_voxel
        DESTINATION lib/${PROJECT_NAME}
)
# Install headers
install(
        DIRECTORY include/
        DESTINATION include
)

# Install RViz plugin description file
install(
        FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)

# Install launch files (commented out - launch directory doesn't exist)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(reachability_map_plugin)
ament_export_dependencies(
        rclcpp
        std_msgs
        geometry_msgs
        rviz_common
        rviz_rendering
        HDF5
        MPI
        sensor_msgs
)

ament_package()



