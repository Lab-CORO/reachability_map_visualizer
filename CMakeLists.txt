cmake_minimum_required(VERSION 3.10)
project(reachability_map_visualizer)

# ------------------------------------------------------------------------------
# Settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

add_executable(load_reachability_map src/load_reachability_map.cpp)
ament_target_dependencies(load_reachability_map
        rclcpp
        std_msgs
        geometry_msgs
)
target_link_libraries(load_reachability_map
        hdf5_dataset
        "${cpp_typesupport_target}"
)

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------

# Install libraries and executable
install(
        TARGETS
        hdf5_dataset
        reachability_map_plugin
        load_reachability_map
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install headers
install(
        DIRECTORY include/
        DESTINATION include/
)

# Install RViz plugin description file
install(
        FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)

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
)

ament_package()











































#cmake_minimum_required(VERSION 3.10)
#project(reachability_map_visualizer)
#
## Required to use Qt and ament
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_AUTOMOC ON)
#
## set(CMAKE_AUTOMOC ON)
## qt5_wrap_cpp(MOC_FILES
##   include/reachability_map_visualizer/hdf5_dataset.h
## )
## ------------------------------------------------------------------------------
## Dependencies
## ------------------------------------------------------------------------------
#
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#
#find_package(rviz_common REQUIRED)
#find_package(rviz_rendering REQUIRED)
#find_package(Qt5 REQUIRED COMPONENTS Widgets)
#find_package(HDF5 REQUIRED)
#find_package(MPI REQUIRED)
##find_package(reachability_map_visualizer REQUIRED)
#find_package(rviz2 REQUIRED)
#find_package(rviz_ogre_vendor REQUIRED)
## find_package(rviz_common REQUIRED)
## find_package(rviz_rendering REQUIRED)
## ------------------------------------------------------------------------------
## Message generation
## ------------------------------------------------------------------------------
##add_compile_options(-fpermissive)
#
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/WsSphere.msg"
#  "msg/WorkSpace.msg"
#  DEPENDENCIES std_msgs geometry_msgs
#)
#
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#
#
## ------------------------------------------------------------------------------
## Include directories
## ------------------------------------------------------------------------------
#
#include_directories(
#  include
#  ${HDF5_INCLUDE_DIRS}
#  ${MPI_CXX_INCLUDE_DIRS}
#)
##set(dependencies
##        message_filters
##        Qt5
##        rviz_common
##        rviz_ogre_vendor
##        rviz_rendering
##        sensor_msgs
##        tf2
##        tf2_ros
##)
## ------------------------------------------------------------------------------
## Libraries
## ------------------------------------------------------------------------------
#
#add_library(hdf5_dataset SHARED
#  src/hdf5_dataset.cpp
#)
#ament_target_dependencies(hdf5_dataset
#  rclcpp
#  std_msgs
#  geometry_msgs
##  reachability_map_visualizer
#)
#target_link_libraries(hdf5_dataset
#  ${HDF5_LIBRARIES}
#  ${MPI_CXX_LIBRARIES}
#)
#
##add_library(reachability_map_plugin SHARED
##  src/reachability_map_visual.cpp
##  src/reachability_map_display.cpp
##)
##target_link_libraries(reachability_map_plugin
##  Qt5::Widgets
##  ${HDF5_LIBRARIES}
##  ${MPI_CXX_LIBRARIES}
##  rviz_ogre_vendor::OgreMain
##  "${cpp_typesupport_target}"
##
##)
##ament_target_dependencies(reachability_map_plugin
##  rclcpp
##  std_msgs
##  geometry_msgs
##  rviz_common
##  rviz_rendering
##)
##target_link_libraries(reachability_map_plugin "${cpp_typesupport_target}")
#
## ------------------------------------------------------------------------------
## Executable
## ------------------------------------------------------------------------------
#
#add_executable(load_reachability_map src/load_reachability_map.cpp)
#ament_target_dependencies(load_reachability_map
#  rclcpp
#  std_msgs
#  geometry_msgs
##  reachability_map_visualizer
#)
#target_link_libraries(load_reachability_map
#  hdf5_dataset
#  "${cpp_typesupport_target}"
#)
##target_link_libraries(load_reachability_map "${cpp_typesupport_target}")
## ------------------------------------------------------------------------------
## Install targets
## ------------------------------------------------------------------------------
#
#install(
#  TARGETS
#    hdf5_dataset
#    load_reachability_map
#    DESTINATION lib/${PROJECT_NAME}
#)
#
#install(DIRECTORY include/
#  DESTINATION include/
#)
#
#
##pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
#
#
#
#
##ament_export_include_directories(include)
## ament_export_libraries(hdf5_dataset reachability_map_plugin)
##ament_export_libraries(reachability_map_plugin)
#
##ament_export_dependencies(
##  rclcpp
##  std_msgs
##  geometry_msgs
##  rviz_common
##  rviz_rendering
##  HDF5
##  MPI
##)
##install(TARGETS reachability_map_plugin
##        ARCHIVE DESTINATION lib
##        LIBRARY DESTINATION lib
##        RUNTIME DESTINATION lib/${PROJECT_NAME}
##)
##
##install(FILES
##        plugin_description.xml
##        DESTINATION share/${PROJECT_NAME}
##)
##ament_export_targets(exportreachability_map_plugin)
## 1) build your plugin
#add_library(reachability_map_plugin SHARED
#        src/reachability_map_visual.cpp
#        src/reachability_map_display.cpp
#)
#ament_target_dependencies(reachability_map_plugin
#        rclcpp std_msgs geometry_msgs
#        rviz_common rviz_rendering
#)
#target_link_libraries(reachability_map_plugin
#        Qt5::Widgets
#        rviz_ogre_vendor::OgreMain
#        ${HDF5_LIBRARIES} ${MPI_CXX_LIBRARIES}
#)
#
## 2) install the library
#install(
#        TARGETS reachability_map_plugin
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME   DESTINATION lib/${PROJECT_NAME}  # if you also build executables
#)
#
## 3) install your plugin XML
#install(
#        FILES plugin_description.xml
#        DESTINATION share/${PROJECT_NAME}
#)
#
## 4) let ament/rviz know about it
#pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
#
#
#ament_package()