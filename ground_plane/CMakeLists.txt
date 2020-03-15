cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

set (CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ground_plane src/ground_plane.cpp)
target_link_libraries (ground_plane ${PCL_LIBRARIES})
target_include_directories(ground_plane PUBLIC include)