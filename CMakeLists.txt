cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointcloud_utils)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convert_bin2ascii src/convert_bin2ascii.cpp src/pointcloud.cpp)
add_executable (convert_ascii2bin src/convert_ascii2bin.cpp src/pointcloud.cpp)
target_link_libraries (convert_bin2ascii ${PCL_LIBRARIES})
target_link_libraries (convert_ascii2bin ${PCL_LIBRARIES})
