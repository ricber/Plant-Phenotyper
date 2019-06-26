cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(data_analysis)

find_package(PCL 1.3 REQUIRED) # COMPONENTS common io

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.9 /usr/include/eigen3/)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(segmentation segmentation.cpp)
target_link_libraries(segmentation ${PCL_LIBRARIES})
