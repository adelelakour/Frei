cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PCL_TEST)

find_package(PCL 1.13 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PCL_TEST main.cpp)
target_link_libraries (PCL_TEST ${PCL_LIBRARIES})