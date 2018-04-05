cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cyl_seg)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
endif()

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
