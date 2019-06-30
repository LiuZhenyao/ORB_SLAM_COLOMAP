cmake_minimum_required(VERSION 3.0)

project(pose_graph_reader_for_COLMAP)

set(CMAKE_CXX_STANDARD 11)

#  Set build type if not set
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif() 

include_directories(${PROJECT_SOURCE_DIR}/src)
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

add_library(read_binFile src/read_binFile.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main read_binFile)


