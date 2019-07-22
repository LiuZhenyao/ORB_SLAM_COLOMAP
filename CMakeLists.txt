cmake_minimum_required(VERSION 3.0)

project(pose_graph_reader_for_COLMAP)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

# tell cmake to output binaries here
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR/bin})
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR/lib})

# Set build type if not set
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif() 

# add library first, then add executable
add_subdirectory(src)
add_subdirectory(exe)

# unit/ google test
# enable_testing()
# add_subdirectory(tests)