add_library(read_binFile SHARED
    read_binFile.h   
    read_binFile.cpp)

target_include_directories(read_binFile PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")