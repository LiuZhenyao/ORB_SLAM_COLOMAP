add_library(pose_graph SHARED
    pose_graph.h   
    pose_graph.cpp)

target_include_directories(pose_graph PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")