# openCV
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )

#eigen3
find_package (Eigen3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_executable(main main.cpp)

target_link_libraries(main
    read_binFile
    ${OpenCV_LIBS}
    Eigen3::Eigen)