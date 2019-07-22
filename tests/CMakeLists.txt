add_subdirectory(
    /usr/src/gtest
    ${PROJECT_BINARY_DIR}/gtest)

include(CTest)

# set(test ${PROJECT_NAME}_test)

add_executable(test test.cpp)

target_link_libraries(test
    gtest gtest_main) 

add_test(test tests/test
)