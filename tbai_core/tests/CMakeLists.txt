find_package(GTest REQUIRED)
function(add_tbai_test TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})
    target_link_libraries(${TEST_NAME}
        PRIVATE
        tbai_core
        GTest::GTest
        GTest::Main
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

add_tbai_test(tbai_core_test ./src/tbai_core_test.cpp)
