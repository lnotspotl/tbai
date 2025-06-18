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

add_tbai_test(test_env src/test_env.cpp)
add_tbai_test(test_config src/test_config.cpp)
