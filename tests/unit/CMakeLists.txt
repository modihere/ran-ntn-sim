add_executable(unit_tests
    test_simulator.cpp
)

target_link_libraries(unit_tests
    PRIVATE
    ntn_common
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
