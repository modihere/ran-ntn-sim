add_executable(unit_tests
    test_simulator.cpp
    test_geometry.cpp
    test_timing.cpp
)

target_link_libraries(unit_tests
    PRIVATE
    ntn_common
    ntn_geometry
    ntn_timing
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)

