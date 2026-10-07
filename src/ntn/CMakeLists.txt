add_library(ntn_geometry
    geometry/orbit_propagator.cpp
    geometry/propagation.cpp
)

add_library(ntn_timing
    timing/timing_engine.cpp
)

target_include_directories(ntn_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntn_geometry PRIVATE ntn_common)

target_include_directories(ntn_timing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntn_timing PRIVATE ntn_common)


