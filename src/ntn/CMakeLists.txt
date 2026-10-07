add_library(ntn_geometry
    geometry/orbit_propagator.cpp
    geometry/propagation.cpp
)

target_include_directories(ntn_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntn_geometry PRIVATE ntn_common)

