# CMakeLists.txt for Satellite gNodeB components (Regenerative Payload)

add_library(ntn_gnb
    rrc/sib19_broadcaster.cpp
    mac/gnb_mac.cpp
)

target_include_directories(ntn_gnb PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ntn_gnb
    PUBLIC
    ntn_common
    ntn_geometry
    PRIVATE
    spdlog::spdlog
)

