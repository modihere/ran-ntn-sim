# CMakeLists.txt for UE components (3GPP TS 38.321 / TS 38.331)

add_library(ntn_ue
    timing/timing_engine.cpp
    mac/ue_rach_controller.cpp
    rrc/ue_rrc.cpp
)

target_include_directories(ntn_ue PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ntn_ue
    PUBLIC
    ntn_common
    PRIVATE
    spdlog::spdlog
)

