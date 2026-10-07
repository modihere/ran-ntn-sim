add_library(ntn_nr
    rach/rach_controller.cpp
)

target_include_directories(ntn_nr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntn_nr 
    PRIVATE 
    ntn_common 
    ntn_timing 
    spdlog::spdlog
)
