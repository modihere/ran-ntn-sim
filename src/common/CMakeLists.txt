add_library(ntn_common
    simulator.cpp
)

target_include_directories(ntn_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Optional: link spdlog if we want logging in common layer later
target_link_libraries(ntn_common PRIVATE spdlog::spdlog)
