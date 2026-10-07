add_library(ntn_ui
    tui/dashboard.cpp
)

target_include_directories(ntn_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ntn_ui PRIVATE spdlog::spdlog)

