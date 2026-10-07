add_executable(simulator
    main.cpp
)

target_link_libraries(simulator
    PRIVATE
    ntn_common
    ntn_geometry
    ntn_timing
    ntn_nr
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)

