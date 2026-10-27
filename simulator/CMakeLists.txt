add_executable(simulator
    main.cpp
)

target_link_libraries(simulator
    PRIVATE
    ntn_common
    ntn_geometry
    ntn_ue
    ntn_gnb
    ntn_ui
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)
