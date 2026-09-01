cmake_minimum_required(VERSION 3.16)

project(HL_VTD VERSION 0.1.0 LANGUAGES CXX)

enable_testing()

add_executable(hl_vtd
    src/main.cpp
    src/network/network_config.cpp
    src/network/vtd_client.cpp
    src/network/receive/packet_parser.cpp
    src/network/receive/gt_receiver.cpp
    src/network/receive/gt_printer.cpp
    src/network/send/control_sender.cpp
    src/planning/local/path_planner.cpp
    src/planning/planning_process.cpp
    src/planning/global/global_path_planner.cpp
    src/planning/local/local_path_planner.cpp
    src/planning/local/lane_change_planner.cpp
    src/planning/behavior/behavior_planner.cpp
    src/planning/collision/collision_checker.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/data/map/map_matcher.cpp
    src/data/map/static_zones.cpp
    src/data/vehicle/vehicle_config.cpp
    src/control/control_process.cpp
    src/control/config/control_config.cpp
    src/control/lateral/lateral_controller.cpp
    src/control/longitudinal/longitudinal_controller.cpp
    src/control/safety/safety_limiter.cpp
    src/logger/logger.cpp
    src/logger/decision_trace.cpp
    src/utils/csv_utils.cpp
    src/utils/geometry.cpp
    src/utils/math_utils.cpp
    src/data/gt/gt_processor.cpp
    src/data/gt/world_process.cpp
)

add_executable(vtd_path_to_checkpoints
    tools/vtd_path_to_checkpoints.cpp
    tools/path_generation/vtd_path_generator.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
)

add_executable(path_shape_to_route
    tools/path_shape_to_route.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
)

add_executable(checkpoints_to_route
    tools/reference_path_test.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
)

add_executable(extract_static_zones
    tools/map_extraction/extract_static_zones.cpp
    src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
)

add_executable(controller_logic_tests
    tests/controller_logic_tests.cpp
    src/planning/behavior/behavior_planner.cpp
    src/planning/local/lane_change_planner.cpp
    src/planning/local/path_planner.cpp
    src/planning/collision/collision_checker.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/data/map/static_zones.cpp
    src/utils/csv_utils.cpp
    src/utils/geometry.cpp
    src/utils/math_utils.cpp
    src/logger/decision_trace.cpp
)
target_compile_features(controller_logic_tests PRIVATE cxx_std_17)
target_include_directories(controller_logic_tests
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME controller_logic_tests COMMAND controller_logic_tests)

target_compile_features(extract_static_zones PRIVATE cxx_std_17)
target_include_directories(extract_static_zones
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(MSVC)
    target_compile_options(extract_static_zones PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(controller_logic_tests PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(extract_static_zones PRIVATE NOMINMAX)
else()
    target_compile_options(extract_static_zones PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_compile_features(hl_vtd PRIVATE cxx_std_17)
target_include_directories(hl_vtd PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(vtd_path_to_checkpoints PRIVATE cxx_std_17)
target_include_directories(vtd_path_to_checkpoints
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/tools")
target_compile_features(path_shape_to_route PRIVATE cxx_std_17)
target_include_directories(path_shape_to_route
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(checkpoints_to_route PRIVATE cxx_std_17)
target_include_directories(checkpoints_to_route
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(MSVC)
    target_compile_options(hl_vtd PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(vtd_path_to_checkpoints PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(hl_vtd PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vtd_path_to_checkpoints
        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(path_shape_to_route
        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(controller_logic_tests
        PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    target_compile_definitions(hl_vtd PRIVATE NOMINMAX)
    target_link_libraries(hl_vtd PRIVATE ws2_32)
endif()

# Allows both `./build/hl_vtd` and `cd build && ./hl_vtd` to use the default path.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
configure_file(config/network.yaml config/network.yaml COPYONLY)
