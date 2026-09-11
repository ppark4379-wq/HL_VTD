cmake_minimum_required(VERSION 3.16)

project(HL_VTD VERSION 0.1.0 LANGUAGES CXX)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_libraries(Threads::Threads)

enable_testing()

add_executable(field_diagnostics_tests tests/field_diagnostics_tests.cpp
    src/logger/field_diagnostics.cpp src/logger/decision_trace.cpp
    src/planning/global/reference_path_generator.cpp src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp)
target_compile_features(field_diagnostics_tests PRIVATE cxx_std_17)
target_include_directories(field_diagnostics_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(MSVC)
    target_compile_options(field_diagnostics_tests PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(field_diagnostics_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
    target_compile_definitions(field_diagnostics_tests PRIVATE NOMINMAX)
endif()
add_test(NAME field_diagnostics_tests COMMAND field_diagnostics_tests)

add_executable(corridor_surface_query_tests tests/corridor_surface_query_tests.cpp)
target_compile_features(corridor_surface_query_tests PRIVATE cxx_std_17)
target_include_directories(corridor_surface_query_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME corridor_surface_query_tests COMMAND corridor_surface_query_tests)

add_executable(vehicle_footprint_tests tests/vehicle_footprint_tests.cpp
    src/utils/geometry.cpp)
target_compile_features(vehicle_footprint_tests PRIVATE cxx_std_17)
target_include_directories(vehicle_footprint_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME vehicle_footprint_tests COMMAND vehicle_footprint_tests)

add_executable(road_elevation_tests tests/road_elevation_tests.cpp
    src/planning/global/reference_path_generator.cpp src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp)
target_compile_features(road_elevation_tests PRIVATE cxx_std_17)
target_include_directories(road_elevation_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME road_elevation_tests COMMAND road_elevation_tests)

add_executable(local_fallback_config_tests tests/local_fallback_config_tests.cpp
    src/control/config/control_config.cpp src/utils/csv_utils.cpp)
target_compile_features(local_fallback_config_tests PRIVATE cxx_std_17)
target_include_directories(local_fallback_config_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME local_fallback_config_tests COMMAND local_fallback_config_tests)
set_tests_properties(local_fallback_config_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(mission_exit_target_tests tests/mission_exit_target_tests.cpp
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp src/utils/csv_utils.cpp src/utils/math_utils.cpp)
target_compile_features(mission_exit_target_tests PRIVATE cxx_std_17)
target_include_directories(mission_exit_target_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME mission_exit_target_tests COMMAND mission_exit_target_tests)

add_executable(gt_receiver_tests tests/gt_receiver_tests.cpp
    src/network/receive/gt_receiver.cpp src/network/receive/packet_parser.cpp
    src/network/vtd_client.cpp)
target_compile_features(gt_receiver_tests PRIVATE cxx_std_17)
target_include_directories(gt_receiver_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(WIN32)
    target_link_libraries(gt_receiver_tests PRIVATE ws2_32)
endif()
add_test(NAME gt_receiver_tests COMMAND gt_receiver_tests)

set(CORRIDOR_SOURCES src/planning/local/corridor_lattice.cpp src/data/map/precomputed_lane_network.cpp)
add_executable(corridor_lattice_tests tests/corridor_lattice_tests.cpp ${CORRIDOR_SOURCES}
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp
    src/planning/collision/collision_checker.cpp src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(corridor_lattice_tests PRIVATE cxx_std_17)
target_include_directories(corridor_lattice_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME corridor_lattice_tests COMMAND corridor_lattice_tests)
add_test(NAME physically_feasible_map_corner COMMAND corridor_lattice_tests --tight-map-corner)
add_test(NAME progress_recovery_selection COMMAND corridor_lattice_tests --progress-recovery-selection)
add_test(NAME bounded_corridor_reentry COMMAND corridor_lattice_tests --bounded-reentry)
add_test(NAME lost_corridor_recovery COMMAND corridor_lattice_tests --lost-corridor-recovery)
add_test(NAME no_cosmetic_replan COMMAND corridor_lattice_tests --no-cosmetic-replan)
add_test(NAME center_first_gateway_route COMMAND corridor_lattice_tests --center-first-global)
add_test(NAME sole_gateway_safe_approach COMMAND corridor_lattice_tests --sole-gateway-approach)
add_test(NAME gateway_prefix_deadline COMMAND corridor_lattice_tests --gateway-prefix)
add_test(NAME multiple_gateway_queue COMMAND corridor_lattice_tests --multi-gateway-queue)
add_test(NAME clear_lane_center_return COMMAND corridor_lattice_tests --clear-center-return)
add_test(NAME extended_gateway_preview COMMAND corridor_lattice_tests --extended-gateway-preview)
add_test(NAME cached_global_priority COMMAND corridor_lattice_tests --cached-global-priority)
set_tests_properties(lost_corridor_recovery PROPERTIES TIMEOUT 10)
set_tests_properties(bounded_corridor_reentry PROPERTIES TIMEOUT 15)
add_executable(global_route_obstacle_tests tests/global_route_obstacle_tests.cpp ${CORRIDOR_SOURCES}
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp
    src/planning/collision/collision_checker.cpp src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(global_route_obstacle_tests PRIVATE cxx_std_17)
target_include_directories(global_route_obstacle_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME global_route_obstacle_tests COMMAND global_route_obstacle_tests)
add_test(NAME dense_global_route_regression COMMAND global_route_obstacle_tests --dense-global)
add_test(NAME expanded_global_route_regression COMMAND global_route_obstacle_tests --expanded-global)
add_executable(gateway_preview_reachability_tests tests/gateway_preview_reachability_tests.cpp ${CORRIDOR_SOURCES}
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp src/planning/collision/collision_checker.cpp
    src/data/map/map_loader.cpp src/utils/csv_utils.cpp src/utils/math_utils.cpp
    src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(gateway_preview_reachability_tests PRIVATE cxx_std_17)
target_include_directories(gateway_preview_reachability_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME gateway_preview_reachability COMMAND gateway_preview_reachability_tests)
add_executable(local_corridor_fallback_tests tests/local_corridor_fallback_tests.cpp ${CORRIDOR_SOURCES}
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp src/planning/collision/collision_checker.cpp
    src/data/map/map_loader.cpp src/utils/csv_utils.cpp src/utils/math_utils.cpp
    src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(local_corridor_fallback_tests PRIVATE cxx_std_17)
target_include_directories(local_corridor_fallback_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME local_corridor_fallback_tests COMMAND local_corridor_fallback_tests)
set_tests_properties(local_corridor_fallback_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 40)
add_executable(reference_reconnect_cache_tests tests/reference_reconnect_cache_tests.cpp ${CORRIDOR_SOURCES}
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp src/planning/collision/collision_checker.cpp
    src/data/map/map_loader.cpp src/utils/csv_utils.cpp src/utils/math_utils.cpp
    src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(reference_reconnect_cache_tests PRIVATE cxx_std_17)
target_include_directories(reference_reconnect_cache_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME reference_reconnect_cache_tests COMMAND reference_reconnect_cache_tests)
add_executable(replay_corridor_scene tools/verification/replay_corridor_scene.cpp ${CORRIDOR_SOURCES}
    src/planning/planning_process.cpp src/planning/behavior/behavior_planner.cpp
    src/planning/local/lane_change_planner.cpp src/planning/local/local_path_planner.cpp
    src/data/map/map_matcher.cpp src/data/map/static_zones.cpp src/data/gt/gt_processor.cpp
    src/planning/global/mission_corridor.cpp src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp src/planning/collision/collision_checker.cpp
    src/data/map/map_loader.cpp src/control/config/control_config.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp src/utils/geometry.cpp src/logger/decision_trace.cpp)
target_compile_features(replay_corridor_scene PRIVATE cxx_std_17)
target_include_directories(replay_corridor_scene PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
foreach(corridor_regression IN ITEMS
        surface-permission
        alignment-deadline
        vehicle-footprint-width
        m05-tail-regression
        m08-obstacle-replay
        m08-gateway-replay
        m08-final-gateway-replay
        m08-corridor-handoff-replay
        m09-exhausted-maneuver-replay
        m09-compact-obstacle-replay
        m11-transition-commitment
        m14-guide-recovery
        m15-exact-gateway-target
        observed-global-replan
        all-scenario-handoff-audit)
    string(REPLACE "-" "_" corridor_test_name "${corridor_regression}")
    add_test(NAME corridor_${corridor_test_name}
             COMMAND corridor_lattice_tests --${corridor_regression})
    set_tests_properties(corridor_${corridor_test_name}
                         PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()

add_executable(check_precomputed_network
    tools/map_precompute/check_precomputed_network.cpp
    src/data/map/precomputed_lane_network.cpp
    src/utils/csv_utils.cpp)
target_compile_features(check_precomputed_network PRIVATE cxx_std_17)
target_include_directories(check_precomputed_network PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
if(NOT MSVC)
    target_compile_options(check_precomputed_network PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(hl_vtd
    ${CORRIDOR_SOURCES}
    src/main.cpp
    src/network/network_config.cpp
    src/network/vtd_client.cpp
    src/network/receive/packet_parser.cpp
    src/network/receive/gt_receiver.cpp
    src/network/receive/rdb_clock.cpp
    src/network/receive/gt_printer.cpp
    src/network/send/control_sender.cpp
    src/planning/local/path_planner.cpp
    src/planning/planning_process.cpp
    src/planning/global/global_path_planner.cpp
    src/planning/global/mission_corridor.cpp
    src/planning/global/mission_precompute.cpp
    src/data/map/mission_map_export.cpp
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
    src/logger/field_diagnostics.cpp
    src/logger/decision_trace.cpp
    src/visualization/visualization_logger.cpp
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

# Offline XODR geometry/link export; no mission snapshot or VTD socket.
add_executable(export_mission_corridors tools/verification/export_mission_corridors.cpp
    src/data/map/mission_map_export.cpp
    src/planning/global/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp src/utils/math_utils.cpp)
target_compile_features(export_mission_corridors PRIVATE cxx_std_17)
target_include_directories(export_mission_corridors PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_definitions(hl_vtd PRIVATE HL_VTD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(extract_static_zones
    tools/map_extraction/extract_static_zones.cpp
    src/planning/global/reference_path_generator.cpp
    src/planning/local/path_planner.cpp
    src/data/map/map_loader.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
)

add_executable(controller_logic_tests
    ${CORRIDOR_SOURCES}
    tests/controller_logic_tests.cpp
    src/network/receive/rdb_clock.cpp
    src/network/vtd_client.cpp
    src/planning/behavior/behavior_planner.cpp
    src/planning/planning_process.cpp
    src/planning/local/lane_change_planner.cpp
    src/planning/local/local_path_planner.cpp
    src/planning/local/path_planner.cpp
    src/planning/collision/collision_checker.cpp
    src/planning/global/reference_path_generator.cpp
    src/planning/global/mission_corridor.cpp
    src/data/gt/gt_processor.cpp
    src/data/map/map_matcher.cpp
    src/data/map/map_loader.cpp
    src/data/map/static_zones.cpp
    src/control/control_process.cpp
    src/control/lateral/lateral_controller.cpp
    src/control/longitudinal/longitudinal_controller.cpp
    src/control/safety/safety_limiter.cpp
    src/utils/csv_utils.cpp
    src/utils/geometry.cpp
    src/utils/math_utils.cpp
    src/logger/decision_trace.cpp
)
target_compile_features(controller_logic_tests PRIVATE cxx_std_17)
if(NOT MSVC)
    target_compile_options(controller_logic_tests PRIVATE -UNDEBUG)
endif()
target_include_directories(controller_logic_tests
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
add_test(NAME controller_logic_tests COMMAND controller_logic_tests)
add_test(NAME school_zone_approach_regression COMMAND controller_logic_tests --school-zone-approach)
add_test(NAME crosswalk_indicator_regression COMMAND controller_logic_tests --crosswalk-indicator)
add_test(NAME gateway_route_handoff COMMAND controller_logic_tests --gateway-route-handoff)
add_test(NAME gateway_queue_detection COMMAND controller_logic_tests --gateway-queue)
add_test(NAME progress_recovery_handoff COMMAND controller_logic_tests --progress-recovery-handoff)
add_test(NAME simulation_contact_progress COMMAND controller_logic_tests --contact-progress)
add_test(NAME simulation_recovery_stop_edges COMMAND controller_logic_tests --recovery-stop-edges)
add_test(NAME consumed_path_steering COMMAND controller_logic_tests --consumed-path)
add_test(NAME signal_stop_policy COMMAND controller_logic_tests --signal-stop-policy)
add_test(NAME recovery_signal_approach COMMAND controller_logic_tests --recovery-signal-approach)
set_tests_properties(crosswalk_indicator_regression PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME dense_static_speed_regression COMMAND controller_logic_tests --dense-static-speed)
add_test(NAME avoidance_safe_handoff_regression COMMAND controller_logic_tests --safe-tail-stopline)
set_tests_properties(controller_logic_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

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
    target_compile_options(path_shape_to_route PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(checkpoints_to_route PRIVATE /W4 /permissive- /utf-8)
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
    target_compile_definitions(controller_logic_tests PRIVATE NOMINMAX)
    target_link_libraries(controller_logic_tests PRIVATE ws2_32)
endif()

# Allows both `./build/hl_vtd` and `cd build && ./hl_vtd` to use the default path.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
configure_file(config/network.yaml config/network.yaml COPYONLY)
