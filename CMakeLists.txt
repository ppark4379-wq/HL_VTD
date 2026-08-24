cmake_minimum_required(VERSION 3.16)

project(HL_VTD VERSION 0.1.0 LANGUAGES CXX)

add_executable(hl_vtd
    src/main.cpp
    src/network/network_config.cpp
    src/network/vtd_client.cpp
    src/network/receive/packet_parser.cpp
    src/network/receive/gt_receiver.cpp
    src/network/receive/gt_printer.cpp
    src/network/send/control_sender.cpp
    src/planning/path_planner.cpp
    src/planning/planning_process.cpp
    src/planning/reference_path_generator.cpp
    src/data/map/map_loader.cpp
    src/control/controller.cpp
    src/control/control_process.cpp
    src/control/control_config.cpp
    src/logger/logger.cpp
    src/utils/csv_utils.cpp
    src/utils/math_utils.cpp
    src/data/gt/gt_processor.cpp
)

target_compile_features(hl_vtd PRIVATE cxx_std_17)
target_include_directories(hl_vtd PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(MSVC)
    target_compile_options(hl_vtd PRIVATE /W4 /permissive-)
else()
    target_compile_options(hl_vtd PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    target_link_libraries(hl_vtd PRIVATE ws2_32)
endif()

# Allows both `./build/hl_vtd` and `cd build && ./hl_vtd` to use the default path.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
configure_file(config/network.yaml config/network.yaml COPYONLY)
