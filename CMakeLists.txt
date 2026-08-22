cmake_minimum_required(VERSION 3.16)

project(HL_VTD VERSION 0.1.0 LANGUAGES CXX)

add_executable(hl_vtd
    src/main.cpp
    src/network/network_config.cpp
    src/network/vtd_client.cpp
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
