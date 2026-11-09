# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
# Native bench tools. A project of its own: it needs no Qt and builds on QuickEIP's C sources.
#   cmake -S tools/ecam_bench -B tools/ecam_bench/build
#   cmake --build tools/ecam_bench/build --config Release
cmake_minimum_required(VERSION 3.21)
project(ecam_bench LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(QUICKEIP ${CMAKE_CURRENT_SOURCE_DIR}/../../QuickEIP)

file(GLOB QUICKEIP_SOURCES ${QUICKEIP}/src/*.c)
add_executable(ecam_bench main.cpp soak.cpp nettest.cpp bench_link.cpp suites.cpp suites_board.cpp suites_motion.cpp suites_live.cpp ${QUICKEIP}/ports/windows/eip_hal_windows.c ${QUICKEIP_SOURCES})
target_include_directories(ecam_bench PRIVATE ${QUICKEIP}/include ${QUICKEIP}/src)
target_compile_definitions(ecam_bench PRIVATE EIP_WIN_POLL_MODE _WINSOCK_DEPRECATED_NO_WARNINGS _CRT_SECURE_NO_WARNINGS NOMINMAX)
target_link_libraries(ecam_bench PRIVATE ws2_32 iphlpapi winmm)
