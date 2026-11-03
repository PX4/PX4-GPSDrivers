cmake_minimum_required(VERSION 3.16)

project(gps-parser-test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(gps-parser-test
    gps-parser-test.cpp
    src/gps_helper.cpp
    src/rtcm.cpp
    src/ubx.cpp
    src/crc.cpp
    src/unicore.cpp
)

target_compile_options(gps-parser-test
    PRIVATE
    -Wall
    -Wextra
    -Wconversion
    -Wpedantic
)

target_compile_definitions(gps-parser-test
    PRIVATE
    GPS_DEFINITIONS_HEADER="ubx-test-definitions.h"
)

target_include_directories(gps-parser-test
    PRIVATE
    .
    src/
)

include(CTest)
enable_testing()

add_test(NAME GPSParserTest COMMAND gps-parser-test)
