cmake_minimum_required(VERSION 3.16)

project(gps-parser-test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(gps-parser-test
    gps-parser-test.cpp
    src/unicore.cpp
    src/crc.cpp
)

target_compile_options(gps-parser-test
    PRIVATE
    -Wall
    -Wextra
    -Wconversion
    -Wpedantic
)

target_include_directories(gps-parser-test
    PRIVATE
    src/
)

include(CTest)
enable_testing()

add_test(NAME GPSParserTest COMMAND gps-parser-test)

add_executable(gps-time-test gps-time-test.cpp)
target_include_directories(gps-time-test PRIVATE src/)

foreach(timezone IN ITEMS UTC0 EST5EDT JST-9)
    add_test(NAME GPSUtcTime-${timezone} COMMAND gps-time-test)
    set_tests_properties(GPSUtcTime-${timezone} PROPERTIES ENVIRONMENT "TZ=${timezone}")
endforeach()
