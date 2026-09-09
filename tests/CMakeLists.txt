set(gps_driver_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/../src")

add_executable(gps-parser-test
    gps-parser-test.cpp
    ${gps_driver_source_dir}/unicore.cpp
    ${gps_driver_source_dir}/crc.cpp
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
    ${gps_driver_source_dir}/
)

add_test(NAME GPSParserTest COMMAND gps-parser-test)

add_executable(gps-time-test gps-time-test.cpp)
target_include_directories(gps-time-test PRIVATE ${gps_driver_source_dir}/)

add_executable(gps-time-fallback-test gps-time-test.cpp)
target_include_directories(gps-time-fallback-test PRIVATE ${gps_driver_source_dir}/)
target_compile_definitions(gps-time-fallback-test PRIVATE GPS_NO_TIMEGM)

foreach(timezone IN ITEMS UTC0 EST5EDT JST-9)
    add_test(NAME GPSUtcTime-${timezone} COMMAND gps-time-test)
    set_tests_properties(GPSUtcTime-${timezone} PROPERTIES ENVIRONMENT "TZ=${timezone}")
    add_test(NAME GPSUtcFallback-${timezone} COMMAND gps-time-fallback-test)
    set_tests_properties(GPSUtcFallback-${timezone} PROPERTIES ENVIRONMENT "TZ=${timezone}")
endforeach()

add_executable(gps-ubx-test
    gps-ubx-test.cpp
    ${gps_driver_source_dir}/ubx.cpp
    ${gps_driver_source_dir}/gps_helper.cpp
    ${gps_driver_source_dir}/rtcm.cpp
)
target_include_directories(gps-ubx-test PRIVATE ${gps_driver_source_dir}/)
target_compile_definitions(gps-ubx-test PRIVATE
    GPS_DEFINITIONS_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/definitions.h"
)
target_compile_options(gps-ubx-test PRIVATE -Wall -Wextra)
add_test(NAME GPSUbxSurveyIn COMMAND gps-ubx-test)
set_tests_properties(GPSUbxSurveyIn PROPERTIES TIMEOUT 10)

add_executable(gps-receiver-mode-test
    gps-receiver-mode-test.cpp
    ${gps_driver_source_dir}/sbf.cpp
    ${gps_driver_source_dir}/femtomes.cpp
    ${gps_driver_source_dir}/gps_helper.cpp
    ${gps_driver_source_dir}/rtcm.cpp
    ${gps_driver_source_dir}/crc.cpp
)
target_include_directories(gps-receiver-mode-test PRIVATE ${gps_driver_source_dir}/)
target_compile_definitions(gps-receiver-mode-test PRIVATE
    GPS_DEFINITIONS_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/definitions.h"
)
target_compile_options(gps-receiver-mode-test PRIVATE -Wall -Wextra)
add_test(NAME GPSReceiverModes COMMAND gps-receiver-mode-test)
set_tests_properties(GPSReceiverModes PROPERTIES TIMEOUT 10)
