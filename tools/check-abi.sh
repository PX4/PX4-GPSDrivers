#!/bin/bash
# shellcheck shell=bash
#
# ABI/API Compatibility Checker for PX4-GPSDrivers
#
# Usage:
#   ./tools/check-abi.sh [base_ref] [head_ref]
#
# Arguments:
#   base_ref  - Git reference for base version (default: origin/main)
#   head_ref  - Git reference for new version (default: HEAD)
#
# Requirements:
#   - libabigail (provides abidw, abidiff)
#   - cmake
#
# Install on Ubuntu/Debian:
#   sudo apt-get install abigail-tools cmake
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BASE_REF="${1:-origin/main}"
HEAD_REF="${2:-HEAD}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_dependencies() {
    local missing=()

    command -v abidw &>/dev/null || missing+=("abigail-tools")
    command -v abidiff &>/dev/null || missing+=("abigail-tools")
    command -v cmake &>/dev/null || missing+=("cmake")

    # Deduplicate
    mapfile -t missing < <(printf "%s\n" "${missing[@]}" | sort -u)

    if [ ${#missing[@]} -ne 0 ]; then
        error "Missing dependencies: ${missing[*]}"
        echo "Install with: sudo apt-get install ${missing[*]}"
        exit 1
    fi
}

create_definitions_h() {
    local dir="$1"
    cat > "$dir/definitions.h" << 'EOF'
#pragma once
#include <cstdint>
#include <ctime>

struct sensor_gps_s {
    uint64_t timestamp;
    uint64_t time_utc_usec;
    double latitude;
    double longitude;
    float altitude_msl;
    float altitude_ellipsoid;
    float eph;
    float epv;
    float s_variance_m_s;
    float c_variance_rad;
    float vel_m_s;
    float vel_n_m_s;
    float vel_e_m_s;
    float vel_d_m_s;
    float cog_rad;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    float heading;
    float heading_offset;
    float heading_accuracy;
    float yaw;
    float yaw_offset;
    uint8_t fix_type;
    uint8_t satellites_used;
    bool vel_ned_valid;
    bool rtcm_injection_rate;
};

struct satellite_info_s {
    uint64_t timestamp;
    uint8_t count;
    uint8_t svid[20];
    uint8_t used[20];
    uint8_t elevation[20];
    uint8_t azimuth[20];
    uint8_t snr[20];
    uint8_t prn[20];
};

struct sensor_gnss_relative_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint16_t time_utc_usec;
    uint16_t reference_station_id;
    float position[3];
    float position_accuracy[3];
    float heading;
    float heading_accuracy;
    float position_length;
    float accuracy_length;
    bool gnss_fix_ok;
    bool differential_solution;
    bool relative_position_valid;
    bool carrier_solution_floating;
    bool carrier_solution_fixed;
    bool moving_base_mode;
    bool reference_position_miss;
    bool reference_observations_miss;
    bool heading_valid;
    bool relative_position_normalized;
};
EOF
}

create_cmake_file() {
    local dir="$1"
    cat > "$dir/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(gps_drivers CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

file(GLOB GPS_SOURCES "src/*.cpp")

add_library(gps_drivers SHARED ${GPS_SOURCES})

target_include_directories(gps_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(gps_drivers PRIVATE
    -g -Og
    -Wall -Wextra
)

target_compile_definitions(gps_drivers PRIVATE
    GPS_DEFINITIONS_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/definitions.h"
)

set_target_properties(gps_drivers PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
)
EOF
}

build_version() {
    local ref="$1"
    local work_dir="$2"
    local version_name="$3"

    info "Checking out $ref to $work_dir..."

    # Create worktree or copy
    if [ "$ref" = "HEAD" ] || [ "$ref" = "." ]; then
        # Use current working tree
        cp -r "$REPO_ROOT/src" "$work_dir/"
        [ -f "$REPO_ROOT/CMakeLists.txt" ] && cp "$REPO_ROOT/CMakeLists.txt" "$work_dir/"
    else
        git -C "$REPO_ROOT" archive "$ref" | tar -x -C "$work_dir"
    fi

    create_definitions_h "$work_dir"
    create_cmake_file "$work_dir"

    info "Building $version_name..."

    local log_file="$TEMP_DIR/${version_name}-build.log"

    (
        cd "$work_dir"

        cmake -B build -S . -G "Unix Makefiles" \
            -DCMAKE_BUILD_TYPE=Debug \
            -DCMAKE_C_FLAGS="-g -Og" \
            -DCMAKE_CXX_FLAGS="-g -Og" \
            >> "$log_file" 2>&1

        cmake --build build >> "$log_file" 2>&1
    )

    if [ ! -f "$work_dir/build/libgps_drivers.so" ]; then
        error "Failed to build $version_name (see $log_file)"
        return 1
    fi

    info "Generating ABI dump for $version_name..."
    abidw --no-corpus-path \
        --headers-dir "$work_dir/src/" \
        "$work_dir/build/libgps_drivers.so" \
        > "$TEMP_DIR/${version_name}.abi" 2>> "$log_file"

    return 0
}

main() {
    check_dependencies

    # Validate git references
    for ref in "$BASE_REF" "$HEAD_REF"; do
        if [ "$ref" != "HEAD" ] && [ "$ref" != "." ]; then
            if ! git -C "$REPO_ROOT" rev-parse --verify "$ref" &>/dev/null; then
                error "Invalid git reference: $ref"
                exit 1
            fi
        fi
    done

    info "Comparing ABI: $BASE_REF -> $HEAD_REF"

    TEMP_DIR=$(mktemp -d)
    cleanup() {
        local rc=$?
        rm -rf "$TEMP_DIR"
        exit $rc
    }
    trap cleanup EXIT

    mkdir -p "$TEMP_DIR/base" "$TEMP_DIR/head"

    # Build both versions
    if ! build_version "$BASE_REF" "$TEMP_DIR/base" "base"; then
        error "Failed to build base version ($BASE_REF)"
        exit 1
    fi

    if ! build_version "$HEAD_REF" "$TEMP_DIR/head" "head"; then
        error "Failed to build head version ($HEAD_REF)"
        exit 1
    fi

    # Compare
    info "Comparing ABI..."
    mkdir -p "$TEMP_DIR/report"

    local diff_output
    local diff_exit=0

    diff_output=$(abidiff \
        --headers-dir1 "$TEMP_DIR/base/src" \
        --headers-dir2 "$TEMP_DIR/head/src" \
        "$TEMP_DIR/base.abi" \
        "$TEMP_DIR/head.abi" 2>&1) || diff_exit=$?

    echo "$diff_output" > "$TEMP_DIR/report/abidiff.txt"

    echo ""
    echo "============================================"
    echo "ABI/API Compatibility Report"
    echo "============================================"
    echo "Base:   $BASE_REF"
    echo "Head:   $HEAD_REF"
    echo ""

    # abidiff exit codes: 0=compatible, 4=ABI change, 8=error
    case $diff_exit in
        0)
            info "No ABI changes detected"
            echo "Status: COMPATIBLE"
            echo "============================================"
            exit 0
            ;;
        4)
            warn "ABI changes detected"
            echo "Status: BREAKING CHANGES"
            echo "============================================"
            echo ""
            echo "$diff_output"
            echo ""
            echo "$diff_output" > "$REPO_ROOT/abi-report.txt"
            info "Report saved to: $REPO_ROOT/abi-report.txt"
            exit 1
            ;;
        *)
            error "abidiff error (exit code: $diff_exit)"
            echo "$diff_output"
            exit 1
            ;;
    esac
}

main "$@"
