cmake_minimum_required(VERSION 3.20)

project(VehicleSim
    VERSION 0.1.0
    DESCRIPTION "2D Vehicle Simulation C++17 library for games"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for language servers and tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Library options
option(BUILD_TESTING "Build the automated test suite" ON)
option(BUILD_EARTH_PATROL "Build Earth Patrol SDL2 game application" ON)
option(BUILD_MOTION_LAB "Build Motion Lab 2D vehicle simulator application" ON)
option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(ENABLE_HARDENING "Enable security hardening flags" ON)
option(ENABLE_ASAN "Enable AddressSanitizer (ASan)" OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer (UBSan)" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer (TSan)" OFF)

# Include custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerFlags)
include(GitHooks)

# Core library
add_subdirectory(libs/VehicleSimCore)

# Test suite
if(BUILD_TESTING)
    include(Testing)
    add_subdirectory(tests)
endif()

# Applications
if(BUILD_EARTH_PATROL)
    add_subdirectory(apps/EarthPatrol)
endif()

if(BUILD_MOTION_LAB)
    add_subdirectory(apps/MotionLab)
endif()
