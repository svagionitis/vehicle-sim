# ==============================================================================
# Testing & Code Coverage Configuration
# ==============================================================================

if(VEHICLESIM_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "VehicleSim: Code coverage enabled (--coverage -O0 -g)")
        add_compile_options(--coverage -O0 -g)
        add_link_options(--coverage)
    else()
        message(STATUS "VehicleSim: VEHICLESIM_ENABLE_COVERAGE active on MSVC. Use OpenCppCoverage for execution analysis.")
    endif()
endif()

enable_testing()

find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
    message(STATUS "VehicleSim: GTest not found in system. Fetching GoogleTest via FetchContent...")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googletest)

    if(TARGET gtest)
        set_target_properties(gtest PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    endif()
    if(TARGET gtest_main)
        set_target_properties(gtest_main PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    endif()
    if(TARGET gmock)
        set_target_properties(gmock PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    endif()
    if(TARGET gmock_main)
        set_target_properties(gmock_main PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
    endif()
endif()
include(GoogleTest)

function(vehiclesim_register_gtest target_name)
    cmake_parse_arguments(ARG "NO_MAIN" "" "" ${ARGN})
    if(ARG_NO_MAIN)
        target_link_libraries(${target_name} PRIVATE GTest::gmock)
    else()
        target_link_libraries(${target_name} PRIVATE GTest::gmock_main)
    endif()
    apply_compiler_flags(${target_name})

    if(WIN32)
        if(TARGET GTest::gmock_main)
            get_target_property(_GMOCK_MAIN_TYPE GTest::gmock_main TYPE)
            if(_GMOCK_MAIN_TYPE STREQUAL "SHARED_LIBRARY")
                add_custom_command(TARGET ${target_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "$<TARGET_FILE:GTest::gmock_main>"
                        "$<TARGET_FILE_DIR:${target_name}>"
                    COMMENT "Copying gmock_main DLL for ${target_name}"
                )
            endif()
        endif()
        if(TARGET GTest::gmock)
            get_target_property(_GMOCK_TYPE GTest::gmock TYPE)
            if(_GMOCK_TYPE STREQUAL "SHARED_LIBRARY")
                add_custom_command(TARGET ${target_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "$<TARGET_FILE:GTest::gmock>"
                        "$<TARGET_FILE_DIR:${target_name}>"
                    COMMENT "Copying gmock DLL for ${target_name}"
                )
            endif()
        endif()
        if(TARGET GTest::gtest)
            get_target_property(_GTEST_TYPE GTest::gtest TYPE)
            if(_GTEST_TYPE STREQUAL "SHARED_LIBRARY")
                add_custom_command(TARGET ${target_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "$<TARGET_FILE:GTest::gtest>"
                        "$<TARGET_FILE_DIR:${target_name}>"
                    COMMENT "Copying gtest DLL for ${target_name}"
                )
            endif()
        endif()
    endif()

    set(_RUNNER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/${target_name}_runner_$<CONFIG>.cmake")
    file(GENERATE OUTPUT "${_RUNNER_SCRIPT}" CONTENT
"# Auto-generated test runner for ${target_name}
set(cmd \"\")
set(found_delim FALSE)
math(EXPR last_idx \"\${CMAKE_ARGC} - 1\")
foreach(i RANGE 0 \${last_idx})
    set(arg \"\${CMAKE_ARGV\${i}}\")
    if(found_delim)
        list(APPEND cmd \"\${arg}\")
    elseif(arg STREQUAL \"--\")
        set(found_delim TRUE)
    endif()
endforeach()

execute_process(
    COMMAND \${cmd}
    RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
    if(res MATCHES \"^[0-9]+$\")
        cmake_language(EXIT \${res})
    else()
        message(FATAL_ERROR \"Execution failed: \${res}\")
    endif()
endif()
")

    set_target_properties(${target_name} PROPERTIES
        TEST_LAUNCHER "${CMAKE_COMMAND};-P;${_RUNNER_SCRIPT};--"
    )

    gtest_discover_tests(${target_name}
        DISCOVERY_MODE PRE_TEST
    )
endfunction()

# Alias for backward compatibility
function(pelcod_register_gtest target_name)
    vehiclesim_register_gtest(${target_name} ${ARGN})
endfunction()

if(WIN32)
    find_program(OPENCPPCOVERAGE_EXE NAMES OpenCppCoverage.exe PATHS "C:/Program Files/OpenCppCoverage")
    if(OPENCPPCOVERAGE_EXE)
        find_program(POWERSHELL_EXE NAMES pwsh powershell)
        if(NOT POWERSHELL_EXE)
            set(POWERSHELL_EXE powershell)
        endif()
        add_custom_target(coverage
            COMMAND "${POWERSHELL_EXE}" -ExecutionPolicy Bypass -File "${CMAKE_SOURCE_DIR}/scripts/RunCoverage.ps1"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            COMMENT "Generating unified code coverage report with OpenCppCoverage..."
        )
    endif()
endif()
