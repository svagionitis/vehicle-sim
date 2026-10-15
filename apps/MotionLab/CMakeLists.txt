if(EXISTS "C:/vcpkg/installed/x64-windows/share/sdl2")
    list(APPEND CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")
endif()

find_package(SDL2 CONFIG REQUIRED)

add_executable(MotionLab
    Main.cpp
    MotionLabApp.h
    MotionLabApp.cpp
    MotionLabCamera.h
    MotionLabCamera.cpp
    SkidRibbonBuffer.h
    SkidRibbonBuffer.cpp
    TrailParticleBuffer.h
    TrailParticleBuffer.cpp
    WireframeRenderer.h
    WireframeRenderer.cpp
    TelemetryHud.h
    TelemetryHud.cpp
)

target_include_directories(MotionLab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MotionLab PRIVATE
    VehicleSimCore
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)

target_compile_definitions(MotionLab PRIVATE SDL_MAIN_HANDLED)

target_compile_features(MotionLab PRIVATE cxx_std_17)
apply_compiler_flags(MotionLab)

if(WIN32)
    if(TARGET SDL2::SDL2)
        get_target_property(_SDL2_TYPE SDL2::SDL2 TYPE)
        if(_SDL2_TYPE STREQUAL "SHARED_LIBRARY")
            add_custom_command(TARGET MotionLab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:SDL2::SDL2>
                    $<TARGET_FILE_DIR:MotionLab>
            )
        endif()
    endif()
endif()

if(BUILD_TESTING)
    add_executable(MotionLabTests
        MotionLabAppTest.cpp
        MotionLabApp.h
        MotionLabApp.cpp
        MotionLabCamera.h
        MotionLabCamera.cpp
        SkidRibbonBuffer.h
        SkidRibbonBuffer.cpp
        TrailParticleBuffer.h
        TrailParticleBuffer.cpp
        WireframeRenderer.h
        WireframeRenderer.cpp
        TelemetryHud.h
        TelemetryHud.cpp
    )

    target_include_directories(MotionLabTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(MotionLabTests PRIVATE
        VehicleSimCore
        $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    )

    target_compile_definitions(MotionLabTests PRIVATE SDL_MAIN_HANDLED)
    target_compile_features(MotionLabTests PRIVATE cxx_std_17)
    apply_compiler_flags(MotionLabTests)
    vehiclesim_register_gtest(MotionLabTests)

    if(WIN32)
        if(TARGET SDL2::SDL2)
            if(_SDL2_TYPE STREQUAL "SHARED_LIBRARY")
                add_custom_command(TARGET MotionLabTests POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_FILE:SDL2::SDL2>
                        $<TARGET_FILE_DIR:MotionLabTests>
                )
            endif()
        endif()
    endif()
endif()
