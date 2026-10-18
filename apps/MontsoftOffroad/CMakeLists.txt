if(EXISTS "C:/vcpkg/installed/x64-windows/share/sdl2")
    list(APPEND CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")
endif()

find_package(SDL2 CONFIG REQUIRED)

add_executable(MontsoftOffroad
    Main.cpp
    GameApp.h
    GameApp.cpp
    CgaPaletteRenderer.h
    CgaPaletteRenderer.cpp
    Camera2D.h
    Camera2D.cpp
    CourseManager.h
    CourseManager.cpp
    InputController.h
    InputController.cpp
    ParticleSystem.h
    ParticleSystem.cpp
    TruckRenderer.h
    TruckRenderer.cpp
    ParallaxRenderer.h
    ParallaxRenderer.cpp
    CombatManager.h
    CombatManager.cpp
    EnemyManager.h
    EnemyManager.cpp
    BossController.h
    BossController.cpp
    SDLWrappers.h
)

target_include_directories(MontsoftOffroad PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MontsoftOffroad PRIVATE
    VehicleSimCore
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)

target_compile_definitions(MontsoftOffroad PRIVATE SDL_MAIN_HANDLED)
target_compile_features(MontsoftOffroad PRIVATE cxx_std_17)
apply_compiler_flags(MontsoftOffroad)

if(WIN32)
    if(TARGET SDL2::SDL2)
        get_target_property(_SDL2_TYPE SDL2::SDL2 TYPE)
        if(_SDL2_TYPE STREQUAL "SHARED_LIBRARY")
            add_custom_command(TARGET MontsoftOffroad POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:SDL2::SDL2>
                    $<TARGET_FILE_DIR:MontsoftOffroad>
            )
        endif()
    endif()
endif()

if(BUILD_TESTING)
    add_executable(MontsoftOffroadTests
        GameAppTest.cpp
        GameApp.h
        GameApp.cpp
        CgaPaletteRenderer.h
        CgaPaletteRenderer.cpp
        Camera2D.h
        Camera2D.cpp
        CourseManager.h
        CourseManager.cpp
        InputController.h
        InputController.cpp
        ParticleSystem.h
        ParticleSystem.cpp
        TruckRenderer.h
        TruckRenderer.cpp
        ParallaxRenderer.h
        ParallaxRenderer.cpp
        CombatManager.h
        CombatManager.cpp
        EnemyManager.h
        EnemyManager.cpp
        BossController.h
        BossController.cpp
        SDLWrappers.h
    )

    target_include_directories(MontsoftOffroadTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(MontsoftOffroadTests PRIVATE
        VehicleSimCore
        $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    )

    target_compile_definitions(MontsoftOffroadTests PRIVATE SDL_MAIN_HANDLED)
    target_compile_features(MontsoftOffroadTests PRIVATE cxx_std_17)
    apply_compiler_flags(MontsoftOffroadTests)
    vehiclesim_register_gtest(MontsoftOffroadTests)

    if(WIN32)
        if(TARGET SDL2::SDL2)
            if(_SDL2_TYPE STREQUAL "SHARED_LIBRARY")
                add_custom_command(TARGET MontsoftOffroadTests POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_FILE:SDL2::SDL2>
                        $<TARGET_FILE_DIR:MontsoftOffroadTests>
                )
            endif()
        endif()
    endif()
endif()
