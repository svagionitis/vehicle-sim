if(EXISTS "C:/vcpkg/installed/x64-windows/share/sdl2")
    list(APPEND CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")
endif()

find_package(SDL2 CONFIG REQUIRED)

add_executable(EarthPatrol
    Main.cpp
    GameApp.h
    GameApp.cpp
    Camera2D.h
    Camera2D.cpp
    CourseManager.h
    CourseManager.cpp
    SDLWrappers.h
)

target_include_directories(EarthPatrol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(EarthPatrol PRIVATE
    VehicleSimCore
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)

target_compile_definitions(EarthPatrol PRIVATE SDL_MAIN_HANDLED)

target_compile_features(EarthPatrol PRIVATE cxx_std_17)
apply_compiler_flags(EarthPatrol)

if(WIN32)
    if(TARGET SDL2::SDL2)
        get_target_property(_SDL2_TYPE SDL2::SDL2 TYPE)
        if(_SDL2_TYPE STREQUAL "SHARED_LIBRARY")
            add_custom_command(TARGET EarthPatrol POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:SDL2::SDL2>
                    $<TARGET_FILE_DIR:EarthPatrol>
            )
        endif()
    endif()
endif()
