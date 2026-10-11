add_library(VehicleSimCore STATIC
    Version.h
    Version.cpp
    math/Vec2.h
    math/Vec2.cpp
    math/Mat2.h
    math/Mat2.cpp
    math/Transform2D.h
    math/Transform2D.cpp
)

target_include_directories(VehicleSimCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(VehicleSimCore PUBLIC cxx_std_17)

apply_compiler_flags(VehicleSimCore)
