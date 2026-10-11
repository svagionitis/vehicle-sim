add_library(VehicleSimCore STATIC
    Version.h
    Version.cpp
)

target_include_directories(VehicleSimCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(VehicleSimCore PUBLIC cxx_std_17)

apply_compiler_flags(VehicleSimCore)
