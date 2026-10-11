add_library(VehicleSimCore STATIC
    Version.h
    Version.cpp
    math/Vec2.h
    math/Vec2.cpp
    math/Mat2.h
    math/Mat2.cpp
    math/Transform2D.h
    math/Transform2D.cpp
    physics/RigidBody2D.h
    physics/RigidBody2D.cpp
    physics/Integrator2D.h
    physics/Integrator2D.cpp
    physics/ContactSolver2D.h
    physics/ContactSolver2D.cpp
    physics/PhysicsWorld2D.h
    physics/PhysicsWorld2D.cpp
    dynamics/IVehicleDynamics.h
    dynamics/ArcadeCarDynamics.h
    dynamics/ArcadeCarDynamics.cpp
    dynamics/MonsterCarDynamics.h
    dynamics/MonsterCarDynamics.cpp
)

target_include_directories(VehicleSimCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(VehicleSimCore PUBLIC cxx_std_17)

apply_compiler_flags(VehicleSimCore)
