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
    dynamics/SideViewCarDynamics.h
    dynamics/SideViewCarDynamics.cpp
    dynamics/RocketDynamics.h
    dynamics/RocketDynamics.cpp
    dynamics/TrackedTankDynamics.h
    dynamics/TrackedTankDynamics.cpp
    dynamics/AerodynamicPlaneDynamics.h
    dynamics/AerodynamicPlaneDynamics.cpp
    dynamics/HovercraftDynamics.h
    dynamics/HovercraftDynamics.cpp
    dynamics/WatercraftDynamics.h
    dynamics/WatercraftDynamics.cpp
    dynamics/TrialsBikeDynamics.h
    dynamics/TrialsBikeDynamics.cpp
    dynamics/HelicopterDynamics.h
    dynamics/HelicopterDynamics.cpp
    dynamics/SubmarineDynamics.h
    dynamics/SubmarineDynamics.cpp
    dynamics/AntiGravityRacerDynamics.h
    dynamics/AntiGravityRacerDynamics.cpp
    dynamics/ArticulatedTruckDynamics.h
    dynamics/ArticulatedTruckDynamics.cpp
    terrain/SurfaceMaterial.h
    terrain/SurfaceMaterial.cpp
    terrain/GradientNoise.h
    terrain/GradientNoise.cpp
    terrain/FractalNoise.h
    terrain/FractalNoise.cpp
    terrain/Heightfield1D.h
    terrain/Heightfield1D.cpp
    terrain/SurfaceMap2D.h
    terrain/SurfaceMap2D.cpp
    terrain/ElevationMap2D.h
)

target_include_directories(VehicleSimCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(VehicleSimCore PUBLIC cxx_std_17)

apply_compiler_flags(VehicleSimCore)
