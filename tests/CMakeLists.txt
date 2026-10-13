add_executable(VehicleSimCoreTests
    SanityTest.cpp
    MathTest.cpp
    ControlTest.cpp
    Spline2DTest.cpp
    PhysicsTest.cpp
    JointTest.cpp
    FluidFieldTest.cpp
    DynamicsTest.cpp
    TerrainTest.cpp
    DeformableTerrainTest.cpp
    NarrowphaseTest.cpp
    TireModelTest.cpp
    StanleyControllerTest.cpp
    SpeedProfilerTest.cpp
    TrajectoryTrackerTest.cpp
    TelemetryTest.cpp
    GroundEffectAeroTest.cpp
    RaceCarDynamicsTest.cpp
    ForkliftDynamicsTest.cpp
    QuadcopterDroneDynamicsTest.cpp
)

target_link_libraries(VehicleSimCoreTests PRIVATE
    VehicleSimCore
)

vehiclesim_register_gtest(VehicleSimCoreTests)
