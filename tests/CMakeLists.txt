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
    NarrowphaseTest.cpp
)

target_link_libraries(VehicleSimCoreTests PRIVATE
    VehicleSimCore
)

vehiclesim_register_gtest(VehicleSimCoreTests)
