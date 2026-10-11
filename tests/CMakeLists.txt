add_executable(VehicleSimCoreTests
    SanityTest.cpp
    MathTest.cpp
    PhysicsTest.cpp
    DynamicsTest.cpp
)

target_link_libraries(VehicleSimCoreTests PRIVATE
    VehicleSimCore
)

vehiclesim_register_gtest(VehicleSimCoreTests)
