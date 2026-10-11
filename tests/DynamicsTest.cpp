#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::ArcadeCarConfig;
using vehicle_sim::dynamics::ArcadeCarDynamics;
using vehicle_sim::dynamics::ArcadeCarInput;
using vehicle_sim::dynamics::DriveType;
using vehicle_sim::dynamics::HandbrakeState;
using vehicle_sim::dynamics::IVehicleDynamics;
using vehicle_sim::dynamics::NitroState;
using vehicle_sim::math::Vec2;
using vehicle_sim::physics::Integrator2D;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kTolerance { 1e-4 };

// -----------------------------------------------------------------------------
// ArcadeCarDynamics Configuration & Layout Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, DefaultConstructionAndWheelLayout)
{
    const ArcadeCarDynamics car { };
    const ArcadeCarConfig& config { car.getConfig() };

    EXPECT_DOUBLE_EQ(config.wheelbase, 2.5);
    EXPECT_DOUBLE_EQ(config.trackWidth, 1.5);
    EXPECT_DOUBLE_EQ(car.getSteerAngle(), 0.0);
    EXPECT_DOUBLE_EQ(car.getSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(car.getFlightHeight(), 0.0);
    EXPECT_DOUBLE_EQ(car.getNitroFuel(), config.maxNitroDuration);
    EXPECT_FALSE(car.isAirborne());
    EXPECT_FALSE(car.isDrifting());
    EXPECT_FALSE(car.isNitroActive());

    const auto& fl { car.getWheelTelemetry(ArcadeCarDynamics::kFrontLeft) };
    const auto& fr { car.getWheelTelemetry(ArcadeCarDynamics::kFrontRight) };
    const auto& rl { car.getWheelTelemetry(ArcadeCarDynamics::kRearLeft) };
    const auto& rr { car.getWheelTelemetry(ArcadeCarDynamics::kRearRight) };

    EXPECT_DOUBLE_EQ(fl.localOffset.x, 1.25);
    EXPECT_DOUBLE_EQ(fl.localOffset.y, 0.75);
    EXPECT_DOUBLE_EQ(fr.localOffset.x, 1.25);
    EXPECT_DOUBLE_EQ(fr.localOffset.y, -0.75);
    EXPECT_DOUBLE_EQ(rl.localOffset.x, -1.25);
    EXPECT_DOUBLE_EQ(rl.localOffset.y, 0.75);
    EXPECT_DOUBLE_EQ(rr.localOffset.x, -1.25);
    EXPECT_DOUBLE_EQ(rr.localOffset.y, -0.75);

    EXPECT_TRUE(fl.isSteering);
    EXPECT_TRUE(fr.isSteering);
    EXPECT_FALSE(rl.isSteering);
    EXPECT_FALSE(rr.isSteering);

    EXPECT_TRUE(fl.isDriven);
    EXPECT_TRUE(fr.isDriven);
    EXPECT_TRUE(rl.isDriven);
    EXPECT_TRUE(rr.isDriven);
}

TEST(DynamicsTest, DrivetrainDistributions)
{
    ArcadeCarConfig config { };
    config.driveType = DriveType::FrontWheelDrive;
    ArcadeCarDynamics fwdCar { config };

    EXPECT_TRUE(fwdCar.getWheelTelemetry(ArcadeCarDynamics::kFrontLeft).isDriven);
    EXPECT_TRUE(fwdCar.getWheelTelemetry(ArcadeCarDynamics::kFrontRight).isDriven);
    EXPECT_FALSE(fwdCar.getWheelTelemetry(ArcadeCarDynamics::kRearLeft).isDriven);
    EXPECT_FALSE(fwdCar.getWheelTelemetry(ArcadeCarDynamics::kRearRight).isDriven);

    config.driveType = DriveType::RearWheelDrive;
    ArcadeCarDynamics rwdCar { config };

    EXPECT_FALSE(rwdCar.getWheelTelemetry(ArcadeCarDynamics::kFrontLeft).isDriven);
    EXPECT_FALSE(rwdCar.getWheelTelemetry(ArcadeCarDynamics::kFrontRight).isDriven);
    EXPECT_TRUE(rwdCar.getWheelTelemetry(ArcadeCarDynamics::kRearLeft).isDriven);
    EXPECT_TRUE(rwdCar.getWheelTelemetry(ArcadeCarDynamics::kRearRight).isDriven);
}

// -----------------------------------------------------------------------------
// Straight-Line Dynamics & Acceleration Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, StraightLineAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };

    ArcadeCarDynamics car { };
    ArcadeCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 5.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, kTolerance);
    EXPECT_NEAR(body.getAngularVelocity(), 0.0, kTolerance);
    EXPECT_GT(body.getPosition().x, 5.0);
    EXPECT_GT(car.getSpeed(), 5.0);
}

TEST(DynamicsTest, BrakingDeceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 25.0, 0.0 });

    ArcadeCarDynamics car { };
    ArcadeCarInput input { };
    input.brake = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, 25.0);
    EXPECT_GT(body.getLinearVelocity().x, 0.0);
}

TEST(DynamicsTest, ReverseDriveKinematics)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };

    ArcadeCarDynamics car { };
    ArcadeCarInput input { };
    input.throttle = -1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, -2.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, kTolerance);
    EXPECT_LT(car.getSpeed(), -2.0);
}

// -----------------------------------------------------------------------------
// Steering & Turning Dynamics Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, SteeringYawRateGeneration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 15.0, 0.0 });

    ArcadeCarDynamics car { };
    ArcadeCarInput input { };
    input.throttle = 0.5;
    input.steer = 0.8;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 40; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getAngularVelocity(), 0.1);
    EXPECT_GT(body.getOrientation(), 0.05);
    EXPECT_GT(car.getSteerAngle(), 0.1);
}

TEST(DynamicsTest, HandbrakeDriftSlipScaling)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 20.0, 0.0 });

    ArcadeCarDynamics car { };
    ArcadeCarInput input { };
    input.throttle = 0.2;
    input.steer = 0.7;
    input.handbrake = HandbrakeState::Engaged;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const auto& rl { car.getWheelTelemetry(ArcadeCarDynamics::kRearLeft) };
    const auto& rr { car.getWheelTelemetry(ArcadeCarDynamics::kRearRight) };

    const double avgRearSlip { 0.5 * (std::abs(rl.lateralVelocity) + std::abs(rr.lateralVelocity)) };
    EXPECT_GT(avgRearSlip, 0.5);
    EXPECT_TRUE(car.isDrifting());
}

// -----------------------------------------------------------------------------
// Off-Road Surface Modifier Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, SurfaceFrictionScaling)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;

    RigidBody2D tarmacBody { bodyConfig };
    tarmacBody.setLinearVelocity(Vec2 { 15.0, 5.0 });

    RigidBody2D iceBody { bodyConfig };
    iceBody.setLinearVelocity(Vec2 { 15.0, 5.0 });

    ArcadeCarDynamics tarmacCar { };
    tarmacCar.setSurfaceModifier(1.0);

    ArcadeCarDynamics iceCar { };
    iceCar.setSurfaceModifier(0.2);

    constexpr double dt { 1.0 / 60.0 };
    tarmacCar.step(tarmacBody, dt);
    iceCar.step(iceBody, dt);

    const double tarmacLateralForce { std::abs(tarmacCar.getWheelTelemetry(ArcadeCarDynamics::kFrontLeft).force.y) };
    const double iceLateralForce { std::abs(iceCar.getWheelTelemetry(ArcadeCarDynamics::kFrontLeft).force.y) };

    EXPECT_GT(tarmacLateralForce, iceLateralForce * 3.0);
}

TEST(DynamicsTest, AsymmetricSurfaceModifier)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 15.0, 0.0 });

    ArcadeCarDynamics car { };
    car.setWheelSurfaceModifier(ArcadeCarDynamics::kFrontRight, 0.1);
    car.setWheelSurfaceModifier(ArcadeCarDynamics::kRearRight, 0.1);

    ArcadeCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_NE(body.getAngularVelocity(), 0.0);
}

// -----------------------------------------------------------------------------
// Airborne Jump Physics Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, JumpParabolicArcAndGroundDetachment)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 10.0, 0.0 });

    ArcadeCarDynamics car { };
    car.triggerJump(5.0);

    EXPECT_TRUE(car.isAirborne());
    EXPECT_GT(car.getFlightHeight(), 0.0);

    constexpr double dt { 1.0 / 60.0 };
    car.step(body, dt);

    for (std::size_t i { 0 }; i < ArcadeCarDynamics::kWheelCount; ++i) {
        const auto& wheel { car.getWheelTelemetry(i) };
        EXPECT_DOUBLE_EQ(wheel.force.x, 0.0);
        EXPECT_DOUBLE_EQ(wheel.force.y, 0.0);
    }

    for (int step { 0 }; step < 80; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_FALSE(car.isAirborne());
    EXPECT_DOUBLE_EQ(car.getFlightHeight(), 0.0);
}

// -----------------------------------------------------------------------------
// Nitro Booster Mechanics Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, NitroBoostAndFuelDepletion)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;

    RigidBody2D normalBody { bodyConfig };
    RigidBody2D nitroBody { bodyConfig };

    ArcadeCarDynamics normalCar { };
    ArcadeCarDynamics nitroCar { };

    ArcadeCarInput normalInput { };
    normalInput.throttle = 1.0;
    normalCar.setInput(normalInput);

    ArcadeCarInput nitroInput { };
    nitroInput.throttle = 1.0;
    nitroInput.nitro = NitroState::Active;
    nitroCar.setInput(nitroInput);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        normalCar.step(normalBody, dt);
        Integrator2D::integrate(normalBody, dt);
        normalBody.clearForces();

        nitroCar.step(nitroBody, dt);
        Integrator2D::integrate(nitroBody, dt);
        nitroBody.clearForces();
    }

    EXPECT_TRUE(nitroCar.isNitroActive());
    EXPECT_LT(nitroCar.getNitroFuel(), 3.0);
    EXPECT_GT(nitroBody.getLinearVelocity().x, normalBody.getLinearVelocity().x + 2.0);
}

// -----------------------------------------------------------------------------
// Swappable Coordinator & Determinism Tests
// -----------------------------------------------------------------------------

TEST(DynamicsTest, SwappableCoordinatorMomentumPreservation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    RigidBody2D body { bodyConfig };

    std::unique_ptr<IVehicleDynamics> coordinator { std::make_unique<ArcadeCarDynamics>() };
    auto* car { static_cast<ArcadeCarDynamics*>(coordinator.get()) };

    ArcadeCarInput input { };
    input.throttle = 1.0;
    input.steer = 0.5;
    car->setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        coordinator->step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const Vec2 preservedVel { body.getLinearVelocity() };
    const Vec2 preservedPos { body.getPosition() };
    const double preservedOmega { body.getAngularVelocity() };
    const double preservedAngle { body.getOrientation() };

    coordinator = std::make_unique<ArcadeCarDynamics>();

    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, preservedVel.x);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, preservedVel.y);
    EXPECT_DOUBLE_EQ(body.getPosition().x, preservedPos.x);
    EXPECT_DOUBLE_EQ(body.getPosition().y, preservedPos.y);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), preservedOmega);
    EXPECT_DOUBLE_EQ(body.getOrientation(), preservedAngle);
}

TEST(DynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1100.0;
    bodyConfig.inertia = 1800.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    ArcadeCarDynamics carA { };
    ArcadeCarDynamics carB { };

    ArcadeCarInput input { };
    input.throttle = 0.8;
    input.steer = -0.4;
    carA.setInput(input);
    carB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 100; ++step) {
        carA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        carB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
}

} // namespace vehicle_sim::testing
