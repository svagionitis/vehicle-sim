#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/dynamics/MonsterCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/RocketDynamics.h"
#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::ArcadeCarConfig;
using vehicle_sim::dynamics::ArcadeCarDynamics;
using vehicle_sim::dynamics::ArcadeCarInput;
using vehicle_sim::dynamics::DriveType;
using vehicle_sim::dynamics::EngineConfig;
using vehicle_sim::dynamics::FacingDirection;
using vehicle_sim::dynamics::HandbrakeState;
using vehicle_sim::dynamics::IVehicleDynamics;
using vehicle_sim::dynamics::MonsterCarConfig;
using vehicle_sim::dynamics::MonsterCarDynamics;
using vehicle_sim::dynamics::MonsterCarInput;
using vehicle_sim::dynamics::MonsterDriveType;
using vehicle_sim::dynamics::NitroState;
using vehicle_sim::dynamics::OilSlickState;
using vehicle_sim::dynamics::RocketConfig;
using vehicle_sim::dynamics::RocketDynamics;
using vehicle_sim::dynamics::RocketInput;
using vehicle_sim::dynamics::RocketTelemetry;
using vehicle_sim::dynamics::SideViewCarConfig;
using vehicle_sim::dynamics::SideViewCarDynamics;
using vehicle_sim::dynamics::SideViewCarInput;
using vehicle_sim::dynamics::SideViewPlatform;
using vehicle_sim::dynamics::TransmissionConfig;
using vehicle_sim::dynamics::TurnaroundState;
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

// -----------------------------------------------------------------------------
// MonsterCarDynamics Configuration & Layout Tests
// -----------------------------------------------------------------------------

TEST(MonsterCarDynamicsTest, DefaultConstructionAndWheelLayout)
{
    const MonsterCarDynamics car { };
    const MonsterCarConfig& config { car.getConfig() };

    EXPECT_DOUBLE_EQ(config.distCgToFront, 1.25);
    EXPECT_DOUBLE_EQ(config.distCgToRear, 1.25);
    EXPECT_DOUBLE_EQ(config.trackWidth, 1.5);
    EXPECT_DOUBLE_EQ(config.mass, 1500.0);
    EXPECT_DOUBLE_EQ(config.gravity, 9.81);
    EXPECT_DOUBLE_EQ(car.getSteerAngle(), 0.0);
    EXPECT_DOUBLE_EQ(car.getSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(car.getEngineRPM(), config.engine.idleRPM);
    EXPECT_EQ(car.getGear(), 1);
    EXPECT_TRUE(car.isAutomatic());
    EXPECT_FALSE(car.isAirborne());
    EXPECT_FALSE(car.isDrifting());

    const auto& fl { car.getWheelTelemetry(MonsterCarDynamics::kFrontLeft) };
    const auto& fr { car.getWheelTelemetry(MonsterCarDynamics::kFrontRight) };
    const auto& rl { car.getWheelTelemetry(MonsterCarDynamics::kRearLeft) };
    const auto& rr { car.getWheelTelemetry(MonsterCarDynamics::kRearRight) };

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

    const double expectedTotalWeight { 1500.0 * 9.81 };
    EXPECT_NEAR(car.getFrontAxleWeight(), 0.5 * expectedTotalWeight, kTolerance);
    EXPECT_NEAR(car.getRearAxleWeight(), 0.5 * expectedTotalWeight, kTolerance);
    EXPECT_NEAR(fl.normalLoad, 0.25 * expectedTotalWeight, kTolerance);
    EXPECT_NEAR(rl.normalLoad, 0.25 * expectedTotalWeight, kTolerance);
}

TEST(MonsterCarDynamicsTest, DrivetrainDistributions)
{
    MonsterCarConfig config { };

    config.driveType = MonsterDriveType::RearWheelDrive;
    MonsterCarDynamics rwdCar { config };
    EXPECT_FALSE(rwdCar.getWheelTelemetry(MonsterCarDynamics::kFrontLeft).isDriven);
    EXPECT_FALSE(rwdCar.getWheelTelemetry(MonsterCarDynamics::kFrontRight).isDriven);
    EXPECT_TRUE(rwdCar.getWheelTelemetry(MonsterCarDynamics::kRearLeft).isDriven);
    EXPECT_TRUE(rwdCar.getWheelTelemetry(MonsterCarDynamics::kRearRight).isDriven);

    config.driveType = MonsterDriveType::FrontWheelDrive;
    MonsterCarDynamics fwdCar { config };
    EXPECT_TRUE(fwdCar.getWheelTelemetry(MonsterCarDynamics::kFrontLeft).isDriven);
    EXPECT_TRUE(fwdCar.getWheelTelemetry(MonsterCarDynamics::kFrontRight).isDriven);
    EXPECT_FALSE(fwdCar.getWheelTelemetry(MonsterCarDynamics::kRearLeft).isDriven);
    EXPECT_FALSE(fwdCar.getWheelTelemetry(MonsterCarDynamics::kRearRight).isDriven);

    config.driveType = MonsterDriveType::AllWheelDrive;
    MonsterCarDynamics awdCar { config };
    EXPECT_TRUE(awdCar.getWheelTelemetry(MonsterCarDynamics::kFrontLeft).isDriven);
    EXPECT_TRUE(awdCar.getWheelTelemetry(MonsterCarDynamics::kFrontRight).isDriven);
    EXPECT_TRUE(awdCar.getWheelTelemetry(MonsterCarDynamics::kRearLeft).isDriven);
    EXPECT_TRUE(awdCar.getWheelTelemetry(MonsterCarDynamics::kRearRight).isDriven);
}

TEST(MonsterCarDynamicsTest, DynamicWeightTransferAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    MonsterCarDynamics car { };
    MonsterCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 0.5);
    EXPECT_GT(car.getAcceleration(), 0.0);
    EXPECT_GT(car.getRearAxleWeight(), car.getFrontAxleWeight());

    const double expectedTotalWeight { 1500.0 * 9.81 };
    EXPECT_NEAR(car.getFrontAxleWeight() + car.getRearAxleWeight(), expectedTotalWeight, 1.0);
}

TEST(MonsterCarDynamicsTest, DynamicWeightTransferBraking)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 25.0, 0.0 });

    MonsterCarDynamics car { };
    MonsterCarInput input { };
    input.throttle = 0.0;
    input.brake = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 15; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(car.getAcceleration(), 0.0);
    EXPECT_GT(car.getFrontAxleWeight(), car.getRearAxleWeight());

    const double expectedTotalWeight { 1500.0 * 9.81 };
    EXPECT_NEAR(car.getFrontAxleWeight() + car.getRearAxleWeight(), expectedTotalWeight, 1.0);
}

TEST(MonsterCarDynamicsTest, PowertrainTorqueCurveAndGears)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    MonsterCarDynamics car { };
    car.setAutomatic(false);

    MonsterCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    car.setGear(1);
    constexpr double dt { 1.0 / 60.0 };
    car.step(body, dt);
    const double driveForceGear1 { car.getDriveForce() };

    car.setGear(4);
    car.step(body, dt);
    const double driveForceGear4 { car.getDriveForce() };

    EXPECT_GT(driveForceGear1, driveForceGear4);

    car.setGear(-1);
    car.step(body, dt);
    EXPECT_LT(car.getDriveForce(), 0.0);

    car.setGear(0);
    car.step(body, dt);
    EXPECT_DOUBLE_EQ(car.getDriveForce(), 0.0);
}

TEST(MonsterCarDynamicsTest, AerodynamicDragAndRollingResistance)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    MonsterCarConfig config { };
    config.airDragCoefficient = 4.0;
    config.rollingResistanceCoeff = 20.0;

    MonsterCarDynamics car { config };
    car.setAutomatic(false);
    car.setGear(2);

    MonsterCarInput input { };
    input.throttle = 0.5;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 2400; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const double speedA { body.getLinearVelocity().x };

    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const double speedB { body.getLinearVelocity().x };
    EXPECT_GT(speedA, 10.0);
    EXPECT_NEAR(speedA, speedB, 0.05);
}

TEST(MonsterCarDynamicsTest, SteeringYawRateAndCornering)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 15.0, 0.0 });

    MonsterCarDynamics car { };
    MonsterCarInput input { };
    input.throttle = 0.3;
    input.steer = 0.5;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getAngularVelocity(), 0.1);
    EXPECT_GT(body.getOrientation(), 0.05);

    const auto& fl { car.getWheelTelemetry(MonsterCarDynamics::kFrontLeft) };
    EXPECT_NE(fl.slipAngle, 0.0);
    EXPECT_NE(fl.lateralForce, 0.0);
}

TEST(MonsterCarDynamicsTest, FrictionCircleSaturation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 10.0, 10.0 });

    MonsterCarDynamics car { };
    car.setSurfaceModifier(0.15, 1.0);

    MonsterCarInput input { };
    input.throttle = 1.0;
    input.steer = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    car.step(body, dt);

    for (std::size_t i { 0 }; i < MonsterCarDynamics::kWheelCount; ++i) {
        const auto& wheel { car.getWheelTelemetry(i) };
        const double fMag { std::hypot(wheel.longitudinalForce, wheel.lateralForce) };
        const double fMax { car.getConfig().tireFrictionCoefficient * wheel.surfaceGrip * wheel.normalLoad };
        EXPECT_LE(fMag, fMax + 1e-2);
        EXPECT_TRUE(wheel.isSaturated);
    }
}

TEST(MonsterCarDynamicsTest, AutomaticTransmissionShifting)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    MonsterCarDynamics car { };
    EXPECT_TRUE(car.isAutomatic());
    EXPECT_EQ(car.getGear(), 1);

    MonsterCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 300; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(car.getGear(), 1);
    EXPECT_GT(car.getSpeed(), 10.0);
}

TEST(MonsterCarDynamicsTest, SwappableCoordinatorWithArcadeCar)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    std::unique_ptr<IVehicleDynamics> coordinator { std::make_unique<ArcadeCarDynamics>() };
    auto* arcade { static_cast<ArcadeCarDynamics*>(coordinator.get()) };

    ArcadeCarInput arcadeInput { };
    arcadeInput.throttle = 0.8;
    arcadeInput.steer = 0.3;
    arcade->setInput(arcadeInput);

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

    coordinator = std::make_unique<MonsterCarDynamics>();

    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, preservedVel.x);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, preservedVel.y);
    EXPECT_DOUBLE_EQ(body.getPosition().x, preservedPos.x);
    EXPECT_DOUBLE_EQ(body.getPosition().y, preservedPos.y);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), preservedOmega);
    EXPECT_DOUBLE_EQ(body.getOrientation(), preservedAngle);

    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_GT(body.getLinearVelocity().length(), 0.0);
}

TEST(MonsterCarDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 2500.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    MonsterCarDynamics carA { };
    MonsterCarDynamics carB { };

    MonsterCarInput input { };
    input.throttle = 0.75;
    input.steer = -0.35;
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

// -----------------------------------------------------------------------------
// SideViewCarDynamics Configuration & Platform Tests
// -----------------------------------------------------------------------------

TEST(SideViewCarDynamicsTest, DefaultConstructionAndWheelLayout)
{
    const SideViewCarDynamics car { };
    const SideViewCarConfig& config { car.getConfig() };

    EXPECT_DOUBLE_EQ(config.wheelbase, 2.2);
    EXPECT_DOUBLE_EQ(config.wheelRadius, 0.35);
    EXPECT_DOUBLE_EQ(config.mass, 1000.0);
    EXPECT_DOUBLE_EQ(config.gravity, 9.81);
    EXPECT_DOUBLE_EQ(car.getSpeed(), 0.0);
    EXPECT_EQ(car.getFacingDirection(), FacingDirection::Right);
    EXPECT_EQ(car.getTurnaroundState(), TurnaroundState::Idle);
    EXPECT_EQ(car.getOilSlickState(), OilSlickState::Normal);
    EXPECT_FALSE(car.isSpinningOut());
    EXPECT_FALSE(car.isTurningAround());
    EXPECT_EQ(car.getPlatformCount(), 1U);

    const auto& rw { car.getWheelTelemetry(SideViewCarDynamics::kRearWheel) };
    const auto& fw { car.getWheelTelemetry(SideViewCarDynamics::kFrontWheel) };

    EXPECT_DOUBLE_EQ(rw.localOffset.x, -1.1);
    EXPECT_DOUBLE_EQ(rw.localOffset.y, -0.35);
    EXPECT_DOUBLE_EQ(fw.localOffset.x, 1.1);
    EXPECT_DOUBLE_EQ(fw.localOffset.y, -0.35);
}

TEST(SideViewCarDynamicsTest, StraightLinePlatformAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    SideViewCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getPosition().x, 0.5);
    EXPECT_GT(body.getLinearVelocity().x, 1.0);
    EXPECT_NEAR(body.getPosition().y, 0.35, 0.05);
    EXPECT_TRUE(car.isGrounded());
    EXPECT_FALSE(car.isAirborne());
    EXPECT_GT(car.getSpeed(), 1.0);
}

TEST(SideViewCarDynamicsTest, BrakingAndReverseKinematics)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };
    bodyConfig.linearDamping = 0.0;

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    SideViewCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const double coastSpeed { body.getLinearVelocity().x };
    EXPECT_GT(coastSpeed, 2.0);

    input.throttle = 0.0;
    input.brake = 1.0;
    car.setInput(input);

    for (int step { 0 }; step < 30; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const double brakedSpeed { body.getLinearVelocity().x };
    EXPECT_LT(brakedSpeed, coastSpeed);

    input.brake = 0.0;
    input.throttle = -1.0;
    car.setInput(input);

    for (int step { 0 }; step < 120; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, 0.0);
}

TEST(SideViewCarDynamicsTest, JumpParabolicArcAndGroundDetachment)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    constexpr double dt { 1.0 / 60.0 };
    car.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_TRUE(car.isGrounded());
    EXPECT_FALSE(car.isAirborne());

    car.triggerJump(8.0);
    car.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_TRUE(car.isAirborne());
    EXPECT_FALSE(car.isGrounded());
    EXPECT_GT(body.getLinearVelocity().y, 5.0);

    double peakHeight { 0.35 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
        if (body.getPosition().y > peakHeight) {
            peakHeight = body.getPosition().y;
        }
    }

    EXPECT_GT(peakHeight, 2.0);

    for (int step { 0 }; step < 80; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(car.isGrounded());
    EXPECT_FALSE(car.isAirborne());
    EXPECT_NEAR(body.getPosition().y, 0.35, 0.05);
}

TEST(SideViewCarDynamicsTest, Instant180TurnaroundManeuver)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    EXPECT_EQ(car.getFacingDirection(), FacingDirection::Right);
    EXPECT_DOUBLE_EQ(body.getOrientation(), 0.0);

    car.triggerTurnaround(true);
    constexpr double dt { 1.0 / 60.0 };
    car.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_EQ(car.getFacingDirection(), FacingDirection::Left);
    EXPECT_NEAR(body.getOrientation(), kPi, 0.01);
    EXPECT_EQ(car.getTurnaroundState(), TurnaroundState::Idle);

    SideViewCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, -0.5);
    EXPECT_LT(body.getPosition().x, -0.1);
}

TEST(SideViewCarDynamicsTest, Timed180TurnaroundSpin)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D body { bodyConfig };
    SideViewCarConfig config { };
    config.turnaroundDuration = 0.15;
    SideViewCarDynamics car { config };

    car.triggerTurnaround(false);
    constexpr double dt { 1.0 / 60.0 };

    car.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_TRUE(car.isTurningAround());
    EXPECT_EQ(car.getTurnaroundState(), TurnaroundState::Active);
    EXPECT_GT(car.getTurnaroundTimer(), 0.0);

    for (int step { 0 }; step < 15; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_FALSE(car.isTurningAround());
    EXPECT_EQ(car.getTurnaroundState(), TurnaroundState::Idle);
    EXPECT_NEAR(body.getOrientation(), kPi, 0.01);
}

TEST(SideViewCarDynamicsTest, OilSlickSpinoutAndRecovery)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    car.triggerOilSlick(0.5);
    EXPECT_TRUE(car.isSpinningOut());
    EXPECT_EQ(car.getOilSlickState(), OilSlickState::Spinning);
    EXPECT_DOUBLE_EQ(car.getOilSlickTimer(), 0.5);

    SideViewCarInput input { };
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 15; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(car.isSpinningOut());
    EXPECT_NE(body.getAngularVelocity(), 0.0);

    for (int step { 0 }; step < 25; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_FALSE(car.isSpinningOut());
    EXPECT_EQ(car.getOilSlickState(), OilSlickState::Normal);
    EXPECT_DOUBLE_EQ(car.getOilSlickTimer(), 0.0);
    EXPECT_NEAR(body.getOrientation(), 0.0, 0.05);
}

TEST(SideViewCarDynamicsTest, MultiTierPlatformLandingAndPassThrough)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 4.0 };

    RigidBody2D body { bodyConfig };
    SideViewCarDynamics car { };

    car.addPlatform(2.0, -10.0, 10.0, 1.0, true);
    EXPECT_EQ(car.getPlatformCount(), 2U);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(car.isGrounded());
    EXPECT_NEAR(body.getPosition().y, 2.35, 0.05);
}

TEST(SideViewCarDynamicsTest, SurfaceFrictionScaling)
{
    RigidBodyConfig configA { };
    configA.mass = 1000.0;
    configA.inertia = 1500.0;
    configA.position = Vec2 { 0.0, 0.35 };

    RigidBodyConfig configB { configA };

    RigidBody2D bodyA { configA };
    RigidBody2D bodyB { configB };

    SideViewCarDynamics carA { };
    SideViewCarDynamics carB { };
    carB.setSurfaceModifier(0.25);

    SideViewCarInput input { };
    input.throttle = 1.0;
    carA.setInput(input);
    carB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        carA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        carB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_GT(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
}

TEST(SideViewCarDynamicsTest, SwappableCoordinatorMomentumPreservation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 5.0, 0.35 };

    RigidBody2D body { bodyConfig };

    std::unique_ptr<IVehicleDynamics> coordinator { std::make_unique<SideViewCarDynamics>() };
    auto* sideViewCar { static_cast<SideViewCarDynamics*>(coordinator.get()) };

    SideViewCarInput input { };
    input.throttle = 1.0;
    sideViewCar->setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        coordinator->step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const Vec2 preservedVel { body.getLinearVelocity() };
    const Vec2 preservedPos { body.getPosition() };
    const double preservedOmega { body.getAngularVelocity() };
    const double preservedAngle { body.getOrientation() };

    coordinator = std::make_unique<MonsterCarDynamics>();

    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, preservedVel.x);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, preservedVel.y);
    EXPECT_DOUBLE_EQ(body.getPosition().x, preservedPos.x);
    EXPECT_DOUBLE_EQ(body.getPosition().y, preservedPos.y);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), preservedOmega);
    EXPECT_DOUBLE_EQ(body.getOrientation(), preservedAngle);

    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    coordinator = std::make_unique<ArcadeCarDynamics>();

    EXPECT_GT(body.getLinearVelocity().length(), 0.0);
    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    coordinator = std::make_unique<SideViewCarDynamics>();
    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();
}

TEST(SideViewCarDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = Vec2 { 0.0, 0.35 };

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    SideViewCarDynamics carA { };
    SideViewCarDynamics carB { };

    SideViewCarInput input { };
    input.throttle = 0.8;
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

// -----------------------------------------------------------------------------
// RocketDynamics Configuration & Space Propulsion Tests
// -----------------------------------------------------------------------------

TEST(RocketDynamicsTest, DefaultConstructionAndInertialGeometry)
{
    const RocketDynamics rocket { };
    const RocketConfig& config { rocket.getConfig() };

    EXPECT_DOUBLE_EQ(config.dryMass, 1500.0);
    EXPECT_DOUBLE_EQ(config.initialFuelMass, 3500.0);
    EXPECT_DOUBLE_EQ(config.length, 6.0);
    EXPECT_DOUBLE_EQ(config.radius, 0.8);
    EXPECT_DOUBLE_EQ(config.maxThrust, 65000.0);
    EXPECT_DOUBLE_EQ(config.specificImpulse, 310.0);

    EXPECT_DOUBLE_EQ(rocket.getDryMass(), 1500.0);
    EXPECT_DOUBLE_EQ(rocket.getFuelMass(), 3500.0);
    EXPECT_DOUBLE_EQ(rocket.getTotalMass(), 5000.0);

    // I = 5000 * (6^2 / 12 + 0.8^2 / 4) = 5000 * (3.0 + 0.16) = 15800.0 kg*m^2
    EXPECT_DOUBLE_EQ(rocket.getInertia(), 15800.0);
    EXPECT_DOUBLE_EQ(rocket.getFuelRatio(), 1.0);
    EXPECT_FALSE(rocket.isFuelEmpty());
    EXPECT_TRUE(rocket.isAirborne());
    EXPECT_DOUBLE_EQ(rocket.getGimbalAngle(), 0.0);
    EXPECT_TRUE(rocket.isSasEnabled());
}

TEST(RocketDynamicsTest, MainEngineForwardThrustAndAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D body { bodyConfig };
    RocketDynamics rocket { };

    RocketInput input { };
    input.throttle = 1.0;
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_DOUBLE_EQ(rocket.getThrustForce(), 65000.0);
    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-6);

    // Initial burn rate dm/dt = 65000 / (310 * 9.80665) ~= 21.3809 kg/s
    const double expectedFuelBurned { (65000.0 / (310.0 * 9.80665)) * dt };
    EXPECT_NEAR(rocket.getFuelMass(), 3500.0 - expectedFuelBurned, 1e-4);
    EXPECT_LT(rocket.getTotalMass(), 5000.0);
    EXPECT_LT(rocket.getInertia(), 15800.0);
    EXPECT_DOUBLE_EQ(body.getMass(), rocket.getTotalMass());
    EXPECT_DOUBLE_EQ(body.getInertia(), rocket.getInertia());
}

TEST(RocketDynamicsTest, GimbalVectoringInducedTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D body { bodyConfig };
    RocketConfig config { };
    config.gimbalSpeed = 0.0; // instantaneous response for test
    RocketDynamics rocket { config };

    RocketInput input { };
    input.throttle = 1.0;
    input.gimbal = 1.0; // Max positive deflection (+10 deg)
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_NEAR(rocket.getGimbalAngle(), config.maxGimbalAngle, 1e-6);
    // r = (-3, 0), F_local = (T cos delta, T sin delta).
    // r x F = -3 * T * sin(delta) < 0 (clockwise torque)
    EXPECT_LT(body.getAngularVelocity(), 0.0);
    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_GT(body.getLinearVelocity().y, 0.0);
}

TEST(RocketDynamicsTest, LateralOffsetThrusterTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D body { bodyConfig };
    RocketConfig config { };
    config.thrusterOffset = Vec2 { 0.0, 0.5 }; // Offset laterally +0.5m above COM
    RocketDynamics rocket { config };

    RocketInput input { };
    input.throttle = 1.0;
    input.gimbal = 0.0; // No gimbal
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    // r = (0, 0.5), F = (T, 0) -> r x F = 0*0 - 0.5*T = -0.5 * 65000 = -32500 N*m
    EXPECT_LT(body.getAngularVelocity(), 0.0);
    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-6);
}

TEST(RocketDynamicsTest, RCSAttitudeTorqueAndRotation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D body { bodyConfig };
    RocketDynamics rocket { };

    RocketInput input { };
    input.throttle = 0.0;
    input.rcs = 1.0; // Counter-clockwise command
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_GT(body.getAngularVelocity(), 0.0);
    EXPECT_NEAR(body.getLinearVelocity().length(), 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(rocket.getTelemetry().rcsTorque, 1500.0);
    EXPECT_LT(rocket.getFuelMass(), 3500.0);
}

TEST(RocketDynamicsTest, SASActiveRateDamping)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D bodySAS { bodyConfig };
    bodySAS.setAngularVelocity(0.5); // Initial spin 0.5 rad/s

    RigidBody2D bodyFree { bodyConfig };
    bodyFree.setAngularVelocity(0.5);

    RocketDynamics rocketSAS { };
    RocketInput inputSAS { };
    inputSAS.sasEnabled = true;
    rocketSAS.setInput(inputSAS);

    RocketDynamics rocketFree { };
    RocketInput inputFree { };
    inputFree.sasEnabled = false;
    rocketFree.setInput(inputFree);

    constexpr double dt { 0.05 };
    for (int step { 0 }; step < 50; ++step) {
        rocketSAS.step(bodySAS, dt);
        Integrator2D::integrate(bodySAS, dt);
        bodySAS.clearForces();

        rocketFree.step(bodyFree, dt);
        Integrator2D::integrate(bodyFree, dt);
        bodyFree.clearForces();
    }

    EXPECT_LT(std::abs(bodySAS.getAngularVelocity()), std::abs(bodyFree.getAngularVelocity()));
    EXPECT_NEAR(bodyFree.getAngularVelocity(), 0.5, 1e-6);
}

TEST(RocketDynamicsTest, TranslationalRCSThrusters)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D body { bodyConfig };
    RocketDynamics rocket { };

    RocketInput input { };
    input.throttle = 0.0;
    input.rcsTranslate = Vec2 { 0.0, 1.0 }; // Local +Y translation
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_GT(body.getLinearVelocity().y, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().x, 0.0, 1e-6);
    EXPECT_NEAR(body.getAngularVelocity(), 0.0, 1e-6);
}

TEST(RocketDynamicsTest, PropellantDepletionAndEngineFlameout)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1510.0;
    bodyConfig.inertia = 5000.0;

    RigidBody2D body { bodyConfig };
    RocketConfig config { };
    config.initialFuelMass = 5.0; // Only 5 kg fuel
    RocketDynamics rocket { config };

    RocketInput input { };
    input.throttle = 1.0;
    rocket.setInput(input);

    constexpr double dt { 0.1 };
    for (int step { 0 }; step < 10; ++step) {
        rocket.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(rocket.isFuelEmpty());
    EXPECT_DOUBLE_EQ(rocket.getFuelMass(), 0.0);
    EXPECT_DOUBLE_EQ(rocket.getTotalMass(), rocket.getDryMass());
    EXPECT_DOUBLE_EQ(rocket.getThrustForce(), 0.0);
    EXPECT_TRUE(rocket.getTelemetry().isFlameout);

    // Record velocity after burnout
    const double burnoutSpeed { body.getLinearVelocity().length() };

    // Step again with throttle 1.0; velocity should not increase
    rocket.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().length(), burnoutSpeed);

    // Refuel
    rocket.refuel();
    EXPECT_FALSE(rocket.isFuelEmpty());
    EXPECT_DOUBLE_EQ(rocket.getFuelMass(), config.initialFuelMass);
}

TEST(RocketDynamicsTest, AtmosphericDragAndTerminalDescent)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 5000.0;

    RigidBody2D body { bodyConfig };
    RocketConfig config { };
    config.atmosphericDensity = 1.225; // Sea level air density
    config.dragCoefficient = 0.5;
    config.crossSectionArea = 2.0;
    config.gravity = Vec2 { 0.0, -9.81 }; // Downward gravity
    config.initialFuelMass = 0.0; // Unpowered drop
    RocketDynamics rocket { config };

    // Equilibrium: m * g = 0.5 * rho * Cd * A * v^2
    // 1500 * 9.81 = 0.5 * 1.225 * 0.5 * 2.0 * v^2 = 0.6125 * v^2
    // v = sqrt((1500 * 9.81) / 0.6125) = sqrt(14715 / 0.6125) ~= 154.998 m/s
    const double expectedTerminalSpeed { std::sqrt((1500.0 * 9.81) / (0.5 * 1.225 * 0.5 * 2.0)) };
    body.setLinearVelocity(Vec2 { 0.0, -expectedTerminalSpeed });

    constexpr double dt { 0.05 };
    for (int step { 0 }; step < 200; ++step) {
        rocket.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().y, 0.0);
    EXPECT_NEAR(std::abs(body.getLinearVelocity().y), expectedTerminalSpeed, 0.1);
}

TEST(RocketDynamicsTest, SwappableCoordinatorFourWayPreservation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2000.0;
    bodyConfig.position = Vec2 { 10.0, 25.0 };
    bodyConfig.orientation = 0.45;

    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 15.0, -8.0 });
    body.setAngularVelocity(0.25);

    std::unique_ptr<IVehicleDynamics> coordinator { std::make_unique<ArcadeCarDynamics>() };
    constexpr double dt { 1.0 / 60.0 };

    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    const Vec2 preservedPos { body.getPosition() };
    const Vec2 preservedVel { body.getLinearVelocity() };
    const double preservedAngle { body.getOrientation() };
    const double preservedOmega { body.getAngularVelocity() };

    // Swap to MonsterCarDynamics
    coordinator = std::make_unique<MonsterCarDynamics>();
    EXPECT_DOUBLE_EQ(body.getPosition().x, preservedPos.x);
    EXPECT_DOUBLE_EQ(body.getPosition().y, preservedPos.y);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, preservedVel.x);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, preservedVel.y);
    EXPECT_DOUBLE_EQ(body.getOrientation(), preservedAngle);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), preservedOmega);

    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    // Swap to SideViewCarDynamics
    coordinator = std::make_unique<SideViewCarDynamics>();
    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    // Swap to RocketDynamics
    coordinator = std::make_unique<RocketDynamics>();
    EXPECT_GT(body.getLinearVelocity().length(), 0.0);

    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    // Swap back to ArcadeCarDynamics
    coordinator = std::make_unique<ArcadeCarDynamics>();
    coordinator->step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_GT(body.getLinearVelocity().length(), 0.0);
}

TEST(RocketDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 5000.0;
    bodyConfig.inertia = 15800.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    RocketDynamics rocketA { };
    RocketDynamics rocketB { };

    RocketInput input { };
    input.throttle = 0.85;
    input.gimbal = 0.3;
    input.rcs = 0.2;
    input.sasEnabled = true;

    rocketA.setInput(input);
    rocketB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        rocketA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        rocketB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
    EXPECT_DOUBLE_EQ(rocketA.getFuelMass(), rocketB.getFuelMass());
}

} // namespace vehicle_sim::testing
