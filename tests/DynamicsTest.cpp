#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "libs/VehicleSimCore/dynamics/AerodynamicPlaneDynamics.h"
#include "libs/VehicleSimCore/dynamics/AntiGravityRacerDynamics.h"
#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/ArticulatedTruckDynamics.h"
#include "libs/VehicleSimCore/dynamics/HelicopterDynamics.h"
#include "libs/VehicleSimCore/dynamics/HovercraftDynamics.h"
#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/dynamics/MonsterCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/RocketDynamics.h"
#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/SubmarineDynamics.h"
#include "libs/VehicleSimCore/dynamics/TrackedTankDynamics.h"
#include "libs/VehicleSimCore/dynamics/TrialsBikeDynamics.h"
#include "libs/VehicleSimCore/dynamics/WatercraftDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::AeroPlaneConfig;
using vehicle_sim::dynamics::AeroPlaneInput;
using vehicle_sim::dynamics::AeroPlaneTelemetry;
using vehicle_sim::dynamics::AerodynamicPlaneDynamics;
using vehicle_sim::dynamics::AntiGravityConfig;
using vehicle_sim::dynamics::AntiGravityInput;
using vehicle_sim::dynamics::AntiGravityRacerDynamics;
using vehicle_sim::dynamics::AntiGravityTelemetry;
using vehicle_sim::dynamics::ArcadeCarConfig;
using vehicle_sim::dynamics::ArcadeCarDynamics;
using vehicle_sim::dynamics::ArcadeCarInput;
using vehicle_sim::dynamics::ArticulatedTruckConfig;
using vehicle_sim::dynamics::ArticulatedTruckDynamics;
using vehicle_sim::dynamics::ArticulatedTruckInput;
using vehicle_sim::dynamics::ArticulatedTruckTelemetry;
using vehicle_sim::dynamics::DriveType;
using vehicle_sim::dynamics::EngineConfig;
using vehicle_sim::dynamics::FacingDirection;
using vehicle_sim::dynamics::HandbrakeState;
using vehicle_sim::dynamics::HelicopterConfig;
using vehicle_sim::dynamics::HelicopterDynamics;
using vehicle_sim::dynamics::HelicopterInput;
using vehicle_sim::dynamics::HelicopterTelemetry;
using vehicle_sim::dynamics::HovercraftConfig;
using vehicle_sim::dynamics::HovercraftDynamics;
using vehicle_sim::dynamics::HovercraftInput;
using vehicle_sim::dynamics::HovercraftTelemetry;
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
using vehicle_sim::dynamics::SubmarineConfig;
using vehicle_sim::dynamics::SubmarineDynamics;
using vehicle_sim::dynamics::SubmarineInput;
using vehicle_sim::dynamics::SubmarineTelemetry;
using vehicle_sim::dynamics::TrackedTankConfig;
using vehicle_sim::dynamics::TrackedTankDynamics;
using vehicle_sim::dynamics::TrackedTankInput;
using vehicle_sim::dynamics::TrackedTankTelemetry;
using vehicle_sim::dynamics::TransmissionConfig;
using vehicle_sim::dynamics::TrialsBikeConfig;
using vehicle_sim::dynamics::TrialsBikeDynamics;
using vehicle_sim::dynamics::TrialsBikeInput;
using vehicle_sim::dynamics::TrialsBikeTelemetry;
using vehicle_sim::dynamics::TurnaroundState;
using vehicle_sim::dynamics::WatercraftConfig;
using vehicle_sim::dynamics::WatercraftDynamics;
using vehicle_sim::dynamics::WatercraftInput;
using vehicle_sim::dynamics::WatercraftTelemetry;
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

// -----------------------------------------------------------------------------
// TrackedTankDynamics Tests
// -----------------------------------------------------------------------------

TEST(TrackedTankDynamicsTest, DefaultConstructionAndLayout)
{
    const TrackedTankDynamics tank { };
    const TrackedTankConfig& config { tank.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 12000.0);
    EXPECT_DOUBLE_EQ(config.trackWidth, 2.4);
    EXPECT_DOUBLE_EQ(config.maxTrackForce, 40000.0);
    EXPECT_DOUBLE_EQ(config.maxTrackSpeed, 14.0);
    EXPECT_DOUBLE_EQ(config.surfaceFriction, 1.0);
    EXPECT_FALSE(tank.isAirborne());
    EXPECT_DOUBLE_EQ(tank.getSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(tank.getYawRate(), 0.0);

    const TrackedTankTelemetry& telem { tank.getTelemetry() };
    EXPECT_FALSE(telem.isRecoiling);
    EXPECT_DOUBLE_EQ(telem.recoilTimer, 0.0);
}

TEST(TrackedTankDynamicsTest, DualTrackStraightAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };

    TrackedTankDynamics tank { };
    TrackedTankInput input { };
    input.leftTrack = 1.0;
    input.rightTrack = 1.0;
    tank.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        tank.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-3);
    EXPECT_NEAR(body.getAngularVelocity(), 0.0, 1e-3);
    EXPECT_GT(tank.getSpeed(), 0.0);
}

TEST(TrackedTankDynamicsTest, NeutralSkidSteerRotationInPlace)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };

    TrackedTankDynamics tank { };
    TrackedTankInput input { };
    input.leftTrack = -1.0;
    input.rightTrack = 1.0; // Counter-rotating tracks for neutral turn
    tank.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        tank.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Chassis should rotate with minimal net linear displacement
    EXPECT_GT(body.getAngularVelocity(), 0.1);
    EXPECT_NEAR(body.getLinearVelocity().length(), 0.0, 0.5);
    EXPECT_GT(tank.getYawRate(), 0.1);
}

TEST(TrackedTankDynamicsTest, SkidScrubbingResistanceTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };
    body.setAngularVelocity(2.0); // Initial spin

    TrackedTankDynamics tank { };
    // Zero throttle commands - tracks should resist yaw rotation
    tank.setInput(TrackedTankInput { });

    constexpr double dt { 1.0 / 60.0 };
    tank.step(body, dt);

    const TrackedTankTelemetry& telem { tank.getTelemetry() };
    // Scrubbing torque should oppose the positive yaw rotation
    EXPECT_LT(telem.skidTorque, 0.0);
    EXPECT_LT(body.getAccumulatedTorque(), 0.0);
}

TEST(TrackedTankDynamicsTest, CannonRecoilImpulseAndOffsetTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };

    TrackedTankConfig config { };
    config.turretOffset = Vec2 { 0.5, 0.5 }; // Offset from CG
    TrackedTankDynamics tank { config };

    TrackedTankInput input { };
    input.fireRecoil = true;
    input.turretAngle = 0.0; // Firing straight forward
    tank.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    tank.step(body, dt);

    const TrackedTankTelemetry& telem { tank.getTelemetry() };
    EXPECT_TRUE(telem.isRecoiling);
    EXPECT_LT(telem.recoilForce.x, 0.0); // Backward push
    // Off-center turret offset produces rotational torque
    EXPECT_NE(body.getAccumulatedTorque(), 0.0);
}

TEST(TrackedTankDynamicsTest, SurfaceFrictionScaling)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;

    RigidBody2D normalBody { bodyConfig };
    RigidBody2D icyBody { bodyConfig };

    TrackedTankDynamics normalTank { };
    TrackedTankDynamics icyTank { };
    icyTank.setSurfaceFriction(0.2); // Low friction ice

    TrackedTankInput input { };
    input.leftTrack = 1.0;
    input.rightTrack = 1.0;
    normalTank.setInput(input);
    icyTank.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    normalTank.step(normalBody, dt);
    icyTank.step(icyBody, dt);

    EXPECT_GT(normalTank.getTelemetry().leftTrackForce, icyTank.getTelemetry().leftTrackForce);
}

TEST(TrackedTankDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 12000.0;
    bodyConfig.inertia = 25000.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    TrackedTankDynamics tankA { };
    TrackedTankDynamics tankB { };

    TrackedTankInput input { };
    input.leftTrack = 0.8;
    input.rightTrack = -0.4;
    tankA.setInput(input);
    tankB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        tankA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        tankB.step(bodyB, dt);
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
// AerodynamicPlaneDynamics Tests
// -----------------------------------------------------------------------------

TEST(AerodynamicPlaneDynamicsTest, DefaultConstructionAndFlightProfile)
{
    const AerodynamicPlaneDynamics plane { };
    const AeroPlaneConfig& config { plane.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 1800.0);
    EXPECT_DOUBLE_EQ(config.wingArea, 16.0);
    EXPECT_DOUBLE_EQ(config.maxThrust, 14000.0);
    EXPECT_DOUBLE_EQ(config.stallAngle, 0.28);
    EXPECT_TRUE(plane.isAirborne());
    EXPECT_DOUBLE_EQ(plane.getAirspeed(), 0.0);
    EXPECT_DOUBLE_EQ(plane.getAngleOfAttack(), 0.0);
    EXPECT_FALSE(plane.isStalled());
}

TEST(AerodynamicPlaneDynamicsTest, ForwardThrustAndAirspeedGain)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;
    RigidBody2D body { bodyConfig };

    AeroPlaneConfig config { };
    config.gravity = Vec2::zero(); // Isolate forward thrust
    AerodynamicPlaneDynamics plane { config };

    AeroPlaneInput input { };
    input.throttle = 1.0;
    plane.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        plane.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_GT(plane.getAirspeed(), 0.0);
    EXPECT_GT(plane.getTelemetry().dynamicPressure, 0.0);
    EXPECT_GT(plane.getTelemetry().thrustForce, 0.0);
}

TEST(AerodynamicPlaneDynamicsTest, AngleOfAttackAndLiftGeneration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 50.0, 0.0 }); // Flying horizontally at 50 m/s (~180 km/h)
    body.setOrientation(0.1); // Pitched nose up by 0.1 rad (~5.7 deg)

    AeroPlaneConfig config { };
    config.gravity = Vec2::zero();
    AerodynamicPlaneDynamics plane { config };

    constexpr double dt { 1.0 / 60.0 };
    plane.step(body, dt);

    const AeroPlaneTelemetry& telem { plane.getTelemetry() };
    EXPECT_GT(telem.angleOfAttack, 0.0);
    EXPECT_GT(telem.liftCoeff, 0.0);
    EXPECT_GT(telem.liftForce, 0.0);
    EXPECT_FALSE(plane.isStalled());
}

TEST(AerodynamicPlaneDynamicsTest, StallSeparationAndLiftDropoff)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;

    RigidBody2D attachedBody { bodyConfig };
    attachedBody.setLinearVelocity(Vec2 { 50.0, 0.0 });
    attachedBody.setOrientation(0.20); // 0.20 rad < 0.28 rad stall boundary

    RigidBody2D stalledBody { bodyConfig };
    stalledBody.setLinearVelocity(Vec2 { 50.0, 0.0 });
    stalledBody.setOrientation(0.45); // 0.45 rad > 0.28 rad critical stall angle

    AeroPlaneConfig config { };
    config.gravity = Vec2::zero();
    AerodynamicPlaneDynamics planeAttached { config };
    AerodynamicPlaneDynamics planeStalled { config };

    constexpr double dt { 1.0 / 60.0 };
    planeAttached.step(attachedBody, dt);
    planeStalled.step(stalledBody, dt);

    EXPECT_FALSE(planeAttached.isStalled());
    EXPECT_TRUE(planeStalled.isStalled());
    // Post-stall drag increases dramatically due to flow separation
    EXPECT_GT(planeStalled.getTelemetry().dragCoeff, planeAttached.getTelemetry().dragCoeff);
}

TEST(AerodynamicPlaneDynamicsTest, ElevatorPitchAuthorityAndDamping)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 40.0, 0.0 });

    AeroPlaneConfig config { };
    config.gravity = Vec2::zero();
    AerodynamicPlaneDynamics plane { config };

    AeroPlaneInput input { };
    input.elevator = 1.0; // Full pitch up command
    plane.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    plane.step(body, dt);

    // Dynamic pressure enables elevator pitch torque
    EXPECT_GT(body.getAccumulatedTorque(), 0.0);
}

TEST(AerodynamicPlaneDynamicsTest, AirbrakeAndFlapDeploymentDrag)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;

    RigidBody2D cleanBody { bodyConfig };
    cleanBody.setLinearVelocity(Vec2 { 50.0, 0.0 });

    RigidBody2D brakedBody { bodyConfig };
    brakedBody.setLinearVelocity(Vec2 { 50.0, 0.0 });

    AeroPlaneConfig config { };
    config.gravity = Vec2::zero();
    AerodynamicPlaneDynamics cleanPlane { config };
    AerodynamicPlaneDynamics brakedPlane { config };

    AeroPlaneInput brakeInput { };
    brakeInput.flaps = 1.0;
    brakeInput.airbrake = 1.0;
    brakedPlane.setInput(brakeInput);

    constexpr double dt { 1.0 / 60.0 };
    cleanPlane.step(cleanBody, dt);
    brakedPlane.step(brakedBody, dt);

    EXPECT_GT(brakedPlane.getTelemetry().dragCoeff, cleanPlane.getTelemetry().dragCoeff);
    EXPECT_GT(brakedPlane.getTelemetry().liftCoeff, cleanPlane.getTelemetry().liftCoeff);
}

TEST(AerodynamicPlaneDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1800.0;
    bodyConfig.inertia = 4000.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };
    bodyA.setLinearVelocity(Vec2 { 30.0, 5.0 });
    bodyB.setLinearVelocity(Vec2 { 30.0, 5.0 });

    AerodynamicPlaneDynamics planeA { };
    AerodynamicPlaneDynamics planeB { };

    AeroPlaneInput input { };
    input.throttle = 0.8;
    input.elevator = 0.25;
    input.flaps = 0.5;
    planeA.setInput(input);
    planeB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        planeA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        planeB.step(bodyB, dt);
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
// HovercraftDynamics Tests
// -----------------------------------------------------------------------------

TEST(HovercraftDynamicsTest, DefaultConstructionAndHoverProfile)
{
    const HovercraftDynamics hovercraft { };
    const HovercraftConfig& config { hovercraft.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 1200.0);
    EXPECT_DOUBLE_EQ(config.hoverHeight, 0.35);
    EXPECT_DOUBLE_EQ(config.maxThrust, 6000.0);
    EXPECT_DOUBLE_EQ(hovercraft.getSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(hovercraft.getDriftSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(hovercraft.getRudderAngle(), 0.0);
    EXPECT_TRUE(hovercraft.isHovering());
}

TEST(HovercraftDynamicsTest, AirCushionSuspensionEquilibrium)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;
    RigidBody2D body { bodyConfig };

    HovercraftDynamics hovercraft { };
    HovercraftInput input { };
    input.liftFanActive = true;
    input.groundDistance = 0.20; // Cushion compressed below 0.35 m
    hovercraft.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    hovercraft.step(body, dt);

    const HovercraftTelemetry& telem { hovercraft.getTelemetry() };
    EXPECT_GT(telem.cushionForce, 0.0);
    EXPECT_TRUE(telem.isHovering);
    EXPECT_FALSE(telem.isSkirtScraping);
}

TEST(HovercraftDynamicsTest, LiftFanPowerOffSkirtDrag)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 10.0, 0.0 });

    HovercraftDynamics hovercraft { };
    HovercraftInput input { };
    input.liftFanActive = false; // Cushion deflated
    hovercraft.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    hovercraft.step(body, dt);

    const HovercraftTelemetry& telem { hovercraft.getTelemetry() };
    EXPECT_DOUBLE_EQ(telem.cushionForce, 0.0);
    EXPECT_FALSE(telem.isHovering);
    EXPECT_TRUE(telem.isSkirtScraping);
    // Skirt dragging friction opposes forward motion
    EXPECT_LT(body.getAccumulatedForce().x, 0.0);
}

TEST(HovercraftDynamicsTest, VectoredThrustAndRudderYawTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;
    RigidBody2D body { bodyConfig };

    HovercraftDynamics hovercraft { };
    HovercraftInput input { };
    input.throttle = 1.0;
    input.steering = 1.0; // Turn rudder
    hovercraft.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    hovercraft.step(body, dt);

    const HovercraftTelemetry& telem { hovercraft.getTelemetry() };
    EXPECT_GT(telem.thrustForce, 0.0);
    EXPECT_GT(hovercraft.getRudderAngle(), 0.0);
    // Rear fan offset + deflected thrust produces turning torque
    EXPECT_NE(body.getAccumulatedTorque(), 0.0);
}

TEST(HovercraftDynamicsTest, LowFrictionHighMomentumDrifting)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 0.0, 15.0 }); // Pure sideways drift

    HovercraftDynamics hovercraft { };
    HovercraftInput input { };
    input.liftFanActive = true;
    input.groundDistance = 0.35; // Perfect hover cushion
    hovercraft.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    hovercraft.step(body, dt);
    Integrator2D::integrate(body, dt);
    body.clearForces();

    // Hovercraft retains high lateral speed due to low drift friction
    EXPECT_GT(body.getLinearVelocity().y, 14.0);
    EXPECT_GT(hovercraft.getDriftSpeed(), 14.0);
}

TEST(HovercraftDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    HovercraftDynamics hoverA { };
    HovercraftDynamics hoverB { };

    HovercraftInput input { };
    input.throttle = 0.7;
    input.steering = 0.3;
    input.groundDistance = 0.30;
    hoverA.setInput(input);
    hoverB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        hoverA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        hoverB.step(bodyB, dt);
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
// WatercraftDynamics Tests
// -----------------------------------------------------------------------------

TEST(WatercraftDynamicsTest, DefaultConstructionAndWaterline)
{
    const WatercraftDynamics boat { };
    const WatercraftConfig& config { boat.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 1500.0);
    EXPECT_DOUBLE_EQ(config.draft, 0.6);
    EXPECT_DOUBLE_EQ(config.waterlineY, 0.0);
    EXPECT_DOUBLE_EQ(config.maxPropellerThrust, 8500.0);
    EXPECT_DOUBLE_EQ(boat.getSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(boat.getRudderAngle(), 0.0);
    EXPECT_FALSE(boat.isPlaning());
    EXPECT_FALSE(boat.isAirborne());
    EXPECT_TRUE(boat.isSubmerged());
}

TEST(WatercraftDynamicsTest, ArchimedesBuoyancyAndHeaveEquilibrium)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 3200.0;
    bodyConfig.position = Vec2 { 0.0, -0.6 }; // Settled at draft depth
    RigidBody2D body { bodyConfig };

    WatercraftDynamics boat { };
    constexpr double dt { 1.0 / 60.0 };
    boat.step(body, dt);

    const WatercraftTelemetry& telem { boat.getTelemetry() };
    EXPECT_GT(telem.buoyancyForce, 0.0);
    EXPECT_TRUE(telem.isSubmerged);
    // At equilibrium draft, buoyancy balances vehicle gravity
    EXPECT_NEAR(telem.buoyancyForce, bodyConfig.mass * 9.80665, 10.0);
}

TEST(WatercraftDynamicsTest, PlaningModeHydrodynamicLift)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 3200.0;
    bodyConfig.position = Vec2 { 0.0, -0.4 };

    RigidBody2D slowBody { bodyConfig };
    slowBody.setLinearVelocity(Vec2 { 4.0, 0.0 }); // 4 m/s < 6 m/s planing threshold

    RigidBody2D fastBody { bodyConfig };
    fastBody.setLinearVelocity(Vec2 { 14.0, 0.0 }); // 14 m/s > 6 m/s planing threshold

    WatercraftDynamics slowBoat { };
    WatercraftDynamics fastBoat { };

    constexpr double dt { 1.0 / 60.0 };
    slowBoat.step(slowBody, dt);
    fastBoat.step(fastBody, dt);

    EXPECT_FALSE(slowBoat.isPlaning());
    EXPECT_TRUE(fastBoat.isPlaning());
    EXPECT_GT(fastBoat.getTelemetry().planingLift, 0.0);
}

TEST(WatercraftDynamicsTest, WaterRudderPropellerThrustVectoring)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 3200.0;
    bodyConfig.position = Vec2 { 0.0, -0.5 };
    RigidBody2D body { bodyConfig };

    WatercraftDynamics boat { };
    WatercraftInput input { };
    input.throttle = 1.0;
    input.rudder = 1.0; // Steer rudder hard over
    boat.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    boat.step(body, dt);

    const WatercraftTelemetry& telem { boat.getTelemetry() };
    EXPECT_GT(telem.thrustForce, 0.0);
    EXPECT_GT(boat.getRudderAngle(), 0.0);
    // Deflected rudder at stern offset produces turning moment
    EXPECT_NE(body.getAccumulatedTorque(), 0.0);
}

TEST(WatercraftDynamicsTest, WaveJumpingAirborneDetachment)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 3200.0;
    bodyConfig.position = Vec2 { 0.0, 1.5 }; // Flown completely out of water
    RigidBody2D body { bodyConfig };

    WatercraftDynamics boat { };
    WatercraftInput input { };
    input.throttle = 1.0;
    boat.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    boat.step(body, dt);

    const WatercraftTelemetry& telem { boat.getTelemetry() };
    EXPECT_TRUE(boat.isAirborne());
    EXPECT_FALSE(telem.isSubmerged);
    EXPECT_DOUBLE_EQ(telem.buoyancyForce, 0.0);
    // Propeller ventilates with minimal thrust outside water
    EXPECT_LT(telem.thrustForce, boat.getConfig().maxPropellerThrust * 0.1);
}

TEST(WatercraftDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1500.0;
    bodyConfig.inertia = 3200.0;
    bodyConfig.position = Vec2 { 0.0, -0.5 };

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    WatercraftDynamics boatA { };
    WatercraftDynamics boatB { };

    WatercraftInput input { };
    input.throttle = 0.8;
    input.rudder = 0.35;
    boatA.setInput(input);
    boatB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        boatA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        boatB.step(bodyB, dt);
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
// TrialsBikeDynamics Tests
// -----------------------------------------------------------------------------

TEST(TrialsBikeDynamicsTest, DefaultConstructionAndGeometry)
{
    const TrialsBikeDynamics bike { };
    const TrialsBikeConfig& config { bike.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 110.0);
    EXPECT_DOUBLE_EQ(config.riderMass, 75.0);
    EXPECT_DOUBLE_EQ(config.wheelbase, 1.35);
    EXPECT_DOUBLE_EQ(bike.getSpeed(), 0.0);
    EXPECT_FALSE(bike.isWheelie());
    EXPECT_FALSE(bike.isStoppie());
    EXPECT_FALSE(bike.isAirborne());
}

TEST(TrialsBikeDynamicsTest, FrontAndRearSuspensionCompression)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 185.0;
    bodyConfig.inertia = 65.0;
    RigidBody2D body { bodyConfig };

    TrialsBikeDynamics bike { };
    TrialsBikeInput input { };
    input.groundDistanceFront = 0.20; // Compressed by 0.15m below 0.35m radius
    input.groundDistanceRear = 0.20;
    bike.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    bike.step(body, dt);

    const TrialsBikeTelemetry& telem { bike.getTelemetry() };
    EXPECT_GT(telem.frontNormalForce, 0.0);
    EXPECT_GT(telem.rearNormalForce, 0.0);
    EXPECT_TRUE(telem.frontContact);
    EXPECT_TRUE(telem.rearContact);
    EXPECT_FALSE(bike.isAirborne());
}

TEST(TrialsBikeDynamicsTest, RiderWeightLeanGeneratesWheelieAndStoppie)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 185.0;
    bodyConfig.inertia = 65.0;

    // Test Wheelie: lean backward lifts front wheel
    RigidBody2D wheelieBody { bodyConfig };
    TrialsBikeDynamics wheelieBike { };
    TrialsBikeInput wheelieInput { };
    wheelieInput.riderLean = -1.0; // Hard lean backward
    wheelieInput.groundDistanceFront = 0.40; // Front wheel unloaded above ground
    wheelieInput.groundDistanceRear = 0.25; // Rear wheel compressed
    wheelieBike.setInput(wheelieInput);

    constexpr double dt { 1.0 / 60.0 };
    wheelieBike.step(wheelieBody, dt);

    EXPECT_TRUE(wheelieBike.isWheelie());
    EXPECT_FALSE(wheelieBike.isStoppie());
    EXPECT_FALSE(wheelieBike.isAirborne());

    // Test Stoppie: lean forward lifts rear wheel
    RigidBody2D stoppieBody { bodyConfig };
    TrialsBikeDynamics stoppieBike { };
    TrialsBikeInput stoppieInput { };
    stoppieInput.riderLean = 1.0; // Hard lean forward
    stoppieInput.groundDistanceFront = 0.25; // Front wheel compressed
    stoppieInput.groundDistanceRear = 0.40; // Rear wheel unloaded
    stoppieBike.setInput(stoppieInput);

    stoppieBike.step(stoppieBody, dt);

    EXPECT_TRUE(stoppieBike.isStoppie());
    EXPECT_FALSE(stoppieBike.isWheelie());
    EXPECT_FALSE(stoppieBike.isAirborne());
}

TEST(TrialsBikeDynamicsTest, InAirWheelAngularMomentumReaction)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 185.0;
    bodyConfig.inertia = 65.0;

    RigidBody2D gasBody { bodyConfig };
    RigidBody2D brakeBody { bodyConfig };

    TrialsBikeDynamics gasBike { };
    TrialsBikeDynamics brakeBike { };

    TrialsBikeInput gasInput { };
    gasInput.groundDistanceFront = 1.0; // Airborne
    gasInput.groundDistanceRear = 1.0;
    gasInput.throttle = 1.0; // Full throttle rev in air

    TrialsBikeInput brakeInput { };
    brakeInput.groundDistanceFront = 1.0; // Airborne
    brakeInput.groundDistanceRear = 1.0;
    brakeInput.rearBrake = 1.0; // Full rear brake in air

    gasBike.setInput(gasInput);
    brakeBike.setInput(brakeInput);

    constexpr double dt { 1.0 / 60.0 };
    gasBike.step(gasBody, dt);
    brakeBike.step(brakeBody, dt);

    EXPECT_TRUE(gasBike.isAirborne());
    EXPECT_TRUE(brakeBike.isAirborne());
    // Throttle in air pitches chassis backward (nose up > 0)
    EXPECT_GT(gasBody.getAccumulatedTorque(), 0.0);
    // Rear brake in air pitches chassis forward (nose down < 0)
    EXPECT_LT(brakeBody.getAccumulatedTorque(), 0.0);
}

TEST(TrialsBikeDynamicsTest, RearWheelDriveAcceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 185.0;
    bodyConfig.inertia = 65.0;
    RigidBody2D body { bodyConfig };

    TrialsBikeDynamics bike { };
    TrialsBikeInput input { };
    input.groundDistanceFront = 0.30;
    input.groundDistanceRear = 0.30;
    input.throttle = 1.0;
    bike.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        bike.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_GT(bike.getSpeed(), 0.0);
}

TEST(TrialsBikeDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 185.0;
    bodyConfig.inertia = 65.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    TrialsBikeDynamics bikeA { };
    TrialsBikeDynamics bikeB { };

    TrialsBikeInput input { };
    input.throttle = 0.75;
    input.riderLean = -0.3;
    input.groundDistanceFront = 0.30;
    input.groundDistanceRear = 0.28;
    bikeA.setInput(input);
    bikeB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        bikeA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        bikeB.step(bodyB, dt);
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
// HelicopterDynamics Tests
// -----------------------------------------------------------------------------

TEST(HelicopterDynamicsTest, DefaultConstructionAndRotorProfile)
{
    const HelicopterDynamics heli { };
    const HelicopterConfig& config { heli.getConfig() };

    EXPECT_DOUBLE_EQ(config.mass, 2200.0);
    EXPECT_DOUBLE_EQ(config.maxRotorRpm, 420.0);
    EXPECT_DOUBLE_EQ(config.maxCollectiveThrust, 32000.0);
    EXPECT_DOUBLE_EQ(heli.getRotorSpeed(), 420.0);
    EXPECT_FALSE(heli.isAutorotating());
    EXPECT_TRUE(heli.isAirborne());
}

TEST(HelicopterDynamicsTest, CollectivePitchVerticalClimb)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;
    RigidBody2D body { bodyConfig };

    HelicopterDynamics heli { };
    HelicopterInput input { };
    input.collective = 1.0; // Maximum collective climb
    input.groundDistance = 15.0; // Free air out of ground effect
    heli.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        heli.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Collective thrust exceeds gravity, climbing upward
    EXPECT_GT(body.getLinearVelocity().y, 0.0);
    EXPECT_GT(heli.getTelemetry().climbRate, 0.0);
    EXPECT_GT(heli.getTelemetry().rotorThrust, 2200.0 * 9.80665);
}

TEST(HelicopterDynamicsTest, InGroundEffectThrustAugmentation)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;

    RigidBody2D igeBody { bodyConfig };
    RigidBody2D ogeBody { bodyConfig };

    HelicopterDynamics igeHeli { };
    HelicopterDynamics ogeHeli { };

    HelicopterInput igeInput { };
    igeInput.collective = 0.7;
    igeInput.groundDistance = 1.5; // Within 6m ground cushion

    HelicopterInput ogeInput { };
    ogeInput.collective = 0.7;
    ogeInput.groundDistance = 20.0; // High altitude out-of-ground effect

    igeHeli.setInput(igeInput);
    ogeHeli.setInput(ogeInput);

    constexpr double dt { 1.0 / 60.0 };
    igeHeli.step(igeBody, dt);
    ogeHeli.step(ogeBody, dt);

    EXPECT_TRUE(igeHeli.getTelemetry().isGroundEffect);
    EXPECT_FALSE(ogeHeli.getTelemetry().isGroundEffect);
    // In-ground effect cushions and amplifies rotor thrust
    EXPECT_GT(igeHeli.getTelemetry().rotorThrust, ogeHeli.getTelemetry().rotorThrust);
}

TEST(HelicopterDynamicsTest, EffectiveTranslationalLiftSpeedBoost)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;

    RigidBody2D hoverBody { bodyConfig };
    hoverBody.setLinearVelocity(Vec2 { 0.0, 0.0 });

    RigidBody2D forwardBody { bodyConfig };
    forwardBody.setLinearVelocity(Vec2 { 18.0, 0.0 }); // 18 m/s > 8 m/s ETL threshold

    HelicopterDynamics hoverHeli { };
    HelicopterDynamics forwardHeli { };

    HelicopterInput input { };
    input.collective = 0.8;
    input.groundDistance = 20.0;
    hoverHeli.setInput(input);
    forwardHeli.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    hoverHeli.step(hoverBody, dt);
    forwardHeli.step(forwardBody, dt);

    EXPECT_FALSE(hoverHeli.getTelemetry().isTranslationalLift);
    EXPECT_TRUE(forwardHeli.getTelemetry().isTranslationalLift);
    EXPECT_GT(forwardHeli.getTelemetry().rotorThrust, hoverHeli.getTelemetry().rotorThrust);
}

TEST(HelicopterDynamicsTest, CyclicPitchStickTiltMoments)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;
    RigidBody2D body { bodyConfig };

    HelicopterDynamics heli { };
    HelicopterInput input { };
    input.collective = 0.8;
    input.cyclicPitch = 1.0; // Push stick forward (nose down)
    heli.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    heli.step(body, dt);

    // Pushing stick forward generates pitch torque and forward thrust vectoring
    EXPECT_LT(body.getAccumulatedTorque(), 0.0);
    EXPECT_GT(body.getAccumulatedForce().x, 0.0);
}

TEST(HelicopterDynamicsTest, TailRotorAntiTorqueBalance)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;

    RigidBody2D pedalLeft { bodyConfig };
    RigidBody2D pedalRight { bodyConfig };

    HelicopterDynamics heliLeft { };
    HelicopterDynamics heliRight { };

    HelicopterInput inputLeft { };
    inputLeft.collective = 0.7;
    inputLeft.rudder = -1.0;

    HelicopterInput inputRight { };
    inputRight.collective = 0.7;
    inputRight.rudder = 1.0;

    heliLeft.setInput(inputLeft);
    heliRight.setInput(inputRight);

    constexpr double dt { 1.0 / 60.0 };
    heliLeft.step(pedalLeft, dt);
    heliRight.step(pedalRight, dt);

    EXPECT_NE(pedalLeft.getAccumulatedTorque(), pedalRight.getAccumulatedTorque());
}

TEST(HelicopterDynamicsTest, EngineFlameoutAutorotationDescent)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 0.0, -12.0 }); // Descending through air

    HelicopterDynamics heli { };
    HelicopterInput input { };
    input.enginePower = false; // Turboshaft engine failure
    input.collective = 0.3;
    heli.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    heli.step(body, dt);

    EXPECT_TRUE(heli.isAutorotating());
    // Upward airflow in autorotation descent sustains rotor RPM
    EXPECT_GT(heli.getRotorSpeed(), 100.0);
}

TEST(HelicopterDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2200.0;
    bodyConfig.inertia = 4800.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    HelicopterDynamics heliA { };
    HelicopterDynamics heliB { };

    HelicopterInput input { };
    input.collective = 0.85;
    input.cyclicPitch = 0.25;
    input.rudder = 0.15;
    input.groundDistance = 8.0;
    heliA.setInput(input);
    heliB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        heliA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        heliB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
    EXPECT_DOUBLE_EQ(heliA.getRotorSpeed(), heliB.getRotorSpeed());
}

// -----------------------------------------------------------------------------
// SubmarineDynamics Tests
// -----------------------------------------------------------------------------

TEST(SubmarineDynamicsTest, DefaultConstructionAndBuoyancyProfile)
{
    SubmarineConfig config { };
    SubmarineDynamics sub { config };

    EXPECT_DOUBLE_EQ(sub.getConfig().dryMass, 45000.0);
    EXPECT_DOUBLE_EQ(sub.getBallastMass(), 7000.0);
    EXPECT_DOUBLE_EQ(sub.getTelemetry().totalMass, 52000.0);
    EXPECT_DOUBLE_EQ(sub.getDepth(), 0.0);
    EXPECT_FALSE(sub.isSubmerged());
    EXPECT_TRUE(sub.isAirborne());
    EXPECT_FALSE(sub.isCrushed());
}

TEST(SubmarineDynamicsTest, BallastFloodingIncreasesMassAndDecreasesNetBuoyancy)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -10.0 }; // Deep underwater
    RigidBody2D body { bodyConfig };

    SubmarineDynamics sub { };
    SubmarineInput input { };
    input.floodRate = 1.0;
    sub.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        sub.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(sub.isSubmerged());
    EXPECT_FALSE(sub.isAirborne());
    EXPECT_GT(sub.getBallastMass(), 7000.0);
    EXPECT_GT(sub.getTelemetry().totalMass, 52000.0);
    EXPECT_LT(sub.getNetBuoyancy(), 0.0); // Negative net buoyancy causes sinking
    EXPECT_LT(body.getLinearVelocity().y, 0.0);
}

TEST(SubmarineDynamicsTest, BallastBlowingRestoresPositiveBuoyancyAndAscent)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -25.0 }; // Submerged
    RigidBody2D body { bodyConfig };

    SubmarineDynamics sub { };
    SubmarineInput input { };
    input.blowRate = 1.0; // Blow air into tanks, expelling water
    sub.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        sub.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(sub.getBallastMass(), 7000.0);
    EXPECT_GT(sub.getNetBuoyancy(), 0.0); // Positive buoyancy produces upward buoyant force
    EXPECT_GT(body.getLinearVelocity().y, 0.0);
}

TEST(SubmarineDynamicsTest, HydrofoilDivePlanePitchAndDepthControl)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -50.0 };
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 8.0, 0.0 }); // Cruising forward underwater

    SubmarineDynamics sub { };
    SubmarineInput input { };
    input.throttle = 0.5;
    input.divePlane = 1.0; // Bow planes angled up
    sub.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 30; ++step) {
        sub.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(sub.getTelemetry().divePlaneAngle, 0.0);
    EXPECT_GT(body.getAccumulatedTorque() + body.getAngularVelocity(), 0.0);
}

TEST(SubmarineDynamicsTest, ScrewPropellerThrustAndWaterRudderYawMoments)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -30.0 };
    RigidBody2D body { bodyConfig };

    SubmarineDynamics sub { };
    SubmarineInput input { };
    input.throttle = 1.0;
    input.rudder = 0.75;
    sub.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        sub.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_NE(body.getAngularVelocity(), 0.0);
    EXPECT_GT(sub.getTelemetry().thrustForce, 0.0);
}

TEST(SubmarineDynamicsTest, CrushDepthWarning)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -500.0 }; // Exceeds 450m crush depth
    RigidBody2D body { bodyConfig };

    SubmarineDynamics sub { };
    constexpr double dt { 1.0 / 60.0 };
    sub.step(body, dt);

    EXPECT_TRUE(sub.isSubmerged());
    EXPECT_TRUE(sub.isCrushed());
    EXPECT_GE(sub.getDepth(), 450.0);
}

TEST(SubmarineDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 52000.0;
    bodyConfig.inertia = 120000.0;
    bodyConfig.position = Vec2 { 0.0, -40.0 };

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    SubmarineDynamics subA { };
    SubmarineDynamics subB { };

    SubmarineInput input { };
    input.throttle = 0.75;
    input.rudder = 0.2;
    input.divePlane = -0.15;
    input.floodRate = 0.1;
    subA.setInput(input);
    subB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        subA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        subB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
    EXPECT_DOUBLE_EQ(subA.getBallastMass(), subB.getBallastMass());
}

// -----------------------------------------------------------------------------
// AntiGravityRacerDynamics Tests
// -----------------------------------------------------------------------------

TEST(AntiGravityRacerDynamicsTest, DefaultConstructionAndMaglevProfile)
{
    AntiGravityConfig config { };
    AntiGravityRacerDynamics racer { config };

    EXPECT_DOUBLE_EQ(racer.getConfig().mass, 1050.0);
    EXPECT_DOUBLE_EQ(racer.getRideHeight(), config.targetRideHeight);
    EXPECT_DOUBLE_EQ(racer.getBoostEnergy(), config.maxBoostEnergy);
    EXPECT_TRUE(racer.isLevitating());
    EXPECT_FALSE(racer.isAirborne());
    EXPECT_FALSE(racer.isBoostActive());
    EXPECT_FALSE(racer.isDrifting());
}

TEST(AntiGravityRacerDynamicsTest, MagneticLevitationRepulsionRestoresTargetRideHeight)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1050.0;
    bodyConfig.inertia = 1800.0;
    RigidBody2D body { bodyConfig };

    AntiGravityRacerDynamics racer { };
    AntiGravityInput input { };
    input.trackDistance = 0.5; // Compressed track clearance below 1.2m target
    racer.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    racer.step(body, dt);

    EXPECT_TRUE(racer.isLevitating());
    EXPECT_GT(racer.getTelemetry().suspensionForce, 0.0);
    // Accumulated upward repulsion exceeds gravity
    EXPECT_GT(body.getAccumulatedForce().y, 0.0);
}

TEST(AntiGravityRacerDynamicsTest, DifferentialAirbrakesCreateRetardingDragAndSharpYawTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1050.0;
    bodyConfig.inertia = 1800.0;
    RigidBody2D bodyLeft { bodyConfig };
    bodyLeft.setLinearVelocity(Vec2 { 60.0, 0.0 }); // High speed straight
    RigidBody2D bodyRight { bodyConfig };
    bodyRight.setLinearVelocity(Vec2 { 60.0, 0.0 });

    AntiGravityRacerDynamics racerLeft { };
    AntiGravityRacerDynamics racerRight { };

    AntiGravityInput inputLeft { };
    inputLeft.leftAirbrake = 1.0;
    racerLeft.setInput(inputLeft);

    AntiGravityInput inputRight { };
    inputRight.rightAirbrake = 1.0;
    racerRight.setInput(inputRight);

    constexpr double dt { 1.0 / 60.0 };
    racerLeft.step(bodyLeft, dt);
    racerRight.step(bodyRight, dt);

    EXPECT_GT(racerLeft.getTelemetry().airbrakeDrag, 0.0);
    EXPECT_GT(racerRight.getTelemetry().airbrakeDrag, 0.0);
    // Left airbrake generates positive counter-clockwise yaw torque (turns nose left)
    EXPECT_GT(racerLeft.getTelemetry().netYawTorque, 0.0);
    // Right airbrake generates negative clockwise yaw torque (turns nose right)
    EXPECT_LT(racerRight.getTelemetry().netYawTorque, 0.0);
}

TEST(AntiGravityRacerDynamicsTest, BoostCapacitorThrustMultiplierAndEnergyDepletion)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1050.0;
    bodyConfig.inertia = 1800.0;
    RigidBody2D bodyNormal { bodyConfig };
    RigidBody2D bodyBoost { bodyConfig };

    AntiGravityRacerDynamics normalRacer { };
    AntiGravityRacerDynamics boostRacer { };

    AntiGravityInput normalInput { };
    normalInput.throttle = 1.0;
    normalRacer.setInput(normalInput);

    AntiGravityInput boostInput { };
    boostInput.throttle = 1.0;
    boostInput.boost = true;
    boostRacer.setInput(boostInput);

    constexpr double dt { 1.0 / 60.0 };
    normalRacer.step(bodyNormal, dt);
    boostRacer.step(bodyBoost, dt);

    EXPECT_TRUE(boostRacer.isBoostActive());
    EXPECT_FALSE(normalRacer.isBoostActive());
    EXPECT_GT(boostRacer.getTelemetry().thrustForce, normalRacer.getTelemetry().thrustForce);
    EXPECT_LT(boostRacer.getBoostEnergy(), normalRacer.getBoostEnergy());
}

TEST(AntiGravityRacerDynamicsTest, HighSpeedLateralSlipDrifting)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1050.0;
    bodyConfig.inertia = 1800.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 45.0, 12.0 }); // Large lateral velocity

    AntiGravityRacerDynamics racer { };
    constexpr double dt { 1.0 / 60.0 };
    racer.step(body, dt);

    EXPECT_TRUE(racer.isDrifting());
    EXPECT_GT(std::abs(racer.getTelemetry().lateralSpeed), 6.5);
}

TEST(AntiGravityRacerDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1050.0;
    bodyConfig.inertia = 1800.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    AntiGravityRacerDynamics racerA { };
    AntiGravityRacerDynamics racerB { };

    AntiGravityInput input { };
    input.throttle = 0.9;
    input.steer = 0.35;
    input.leftAirbrake = 0.25;
    input.trackDistance = 1.1;
    racerA.setInput(input);
    racerB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        racerA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        racerB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
    EXPECT_DOUBLE_EQ(racerA.getBoostEnergy(), racerB.getBoostEnergy());
}

// -----------------------------------------------------------------------------
// ArticulatedTruckDynamics Tests
// -----------------------------------------------------------------------------

TEST(ArticulatedTruckDynamicsTest, DefaultConstructionAndFifthWheelCoupling)
{
    ArticulatedTruckConfig config { };
    ArticulatedTruckDynamics truck { config };

    EXPECT_DOUBLE_EQ(truck.getConfig().tractorMass, 7500.0);
    EXPECT_DOUBLE_EQ(truck.getConfig().trailerMass, 24000.0);
    EXPECT_DOUBLE_EQ(truck.getHitchAngle(), 0.0);
    EXPECT_DOUBLE_EQ(truck.getTrailerHeading(), 0.0);
    EXPECT_DOUBLE_EQ(truck.getSteerAngle(), 0.0);
    EXPECT_DOUBLE_EQ(truck.getSpeed(), 0.0);
    EXPECT_FALSE(truck.isJackknifed());
    EXPECT_FALSE(truck.isAirborne());
}

TEST(ArticulatedTruckDynamicsTest, FifthWheelArticulationFollowsTractorTurn)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 7500.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };

    ArticulatedTruckDynamics truck { };
    ArticulatedTruckInput input { };
    input.throttle = 0.8;
    input.steer = 0.5; // Steer left
    truck.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 90; ++step) {
        truck.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(truck.getSpeed(), 0.0);
    EXPECT_GT(truck.getSteerAngle(), 0.0);
    // Articulation angle develops as trailer trails the turning tractor
    EXPECT_NE(truck.getHitchAngle(), 0.0);
    EXPECT_GT(truck.getTelemetry().offTrackingDistance, 0.0);
}

TEST(ArticulatedTruckDynamicsTest, ReverseManeuveringInducesJackknifeDivergence)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 7500.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };

    ArticulatedTruckDynamics truck { };
    truck.setHitchAngle(0.85); // Articulation offset entering reverse
    ArticulatedTruckInput input { };
    input.throttle = -0.8; // Hard reverse
    input.steer = -0.45; // Steering opposite to hitch angle induces rapid jackknife divergence
    truck.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 240; ++step) {
        truck.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, 0.0);
    // Reversing at an angle triggers jackknifing instability
    EXPECT_TRUE(truck.isJackknifed());
    EXPECT_GE(std::abs(truck.getHitchAngle()), truck.getConfig().jackknifeThresholdAngle);
}

TEST(ArticulatedTruckDynamicsTest, IndependentTrailerBrakeStretchesRigAndSuppressesJackknife)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 7500.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D bodyNormal { bodyConfig };
    bodyNormal.setLinearVelocity(Vec2 { 15.0, 0.0 });
    RigidBody2D bodyTrailerBrake { bodyConfig };
    bodyTrailerBrake.setLinearVelocity(Vec2 { 15.0, 0.0 });

    ArticulatedTruckDynamics truckNormal { };
    ArticulatedTruckDynamics truckTrailerBrake { };

    ArticulatedTruckInput inputNormal { };
    inputNormal.serviceBrake = 1.0; // Hard foot brake
    inputNormal.steer = 0.5;
    truckNormal.setInput(inputNormal);

    ArticulatedTruckInput inputTrailerBrake { };
    inputTrailerBrake.serviceBrake = 1.0;
    inputTrailerBrake.trailerBrake = 1.0; // Trolley valve engaged pulling trailer back
    inputTrailerBrake.steer = 0.5;
    truckTrailerBrake.setInput(inputTrailerBrake);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        truckNormal.step(bodyNormal, dt);
        Integrator2D::integrate(bodyNormal, dt);
        bodyNormal.clearForces();

        truckTrailerBrake.step(bodyTrailerBrake, dt);
        Integrator2D::integrate(bodyTrailerBrake, dt);
        bodyTrailerBrake.clearForces();
    }

    EXPECT_GT(truckTrailerBrake.getTelemetry().trailerBrakeForce, 0.0);
    // Trolley valve hand-brake tension stabilizes the kingpin hitch angle
    EXPECT_LT(std::abs(truckTrailerBrake.getHitchAngle()), std::abs(truckNormal.getHitchAngle()) + 0.1);
}

TEST(ArticulatedTruckDynamicsTest, TractorAndTrailerBrakingDeceleration)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 7500.0;
    bodyConfig.inertia = 25000.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(Vec2 { 22.0, 0.0 });

    ArticulatedTruckDynamics truck { };
    ArticulatedTruckInput input { };
    input.serviceBrake = 1.0;
    truck.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        truck.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(body.getLinearVelocity().x, 22.0);
    EXPECT_GT(truck.getTelemetry().tractorBrakeForce, 0.0);
    EXPECT_GT(truck.getTelemetry().trailerBrakeForce, 0.0);
}

TEST(ArticulatedTruckDynamicsTest, DeterministicRepeatability)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 7500.0;
    bodyConfig.inertia = 25000.0;

    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };

    ArticulatedTruckDynamics truckA { };
    ArticulatedTruckDynamics truckB { };

    ArticulatedTruckInput input { };
    input.throttle = 0.6;
    input.steer = 0.25;
    input.serviceBrake = 0.1;
    truckA.setInput(input);
    truckB.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 120; ++step) {
        truckA.step(bodyA, dt);
        Integrator2D::integrate(bodyA, dt);
        bodyA.clearForces();

        truckB.step(bodyB, dt);
        Integrator2D::integrate(bodyB, dt);
        bodyB.clearForces();
    }

    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
    EXPECT_DOUBLE_EQ(bodyA.getOrientation(), bodyB.getOrientation());
    EXPECT_DOUBLE_EQ(bodyA.getAngularVelocity(), bodyB.getAngularVelocity());
    EXPECT_DOUBLE_EQ(truckA.getHitchAngle(), truckB.getHitchAngle());
}

// -----------------------------------------------------------------------------
// Thirteen-Way Swappable Coordinator Polymorphism Tests
// -----------------------------------------------------------------------------

TEST(VehicleDynamicsCoordinatorTest, AllThirteenDynamicsModelsPolymorphicSwapping)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 2000.0;
    bodyConfig.inertia = 3500.0;
    RigidBody2D sharedBody { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };

    std::unique_ptr<IVehicleDynamics> models[13] {
        std::make_unique<ArcadeCarDynamics>(),
        std::make_unique<MonsterCarDynamics>(),
        std::make_unique<SideViewCarDynamics>(),
        std::make_unique<RocketDynamics>(),
        std::make_unique<TrackedTankDynamics>(),
        std::make_unique<AerodynamicPlaneDynamics>(),
        std::make_unique<HovercraftDynamics>(),
        std::make_unique<WatercraftDynamics>(),
        std::make_unique<TrialsBikeDynamics>(),
        std::make_unique<HelicopterDynamics>(),
        std::make_unique<SubmarineDynamics>(),
        std::make_unique<AntiGravityRacerDynamics>(),
        std::make_unique<ArticulatedTruckDynamics>()
    };

    for (auto& coordinator : models) {
        ASSERT_NE(coordinator, nullptr);
        coordinator->reset();
        coordinator->step(sharedBody, dt);
        Integrator2D::integrate(sharedBody, dt);
        sharedBody.clearForces();
    }
}

} // namespace vehicle_sim::testing


