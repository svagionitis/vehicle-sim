#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/dynamics/VTOLTiltRotorDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::VTOLFlightMode;
using vehicle_sim::dynamics::VTOLTiltRotorConfig;
using vehicle_sim::dynamics::VTOLTiltRotorDynamics;
using vehicle_sim::dynamics::VTOLTiltRotorInput;
using vehicle_sim::physics::Integrator2D;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kTol { 1e-4 };

// -----------------------------------------------------------------------------
// VTOLTiltRotorDynamics Tests
// -----------------------------------------------------------------------------

TEST(VTOLTiltRotorDynamicsTest, DefaultConstructionAndProfile)
{
    const VTOLTiltRotorDynamics vtol { };
    const auto& config = vtol.getConfig();

    EXPECT_GT(config.mass, 0.0);
    EXPECT_GT(config.wingArea, 0.0);
    EXPECT_GT(config.maxTotalThrust, 0.0);
    EXPECT_NEAR(vtol.getNacelleAngle(), config.maxNacelleAngle, kTol);
    EXPECT_NEAR(vtol.getNacelleDeg(), 90.0, 0.1);
    EXPECT_EQ(vtol.getFlightMode(), VTOLFlightMode::Hover);
    EXPECT_TRUE(vtol.isCorridorSafe());
    EXPECT_TRUE(vtol.isAirborne());
}

TEST(VTOLTiltRotorDynamicsTest, HoverModeVerticalClimb)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 }; // Zero gravity to measure pure thrust

    VTOLTiltRotorDynamics vtol { config };
    VTOLTiltRotorInput input { };
    input.throttle = 1.0;
    input.targetNacelleAngle = config.maxNacelleAngle; // 90 deg
    vtol.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body { bodyConfig };

    constexpr double dt { 0.1 };
    vtol.step(body, dt);

    const auto& telem = vtol.getTelemetry();
    EXPECT_GT(telem.rotorThrust, 0.0);
    EXPECT_NEAR(telem.wingLift, 0.0, kTol);
    EXPECT_EQ(telem.flightMode, VTOLFlightMode::Hover);

    // Forces should produce upward acceleration along +y
    Integrator2D::integrate(body, dt);
    EXPECT_GT(body.getLinearVelocity().y, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().x, 0.0, kTol);
}

TEST(VTOLTiltRotorDynamicsTest, NacelleConversionSlewRate)
{
    VTOLTiltRotorConfig config { };
    config.maxTiltSlewRate = 0.14; // rad/s
    VTOLTiltRotorDynamics vtol { config };

    // Command full transition to cruise mode (0.0 rad)
    VTOLTiltRotorInput input { };
    input.targetNacelleAngle = 0.0;
    vtol.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 }; // 1.0 second elapsed
    vtol.step(body, dt);

    // Initial was ~1.5708. After 1 sec at 0.14 rad/s, it should be clamped to ~1.4308
    const double expectedAngle { config.maxNacelleAngle - (config.maxTiltSlewRate * dt) };
    EXPECT_NEAR(vtol.getNacelleAngle(), expectedAngle, kTol);
    EXPECT_LT(vtol.getNacelleDeg(), 90.0);
    EXPECT_GT(vtol.getNacelleDeg(), 0.0);
}

TEST(VTOLTiltRotorDynamicsTest, AirplaneCruiseForwardFlight)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 };

    VTOLTiltRotorDynamics vtol { config };

    // Fast forward conversion to 0.0 rad
    VTOLTiltRotorInput input { };
    input.throttle = 1.0;
    input.targetNacelleAngle = 0.0;
    vtol.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(math::Vec2 { 60.0, 0.0 }); // Cruise airspeed 60 m/s

    // Step enough ticks to reach cruise mode (0.0 rad)
    constexpr double dt { 0.1 };
    for (int i { 0 }; i < 150; ++i) {
        vtol.step(body, dt);
    }

    EXPECT_NEAR(vtol.getNacelleAngle(), 0.0, kTol);
    EXPECT_EQ(vtol.getFlightMode(), VTOLFlightMode::AirplaneCruise);

    const auto& telem = vtol.getTelemetry();
    EXPECT_GT(telem.airspeed, 50.0);
    EXPECT_GT(telem.rotorThrust, 0.0);

    // Clear forces and step one tick
    body.clearForces();
    vtol.step(body, dt);
    Integrator2D::integrate(body, dt);

    // Thrust in cruise acts horizontally along +x
    EXPECT_GT(body.getLinearVelocity().x, 60.0);
}

TEST(VTOLTiltRotorDynamicsTest, DynamicLiftBlendingTransition)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 };

    VTOLTiltRotorDynamics vtol { config };
    VTOLTiltRotorInput input { };
    input.throttle = 0.8;
    input.targetNacelleAngle = kPi / 4.0; // 45 degrees
    vtol.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(math::Vec2 { 30.0, 0.0 });

    // Slew towards 45 deg
    for (int i { 0 }; i < 80; ++i) {
        vtol.step(body, 0.1);
    }

    EXPECT_NEAR(vtol.getNacelleAngle(), kPi / 4.0, 0.02);
    const auto& telem = vtol.getTelemetry();

    // In mid-conversion, rotor contributes both vertical lift and forward thrust
    EXPECT_GT(telem.rotorThrust, 0.0);
    EXPECT_GT(telem.dynamicPressure, 0.0);
    EXPECT_TRUE(telem.flightMode == VTOLFlightMode::ConversionForward
        || telem.flightMode == VTOLFlightMode::ConversionAft
        || telem.flightMode == VTOLFlightMode::Hover);
}

TEST(VTOLTiltRotorDynamicsTest, WingDownloadHoverPenalty)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 };

    VTOLTiltRotorDynamics cleanVtol { config };
    VTOLTiltRotorDynamics flappedVtol { config };

    VTOLTiltRotorInput cleanInput { };
    cleanInput.throttle = 1.0;
    cleanInput.flaps = 0.0;
    cleanInput.targetNacelleAngle = config.maxNacelleAngle;
    cleanVtol.setInput(cleanInput);

    VTOLTiltRotorInput flappedInput { };
    flappedInput.throttle = 1.0;
    flappedInput.flaps = 1.0;
    flappedInput.targetNacelleAngle = config.maxNacelleAngle;
    flappedVtol.setInput(flappedInput);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D cleanBody { bodyConfig };
    RigidBody2D flappedBody { bodyConfig };

    cleanVtol.step(cleanBody, 0.1);
    flappedVtol.step(flappedBody, 0.1);

    const auto& cleanTelem = cleanVtol.getTelemetry();
    const auto& flappedTelem = flappedVtol.getTelemetry();

    EXPECT_GT(cleanTelem.wingDownload, 0.0);
    EXPECT_GT(flappedTelem.wingDownload, 0.0);

    // Deploying flaps decreases wing download obstruction
    EXPECT_LT(flappedTelem.wingDownload, cleanTelem.wingDownload);
}

TEST(VTOLTiltRotorDynamicsTest, InGroundEffectAugmentation)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 };

    VTOLTiltRotorDynamics igeVtol { config };
    VTOLTiltRotorDynamics ogeVtol { config };

    VTOLTiltRotorInput igeInput { };
    igeInput.throttle = 1.0;
    igeInput.groundDistance = 2.0; // In-ground effect (ceiling is 8m)
    igeInput.targetNacelleAngle = config.maxNacelleAngle;
    igeVtol.setInput(igeInput);

    VTOLTiltRotorInput ogeInput { };
    ogeInput.throttle = 1.0;
    ogeInput.groundDistance = 25.0; // Out of ground effect
    ogeInput.targetNacelleAngle = config.maxNacelleAngle;
    ogeVtol.setInput(ogeInput);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body1 { bodyConfig };
    RigidBody2D body2 { bodyConfig };

    igeVtol.step(body1, 0.1);
    ogeVtol.step(body2, 0.1);

    EXPECT_TRUE(igeVtol.getTelemetry().isGroundEffect);
    EXPECT_FALSE(ogeVtol.getTelemetry().isGroundEffect);
    EXPECT_GT(igeVtol.getRotorThrust(), ogeVtol.getRotorThrust());
}

TEST(VTOLTiltRotorDynamicsTest, ConversionCorridorViolation)
{
    VTOLTiltRotorConfig config { };
    config.stallAirspeed = 40.0;
    VTOLTiltRotorDynamics vtol { config };

    // Low forward speed with nacelle tilted to near-horizontal cruise
    VTOLTiltRotorInput input { };
    input.targetNacelleAngle = 0.1; // ~5.7 deg
    vtol.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity(math::Vec2 { 5.0, 0.0 }); // Dangerously below stall speed

    // Step to let nacelle tilt forward
    for (int i { 0 }; i < 150; ++i) {
        vtol.step(body, 0.1);
    }

    const auto& telem = vtol.getTelemetry();
    EXPECT_GT(telem.minCorridorSpeed, 10.0);
    EXPECT_LT(telem.airspeed, telem.minCorridorSpeed);
    EXPECT_FALSE(vtol.isCorridorSafe());
}

TEST(VTOLTiltRotorDynamicsTest, ControlCrossFadingPitch)
{
    VTOLTiltRotorConfig config { };
    config.gravity = math::Vec2 { 0.0, 0.0 };
    config.cyclicPitchAuth = 20000.0;
    config.elevatorPitchAuth = 30000.0;

    // Hover case: zero airspeed, nacelle at 90 deg
    VTOLTiltRotorDynamics hoverVtol { config };
    VTOLTiltRotorInput hoverInput { };
    hoverInput.pitchStick = 1.0;
    hoverInput.targetNacelleAngle = config.maxNacelleAngle;
    hoverVtol.setInput(hoverInput);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = config.mass;
    RigidBody2D hoverBody { bodyConfig };

    hoverVtol.step(hoverBody, 0.1);
    Integrator2D::integrate(hoverBody, 0.1);

    // Angular velocity should be non-zero due to cyclic torque
    EXPECT_NE(hoverBody.getAngularVelocity(), 0.0);
}

TEST(VTOLTiltRotorDynamicsTest, FloatExactDeterminism)
{
    VTOLTiltRotorDynamics vtolA { };
    VTOLTiltRotorDynamics vtolB { };

    VTOLTiltRotorInput input { };
    input.throttle = 0.75;
    input.targetNacelleAngle = 0.5;
    input.pitchStick = 0.25;
    input.flaps = 0.5;

    vtolA.setInput(input);
    vtolB.setInput(input);

    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 6800.0;
    RigidBody2D bodyA { bodyConfig };
    RigidBody2D bodyB { bodyConfig };
    bodyA.setLinearVelocity(math::Vec2 { 25.0, 2.0 });
    bodyB.setLinearVelocity(math::Vec2 { 25.0, 2.0 });

    for (int step { 0 }; step < 60; ++step) {
        vtolA.step(bodyA, 1.0 / 60.0);
        vtolB.step(bodyB, 1.0 / 60.0);
        Integrator2D::integrate(bodyA, 1.0 / 60.0);
        Integrator2D::integrate(bodyB, 1.0 / 60.0);
    }

    EXPECT_DOUBLE_EQ(vtolA.getAirspeed(), vtolB.getAirspeed());
    EXPECT_DOUBLE_EQ(vtolA.getNacelleAngle(), vtolB.getNacelleAngle());
    EXPECT_DOUBLE_EQ(vtolA.getRotorThrust(), vtolB.getRotorThrust());
    EXPECT_DOUBLE_EQ(vtolA.getWingLift(), vtolB.getWingLift());
    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
}

} // namespace vehicle_sim::testing
