#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/BicycleDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

TEST(BicycleDynamicsTest, DefaultConstructionAndTrail) {
    const BicycleDynamics bike{};
    EXPECT_FALSE(bike.isAirborne());

    const auto& config = bike.getConfig();
    EXPECT_GT(config.totalMass, 0.0);
    EXPECT_GT(config.wheelbase, 0.0);
    EXPECT_GT(config.frontWheelRadius, 0.0);

    const auto& telem = bike.getTelemetry();
    EXPECT_DOUBLE_EQ(telem.rollAngle, 0.0);
}

TEST(BicycleDynamicsTest, MechanicalTrailPositive) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 5.0, 0.0 });
    bike.step(body, 0.016);

    const auto& telem = bike.getTelemetry();
    EXPECT_GT(telem.mechanicalTrail, 0.02); // Typical bicycle trail ~0.05-0.07 m
}

TEST(BicycleDynamicsTest, CounterSteeringInitiation) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 6.0, 0.0 });

    // Rider initiates right turn by momentarily steering left (negative steer torque)
    BicycleInput input{};
    input.steerTorque = -8.0;
    bike.setInput(input);

    bike.step(body, 0.016);

    const auto& telem = bike.getTelemetry();
    EXPECT_LT(telem.steerAngle, 0.0); // Steer deflected left
    EXPECT_GT(telem.rollRate, 0.0); // Roll accelerating right
    EXPECT_TRUE(telem.isCounterSteering);
}

TEST(BicycleDynamicsTest, SelfStabilityRegimes) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    // Below weave speed (v = 2.0 m/s)
    body.setLinearVelocity(math::Vec2{ 2.0, 0.0 });
    bike.step(body, 0.016);
    EXPECT_FALSE(bike.getTelemetry().isSelfStable);
    EXPECT_FALSE(bike.getTelemetry().isCapsizeMode);

    // Hands-free stable speed (v = 6.0 m/s)
    body.setLinearVelocity(math::Vec2{ 6.0, 0.0 });
    bike.step(body, 0.016);
    EXPECT_TRUE(bike.getTelemetry().isSelfStable);
    EXPECT_FALSE(bike.getTelemetry().isCapsizeMode);

    // Above capsize speed (v = 11.0 m/s)
    body.setLinearVelocity(math::Vec2{ 11.0, 0.0 });
    bike.step(body, 0.016);
    EXPECT_FALSE(bike.getTelemetry().isSelfStable);
    EXPECT_TRUE(bike.getTelemetry().isCapsizeMode);
}

TEST(BicycleDynamicsTest, CamberThrustContribution) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 6.0, 0.0 });

    // Rider leans body to right
    BicycleInput input{};
    input.riderLeanRoll = 1.0;
    bike.setInput(input);

    for (int i = 0; i < 15; ++i) {
        bike.step(body, 0.016);
    }

    const auto& telem = bike.getTelemetry();
    EXPECT_GT(telem.rollAngle, 0.0);
    EXPECT_GT(telem.frontCamberThrust, 0.0);
    EXPECT_GT(telem.rearCamberThrust, 0.0);
}

TEST(BicycleDynamicsTest, BrakingPitchLoadTransfer) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 8.0, 0.0 });

    // Initial steady state step
    bike.step(body, 0.016);
    const double initialRearLoad = bike.getTelemetry().rearNormalLoad;

    // Apply hard front brake
    BicycleInput input{};
    input.frontBrake = 1.0;
    bike.setInput(input);

    bike.step(body, 0.016);
    const auto& telem = bike.getTelemetry();

    EXPECT_GT(telem.frontNormalLoad, telem.rearNormalLoad);
    EXPECT_LT(telem.rearNormalLoad, initialRearLoad);
}

TEST(BicycleDynamicsTest, ResetClearsTransientState) {
    BicycleDynamics bike{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 85.0;
    physics::RigidBody2D body{ bodyConfig };

    BicycleInput input{};
    input.pedalingThrottle = 1.0;
    input.steerTorque = 4.0;
    bike.setInput(input);

    bike.step(body, 0.016);
    bike.reset();

    const auto& telem = bike.getTelemetry();
    EXPECT_DOUBLE_EQ(telem.forwardSpeed, 0.0);
    EXPECT_DOUBLE_EQ(telem.steerAngle, 0.0);
    EXPECT_DOUBLE_EQ(bike.getInput().pedalingThrottle, 0.0);
}

} // namespace
} // namespace vehicle_sim::dynamics
