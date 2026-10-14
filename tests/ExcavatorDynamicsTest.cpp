#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/dynamics/ExcavatorDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::math;
using namespace vehicle_sim::physics;

TEST(ExcavatorDynamicsTest, DefaultConstructionAndSanity)
{
    ExcavatorDynamics excavator;
    EXPECT_FALSE(excavator.isAirborne());
    EXPECT_FALSE(excavator.isTippedOver());
    EXPECT_NEAR(excavator.getSlewAngle(), 0.0, 1.0e-4);
    EXPECT_GT(excavator.getTcpPosition().x, 0.0);
}

TEST(ExcavatorDynamicsTest, SkidSteerNeutralTurn)
{
    ExcavatorDynamics excavator;
    RigidBody2D body;

    ExcavatorInput input;
    input.leftTrack = -1.0;
    input.rightTrack = 1.0;
    excavator.setInput(input);

    for (int i = 0; i < 20; ++i) {
        excavator.step(body, 0.05);
    }

    const auto& telemetry = excavator.getTelemetry();
    // Neutral turn: forward speed ~ 0, yaw rate > 0
    EXPECT_NEAR(telemetry.forwardSpeed, 0.0, 1.0e-3);
    EXPECT_GT(telemetry.yawRate, 0.0);
}

TEST(ExcavatorDynamicsTest, SlewRingContinuousRotation)
{
    ExcavatorDynamics excavator;
    RigidBody2D body;

    ExcavatorInput input;
    input.slewCommand = 1.0;
    excavator.setInput(input);

    for (int i = 0; i < 50; ++i) {
        excavator.step(body, 0.1);
    }

    EXPECT_GT(std::abs(excavator.getSlewAngle()), 0.0);
    EXPECT_GT(excavator.getTelemetry().slewSpeed, 0.0);

    // Test engaging slew brake
    input.slewCommand = 0.0;
    input.slewBrakeActive = true;
    excavator.setInput(input);

    for (int i = 0; i < 20; ++i) {
        excavator.step(body, 0.05);
    }

    EXPECT_NEAR(excavator.getTelemetry().slewSpeed, 0.0, 0.05);
}

TEST(ExcavatorDynamicsTest, ArmForwardKinematics)
{
    ExcavatorDynamics excavator;
    const auto& config = excavator.getConfig();

    const double boom = 0.5;
    const double stick = 1.0;
    const double bucket = 0.5;
    excavator.setArmJointAngles(boom, stick, bucket);

    const auto tcp = excavator.getTcpPosition();
    const double expectedReach = config.arm.boomLength * std::cos(0.5) + config.arm.stickLength * std::cos(1.5)
        + config.arm.bucketRadius * std::cos(2.0);
    const double expectedHeight = config.arm.boomLength * std::sin(0.5) + config.arm.stickLength * std::sin(1.5)
        + config.arm.bucketRadius * std::sin(2.0);
    EXPECT_NEAR(tcp.x, expectedReach, 1.0e-3);
    EXPECT_NEAR(tcp.y, expectedHeight, 1.0e-3);
}

TEST(ExcavatorDynamicsTest, JacobianVelocityMapping)
{
    ExcavatorDynamics excavator;
    RigidBody2D body;

    excavator.setArmJointAngles(0.3, 1.0, 0.5);

    ExcavatorInput input;
    input.boomCommand = 1.0;
    excavator.setInput(input);

    excavator.step(body, 0.01);
    const auto& telemetry = excavator.getTelemetry();

    // When boom elevates (+angular rate), TCP position should have non-zero velocity
    EXPECT_GT(telemetry.tcpVelocity.length(), 0.0);
}

TEST(ExcavatorDynamicsTest, DiggingForceCylinderStall)
{
    ExcavatorDynamics excavator;
    RigidBody2D body;

    // Apply high opposing digging force exceeding breakout capacity
    excavator.setDigForce(Vec2 { -500000.0, -500000.0 });

    ExcavatorInput input;
    input.bucketCommand = 1.0;
    excavator.setInput(input);

    excavator.step(body, 0.05);

    EXPECT_TRUE(excavator.getTelemetry().isArmStalled);
}

TEST(ExcavatorDynamicsTest, CrawlerTippingStability)
{
    ExcavatorDynamics excavator;
    RigidBody2D body;

    // Normal pose: fully stable
    excavator.step(body, 0.05);
    EXPECT_FALSE(excavator.isTippedOver());
    EXPECT_GT(excavator.getTelemetry().tippingMargin, 0.0);

    // Extreme case: fully extended arm, max payload, plus strong downward digging force
    excavator.setArmJointAngles(0.0, 0.52, 0.0);
    excavator.setBucketPayload(3000.0);
    excavator.setDigForce(Vec2 { 0.0, -800000.0 }); // Strong downward pull at bucket

    excavator.step(body, 0.05);
    EXPECT_TRUE(excavator.isTippedOver());
    EXPECT_NEAR(excavator.getTelemetry().tippingMargin, 0.0, 1.0e-3);
}
