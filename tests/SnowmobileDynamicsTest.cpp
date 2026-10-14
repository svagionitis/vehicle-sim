#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/SnowmobileDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

TEST(SnowmobileDynamicsTest, DefaultConstructionAndGeometry) {
    const SnowmobileDynamics snowmobile{};
    EXPECT_FALSE(snowmobile.isAirborne());

    const auto& config = snowmobile.getConfig();
    EXPECT_GT(config.chassisMass, 0.0);
    EXPECT_GT(config.distFrontSkis, 0.0);
    EXPECT_GT(config.distRearTrack, 0.0);
    EXPECT_GT(config.skiStanceWidth, 0.0);
}

TEST(SnowmobileDynamicsTest, StraightLineAcceleration) {
    SnowmobileDynamics snowmobile{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    SnowmobileInput input{};
    input.throttle = 1.0;
    snowmobile.setInput(input);

    snowmobile.step(body, 0.016);

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_GT(telem.trackTractiveForce, 0.0);
    EXPECT_GT(body.getAccumulatedForce().x, 0.0);
}

TEST(SnowmobileDynamicsTest, SnowTrenchingDrag) {
    SnowmobileDynamics snowmobile{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    SnowmobileInput input{};
    input.throttle = 1.0;
    snowmobile.setInput(input);

    body.setLinearVelocity(math::Vec2{ 15.0, 0.0 });
    snowmobile.step(body, 0.016);

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_GT(telem.snowSinkageDepth, 0.0);
    EXPECT_GT(telem.trackTrenchingDrag, 0.0);
}

TEST(SnowmobileDynamicsTest, SkiCarvingOnRoll) {
    SnowmobileDynamics snowmobile{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 12.0, 0.0 });

    SnowmobileInput input{};
    input.steer = 0.5;
    input.riderLean = 0.8;
    snowmobile.setInput(input);

    for (int i = 0; i < 10; ++i) {
        snowmobile.step(body, 0.016);
    }

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_NE(telem.leftSkiCarveForce + telem.rightSkiCarveForce, 0.0);
    EXPECT_NE(telem.rollAngle, 0.0);
}

TEST(SnowmobileDynamicsTest, InsideSkiLift) {
    SnowmobileDynamics snowmobile{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 18.0, 0.0 });
    body.setAngularVelocity(1.8);

    SnowmobileInput input{};
    input.steer = 1.0;
    input.riderLean = -1.0; // Leaning outward, inducing inside ski lift
    snowmobile.setInput(input);

    snowmobile.step(body, 0.016);

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_TRUE(telem.isInsideSkiLifted);
}

TEST(SnowmobileDynamicsTest, PowerWheelie) {
    SnowmobileDynamics snowmobile{};
    auto config = snowmobile.getConfig();
    config.maxDriveForce = 12000.0; // High tractive torque
    snowmobile.setConfig(config);

    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    SnowmobileInput input{};
    input.throttle = 1.0;
    snowmobile.setInput(input);

    snowmobile.step(body, 0.016);

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_TRUE(telem.isPowerWheelie);
    EXPECT_DOUBLE_EQ(telem.leftSkiLoad, 0.0);
    EXPECT_DOUBLE_EQ(telem.rightSkiLoad, 0.0);
}

TEST(SnowmobileDynamicsTest, ResetClearsTransientState) {
    SnowmobileDynamics snowmobile{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 310.0;
    physics::RigidBody2D body{ bodyConfig };

    SnowmobileInput input{};
    input.throttle = 0.8;
    input.steer = 0.5;
    snowmobile.setInput(input);

    snowmobile.step(body, 0.016);
    snowmobile.reset();

    const auto& telem = snowmobile.getTelemetry();
    EXPECT_DOUBLE_EQ(telem.forwardSpeed, 0.0);
    EXPECT_DOUBLE_EQ(telem.rollAngle, 0.0);
    EXPECT_DOUBLE_EQ(snowmobile.getInput().throttle, 0.0);
}

} // namespace
} // namespace vehicle_sim::dynamics
