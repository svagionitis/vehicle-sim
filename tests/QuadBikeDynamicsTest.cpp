#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/QuadBikeDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

TEST(QuadBikeDynamicsTest, DefaultConstructionAndGeometry) {
    const QuadBikeDynamics quad{};
    EXPECT_FALSE(quad.isAirborne());

    const auto& config = quad.getConfig();
    EXPECT_GT(config.chassisMass, 0.0);
    EXPECT_GT(config.wheelbase, 0.0);
    EXPECT_GT(config.trackWidthFront, 0.0);
    EXPECT_GT(config.trackWidthRear, 0.0);
}

TEST(QuadBikeDynamicsTest, BalloonTireComplianceAndExpansion) {
    QuadBikeDynamics quad{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    quad.step(body, 0.016);

    const auto& telem = quad.getTelemetry();
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_GT(telem.wheels[i].normalLoad, 0.0);
        EXPECT_GT(telem.wheels[i].carcassDeflection, 0.0);
        EXPECT_GT(telem.wheels[i].contactArea, quad.getConfig().nominalPatchArea);
    }
}

TEST(QuadBikeDynamicsTest, SpoolAxleBindingScrub) {
    QuadBikeDynamics quad{};
    auto config = quad.getConfig();
    config.rearDiffMode = QuadDiffMode::LockedSpool;
    quad.setConfig(config);

    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 10.0, 0.0 });
    body.setAngularVelocity(0.40); // Turning in corner

    quad.step(body, 0.016);

    const auto& telem = quad.getTelemetry();
    EXPECT_NE(telem.rearAxleBindingTorque, 0.0);
}

TEST(QuadBikeDynamicsTest, OpenDifferentialNoBinding) {
    QuadBikeDynamics quad{};
    auto config = quad.getConfig();
    config.rearDiffMode = QuadDiffMode::OpenDifferential;
    quad.setConfig(config);

    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 10.0, 0.0 });
    body.setAngularVelocity(0.40); // Turning in corner

    quad.step(body, 0.016);

    const auto& telem = quad.getTelemetry();
    EXPECT_DOUBLE_EQ(telem.rearAxleBindingTorque, 0.0);
}

TEST(QuadBikeDynamicsTest, ActiveRiderPitchShift) {
    QuadBikeDynamics quad{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    // Rider leans forward aggressively (+1.0)
    QuadBikeInput input{};
    input.riderLeanPitch = 1.0;
    quad.setInput(input);

    quad.step(body, 0.016);

    const auto& telem = quad.getTelemetry();
    EXPECT_GT(telem.compositeCogOffset.x, 0.0); // COG moved forward
    // Front normal loads should be higher than static
    EXPECT_GT(telem.wheels[0].normalLoad + telem.wheels[1].normalLoad,
              telem.wheels[2].normalLoad + telem.wheels[3].normalLoad);
}

TEST(QuadBikeDynamicsTest, LateralRolloverMarginAndTwoWheelRiding) {
    QuadBikeDynamics quad{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    body.setLinearVelocity(math::Vec2{ 16.0, 0.0 });
    body.setAngularVelocity(2.2); // Violent cornering

    QuadBikeInput input{};
    input.steer = 1.0;
    input.riderLeanRoll = -1.0; // Rider leaning wrong way (outward)
    quad.setInput(input);

    quad.step(body, 0.016);

    const auto& telem = quad.getTelemetry();
    EXPECT_GT(telem.lateralRolloverIndex, 0.5);
    EXPECT_TRUE(telem.isTwoWheelSideRiding);
}

TEST(QuadBikeDynamicsTest, ResetClearsTransientState) {
    QuadBikeDynamics quad{};
    physics::RigidBodyConfig bodyConfig{};
    bodyConfig.mass = 288.0;
    physics::RigidBody2D body{ bodyConfig };

    QuadBikeInput input{};
    input.throttle = 1.0;
    input.steer = 0.5;
    quad.setInput(input);

    quad.step(body, 0.016);
    quad.reset();

    const auto& telem = quad.getTelemetry();
    EXPECT_DOUBLE_EQ(telem.forwardSpeed, 0.0);
    EXPECT_DOUBLE_EQ(quad.getInput().throttle, 0.0);
}

} // namespace
} // namespace vehicle_sim::dynamics
