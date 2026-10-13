#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/GoKartDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

    TEST(GoKartDynamicsTest, DefaultConstructionAndGeometry)
    {
        GoKartDynamics kart {};
        EXPECT_FALSE(kart.isAirborne());
        EXPECT_FALSE(kart.isInnerWheelLifted());
        EXPECT_NEAR(kart.getAxleScrubTorque(), 0.0, 1e-3);
        EXPECT_NEAR(kart.getJackingForce(), 0.0, 1e-3);

        const auto& wheels { kart.getWheelStates() };
        EXPECT_GT(wheels[0].localOffset.x, 0.0); // Front axle
        EXPECT_GT(wheels[0].localOffset.y, 0.0); // Left side
        EXPECT_LT(wheels[2].localOffset.x, 0.0); // Rear axle
        EXPECT_LT(wheels[3].localOffset.y, 0.0); // Right side
    }

    TEST(GoKartDynamicsTest, Direct1To1SteeringResponse)
    {
        GoKartDynamics kart {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 165.0;
        physics::RigidBody2D body { bodyConfig };

        kart.setSteer(0.75);
        kart.step(body, 0.016);

        const auto& wheels { kart.getWheelStates() };
        const double expectedSteer { 0.75 * kart.getConfig().maxSteerAngle };
        EXPECT_NEAR(wheels[0].steerAngle, expectedSteer, 1e-4);
        EXPECT_NEAR(wheels[1].steerAngle, expectedSteer, 1e-4);
        EXPECT_NEAR(wheels[2].steerAngle, 0.0, 1e-4);
        EXPECT_NEAR(wheels[3].steerAngle, 0.0, 1e-4);
    }

    TEST(GoKartDynamicsTest, SolidAxleScrubTorqueInGentleTurn)
    {
        GoKartDynamics kart {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 165.0;
        physics::RigidBody2D body { bodyConfig };

        // Mild turn at moderate speed where inside rear tire has NOT lifted yet
        body.setLinearVelocity(math::Vec2 { 10.0, 0.0 });
        body.setAngularVelocity(0.15);
        kart.setSteer(0.05); // Tiny steer demand
        kart.step(body, 0.016);

        // Both rear wheels remain on track
        EXPECT_FALSE(kart.isInnerWheelLifted());
        const auto& wheels { kart.getWheelStates() };
        EXPECT_GT(wheels[2].normalLoad, 0.0);
        EXPECT_GT(wheels[3].normalLoad, 0.0);

        // Solid axle scrub induces opposing longitudinal forces
        EXPECT_NE(kart.getAxleScrubTorque(), 0.0);
    }

    TEST(GoKartDynamicsTest, CasterJackingLiftsInsideRearWheel)
    {
        GoKartDynamics kart {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 165.0;
        physics::RigidBody2D body { bodyConfig };

        // Fast sharp left turn
        body.setLinearVelocity(math::Vec2 { 18.0, 0.0 });
        body.setAngularVelocity(1.2);
        kart.setSteer(0.9); // Sharp left steer command

        kart.step(body, 0.016);

        // Jacking force unweights inside rear wheel (RL: index 2)
        EXPECT_GT(kart.getJackingForce(), 100.0);
        EXPECT_TRUE(kart.isInnerWheelLifted());

        const auto& wheels { kart.getWheelStates() };
        EXPECT_TRUE(wheels[2].isLifted);
        EXPECT_NEAR(wheels[2].normalLoad, 0.0, 1e-3);
        EXPECT_GT(wheels[3].normalLoad, 0.0); // Outside wheel loaded
    }

    TEST(GoKartDynamicsTest, TripodCorneringEliminatesAxleScrubTorque)
    {
        GoKartDynamics kart {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 165.0;
        physics::RigidBody2D body { bodyConfig };

        // Force inside wheel lift
        body.setLinearVelocity(math::Vec2 { 16.0, 0.0 });
        body.setAngularVelocity(1.0);
        kart.setSteer(0.85);

        kart.step(body, 0.016);

        EXPECT_TRUE(kart.isInnerWheelLifted());
        // In tripod cornering mode, axle binding scrub drops to 0
        EXPECT_NEAR(kart.getAxleScrubTorque(), 0.0, 1e-3);
    }

    TEST(GoKartDynamicsTest, AirborneDisengagesWheelForces)
    {
        GoKartDynamics kart {};
        physics::RigidBodyConfig bodyConfig {};
        physics::RigidBody2D body { bodyConfig };

        kart.setAirborne(true);
        kart.step(body, 0.016);

        EXPECT_TRUE(kart.isAirborne());
        const auto& wheels { kart.getWheelStates() };
        for (const auto& wheel : wheels) {
            EXPECT_NEAR(wheel.normalLoad, 0.0, 1e-4);
            EXPECT_NEAR(wheel.tractiveForce, 0.0, 1e-4);
            EXPECT_NEAR(wheel.corneringForce, 0.0, 1e-4);
        }
        EXPECT_NEAR(kart.getAxleScrubTorque(), 0.0, 1e-4);
    }

} // namespace
} // namespace vehicle_sim::dynamics
