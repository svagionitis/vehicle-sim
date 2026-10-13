#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/DriftCarDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

    TEST(DriftCarDynamicsTest, DefaultConstructionAndWheelInitialization)
    {
        DriftCarDynamics car {};
        EXPECT_FALSE(car.isAirborne());
        EXPECT_FALSE(car.isDrifting());
        EXPECT_NEAR(car.getDriftAngle(), 0.0, 1e-3);
        EXPECT_NEAR(car.getAligningTorque(), 0.0, 1e-3);

        const auto& wheels { car.getWheelStates() };
        EXPECT_GT(wheels[0].localOffset.x, 0.0); // Front axle
        EXPECT_GT(wheels[0].localOffset.y, 0.0); // Left side
        EXPECT_LT(wheels[2].localOffset.x, 0.0); // Rear axle
        EXPECT_LT(wheels[3].localOffset.y, 0.0); // Right side

        const auto temps { car.getTireTemperatures() };
        for (double t : temps) {
            EXPECT_NEAR(t, 25.0, 1e-1);
        }
    }

    TEST(DriftCarDynamicsTest, HandbrakeLocksRearAxleOnly)
    {
        DriftCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1250.0;
        physics::RigidBody2D body { bodyConfig };

        body.setLinearVelocity(math::Vec2 { 18.0, 0.0 });
        car.setHandbrake(1.0); // Full hydraulic handbrake
        car.step(body, 0.016);

        const auto& wheels { car.getWheelStates() };
        // Front wheels free rolling
        EXPECT_NEAR(wheels[0].slipRatio, 0.0, 0.05);
        EXPECT_NEAR(wheels[1].slipRatio, 0.0, 0.05);

        // Rear wheels locked into extreme negative slip ratio
        EXPECT_LT(wheels[2].slipRatio, -0.80);
        EXPECT_LT(wheels[3].slipRatio, -0.80);
        EXPECT_LT(wheels[2].tractiveForce, 0.0);
        EXPECT_LT(wheels[3].tractiveForce, 0.0);
    }

    TEST(DriftCarDynamicsTest, ClutchKickTorqueSurge)
    {
        DriftCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1250.0;
        physics::RigidBody2D body { bodyConfig };

        body.setLinearVelocity(math::Vec2 { 12.0, 0.0 });
        car.setThrottle(0.8);
        car.triggerClutchKick();

        car.step(body, 0.016);
        const auto& wheels { car.getWheelStates() };

        // Rear slip ratio surges positive breaking static traction
        EXPECT_GT(wheels[2].slipRatio, 0.50);
        EXPECT_GT(wheels[3].slipRatio, 0.50);
    }

    TEST(DriftCarDynamicsTest, SustainedDriftDetectionAndAligningTorque)
    {
        DriftCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1250.0;
        physics::RigidBody2D body { bodyConfig };

        // Moving forward with high lateral slide velocity (large body slip angle)
        body.setLinearVelocity(math::Vec2 { 15.0, 6.0 }); // beta ~ 0.38 rad (~22 deg)
        body.setAngularVelocity(-0.35); // Counter-steering slide
        car.setThrottle(0.9);
        car.setSteer(-0.6); // Counter-steer into the slide

        car.step(body, 0.016);

        EXPECT_GT(std::abs(car.getDriftAngle()), 0.20);
        EXPECT_TRUE(car.isDrifting());

        // Front caster & pneumatic trail produces aligning torque tending to counter-steer
        EXPECT_NE(car.getAligningTorque(), 0.0);
    }

    TEST(DriftCarDynamicsTest, ThermalGripDegradationUnderProlongedSlide)
    {
        DriftCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1250.0;
        physics::RigidBody2D body { bodyConfig };

        body.setLinearVelocity(math::Vec2 { 15.0, 5.0 });
        car.setThrottle(1.0);
        car.setHandbrake(0.5);

        // Simulate 200 simulation steps of heavy wheelspin and sliding
        for (int i { 0 }; i < 200; ++i) {
            car.step(body, 0.016);
        }

        const auto& wheels { car.getWheelStates() };
        // Rear tire temperatures rise substantially
        EXPECT_GT(wheels[2].temperature, 35.0);
        EXPECT_GT(wheels[3].temperature, 35.0);

        // Effective friction degrades as temperature rises
        if (wheels[2].temperature > 85.0) {
            EXPECT_LT(wheels[2].effectiveMu, 1.0);
        }
    }

    TEST(DriftCarDynamicsTest, AirborneDisengagesWheelForces)
    {
        DriftCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        physics::RigidBody2D body { bodyConfig };

        car.setAirborne(true);
        car.step(body, 0.016);

        EXPECT_TRUE(car.isAirborne());
        const auto& wheels { car.getWheelStates() };
        for (const auto& wheel : wheels) {
            EXPECT_NEAR(wheel.normalLoad, 0.0, 1e-4);
            EXPECT_NEAR(wheel.tractiveForce, 0.0, 1e-4);
            EXPECT_NEAR(wheel.corneringForce, 0.0, 1e-4);
        }
        EXPECT_NEAR(car.getAligningTorque(), 0.0, 1e-4);
    }

} // namespace
} // namespace vehicle_sim::dynamics
