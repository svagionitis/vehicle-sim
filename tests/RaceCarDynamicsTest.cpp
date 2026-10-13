#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/RaceCarDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

    TEST(RaceCarDynamicsTest, DefaultConstructionAndWheelInitialization)
    {
        RaceCarDynamics car {};
        EXPECT_FALSE(car.isAirborne());
        EXPECT_NEAR(car.getDownforce(), 0.0, 1e-3);
        EXPECT_NEAR(car.getDrsAngleDeg(), 0.0, 1e-3);

        const auto& wheels { car.getWheelStates() };
        EXPECT_GT(wheels[0].localOffset.x, 0.0); // Front axle
        EXPECT_GT(wheels[0].localOffset.y, 0.0); // Left side
        EXPECT_LT(wheels[2].localOffset.x, 0.0); // Rear axle
        EXPECT_LT(wheels[3].localOffset.y, 0.0); // Right side
    }

    TEST(RaceCarDynamicsTest, LongitudinalLoadTransferUnderAcceleration)
    {
        RaceCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 750.0;
        bodyConfig.inertia = 900.0;
        physics::RigidBody2D body { bodyConfig };

        // Initially static at rest
        car.step(body, 0.016);
        const auto initialWheels { car.getWheelStates() };
        const double initialFront { initialWheels[0].normalLoad + initialWheels[1].normalLoad };
        const double initialRear { initialWheels[2].normalLoad + initialWheels[3].normalLoad };
        EXPECT_GT(initialFront, 0.0);
        EXPECT_GT(initialRear, 0.0);

        // Apply throttle and accelerate forward
        car.setThrottle(1.0);
        body.setLinearVelocity(math::Vec2 { 10.0, 0.0 });
        car.step(body, 0.016);

        const auto accelWheels { car.getWheelStates() };
        const double accelFront { accelWheels[0].normalLoad + accelWheels[1].normalLoad };
        const double accelRear { accelWheels[2].normalLoad + accelWheels[3].normalLoad };

        // Under forward acceleration, weight shifts to rear
        EXPECT_GT(accelRear, initialRear);
        EXPECT_LT(accelFront, initialFront);
    }

    TEST(RaceCarDynamicsTest, LateralLoadTransferUnderCornering)
    {
        RaceCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 750.0;
        physics::RigidBody2D body { bodyConfig };

        // Moving forward and steering left (positive yaw rate)
        body.setLinearVelocity(math::Vec2 { 25.0, 0.0 });
        body.setAngularVelocity(0.5); // Turning left
        car.setSteer(0.5);

        car.step(body, 0.016);
        const auto wheels { car.getWheelStates() };

        // In a left turn, weight transfers to outside (Right) wheels (FR: 1, RR: 3)
        const double leftLoads { wheels[0].normalLoad + wheels[2].normalLoad };
        const double rightLoads { wheels[1].normalLoad + wheels[3].normalLoad };
        EXPECT_GT(rightLoads, leftLoads);
    }

    TEST(RaceCarDynamicsTest, AerodynamicDownforceScalingWithSpeed)
    {
        RaceCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 750.0;
        physics::RigidBody2D body { bodyConfig };

        body.setLinearVelocity(math::Vec2 { 10.0, 0.0 });
        car.step(body, 0.016);
        const double lowSpeedDownforce { car.getDownforce() };

        body.setLinearVelocity(math::Vec2 { 50.0, 0.0 }); // High speed (~180 km/h)
        car.step(body, 0.016);
        const double highSpeedDownforce { car.getDownforce() };

        EXPECT_GT(highSpeedDownforce, lowSpeedDownforce * 10.0);
    }

    TEST(RaceCarDynamicsTest, DrsFlapDeployment)
    {
        RaceCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 750.0;
        physics::RigidBody2D body { bodyConfig };

        body.setLinearVelocity(math::Vec2 { 60.0, 0.0 });
        car.setDrsActive(true);

        for (int i { 0 }; i < 20; ++i) {
            car.step(body, 0.016);
        }

        EXPECT_GT(car.getDrsAngleDeg(), 5.0);
    }

    TEST(RaceCarDynamicsTest, AirborneDisengagesWheelLoads)
    {
        RaceCarDynamics car {};
        physics::RigidBodyConfig bodyConfig {};
        physics::RigidBody2D body { bodyConfig };

        car.setAirborne(true);
        car.step(body, 0.016);

        EXPECT_TRUE(car.isAirborne());
        const auto wheels { car.getWheelStates() };
        for (const auto& wheel : wheels) {
            EXPECT_NEAR(wheel.normalLoad, 0.0, 1e-4);
            EXPECT_NEAR(wheel.tractiveForce, 0.0, 1e-4);
        }
    }

} // namespace
} // namespace vehicle_sim::dynamics
