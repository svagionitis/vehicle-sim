#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/QuadcopterDroneDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

    TEST(QuadcopterDroneDynamicsTest, DisarmedProducesZeroThrust)
    {
        QuadcopterDroneDynamics drone {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1.5;
        physics::RigidBody2D body { bodyConfig };

        drone.setArmed(false);
        drone.setThrottle(1.0);
        drone.step(body, 0.016);

        EXPECT_FALSE(drone.isAirborne());
        EXPECT_NEAR(drone.getTotalThrust(), 0.0, 1e-4);
        for (double speed : drone.getMotorSpeeds()) {
            EXPECT_NEAR(speed, 0.0, 1e-4);
        }
    }

    TEST(QuadcopterDroneDynamicsTest, ArmedThrottleGeneratesThrust)
    {
        QuadcopterDroneDynamics drone {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1.5;
        physics::RigidBody2D body { bodyConfig };

        drone.setArmed(true);
        drone.setThrottle(0.6);

        // Run multiple ticks to allow ESC spool up
        for (int i { 0 }; i < 30; ++i) {
            drone.step(body, 0.016);
        }

        EXPECT_GT(drone.getTotalThrust(), 10.0);
        const auto speeds { drone.getMotorSpeeds() };
        for (double speed : speeds) {
            EXPECT_GT(speed, 500.0);
        }
    }

    TEST(QuadcopterDroneDynamicsTest, SymmetricHoverCancelsYawTorque)
    {
        QuadcopterDroneDynamics drone {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1.5;
        physics::RigidBody2D body { bodyConfig };

        drone.setArmed(true);
        drone.setThrottle(0.5);
        drone.setPitchAngle(0.0);

        for (int i { 0 }; i < 30; ++i) {
            drone.step(body, 0.016);
        }

        // In symmetric hover, reactionary drag torques of CW and CCW pairs cancel exactly
        EXPECT_NEAR(drone.getYawTorque(), 0.0, 1e-4);
    }

    TEST(QuadcopterDroneDynamicsTest, PitchTiltCommandsDifferentialThrust)
    {
        QuadcopterDroneDynamics drone {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1.5;
        physics::RigidBody2D body { bodyConfig };

        drone.setArmed(true);
        drone.setThrottle(0.5);
        drone.setPitchAngle(0.2); // Command pitch up

        for (int i { 0 }; i < 20; ++i) {
            body.clearForces();
            drone.step(body, 0.016);
        }

        const auto speeds { drone.getMotorSpeeds() };
        // Motors 0 & 1 are front, motors 2 & 3 are rear
        // To pitch up, rear motors must spin faster than front motors
        EXPECT_GT(speeds[2], speeds[0]);
        EXPECT_GT(speeds[3], speeds[1]);
        EXPECT_GT(body.getAccumulatedTorque(), 0.0);
    }

    TEST(QuadcopterDroneDynamicsTest, MotorSpoolLag)
    {
        QuadcopterDroneDynamics drone {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 1.5;
        physics::RigidBody2D body { bodyConfig };

        drone.setArmed(true);
        drone.setThrottle(1.0);

        // Step only one single small timestep
        drone.step(body, 0.005);
        const double speedTick1 { drone.getMotorSpeeds()[0] };

        // Step several more timesteps
        for (int i { 0 }; i < 15; ++i) {
            drone.step(body, 0.016);
        }
        const double speedLater { drone.getMotorSpeeds()[0] };

        EXPECT_GT(speedLater, speedTick1 * 2.0);
    }

} // namespace
} // namespace vehicle_sim::dynamics
