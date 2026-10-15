#include <gtest/gtest.h>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::physics;

TEST(MotionLabCarDynamicsTest, DefaultConstructionAndGeometry)
{
    MotionLabCarDynamics car { };
    const auto& config = car.getConfig();
    EXPECT_DOUBLE_EQ(config.mass, 1000.0);
    EXPECT_DOUBLE_EQ(config.length, 4.0);
    EXPECT_DOUBLE_EQ(config.width, 2.0);
    EXPECT_DOUBLE_EQ(config.engineForce, 9750.0);
    EXPECT_DOUBLE_EQ(config.rollingResistance, 32.5);
    EXPECT_DOUBLE_EQ(config.lateralResistance, 1100.0);

    const auto& tires = car.getTires();
    EXPECT_TRUE(tires[MotionLabCarDynamics::kFrontLeft].steered);
    EXPECT_TRUE(tires[MotionLabCarDynamics::kFrontRight].steered);
    EXPECT_FALSE(tires[MotionLabCarDynamics::kRearLeft].steered);
    EXPECT_FALSE(tires[MotionLabCarDynamics::kRearRight].steered);

    EXPECT_DOUBLE_EQ(tires[MotionLabCarDynamics::kFrontLeft].position.x, 1.35);
    EXPECT_DOUBLE_EQ(tires[MotionLabCarDynamics::kFrontLeft].position.y, 0.70);
    EXPECT_DOUBLE_EQ(tires[MotionLabCarDynamics::kRearRight].position.x, -1.35);
    EXPECT_DOUBLE_EQ(tires[MotionLabCarDynamics::kRearRight].position.y, -0.70);

    EXPECT_FALSE(car.isAirborne());
    EXPECT_FALSE(car.isSkidding());
    EXPECT_DOUBLE_EQ(car.getRearGripLoss(), 0.0);
}

TEST(MotionLabCarDynamicsTest, StraightLineAccelerationReachesTerminalSpeed)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };
    car.setInput({ 1.0, 0.0, false });

    // Step for 50 seconds at 120 Hz (6000 steps > 6.5 time constants where tau = m / c = 1000 / 130 = 7.69s)
    constexpr double dt { 1.0 / 120.0 };
    for (int i { 0 }; i < 6000; ++i) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
    }

    // Terminal velocity equilibrium: F_engine = 4 * c_roll * v_term
    // v_term = 9750 / (4 * 32.5) = 9750 / 130 = 75.0 m/s
    EXPECT_NEAR(body.getLinearVelocity().x, 75.0, 0.5);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-4);
    EXPECT_NEAR(body.getAngularVelocity(), 0.0, 1e-4);
    EXPECT_NEAR(body.getOrientation(), 0.0, 1e-4);
}

TEST(MotionLabCarDynamicsTest, ReverseThrottleAttenuatedAfterStop)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };
    // Reverse from standstill
    car.setInput({ -1.0, 0.0, false });

    constexpr double dt { 1.0 / 120.0 };
    for (int i { 0 }; i < 6000; ++i) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
    }

    // Reverse terminal velocity with 0.75 factor:
    // v_term_rev = - (9750 * 0.75) / 130 = -56.25 m/s
    EXPECT_NEAR(body.getLinearVelocity().x, -56.25, 0.5);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-4);
}

TEST(MotionLabCarDynamicsTest, SteeringGeneratesYawTorqueAndTurning)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity({ 15.0, 0.0 }); // Forward moving

    MotionLabCarDynamics car { };
    // Steer left while maintaining throttle
    car.setInput({ 1.0, 1.0, false });

    constexpr double dt { 1.0 / 120.0 };
    for (int i { 0 }; i < 60; ++i) { // 0.5s of turning
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
    }

    // Car should have yawed counter-clockwise (positive angular velocity & orientation)
    EXPECT_GT(body.getAngularVelocity(), 0.0);
    EXPECT_GT(body.getOrientation(), 0.0);
    EXPECT_GT(body.getLinearVelocity().y, 0.0);
}

TEST(MotionLabCarDynamicsTest, HandbrakeInducesRearGripDecayAndLinearRecovery)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };

    // Handbrake engaged -> instant 1.0 grip loss
    car.setInput({ 1.0, 0.0, true });
    car.step(body, 1.0 / 120.0);
    EXPECT_DOUBLE_EQ(car.getRearGripLoss(), 1.0);

    // Handbrake released -> recovers linearly over 0.15s (18 steps at 120Hz)
    car.setInput({ 1.0, 0.0, false });
    for (int i { 0 }; i < 9; ++i) {
        car.step(body, 1.0 / 120.0);
    }
    // Halfway through recovery -> ~0.50
    EXPECT_NEAR(car.getRearGripLoss(), 0.5, 0.05);

    for (int i { 0 }; i < 10; ++i) {
        car.step(body, 1.0 / 120.0);
    }
    // Fully recovered -> 0.0
    EXPECT_DOUBLE_EQ(car.getRearGripLoss(), 0.0);
}

TEST(MotionLabCarDynamicsTest, SkidDetectionHysteresis)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };
    car.setInput({ 0.0, 0.0, false });

    // Give chassis pure lateral slide: v_lat = 2.5 m/s -> 2.5 * 0.7 = 1.75 > 1.5 start threshold
    body.setLinearVelocity({ 0.0, 2.5 });
    car.step(body, 1.0 / 120.0);

    EXPECT_TRUE(car.isSkidding());
    const auto& tires = car.getTires();
    EXPECT_TRUE(tires[MotionLabCarDynamics::kRearLeft].skidding);
    EXPECT_TRUE(tires[MotionLabCarDynamics::kRearRight].skidding);
    // Front tires should never mark skid
    EXPECT_FALSE(tires[MotionLabCarDynamics::kFrontLeft].skidding);
    EXPECT_FALSE(tires[MotionLabCarDynamics::kFrontRight].skidding);
}

TEST(MotionLabCarDynamicsTest, ResetClearsTransientState)
{
    RigidBodyConfig bodyConfig { };
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };
    car.setInput({ 1.0, 0.0, true });
    car.step(body, 1.0 / 120.0);
    EXPECT_DOUBLE_EQ(car.getRearGripLoss(), 1.0);

    car.reset();
    EXPECT_DOUBLE_EQ(car.getRearGripLoss(), 0.0);
    EXPECT_FALSE(car.isSkidding());
}

TEST(MotionLabCarDynamicsTest, WorldPointsAndMidpointSupport)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.position = math::Vec2 { 10.0, 20.0 };
    bodyConfig.orientation = 0.0;
    RigidBody2D body { bodyConfig };

    MotionLabCarDynamics car { };
    const auto corners = car.computeWorldPoints(body);

    // Box 4.0m x 2.0m centered at (10, 20):
    // corners: (8, 19), (12, 19), (12, 21), (8, 21)
    EXPECT_NEAR(corners[0].x, 8.0, 1e-4);
    EXPECT_NEAR(corners[0].y, 19.0, 1e-4);
    EXPECT_NEAR(corners[1].x, 12.0, 1e-4);
    EXPECT_NEAR(corners[1].y, 19.0, 1e-4);
    EXPECT_NEAR(corners[2].x, 12.0, 1e-4);
    EXPECT_NEAR(corners[2].y, 21.0, 1e-4);
    EXPECT_NEAR(corners[3].x, 8.0, 1e-4);
    EXPECT_NEAR(corners[3].y, 21.0, 1e-4);

    // Normal [-1, 0] points toward right boundary. Furthest surface facing normal has maximum x.
    // Projections of (12, 19) and (12, 21) onto [-1, 0] are -12, which is the minimum!
    // Support point MUST be the exact midpoint: (12, 20)
    const math::Vec2 rightSupport = car.computeSupportPoint(body, math::Vec2 { -1.0, 0.0 });
    EXPECT_NEAR(rightSupport.x, 12.0, 1e-4);
    EXPECT_NEAR(rightSupport.y, 20.0, 1e-4);
}
