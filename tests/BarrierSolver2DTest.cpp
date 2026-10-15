#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/BarrierSolver2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::physics;

TEST(BarrierSolver2DTest, InteriorBodyUnchanged)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.position = math::Vec2 { 0.0, 0.0 };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity({ 5.0, 2.0 });

    BarrierBounds2D bounds { -50.0, 50.0, -50.0, 50.0 };
    MotionLabCarDynamics car { };

    BarrierSolver2D::solveInside(body, bounds, [&](const math::Vec2& normal) {
        return car.computeSupportPoint(body, normal);
    });

    EXPECT_DOUBLE_EQ(body.getPosition().x, 0.0);
    EXPECT_DOUBLE_EQ(body.getPosition().y, 0.0);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 5.0);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, 2.0);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), 0.0);
}

TEST(BarrierSolver2DTest, FlatFaceContactGeneratesZeroRotationalTorque)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    // Car length is 4.0m -> halfLength is 2.0m. Placing center at x = 49.5 means right edge is at 51.5 (penetration = 1.5m into wall at 50.0)
    bodyConfig.position = math::Vec2 { 49.5, 0.0 };
    bodyConfig.orientation = 0.0;
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity({ 10.0, 0.0 }); // Heading straight into right boundary

    BarrierBounds2D bounds { -50.0, 50.0, -50.0, 50.0 };
    MotionLabCarDynamics car { };

    BarrierSolver2D::solveInside(body, bounds, [&](const math::Vec2& normal) {
        return car.computeSupportPoint(body, normal);
    }, 0.55);

    // Car must be positioned back inside so right edge <= 50.0 -> center <= 48.0
    EXPECT_LE(body.getPosition().x, 48.0 + 1e-4);
    // Linear velocity along X should rebound to negative
    EXPECT_LT(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().x, -5.5, 0.5); // -(1 + 0.55) * 10 / effectiveMass
    // Zero rotational spin because face midpoint was used!
    EXPECT_NEAR(body.getAngularVelocity(), 0.0, 1e-8);
}

TEST(BarrierSolver2DTest, CornerContactAppliesTorqueAndRotationalImpulse)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    // Rotate 45 degrees so single front-right corner strikes the right wall
    bodyConfig.position = math::Vec2 { 48.5, 0.0 };
    bodyConfig.orientation = 0.785398; // ~45 degrees
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity({ 8.0, 0.0 });

    BarrierBounds2D bounds { -50.0, 50.0, -50.0, 50.0 };
    MotionLabCarDynamics car { };

    BarrierSolver2D::solveInside(body, bounds, [&](const math::Vec2& normal) {
        return car.computeSupportPoint(body, normal);
    }, 0.55);

    // Off-center contact must induce non-zero angular spin
    EXPECT_NE(body.getAngularVelocity(), 0.0);
    EXPECT_LT(body.getLinearVelocity().x, 0.0);
}

TEST(BarrierSolver2DTest, FourPassCornerPenetrationResolution)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    // Deep penetration into both right and top boundaries (corner penetration)
    bodyConfig.position = math::Vec2 { 51.0, 51.0 };
    RigidBody2D body { bodyConfig };
    body.setLinearVelocity({ 5.0, 5.0 });

    BarrierBounds2D bounds { -50.0, 50.0, -50.0, 50.0 };
    MotionLabCarDynamics car { };

    BarrierSolver2D::solveInside(body, bounds, [&](const math::Vec2& normal) {
        return car.computeSupportPoint(body, normal);
    });

    // Body should be cleanly inside both boundaries
    EXPECT_LE(body.getPosition().x, 48.0 + 1e-4);
    EXPECT_LE(body.getPosition().y, 49.0 + 1e-4);
}

TEST(BarrierSolver2DTest, RestingContactSuppressesTinyBounce)
{
    RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    bodyConfig.position = math::Vec2 { 0.0, -49.1 }; // Bottom wall at -50.0, bottom edge is at -50.1
    RigidBody2D body { bodyConfig };
    // Very gentle downward velocity (speed < 0.5 m/s)
    body.setLinearVelocity({ 0.0, -0.3 });

    BarrierBounds2D bounds { -50.0, 50.0, -50.0, 50.0 };
    MotionLabCarDynamics car { };

    BarrierSolver2D::solveInside(body, bounds, [&](const math::Vec2& normal) {
        return car.computeSupportPoint(body, normal);
    }, 0.55);

    // Bounce should be suppressed (e = 0.0), bringing normal speed to 0.0 without spring rebound
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 1e-4);
}
