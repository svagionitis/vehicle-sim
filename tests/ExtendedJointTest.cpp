#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/PrismaticJoint2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/physics/WeldJoint2D.h"
#include "libs/VehicleSimCore/physics/WheelJoint2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::BodyType;
using vehicle_sim::physics::JointType;
using vehicle_sim::physics::PhysicsWorld2D;
using vehicle_sim::physics::PrismaticJoint2D;
using vehicle_sim::physics::PrismaticJointConfig;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;
using vehicle_sim::physics::WeldJoint2D;
using vehicle_sim::physics::WeldJointConfig;
using vehicle_sim::physics::WheelJoint2D;
using vehicle_sim::physics::WheelJointConfig;

// -----------------------------------------------------------------------------
// PrismaticJoint2D Tests
// -----------------------------------------------------------------------------

TEST(ExtendedJointTest, PrismaticLineLock)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D staticBody { staticCfg };

    RigidBodyConfig sliderCfg {};
    sliderCfg.bodyType = BodyType::Dynamic;
    sliderCfg.mass = 5.0;
    sliderCfg.inertia = 2.0;
    sliderCfg.position = Vec2 { 1.0, 0.0 };
    RigidBody2D sliderBody { sliderCfg };

    world.addBody(staticBody);
    world.addBody(sliderBody);

    PrismaticJointConfig jointCfg {};
    jointCfg.bodyA = &staticBody;
    jointCfg.bodyB = &sliderBody;
    jointCfg.localAnchorA = Vec2 { 0.0, 0.0 };
    jointCfg.localAnchorB = Vec2 { 0.0, 0.0 };
    jointCfg.localAxisA = Vec2 { 1.0, 0.0 }; // Horizontal line of action

    PrismaticJoint2D joint { jointCfg };
    EXPECT_EQ(joint.getType(), JointType::Prismatic);
    EXPECT_NEAR(joint.getJointTranslation(), 1.0, 1e-6);

    world.addJoint(joint);

    constexpr double dt { 0.016 };
    // Apply diagonal forces with strong perpendicular (y) component
    for (int i = 0; i < 200; ++i) {
        sliderBody.applyForce(Vec2 { 100.0, 500.0 });
        world.step(dt);

        // Perpendicular drift must be virtually zero
        EXPECT_NEAR(sliderBody.getPosition().y, 0.0, 0.005) << "Step " << i;
    }

    // Body should have translated along x
    EXPECT_GT(sliderBody.getPosition().x, 1.0);
    EXPECT_NEAR(sliderBody.getOrientation(), 0.0, 0.01);
}

TEST(ExtendedJointTest, PrismaticAngularLock)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D staticBody { staticCfg };

    RigidBodyConfig sliderCfg {};
    sliderCfg.bodyType = BodyType::Dynamic;
    sliderCfg.mass = 2.0;
    sliderCfg.inertia = 1.0;
    sliderCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D sliderBody { sliderCfg };

    world.addBody(staticBody);
    world.addBody(sliderBody);

    PrismaticJointConfig jointCfg {};
    jointCfg.bodyA = &staticBody;
    jointCfg.bodyB = &sliderBody;
    jointCfg.localAxisA = Vec2 { 1.0, 0.0 };

    PrismaticJoint2D joint { jointCfg };
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    for (int i = 0; i < 150; ++i) {
        sliderBody.applyTorque(250.0);
        world.step(dt);
        EXPECT_NEAR(sliderBody.getOrientation(), 0.0, 0.005);
    }
}

TEST(ExtendedJointTest, PrismaticTravelLimits)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D staticBody { staticCfg };

    RigidBodyConfig sliderCfg {};
    sliderCfg.bodyType = BodyType::Dynamic;
    sliderCfg.mass = 4.0;
    sliderCfg.inertia = 1.0;
    sliderCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D sliderBody { sliderCfg };

    world.addBody(staticBody);
    world.addBody(sliderBody);

    PrismaticJointConfig jointCfg {};
    jointCfg.bodyA = &staticBody;
    jointCfg.bodyB = &sliderBody;
    jointCfg.localAxisA = Vec2 { 1.0, 0.0 };
    jointCfg.enableLimit = true;
    jointCfg.lowerTranslation = -0.5;
    jointCfg.upperTranslation = 1.5;

    PrismaticJoint2D joint { jointCfg };
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    // Push aggressively in positive x direction
    for (int i = 0; i < 200; ++i) {
        sliderBody.applyForce(Vec2 { 800.0, 0.0 });
        world.step(dt);
        EXPECT_LE(joint.getJointTranslation(), 1.51) << "Step " << i;
    }

    EXPECT_NEAR(joint.getJointTranslation(), 1.5, 0.02);
}

TEST(ExtendedJointTest, PrismaticMotorVerticalLift)
{
    // Gravity pointing downward
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D staticBody { staticCfg };

    RigidBodyConfig liftCfg {};
    liftCfg.bodyType = BodyType::Dynamic;
    liftCfg.mass = 5.0; // Weight = 5 * 9.81 = 49.05 N
    liftCfg.inertia = 1.0;
    liftCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D liftBody { liftCfg };

    world.addBody(staticBody);
    world.addBody(liftBody);

    PrismaticJointConfig jointCfg {};
    jointCfg.bodyA = &staticBody;
    jointCfg.bodyB = &liftBody;
    jointCfg.localAxisA = Vec2 { 0.0, 1.0 }; // Vertical elevator/mast
    jointCfg.enableMotor = true;
    jointCfg.motorSpeed = 1.5; // 1.5 m/s upward
    jointCfg.maxMotorForce = 200.0; // Plenty of force to lift 49 N

    PrismaticJoint2D joint { jointCfg };
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    for (int i = 0; i < 200; ++i) {
        world.step(dt);
    }

    // Elevator should be traveling upward at approximately 1.5 m/s
    EXPECT_NEAR(liftBody.getLinearVelocity().y, 1.5, 0.05);
    EXPECT_GT(liftBody.getPosition().y, 2.0);
    EXPECT_GT(joint.getMotorForce(1.0 / dt), 40.0); // Counteracting gravity
}

// -----------------------------------------------------------------------------
// WheelJoint2D Tests
// -----------------------------------------------------------------------------

TEST(ExtendedJointTest, WheelPerpendicularLock)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig chassisCfg {};
    chassisCfg.bodyType = BodyType::Static;
    chassisCfg.position = Vec2 { 0.0, 2.0 };
    RigidBody2D chassis { chassisCfg };

    RigidBodyConfig wheelCfg {};
    wheelCfg.bodyType = BodyType::Dynamic;
    wheelCfg.mass = 10.0;
    wheelCfg.inertia = 1.0;
    wheelCfg.position = Vec2 { 0.0, 1.0 };
    RigidBody2D wheel { wheelCfg };

    world.addBody(chassis);
    world.addBody(wheel);

    WheelJointConfig jointCfg {};
    jointCfg.bodyA = &chassis;
    jointCfg.bodyB = &wheel;
    jointCfg.localAxisA = Vec2 { 0.0, 1.0 }; // Vertical strut
    jointCfg.frequencyHz = 4.0;
    jointCfg.dampingRatio = 0.7;

    WheelJoint2D joint { jointCfg };
    EXPECT_EQ(joint.getType(), JointType::Wheel);
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    // Apply huge lateral force
    for (int i = 0; i < 150; ++i) {
        wheel.applyForce(Vec2 { 1000.0, 0.0 });
        world.step(dt);
        EXPECT_NEAR(wheel.getPosition().x, 0.0, 0.005) << "Step " << i;
    }
}

TEST(ExtendedJointTest, WheelHubFreeSpin)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig chassisCfg {};
    chassisCfg.bodyType = BodyType::Dynamic;
    chassisCfg.mass = 50.0;
    chassisCfg.inertia = 10.0;
    chassisCfg.position = Vec2 { 0.0, 2.0 };
    RigidBody2D chassis { chassisCfg };

    RigidBodyConfig wheelCfg {};
    wheelCfg.bodyType = BodyType::Dynamic;
    wheelCfg.mass = 5.0;
    wheelCfg.inertia = 0.5;
    wheelCfg.position = Vec2 { 0.0, 1.0 };
    RigidBody2D wheel { wheelCfg };

    world.addBody(chassis);
    world.addBody(wheel);

    WheelJointConfig jointCfg {};
    jointCfg.bodyA = &chassis;
    jointCfg.bodyB = &wheel;
    jointCfg.localAxisA = Vec2 { 0.0, 1.0 };

    WheelJoint2D joint { jointCfg };
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    // Apply torque directly to wheel
    for (int i = 0; i < 100; ++i) {
        wheel.applyTorque(20.0);
        world.step(dt);
    }

    // Wheel should be spinning rapidly
    EXPECT_GT(wheel.getAngularVelocity(), 10.0);
    // Chassis should have zero or near-zero rotation
    EXPECT_NEAR(chassis.getAngularVelocity(), 0.0, 0.01);
}

TEST(ExtendedJointTest, WheelHubMotorDrive)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig chassisCfg {};
    chassisCfg.bodyType = BodyType::Dynamic;
    chassisCfg.mass = 50.0;
    chassisCfg.inertia = 20.0;
    chassisCfg.position = Vec2 { 0.0, 2.0 };
    RigidBody2D chassis { chassisCfg };

    RigidBodyConfig wheelCfg {};
    wheelCfg.bodyType = BodyType::Dynamic;
    wheelCfg.mass = 5.0;
    wheelCfg.inertia = 0.5;
    wheelCfg.position = Vec2 { 0.0, 1.0 };
    RigidBody2D wheel { wheelCfg };

    world.addBody(chassis);
    world.addBody(wheel);

    WheelJointConfig jointCfg {};
    jointCfg.bodyA = &chassis;
    jointCfg.bodyB = &wheel;
    jointCfg.enableMotor = true;
    jointCfg.motorSpeed = 30.0; // 30 rad/s
    jointCfg.maxMotorTorque = 100.0;

    WheelJoint2D joint { jointCfg };
    world.addJoint(joint);

    constexpr double dt { 0.016 };
    for (int i = 0; i < 100; ++i) {
        world.step(dt);
    }

    // Relative speed between wheel and chassis should match target motor speed
    EXPECT_NEAR(joint.getJointAngularSpeed(), 30.0, 0.5);
    // Reactive torque caused chassis to rotate in opposite direction
    EXPECT_LT(chassis.getAngularVelocity(), 0.0);
    EXPECT_GT(wheel.getAngularVelocity(), 0.0);
}

// -----------------------------------------------------------------------------
// WeldJoint2D Tests
// -----------------------------------------------------------------------------

TEST(ExtendedJointTest, WeldRigidCantileverBeam)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D wall { staticCfg };

    RigidBodyConfig beamCfg {};
    beamCfg.bodyType = BodyType::Dynamic;
    beamCfg.mass = 5.0;
    beamCfg.inertia = 2.0;
    beamCfg.position = Vec2 { 2.0, 0.0 };
    RigidBody2D beam { beamCfg };

    world.addBody(wall);
    world.addBody(beam);

    WeldJointConfig weldCfg {};
    weldCfg.bodyA = &wall;
    weldCfg.bodyB = &beam;
    weldCfg.localAnchorA = Vec2 { 0.0, 0.0 };
    weldCfg.localAnchorB = Vec2 { -2.0, 0.0 };

    WeldJoint2D weld { weldCfg };
    EXPECT_EQ(weld.getType(), JointType::Weld);
    EXPECT_FALSE(weld.isBroken());
    world.addJoint(weld);

    constexpr double dt { 0.016 };
    for (int i = 0; i < 200; ++i) {
        world.step(dt);
        // Cantilever beam under gravity must maintain rigidity
        EXPECT_NEAR(beam.getPosition().y, 0.0, 0.01) << "Step " << i;
        EXPECT_NEAR(beam.getOrientation(), 0.0, 0.01) << "Step " << i;
    }

    const Vec2 reactionForce { weld.getReactionForce(1.0 / dt) };
    EXPECT_NEAR(reactionForce.y, 5.0 * 9.81, 1.0); // Supports beam weight
}

TEST(ExtendedJointTest, WeldBreakForceRupture)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig bodyACfg {};
    bodyACfg.bodyType = BodyType::Static;
    bodyACfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { bodyACfg };

    RigidBodyConfig bodyBCfg {};
    bodyBCfg.bodyType = BodyType::Dynamic;
    bodyBCfg.mass = 5.0;
    bodyBCfg.inertia = 1.0;
    bodyBCfg.position = Vec2 { 1.0, 0.0 };
    RigidBody2D bodyB { bodyBCfg };

    world.addBody(bodyA);
    world.addBody(bodyB);

    WeldJointConfig weldCfg {};
    weldCfg.bodyA = &bodyA;
    weldCfg.bodyB = &bodyB;
    weldCfg.localAnchorA = Vec2 { 1.0, 0.0 };
    weldCfg.localAnchorB = Vec2 { 0.0, 0.0 };
    weldCfg.breakForce = 500.0; // Fractures at 500 N

    WeldJoint2D weld { weldCfg };
    bool breakCallbackInvoked { false };
    weld.setOnBreakCallback([&breakCallbackInvoked](WeldJoint2D&) {
        breakCallbackInvoked = true;
    });

    world.addJoint(weld);

    constexpr double dt { 0.016 };
    // Step with safe force (200 N < 500 N)
    for (int i = 0; i < 50; ++i) {
        bodyB.applyForce(Vec2 { 200.0, 0.0 });
        world.step(dt);
        EXPECT_FALSE(weld.isBroken());
        EXPECT_FALSE(breakCallbackInvoked);
    }

    // Exceed threshold (800 N > 500 N)
    bodyB.applyForce(Vec2 { 800.0, 0.0 });
    world.step(dt);

    EXPECT_TRUE(weld.isBroken());
    EXPECT_TRUE(breakCallbackInvoked);

    // After rupture, body B detaches and accelerates freely
    for (int i = 0; i < 50; ++i) {
        bodyB.applyForce(Vec2 { 100.0, 0.0 });
        world.step(dt);
    }
    EXPECT_GT(bodyB.getPosition().x, 1.5);
}

// -----------------------------------------------------------------------------
// Cross-Platform Determinism Test
// -----------------------------------------------------------------------------

TEST(ExtendedJointTest, ExtendedJointDeterminism1000Steps)
{
    auto runSimulation = []() -> std::uint64_t {
        PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

        RigidBodyConfig baseCfg {};
        baseCfg.bodyType = BodyType::Static;
        RigidBody2D base { baseCfg };
        world.addBody(base);

        RigidBodyConfig sliderCfg {};
        sliderCfg.bodyType = BodyType::Dynamic;
        sliderCfg.mass = 10.0;
        sliderCfg.inertia = 5.0;
        RigidBody2D slider { sliderCfg };
        world.addBody(slider);

        PrismaticJointConfig pCfg {};
        pCfg.bodyA = &base;
        pCfg.bodyB = &slider;
        pCfg.localAxisA = Vec2 { 1.0, 0.5 }.normalized();
        pCfg.enableLimit = true;
        pCfg.lowerTranslation = -1.0;
        pCfg.upperTranslation = 2.0;
        PrismaticJoint2D pJoint { pCfg };
        world.addJoint(pJoint);

        RigidBodyConfig wheelCfg {};
        wheelCfg.bodyType = BodyType::Dynamic;
        wheelCfg.mass = 2.0;
        wheelCfg.inertia = 0.5;
        RigidBody2D wheel { wheelCfg };
        world.addBody(wheel);

        WheelJointConfig wCfg {};
        wCfg.bodyA = &slider;
        wCfg.bodyB = &wheel;
        wCfg.frequencyHz = 5.0;
        wCfg.dampingRatio = 0.6;
        WheelJoint2D wJoint { wCfg };
        world.addJoint(wJoint);

        constexpr double dt { 0.016 };
        for (int i = 0; i < 1000; ++i) {
            slider.applyForce(Vec2 { std::sin(i * 0.1) * 50.0, 0.0 });
            world.step(dt);
        }

        // 64-bit FNV-1a hash
        std::uint64_t hash { 14695981039346656037ULL };
        auto hashFloat = [&hash](double val) {
            std::uint64_t bits { 0 };
            std::memcpy(&bits, &val, sizeof(double));
            hash ^= bits;
            hash *= 1099511628211ULL;
        };

        hashFloat(slider.getPosition().x);
        hashFloat(slider.getPosition().y);
        hashFloat(slider.getLinearVelocity().x);
        hashFloat(slider.getLinearVelocity().y);
        hashFloat(wheel.getPosition().x);
        hashFloat(wheel.getPosition().y);
        hashFloat(wheel.getAngularVelocity());

        return hash;
    };

    const std::uint64_t hash1 { runSimulation() };
    const std::uint64_t hash2 { runSimulation() };

    EXPECT_EQ(hash1, hash2) << "Extended joints simulation must be 100% bitwise deterministic";
}

} // namespace vehicle_sim::testing
