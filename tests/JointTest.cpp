#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/DistanceJoint2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RevoluteJoint2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::BodyType;
using vehicle_sim::physics::DistanceJoint2D;
using vehicle_sim::physics::DistanceJointConfig;
using vehicle_sim::physics::JointType;
using vehicle_sim::physics::PhysicsWorld2D;
using vehicle_sim::physics::RevoluteJoint2D;
using vehicle_sim::physics::RevoluteJointConfig;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kEpsilon { 1e-4 };

// -----------------------------------------------------------------------------
// DistanceJoint2D Tests
// -----------------------------------------------------------------------------

TEST(JointTest, DistanceJointRestLengthConservation)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.mass = 5.0;
    configA.inertia = 2.0;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.mass = 5.0;
    configB.inertia = 2.0;
    configB.position = Vec2 { 3.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    DistanceJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    jointConfig.localAnchorB = Vec2 { 0.0, 0.0 };
    jointConfig.targetDistance = 3.0;

    DistanceJoint2D joint { jointConfig };
    EXPECT_EQ(joint.getType(), JointType::Distance);
    EXPECT_DOUBLE_EQ(joint.getTargetDistance(), 3.0);
    EXPECT_NEAR(joint.getCurrentDistance(), 3.0, 1e-6);

    world.addJoint(joint);
    EXPECT_EQ(world.getJointCount(), 1U);

    // Apply strong opposing outward forces for 300 steps
    constexpr double dt { 0.016 };
    for (int i = 0; i < 300; ++i) {
        bodyA.applyForce(Vec2 { -800.0, 0.0 });
        bodyB.applyForce(Vec2 { 800.0, 0.0 });
        world.step(dt);

        const double currentDistance { joint.getCurrentDistance() };
        EXPECT_NEAR(currentDistance, 3.0, 0.01) << "Step " << i;
    }

    const Vec2 reactionForce { joint.getReactionForce(1.0 / dt) };
    EXPECT_GT(reactionForce.length(), 0.0);
    EXPECT_DOUBLE_EQ(joint.getReactionTorque(1.0 / dt), 0.0);
}

TEST(JointTest, DistanceJointStaticAnchorPendulum)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    RigidBodyConfig staticConfig {};
    staticConfig.bodyType = BodyType::Static;
    staticConfig.position = Vec2 { 0.0, 0.0 };
    RigidBody2D anchorBody { staticConfig };

    RigidBodyConfig bobConfig {};
    bobConfig.bodyType = BodyType::Dynamic;
    bobConfig.mass = 2.0;
    bobConfig.inertia = 1.0;
    bobConfig.position = Vec2 { 2.0, 0.0 }; // Released horizontally
    RigidBody2D bobBody { bobConfig };

    world.addBody(anchorBody);
    world.addBody(bobBody);

    DistanceJointConfig jointConfig {};
    jointConfig.bodyA = &anchorBody;
    jointConfig.bodyB = &bobBody;
    jointConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    jointConfig.localAnchorB = Vec2 { 0.0, 0.0 };
    jointConfig.targetDistance = 2.0;

    DistanceJoint2D joint { jointConfig };
    world.addJoint(joint);

    constexpr double dt { 0.01 };
    for (int i = 0; i < 200; ++i) {
        world.step(dt);
        const double distance { joint.getCurrentDistance() };
        EXPECT_NEAR(distance, 2.0, 0.01) << "Step " << i;
    }

    // Pendulum bob should have swung downward
    EXPECT_LT(bobBody.getPosition().y, 0.0);
    EXPECT_GT(bobBody.getLinearVelocity().length(), 0.0);
}

TEST(JointTest, DistanceJointSoftSpringCompliance)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.bodyType = BodyType::Static;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.bodyType = BodyType::Dynamic;
    configB.mass = 1.0;
    configB.inertia = 1.0;
    configB.position = Vec2 { 3.0, 0.0 }; // Displaced by +1.0m from rest length of 2.0m
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    DistanceJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.targetDistance = 2.0;
    jointConfig.frequencyHz = 2.0;
    jointConfig.dampingRatio = 0.5;

    DistanceJoint2D joint { jointConfig };
    EXPECT_DOUBLE_EQ(joint.getFrequencyHz(), 2.0);
    EXPECT_DOUBLE_EQ(joint.getDampingRatio(), 0.5);

    world.addJoint(joint);

    constexpr double dt { 0.016 };
    // Step forward 200 steps (over ~3 seconds)
    for (int i = 0; i < 200; ++i) {
        world.step(dt);
    }

    // Damped harmonic motion should have decayed close to 2.0m rest distance
    const double finalDistance { joint.getCurrentDistance() };
    EXPECT_NEAR(finalDistance, 2.0, 0.08);
}

TEST(JointTest, DistanceJointZeroLengthProtection)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig config {};
    config.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { config };
    RigidBody2D bodyB { config };

    world.addBody(bodyA);
    world.addBody(bodyB);

    DistanceJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.targetDistance = 0.0;

    DistanceJoint2D joint { jointConfig };
    world.addJoint(joint);

    EXPECT_NO_THROW({ world.step(0.016); });

    EXPECT_FALSE(std::isnan(bodyA.getPosition().x));
    EXPECT_FALSE(std::isnan(bodyB.getPosition().x));
}

// -----------------------------------------------------------------------------
// RevoluteJoint2D Tests
// -----------------------------------------------------------------------------

TEST(JointTest, RevoluteJointAnchorCoincidence)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.mass = 10.0;
    configA.inertia = 5.0;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.mass = 8.0;
    configB.inertia = 4.0;
    configB.position = Vec2 { 2.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    // Pivot pin located at (1.0, 0.0) in world space
    RevoluteJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.localAnchorA = Vec2 { 1.0, 0.0 };
    jointConfig.localAnchorB = Vec2 { -1.0, 0.0 };

    RevoluteJoint2D joint { jointConfig };
    EXPECT_EQ(joint.getType(), JointType::Revolute);
    world.addJoint(joint);

    // Apply transverse forces creating rotation and pull
    constexpr double dt { 0.016 };
    for (int i = 0; i < 200; ++i) {
        bodyA.applyForce(Vec2 { -500.0, 200.0 });
        bodyB.applyForce(Vec2 { 500.0, -200.0 });
        world.step(dt);

        const Vec2 anchorA { joint.getWorldAnchorA() };
        const Vec2 anchorB { joint.getWorldAnchorB() };
        const double separation { (anchorB - anchorA).length() };
        EXPECT_NEAR(separation, 0.0, 0.01) << "Step " << i;
    }

    const Vec2 reactionForce { joint.getReactionForce(1.0 / dt) };
    EXPECT_GT(reactionForce.length(), 0.0);
}

TEST(JointTest, RevoluteJointAngularLimits)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.bodyType = BodyType::Static;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.bodyType = BodyType::Dynamic;
    configB.mass = 2.0;
    configB.inertia = 1.0;
    configB.position = Vec2 { 1.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    RevoluteJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    jointConfig.localAnchorB = Vec2 { -1.0, 0.0 };
    jointConfig.enableLimit = true;
    jointConfig.lowerAngle = -0.4;
    jointConfig.upperAngle = 0.4;

    RevoluteJoint2D joint { jointConfig };
    EXPECT_TRUE(joint.isLimitEnabled());
    EXPECT_DOUBLE_EQ(joint.getLowerLimit(), -0.4);
    EXPECT_DOUBLE_EQ(joint.getUpperLimit(), 0.4);

    world.addJoint(joint);

    constexpr double dt { 0.016 };

    // Apply large positive torque driving into upper limit
    for (int i = 0; i < 100; ++i) {
        bodyB.applyTorque(400.0);
        world.step(dt);
    }

    double angle { joint.getJointAngle() };
    EXPECT_LE(angle, 0.4 + 0.04);
    EXPECT_GE(angle, 0.35);

    // Apply large negative torque driving into lower limit
    for (int i = 0; i < 150; ++i) {
        bodyB.applyTorque(-400.0);
        world.step(dt);
    }

    angle = joint.getJointAngle();
    EXPECT_GE(angle, -0.4 - 0.04);
    EXPECT_LE(angle, -0.35);
}

TEST(JointTest, RevoluteJointActiveMotorDrive)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.bodyType = BodyType::Static;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.bodyType = BodyType::Dynamic;
    configB.mass = 2.0;
    configB.inertia = 1.0;
    configB.position = Vec2 { 1.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    RevoluteJointConfig jointConfig {};
    jointConfig.bodyA = &bodyA;
    jointConfig.bodyB = &bodyB;
    jointConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    jointConfig.localAnchorB = Vec2 { -1.0, 0.0 };
    jointConfig.enableMotor = true;
    jointConfig.motorSpeed = 4.0; // 4.0 rad/s
    jointConfig.maxMotorTorque = 50.0;

    RevoluteJoint2D joint { jointConfig };
    EXPECT_TRUE(joint.isMotorEnabled());
    EXPECT_DOUBLE_EQ(joint.getMotorSpeed(), 4.0);
    EXPECT_DOUBLE_EQ(joint.getMaxMotorTorque(), 50.0);

    world.addJoint(joint);

    constexpr double dt { 0.016 };
    for (int i = 0; i < 80; ++i) {
        world.step(dt);
    }

    EXPECT_NEAR(joint.getJointSpeed(), 4.0, 0.1);
    EXPECT_GT(joint.getMotorTorque(1.0 / dt), -1e-6);
}

TEST(JointTest, RevoluteJointDoublePendulum)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    // Static base
    RigidBodyConfig config0 {};
    config0.bodyType = BodyType::Static;
    config0.position = Vec2 { 0.0, 0.0 };
    RigidBody2D base { config0 };

    // Link 1
    RigidBodyConfig config1 {};
    config1.bodyType = BodyType::Dynamic;
    config1.mass = 2.0;
    config1.inertia = 1.0;
    config1.position = Vec2 { 1.0, 0.0 };
    RigidBody2D link1 { config1 };

    // Link 2
    RigidBodyConfig config2 {};
    config2.bodyType = BodyType::Dynamic;
    config2.mass = 2.0;
    config2.inertia = 1.0;
    config2.position = Vec2 { 3.0, 0.0 };
    RigidBody2D link2 { config2 };

    world.addBody(base);
    world.addBody(link1);
    world.addBody(link2);

    // Joint 1: base to link1
    RevoluteJointConfig j1Config {};
    j1Config.bodyA = &base;
    j1Config.bodyB = &link1;
    j1Config.localAnchorA = Vec2 { 0.0, 0.0 };
    j1Config.localAnchorB = Vec2 { -1.0, 0.0 };
    RevoluteJoint2D joint1 { j1Config };

    // Joint 2: link1 to link2
    RevoluteJointConfig j2Config {};
    j2Config.bodyA = &link1;
    j2Config.bodyB = &link2;
    j2Config.localAnchorA = Vec2 { 1.0, 0.0 };
    j2Config.localAnchorB = Vec2 { -1.0, 0.0 };
    RevoluteJoint2D joint2 { j2Config };

    world.addJoint(joint1);
    world.addJoint(joint2);
    EXPECT_EQ(world.getJointCount(), 2U);

    constexpr double dt { 0.01 };
    for (int i = 0; i < 300; ++i) {
        world.step(dt);

        const double sep1 { (joint1.getWorldAnchorB() - joint1.getWorldAnchorA()).length() };
        const double sep2 { (joint2.getWorldAnchorB() - joint2.getWorldAnchorA()).length() };

        EXPECT_NEAR(sep1, 0.0, 0.01) << "Joint1 step " << i;
        EXPECT_NEAR(sep2, 0.0, 0.01) << "Joint2 step " << i;
    }

    EXPECT_FALSE(std::isnan(link1.getPosition().x));
    EXPECT_FALSE(std::isnan(link2.getPosition().x));
}

// -----------------------------------------------------------------------------
// PhysicsWorld Lifecycle & Determinism Tests
// -----------------------------------------------------------------------------

TEST(JointTest, PhysicsWorldJointLifecycle)
{
    PhysicsWorld2D world { Vec2::zero() };
    EXPECT_EQ(world.getJointCount(), 0U);

    RigidBodyConfig cfg {};
    RigidBody2D b1 { cfg };
    RigidBody2D b2 { cfg };

    DistanceJointConfig dCfg {};
    dCfg.bodyA = &b1;
    dCfg.bodyB = &b2;
    DistanceJoint2D j1 { dCfg };

    RevoluteJointConfig rCfg {};
    rCfg.bodyA = &b1;
    rCfg.bodyB = &b2;
    RevoluteJoint2D j2 { rCfg };

    world.addJoint(j1);
    EXPECT_EQ(world.getJointCount(), 1U);

    world.addJoint(j2);
    EXPECT_EQ(world.getJointCount(), 2U);

    // Prevent duplicate addition
    world.addJoint(j1);
    EXPECT_EQ(world.getJointCount(), 2U);

    world.removeJoint(j1);
    EXPECT_EQ(world.getJointCount(), 1U);

    world.clearJoints();
    EXPECT_EQ(world.getJointCount(), 0U);

    // Ensure step on empty joint world functions normally
    world.step(0.016);
}

TEST(JointTest, CrossPlatformDeterminism)
{
    auto runSim = [](double& outX, double& outY, double& outAngle) {
        PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

        RigidBodyConfig bConfig {};
        bConfig.mass = 4.0;
        bConfig.inertia = 2.0;
        bConfig.position = Vec2 { 1.5, 0.0 };
        RigidBody2D body { bConfig };

        RigidBodyConfig staticConfig {};
        staticConfig.bodyType = BodyType::Static;
        RigidBody2D anchor { staticConfig };

        world.addBody(anchor);
        world.addBody(body);

        RevoluteJointConfig jConfig {};
        jConfig.bodyA = &anchor;
        jConfig.bodyB = &body;
        jConfig.localAnchorA = Vec2 { 0.0, 0.0 };
        jConfig.localAnchorB = Vec2 { -1.5, 0.0 };
        RevoluteJoint2D joint { jConfig };
        world.addJoint(joint);

        constexpr double dt { 0.016 };
        for (int i = 0; i < 100; ++i) {
            world.step(dt);
        }

        outX = body.getPosition().x;
        outY = body.getPosition().y;
        outAngle = body.getOrientation();
    };

    double x1 { 0.0 };
    double y1 { 0.0 };
    double a1 { 0.0 };
    double x2 { 0.0 };
    double y2 { 0.0 };
    double a2 { 0.0 };

    runSim(x1, y1, a1);
    runSim(x2, y2, a2);

    EXPECT_DOUBLE_EQ(x1, x2);
    EXPECT_DOUBLE_EQ(y1, y2);
    EXPECT_DOUBLE_EQ(a1, a2);
}

} // namespace vehicle_sim::testing
