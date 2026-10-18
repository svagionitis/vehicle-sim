#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/CableTether2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::BodyType;
using vehicle_sim::physics::CableTether2D;
using vehicle_sim::physics::CableTetherConfig;
using vehicle_sim::physics::JointType;
using vehicle_sim::physics::PhysicsWorld2D;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;
using vehicle_sim::physics::WinchMode;

namespace {
    inline constexpr double kEpsilon { 1e-3 };

    [[nodiscard]] std::uint64_t fnv1aHash64(const void* data, std::size_t size, std::uint64_t hash = 14695981039346656037ULL) noexcept
    {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t i { 0 }; i < size; ++i) {
            hash ^= bytes[i];
            hash *= 1099511628211ULL;
        }
        return hash;
    }
} // namespace

// -----------------------------------------------------------------------------
// Unilateral Tension & Slack Tests
// -----------------------------------------------------------------------------

TEST(CableTetherTest, CableTautTensionTransmission)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.mass = 10.0;
    configA.inertia = 5.0;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.mass = 10.0;
    configB.inertia = 5.0;
    configB.position = Vec2 { 5.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 5.0;

    CableTether2D cable { cableConfig };
    EXPECT_EQ(cable.getType(), JointType::Cable);
    EXPECT_DOUBLE_EQ(cable.getRestLength(), 5.0);
    EXPECT_FALSE(cable.isBroken());
    world.addJoint(cable);

    // Apply force to body A pulling it leftward (away from body B)
    for (int step = 0; step < 50; ++step) {
        bodyA.applyForce(Vec2 { -200.0, 0.0 });
        world.step(0.016);
    }

    // Both bodies should have accelerated leftward due to cable tension
    EXPECT_LT(bodyA.getLinearVelocity().x, -1.0);
    EXPECT_LT(bodyB.getLinearVelocity().x, -1.0);
    EXPECT_GE(cable.getCurrentLength(), 5.0 - kEpsilon);
    EXPECT_TRUE(cable.isTaut());
}

TEST(CableTetherTest, CableSlackZeroCompression)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.mass = 10.0;
    configA.inertia = 5.0;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.mass = 10.0;
    configB.inertia = 5.0;
    configB.position = Vec2 { 5.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 10.0; // Cable is slack (current distance is 5.0m)

    CableTether2D cable { cableConfig };
    world.addJoint(cable);

    EXPECT_TRUE(cable.isSlack());
    EXPECT_FALSE(cable.isTaut());

    // Push Body A towards Body B
    for (int step = 0; step < 30; ++step) {
        bodyA.applyForce(Vec2 { 100.0, 0.0 });
        world.step(0.016);
    }

    // Body A moved right, but Body B remained stationary because cable cannot push
    EXPECT_GT(bodyA.getLinearVelocity().x, 0.5);
    EXPECT_NEAR(bodyB.getLinearVelocity().x, 0.0, 1e-6);
    EXPECT_NEAR(bodyB.getLinearVelocity().y, 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(cable.getTension(), 0.0);
}

TEST(CableTetherTest, CablePendulumMotion)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    // Anchor A is a fixed world point (bodyA = nullptr) at (0, 0)
    RigidBodyConfig configB {};
    configB.mass = 2.0;
    configB.inertia = 0.5;
    configB.position = Vec2 { 2.0, 0.0 }; // Released horizontally at length 2m
    RigidBody2D bodyB { configB };
    world.addBody(bodyB);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = nullptr; // World anchor
    cableConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    cableConfig.bodyB = &bodyB;
    cableConfig.localAnchorB = Vec2 { 0.0, 0.0 };
    cableConfig.restLength = 2.0;

    CableTether2D cable { cableConfig };
    world.addJoint(cable);

    // Step through a quarter swing (about 0.7 seconds)
    for (int step = 0; step < 45; ++step) {
        world.step(0.016);
        // Distance should be conserved to <= 2.0m (unilateral)
        EXPECT_LE(bodyB.getPosition().length(), 2.05);
    }

    // Body should be near the bottom with high velocity
    EXPECT_LT(bodyB.getPosition().y, -1.0);
    EXPECT_LT(bodyB.getLinearVelocity().x, -2.0); // Moving left at bottom of pendulum
}

// -----------------------------------------------------------------------------
// Winch Spooling & Motor Tests
// -----------------------------------------------------------------------------

TEST(CableTetherTest, WinchHaulInVerticalLift)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    RigidBodyConfig configB {};
    configB.mass = 10.0;
    configB.inertia = 2.0;
    configB.position = Vec2 { 0.0, -5.0 };
    RigidBody2D payload { configB };
    world.addBody(payload);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = nullptr; // Ceiling anchor at (0, 0)
    cableConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    cableConfig.bodyB = &payload;
    cableConfig.restLength = 5.0;
    cableConfig.minRestLength = 1.0;
    cableConfig.spoolSpeed = 1.0; // 1 m/s haul in
    cableConfig.maxPullForce = 5000.0;
    cableConfig.winchMode = WinchMode::HaulIn;

    CableTether2D cable { cableConfig };
    world.addJoint(cable);

    const double initialY { payload.getPosition().y };

    for (int step = 0; step < 100; ++step) {
        world.step(0.016);
    }

    // Cable should have reeled in and lifted payload vertically
    EXPECT_LT(cable.getRestLength(), 5.0);
    EXPECT_GT(payload.getPosition().y, initialY);
}

TEST(CableTetherTest, WinchMotorStallUnderOverload)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    RigidBodyConfig configB {};
    configB.mass = 100.0; // Weight ~ 981 N
    configB.inertia = 10.0;
    configB.position = Vec2 { 0.0, -5.0 };
    RigidBody2D payload { configB };
    world.addBody(payload);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = nullptr;
    cableConfig.localAnchorA = Vec2 { 0.0, 0.0 };
    cableConfig.bodyB = &payload;
    cableConfig.restLength = 5.0;
    cableConfig.spoolSpeed = 1.0;
    cableConfig.maxPullForce = 500.0; // Weak motor: 500 N < 981 N weight!
    cableConfig.winchMode = WinchMode::HaulIn;

    CableTether2D cable { cableConfig };
    world.addJoint(cable);

    // Let system reach equilibrium under tension
    for (int step = 0; step < 60; ++step) {
        world.step(0.016);
    }

    const double stalledRestLength { cable.getRestLength() };
    for (int step = 0; step < 60; ++step) {
        world.step(0.016);
    }
    EXPECT_NEAR(cable.getRestLength(), stalledRestLength, 0.05);
}

TEST(CableTetherTest, WinchPayoutAndFreeWheel)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.position = Vec2 { 2.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 2.0;
    cableConfig.maxRestLength = 10.0;
    cableConfig.spoolSpeed = 2.0; // 2 m/s payout
    cableConfig.winchMode = WinchMode::Payout;

    CableTether2D cable { cableConfig };
    world.addJoint(cable);

    // Step for 1 second (approx 62 steps)
    for (int step = 0; step < 62; ++step) {
        world.step(0.016);
    }

    // Rest length should have grown to approximately 2.0 + 2.0 * 1.0 = ~4.0m
    EXPECT_GT(cable.getRestLength(), 3.8);
    EXPECT_LE(cable.getRestLength(), 10.0);
}

// -----------------------------------------------------------------------------
// Rupture Mechanics & Snap Callbacks
// -----------------------------------------------------------------------------

TEST(CableTetherTest, CableBreakTensionSnap)
{
    PhysicsWorld2D world { Vec2::zero() };

    RigidBodyConfig configA {};
    configA.mass = 10.0;
    configA.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB {};
    configB.mass = 10.0;
    configB.position = Vec2 { 5.0, 0.0 };
    RigidBody2D bodyB { configB };

    world.addBody(bodyA);
    world.addBody(bodyB);

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 5.0;
    cableConfig.breakTension = 500.0; // Break threshold: 500 N

    CableTether2D cable { cableConfig };
    bool snapped { false };
    double recordedSnapTension { 0.0 };

    cable.setOnSnapCallback([&](CableTether2D& joint, double tension) {
        snapped = true;
        recordedSnapTension = tension;
        EXPECT_EQ(&joint, &cable);
    });

    world.addJoint(cable);

    // Apply moderate force (200 N < 500 N)
    for (int step = 0; step < 20; ++step) {
        bodyA.applyForce(Vec2 { -200.0, 0.0 });
        world.step(0.016);
    }
    EXPECT_FALSE(cable.isBroken());
    EXPECT_FALSE(snapped);

    // Apply sudden massive force (2,000 N > 500 N)
    for (int step = 0; step < 20; ++step) {
        bodyA.applyForce(Vec2 { -2000.0, 0.0 });
        world.step(0.016);
        if (cable.isBroken()) {
            break;
        }
    }

    EXPECT_TRUE(cable.isBroken());
    EXPECT_TRUE(snapped);
    EXPECT_GE(recordedSnapTension, 500.0);
    EXPECT_DOUBLE_EQ(cable.getTension(), 0.0);
}

// -----------------------------------------------------------------------------
// Snapshot & State Capture
// -----------------------------------------------------------------------------

TEST(CableTetherTest, SnapshotSaveAndRestore)
{
    CableTetherConfig cableConfig {};
    cableConfig.restLength = 8.5;
    CableTether2D cable { cableConfig };

    cable.setRestLength(6.2);
    auto snapshot = cable.captureState();
    EXPECT_DOUBLE_EQ(snapshot.motorImpulse, 6.2);
    EXPECT_FALSE(snapshot.isBroken);

    cable.setRestLength(15.0);
    EXPECT_DOUBLE_EQ(cable.getRestLength(), 15.0);

    cable.restoreState(snapshot);
    EXPECT_DOUBLE_EQ(cable.getRestLength(), 6.2);
}

// -----------------------------------------------------------------------------
// Catenary Sag Curve Sampling Tests
// -----------------------------------------------------------------------------

TEST(CableTetherTest, CatenaryStraightWhenTaut)
{
    RigidBodyConfig configA { 1.0, 1.0, 0.0, 0.0, 1.0, BodyType::Static, Vec2 { 0.0, 0.0 }, 0.0, false };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB { 1.0, 1.0, 0.0, 0.0, 1.0, BodyType::Static, Vec2 { 10.0, 0.0 }, 0.0, false };
    RigidBody2D bodyB { configB };

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 10.0; // Distance == restLength (taut)

    CableTether2D cable { cableConfig };
    std::vector<Vec2> points {};
    cable.sampleCatenary(11, Vec2 { 0.0, -9.81 }, points);

    ASSERT_EQ(points.size(), 11u);
    // All points should lie along y = 0
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double expectedX = static_cast<double>(i);
        EXPECT_NEAR(points[i].x, expectedX, 1e-4);
        EXPECT_NEAR(points[i].y, 0.0, 1e-4);
    }
}

TEST(CableTetherTest, CatenarySagWhenSlack)
{
    RigidBodyConfig configA { 1.0, 1.0, 0.0, 0.0, 1.0, BodyType::Static, Vec2 { 0.0, 0.0 }, 0.0, false };
    RigidBody2D bodyA { configA };

    RigidBodyConfig configB { 1.0, 1.0, 0.0, 0.0, 1.0, BodyType::Static, Vec2 { 6.0, 0.0 }, 0.0, false };
    RigidBody2D bodyB { configB };

    CableTetherConfig cableConfig {};
    cableConfig.bodyA = &bodyA;
    cableConfig.bodyB = &bodyB;
    cableConfig.restLength = 10.0; // Slack: d = 6.0m < L0 = 10.0m

    CableTether2D cable { cableConfig };
    std::vector<Vec2> points {};
    cable.sampleCatenary(11, Vec2 { 0.0, -9.81 }, points);

    ASSERT_EQ(points.size(), 11u);
    // Midpoint (index 5) should sag downward in y
    EXPECT_NEAR(points[0].x, 0.0, 1e-4);
    EXPECT_NEAR(points[0].y, 0.0, 1e-4);
    EXPECT_NEAR(points[10].x, 6.0, 1e-4);
    EXPECT_NEAR(points[10].y, 0.0, 1e-4);

    EXPECT_NEAR(points[5].x, 3.0, 1e-4);
    EXPECT_LT(points[5].y, -1.0); // Significant sag depth
}

// -----------------------------------------------------------------------------
// Cross-Platform Determinism Verification (1,000 Steps)
// -----------------------------------------------------------------------------

TEST(CableTetherTest, CableTetherDeterminism1000Steps)
{
    auto runSim = []() -> std::uint64_t {
        PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

        RigidBodyConfig cfgA {};
        cfgA.mass = 50.0;
        cfgA.inertia = 20.0;
        cfgA.position = Vec2 { 0.0, 10.0 };
        RigidBody2D bodyA { cfgA };

        RigidBodyConfig cfgB {};
        cfgB.mass = 20.0;
        cfgB.inertia = 8.0;
        cfgB.position = Vec2 { 4.0, 10.0 };
        RigidBody2D bodyB { cfgB };

        world.addBody(bodyA);
        world.addBody(bodyB);

        CableTetherConfig tetherCfg {};
        tetherCfg.bodyA = &bodyA;
        tetherCfg.bodyB = &bodyB;
        tetherCfg.restLength = 4.0;
        tetherCfg.spoolSpeed = 0.5;
        tetherCfg.winchMode = WinchMode::HaulIn;
        CableTether2D tether { tetherCfg };
        world.addJoint(tether);

        std::uint64_t hash { 14695981039346656037ULL };

        for (int step = 0; step < 1000; ++step) {
            bodyA.applyForce(Vec2 { std::sin(step * 0.05) * 100.0, 0.0 });
            world.step(0.016);

            const Vec2 posA { bodyA.getPosition() };
            const Vec2 velA { bodyA.getLinearVelocity() };
            const Vec2 posB { bodyB.getPosition() };
            const Vec2 velB { bodyB.getLinearVelocity() };
            const double restLen { tether.getRestLength() };
            const double tension { tether.getTension() };

            hash = fnv1aHash64(&posA, sizeof(posA), hash);
            hash = fnv1aHash64(&velA, sizeof(velA), hash);
            hash = fnv1aHash64(&posB, sizeof(posB), hash);
            hash = fnv1aHash64(&velB, sizeof(velB), hash);
            hash = fnv1aHash64(&restLen, sizeof(restLen), hash);
            hash = fnv1aHash64(&tension, sizeof(tension), hash);
        }
        return hash;
    };

    const std::uint64_t hash1 { runSim() };
    const std::uint64_t hash2 { runSim() };
    EXPECT_EQ(hash1, hash2);
    EXPECT_NE(hash1, 0ULL);
}

} // namespace vehicle_sim::testing
