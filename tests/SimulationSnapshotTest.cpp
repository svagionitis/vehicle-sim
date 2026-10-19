#include <gtest/gtest.h>

#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RevoluteJoint2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/physics/SimulationSnapshot2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::physics;

TEST(SimulationSnapshotTest, DefaultConstructionAndChecksum)
{
    SimulationSnapshot2D snapshot {};
    snapshot.frameIndex = 10U;
    snapshot.timestamp = 0.166667;
    snapshot.bodyCount = 1U;
    snapshot.bodies[0].bodyId = 0U;
    snapshot.bodies[0].position = math::Vec2 { 1.5, 2.5 };
    snapshot.bodies[0].linearVelocity = math::Vec2 { 0.5, -1.0 };
    snapshot.bodies[0].orientation = 0.785;
    snapshot.bodies[0].angularVelocity = 0.1;

    snapshot.stateChecksum = snapshot.computeChecksum();
    EXPECT_NE(snapshot.stateChecksum, 0ULL);
    EXPECT_TRUE(snapshot.isValid());

    // Tampering test: modify position without updating checksum
    snapshot.bodies[0].position.x += 0.001;
    EXPECT_FALSE(snapshot.isValid());
}

TEST(SimulationSnapshotTest, RingBufferLifecycle)
{
    SnapshotRingBuffer2D<4U> ringBuffer {};
    EXPECT_TRUE(ringBuffer.empty());
    EXPECT_EQ(ringBuffer.size(), 0U);
    EXPECT_EQ(ringBuffer.capacity(), 4U);
    EXPECT_EQ(ringBuffer.getLatest(), nullptr);

    for (std::uint32_t i { 0U }; i < 6U; ++i) {
        SimulationSnapshot2D snap {};
        snap.frameIndex = i;
        snap.stateChecksum = snap.computeChecksum();
        ringBuffer.push(snap);
    }

    EXPECT_EQ(ringBuffer.size(), 4U);
    EXPECT_FALSE(ringBuffer.empty());

    // Oldest frames 0 and 1 should have been overwritten
    EXPECT_EQ(ringBuffer.get(0U), nullptr);
    EXPECT_EQ(ringBuffer.get(1U), nullptr);

    // Frames 2, 3, 4, 5 should be present
    EXPECT_NE(ringBuffer.get(2U), nullptr);
    EXPECT_NE(ringBuffer.get(3U), nullptr);
    EXPECT_NE(ringBuffer.get(4U), nullptr);
    EXPECT_NE(ringBuffer.get(5U), nullptr);

    const auto* latest { ringBuffer.getLatest() };
    ASSERT_NE(latest, nullptr);
    EXPECT_EQ(latest->frameIndex, 5U);

    ringBuffer.clear();
    EXPECT_TRUE(ringBuffer.empty());
    EXPECT_EQ(ringBuffer.size(), 0U);
}

TEST(SimulationSnapshotTest, CaptureAndRestoreRigidBodyState)
{
    PhysicsWorld2D world { math::Vec2 { 0.0, -9.81 } };

    RigidBodyConfig cfg1 {};
    cfg1.mass = 2.0;
    cfg1.inertia = 1.0;
    cfg1.position = math::Vec2 { 0.0, 10.0 };
    RigidBody2D body1 { cfg1 };
    world.addBody(body1);

    RigidBodyConfig cfg2 {};
    cfg2.mass = 1.5;
    cfg2.inertia = 0.8;
    cfg2.position = math::Vec2 { 5.0, 10.0 };
    RigidBody2D body2 { cfg2 };
    world.addBody(body2);

    constexpr double dt { 1.0 / 60.0 };

    // Step 15 frames
    for (std::uint32_t f { 0U }; f < 15U; ++f) {
        body1.applyForce(math::Vec2 { 10.0, 0.0 });
        world.step(dt);
    }

    SimulationSnapshot2D snapshot15 {};
    EXPECT_TRUE(world.captureSnapshot(15U, snapshot15));
    EXPECT_TRUE(snapshot15.isValid());
    EXPECT_EQ(snapshot15.bodyCount, 2U);

    const math::Vec2 savedPos1 { body1.getPosition() };
    const math::Vec2 savedVel1 { body1.getLinearVelocity() };
    const math::Vec2 savedPos2 { body2.getPosition() };
    const math::Vec2 savedVel2 { body2.getLinearVelocity() };

    // Step another 20 frames
    for (std::uint32_t f { 15U }; f < 35U; ++f) {
        body1.applyForce(math::Vec2 { -5.0, 0.0 });
        world.step(dt);
    }

    EXPECT_NE(body1.getPosition().y, savedPos1.y);

    // Restore to frame 15
    EXPECT_TRUE(world.restoreSnapshot(snapshot15));

    EXPECT_DOUBLE_EQ(body1.getPosition().x, savedPos1.x);
    EXPECT_DOUBLE_EQ(body1.getPosition().y, savedPos1.y);
    EXPECT_DOUBLE_EQ(body1.getLinearVelocity().x, savedVel1.x);
    EXPECT_DOUBLE_EQ(body1.getLinearVelocity().y, savedVel1.y);

    EXPECT_DOUBLE_EQ(body2.getPosition().x, savedPos2.x);
    EXPECT_DOUBLE_EQ(body2.getPosition().y, savedPos2.y);
    EXPECT_DOUBLE_EQ(body2.getLinearVelocity().x, savedVel2.x);
    EXPECT_DOUBLE_EQ(body2.getLinearVelocity().y, savedVel2.y);
}

TEST(SimulationSnapshotTest, CaptureAndRestoreJointWarmStarting)
{
    PhysicsWorld2D world { math::Vec2 { 0.0, -9.81 } };

    RigidBodyConfig cfgA {};
    cfgA.mass = 10.0;
    cfgA.position = math::Vec2 { 0.0, 5.0 };
    RigidBody2D bodyA { cfgA };
    world.addBody(bodyA);

    RigidBodyConfig cfgB {};
    cfgB.mass = 5.0;
    cfgB.position = math::Vec2 { 2.0, 5.0 };
    RigidBody2D bodyB { cfgB };
    world.addBody(bodyB);

    RevoluteJointConfig jcfg {};
    jcfg.bodyA = &bodyA;
    jcfg.bodyB = &bodyB;
    jcfg.localAnchorA = math::Vec2 { 1.0, 0.0 };
    jcfg.localAnchorB = math::Vec2 { -1.0, 0.0 };
    jcfg.enableLimit = true;
    jcfg.lowerAngle = -0.5;
    jcfg.upperAngle = 0.5;
    jcfg.enableMotor = true;
    jcfg.motorSpeed = 1.0;
    jcfg.maxMotorTorque = 50.0;

    RevoluteJoint2D joint { jcfg };
    world.addJoint(joint);

    constexpr double dt { 1.0 / 60.0 };
    for (std::uint32_t i { 0U }; i < 10U; ++i) {
        world.step(dt);
    }

    SimulationSnapshot2D snap10 {};
    EXPECT_TRUE(world.captureSnapshot(10U, snap10));
    EXPECT_EQ(snap10.jointCount, 1U);
    EXPECT_TRUE(snap10.isValid());

    const auto stateBefore { joint.captureState() };

    // Advance 10 more frames
    for (std::uint32_t i { 10U }; i < 20U; ++i) {
        world.step(dt);
    }

    // Restore to frame 10
    EXPECT_TRUE(world.restoreSnapshot(snap10));
    const auto stateAfter { joint.captureState() };

    EXPECT_DOUBLE_EQ(stateBefore.linearImpulse.x, stateAfter.linearImpulse.x);
    EXPECT_DOUBLE_EQ(stateBefore.linearImpulse.y, stateAfter.linearImpulse.y);
    EXPECT_DOUBLE_EQ(stateBefore.motorImpulse, stateAfter.motorImpulse);
    EXPECT_DOUBLE_EQ(stateBefore.limitImpulse, stateAfter.limitImpulse);
}

TEST(SimulationSnapshotTest, BitExactDeterminismRollbackAndReplay)
{
    // Continuous run
    PhysicsWorld2D worldA { math::Vec2 { 0.0, -9.81 } };
    RigidBodyConfig cfg {};
    cfg.mass = 3.0;
    cfg.inertia = 1.5;
    cfg.position = math::Vec2 { -2.0, 8.0 };
    RigidBody2D bodyA { cfg };
    worldA.addBody(bodyA);

    constexpr double dt { 1.0 / 60.0 };
    for (std::uint32_t f { 0U }; f < 40U; ++f) {
        const double forceX = std::sin(static_cast<double>(f) * 0.2) * 15.0;
        bodyA.applyForce(math::Vec2 { forceX, 0.0 });
        worldA.step(dt);
    }

    SimulationSnapshot2D snapEndA {};
    EXPECT_TRUE(worldA.captureSnapshot(40U, snapEndA));

    // Rewound run: run 20, capture, run to 35, rewind to 20, replay to 40
    PhysicsWorld2D worldB { math::Vec2 { 0.0, -9.81 } };
    RigidBody2D bodyB { cfg };
    worldB.addBody(bodyB);

    for (std::uint32_t f { 0U }; f < 20U; ++f) {
        const double forceX = std::sin(static_cast<double>(f) * 0.2) * 15.0;
        bodyB.applyForce(math::Vec2 { forceX, 0.0 });
        worldB.step(dt);
    }

    SimulationSnapshot2D snap20B {};
    EXPECT_TRUE(worldB.captureSnapshot(20U, snap20B));

    for (std::uint32_t f { 20U }; f < 35U; ++f) {
        const double forceX = std::sin(static_cast<double>(f) * 0.2) * 15.0;
        bodyB.applyForce(math::Vec2 { forceX, 0.0 });
        worldB.step(dt);
    }

    // Rewind back to 20 and replay using rollbackAndReplay
    const bool replayed = worldB.rollbackAndReplay(snap20B, 40U, dt, [&](std::uint32_t f) {
        const double forceX = std::sin(static_cast<double>(f) * 0.2) * 15.0;
        bodyB.applyForce(math::Vec2 { forceX, 0.0 });
    });
    EXPECT_TRUE(replayed);

    SimulationSnapshot2D snapEndB {};
    EXPECT_TRUE(worldB.captureSnapshot(40U, snapEndB));

    // Bit-exact verification: checksums and states must match 100%
    EXPECT_EQ(snapEndA.stateChecksum, snapEndB.stateChecksum);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().x, bodyB.getPosition().x);
    EXPECT_DOUBLE_EQ(bodyA.getPosition().y, bodyB.getPosition().y);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().x, bodyB.getLinearVelocity().x);
    EXPECT_DOUBLE_EQ(bodyA.getLinearVelocity().y, bodyB.getLinearVelocity().y);
}

TEST(SimulationSnapshotTest, InvalidChecksumRestoreFails)
{
    PhysicsWorld2D world { math::Vec2 { 0.0, -9.81 } };
    RigidBody2D body {};
    world.addBody(body);

    SimulationSnapshot2D snapshot {};
    EXPECT_TRUE(world.captureSnapshot(0U, snapshot));

    // Corrupt checksum
    snapshot.stateChecksum ^= 0xDEADBEEFULL;
    EXPECT_FALSE(world.restoreSnapshot(snapshot));
}
