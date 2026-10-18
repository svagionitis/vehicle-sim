#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/SweptShape2D.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/ContactSolver2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/physics/SpeculativeContact2D.h"
#include "libs/VehicleSimCore/physics/TOISolver2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

using namespace vehicle_sim::math;
using namespace vehicle_sim::physics;
using namespace vehicle_sim::terrain;

// ============================================================================
// Phase 1: Analytical Swept Shapes Tests
// ============================================================================

TEST(ContinuousCollisionDetectionTest, SolveQuadraticRoots)
{
    double root { 0.0 };
    // (t - 2)(t - 5) = t^2 - 7t + 10 = 0 -> roots 2.0 and 5.0 (earliest 2.0)
    EXPECT_TRUE(SweptShape2D::solveQuadratic(1.0, -7.0, 10.0, root));
    EXPECT_NEAR(root, 2.0, 1e-9);

    // Negative discriminant: t^2 + 1 = 0
    EXPECT_FALSE(SweptShape2D::solveQuadratic(1.0, 0.0, 1.0, root));

    // Both negative roots: (t + 2)(t + 5) = t^2 + 7t + 10 = 0
    EXPECT_FALSE(SweptShape2D::solveQuadratic(1.0, 7.0, 10.0, root));

    // Linear degenerate: 2t - 6 = 0 -> t = 3
    EXPECT_TRUE(SweptShape2D::solveQuadratic(0.0, 2.0, -6.0, root));
    EXPECT_NEAR(root, 3.0, 1e-9);
}

TEST(ContinuousCollisionDetectionTest, SweptCircleCircleDirectHit)
{
    // Circle A at (0, 0) r = 0.5 moving right at 10 m/s
    // Circle B at (10, 0) r = 0.5 stationary
    // Touch occurs when distance between centers = 1.0 -> dist to travel = 9.0 -> t = 0.9s
    const Circle2D a { Vec2 { 0.0, 0.0 }, 0.5 };
    const Vec2 velA { 10.0, 0.0 };
    const Circle2D b { Vec2 { 10.0, 0.0 }, 0.5 };
    const Vec2 velB { 0.0, 0.0 };

    double tHit { 0.0 };
    Vec2 nHit { 0.0, 1.0 };
    Vec2 ptHit { 0.0, 0.0 };

    EXPECT_TRUE(SweptShape2D::solveSweptCircleCircle(a, velA, b, velB, 1.0, tHit, nHit, ptHit));
    EXPECT_NEAR(tHit, 0.9, 1e-6);
    EXPECT_NEAR(nHit.x, 1.0, 1e-6);
    EXPECT_NEAR(nHit.y, 0.0, 1e-6);
    EXPECT_NEAR(ptHit.x, 9.5, 1e-6);
    EXPECT_NEAR(ptHit.y, 0.0, 1e-6);
}

TEST(ContinuousCollisionDetectionTest, SweptCircleCircleMissing)
{
    // Circle A moving along y = 2.0, Circle B at y = 0.0, radii 0.5 (min dist = 2.0 > 1.0)
    const Circle2D a { Vec2 { 0.0, 2.0 }, 0.5 };
    const Vec2 velA { 10.0, 0.0 };
    const Circle2D b { Vec2 { 5.0, 0.0 }, 0.5 };
    const Vec2 velB { 0.0, 0.0 };

    double tHit { 0.0 };
    Vec2 nHit { 0.0, 1.0 };
    Vec2 ptHit { 0.0, 0.0 };

    EXPECT_FALSE(SweptShape2D::solveSweptCircleCircle(a, velA, b, velB, 1.0, tHit, nHit, ptHit));
}

TEST(ContinuousCollisionDetectionTest, SweptCircleSegmentImpact)
{
    // Circle at (0, 5) r = 0.5 moving downward at -10 m/s
    // Segment from (-5, 0) to (5, 0)
    // Contact when circle center y = 0.5 -> dist = 4.5 -> t = 0.45s
    const Circle2D circ { Vec2 { 0.0, 5.0 }, 0.5 };
    const Vec2 vel { 0.0, -10.0 };
    const Vec2 p1 { -5.0, 0.0 };
    const Vec2 p2 { 5.0, 0.0 };

    double tHit { 0.0 };
    Vec2 nHit { 0.0, 1.0 };
    Vec2 ptHit { 0.0, 0.0 };

    EXPECT_TRUE(SweptShape2D::solveSweptCircleSegment(circ, vel, p1, p2, 1.0, tHit, nHit, ptHit));
    EXPECT_NEAR(tHit, 0.45, 1e-6);
    EXPECT_NEAR(ptHit.x, 0.0, 1e-6);
    EXPECT_NEAR(ptHit.y, 0.0, 1e-6);
    EXPECT_NEAR(nHit.y, -1.0, 1e-6);
}

TEST(ContinuousCollisionDetectionTest, SweptCircleOBBImpact)
{
    // Circle at (0, 0) r = 0.5 moving right at 20 m/s
    // OBB at (10, 0) extents (1.0, 2.0), unrotated
    // Box left face is at x = 9.0. Touch when circle reaches x = 8.5 -> dist = 8.5 -> t = 8.5 / 20 = 0.425s
    const Circle2D circ { Vec2 { 0.0, 0.0 }, 0.5 };
    const Vec2 velCirc { 20.0, 0.0 };
    const OBB2D obb { Vec2 { 10.0, 0.0 }, Vec2 { 1.0, 2.0 }, 0.0 };
    const Vec2 velOBB { 0.0, 0.0 };

    double tHit { 0.0 };
    Vec2 nHit { 0.0, 1.0 };
    Vec2 ptHit { 0.0, 0.0 };

    EXPECT_TRUE(SweptShape2D::solveSweptCircleOBB(circ, velCirc, obb, velOBB, 1.0, tHit, nHit, ptHit));
    EXPECT_NEAR(tHit, 0.425, 1e-6);
    EXPECT_NEAR(ptHit.x, 9.0, 1e-6);
    EXPECT_NEAR(nHit.x, 1.0, 1e-6);
}

// ============================================================================
// Phase 2: TOISolver2D Tests
// ============================================================================

TEST(ContinuousCollisionDetectionTest, TOISolverCircleCircle)
{
    const Circle2D cA { Vec2 { 0.0, 0.0 }, 0.5 };
    const Circle2D cB { Vec2 { 10.0, 0.0 }, 0.5 };
    const Vec2 vA { 10.0, 0.0 };
    const Vec2 vB { 0.0, 0.0 };

    TOIInput input {};
    input.dt = 1.0;

    const TOIOutput out { TOISolver2D::solveCircleCircleTOI(cA, vA, cB, vB, input) };
    EXPECT_EQ(out.state, TOIState::Touching);
    EXPECT_NEAR(out.time, 0.9, 1e-6);
}

TEST(ContinuousCollisionDetectionTest, TOISolverOBBOBBConservativeAdvancement)
{
    // Box A at (0, 0) extents (1, 1) moving right at 10 m/s
    // Box B at (10, 0) extents (1, 1) stationary
    // Separation = 10 - 2 = 8m. Speed = 10 m/s. Hit at t = 0.8s -> normalized time = 0.8 / dt (dt = 1.0)
    const OBB2D bA { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0 };
    const OBB2D bB { Vec2 { 10.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0 };

    TOIInput input {};
    input.dt = 1.0;
    input.tolerance = 1e-3;

    const TOIOutput out { TOISolver2D::solveOBBOBBTOI(
        bA, Vec2 { 10.0, 0.0 }, 0.0, bB, Vec2::zero(), 0.0, input) };
    EXPECT_EQ(out.state, TOIState::Touching);
    EXPECT_NEAR(out.time, 0.8, 0.02);
}

// ============================================================================
// Phase 3: Speculative Contacts Tests
// ============================================================================

TEST(ContinuousCollisionDetectionTest, SpeculativeContactsGenerationAndBraking)
{
    // Setup two bodies: bodyA at (0, 0) moving towards static bodyB at (2.0, 0)
    // dt = 0.1s. Velocity of bodyA = (20.0, 0.0) -> would travel 2.0m and penetrate
    // Radii: 0.5 each -> separation distance = 2.0 - 1.0 = 1.0m
    // Speculative contact generated with target velocity = -1.0 / 0.1 = -10.0 m/s
    RigidBodyConfig cfgA {};
    cfgA.position = Vec2 { 0.0, 0.0 };
    cfgA.mass = 10.0;
    RigidBody2D bodyA { cfgA };
    bodyA.setLinearVelocity(Vec2 { 20.0, 0.0 });
    Collider2D colA { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &bodyA };

    RigidBodyConfig cfgB {};
    cfgB.bodyType = BodyType::Static;
    cfgB.position = Vec2 { 2.0, 0.0 };
    RigidBody2D bodyB { cfgB };
    Collider2D colB { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &bodyB };

    SpeculativeSettings settings {};
    settings.maxDistance = 1.5;

    ContactManifold2D manifold {};
    const bool generated { SpeculativeContact2D::generateContacts(colA, colB, 0.1, settings, manifold) };
    EXPECT_TRUE(generated);
    EXPECT_EQ(manifold.contactCount, 1u);
    EXPECT_TRUE(manifold.contacts[0].isSpeculative);
    EXPECT_NEAR(manifold.contacts[0].targetVelocity, -10.0, 1e-4);

    // Apply ContactSolver2D resolution: must decelerate bodyA from 20 m/s to target 10 m/s
    const bool resolved { ContactSolver2D::resolve(bodyA, &bodyB, manifold.contacts[0]) };
    EXPECT_TRUE(resolved);

    // Relative velocity along normal after resolution must be clamped to targetVelocity (-10.0)
    // Because bodyB is static (vB = 0), vA.x after impulse should be braked to exactly 10.0 m/s
    EXPECT_NEAR(bodyA.getLinearVelocity().x, 10.0, 1e-4);
}

TEST(ContinuousCollisionDetectionTest, SpeculativeGhostCollisionAvoidance)
{
    // Body moving tangentially parallel to a surface
    // Body at (0.0, 2.0) moving right at (20.0, 0.0)
    // Wall surface is horizontal at y = 1.2 (separation = 0.3m, within maxDistance)
    // Because motion is purely tangential (approach angle ~ 90 deg), speculative contact must be rejected!
    RigidBodyConfig cfgA {};
    cfgA.position = Vec2 { 0.0, 2.0 };
    RigidBody2D bodyA { cfgA };
    bodyA.setLinearVelocity(Vec2 { 20.0, 0.0 });
    Collider2D colA { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &bodyA };

    RigidBodyConfig cfgB {};
    cfgB.bodyType = BodyType::Static;
    cfgB.position = Vec2 { 0.0, 0.0 };
    RigidBody2D bodyB { cfgB };
    Collider2D colB { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 1.2 }, 0.0 }, &bodyB };

    SpeculativeSettings settings {};
    settings.maxDistance = 1.0;
    settings.enableVoronoiClip = true;

    ContactManifold2D manifold {};
    const bool generated { SpeculativeContact2D::generateContacts(colA, colB, 0.1, settings, manifold) };
    EXPECT_FALSE(generated); // Ghost collision successfully avoided!
}

// ============================================================================
// Phase 4: Full PhysicsWorld2D Anti-Tunneling Integration Tests
// ============================================================================

TEST(ContinuousCollisionDetectionTest, ExtremeSpeedThinWallTunnelingPrevention)
{
    // Bullet projectile moving at 200 m/s towards a 0.05m thin wall
    // Without CCD, at 60Hz step (dt = 0.0167s), body moves 3.33m and completely bypasses wall!
    PhysicsWorld2D world { Vec2::zero() };
    world.setCCDEnabled(true);

    // Thin wall at x = 5.0m, half extents (0.025, 5.0) -> thickness 0.05m
    RigidBodyConfig wallCfg {};
    wallCfg.bodyType = BodyType::Static;
    wallCfg.position = Vec2 { 5.0, 0.0 };
    RigidBody2D wallBody { wallCfg };
    Collider2D wallCol { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 0.025, 5.0 }, 0.0 }, &wallBody };
    world.addBody(wallBody);
    world.addCollider(wallCol);

    // Bullet body at x = 3.0m, r = 0.25m, velocity = (200.0, 0.0) m/s
    RigidBodyConfig bulletCfg {};
    bulletCfg.bodyType = BodyType::Dynamic;
    bulletCfg.position = Vec2 { 3.0, 0.0 };
    bulletCfg.isBullet = true;
    RigidBody2D bulletBody { bulletCfg };
    bulletBody.setLinearVelocity(Vec2 { 200.0, 0.0 });
    Collider2D bulletCol { Circle2D { Vec2 { 0.0, 0.0 }, 0.25 }, &bulletBody };
    bulletCol.setCCDEnabled(true);
    world.addBody(bulletBody);
    world.addCollider(bulletCol);

    // Single step of 1/60 second
    world.step(1.0 / 60.0);

    // Wall front boundary is at 5.0 - 0.025 = 4.975m
    // Bullet center must NOT pass 4.975 - 0.25 = 4.725m!
    EXPECT_LE(bulletBody.getPosition().x, 4.725 + 1e-2);
    // Linear velocity must be halted or reflected
    EXPECT_LE(bulletBody.getLinearVelocity().x, 0.0);
}

TEST(ContinuousCollisionDetectionTest, HighSpeedTerrainImpactCCD)
{
    // Fast vehicle diving into 1D continuous terrain at high vertical speed (-120 m/s)
    PhysicsWorld2D world { Vec2::zero() };
    world.setCCDEnabled(true);

    Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    terrain.enableNoise(false);
    world.setTerrain(&terrain);

    RigidBodyConfig bulletCfg {};
    bulletCfg.bodyType = BodyType::Dynamic;
    bulletCfg.position = Vec2 { 0.0, 1.5 };
    bulletCfg.isBullet = true;
    RigidBody2D bulletBody { bulletCfg };
    bulletBody.setLinearVelocity(Vec2 { 0.0, -120.0 });
    Collider2D bulletCol { Circle2D { Vec2 { 0.0, 0.0 }, 0.25 }, &bulletBody };
    bulletCol.setCCDEnabled(true);
    world.addBody(bulletBody);
    world.addCollider(bulletCol);

    // Step 1/60s (displacement would be -2.0m, pushing center to -0.5m without CCD)
    world.step(1.0 / 60.0);

    // Ground elevation is 0.0. With radius 0.25, center must not penetrate deep into ground
    EXPECT_GE(bulletBody.getPosition().y, 0.25 - 1e-2);
}

TEST(ContinuousCollisionDetectionTest, WorldTogglesAndConfiguration)
{
    PhysicsWorld2D world {};
    EXPECT_TRUE(world.isCCDEnabled());
    EXPECT_TRUE(world.isSpeculativeEnabled());
    EXPECT_EQ(world.getMaxCCDSubsteps(), 3u);

    world.setCCDEnabled(false);
    EXPECT_FALSE(world.isCCDEnabled());

    world.setSpeculativeEnabled(false);
    EXPECT_FALSE(world.isSpeculativeEnabled());

    world.setMaxCCDSubsteps(5);
    EXPECT_EQ(world.getMaxCCDSubsteps(), 5u);

    SpeculativeSettings settings {};
    settings.maxDistance = 0.8;
    world.setSpeculativeSettings(settings);
    EXPECT_NEAR(world.getSpeculativeSettings().maxDistance, 0.8, 1e-6);
}
