#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Capsule2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/CompoundShape2D.h"
#include "libs/VehicleSimCore/physics/ContactManifold2D.h"
#include "libs/VehicleSimCore/physics/Narrowphase2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Capsule2D;
using vehicle_sim::math::Circle2D;
using vehicle_sim::math::OBB2D;
using vehicle_sim::math::Polygon2D;
using vehicle_sim::math::Vec2;
using vehicle_sim::physics::BodyType;
using vehicle_sim::physics::Collider2D;
using vehicle_sim::physics::ColliderMaterial;
using vehicle_sim::physics::CompoundShape2D;
using vehicle_sim::physics::ContactManifold2D;
using vehicle_sim::physics::Narrowphase2D;
using vehicle_sim::physics::PhysicsWorld2D;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;
using vehicle_sim::terrain::Heightfield1D;
using vehicle_sim::terrain::HeightSample1D;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kEps { 1e-6 };

// -----------------------------------------------------------------------------
// Circle vs Circle Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, CircleCircleSeparated)
{
    const Circle2D a { Vec2 { 0.0, 0.0 }, 1.0 };
    const Circle2D b { Vec2 { 3.0, 0.0 }, 1.0 };
    const ColliderMaterial mat { 0.5, 0.5 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleCircle(a, b, mat, mat, manifold) };
    EXPECT_FALSE(hit);
    EXPECT_EQ(manifold.contactCount, 0U);
}

TEST(NarrowphaseTest, CircleCirclePenetrating)
{
    const Circle2D a { Vec2 { 0.0, 0.0 }, 1.0 };
    const Circle2D b { Vec2 { 1.5, 0.0 }, 1.0 };
    const ColliderMaterial matA { 0.4, 0.6 };
    const ColliderMaterial matB { 0.9, 0.6 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleCircle(a, b, matA, matB, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);

    const auto& c = manifold.contacts[0];
    EXPECT_NEAR(c.penetration, 0.5, kEps);
    EXPECT_NEAR(c.normal.x, 1.0, kEps); // Points A -> B
    EXPECT_NEAR(c.normal.y, 0.0, kEps);
    EXPECT_NEAR(c.point.x, 0.75, kEps);
    EXPECT_NEAR(c.point.y, 0.0, kEps);
    EXPECT_NEAR(c.friction, 0.6, kEps);
    EXPECT_NEAR(c.restitution, std::sqrt(0.36), kEps);
}

TEST(NarrowphaseTest, CircleCircleConcentric)
{
    const Circle2D a { Vec2 { 2.0, 2.0 }, 1.0 };
    const Circle2D b { Vec2 { 2.0, 2.0 }, 0.5 };
    const ColliderMaterial mat { 0.0, 0.0 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleCircle(a, b, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);
    EXPECT_NEAR(manifold.contacts[0].penetration, 1.5, kEps);
    EXPECT_FALSE(std::isnan(manifold.contacts[0].normal.x));
    EXPECT_FALSE(std::isnan(manifold.contacts[0].normal.y));
}

// -----------------------------------------------------------------------------
// Circle vs OBB Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, CircleOBBEdgeContact)
{
    const Circle2D circle { Vec2 { -2.0, 0.0 }, 1.0 };
    const OBB2D obb { Vec2 { 0.0, 0.0 }, Vec2 { 1.5, 1.0 }, 0.0 };
    const ColliderMaterial mat { 0.2, 0.4 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleOBB(circle, obb, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);

    const auto& c = manifold.contacts[0];
    EXPECT_NEAR(c.penetration, 0.5, kEps);
    EXPECT_NEAR(c.normal.x, 1.0, kEps); // Points from circle to box (+X)
    EXPECT_NEAR(c.normal.y, 0.0, kEps);
    EXPECT_NEAR(c.point.x, -1.5, kEps);
    EXPECT_NEAR(c.point.y, 0.0, kEps);
}

TEST(NarrowphaseTest, CircleOBBCornerContact)
{
    const Circle2D circle { Vec2 { 1.5, 1.5 }, 1.0 };
    const OBB2D obb { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0 };
    const ColliderMaterial mat { 0.2, 0.4 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleOBB(circle, obb, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);

    const auto& c = manifold.contacts[0];
    const double expectedDist { std::sqrt(0.5) };
    EXPECT_NEAR(c.penetration, 1.0 - expectedDist, kEps);
    EXPECT_LT(c.normal.x, 0.0); // Points towards box center (-X, -Y)
    EXPECT_LT(c.normal.y, 0.0);
}

TEST(NarrowphaseTest, CircleOBBDeepPenetration)
{
    // Circle center inside box
    const Circle2D circle { Vec2 { 0.8, 0.0 }, 0.5 };
    const OBB2D obb { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0 };
    const ColliderMaterial mat { 0.2, 0.4 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCircleOBB(circle, obb, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);
    EXPECT_GT(manifold.contacts[0].penetration, 0.5);
}

// -----------------------------------------------------------------------------
// Capsule Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, CapsuleCircleContact)
{
    const Capsule2D cap { Vec2 { -2.0, 0.0 }, Vec2 { 2.0, 0.0 }, 0.5 };
    const Circle2D circ { Vec2 { 0.0, 0.8 }, 0.5 };
    const ColliderMaterial mat { 0.3, 0.5 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectCapsuleCircle(cap, circ, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);
    EXPECT_NEAR(manifold.contacts[0].penetration, 0.2, kEps);
}

TEST(NarrowphaseTest, CapsuleCapsuleParallelAndCrossed)
{
    // Parallel capsules
    const Capsule2D capA { Vec2 { -2.0, 0.0 }, Vec2 { 2.0, 0.0 }, 0.5 };
    const Capsule2D capB { Vec2 { -2.0, 0.8 }, Vec2 { 2.0, 0.8 }, 0.5 };
    const ColliderMaterial mat { 0.0, 0.0 };
    ContactManifold2D manifold {};

    EXPECT_TRUE(Narrowphase2D::detectCapsuleCapsule(capA, capB, mat, mat, manifold));

    // Perpendicular crossed capsules
    const Capsule2D capCross { Vec2 { 0.0, -2.0 }, Vec2 { 0.0, 2.0 }, 0.5 };
    ContactManifold2D crossManifold {};
    EXPECT_TRUE(Narrowphase2D::detectCapsuleCapsule(capA, capCross, mat, mat, crossManifold));
}

// -----------------------------------------------------------------------------
// OBB vs OBB SAT & Manifold Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, OBBOBBFaceToFaceGeneratesTwoPoints)
{
    // Box A resting directly on Box B
    const OBB2D boxA { Vec2 { 0.0, 1.8 }, Vec2 { 2.0, 1.0 }, 0.0 };
    const OBB2D boxB { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 1.0 }, 0.0 };
    const ColliderMaterial mat { 0.0, 0.8 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectOBBOBB(boxA, boxB, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    // Face-to-face collision generates 2 distinct contact points for rotational stability!
    EXPECT_EQ(manifold.contactCount, 2U);

    for (std::size_t i { 0 }; i < manifold.contactCount; ++i) {
        EXPECT_NEAR(manifold.contacts[i].penetration, 0.2, kEps);
        EXPECT_NEAR(manifold.contacts[i].normal.y, -1.0, kEps); // Points from A downward to B
    }
}

TEST(NarrowphaseTest, OBBOBBCornerToFaceGeneratesContact)
{
    // Box A rotated 45 degrees landing corner-first onto flat Box B
    const OBB2D boxA { Vec2 { 0.0, 2.0 }, Vec2 { 1.0, 1.0 }, kPi * 0.25 };
    const OBB2D boxB { Vec2 { 0.0, 0.0 }, Vec2 { 3.0, 1.0 }, 0.0 };
    const ColliderMaterial mat { 0.2, 0.5 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectOBBOBB(boxA, boxB, mat, mat, manifold) };
    ASSERT_TRUE(hit);
    EXPECT_GE(manifold.contactCount, 1U);
}

// -----------------------------------------------------------------------------
// Polygon & Ellipse Approximation Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, PolygonPolygonEllipseApproximations)
{
    // Two 8-gon ellipse approximations overlapping
    const Polygon2D elA { Polygon2D::fromEllipse(Vec2 { 0.0, 0.0 }, 2.0, 1.0, 0.0, 8) };
    const Polygon2D elB { Polygon2D::fromEllipse(Vec2 { 3.0, 0.0 }, 2.0, 1.0, 0.0, 8) };
    const ColliderMaterial mat { 0.1, 0.5 };
    ContactManifold2D manifold {};

    const bool hit { Narrowphase2D::detectPolygonPolygon(elA, elB, mat, mat, manifold) };
    EXPECT_TRUE(hit);
    EXPECT_GE(manifold.contactCount, 1U);
}

// -----------------------------------------------------------------------------
// Compound Shape Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, CompoundShapeMultiBoxCollision)
{
    // Compound shape representing truck cab + bed
    CompoundShape2D truckHull {};
    truckHull.addChild(OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 1.5, 0.5 }, 0.0 }, math::Transform2D::identity());
    truckHull.addChild(OBB2D { Vec2 { 1.0, 0.8 }, Vec2 { 0.5, 0.5 }, 0.0 }, math::Transform2D::identity());

    RigidBody2D bodyTruck {};
    bodyTruck.setPosition(Vec2 { 0.0, 0.0 });
    const Collider2D truckCol { truckHull, &bodyTruck };

    // Obstacle box hitting the cab
    RigidBody2D bodyObs {};
    bodyObs.setPosition(Vec2 { 1.0, 1.5 });
    const Collider2D obsCol { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 0.5, 0.5 }, 0.0 }, &bodyObs };

    ContactManifold2D manifold {};
    const bool hit { Narrowphase2D::detectColliders(truckCol, obsCol, manifold) };
    EXPECT_TRUE(hit);
    EXPECT_GE(manifold.contactCount, 1U);
}

// -----------------------------------------------------------------------------
// Terrain Narrowphase Tests
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, TerrainCircleResting)
{
    Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);

    RigidBody2D body {};
    body.setPosition(Vec2 { 5.0, 0.8 }); // Penetrates y = 0.0 by 0.2m (radius = 1.0)
    const Collider2D circleCol { Circle2D { Vec2 { 0.0, 0.0 }, 1.0 }, &body };

    ContactManifold2D manifold {};
    const bool hit { Narrowphase2D::detectTerrain(circleCol, terrain, manifold) };
    ASSERT_TRUE(hit);
    ASSERT_EQ(manifold.contactCount, 1U);

    EXPECT_NEAR(manifold.contacts[0].penetration, 0.2, kEps);
    EXPECT_NEAR(manifold.contacts[0].normal.y, -1.0, kEps); // Normal points toward ground barrier
}

TEST(NarrowphaseTest, TerrainOBBTwoPointsResting)
{
    Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);

    RigidBody2D body {};
    body.setPosition(Vec2 { 0.0, 0.8 }); // Lower edge is at y = 0.8 - 1.0 = -0.2m
    const Collider2D boxCol { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 1.0 }, 0.0 }, &body };

    ContactManifold2D manifold {};
    const bool hit { Narrowphase2D::detectTerrain(boxCol, terrain, manifold) };
    ASSERT_TRUE(hit);
    // Flat box bottom vertices generate 2 contact points with terrain
    EXPECT_EQ(manifold.contactCount, 2U);

    for (std::size_t i { 0 }; i < manifold.contactCount; ++i) {
        EXPECT_NEAR(manifold.contacts[i].penetration, 0.2, kEps);
    }
}

// -----------------------------------------------------------------------------
// PhysicsWorld2D Pipeline Integration Test
// -----------------------------------------------------------------------------

TEST(NarrowphaseTest, PhysicsWorldAutomatedCollisionBounce)
{
    PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

    // Static ground box at y = 0
    RigidBodyConfig staticCfg {};
    staticCfg.bodyType = BodyType::Static;
    staticCfg.position = Vec2 { 0.0, 0.0 };
    RigidBody2D floorBody { staticCfg };
    Collider2D floorCol { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 0.5 }, 0.0 }, &floorBody };
    floorCol.setMaterial(ColliderMaterial { 0.8, 0.5 }); // High elasticity

    // Dynamic ball falling from above
    RigidBodyConfig dynCfg {};
    dynCfg.bodyType = BodyType::Dynamic;
    dynCfg.mass = 5.0;
    dynCfg.position = Vec2 { 0.0, 0.95 }; // Sits penetrating floor top (0.5m) by 0.05m
    dynCfg.inertia = 1.0;
    RigidBody2D ballBody { dynCfg };
    ballBody.setLinearVelocity(Vec2 { 0.0, -5.0 }); // Rapid downward velocity
    Collider2D ballCol { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &ballBody };
    ballCol.setMaterial(ColliderMaterial { 0.8, 0.5 });

    world.addBody(floorBody);
    world.addBody(ballBody);
    world.addCollider(floorCol);
    world.addCollider(ballCol);

    EXPECT_EQ(world.getColliderCount(), 2U);

    // Step world simulation
    world.step(0.016);

    // Ball should have collided with the floor and reversed vertical velocity upward
    EXPECT_GT(ballBody.getLinearVelocity().y, 0.0);
}

} // namespace vehicle_sim::testing
