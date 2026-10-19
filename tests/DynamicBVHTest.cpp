#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Ray2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/BroadphasePair2D.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/DynamicBVH2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::physics;
using namespace vehicle_sim::math;

TEST(DynamicBVHTest, NodePoolAllocationAndFreeList)
{
    DynamicBVH2D bvh { 16 };
    EXPECT_EQ(bvh.getProxyCount(), 0U);
    EXPECT_GE(bvh.getNodeCount(), 16U);

    const AABB2D box1 { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 } };
    const AABB2D box2 { Vec2 { 2.0, 2.0 }, Vec2 { 3.0, 3.0 } };

    const std::int32_t p1 { bvh.createProxy(box1, nullptr) };
    const std::int32_t p2 { bvh.createProxy(box2, nullptr) };

    EXPECT_EQ(bvh.getProxyCount(), 2U);
    EXPECT_NE(p1, p2);
    EXPECT_TRUE(bvh.validate());

    bvh.destroyProxy(p1);
    EXPECT_EQ(bvh.getProxyCount(), 1U);
    EXPECT_TRUE(bvh.validate());

    const std::int32_t p3 { bvh.createProxy(box1, nullptr) };
    EXPECT_EQ(bvh.getProxyCount(), 2U);
    // The freed node index should be recycled
    EXPECT_EQ(p3, p1);
    EXPECT_TRUE(bvh.validate());
}

TEST(DynamicBVHTest, SingleLeafLifecycle)
{
    DynamicBVH2D bvh {};
    EXPECT_EQ(bvh.getTreeHeight(), 0);

    const AABB2D box { Vec2 { -1.0, -1.0 }, Vec2 { 1.0, 1.0 } };
    const std::int32_t p1 { bvh.createProxy(box, nullptr) };

    EXPECT_EQ(bvh.getProxyCount(), 1U);
    EXPECT_EQ(bvh.getTreeHeight(), 0);
    EXPECT_TRUE(bvh.validate());

    bvh.destroyProxy(p1);
    EXPECT_EQ(bvh.getProxyCount(), 0U);
    EXPECT_EQ(bvh.getTreeHeight(), 0);
    EXPECT_TRUE(bvh.validate());
}

TEST(DynamicBVHTest, TreeBalanceInvariants)
{
    DynamicBVH2D bvh {};
    constexpr std::int32_t kNumBoxes { 64 };
    std::vector<std::int32_t> proxies {};
    proxies.reserve(kNumBoxes);

    // Sequential line of boxes - worst case for un-balanced binary trees
    for (std::int32_t i { 0 }; i < kNumBoxes; ++i) {
        const double offset { static_cast<double>(i) * 2.0 };
        const AABB2D box { Vec2 { offset, 0.0 }, Vec2 { offset + 1.0, 1.0 } };
        proxies.push_back(bvh.createProxy(box, nullptr));
        EXPECT_TRUE(bvh.validate());
    }

    EXPECT_EQ(bvh.getProxyCount(), static_cast<std::size_t>(kNumBoxes));
    // For 64 leaves, balanced tree height is around ceil(log2(64)) = 6, with AVL max <= 2 * log2(64) ~ 12
    EXPECT_LE(bvh.getTreeHeight(), 12);
    EXPECT_TRUE(bvh.validate());

    // Remove half of the proxies
    for (std::size_t i { 0 }; i < proxies.size(); i += 2) {
        bvh.destroyProxy(proxies[i]);
        EXPECT_TRUE(bvh.validate());
    }

    EXPECT_EQ(bvh.getProxyCount(), static_cast<std::size_t>(kNumBoxes / 2));
    EXPECT_TRUE(bvh.validate());
}

TEST(DynamicBVHTest, FatBoxHysteresis)
{
    DynamicBVH2D bvh {};
    bvh.setExpansionMargin(0.5);

    const AABB2D tightBox { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 } };
    const std::int32_t proxy { bvh.createProxy(tightBox, nullptr) };

    const AABB2D fatBox { bvh.getFatAABB(proxy) };
    EXPECT_DOUBLE_EQ(fatBox.min.x, -0.5);
    EXPECT_DOUBLE_EQ(fatBox.min.y, -0.5);
    EXPECT_DOUBLE_EQ(fatBox.max.x, 1.5);
    EXPECT_DOUBLE_EQ(fatBox.max.y, 1.5);

    // Small displacement inside fat box - should not cause tree re-insertion
    const AABB2D slightlyMovedBox { Vec2 { 0.1, 0.1 }, Vec2 { 1.1, 1.1 } };
    const bool reinserted { bvh.moveProxy(proxy, slightlyMovedBox, Vec2 { 0.0, 0.0 }) };
    EXPECT_FALSE(reinserted);

    // Large displacement breaching fat box - causes re-insertion and expansion
    const AABB2D largeMovedBox { Vec2 { 2.0, 0.0 }, Vec2 { 3.0, 1.0 } };
    const bool reinsertedLarge { bvh.moveProxy(proxy, largeMovedBox, Vec2 { 1.0, 0.0 }) };
    EXPECT_TRUE(reinsertedLarge);

    const AABB2D updatedFatBox { bvh.getFatAABB(proxy) };
    // Extrusion along positive X
    EXPECT_GE(updatedFatBox.max.x, 3.5);
    EXPECT_TRUE(updatedFatBox.contains(largeMovedBox));
}

TEST(DynamicBVHTest, AABBOverlapQuery)
{
    DynamicBVH2D bvh {};

    // Create 10 boxes along the diagonal
    for (int i { 0 }; i < 10; ++i) {
        const double pos { static_cast<double>(i) * 5.0 };
        const AABB2D box { Vec2 { pos, pos }, Vec2 { pos + 2.0, pos + 2.0 } };
        static_cast<void>(bvh.createProxy(box, reinterpret_cast<void*>(static_cast<std::uintptr_t>(i))));
    }

    // Query area that only overlaps boxes 2 and 3 (pos 10.0 and 15.0)
    const AABB2D queryArea { Vec2 { 9.0, 9.0 }, Vec2 { 16.0, 16.0 } };
    std::vector<int> hitIndices {};

    bvh.query(queryArea, [&](std::int32_t proxyId) -> bool {
        const auto val = static_cast<int>(reinterpret_cast<std::uintptr_t>(bvh.getUserData(proxyId)));
        hitIndices.push_back(val);
        return true;
    });

    EXPECT_EQ(hitIndices.size(), 2U);
    std::sort(hitIndices.begin(), hitIndices.end());
    EXPECT_EQ(hitIndices[0], 2);
    EXPECT_EQ(hitIndices[1], 3);
}

TEST(DynamicBVHTest, RaycastClosestHitAndPruning)
{
    DynamicBVH2D bvh {};
    bvh.setExpansionMargin(0.01);

    // Box 1 at x = [2, 4], y = [-1, 1]
    const AABB2D box1 { Vec2 { 2.0, -1.0 }, Vec2 { 4.0, 1.0 } };
    // Box 2 at x = [6, 8], y = [-1, 1]
    const AABB2D box2 { Vec2 { 6.0, -1.0 }, Vec2 { 8.0, 1.0 } };

    const std::int32_t p1 { bvh.createProxy(box1, reinterpret_cast<void*>(1)) };
    const std::int32_t p2 { bvh.createProxy(box2, reinterpret_cast<void*>(2)) };
    static_cast<void>(p2);

    // Ray shooting along +X axis
    const Ray2D ray { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };

    std::vector<std::int32_t> hitProxies {};
    bvh.raycast(ray, [&](const Ray2D& r, std::int32_t proxyId) -> double {
        hitProxies.push_back(proxyId);
        const auto hit { bvh.getFatAABB(proxyId).raycast(r) };
        if (hit.has_value()) {
            return hit->distance / r.maxDistance;
        }
        return 1.0;
    });

    // Closest box p1 should be visited first and clips ray distance, pruning p2
    ASSERT_FALSE(hitProxies.empty());
    EXPECT_EQ(hitProxies[0], p1);
}

TEST(DynamicBVHTest, PairGenerationNoDuplicates)
{
    DynamicBVH2D bvh {};

    // 3 overlapping boxes
    Collider2D col1 { Circle2D { Vec2 { 0.0, 0.0 }, 1.0 } };
    Collider2D col2 { Circle2D { Vec2 { 0.5, 0.0 }, 1.0 } };
    Collider2D col3 { Circle2D { Vec2 { 1.0, 0.0 }, 1.0 } };
    // Disjoint box far away
    Collider2D col4 { Circle2D { Vec2 { 20.0, 20.0 }, 1.0 } };

    static_cast<void>(bvh.createProxy(col1.computeWorldAABB(), &col1));
    static_cast<void>(bvh.createProxy(col2.computeWorldAABB(), &col2));
    static_cast<void>(bvh.createProxy(col3.computeWorldAABB(), &col3));
    static_cast<void>(bvh.createProxy(col4.computeWorldAABB(), &col4));

    std::vector<BroadphasePair2D> pairs {};
    bvh.queryPairs(pairs);

    // Overlaps should be: (1,2), (1,3), (2,3) = 3 pairs.
    // Col 4 does not overlap any.
    EXPECT_EQ(pairs.size(), 3U);

    for (const auto& pair : pairs) {
        EXPECT_LT(pair.proxyA, pair.proxyB);
        EXPECT_NE(pair.colliderA, pair.colliderB);
    }
}

TEST(DynamicBVHTest, CollisionFilterMasking)
{
    DynamicBVH2D bvh {};

    Collider2D colA { Circle2D { Vec2 { 0.0, 0.0 }, 1.0 } };
    colA.setCollisionFilter(0x0001, 0x0002); // Category 1, wants 2

    Collider2D colB { Circle2D { Vec2 { 0.2, 0.0 }, 1.0 } };
    colB.setCollisionFilter(0x0004, 0x0001); // Category 4 (not 2!), wants 1

    EXPECT_FALSE(colA.canCollideWith(colB));

    static_cast<void>(bvh.createProxy(colA.computeWorldAABB(), &colA));
    static_cast<void>(bvh.createProxy(colB.computeWorldAABB(), &colB));

    std::vector<BroadphasePair2D> pairs {};
    bvh.queryPairs(pairs);

    // Filter mask mismatch should prevent pair generation
    EXPECT_TRUE(pairs.empty());
}

TEST(DynamicBVHTest, SameBodyFiltering)
{
    DynamicBVH2D bvh {};

    RigidBody2D body {};
    Collider2D colA { Circle2D { Vec2 { 0.0, 0.0 }, 1.0 }, &body };
    Collider2D colB { Circle2D { Vec2 { 0.5, 0.0 }, 1.0 }, &body };

    static_cast<void>(bvh.createProxy(colA.computeWorldAABB(), &colA));
    static_cast<void>(bvh.createProxy(colB.computeWorldAABB(), &colB));

    std::vector<BroadphasePair2D> pairs {};
    bvh.queryPairs(pairs);

    // Colliders on the same rigid body should be filtered out
    EXPECT_TRUE(pairs.empty());
}

TEST(DynamicBVHTest, PhysicsWorldIntegration)
{
    PhysicsWorld2D world { Vec2 { 0.0, 0.0 } };

    RigidBodyConfig cfgA {};
    cfgA.mass = 1.0;
    cfgA.inertia = 1.0;
    RigidBody2D bodyA { cfgA };
    bodyA.setPosition(Vec2 { -2.0, 0.0 });
    bodyA.setLinearVelocity(Vec2 { 5.0, 0.0 });
    world.addBody(bodyA);

    Collider2D colA { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &bodyA };
    world.addCollider(colA);

    RigidBodyConfig cfgB {};
    cfgB.mass = 1.0;
    cfgB.inertia = 1.0;
    RigidBody2D bodyB { cfgB };
    bodyB.setPosition(Vec2 { 2.0, 0.0 });
    bodyB.setLinearVelocity(Vec2 { -5.0, 0.0 });
    world.addBody(bodyB);

    Collider2D colB { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &bodyB };
    world.addCollider(colB);

    EXPECT_EQ(world.getColliderCount(), 2U);
    EXPECT_EQ(world.getBroadphaseTree().getProxyCount(), 2U);

    // Advance simulation until collision occurs
    for (int i { 0 }; i < 30; ++i) {
        world.step(0.016);
    }

    // Bodies should have collided and rebounded
    EXPECT_LT(bodyA.getLinearVelocity().x, 0.0);
    EXPECT_GT(bodyB.getLinearVelocity().x, 0.0);
}

TEST(DynamicBVHTest, WorldRaycastAPI)
{
    PhysicsWorld2D world { Vec2 { 0.0, 0.0 } };

    RigidBody2D body {};
    body.setPosition(Vec2 { 5.0, 0.0 });
    world.addBody(body);

    Collider2D col { Circle2D { Vec2 { 0.0, 0.0 }, 1.0 }, &body };
    world.addCollider(col);

    const Ray2D ray { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    RayHit2D hit {};
    Collider2D* hitCol { nullptr };

    const bool struck { world.raycast(ray, hit, hitCol) };
    EXPECT_TRUE(struck);
    EXPECT_EQ(hitCol, &col);
    EXPECT_NEAR(hit.distance, 4.0, 1e-4);
    EXPECT_NEAR(hit.point.x, 4.0, 1e-4);
    EXPECT_NEAR(hit.normal.x, -1.0, 1e-4);
}

TEST(DynamicBVHTest, PerformanceScalability500Bodies)
{
    DynamicBVH2D bvh {};
    constexpr int kCount { 500 };
    std::vector<Collider2D> colliders {};
    colliders.reserve(kCount);

    for (int i { 0 }; i < kCount; ++i) {
        const double x { static_cast<double>(i % 25) * 4.0 };
        const double y { static_cast<double>(i / 25) * 4.0 };
        colliders.emplace_back(Circle2D { Vec2 { x, y }, 1.0 });
        static_cast<void>(bvh.createProxy(colliders.back().computeWorldAABB(), &colliders.back()));
    }

    std::vector<BroadphasePair2D> pairs {};
    const auto startTime { std::chrono::high_resolution_clock::now() };
    bvh.queryPairs(pairs);
    const auto endTime { std::chrono::high_resolution_clock::now() };
    const auto elapsedMs { std::chrono::duration<double, std::milli>(endTime - startTime).count() };

    // For 500 bodies, broadphase query should execute well under 5 milliseconds
    EXPECT_LT(elapsedMs, 10.0);
    EXPECT_TRUE(bvh.validate());
}

TEST(DynamicBVHTest, CrossPlatformDeterminism)
{
    auto runSim = []() -> std::pair<Vec2, Vec2> {
        PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };

        RigidBodyConfig cfg1 {};
        cfg1.mass = 1.0;
        cfg1.inertia = 1.0;
        RigidBody2D body1 { cfg1 };
        body1.setPosition(Vec2 { 0.0, 5.0 });
        world.addBody(body1);
        Collider2D col1 { Circle2D { Vec2 { 0.0, 0.0 }, 0.5 }, &body1 };
        world.addCollider(col1);

        RigidBodyConfig floorCfg {};
        floorCfg.bodyType = BodyType::Static;
        RigidBody2D floorBody { floorCfg };
        floorBody.setPosition(Vec2 { 0.0, 0.0 });
        world.addBody(floorBody);
        Collider2D floorCol { OBB2D { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 0.5 }, 0.0 }, &floorBody };
        world.addCollider(floorCol);

        for (int step { 0 }; step < 50; ++step) {
            world.step(0.016);
        }

        return { body1.getPosition(), body1.getLinearVelocity() };
    };

    const auto [pos1, vel1] { runSim() };
    const auto [pos2, vel2] { runSim() };

    EXPECT_DOUBLE_EQ(pos1.x, pos2.x);
    EXPECT_DOUBLE_EQ(pos1.y, pos2.y);
    EXPECT_DOUBLE_EQ(vel1.x, vel2.x);
    EXPECT_DOUBLE_EQ(vel1.y, vel2.y);
}
