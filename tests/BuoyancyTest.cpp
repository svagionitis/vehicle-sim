#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/Buoyancy2D.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/HullSlicer2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using namespace vehicle_sim::physics;
using namespace vehicle_sim::math;

TEST(BuoyancyTest, FullySubmergedBoxArchimedesInvariant) {
    // 2m wide, 1m high box centered at (0, -5), fully submerged below waterline at y = 0
    const OBB2D box { Vec2 { 0.0, -5.0 }, Vec2 { 1.0, 0.5 }, 0.0 };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);

    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(box, waterline);
    EXPECT_TRUE(slice.isSubmerged());
    EXPECT_EQ(slice.state, SubmergenceState::FullySubmerged);
    EXPECT_NEAR(slice.submergedArea, 2.0, 1e-6);
    EXPECT_NEAR(slice.centerOfBuoyancy.x, 0.0, 1e-6);
    EXPECT_NEAR(slice.centerOfBuoyancy.y, -5.0, 1e-6);

    const BuoyancyConfig2D config {};
    const BuoyancyResult2D res = Buoyancy2D::computeForces(
        slice,
        Vec2 { 0.0, -5.0 },
        Vec2 { 0.0, 0.0 },
        0.0,
        0.0,
        Vec2 { 0.0, 0.0 },
        config);

    // Archimedes force: F_b = rho * A_sub * g = 1000 * 2.0 * 9.80665 = 19613.3 N
    EXPECT_NEAR(res.buoyantForce.x, 0.0, 1e-6);
    EXPECT_NEAR(res.buoyantForce.y, 1000.0 * 2.0 * 9.80665, 1e-3);
    EXPECT_NEAR(res.rightingTorque, 0.0, 1e-6);
}

TEST(BuoyancyTest, FullyAirborneHullZeroBuoyancy) {
    // 2m wide, 1m high box centered at (0, 5), completely airborne above waterline at y = 0
    const OBB2D box { Vec2 { 0.0, 5.0 }, Vec2 { 1.0, 0.5 }, 0.0 };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);

    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(box, waterline);
    EXPECT_FALSE(slice.isSubmerged());
    EXPECT_EQ(slice.state, SubmergenceState::Airborne);
    EXPECT_NEAR(slice.submergedArea, 0.0, 1e-9);

    const BuoyancyConfig2D config {};
    const BuoyancyResult2D res = Buoyancy2D::computeForces(
        slice,
        Vec2 { 0.0, 5.0 },
        Vec2 { 0.0, 0.0 },
        0.0,
        0.0,
        Vec2 { 0.0, 0.0 },
        config);

    EXPECT_NEAR(res.buoyantForce.length(), 0.0, 1e-9);
    EXPECT_NEAR(res.totalForce.length(), 0.0, 1e-9);
    EXPECT_NEAR(res.totalTorque, 0.0, 1e-9);
}

TEST(BuoyancyTest, PartiallySubmergedBoxHalfDraft) {
    // Unit square box 1m x 1m centered at (0, 0)
    // Lower half from y = -0.5 to y = 0.0 is submerged under waterline at y = 0
    const OBB2D box { Vec2 { 0.0, 0.0 }, Vec2 { 0.5, 0.5 }, 0.0 };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);

    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(box, waterline);
    EXPECT_TRUE(slice.isSubmerged());
    EXPECT_EQ(slice.state, SubmergenceState::PartiallySubmerged);

    // Submerged rectangle: width = 1.0, height = 0.5 -> Area = 0.5 m^2
    EXPECT_NEAR(slice.submergedArea, 0.50, 1e-6);
    // Centroid of submerged rectangle: center at (0, -0.25)
    EXPECT_NEAR(slice.centerOfBuoyancy.x, 0.0, 1e-6);
    EXPECT_NEAR(slice.centerOfBuoyancy.y, -0.25, 1e-6);
    // Waterplane width = 1.0m
    EXPECT_NEAR(slice.waterplaneWidth, 1.0, 1e-6);
    // Waterplane inertia I_w = L^3 / 12 = 1.0 / 12
    EXPECT_NEAR(slice.waterplaneInertia, 1.0 / 12.0, 1e-6);

    const BuoyancyConfig2D config {};
    const BuoyancyResult2D res = Buoyancy2D::computeForces(
        slice,
        Vec2 { 0.0, 0.0 },
        Vec2 { 0.0, 0.0 },
        0.0,
        0.0,
        Vec2 { 0.0, 0.0 },
        config);

    EXPECT_NEAR(res.buoyantForce.y, 1000.0 * 0.5 * 9.80665, 1e-3);
    // Metacentric height GM:
    // BM = I_w / A_sub = (1/12) / 0.5 = 1/6
    // KB = -0.25, KG = 0.0 -> GM = 1/6 - 0.25 = -0.0833
    EXPECT_NEAR(res.metacentricHeight, (1.0 / 6.0) - 0.25, 1e-4);
}

TEST(BuoyancyTest, MetacentricRestoringMomentTilt) {
    // Broad rectangular barge: width = 4m, height = 1m centered at (0, 0)
    // Heel clockwise by theta = +0.10 rad (~5.7 deg)
    const double heelAngle = 0.10;
    const OBB2D barge { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 0.5 }, heelAngle };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);

    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(barge, waterline);
    EXPECT_TRUE(slice.isSubmerged());
    EXPECT_EQ(slice.state, SubmergenceState::PartiallySubmerged);

    // When heeled with positive angle (rotated CCW), the right side dips into water (positive X),
    // so center of buoyancy must shift to the right (+X)
    // Wait, with rotation theta > 0, local +X rotates up and local -X dips down, or vice versa
    // Let's verify sign: arm.x * F_b.y gives torque.
    // A stable hull must generate a restoring torque opposing the rotation!
    const BuoyancyConfig2D config {};
    const BuoyancyResult2D res = Buoyancy2D::computeForces(
        slice,
        Vec2 { 0.0, 0.0 },
        Vec2 { 0.0, 0.0 },
        0.0,
        heelAngle,
        Vec2 { 0.0, 0.0 },
        config);

    // Wide beam (4m) gives massive BM = L^3 / (12 * A) = 64 / (12 * 2) = 2.67m,
    // making GM strongly positive (highly stable)
    EXPECT_GT(res.metacentricHeight, 1.0);
    // Restoring torque must oppose rotation: res.rightingTorque * heelAngle < 0
    EXPECT_LT(res.rightingTorque * heelAngle, 0.0);
}

TEST(BuoyancyTest, SlopedWaterlineWaveEncounter) {
    // 2m x 1m box at origin
    const OBB2D box { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.5 }, 0.0 };
    // Waterline passing through (0, 0) with normal tilted 15 degrees: ( -sin(15), cos(15) )
    const double slopeRad = 15.0 * 3.14159265358979323846 / 180.0;
    const Waterline2D waveLine { Vec2 { 0.0, 0.0 }, Vec2 { -std::sin(slopeRad), std::cos(slopeRad) } };

    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(box, waveLine);
    EXPECT_TRUE(slice.isSubmerged());
    EXPECT_EQ(slice.state, SubmergenceState::PartiallySubmerged);
    EXPECT_GT(slice.submergedArea, 0.0);
    EXPECT_LT(slice.submergedArea, 2.0);
}

TEST(BuoyancyTest, HydrodynamicFormAndSkinDrag) {
    // 2m x 1m box fully submerged at (0, -2) moving forward at 5 m/s
    const OBB2D box { Vec2 { 0.0, -2.0 }, Vec2 { 1.0, 0.5 }, 0.0 };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);
    const SubmergedSlice2D slice = HullSlicer2D::sliceOBB(box, waterline);

    const BuoyancyConfig2D config {};
    const Vec2 velocity { 5.0, 0.0 };
    const double angularVel = 2.0;

    const BuoyancyResult2D res = Buoyancy2D::computeForces(
        slice,
        Vec2 { 0.0, -2.0 },
        velocity,
        angularVel,
        0.0,
        Vec2 { 0.0, 0.0 },
        config);

    // Linear drag opposes velocity
    EXPECT_LT(res.dragForce.dot(velocity), 0.0);
    // Rotational damping opposes angular velocity
    EXPECT_LT(res.dampingTorque * angularVel, 0.0);
}

TEST(BuoyancyTest, DirectRigidBodyApplication) {
    RigidBodyConfig bodyCfg {};
    bodyCfg.position = Vec2 { 0.0, 0.0 };
    bodyCfg.mass = 1000.0;
    bodyCfg.linearDamping = 0.0;
    bodyCfg.angularDamping = 0.0;
    RigidBody2D body { bodyCfg };

    const Polygon2D hull = Polygon2D::fromOBB(Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.5 }, 0.0);
    const Waterline2D waterline = Waterline2D::horizontal(0.0);
    const BuoyancyConfig2D config {};

    const BuoyancyResult2D res = Buoyancy2D::applyBuoyancy(body, hull, waterline, config);
    EXPECT_EQ(res.state, SubmergenceState::PartiallySubmerged);
    EXPECT_GT(body.getAccumulatedForce().y, 0.0);
}

TEST(BuoyancyTest, CircleAndColliderSlicing) {
    // 1. Circle fully submerged at y = -3 with radius 1.0
    const Circle2D circleA { Vec2 { 0.0, -3.0 }, 1.0 };
    const Waterline2D waterline = Waterline2D::horizontal(0.0);
    const SubmergedSlice2D sliceA = HullSlicer2D::sliceCircle(circleA, waterline);
    EXPECT_EQ(sliceA.state, SubmergenceState::FullySubmerged);
    constexpr double kPi { 3.14159265358979323846 };
    EXPECT_NEAR(sliceA.submergedArea, kPi * 1.0, 1e-4);

    // 2. Circle airborne at y = +3 with radius 1.0
    const Circle2D circleB { Vec2 { 0.0, 3.0 }, 1.0 };
    const SubmergedSlice2D sliceB = HullSlicer2D::sliceCircle(circleB, waterline);
    EXPECT_EQ(sliceB.state, SubmergenceState::Airborne);
    EXPECT_NEAR(sliceB.submergedArea, 0.0, 1e-9);

    // 3. Circle partially submerged at y = 0 with radius 1.0
    const Circle2D circleC { Vec2 { 0.0, 0.0 }, 1.0 };
    const SubmergedSlice2D sliceC = HullSlicer2D::sliceCircle(circleC, waterline, 8U);
    EXPECT_TRUE(sliceC.isSubmerged());
    EXPECT_EQ(sliceC.state, SubmergenceState::PartiallySubmerged);
    // Semicircle area ~ pi * r^2 / 2 ~ 1.5708
    EXPECT_NEAR(sliceC.submergedArea, (kPi * 1.0) / 2.0, 0.25);

    // 4. Collider2D integration test
    RigidBody2D body {};
    const Collider2D col { circleC, &body };
    const BuoyancyConfig2D config {};
    const BuoyancyResult2D colRes = Buoyancy2D::applyToCollider(body, col, waterline, config);
    EXPECT_TRUE(colRes.state != SubmergenceState::Airborne);
    EXPECT_GT(colRes.buoyantForce.y, 0.0);
}

} // namespace vehicle_sim::testing
