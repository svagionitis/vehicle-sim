#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/Capsule2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/MathUtils.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Ray2D.h"
#include "libs/VehicleSimCore/math/Transform2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::AABB2D;
using vehicle_sim::math::Angle2D;
using vehicle_sim::math::Capsule2D;
using vehicle_sim::math::Circle2D;
using vehicle_sim::math::Mat2;
using vehicle_sim::math::MathUtils;
using vehicle_sim::math::OBB2D;
using vehicle_sim::math::OrientedBox2D;
using vehicle_sim::math::Polygon2D;
using vehicle_sim::math::Ray2D;
using vehicle_sim::math::RayHit2D;
using vehicle_sim::math::Transform2D;
using vehicle_sim::math::Vec2;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kEpsilon { 1e-9 };

// -----------------------------------------------------------------------------
// Vec2 Tests
// -----------------------------------------------------------------------------

TEST(Vec2Test, DefaultAndStaticConstructors)
{
    const Vec2 vDefault {};
    EXPECT_DOUBLE_EQ(vDefault.x, 0.0);
    EXPECT_DOUBLE_EQ(vDefault.y, 0.0);

    const Vec2 vZero { Vec2::zero() };
    EXPECT_DOUBLE_EQ(vZero.x, 0.0);
    EXPECT_DOUBLE_EQ(vZero.y, 0.0);

    const Vec2 vUnitX { Vec2::unitX() };
    EXPECT_DOUBLE_EQ(vUnitX.x, 1.0);
    EXPECT_DOUBLE_EQ(vUnitX.y, 0.0);

    const Vec2 vUnitY { Vec2::unitY() };
    EXPECT_DOUBLE_EQ(vUnitY.x, 0.0);
    EXPECT_DOUBLE_EQ(vUnitY.y, 1.0);
}

TEST(Vec2Test, ArithmeticOperators)
{
    const Vec2 a { 3.0, -2.0 };
    const Vec2 b { -1.0, 5.0 };

    const Vec2 sum { a + b };
    EXPECT_DOUBLE_EQ(sum.x, 2.0);
    EXPECT_DOUBLE_EQ(sum.y, 3.0);

    const Vec2 diff { a - b };
    EXPECT_DOUBLE_EQ(diff.x, 4.0);
    EXPECT_DOUBLE_EQ(diff.y, -7.0);

    const Vec2 neg { -a };
    EXPECT_DOUBLE_EQ(neg.x, -3.0);
    EXPECT_DOUBLE_EQ(neg.y, 2.0);

    const Vec2 scaled { a * 2.5 };
    EXPECT_DOUBLE_EQ(scaled.x, 7.5);
    EXPECT_DOUBLE_EQ(scaled.y, -5.0);

    const Vec2 leftScaled { 2.5 * a };
    EXPECT_DOUBLE_EQ(leftScaled.x, 7.5);
    EXPECT_DOUBLE_EQ(leftScaled.y, -5.0);

    const Vec2 divided { a / 2.0 };
    EXPECT_DOUBLE_EQ(divided.x, 1.5);
    EXPECT_DOUBLE_EQ(divided.y, -1.0);
}

TEST(Vec2Test, InPlaceOperators)
{
    Vec2 v { 1.0, 2.0 };
    v += Vec2 { 3.0, 4.0 };
    EXPECT_DOUBLE_EQ(v.x, 4.0);
    EXPECT_DOUBLE_EQ(v.y, 6.0);

    v -= Vec2 { 1.0, 2.0 };
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);

    v *= 2.0;
    EXPECT_DOUBLE_EQ(v.x, 6.0);
    EXPECT_DOUBLE_EQ(v.y, 8.0);

    v /= 2.0;
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
}

TEST(Vec2Test, EqualityAndComparison)
{
    const Vec2 a { 1.0, 2.0 };
    const Vec2 b { 1.0, 2.0 };
    const Vec2 c { 1.0000000001, 2.0 };

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
    EXPECT_FALSE(a == c);
    EXPECT_TRUE(a != c);
    EXPECT_TRUE(a.isApproxEqual(c, 1e-8));
    EXPECT_FALSE(a.isApproxEqual(c, 1e-11));
}

TEST(Vec2Test, DotAndCrossProducts)
{
    const Vec2 u { 1.0, 0.0 };
    const Vec2 v { 0.0, 1.0 };

    EXPECT_DOUBLE_EQ(u.dot(v), 0.0);
    EXPECT_DOUBLE_EQ(dot(u, v), 0.0);
    EXPECT_DOUBLE_EQ(u.cross(v), 1.0);
    EXPECT_DOUBLE_EQ(cross(u, v), 1.0);
    EXPECT_DOUBLE_EQ(v.cross(u), -1.0);

    const Vec2 p { 2.0, 3.0 };
    const Vec2 q { 4.0, 5.0 };
    EXPECT_DOUBLE_EQ(p.dot(q), 23.0);
    EXPECT_DOUBLE_EQ(p.cross(q), -2.0);

    // Angular velocity cross products: omega x r
    const double omega { 2.5 };
    const Vec2 r { 0.0, 4.0 };
    const Vec2 tangential { cross(omega, r) };
    EXPECT_DOUBLE_EQ(tangential.x, -10.0);
    EXPECT_DOUBLE_EQ(tangential.y, 0.0);

    const Vec2 rCrossOmega { cross(r, omega) };
    EXPECT_DOUBLE_EQ(rCrossOmega.x, 10.0);
    EXPECT_DOUBLE_EQ(rCrossOmega.y, 0.0);
}

TEST(Vec2Test, LengthAndNormalization)
{
    const Vec2 v { 3.0, 4.0 };
    EXPECT_DOUBLE_EQ(v.lengthSquared(), 25.0);
    EXPECT_DOUBLE_EQ(v.length(), 5.0);

    const Vec2 u { v.normalized() };
    EXPECT_NEAR(u.length(), 1.0, kEpsilon);
    EXPECT_DOUBLE_EQ(u.x, 0.6);
    EXPECT_DOUBLE_EQ(u.y, 0.8);

    Vec2 vMut { 3.0, 4.0 };
    vMut.normalize();
    EXPECT_NEAR(vMut.length(), 1.0, kEpsilon);

    const Vec2 zeroVec { 0.0, 0.0 };
    EXPECT_TRUE(zeroVec.isZero());
    EXPECT_DOUBLE_EQ(zeroVec.normalized().x, 0.0);
    EXPECT_DOUBLE_EQ(zeroVec.normalized().y, 0.0);
}

TEST(Vec2Test, PerpendicularAndRotation)
{
    const Vec2 v { 3.0, 4.0 };
    const Vec2 perp { v.perpendicular() };
    EXPECT_DOUBLE_EQ(perp.x, -4.0);
    EXPECT_DOUBLE_EQ(perp.y, 3.0);
    EXPECT_DOUBLE_EQ(v.dot(perp), 0.0);

    const Vec2 unitX { 1.0, 0.0 };
    const Vec2 rot90 { unitX.rotated(kPi * 0.5) };
    EXPECT_NEAR(rot90.x, 0.0, kEpsilon);
    EXPECT_NEAR(rot90.y, 1.0, kEpsilon);

    const Vec2 rot180 { unitX.rotated(kPi) };
    EXPECT_NEAR(rot180.x, -1.0, kEpsilon);
    EXPECT_NEAR(rot180.y, 0.0, kEpsilon);
}

// -----------------------------------------------------------------------------
// Mat2 Tests
// -----------------------------------------------------------------------------

TEST(Mat2Test, ConstructorsAndConstants)
{
    const Mat2 ident { Mat2::identity() };
    EXPECT_DOUBLE_EQ(ident.m00, 1.0);
    EXPECT_DOUBLE_EQ(ident.m01, 0.0);
    EXPECT_DOUBLE_EQ(ident.m10, 0.0);
    EXPECT_DOUBLE_EQ(ident.m11, 1.0);

    const Mat2 zero { Mat2::zero() };
    EXPECT_DOUBLE_EQ(zero.m00, 0.0);
    EXPECT_DOUBLE_EQ(zero.m11, 0.0);

    const Mat2 fromR { Mat2::fromRows(Vec2 { 1.0, 2.0 }, Vec2 { 3.0, 4.0 }) };
    EXPECT_DOUBLE_EQ(fromR.m00, 1.0);
    EXPECT_DOUBLE_EQ(fromR.m01, 2.0);
    EXPECT_DOUBLE_EQ(fromR.m10, 3.0);
    EXPECT_DOUBLE_EQ(fromR.m11, 4.0);

    const Mat2 fromC { Mat2::fromColumns(Vec2 { 1.0, 3.0 }, Vec2 { 2.0, 4.0 }) };
    EXPECT_EQ(fromR, fromC);
}

TEST(Mat2Test, MatrixArithmeticAndDeterminant)
{
    const Mat2 m1 { 1.0, 2.0, 3.0, 4.0 };
    const Mat2 m2 { 5.0, 6.0, 7.0, 8.0 };

    const Mat2 sum { m1 + m2 };
    EXPECT_DOUBLE_EQ(sum.m00, 6.0);
    EXPECT_DOUBLE_EQ(sum.m01, 8.0);
    EXPECT_DOUBLE_EQ(sum.m10, 10.0);
    EXPECT_DOUBLE_EQ(sum.m11, 12.0);

    const Mat2 prod { m1 * m2 };
    EXPECT_DOUBLE_EQ(prod.m00, 19.0);
    EXPECT_DOUBLE_EQ(prod.m01, 22.0);
    EXPECT_DOUBLE_EQ(prod.m10, 43.0);
    EXPECT_DOUBLE_EQ(prod.m11, 50.0);

    EXPECT_DOUBLE_EQ(m1.det(), -2.0);
    EXPECT_DOUBLE_EQ(m1.transposed().m01, 3.0);
    EXPECT_DOUBLE_EQ(m1.transposed().m10, 2.0);
}

TEST(Mat2Test, VectorTransformationAndRotation)
{
    const Mat2 rot90 { Mat2::fromAngle(kPi * 0.5) };
    const Vec2 v { 1.0, 0.0 };
    const Vec2 vRot { rot90 * v };

    EXPECT_NEAR(vRot.x, 0.0, kEpsilon);
    EXPECT_NEAR(vRot.y, 1.0, kEpsilon);

    EXPECT_NEAR(rot90.det(), 1.0, kEpsilon);
}

TEST(Mat2Test, InverseMatrix)
{
    const Mat2 m { 4.0, 7.0, 2.0, 6.0 };
    const auto invOpt { m.inverse() };
    ASSERT_TRUE(invOpt.has_value());

    const Mat2 inv { *invOpt };
    const Mat2 identityProduct { m * inv };
    EXPECT_TRUE(identityProduct.isApproxEqual(Mat2::identity(), kEpsilon));

    const Mat2 singular { 1.0, 2.0, 2.0, 4.0 };
    EXPECT_FALSE(singular.inverse().has_value());
}

// -----------------------------------------------------------------------------
// Transform2D Tests
// -----------------------------------------------------------------------------

TEST(Transform2DTest, IdentityTransform)
{
    const Transform2D t { Transform2D::identity() };
    const Vec2 point { 5.0, -3.0 };

    EXPECT_EQ(t.transformPoint(point), point);
    EXPECT_EQ(t.inverseTransformPoint(point), point);
    EXPECT_EQ(t.transformVector(point), point);
    EXPECT_EQ(t.inverseTransformVector(point), point);
}

TEST(Transform2DTest, TranslationAndRotation)
{
    const Transform2D t { Vec2 { 10.0, 5.0 }, kPi * 0.5 };
    const Vec2 localPoint { 1.0, 0.0 };

    // In local frame, (1, 0) rotated by 90 deg is (0, 1), plus (10, 5) -> (10, 6)
    const Vec2 worldPoint { t.transformPoint(localPoint) };
    EXPECT_NEAR(worldPoint.x, 10.0, kEpsilon);
    EXPECT_NEAR(worldPoint.y, 6.0, kEpsilon);

    // Round-trip back to local frame
    const Vec2 backLocal { t.inverseTransformPoint(worldPoint) };
    EXPECT_NEAR(backLocal.x, localPoint.x, kEpsilon);
    EXPECT_NEAR(backLocal.y, localPoint.y, kEpsilon);

    // Vector transform should ignore position offset
    const Vec2 worldVec { t.transformVector(localPoint) };
    EXPECT_NEAR(worldVec.x, 0.0, kEpsilon);
    EXPECT_NEAR(worldVec.y, 1.0, kEpsilon);

    const Vec2 backVec { t.inverseTransformVector(worldVec) };
    EXPECT_NEAR(backVec.x, localPoint.x, kEpsilon);
    EXPECT_NEAR(backVec.y, localPoint.y, kEpsilon);
}

TEST(Transform2DTest, InverseAndCombine)
{
    const Transform2D t1 { Vec2 { 2.0, 3.0 }, 0.5 };
    const Transform2D tInv { t1.inverse() };

    const Transform2D combined { t1 * tInv };
    EXPECT_TRUE(combined.isApproxEqual(Transform2D::identity(), kEpsilon));

    const Transform2D t2 { Vec2 { -1.0, 4.0 }, -0.2 };
    const Transform2D t12 { t1.combine(t2) };
    const Vec2 testPt { 3.0, 7.0 };

    // t12.transformPoint(p) should equal t1.transformPoint(t2.transformPoint(p))
    const Vec2 expectedPt { t1.transformPoint(t2.transformPoint(testPt)) };
    const Vec2 actualPt { t12.transformPoint(testPt) };
    EXPECT_TRUE(actualPt.isApproxEqual(expectedPt, kEpsilon));
}

TEST(Transform2DTest, AngleAccessors)
{
    Transform2D t { Vec2 { 0.0, 0.0 }, 0.75 };
    EXPECT_NEAR(t.getAngle(), 0.75, kEpsilon);

    t.setAngle(-1.25);
    EXPECT_NEAR(t.getAngle(), -1.25, kEpsilon);
}

// -----------------------------------------------------------------------------
// Angle2D Tests
// -----------------------------------------------------------------------------

TEST(Angle2DTest, DefaultAndStaticConstructors)
{
    const Angle2D aDefault {};
    EXPECT_DOUBLE_EQ(aDefault.radians(), 0.0);
    EXPECT_DOUBLE_EQ(aDefault.degrees(), 0.0);

    const Angle2D aZero { Angle2D::zero() };
    EXPECT_DOUBLE_EQ(aZero.radians(), 0.0);

    const Angle2D aPi { Angle2D::pi() };
    EXPECT_DOUBLE_EQ(aPi.radians(), -Angle2D::kPi);
}

TEST(Angle2DTest, FromRadiansAndDegrees)
{
    const Angle2D aRad { Angle2D::fromRadians(Angle2D::kHalfPi) };
    EXPECT_NEAR(aRad.radians(), Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(aRad.degrees(), 90.0, kEpsilon);

    const Angle2D aDeg { Angle2D::fromDegrees(180.0) };
    EXPECT_NEAR(aDeg.radians(), Angle2D::kPi, kEpsilon);

    const Angle2D aWrap { Angle2D::fromDegrees(450.0) };
    EXPECT_NEAR(aWrap.degrees(), 90.0, kEpsilon);

    EXPECT_NEAR(Angle2D::degToRad(180.0), Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::radToDeg(Angle2D::kPi), 180.0, kEpsilon);
}

TEST(Angle2DTest, VectorConversionRoundTrip)
{
    const Angle2D aAngle { Angle2D::fromRadians(Angle2D::kHalfPi) };
    const Vec2 vDir { aAngle.toVector() };
    EXPECT_NEAR(vDir.x, 0.0, kEpsilon);
    EXPECT_NEAR(vDir.y, 1.0, kEpsilon);

    const Angle2D aRecovered { Angle2D::fromVector(vDir) };
    EXPECT_NEAR(aRecovered.radians(), Angle2D::kHalfPi, kEpsilon);

    const Vec2 vUnitX { 1.0, 0.0 };
    EXPECT_NEAR(Angle2D::toAngle(vUnitX), 0.0, kEpsilon);
}

TEST(Angle2DTest, OperatorsAndWrapping)
{
    const Angle2D a1 { Angle2D::fromDegrees(120.0) };
    const Angle2D a2 { Angle2D::fromDegrees(90.0) };

    const Angle2D aSum { a1 + a2 };
    // 210 deg wraps to -150 deg
    EXPECT_NEAR(aSum.degrees(), -150.0, kEpsilon);

    const Angle2D aDiff { a1 - a2 };
    EXPECT_NEAR(aDiff.degrees(), 30.0, kEpsilon);

    const Angle2D aNeg { -a1 };
    EXPECT_NEAR(aNeg.degrees(), -120.0, kEpsilon);

    Angle2D aMut { a1 };
    aMut += a2;
    EXPECT_NEAR(aMut.degrees(), -150.0, kEpsilon);
    aMut -= a2;
    EXPECT_NEAR(aMut.degrees(), 120.0, kEpsilon);

    EXPECT_TRUE(a1 == Angle2D::fromDegrees(120.0));
    EXPECT_FALSE(a1 == a2);
    EXPECT_TRUE(a1 != a2);
}

TEST(Angle2DTest, NormalizeRadBoundaries)
{
    EXPECT_NEAR(Angle2D::normalizeRad(0.0), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(Angle2D::kHalfPi), Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(-Angle2D::kHalfPi), -Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(-Angle2D::kPi), -Angle2D::kPi, kEpsilon);

    // Boundary +Pi preserves sign on positive odd multiples
    EXPECT_NEAR(Angle2D::normalizeRad(Angle2D::kPi), Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(3.0 * Angle2D::kPi), Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(-3.0 * Angle2D::kPi), -Angle2D::kPi, kEpsilon);

    // Multiple revolutions
    EXPECT_NEAR(Angle2D::normalizeRad(Angle2D::kTwoPi), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(-Angle2D::kTwoPi), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(100.0 * Angle2D::kPi), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad(101.0 * Angle2D::kPi), Angle2D::kPi, kEpsilon);

    // Non-finite values
    EXPECT_DOUBLE_EQ(Angle2D::normalizeRad(std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_DOUBLE_EQ(Angle2D::normalizeRad(-std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_DOUBLE_EQ(Angle2D::normalizeRad(std::numeric_limits<double>::quiet_NaN()), 0.0);
}

TEST(Angle2DTest, NormalizeDegBoundaries)
{
    EXPECT_NEAR(Angle2D::normalizeDeg(0.0), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(90.0), 90.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(-90.0), -90.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(180.0), 180.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(360.0), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(540.0), 180.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeDeg(-540.0), -180.0, kEpsilon);

    EXPECT_DOUBLE_EQ(Angle2D::normalizeDeg(std::numeric_limits<double>::infinity()), 0.0);
}

TEST(Angle2DTest, NormalizeRad0To2Pi)
{
    EXPECT_NEAR(Angle2D::normalizeRad0To2Pi(0.0), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad0To2Pi(Angle2D::kPi), Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad0To2Pi(-Angle2D::kHalfPi), 1.5 * Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad0To2Pi(Angle2D::kTwoPi), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::normalizeRad0To2Pi(3.0 * Angle2D::kPi), Angle2D::kPi, kEpsilon);
}

TEST(Angle2DTest, DeltaAngleAndDistance)
{
    EXPECT_NEAR(Angle2D::deltaAngle(0.0, 0.0), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::deltaAngle(0.0, Angle2D::kHalfPi), Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(Angle2D::deltaAngle(Angle2D::kHalfPi, 0.0), -Angle2D::kHalfPi, kEpsilon);

    // Shortest path crossing branch cut
    const double a1 { 0.95 * Angle2D::kPi };
    const double a2 { -0.95 * Angle2D::kPi };
    EXPECT_NEAR(Angle2D::deltaAngle(a1, a2), 0.1 * Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::deltaAngle(a2, a1), -0.1 * Angle2D::kPi, kEpsilon);

    EXPECT_NEAR(Angle2D::angularDistance(a1, a2), 0.1 * Angle2D::kPi, kEpsilon);
    EXPECT_NEAR(Angle2D::angularDistance(a2, a1), 0.1 * Angle2D::kPi, kEpsilon);
}

TEST(Angle2DTest, LerpAngleAcrossBranchCut)
{
    const double a1 { 0.9 * Angle2D::kPi };
    const double a2 { -0.9 * Angle2D::kPi };

    // Interpolating halfway from 0.9pi to -0.9pi passes through boundary (-pi / pi)
    const double mid { Angle2D::lerpAngle(a1, a2, 0.5) };
    EXPECT_NEAR(std::abs(mid), Angle2D::kPi, kEpsilon);

    EXPECT_NEAR(Angle2D::lerpAngle(a1, a2, 0.0), a1, kEpsilon);
    EXPECT_NEAR(Angle2D::lerpAngle(a1, a2, 1.0), a2, kEpsilon);
}

TEST(Angle2DTest, Slerp2DVectors)
{
    const Vec2 vX { 1.0, 0.0 };
    const Vec2 vY { 0.0, 1.0 };

    const Vec2 vMid { Angle2D::slerp2D(vX, vY, 0.5) };
    EXPECT_NEAR(vMid.x, std::cos(Angle2D::kHalfPi * 0.5), kEpsilon);
    EXPECT_NEAR(vMid.y, std::sin(Angle2D::kHalfPi * 0.5), kEpsilon);

    // Collinear vectors
    const Vec2 vCollinear1 { 2.0, 0.0 };
    const Vec2 vCollinear2 { 4.0, 0.0 };
    const Vec2 vCollinearMid { Angle2D::slerp2D(vCollinear1, vCollinear2, 0.5) };
    EXPECT_NEAR(vCollinearMid.x, 3.0, kEpsilon);
    EXPECT_NEAR(vCollinearMid.y, 0.0, kEpsilon);

    // Opposite vectors
    const Vec2 vOpposite { -1.0, 0.0 };
    const Vec2 vOppositeMid { Angle2D::slerp2D(vX, vOpposite, 0.5) };
    EXPECT_NEAR(vOppositeMid.length(), 1.0, kEpsilon);
    EXPECT_NEAR(vOppositeMid.x, 0.0, kEpsilon);
}

TEST(Angle2DTest, SignedAngleBetween)
{
    const Vec2 vX { 1.0, 0.0 };
    const Vec2 vY { 0.0, 1.0 };

    EXPECT_NEAR(Angle2D::signedAngleBetween(vX, vY), Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(Angle2D::signedAngleBetween(vY, vX), -Angle2D::kHalfPi, kEpsilon);
    EXPECT_NEAR(Angle2D::signedAngleBetween(vX, vX), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::signedAngleBetween(Vec2::zero(), vX), 0.0, kEpsilon);
}

TEST(Angle2DTest, ClampAngle)
{
    const double minA { -Angle2D::kHalfPi };
    const double maxA { Angle2D::kHalfPi };

    EXPECT_NEAR(Angle2D::clampAngle(0.0, minA, maxA), 0.0, kEpsilon);
    EXPECT_NEAR(Angle2D::clampAngle(Angle2D::kPi, minA, maxA), maxA, kEpsilon);
    EXPECT_NEAR(Angle2D::clampAngle(-2.0, minA, maxA), minA, kEpsilon);
}

// -----------------------------------------------------------------------------
// MathUtils Tests
// -----------------------------------------------------------------------------

TEST(MathUtilsTest, LerpAndInverseLerp)
{
    EXPECT_DOUBLE_EQ(MathUtils::lerp(10.0, 20.0, 0.0), 10.0);
    EXPECT_DOUBLE_EQ(MathUtils::lerp(10.0, 20.0, 1.0), 20.0);
    EXPECT_DOUBLE_EQ(MathUtils::lerp(10.0, 20.0, 0.5), 15.0);

    EXPECT_DOUBLE_EQ(MathUtils::inverseLerp(10.0, 20.0, 15.0), 0.5);
    EXPECT_DOUBLE_EQ(MathUtils::inverseLerp(10.0, 20.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(MathUtils::inverseLerp(10.0, 20.0, 20.0), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::inverseLerp(10.0, 10.0, 10.0), 0.0);
}

TEST(MathUtilsTest, RemapAndClamp01)
{
    EXPECT_DOUBLE_EQ(MathUtils::clamp01(-0.5), 0.0);
    EXPECT_DOUBLE_EQ(MathUtils::clamp01(0.5), 0.5);
    EXPECT_DOUBLE_EQ(MathUtils::clamp01(1.5), 1.0);

    EXPECT_DOUBLE_EQ(MathUtils::remap(0.0, 10.0, 100.0, 200.0, 5.0), 150.0);
}

TEST(MathUtilsTest, SmoothstepAndSmootherstep)
{
    EXPECT_DOUBLE_EQ(MathUtils::smoothstep(0.0, 1.0, -0.5), 0.0);
    EXPECT_DOUBLE_EQ(MathUtils::smoothstep(0.0, 1.0, 1.5), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::smoothstep(0.0, 1.0, 0.5), 0.5);

    EXPECT_DOUBLE_EQ(MathUtils::smootherstep(0.0, 1.0, -0.5), 0.0);
    EXPECT_DOUBLE_EQ(MathUtils::smootherstep(0.0, 1.0, 1.5), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::smootherstep(0.0, 1.0, 0.5), 0.5);

    EXPECT_DOUBLE_EQ(MathUtils::smoothstep(5.0, 5.0, 5.0), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::smoothstep(5.0, 5.0, 4.0), 0.0);
}

TEST(MathUtilsTest, ApplyDeadband)
{
    EXPECT_DOUBLE_EQ(MathUtils::applyDeadband(0.1, 0.2), 0.0);
    EXPECT_DOUBLE_EQ(MathUtils::applyDeadband(-0.1, 0.2), 0.0);
    EXPECT_NEAR(MathUtils::applyDeadband(1.0, 0.2), 1.0, kEpsilon);
    EXPECT_NEAR(MathUtils::applyDeadband(-1.0, 0.2), -1.0, kEpsilon);
    EXPECT_NEAR(MathUtils::applyDeadband(0.6, 0.2), 0.5, kEpsilon);
    EXPECT_DOUBLE_EQ(MathUtils::applyDeadband(0.5, 0.0), 0.5);
    EXPECT_DOUBLE_EQ(MathUtils::applyDeadband(0.5, 1.5, 1.0), 0.0);
}

TEST(MathUtilsTest, SignAndSignNonZero)
{
    EXPECT_DOUBLE_EQ(MathUtils::sign(5.0), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::sign(-5.0), -1.0);
    EXPECT_DOUBLE_EQ(MathUtils::sign(0.0), 0.0);

    EXPECT_DOUBLE_EQ(MathUtils::signNonZero(5.0), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::signNonZero(0.0), 1.0);
    EXPECT_DOUBLE_EQ(MathUtils::signNonZero(-5.0), -1.0);
}

TEST(MathUtilsTest, NearlyEqualAndNearlyZero)
{
    EXPECT_TRUE(MathUtils::nearlyEqual(1.0, 1.0000000001, 1e-8));
    EXPECT_FALSE(MathUtils::nearlyEqual(1.0, 1.1, 1e-4));

    EXPECT_TRUE(MathUtils::nearlyZero(1e-10));
    EXPECT_FALSE(MathUtils::nearlyZero(0.01));
}

TEST(MathUtilsTest, SafeDivide)
{
    EXPECT_DOUBLE_EQ(MathUtils::safeDivide(10.0, 2.0), 5.0);
    EXPECT_DOUBLE_EQ(MathUtils::safeDivide(10.0, 0.0, -1.0), -1.0);
    EXPECT_DOUBLE_EQ(MathUtils::safeDivide(10.0, 1e-13, 99.0), 99.0);
}

// -----------------------------------------------------------------------------
// Ray2D Tests
// -----------------------------------------------------------------------------

TEST(Ray2DTest, DefaultAndParameterizedConstruction)
{
    const Ray2D defaultRay {};
    EXPECT_DOUBLE_EQ(defaultRay.origin.x, 0.0);
    EXPECT_DOUBLE_EQ(defaultRay.origin.y, 0.0);
    EXPECT_DOUBLE_EQ(defaultRay.direction.x, 1.0);
    EXPECT_DOUBLE_EQ(defaultRay.direction.y, 0.0);
    EXPECT_TRUE(std::isinf(defaultRay.maxDistance));

    const Vec2 origin { 2.0, 3.0 };
    const Vec2 unnormalizedDir { 3.0, 4.0 };
    const Ray2D ray { origin, unnormalizedDir, 10.0 };

    EXPECT_DOUBLE_EQ(ray.origin.x, 2.0);
    EXPECT_DOUBLE_EQ(ray.origin.y, 3.0);
    EXPECT_NEAR(ray.direction.x, 0.6, kEpsilon);
    EXPECT_NEAR(ray.direction.y, 0.8, kEpsilon);
    EXPECT_DOUBLE_EQ(ray.maxDistance, 10.0);

    const Ray2D degenerateRay { origin, Vec2::zero() };
    EXPECT_DOUBLE_EQ(degenerateRay.direction.x, 1.0);
    EXPECT_DOUBLE_EQ(degenerateRay.direction.y, 0.0);
}

TEST(Ray2DTest, PointAtAndWithMaxDistance)
{
    const Ray2D ray { Vec2 { 1.0, 2.0 }, Vec2 { 0.0, 1.0 }, 5.0 };
    const Vec2 p0 { ray.pointAt(0.0) };
    EXPECT_DOUBLE_EQ(p0.x, 1.0);
    EXPECT_DOUBLE_EQ(p0.y, 2.0);

    const Vec2 p5 { ray.pointAt(5.0) };
    EXPECT_DOUBLE_EQ(p5.x, 1.0);
    EXPECT_DOUBLE_EQ(p5.y, 7.0);

    const Ray2D clamped { ray.withMaxDistance(2.5) };
    EXPECT_DOUBLE_EQ(clamped.maxDistance, 2.5);
}

TEST(Ray2DTest, Transformed)
{
    const Ray2D ray { Vec2 { 1.0, 0.0 }, Vec2 { 1.0, 0.0 } };
    const Ray2D transformed { ray.transformed(Vec2 { 0.0, 5.0 }, kPi * 0.5) };

    EXPECT_NEAR(transformed.origin.x, 0.0, kEpsilon);
    EXPECT_NEAR(transformed.origin.y, 6.0, kEpsilon);
    EXPECT_NEAR(transformed.direction.x, 0.0, kEpsilon);
    EXPECT_NEAR(transformed.direction.y, 1.0, kEpsilon);
}

// -----------------------------------------------------------------------------
// AABB2D Tests
// -----------------------------------------------------------------------------

TEST(AABB2DTest, ConstructionAndMetrics)
{
    const AABB2D aabb { Vec2 { -2.0, -1.0 }, Vec2 { 3.0, 4.0 } };
    EXPECT_TRUE(aabb.isValid());

    const Vec2 c { aabb.center() };
    EXPECT_DOUBLE_EQ(c.x, 0.5);
    EXPECT_DOUBLE_EQ(c.y, 1.5);

    EXPECT_DOUBLE_EQ(aabb.width(), 5.0);
    EXPECT_DOUBLE_EQ(aabb.height(), 5.0);
    EXPECT_DOUBLE_EQ(aabb.area(), 25.0);

    const Vec2 ext { aabb.extents() };
    EXPECT_DOUBLE_EQ(ext.x, 5.0);
    EXPECT_DOUBLE_EQ(ext.y, 5.0);

    const Vec2 halfExt { aabb.halfExtents() };
    EXPECT_DOUBLE_EQ(halfExt.x, 2.5);
    EXPECT_DOUBLE_EQ(halfExt.y, 2.5);

    const AABB2D fromFactory { AABB2D::fromCenterExtents(Vec2 { 1.0, 2.0 }, Vec2 { 3.0, 4.0 }) };
    EXPECT_DOUBLE_EQ(fromFactory.min.x, -2.0);
    EXPECT_DOUBLE_EQ(fromFactory.min.y, -2.0);
    EXPECT_DOUBLE_EQ(fromFactory.max.x, 4.0);
    EXPECT_DOUBLE_EQ(fromFactory.max.y, 6.0);
}

TEST(AABB2DTest, FromPoints)
{
    const Vec2 points[4] { Vec2 { 1.0, -3.0 }, Vec2 { 5.0, 2.0 }, Vec2 { -4.0, 0.0 }, Vec2 { 2.0, 6.0 } };

    const AABB2D bounds { AABB2D::fromPoints(points, 4U) };
    EXPECT_DOUBLE_EQ(bounds.min.x, -4.0);
    EXPECT_DOUBLE_EQ(bounds.min.y, -3.0);
    EXPECT_DOUBLE_EQ(bounds.max.x, 5.0);
    EXPECT_DOUBLE_EQ(bounds.max.y, 6.0);

    const AABB2D emptyBounds { AABB2D::fromPoints(nullptr, 0U) };
    EXPECT_DOUBLE_EQ(emptyBounds.min.x, 0.0);
    EXPECT_DOUBLE_EQ(emptyBounds.max.x, 0.0);
}

TEST(AABB2DTest, ContainmentAndIntersection)
{
    const AABB2D box { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 10.0 } };

    EXPECT_TRUE(box.contains(Vec2 { 5.0, 5.0 }));
    EXPECT_TRUE(box.contains(Vec2 { 0.0, 10.0 }));
    EXPECT_FALSE(box.contains(Vec2 { -0.1, 5.0 }));
    EXPECT_FALSE(box.contains(Vec2 { 5.0, 10.1 }));

    const AABB2D inner { Vec2 { 2.0, 2.0 }, Vec2 { 8.0, 8.0 } };
    const AABB2D overlapping { Vec2 { 5.0, 5.0 }, Vec2 { 15.0, 15.0 } };
    const AABB2D disjoint { Vec2 { 12.0, 12.0 }, Vec2 { 15.0, 15.0 } };

    EXPECT_TRUE(box.contains(inner));
    EXPECT_FALSE(box.contains(overlapping));

    EXPECT_TRUE(box.intersects(inner));
    EXPECT_TRUE(box.intersects(overlapping));
    EXPECT_FALSE(box.intersects(disjoint));
}

TEST(AABB2DTest, ExpansionAndEncapsulation)
{
    AABB2D box { Vec2 { 2.0, 3.0 }, Vec2 { 6.0, 7.0 } };
    const AABB2D expandedUniform { box.expanded(1.0) };
    EXPECT_DOUBLE_EQ(expandedUniform.min.x, 1.0);
    EXPECT_DOUBLE_EQ(expandedUniform.min.y, 2.0);
    EXPECT_DOUBLE_EQ(expandedUniform.max.x, 7.0);
    EXPECT_DOUBLE_EQ(expandedUniform.max.y, 8.0);

    const AABB2D expandedAxes { box.expanded(Vec2 { 2.0, 0.5 }) };
    EXPECT_DOUBLE_EQ(expandedAxes.min.x, 0.0);
    EXPECT_DOUBLE_EQ(expandedAxes.min.y, 2.5);
    EXPECT_DOUBLE_EQ(expandedAxes.max.x, 8.0);
    EXPECT_DOUBLE_EQ(expandedAxes.max.y, 7.5);

    box.encapsulate(Vec2 { -1.0, 10.0 });
    EXPECT_DOUBLE_EQ(box.min.x, -1.0);
    EXPECT_DOUBLE_EQ(box.max.y, 10.0);

    box.encapsulate(AABB2D { Vec2 { -5.0, 0.0 }, Vec2 { 0.0, 12.0 } });
    EXPECT_DOUBLE_EQ(box.min.x, -5.0);
    EXPECT_DOUBLE_EQ(box.max.y, 12.0);
}

TEST(AABB2DTest, ClosestPointAndToOBB)
{
    const AABB2D box { Vec2 { -2.0, -1.0 }, Vec2 { 2.0, 1.0 } };

    const Vec2 inside { 0.5, 0.5 };
    EXPECT_DOUBLE_EQ(box.closestPoint(inside).x, 0.5);
    EXPECT_DOUBLE_EQ(box.closestPoint(inside).y, 0.5);

    const Vec2 outsideX { 5.0, 0.0 };
    EXPECT_DOUBLE_EQ(box.closestPoint(outsideX).x, 2.0);
    EXPECT_DOUBLE_EQ(box.closestPoint(outsideX).y, 0.0);

    const Vec2 outsideCorner { -10.0, -10.0 };
    EXPECT_DOUBLE_EQ(box.closestPoint(outsideCorner).x, -2.0);
    EXPECT_DOUBLE_EQ(box.closestPoint(outsideCorner).y, -1.0);

    const OBB2D obb { box.toOBB() };
    EXPECT_DOUBLE_EQ(obb.center.x, 0.0);
    EXPECT_DOUBLE_EQ(obb.center.y, 0.0);
    EXPECT_DOUBLE_EQ(obb.halfExtents.x, 2.0);
    EXPECT_DOUBLE_EQ(obb.halfExtents.y, 1.0);
    EXPECT_DOUBLE_EQ(obb.rotation, 0.0);
}

TEST(AABB2DTest, RaycastHitsAndMisses)
{
    const AABB2D box { Vec2 { -2.0, -2.0 }, Vec2 { 2.0, 2.0 } };

    // Raycast hit from left face
    const Ray2D rayLeft { Vec2 { -5.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hitLeft { box.raycast(rayLeft) };
    ASSERT_TRUE(hitLeft.has_value());
    EXPECT_NEAR(hitLeft->distance, 3.0, kEpsilon);
    EXPECT_NEAR(hitLeft->point.x, -2.0, kEpsilon);
    EXPECT_NEAR(hitLeft->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hitLeft->normal.x, -1.0, kEpsilon);
    EXPECT_NEAR(hitLeft->normal.y, 0.0, kEpsilon);
    EXPECT_NEAR(hitLeft->fraction, 0.3, kEpsilon);

    // Raycast hit from top face
    const Ray2D rayTop { Vec2 { 0.0, 5.0 }, Vec2 { 0.0, -1.0 }, 10.0 };
    const std::optional<RayHit2D> hitTop { box.raycast(rayTop) };
    ASSERT_TRUE(hitTop.has_value());
    EXPECT_NEAR(hitTop->distance, 3.0, kEpsilon);
    EXPECT_NEAR(hitTop->point.x, 0.0, kEpsilon);
    EXPECT_NEAR(hitTop->point.y, 2.0, kEpsilon);
    EXPECT_NEAR(hitTop->normal.x, 0.0, kEpsilon);
    EXPECT_NEAR(hitTop->normal.y, 1.0, kEpsilon);

    // Raycast miss: pointing away
    const Ray2D rayAway { Vec2 { -5.0, 0.0 }, Vec2 { -1.0, 0.0 }, 10.0 };
    EXPECT_FALSE(box.raycast(rayAway).has_value());

    // Raycast miss: range too short
    const Ray2D rayShort { Vec2 { -5.0, 0.0 }, Vec2 { 1.0, 0.0 }, 2.0 };
    EXPECT_FALSE(box.raycast(rayShort).has_value());

    // Raycast parallel outside
    const Ray2D rayParallel { Vec2 { -5.0, 3.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    EXPECT_FALSE(box.raycast(rayParallel).has_value());

    // Raycast origin inside box
    const Ray2D rayInside { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hitInside { box.raycast(rayInside) };
    ASSERT_TRUE(hitInside.has_value());
    EXPECT_NEAR(hitInside->distance, 0.0, kEpsilon);
    EXPECT_NEAR(hitInside->point.x, 0.0, kEpsilon);
    EXPECT_NEAR(hitInside->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hitInside->normal.x, -1.0, kEpsilon);
}

// -----------------------------------------------------------------------------
// OBB2D Tests
// -----------------------------------------------------------------------------

TEST(OBB2DTest, ConstructionAndVertices)
{
    const OBB2D obb { Vec2 { 2.0, 3.0 }, Vec2 { 2.0, 1.0 }, 0.0 };
    EXPECT_DOUBLE_EQ(obb.axisX().x, 1.0);
    EXPECT_DOUBLE_EQ(obb.axisX().y, 0.0);
    EXPECT_DOUBLE_EQ(obb.axisY().x, 0.0);
    EXPECT_DOUBLE_EQ(obb.axisY().y, 1.0);

    const std::array<Vec2, 4> verts { obb.vertices() };
    EXPECT_DOUBLE_EQ(verts[0].x, 0.0); // center - ax - ay
    EXPECT_DOUBLE_EQ(verts[0].y, 2.0);
    EXPECT_DOUBLE_EQ(verts[1].x, 4.0); // center + ax - ay
    EXPECT_DOUBLE_EQ(verts[1].y, 2.0);
    EXPECT_DOUBLE_EQ(verts[2].x, 4.0); // center + ax + ay
    EXPECT_DOUBLE_EQ(verts[2].y, 4.0);
    EXPECT_DOUBLE_EQ(verts[3].x, 0.0); // center - ax + ay
    EXPECT_DOUBLE_EQ(verts[3].y, 4.0);
}

TEST(OBB2DTest, LocalAndWorldTransforms)
{
    const OBB2D rotated { Vec2 { 10.0, 5.0 }, Vec2 { 2.0, 1.0 }, kPi * 0.5 };

    const Vec2 worldPt { 9.0, 5.0 }; // 1 unit along -X in world is 1 unit along +Y in local
    const Vec2 localPt { rotated.toLocal(worldPt) };
    EXPECT_NEAR(localPt.x, 0.0, kEpsilon);
    EXPECT_NEAR(localPt.y, 1.0, kEpsilon);

    const Vec2 restoredWorld { rotated.toWorld(localPt) };
    EXPECT_NEAR(restoredWorld.x, worldPt.x, kEpsilon);
    EXPECT_NEAR(restoredWorld.y, worldPt.y, kEpsilon);
}

TEST(OBB2DTest, ContainmentAndClosestPoint)
{
    const OBB2D obb { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 1.0 }, kPi * 0.25 };

    EXPECT_TRUE(obb.contains(Vec2 { 0.0, 0.0 }));

    // Point outside along rotated major axis
    const Vec2 farPt { 5.0, 5.0 };
    EXPECT_FALSE(obb.contains(farPt));

    const Vec2 closest { obb.closestPoint(farPt) };
    // Major axis at 45 deg has half-extent 2.0 -> point is (2*cos(45), 2*sin(45))
    const double expectedDist { 2.0 };
    EXPECT_NEAR(closest.length(), expectedDist, kEpsilon);
    EXPECT_NEAR(closest.x, closest.y, kEpsilon);
}

TEST(OBB2DTest, ComputeAABB)
{
    // Rotated 45 degrees box with half extents 1, 1:
    // rx = 1 * cos(45) + 1 * sin(45) = sqrt(2)
    const OBB2D diamond { Vec2 { 5.0, 5.0 }, Vec2 { 1.0, 1.0 }, kPi * 0.25 };
    const AABB2D tightAABB { diamond.computeAABB() };

    const double expectedR { std::sqrt(2.0) };
    EXPECT_NEAR(tightAABB.min.x, 5.0 - expectedR, kEpsilon);
    EXPECT_NEAR(tightAABB.min.y, 5.0 - expectedR, kEpsilon);
    EXPECT_NEAR(tightAABB.max.x, 5.0 + expectedR, kEpsilon);
    EXPECT_NEAR(tightAABB.max.y, 5.0 + expectedR, kEpsilon);
}

TEST(OBB2DTest, SATIntersections)
{
    const OBB2D boxA { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 1.0 }, 0.0 };
    const OBB2D boxB { Vec2 { 3.0, 0.0 }, Vec2 { 2.0, 1.0 }, 0.0 };
    const OBB2D boxC { Vec2 { 10.0, 0.0 }, Vec2 { 2.0, 1.0 }, 0.0 };

    // Overlapping along X
    EXPECT_TRUE(boxA.intersects(boxB));
    EXPECT_TRUE(boxB.intersects(boxA));

    // Far separated
    EXPECT_FALSE(boxA.intersects(boxC));

    // Rotated diamond grazing boxA
    const OBB2D diamond { Vec2 { 3.4, 0.0 }, Vec2 { 1.0, 1.0 }, kPi * 0.25 };
    // A extends to 2.0. Diamond center at 3.4 has left extent 3.4 - sqrt(2) = 3.4 - 1.414 = 1.986
    // Since 1.986 < 2.0, they overlap!
    EXPECT_TRUE(boxA.intersects(diamond));

    // Separated diamond
    const OBB2D diamondFar { Vec2 { 4.0, 0.0 }, Vec2 { 1.0, 1.0 }, kPi * 0.25 };
    EXPECT_FALSE(boxA.intersects(diamondFar));

    // Intersects AABB
    const AABB2D aabb { Vec2 { -5.0, -1.0 }, Vec2 { -1.0, 1.0 } };
    EXPECT_TRUE(boxA.intersects(aabb));
    const AABB2D aabbDisjoint { Vec2 { -10.0, -1.0 }, Vec2 { -8.0, 1.0 } };
    EXPECT_FALSE(boxA.intersects(aabbDisjoint));
}

TEST(OBB2DTest, RaycastRotated)
{
    // Rotated 45 degrees diamond box at origin with half extents (1, 1)
    const OBB2D diamond { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 }, kPi * 0.25 };

    // Ray coming from the left along X axis
    const Ray2D ray { Vec2 { -5.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hit { diamond.raycast(ray) };

    ASSERT_TRUE(hit.has_value());
    const double expectedX { -std::sqrt(2.0) };
    EXPECT_NEAR(hit->point.x, expectedX, kEpsilon);
    EXPECT_NEAR(hit->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hit->distance, 5.0 - std::sqrt(2.0), kEpsilon);

    // Normal should point outward along the hit face (-cos(45), -sin(45)) in local -> rotated to world
    EXPECT_LT(hit->normal.x, 0.0);
    EXPECT_NEAR(hit->normal.length(), 1.0, kEpsilon);

    // Miss ray
    const Ray2D rayMiss { Vec2 { -5.0, 5.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    EXPECT_FALSE(diamond.raycast(rayMiss).has_value());
}

// -----------------------------------------------------------------------------
// Circle2D Tests
// -----------------------------------------------------------------------------

TEST(Circle2DTest, ConstructionAndQueries)
{
    const Circle2D defaultCircle {};
    EXPECT_DOUBLE_EQ(defaultCircle.center.x, 0.0);
    EXPECT_DOUBLE_EQ(defaultCircle.center.y, 0.0);
    EXPECT_DOUBLE_EQ(defaultCircle.radius, 0.5);

    const Circle2D circle { Vec2 { 3.0, 4.0 }, 2.0 };
    EXPECT_TRUE(circle.contains(Vec2 { 3.0, 4.0 }));
    EXPECT_TRUE(circle.contains(Vec2 { 5.0, 4.0 }));
    EXPECT_FALSE(circle.contains(Vec2 { 5.1, 4.0 }));

    const Vec2 closestInside { circle.closestPoint(Vec2 { 3.5, 4.0 }) };
    EXPECT_NEAR(closestInside.x, 3.5, kEpsilon);
    EXPECT_NEAR(closestInside.y, 4.0, kEpsilon);

    const Vec2 closestOutside { circle.closestPoint(Vec2 { 10.0, 4.0 }) };
    EXPECT_NEAR(closestOutside.x, 5.0, kEpsilon);
    EXPECT_NEAR(closestOutside.y, 4.0, kEpsilon);

    const AABB2D aabb { circle.computeAABB() };
    EXPECT_NEAR(aabb.min.x, 1.0, kEpsilon);
    EXPECT_NEAR(aabb.min.y, 2.0, kEpsilon);
    EXPECT_NEAR(aabb.max.x, 5.0, kEpsilon);
    EXPECT_NEAR(aabb.max.y, 6.0, kEpsilon);
}

TEST(Circle2DTest, Intersections)
{
    const Circle2D a { Vec2 { 0.0, 0.0 }, 1.0 };
    const Circle2D b { Vec2 { 1.5, 0.0 }, 1.0 };
    const Circle2D c { Vec2 { 3.0, 0.0 }, 0.5 };

    EXPECT_TRUE(a.intersects(b));
    EXPECT_FALSE(a.intersects(c));

    const AABB2D aabbNear { Vec2 { 0.8, -0.5 }, Vec2 { 2.0, 0.5 } };
    EXPECT_TRUE(a.intersects(aabbNear));

    const AABB2D aabbFar { Vec2 { 2.0, 2.0 }, Vec2 { 4.0, 4.0 } };
    EXPECT_FALSE(a.intersects(aabbFar));

    const OBB2D obbNear { Vec2 { 1.5, 0.0 }, Vec2 { 0.6, 0.6 }, kPi * 0.25 };
    EXPECT_TRUE(a.intersects(obbNear));
}

TEST(Circle2DTest, Raycast)
{
    const Circle2D circle { Vec2 { 0.0, 0.0 }, 2.0 };
    const Ray2D rayHit { Vec2 { -5.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hit { circle.raycast(rayHit) };

    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->point.x, -2.0, kEpsilon);
    EXPECT_NEAR(hit->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hit->distance, 3.0, kEpsilon);
    EXPECT_NEAR(hit->normal.x, -1.0, kEpsilon);
    EXPECT_NEAR(hit->normal.y, 0.0, kEpsilon);

    const Ray2D rayMiss { Vec2 { -5.0, 3.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    EXPECT_FALSE(circle.raycast(rayMiss).has_value());
}

// -----------------------------------------------------------------------------
// Capsule2D Tests
// -----------------------------------------------------------------------------

TEST(Capsule2DTest, ConstructionAndQueries)
{
    const Capsule2D capsule { Vec2 { -2.0, 0.0 }, Vec2 { 2.0, 0.0 }, 1.0 };
    EXPECT_DOUBLE_EQ(capsule.segmentLength(), 4.0);

    EXPECT_TRUE(capsule.contains(Vec2 { 0.0, 0.5 }));
    EXPECT_TRUE(capsule.contains(Vec2 { -2.5, 0.0 }));
    EXPECT_TRUE(capsule.contains(Vec2 { 2.8, 0.0 }));
    EXPECT_FALSE(capsule.contains(Vec2 { 3.2, 0.0 }));

    const Vec2 ptNearCenter { capsule.closestPoint(Vec2 { 0.0, 3.0 }) };
    EXPECT_NEAR(ptNearCenter.x, 0.0, kEpsilon);
    EXPECT_NEAR(ptNearCenter.y, 1.0, kEpsilon);

    const AABB2D aabb { capsule.computeAABB() };
    EXPECT_NEAR(aabb.min.x, -3.0, kEpsilon);
    EXPECT_NEAR(aabb.min.y, -1.0, kEpsilon);
    EXPECT_NEAR(aabb.max.x, 3.0, kEpsilon);
    EXPECT_NEAR(aabb.max.y, 1.0, kEpsilon);
}

TEST(Capsule2DTest, Intersections)
{
    const Capsule2D capA { Vec2 { -2.0, 0.0 }, Vec2 { 2.0, 0.0 }, 0.5 };
    const Capsule2D capB { Vec2 { 0.0, -2.0 }, Vec2 { 0.0, 2.0 }, 0.5 };
    const Capsule2D capFar { Vec2 { 0.0, 10.0 }, Vec2 { 2.0, 10.0 }, 0.5 };

    EXPECT_TRUE(capA.intersects(capB));
    EXPECT_FALSE(capA.intersects(capFar));

    const Circle2D circNear { Vec2 { 2.5, 0.0 }, 0.5 };
    EXPECT_TRUE(capA.intersects(circNear));

    const Circle2D circFar { Vec2 { 5.0, 0.0 }, 0.5 };
    EXPECT_FALSE(capA.intersects(circFar));
}

TEST(Capsule2DTest, Raycast)
{
    const Capsule2D capsule { Vec2 { -2.0, 0.0 }, Vec2 { 2.0, 0.0 }, 1.0 };

    const Ray2D rayBody { Vec2 { 0.0, 5.0 }, Vec2 { 0.0, -1.0 }, 10.0 };
    const std::optional<RayHit2D> hitBody { capsule.raycast(rayBody) };
    ASSERT_TRUE(hitBody.has_value());
    EXPECT_NEAR(hitBody->point.x, 0.0, kEpsilon);
    EXPECT_NEAR(hitBody->point.y, 1.0, kEpsilon);
    EXPECT_NEAR(hitBody->distance, 4.0, kEpsilon);

    const Ray2D rayCap { Vec2 { 6.0, 0.0 }, Vec2 { -1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hitCap { capsule.raycast(rayCap) };
    ASSERT_TRUE(hitCap.has_value());
    EXPECT_NEAR(hitCap->point.x, 3.0, kEpsilon);
    EXPECT_NEAR(hitCap->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hitCap->distance, 3.0, kEpsilon);
}

// -----------------------------------------------------------------------------
// Polygon2D Tests
// -----------------------------------------------------------------------------

TEST(Polygon2DTest, ConstructionAndOBBConversion)
{
    const Polygon2D polyFromOBB { Polygon2D::fromOBB(Vec2 { 1.0, 2.0 }, Vec2 { 2.0, 1.0 }, 0.0) };
    EXPECT_EQ(polyFromOBB.vertexCount(), 4U);
    EXPECT_NEAR(polyFromOBB.computeArea(), 8.0, kEpsilon);

    const Vec2 centroid { polyFromOBB.computeCentroid() };
    EXPECT_NEAR(centroid.x, 1.0, kEpsilon);
    EXPECT_NEAR(centroid.y, 2.0, kEpsilon);

    EXPECT_TRUE(polyFromOBB.contains(Vec2 { 1.0, 2.0 }));
    EXPECT_TRUE(polyFromOBB.contains(Vec2 { 2.5, 2.5 }));
    EXPECT_FALSE(polyFromOBB.contains(Vec2 { 4.0, 2.0 }));
}

TEST(Polygon2DTest, EllipseFactoryApproximation)
{
    const Polygon2D ellipse8 { Polygon2D::fromEllipse(Vec2 { 0.0, 0.0 }, 2.0, 1.0, 0.0, 8) };
    EXPECT_EQ(ellipse8.vertexCount(), 8U);

    // Exact ellipse area = pi * a * b = pi * 2 * 1 = 2*pi ~= 6.283
    // Regular 8-gon approximation underestimates by cos(pi/8) factor
    const double area { ellipse8.computeArea() };
    EXPECT_GT(area, 5.5);
    EXPECT_LE(area, 2.0 * kPi);

    const Vec2 centroid { ellipse8.computeCentroid() };
    EXPECT_NEAR(centroid.x, 0.0, kEpsilon);
    EXPECT_NEAR(centroid.y, 0.0, kEpsilon);

    EXPECT_TRUE(ellipse8.contains(Vec2 { 0.0, 0.0 }));
    EXPECT_TRUE(ellipse8.contains(Vec2 { 1.0, 0.5 }));
    EXPECT_FALSE(ellipse8.contains(Vec2 { 3.0, 0.0 }));
}

TEST(Polygon2DTest, IntersectionsAndRaycast)
{
    const Polygon2D boxA { Polygon2D::fromOBB(Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0) };
    const Polygon2D boxB { Polygon2D::fromOBB(Vec2 { 1.5, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0) };
    const Polygon2D boxFar { Polygon2D::fromOBB(Vec2 { 5.0, 0.0 }, Vec2 { 1.0, 1.0 }, 0.0) };

    EXPECT_TRUE(boxA.intersects(boxB));
    EXPECT_FALSE(boxA.intersects(boxFar));

    const Ray2D rayHit { Vec2 { -5.0, 0.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    const std::optional<RayHit2D> hit { boxA.raycast(rayHit) };
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->point.x, -1.0, kEpsilon);
    EXPECT_NEAR(hit->point.y, 0.0, kEpsilon);
    EXPECT_NEAR(hit->distance, 4.0, kEpsilon);

    const Ray2D rayMiss { Vec2 { -5.0, 5.0 }, Vec2 { 1.0, 0.0 }, 10.0 };
    EXPECT_FALSE(boxA.raycast(rayMiss).has_value());
}

} // namespace vehicle_sim::testing
