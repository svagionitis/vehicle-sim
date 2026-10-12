#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/MathUtils.h"
#include "libs/VehicleSimCore/math/Transform2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Angle2D;
using vehicle_sim::math::Mat2;
using vehicle_sim::math::MathUtils;
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

} // namespace vehicle_sim::testing
