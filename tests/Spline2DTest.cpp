#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/Spline2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Spline2D;
using vehicle_sim::math::SplineClosestPoint;
using vehicle_sim::math::SplineKnotMode;
using vehicle_sim::math::SplineSample2D;
using vehicle_sim::math::Vec2;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kTolerance { 1e-4 };
inline constexpr double kFineTolerance { 1e-8 };

TEST(Spline2DTest, DefaultConstructionAndEmptyState)
{
    const Spline2D spline {};
    EXPECT_EQ(spline.getPointCount(), 0U);
    EXPECT_EQ(spline.getSegmentCount(), 0U);
    EXPECT_FALSE(spline.isClosed());
    EXPECT_DOUBLE_EQ(spline.getTotalLength(), 0.0);

    const Vec2 pos = spline.evaluate(0.5);
    EXPECT_DOUBLE_EQ(pos.x, 0.0);
    EXPECT_DOUBLE_EQ(pos.y, 0.0);

    const SplineSample2D sample = spline.evaluateSample(0.5);
    EXPECT_DOUBLE_EQ(sample.point.x, 0.0);
    EXPECT_DOUBLE_EQ(sample.point.y, 0.0);
    EXPECT_DOUBLE_EQ(sample.curvature, 0.0);

    const auto pts = spline.discretize(1.0);
    EXPECT_TRUE(pts.empty());
}

TEST(Spline2DTest, SinglePointDegenerateCase)
{
    const std::vector<Vec2> singlePoint { Vec2 { 5.0, -3.0 } };
    const Spline2D spline { singlePoint };

    EXPECT_EQ(spline.getPointCount(), 1U);
    EXPECT_EQ(spline.getSegmentCount(), 0U);
    EXPECT_DOUBLE_EQ(spline.getTotalLength(), 0.0);

    const Vec2 pt = spline.evaluate(1.0);
    EXPECT_DOUBLE_EQ(pt.x, 5.0);
    EXPECT_DOUBLE_EQ(pt.y, -3.0);

    const auto closest = spline.findClosest(Vec2 { 10.0, -3.0 });
    EXPECT_DOUBLE_EQ(closest.point.x, 5.0);
    EXPECT_DOUBLE_EQ(closest.point.y, -3.0);
    EXPECT_DOUBLE_EQ(closest.distance, 5.0);
}

TEST(Spline2DTest, StraightLineCollinearInvariants)
{
    const std::vector<Vec2> linePoints { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 0.0 }, Vec2 { 20.0, 0.0 },
        Vec2 { 30.0, 0.0 } };

    const Spline2D spline { linePoints, false, SplineKnotMode::Centripetal };
    EXPECT_EQ(spline.getPointCount(), 4U);
    EXPECT_EQ(spline.getSegmentCount(), 3U);
    EXPECT_NEAR(spline.getTotalLength(), 30.0, kTolerance);

    // Midpoint evaluation at parameter t = 1.5 should be (15, 0)
    const Vec2 mid = spline.evaluate(1.5);
    EXPECT_NEAR(mid.x, 15.0, kTolerance);
    EXPECT_NEAR(mid.y, 0.0, kTolerance);

    // Tangent must be unit X, normal unit Y, curvature zero
    for (double t { 0.0 }; t <= 3.0; t += 0.5) {
        const SplineSample2D sample = spline.evaluateSample(t);
        EXPECT_NEAR(sample.tangent.x, 1.0, kTolerance);
        EXPECT_NEAR(sample.tangent.y, 0.0, kTolerance);
        EXPECT_NEAR(sample.normal.x, 0.0, kTolerance);
        EXPECT_NEAR(sample.normal.y, 1.0, kTolerance);
        EXPECT_NEAR(sample.curvature, 0.0, kTolerance);
    }

    // Arc-length evaluation at distance s = 25.0 must yield x = 25.0
    const SplineSample2D distSample = spline.evaluateAtDistance(25.0);
    EXPECT_NEAR(distSample.point.x, 25.0, kTolerance);
    EXPECT_NEAR(distSample.point.y, 0.0, kTolerance);
}

TEST(Spline2DTest, ClosedLoopCircleCurvatureInvariants)
{
    constexpr double kRadius { 20.0 };
    constexpr std::size_t kNumNodes { 8 };
    std::vector<Vec2> circleNodes {};
    circleNodes.reserve(kNumNodes);

    for (std::size_t i { 0 }; i < kNumNodes; ++i) {
        const double theta { (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(kNumNodes) };
        circleNodes.emplace_back(kRadius * std::cos(theta), kRadius * std::sin(theta));
    }

    const Spline2D spline { circleNodes, true, SplineKnotMode::Centripetal };
    EXPECT_TRUE(spline.isClosed());
    EXPECT_EQ(spline.getSegmentCount(), kNumNodes);

    // Closed loop circumference should be close to 2 * pi * R
    const double expectedCircumference { 2.0 * kPi * kRadius };
    EXPECT_NEAR(spline.getTotalLength(), expectedCircumference, 1.0); // Within 1.5% for 8 nodes

    // Every sample point should lie close to radius R
    for (double t { 0.0 }; t < static_cast<double>(kNumNodes); t += 0.25) {
        const SplineSample2D sample = spline.evaluateSample(t);
        const double distFromOrigin = sample.point.length();
        EXPECT_NEAR(distFromOrigin, kRadius, 0.8);

        // Curvature on circle of radius R is nominally 1 / R = 0.05
        EXPECT_GT(sample.curvature, 0.035);
        EXPECT_LT(sample.curvature, 0.085);

        // Tangent and normal must remain orthogonal and unit length
        EXPECT_NEAR(sample.tangent.length(), 1.0, kFineTolerance);
        EXPECT_NEAR(sample.normal.length(), 1.0, kFineTolerance);
        EXPECT_NEAR(sample.tangent.dot(sample.normal), 0.0, kFineTolerance);
    }
}

TEST(Spline2DTest, KnotModesComparison)
{
    const std::vector<Vec2> nodes { Vec2 { 0.0, 0.0 }, Vec2 { 5.0, 12.0 }, Vec2 { 25.0, 15.0 }, Vec2 { 30.0, 0.0 } };

    const Spline2D centripetal { nodes, false, SplineKnotMode::Centripetal };
    const Spline2D uniform { nodes, false, SplineKnotMode::Uniform };
    const Spline2D chordal { nodes, false, SplineKnotMode::Chordal };

    EXPECT_EQ(centripetal.getKnotMode(), SplineKnotMode::Centripetal);
    EXPECT_EQ(uniform.getKnotMode(), SplineKnotMode::Uniform);
    EXPECT_EQ(chordal.getKnotMode(), SplineKnotMode::Chordal);

    // All should produce positive length and valid samples
    EXPECT_GT(centripetal.getTotalLength(), 30.0);
    EXPECT_GT(uniform.getTotalLength(), 30.0);
    EXPECT_GT(chordal.getTotalLength(), 30.0);

    const auto pCentripetal = centripetal.evaluate(1.5);
    const auto pUniform = uniform.evaluate(1.5);
    const auto pChordal = chordal.evaluate(1.5);

    EXPECT_FALSE(std::isnan(pCentripetal.x) || std::isnan(pCentripetal.y));
    EXPECT_FALSE(std::isnan(pUniform.x) || std::isnan(pUniform.y));
    EXPECT_FALSE(std::isnan(pChordal.x) || std::isnan(pChordal.y));
}

TEST(Spline2DTest, ArcLengthMonotonicityAndDistanceEvaluation)
{
    const std::vector<Vec2> sCurve { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 10.0 }, Vec2 { 20.0, -10.0 }, Vec2 { 30.0, 0.0 } };

    const Spline2D spline { sCurve, false, SplineKnotMode::Centripetal };
    const double totalLen { spline.getTotalLength() };
    EXPECT_GT(totalLen, 30.0);

    // Evaluating at distance s should return sample with matching distance
    constexpr std::size_t kSteps { 20 };
    double lastDist { -1.0 };

    for (std::size_t i { 0 }; i <= kSteps; ++i) {
        const double s { (static_cast<double>(i) * totalLen) / static_cast<double>(kSteps) };
        const SplineSample2D sample = spline.evaluateAtDistance(s);

        EXPECT_NEAR(sample.distance, s, 0.05);
        EXPECT_GT(sample.distance, lastDist);
        lastDist = sample.distance;
    }
}

TEST(Spline2DTest, ClosestPointProjection)
{
    const std::vector<Vec2> path { Vec2 { 0.0, 0.0 }, Vec2 { 20.0, 0.0 }, Vec2 { 40.0, 0.0 } };

    const Spline2D spline { path, false, SplineKnotMode::Centripetal };

    // Point located at (15, 5) is 5 units to the left of the centerline at x = 15
    const Vec2 query { 15.0, 5.0 };
    const SplineClosestPoint closest = spline.findClosest(query);

    EXPECT_NEAR(closest.point.x, 15.0, kTolerance);
    EXPECT_NEAR(closest.point.y, 0.0, kTolerance);
    EXPECT_NEAR(closest.distance, 5.0, kTolerance);
    EXPECT_NEAR(closest.arcLength, 15.0, kTolerance);
    EXPECT_GT(closest.signedOffset, 0.0); // Positive indicates left of tangent
    EXPECT_NEAR(closest.signedOffset, 5.0, kTolerance);

    // Point located at (25, -3) is 3 units to the right
    const Vec2 queryRight { 25.0, -3.0 };
    const SplineClosestPoint closestRight = spline.findClosest(queryRight);

    EXPECT_NEAR(closestRight.point.x, 25.0, kTolerance);
    EXPECT_NEAR(closestRight.point.y, 0.0, kTolerance);
    EXPECT_NEAR(closestRight.distance, 3.0, kTolerance);
    EXPECT_LT(closestRight.signedOffset, 0.0); // Negative indicates right of tangent
    EXPECT_NEAR(closestRight.signedOffset, -3.0, kTolerance);
}

TEST(Spline2DTest, DiscretizationStepSize)
{
    const std::vector<Vec2> path { Vec2 { 0.0, 0.0 }, Vec2 { 25.0, 0.0 }, Vec2 { 50.0, 0.0 } };

    const Spline2D spline { path, false, SplineKnotMode::Centripetal };
    constexpr double kStep { 5.0 };
    const auto polyline = spline.discretize(kStep);

    // Path is 50m long, stepping by 5m should yield ~11 points
    EXPECT_GE(polyline.size(), 10U);
    EXPECT_LE(polyline.size(), 12U);

    EXPECT_NEAR(polyline.front().x, 0.0, kTolerance);
    EXPECT_NEAR(polyline.back().x, 50.0, kTolerance);

    // Check adjacent vertex spacing is approximately kStep
    for (std::size_t i { 0 }; i + 1 < polyline.size(); ++i) {
        const double segLen = (polyline[i + 1] - polyline[i]).length();
        EXPECT_NEAR(segLen, kStep, 0.2);
    }
}

TEST(Spline2DTest, ClearAndReconfiguration)
{
    Spline2D spline {};
    EXPECT_EQ(spline.getPointCount(), 0U);

    const std::vector<Vec2> pts1 { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 10.0 } };
    spline.setControlPoints(pts1, false);
    EXPECT_EQ(spline.getPointCount(), 2U);
    EXPECT_GT(spline.getTotalLength(), 0.0);

    spline.clear();
    EXPECT_EQ(spline.getPointCount(), 0U);
    EXPECT_DOUBLE_EQ(spline.getTotalLength(), 0.0);

    const std::vector<Vec2> pts2 { Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 0.0 }, Vec2 { 20.0, 0.0 } };
    spline.setControlPoints(pts2, false);
    EXPECT_EQ(spline.getPointCount(), 3U);
    EXPECT_NEAR(spline.getTotalLength(), 20.0, kTolerance);
}

} // namespace vehicle_sim::testing
