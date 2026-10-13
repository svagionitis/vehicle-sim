#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/SpeedProfiler2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::SpeedMilestone;
using vehicle_sim::math::SpeedProfileLimits;
using vehicle_sim::math::SpeedProfiler2D;
using vehicle_sim::math::Spline2D;
using vehicle_sim::math::Vec2;

class SpeedProfilerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // 1. Straight path
        std::vector<Vec2> straightPts { { 0.0, 0.0 }, { 50.0, 0.0 }, { 100.0, 0.0 }, { 150.0, 0.0 }, { 200.0, 0.0 } };
        mStraightPath.setControlPoints(straightPts, false);

        // 2. Path with a 90-degree corner
        std::vector<Vec2> cornerPts { { 0.0, 0.0 }, { 100.0, 0.0 }, { 150.0, 10.0 }, { 180.0, 50.0 }, { 190.0, 100.0 },
            { 190.0, 200.0 } };
        mCornerPath.setControlPoints(cornerPts, false);
    }

    Spline2D mStraightPath {};
    Spline2D mCornerPath {};
};

TEST_F(SpeedProfilerTest, DefaultConstruction)
{
    const SpeedProfiler2D profiler {};
    EXPECT_DOUBLE_EQ(profiler.getLimits().maxCruiseSpeed, 40.0);
    EXPECT_DOUBLE_EQ(profiler.getLimits().maxLatAccel, 9.81);
    EXPECT_TRUE(profiler.getMilestones().empty());
}

TEST_F(SpeedProfilerTest, StraightPathCruising)
{
    SpeedProfileLimits limits {};
    limits.maxCruiseSpeed = 30.0;
    limits.sampleStep = 5.0;

    SpeedProfiler2D profiler { limits };
    EXPECT_TRUE(profiler.buildProfile(mStraightPath));
    EXPECT_FALSE(profiler.getMilestones().empty());

    // Straight path curvature is ~0, so speed should equal maxCruiseSpeed across all milestones
    for (const auto& milestone : profiler.getMilestones()) {
        EXPECT_NEAR(milestone.targetSpeed, 30.0, 0.5);
    }

    // Midpoint query
    EXPECT_NEAR(profiler.getSpeedAt(100.0), 30.0, 0.5);
}

TEST_F(SpeedProfilerTest, CornerApexDeceleration)
{
    SpeedProfileLimits limits {};
    limits.maxCruiseSpeed = 40.0;
    limits.maxLatAccel = 9.81;
    limits.sampleStep = 2.0;

    SpeedProfiler2D profiler { limits };
    EXPECT_TRUE(profiler.buildProfile(mCornerPath));

    // The corner section must have a lower speed than the straightaway
    double minSpeed { 100.0 };
    double maxSpeed { 0.0 };

    for (const auto& milestone : profiler.getMilestones()) {
        minSpeed = std::min(minSpeed, milestone.targetSpeed);
        maxSpeed = std::max(maxSpeed, milestone.targetSpeed);
    }

    EXPECT_LT(minSpeed, maxSpeed);
    EXPECT_LT(minSpeed, limits.maxCruiseSpeed);
}

TEST_F(SpeedProfilerTest, BrakingLookaheadProgression)
{
    SpeedProfileLimits limits {};
    limits.maxCruiseSpeed = 35.0;
    limits.maxBrakingDecel = 6.0;
    limits.sampleStep = 2.0;

    SpeedProfiler2D profiler { limits };
    EXPECT_TRUE(profiler.buildProfile(mCornerPath));

    const auto& milestones = profiler.getMilestones();
    ASSERT_GT(milestones.size(), 10U);

    // Verify deceleration constraint between adjacent milestones
    for (std::size_t i { 0 }; i < milestones.size() - 1; ++i) {
        const double ds { milestones[i + 1].distance - milestones[i].distance };
        const double vCurr { milestones[i].targetSpeed };
        const double vNext { milestones[i + 1].targetSpeed };

        // If braking (vCurr > vNext), deceleration must not exceed maxBrakingDecel with tolerance
        if (vCurr > vNext) {
            const double allowedDecelSq { (vNext * vNext) + (2.0 * limits.maxBrakingDecel * ds) + 1.0 };
            EXPECT_LE(vCurr * vCurr, allowedDecelSq);
        }
    }
}

TEST_F(SpeedProfilerTest, ClosedLoopContinuity)
{
    // Circular closed track
    std::vector<Vec2> circlePts { { 0.0, 50.0 }, { 50.0, 0.0 }, { 0.0, -50.0 }, { -50.0, 0.0 } };
    Spline2D circlePath { circlePts, true };

    SpeedProfileLimits limits {};
    limits.maxCruiseSpeed = 30.0;
    limits.sampleStep = 5.0;

    SpeedProfiler2D profiler { limits };
    EXPECT_TRUE(profiler.buildProfile(circlePath));

    // Query speed at s = 0 and s = totalLength, must be close
    const double speedStart { profiler.getSpeedAt(0.0) };
    const double speedEnd { profiler.getSpeedAt(circlePath.getTotalLength() - 0.1) };
    EXPECT_NEAR(speedStart, speedEnd, 1.0);
}

} // namespace vehicle_sim::testing
