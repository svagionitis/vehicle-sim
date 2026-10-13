#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/dynamics/TrajectoryTracker2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::AutopilotOutput;
using vehicle_sim::dynamics::TrajectoryTracker2D;
using vehicle_sim::math::Spline2D;
using vehicle_sim::math::Vec2;

class TrajectoryTrackerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::vector<Vec2> waypoints { { 0.0, 0.0 }, { 50.0, 0.0 }, { 100.0, 10.0 }, { 150.0, 40.0 }, { 150.0, 100.0 } };
        mPath.setControlPoints(waypoints, false);
    }

    Spline2D mPath {};
};

TEST_F(TrajectoryTrackerTest, DefaultConstruction)
{
    const TrajectoryTracker2D tracker {};
    EXPECT_DOUBLE_EQ(tracker.getStanley().getConfig().crossTrackGain, 1.2);
    EXPECT_DOUBLE_EQ(tracker.getProfiler().getLimits().maxCruiseSpeed, 40.0);
}

TEST_F(TrajectoryTrackerTest, SetTrajectoryBuildsProfile)
{
    TrajectoryTracker2D tracker {};
    EXPECT_TRUE(tracker.setTrajectory(&mPath));
    EXPECT_FALSE(tracker.getProfiler().getMilestones().empty());
}

TEST_F(TrajectoryTrackerTest, AutopilotStepOutputBounds)
{
    TrajectoryTracker2D tracker {};
    ASSERT_TRUE(tracker.setTrajectory(&mPath));

    // Vehicle at start, speed = 10 m/s
    const Vec2 cog { 0.0, 1.0 }; // 1m offset from path
    const double dt { 0.02 }; // 50 Hz update

    const AutopilotOutput out { tracker.update(dt, cog, 0.0, 10.0, 0.0) };

    EXPECT_GE(out.normalizedSteer, -1.0);
    EXPECT_LE(out.normalizedSteer, 1.0);
    EXPECT_GE(out.throttle, 0.0);
    EXPECT_LE(out.throttle, 1.0);
    EXPECT_GE(out.brake, 0.0);
    EXPECT_LE(out.brake, 1.0);
    EXPECT_GT(out.targetSpeed, 0.0);
    EXPECT_TRUE(std::isfinite(out.steerAngleRad));
}

TEST_F(TrajectoryTrackerTest, SteerSlewRateLimiting)
{
    TrajectoryTracker2D tracker {};
    ASSERT_TRUE(tracker.setTrajectory(&mPath));

    // Large offset forcing immediate demand for maximum steering
    const Vec2 cog { 0.0, 20.0 };
    const double dt { 0.01 };

    const AutopilotOutput step1 { tracker.update(dt, cog, 0.0, 15.0, 0.0) };
    const AutopilotOutput step2 { tracker.update(dt, cog, 0.0, 15.0, 0.0) };

    // Angular change between two consecutive 10ms steps must be bounded by steer slew rate
    const double steerDelta { std::abs(step2.steerAngleRad - step1.steerAngleRad) };
    EXPECT_LE(steerDelta, 0.05); // slew rate * dt = 2.5 * 0.01 = 0.025 rad
}

} // namespace vehicle_sim::testing
