#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/StanleyController.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Spline2D;
using vehicle_sim::math::StanleyConfig;
using vehicle_sim::math::StanleyController;
using vehicle_sim::math::StanleyState;
using vehicle_sim::math::Vec2;

class StanleyControllerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Straight line path along X axis: (0, 0) -> (100, 0)
        std::vector<Vec2> straightPoints { { 0.0, 0.0 }, { 25.0, 0.0 }, { 50.0, 0.0 }, { 75.0, 0.0 }, { 100.0, 0.0 } };
        mStraightPath.setControlPoints(straightPoints, false);
    }

    Spline2D mStraightPath {};
};

TEST_F(StanleyControllerTest, DefaultConstruction)
{
    const StanleyController controller {};
    EXPECT_DOUBLE_EQ(controller.getConfig().crossTrackGain, 1.2);
    EXPECT_DOUBLE_EQ(controller.getConfig().frontAxleOffset, 1.4);
    EXPECT_EQ(controller.getPath(), nullptr);
}

TEST_F(StanleyControllerTest, FrontAxleCalculation)
{
    const StanleyController controller {};
    // At heading 0 (along X), front axle is at X + Lf, Y
    const Vec2 cog { 10.0, 5.0 };
    const Vec2 frontAxle { controller.getFrontAxle(cog, 0.0) };
    EXPECT_NEAR(frontAxle.x, 11.4, 1e-9);
    EXPECT_NEAR(frontAxle.y, 5.0, 1e-9);

    // At heading pi/2 (along Y), front axle is at X, Y + Lf
    const Vec2 frontAxleY { controller.getFrontAxle(cog, 1.5707963267948966) };
    EXPECT_NEAR(frontAxleY.x, 10.0, 1e-5);
    EXPECT_NEAR(frontAxleY.y, 6.4, 1e-5);
}

TEST_F(StanleyControllerTest, StraightPathOffsetSteering)
{
    StanleyController controller {};
    controller.setPath(&mStraightPath);

    // Vehicle at y = +2.0 (left of path), heading = 0 (parallel to path), speed = 10 m/s
    // Should steer right (negative steering angle) to return to path
    const Vec2 cog { 20.0, 2.0 };
    const StanleyState state { controller.computeSteer(cog, 0.0, 10.0, 0.0) };

    EXPECT_GT(state.crossTrackError, 1.9); // offset ~2.0
    EXPECT_NEAR(state.headingError, 0.0, 1e-4);
    EXPECT_LT(state.steerAngle, 0.0); // steers right towards y = 0
}

TEST_F(StanleyControllerTest, RightOffsetSteering)
{
    StanleyController controller {};
    controller.setPath(&mStraightPath);

    // Vehicle at y = -2.0 (right of path), heading = 0, speed = 10 m/s
    // Should steer left (positive steering angle) to return to path
    const Vec2 cog { 20.0, -2.0 };
    const StanleyState state { controller.computeSteer(cog, 0.0, 10.0, 0.0) };

    EXPECT_LT(state.crossTrackError, -1.9);
    EXPECT_NEAR(state.headingError, 0.0, 1e-4);
    EXPECT_GT(state.steerAngle, 0.0); // steers left towards y = 0
}

TEST_F(StanleyControllerTest, StandstillAndLowSpeedStability)
{
    StanleyController controller {};
    controller.setPath(&mStraightPath);

    const Vec2 cog { 20.0, 5.0 };
    // Test at zero speed
    const StanleyState zeroState { controller.computeSteer(cog, 0.0, 0.0, 0.0) };
    EXPECT_TRUE(std::isfinite(zeroState.steerAngle));
    EXPECT_LE(std::abs(zeroState.steerAngle), controller.getConfig().maxSteerAngle);

    // Test at very small speed
    const StanleyState lowState { controller.computeSteer(cog, 0.0, 0.05, 0.0) };
    EXPECT_TRUE(std::isfinite(lowState.steerAngle));
    EXPECT_LE(std::abs(lowState.steerAngle), controller.getConfig().maxSteerAngle);
}

TEST_F(StanleyControllerTest, ReverseMotionPolarity)
{
    StanleyController controller {};
    controller.setPath(&mStraightPath);

    // Vehicle reversing at speed = -5.0 m/s, facing backwards (heading = pi)
    // At y = 2.0 (offset), should reverse steer correctly
    const Vec2 cog { 50.0, 2.0 };
    const StanleyState revState { controller.computeSteer(cog, 3.141592653589793, -5.0, 0.0) };
    EXPECT_TRUE(std::isfinite(revState.steerAngle));
    EXPECT_LE(std::abs(revState.steerAngle), controller.getConfig().maxSteerAngle);
}

TEST_F(StanleyControllerTest, DampingTermReducesOscillation)
{
    StanleyConfig config {};
    config.dampingGain = 0.5;
    StanleyController controller { config };
    controller.setPath(&mStraightPath);

    const Vec2 cog { 20.0, 0.0 }; // On path
    // With positive yaw rate (turning left), damping should push steering right (negative)
    const StanleyState dampedState { controller.computeSteer(cog, 0.0, 10.0, 1.0) };
    EXPECT_LT(dampedState.steerAngle, 0.0);
}

} // namespace vehicle_sim::testing
