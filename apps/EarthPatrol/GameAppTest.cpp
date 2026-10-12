#include <gtest/gtest.h>

#include "apps/EarthPatrol/GameApp.h"
#include "apps/EarthPatrol/GameState.h"
#include "apps/EarthPatrol/PixelFont.h"

namespace earth_patrol {

TEST(GameAppTest, DefaultStateIsTitleScreen)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };

    EXPECT_TRUE(app.initialize());
    EXPECT_EQ(app.getState(), GameState::TitleScreen);
    EXPECT_EQ(app.getScore(), 0);
    EXPECT_EQ(app.getStage(), 1);
    EXPECT_EQ(app.getLives(), 3);
    EXPECT_FALSE(app.isCrashed());
}

TEST(GameAppTest, StartNewGameTransitionsToPlaying)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();
    EXPECT_EQ(app.getState(), GameState::Playing);
    EXPECT_EQ(app.getScore(), 0);
    EXPECT_EQ(app.getStage(), 1);
    EXPECT_EQ(app.getBiomeIndex(), 0);
    EXPECT_FALSE(app.isCrashed());
}

TEST(GameAppTest, AdvanceStageCyclesBiomes)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();
    EXPECT_EQ(app.getBiomeIndex(), 0);

    app.advanceStage();
    EXPECT_EQ(app.getStage(), 2);
    EXPECT_EQ(app.getBiomeIndex(), 1);

    app.advanceStage();
    EXPECT_EQ(app.getStage(), 3);
    EXPECT_EQ(app.getBiomeIndex(), 2);

    app.advanceStage();
    EXPECT_EQ(app.getStage(), 4);
    EXPECT_EQ(app.getBiomeIndex(), 0); // Cycles back to Highway
}

TEST(GameAppTest, CheckpointCrossingAwardsPoints)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();
    EXPECT_EQ(app.getScore(), 0);

    // Teleport rover past Point A (x = 100m)
    app.getRoverBody().setPosition(vehicle_sim::math::Vec2 { 105.0, 1.20 });
    app.updatePhysics(0.016);

    EXPECT_GE(app.getScore(), 1000);
}

TEST(GameAppTest, FatalCollisionCausesCrashAndReducesLife)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();
    EXPECT_EQ(app.getLives(), 3);

    // Drop rover into deep pit
    app.getRoverBody().setPosition(vehicle_sim::math::Vec2 { 25.0, -3.0 });
    app.updatePhysics(0.016);

    EXPECT_TRUE(app.isCrashed());
    EXPECT_EQ(app.getLives(), 2);
    EXPECT_EQ(app.getState(), GameState::Crashed);
}

TEST(GameAppTest, CourseClearanceTransitionsToStageClear)
{
    GameAppConfig config { };
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();

    // Move rover past finish line Milestone E (500m)
    app.getRoverBody().setPosition(vehicle_sim::math::Vec2 { 505.0, 1.20 });
    app.updatePhysics(0.016);

    EXPECT_EQ(app.getState(), GameState::StageClear);
    EXPECT_GE(app.getScore(), 5000);
}

TEST(PixelFontTest, TextWidthCalculations)
{
    EXPECT_EQ(PixelFont::getTextWidth("", 2), 0);
    // Each char is (3 + 1) * scale = 4 * scale = 8 px at scale 2
    EXPECT_EQ(PixelFont::getTextWidth("EARTH", 2), 5 * 8);
    EXPECT_EQ(PixelFont::getTextWidth("PATROL", 3), 6 * 12);
}

} // namespace earth_patrol
