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

TEST(ParticleSystemTest, EmissionAndEulerDecay)
{
    ParticleSystem ps { };
    EXPECT_EQ(ps.getActiveCount(), 0);

    ps.emitTireDust(vehicle_sim::math::Vec2 { 10.0, 1.0 }, 0, 4);
    EXPECT_GE(ps.getActiveCount(), 4);

    ps.emitCrashExplosion(vehicle_sim::math::Vec2 { 10.0, 1.0 });
    EXPECT_GE(ps.getActiveCount(), 20);

    // Step physics forward past max lifetime (1.5s)
    ps.update(2.0);
    EXPECT_EQ(ps.getActiveCount(), 0);
}

TEST(CameraShakeTest, TraumaActivationAndDecay)
{
    Camera2D cam { };
    EXPECT_DOUBLE_EQ(cam.getShakeOffset().x, 0.0);
    EXPECT_DOUBLE_EQ(cam.getShakeOffset().y, 0.0);

    cam.addScreenShake(0.5, 0.4);
    cam.update(vehicle_sim::math::Vec2 { 0.0, 0.0 }, 10.0, 0.05);

    // During active trauma, offset is non-zero
    const auto offset { cam.getShakeOffset() };
    EXPECT_TRUE(offset.x != 0.0 || offset.y != 0.0);

    // After full duration expires, offset resets to zero
    cam.update(vehicle_sim::math::Vec2 { 0.0, 0.0 }, 10.0, 0.50);
    EXPECT_DOUBLE_EQ(cam.getShakeOffset().x, 0.0);
    EXPECT_DOUBLE_EQ(cam.getShakeOffset().y, 0.0);
}

TEST(AudioSystemTest, HeadlessInitializationAndMuteToggle)
{
    AudioSystem audio { };
    EXPECT_FALSE(audio.isInitialized());

    EXPECT_TRUE(audio.initialize(true)); // Headless mode
    EXPECT_TRUE(audio.isInitialized());
    EXPECT_FALSE(audio.isMuted());

    audio.setMuted(true);
    EXPECT_TRUE(audio.isMuted());

    // Safe to trigger audio hooks without throwing or crashing
    audio.playJump();
    audio.playLanding();
    audio.playCrash();
    audio.playMilestone();
    audio.playStageClear();
}

} // namespace earth_patrol
