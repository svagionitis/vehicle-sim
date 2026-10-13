#include <gtest/gtest.h>

#include "apps/EarthPatrol/GameApp.h"
#include "apps/EarthPatrol/GameState.h"
#include "apps/EarthPatrol/PixelFont.h"

namespace earth_patrol {

TEST(GameAppTest, DefaultStateIsTitleScreen)
{
    GameAppConfig config {};
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
    GameAppConfig config {};
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
    GameAppConfig config {};
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
    GameAppConfig config {};
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
    GameAppConfig config {};
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
    GameAppConfig config {};
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

TEST(GameAppTest, CameraMaintainsGroundConformityDuringJump)
{
    GameAppConfig config {};
    config.headless = true;
    GameApp app { config };
    ASSERT_TRUE(app.initialize());

    app.startNewGame();

    // Settle rover onto terrain suspension (0.2s)
    for (int i { 0 }; i < 15; ++i) {
        app.updatePhysics(0.016);
    }

    // Trigger jump impulse
    app.getRover().triggerJump();

    // Step physics forward through ascent phase (0.4s)
    for (int i { 0 }; i < 25; ++i) {
        app.updatePhysics(0.016);
    }

    // Rover chassis should be airborne above ground
    EXPECT_GT(app.getRoverBody().getPosition().y, 2.5);

    // Camera vertical anchor should conform to ground elevation (<= 1.5m),
    // not follow the jumping rover into the air and cause background voids.
    EXPECT_LE(app.getCamera().getPosition().y, 1.5);
}

TEST(ParallaxRendererTest, BackgroundCoversEntireViewportHeight)
{
    SDL_Surface* surface { SDL_CreateRGBSurfaceWithFormat(0, 320, 240, 32, SDL_PIXELFORMAT_RGBA8888) };
    ASSERT_NE(surface, nullptr);
    SDL_Renderer* renderer { SDL_CreateSoftwareRenderer(surface) };
    ASSERT_NE(renderer, nullptr);

    // Clear surface with transparent / empty (alpha = 0)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    CameraConfig camCfg {};
    camCfg.viewportWidth = 320;
    camCfg.viewportHeight = 240;
    Camera2D camera { camCfg };

    ParallaxRenderer parallax {};
    parallax.render(renderer, camera, 0);

    // Query bottom pixel at y = 239
    const auto* pixels { static_cast<const Uint32*>(surface->pixels) };
    const Uint32 bottomPixel { pixels[(239 * 320) + 160] };

    Uint8 r { 0 };
    Uint8 g { 0 };
    Uint8 b { 0 };
    Uint8 a { 0 };
    SDL_GetRGBA(bottomPixel, surface->format, &r, &g, &b, &a);

    SDL_DestroyRenderer(renderer);
    SDL_FreeSurface(surface);

    EXPECT_EQ(a, 255);
    EXPECT_GT(static_cast<int>(r) + static_cast<int>(g) + static_cast<int>(b), 0);
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
    ParticleSystem ps {};
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
    Camera2D cam {};
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
    AudioSystem audio {};
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

TEST(DynamicObstacleTest, KinematicUpdateAndTerrainFollowing)
{
    CourseManager course {};
    course.buildCourse(1); // Desert

    ObstacleManager obs {};
    obs.generateObstacles(course, 1);

    const auto& dyns { obs.getDynamicObstacles() };
    ASSERT_FALSE(dyns.empty());

    // Check that we have a rolling boulder
    bool foundBoulder { false };
    double initialX { 0.0 };
    double initialAngle { 0.0 };
    for (const auto& d : dyns) {
        if (d.type == DynamicObstacleType::RollingBoulder) {
            foundBoulder = true;
            initialX = d.position.x;
            initialAngle = d.rotationAngle;
            break;
        }
    }
    ASSERT_TRUE(foundBoulder);

    constexpr double dt { 0.1 };
    obs.updateDynamic(dt, course.getHeightfield());

    const auto& updatedDyns { obs.getDynamicObstacles() };
    for (const auto& d : updatedDyns) {
        if (d.type == DynamicObstacleType::RollingBoulder) {
            // Velocity is negative (moving towards player), so X decreases
            EXPECT_LT(d.position.x, initialX);
            // Angle should change according to omega = -vx / r
            EXPECT_GT(d.rotationAngle, initialAngle);
            // Y position conforms to terrain height + radius
            const double expectedY { course.getHeightfield().getHeight(d.position.x) + d.radius };
            EXPECT_NEAR(d.position.y, expectedY, 1e-4);
            break;
        }
    }
}

TEST(DynamicObstacleTest, PatrolDroneBoundaryReversal)
{
    CourseManager course {};
    course.buildCourse(0); // Highway

    ObstacleManager obs {};
    obs.generateObstacles(course, 0);

    const auto& dyns { obs.getDynamicObstacles() };
    ASSERT_FALSE(dyns.empty());

    bool foundDrone { false };
    for (const auto& d : dyns) {
        if (d.type == DynamicObstacleType::PatrolDrone) {
            foundDrone = true;
            break;
        }
    }
    ASSERT_TRUE(foundDrone);

    // Update for 10 seconds to ensure drone hits patrol boundary and reverses
    for (int step { 0 }; step < 100; ++step) {
        obs.updateDynamic(0.1, course.getHeightfield());
    }

    const auto& updatedDyns { obs.getDynamicObstacles() };
    for (const auto& d : updatedDyns) {
        if (d.type == DynamicObstacleType::PatrolDrone) {
            // Drone stays within [patrolMinX, patrolMaxX]
            EXPECT_GE(d.position.x, d.patrolMinX - 0.5);
            EXPECT_LE(d.position.x, d.patrolMaxX + 0.5);
        }
    }
}

TEST(DynamicObstacleTest, CollisionDetectionAndClearCount)
{
    CourseManager course {};
    course.buildCourse(1);

    ObstacleManager obs {};
    obs.generateObstacles(course, 1);

    const auto& dyns { obs.getDynamicObstacles() };
    ASSERT_FALSE(dyns.empty());
    const auto targetPos { dyns.front().position };

    vehicle_sim::physics::RigidBodyConfig bodyConfig {};
    bodyConfig.position = targetPos;
    vehicle_sim::physics::RigidBody2D body { bodyConfig };

    bool fatal { false };
    EXPECT_TRUE(obs.checkDynamicColl(body, fatal));
    EXPECT_TRUE(fatal);

    // Move rover far past the obstacle
    body.setPosition(vehicle_sim::math::Vec2 { targetPos.x + 10.0, targetPos.y });
    bool clearFatal { false };
    EXPECT_FALSE(obs.checkDynamicColl(body, clearFatal));
    EXPECT_GT(obs.getClearedCount(), 0);
}

} // namespace earth_patrol
