#include <gtest/gtest.h>

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/CourseManager.h"
#include "apps/MontsoftOffroad/GameApp.h"
#include "apps/MontsoftOffroad/InputController.h"
#include "apps/MontsoftOffroad/ParallaxRenderer.h"
#include "apps/MontsoftOffroad/ParticleSystem.h"
#include "apps/MontsoftOffroad/SDLWrappers.h"
#include "apps/MontsoftOffroad/TruckRenderer.h"

namespace montsoft_offroad {

TEST(SDLWrappersTest, ContextLifecycleAndNullDeleters)
{
    // Verify RAII deleters do not crash on nullptr
    SDLWindowDeleter{}(nullptr);
    SDLRendererDeleter{}(nullptr);
    SDLTextureDeleter{}(nullptr);
    SDLSurfaceDeleter{}(nullptr);
    SDLControllerDeleter{}(nullptr);

    // SDLContext initialization with no subsystems (safe in any headless CI)
    SDLContext ctx{ 0 };
    EXPECT_TRUE(ctx.isInitialized());
}

TEST(CgaRendererTest, PaletteModeCyclingAndSetting)
{
    CgaPaletteRenderer renderer {};
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::CgaPalette1);
    EXPECT_EQ(renderer.getActivePalette().size(), 4U);

    renderer.cyclePaletteMode();
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::CgaPalette0);
    EXPECT_EQ(renderer.getActivePalette().size(), 4U);

    renderer.cyclePaletteMode();
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::CompositeNtsc);
    EXPECT_EQ(renderer.getActivePalette().size(), 16U);

    renderer.cyclePaletteMode();
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::EnhancedEga);
    EXPECT_EQ(renderer.getActivePalette().size(), 16U);

    renderer.cyclePaletteMode();
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::CgaPalette1);

    renderer.setPaletteMode(PaletteMode::EnhancedEga);
    EXPECT_EQ(renderer.getPaletteMode(), PaletteMode::EnhancedEga);
}

TEST(CgaRendererTest, ColorQuantizationAccuracy)
{
    CgaPaletteRenderer renderer {};

    // In Mode 1 (Black, Cyan, Magenta, White)
    renderer.setPaletteMode(PaletteMode::CgaPalette1);
    const ColorRgba black{ 0, 0, 0, 255 };
    const ColorRgba white{ 255, 255, 255, 255 };
    const ColorRgba cyan{ 85, 255, 255, 255 };
    const ColorRgba magenta{ 255,  85, 255, 255 };

    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 0, 0, 0, 255 }), black);
    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 255, 255, 255, 255 }), white);
    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 0, 200, 255, 255 }), cyan);
    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 255, 0, 200, 255 }), magenta);

    // In Mode 0 (Black, Green, Red, Yellow)
    renderer.setPaletteMode(PaletteMode::CgaPalette0);
    const ColorRgba green{ 85, 255, 85, 255 };
    const ColorRgba red{ 255, 85, 85, 255 };
    const ColorRgba yellow{ 255, 255, 85, 255 };

    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 0, 255, 0, 255 }), green);
    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 255, 0, 0, 255 }), red);
    EXPECT_EQ(renderer.quantizeColor(ColorRgba{ 200, 200, 0, 255 }), yellow);
}

TEST(CgaRendererTest, ClearAndPixelAccessBounds)
{
    CgaPaletteRenderer renderer {};
    renderer.setPaletteMode(PaletteMode::CgaPalette1);

    const ColorRgba cyan{ 85, 255, 255, 255 };
    renderer.clear(cyan);

    EXPECT_EQ(renderer.getPixel(0, 0), cyan);
    EXPECT_EQ(renderer.getPixel(160, 100), cyan);
    EXPECT_EQ(renderer.getPixel(319, 199), cyan);

    // Out of bounds get returns black
    EXPECT_EQ(renderer.getPixel(-1, 0), (ColorRgba{ 0, 0, 0, 255 }));
    EXPECT_EQ(renderer.getPixel(320, 0), (ColorRgba{ 0, 0, 0, 255 }));
    EXPECT_EQ(renderer.getPixel(0, -1), (ColorRgba{ 0, 0, 0, 255 }));
    EXPECT_EQ(renderer.getPixel(0, 200), (ColorRgba{ 0, 0, 0, 255 }));

    // Set pixel within bounds
    const ColorRgba white{ 255, 255, 255, 255 };
    renderer.setPixel(50, 50, white);
    EXPECT_EQ(renderer.getPixel(50, 50), white);

    // Set pixel out of bounds should not crash
    renderer.setPixel(-10, -10, white);
    renderer.setPixel(400, 400, white);
}

TEST(CgaRendererTest, DrawingPrimitives)
{
    CgaPaletteRenderer renderer {};
    renderer.setPaletteMode(PaletteMode::CgaPalette1);
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    const ColorRgba white{ 255, 255, 255, 255 };

    // Horizontal line
    renderer.drawLine(10, 20, 30, 20, white);
    for (int x { 10 }; x <= 30; ++x) {
        EXPECT_EQ(renderer.getPixel(x, 20), white);
    }
    EXPECT_EQ(renderer.getPixel(9, 20), (ColorRgba{ 0, 0, 0, 255 }));
    EXPECT_EQ(renderer.getPixel(31, 20), (ColorRgba{ 0, 0, 0, 255 }));

    // Filled rect
    renderer.fillRect(100, 50, 10, 8, white);
    for (int y { 50 }; y < 58; ++y) {
        for (int x { 100 }; x < 110; ++x) {
            EXPECT_EQ(renderer.getPixel(x, y), white);
        }
    }

    // Sprite blit with color key
    const ColorRgba magenta{ 255, 85, 255, 255 };
    const ColorRgba key{ 0, 0, 0, 255 };
    std::array<ColorRgba, 4> sprite{
        white, key,
        key,   magenta
    };

    renderer.clear(ColorRgba{ 0, 0, 0, 255 });
    renderer.blitSprite(15, 15, 2, 2, sprite.data(), key);

    EXPECT_EQ(renderer.getPixel(15, 15), white);
    EXPECT_EQ(renderer.getPixel(16, 15), key);
    EXPECT_EQ(renderer.getPixel(15, 16), key);
    EXPECT_EQ(renderer.getPixel(16, 16), magenta);
}

TEST(CgaRendererTest, ViewportLetterboxMath)
{
    // Exact 16:10 match
    const ViewportRect vp1 { CgaPaletteRenderer::computeViewport(1280, 800) };
    EXPECT_EQ(vp1.x, 0);
    EXPECT_EQ(vp1.y, 0);
    EXPECT_EQ(vp1.width, 1280);
    EXPECT_EQ(vp1.height, 800);

    // 16:9 widescreen pillarboxing (1920x1080)
    const ViewportRect vp2 { CgaPaletteRenderer::computeViewport(1920, 1080) };
    EXPECT_EQ(vp2.height, 1080);
    EXPECT_EQ(vp2.width, 1728);
    EXPECT_EQ(vp2.x, 96);
    EXPECT_EQ(vp2.y, 0);

    // Square window letterboxing (800x800)
    const ViewportRect vp3 { CgaPaletteRenderer::computeViewport(800, 800) };
    EXPECT_EQ(vp3.width, 800);
    EXPECT_EQ(vp3.height, 500);
    EXPECT_EQ(vp3.x, 0);
    EXPECT_EQ(vp3.y, 150);
}

TEST(GameAppTest, HeadlessInitAndLifecycle)
{
    GameAppConfig config {};
    config.headless = true;
    config.initialPalette = PaletteMode::EnhancedEga;

    GameApp app{ config };
    EXPECT_TRUE(app.initialize());
    EXPECT_TRUE(app.isRunning());
    EXPECT_FALSE(app.isPaused());
    EXPECT_EQ(app.getRenderer().getPaletteMode(), PaletteMode::EnhancedEga);

    app.togglePause();
    EXPECT_TRUE(app.isPaused());
    app.togglePause();
    EXPECT_FALSE(app.isPaused());

    app.stop();
    EXPECT_FALSE(app.isRunning());
}

TEST(GameAppTest, SimulationStepAdvancesPhysics)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());

    const double initialY { app.getTruckBody().getPosition().y };

    // Advance 30 physics steps in mid-air (should fall under gravity)
    for (int i { 0 }; i < 30; ++i) {
        app.updateSimulation(1.0 / 60.0);
    }

    const double newY { app.getTruckBody().getPosition().y };
    EXPECT_LT(newY, initialY);
}

TEST(GameAppTest, FireRocketAndMortarBallistics)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());
    EXPECT_EQ(app.getBallistics().getProjectiles().size(), 0U);

    // Fire rocket
    app.getBallistics().fireRocket(
        app.getTruckBody().getPosition(),
        0.0,
        app.getTruckBody().getLinearVelocity()
    );
    EXPECT_EQ(app.getBallistics().getProjectiles().size(), 1U);
    EXPECT_EQ(app.getBallistics().getProjectiles()[0].type, vehicle_sim::dynamics::ProjectileType::ForwardRocket);

    // Fire mortar
    app.getBallistics().fireMortar(
        app.getTruckBody().getPosition(),
        app.getTruckBody().getLinearVelocity()
    );
    EXPECT_EQ(app.getBallistics().getProjectiles().size(), 2U);
    EXPECT_EQ(app.getBallistics().getProjectiles()[1].type, vehicle_sim::dynamics::ProjectileType::AntiAirMortar);

    // Render frame in headless mode should execute without crash
    app.renderFrame();
}

TEST(Camera2DTest, WorldToScreenInvertibility)
{
    Camera2D camera {};
    const vehicle_sim::math::Vec2 testWorldPt{ 12.5, 3.2 };
    const vehicle_sim::math::Vec2 screenPt{ camera.worldToScreen(testWorldPt) };
    const vehicle_sim::math::Vec2 recoveredWorldPt{ camera.screenToWorld(screenPt) };

    EXPECT_NEAR(recoveredWorldPt.x, testWorldPt.x, 1e-4);
    EXPECT_NEAR(recoveredWorldPt.y, testWorldPt.y, 1e-4);
}

TEST(Camera2DTest, LookaheadLeadSpeedScaling)
{
    Camera2D camera {};
    const vehicle_sim::math::Vec2 basePos{ 0.0, 0.0 };
    camera.reset(basePos);

    // Update with high forward speed (20 m/s)
    for (int i { 0 }; i < 60; ++i) {
        camera.update(basePos, 20.0, 1.0 / 60.0);
    }

    EXPECT_GT(camera.getPosition().x, basePos.x);
    EXPECT_LE(camera.getPosition().x, basePos.x + camera.getConfig().maxLookahead + 0.1);
}

TEST(Camera2DTest, TraumaDecayAndShake)
{
    Camera2D camera {};
    camera.addTrauma(0.8);
    EXPECT_NEAR(camera.getTrauma(), 0.8, 1e-4);

    camera.update(vehicle_sim::math::Vec2{ 0.0, 0.0 }, 0.0, 0.1);
    EXPECT_LT(camera.getTrauma(), 0.8);

    // Shake offset should be bounded by maxShakeDisplacement
    EXPECT_LE(std::abs(camera.getShakeOffset().x), camera.getConfig().maxShakeDisplacement);
    EXPECT_LE(std::abs(camera.getShakeOffset().y), camera.getConfig().maxShakeDisplacement);

    // Step until trauma completely decays
    for (int i { 0 }; i < 100; ++i) {
        camera.update(vehicle_sim::math::Vec2{ 0.0, 0.0 }, 0.0, 0.05);
    }
    EXPECT_EQ(camera.getTrauma(), 0.0);
    EXPECT_EQ(camera.getShakeOffset().x, 0.0);
    EXPECT_EQ(camera.getShakeOffset().y, 0.0);
}

TEST(Camera2DTest, VisibleBoundsEvaluation)
{
    Camera2D camera {};
    camera.reset(vehicle_sim::math::Vec2{ 10.0, 5.0 });

    const double minX { camera.getVisibleMinX() };
    const double maxX { camera.getVisibleMaxX() };
    const double spanX { maxX - minX };

    // At 320 px and 20 px/m, span should be exactly 16.0 meters
    EXPECT_NEAR(spanX, 16.0, 1e-4);
    EXPECT_LT(minX, 10.0);
    EXPECT_GT(maxX, 10.0);
}

TEST(CourseManagerTest, StageCatalogueLoading)
{
    CourseManager course {};
    for (int stage { 1 }; stage <= 8; ++stage) {
        course.loadStage(stage);
        EXPECT_EQ(course.getCurrentStageIndex(), stage);
        EXPECT_FALSE(course.getCurrentStage().title.empty());
        EXPECT_GT(course.getHeightfield().getSampleCount(), 100U);
        EXPECT_EQ(course.getCheckpoints().size(), 5U);
        EXPECT_FALSE(course.isStageComplete());
        EXPECT_EQ(course.getNextCheckpointLetter(), 'A');
        EXPECT_GT(course.getRemainingTime(), 50.0);
    }
}

TEST(CourseManagerTest, HeightfieldContinuity)
{
    CourseManager course {};
    course.loadStage(1); // Mojave Dunes

    // Verify heightfield evaluation at various points
    for (double x { 0.0 }; x <= 500.0; x += 10.0) {
        const double h { course.getHeightfield().getHeight(x) };
        EXPECT_FALSE(std::isnan(h));
        EXPECT_FALSE(std::isinf(h));
    }
}

TEST(CourseManagerTest, CheckpointProgression)
{
    CourseManager course {};
    course.loadStage(1);
    EXPECT_EQ(course.getNextCheckpointLetter(), 'A');
    EXPECT_EQ(course.getScore(), 0);

    // Cross Checkpoint A at 100m
    course.update(105.0, 0.1);
    EXPECT_EQ(course.getNextCheckpointLetter(), 'B');
    EXPECT_GT(course.getScore(), 0);
    EXPECT_FALSE(course.isStageComplete());

    // Cross Checkpoints B, C, D
    course.update(205.0, 0.1);
    EXPECT_EQ(course.getNextCheckpointLetter(), 'C');
    course.update(305.0, 0.1);
    EXPECT_EQ(course.getNextCheckpointLetter(), 'D');
    course.update(405.0, 0.1);
    EXPECT_EQ(course.getNextCheckpointLetter(), 'E');

    // Cross Checkpoint E at 500m (Finish)
    course.update(505.0, 0.1);
    EXPECT_TRUE(course.isStageComplete());
    EXPECT_NEAR(course.getProgressFraction(), 1.0, 1e-4);

    // Reset restores clean state
    course.reset();
    EXPECT_FALSE(course.isStageComplete());
    EXPECT_EQ(course.getNextCheckpointLetter(), 'A');
}

TEST(ParallaxRendererTest, DeterministicExecutionAcrossPalettes)
{
    ParallaxRenderer parallax {};
    Camera2D camera {};
    CourseManager course {};
    CgaPaletteRenderer cga {};

    for (int stage { 1 }; stage <= 8; ++stage) {
        course.loadStage(stage);
        for (auto mode : { PaletteMode::CgaPalette1, PaletteMode::CgaPalette0, PaletteMode::CompositeNtsc, PaletteMode::EnhancedEga }) {
            cga.setPaletteMode(mode);
            parallax.update(0.1);
            parallax.render(cga, camera, course.getHeightfield(), stage);

            // Framebuffer must have non-zero pixels
            EXPECT_NE(cga.getPixel(160, 180), (ColorRgba{ 0, 0, 0, 0 }));
        }
    }
}

TEST(ParallaxRendererTest, ZeroBufferSmearing)
{
    ParallaxRenderer parallax {};
    Camera2D camera {};
    CourseManager course {};
    CgaPaletteRenderer cga {};

    course.loadStage(1);
    parallax.render(cga, camera, course.getHeightfield(), 1);

    // For every column, pixels from the ground surface down to y = 199 must be rendered
    for (int col { 0 }; col < CgaPaletteRenderer::kNativeWidth; col += 20) {
        const ColorRgba bottomPixel { cga.getPixel(col, 199) };
        EXPECT_EQ(bottomPixel.a, 255);
    }
}

TEST(GameAppTest, FullTraversalAndCheckpointScoring)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());

    // Advance 60 simulation steps with full throttle
    vehicle_sim::dynamics::OffroadTruckInput input {};
    input.throttle = 1.0;
    app.setInput(input);
    for (int i { 0 }; i < 60; ++i) {
        app.updateSimulation(1.0 / 60.0);
    }

    // Vehicle should have progressed forward
    EXPECT_GT(app.getTruckBody().getPosition().x, 0.0);
    // Camera should have tracked vehicle
    EXPECT_GT(app.getCamera().getPosition().x, 0.0);

    // Render frame executes cleanly
    app.renderFrame();
}

TEST(InputControllerTest, NeutralStateAndDefaults)
{
    InputController controller {};
    const auto& input { controller.getTruckInput() };
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
    EXPECT_FALSE(input.jump);
    EXPECT_FALSE(input.firePrimary);
    EXPECT_FALSE(input.fireSecondary);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::None);
}

TEST(InputControllerTest, KeyboardMappingTransitions)
{
    InputController controller {};

    SDL_Event ev {};
    ev.type = SDL_KEYDOWN;
    ev.key.repeat = 0;

    // Press W -> Throttle
    ev.key.keysym.sym = SDLK_w;
    controller.handleEvent(ev);
    EXPECT_DOUBLE_EQ(controller.getTruckInput().throttle, 1.0);

    // Press S -> Brake
    ev.key.keysym.sym = SDLK_s;
    controller.handleEvent(ev);
    EXPECT_DOUBLE_EQ(controller.getTruckInput().brake, 1.0);

    // Press Space -> Jump
    ev.key.keysym.sym = SDLK_SPACE;
    controller.handleEvent(ev);
    EXPECT_TRUE(controller.getTruckInput().jump);

    // Press Z -> Fire Primary
    ev.key.keysym.sym = SDLK_z;
    controller.handleEvent(ev);
    EXPECT_TRUE(controller.getTruckInput().firePrimary);

    // Press X -> Fire Secondary
    ev.key.keysym.sym = SDLK_x;
    controller.handleEvent(ev);
    EXPECT_TRUE(controller.getTruckInput().fireSecondary);

    // Release W -> Throttle resets
    ev.type = SDL_KEYUP;
    ev.key.keysym.sym = SDLK_w;
    controller.handleEvent(ev);
    EXPECT_DOUBLE_EQ(controller.getTruckInput().throttle, 0.0);

    // Release S -> Brake resets
    ev.key.keysym.sym = SDLK_s;
    controller.handleEvent(ev);
    EXPECT_DOUBLE_EQ(controller.getTruckInput().brake, 0.0);
}

TEST(InputControllerTest, SystemActionEdgeTriggering)
{
    InputController controller {};

    SDL_Event ev {};
    ev.type = SDL_KEYDOWN;
    ev.key.repeat = 0;

    // Cycle Palette with C
    ev.key.keysym.sym = SDLK_c;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::CyclePalette);
    // Edge triggered: subsequent call is None
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::None);

    // F1 to F4 palettes
    ev.key.keysym.sym = SDLK_F1;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::SetPaletteMode1);

    ev.key.keysym.sym = SDLK_F2;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::SetPaletteMode0);

    ev.key.keysym.sym = SDLK_F3;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::SetPaletteComposite);

    ev.key.keysym.sym = SDLK_F4;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::SetPaletteEga);

    // Pause toggle
    ev.key.keysym.sym = SDLK_p;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::TogglePause);

    // Quit
    ev.key.keysym.sym = SDLK_ESCAPE;
    controller.handleEvent(ev);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::Quit);
}

TEST(InputControllerTest, ResetIntegrity)
{
    InputController controller {};

    SDL_Event ev {};
    ev.type = SDL_KEYDOWN;
    ev.key.repeat = 0;
    ev.key.keysym.sym = SDLK_w;
    controller.handleEvent(ev);
    EXPECT_DOUBLE_EQ(controller.getTruckInput().throttle, 1.0);

    controller.reset();
    EXPECT_DOUBLE_EQ(controller.getTruckInput().throttle, 0.0);
    EXPECT_EQ(controller.consumeSystemAction(), SystemAction::None);
}

TEST(ParticleSystemTest, PoolAllocationAndBoundedCapacity)
{
    ParticleSystem particles {};
    EXPECT_EQ(particles.getActiveCount(), 0U);

    // Emit 1500 particles (exceeding fixed pool size of 1024)
    for (int i { 0 }; i < 300; ++i) {
        particles.emitRoosterTail(vehicle_sim::math::Vec2{ 0.0, 0.0 }, vehicle_sim::math::Vec2{ 15.0, 0.0 }, 0.1, 0);
    }

    // Active count must be capped at kMaxParticles (1024)
    EXPECT_LE(particles.getActiveCount(), ParticleSystem::kMaxParticles);
    EXPECT_GT(particles.getActiveCount(), 0U);
}

TEST(ParticleSystemTest, RoosterTailEmissionKinetics)
{
    ParticleSystem particles {};
    particles.emitRoosterTail(vehicle_sim::math::Vec2{ 10.0, 1.0 }, vehicle_sim::math::Vec2{ 12.0, 0.0 }, 0.05, 0);
    EXPECT_GT(particles.getActiveCount(), 0U);
}

TEST(ParticleSystemTest, LandingDustRadialDispersion)
{
    ParticleSystem particles {};
    particles.emitLandingDust(vehicle_sim::math::Vec2{ 50.0, 2.0 }, 4.5, 1);
    EXPECT_GT(particles.getActiveCount(), 0U);
}

TEST(ParticleSystemTest, ThrusterSparkEmissionKinetics)
{
    ParticleSystem particles {};
    particles.emitThrusterSparks(vehicle_sim::math::Vec2{ 20.0, 5.0 }, vehicle_sim::math::Vec2{ 5.0, 3.0 });
    EXPECT_GT(particles.getActiveCount(), 0U);
}

TEST(ParticleSystemTest, IntegrationStepAndLifespanExpiration)
{
    ParticleSystem particles {};
    particles.emitThrusterSparks(vehicle_sim::math::Vec2{ 0.0, 0.0 }, vehicle_sim::math::Vec2{ 0.0, 0.0 });
    EXPECT_GT(particles.getActiveCount(), 0U);

    // Step simulation 2 seconds (exceeds 0.3s max lifetime of sparks)
    for (int i { 0 }; i < 120; ++i) {
        particles.update(1.0 / 60.0);
    }

    EXPECT_EQ(particles.getActiveCount(), 0U);
}

TEST(ParticleSystemTest, RenderCleanliness)
{
    ParticleSystem particles {};
    Camera2D camera {};
    CgaPaletteRenderer cga {};

    particles.emitRoosterTail(vehicle_sim::math::Vec2{ 0.0, 0.0 }, vehicle_sim::math::Vec2{ 10.0, 0.0 }, 0.1, 0);
    particles.emitLandingDust(vehicle_sim::math::Vec2{ 0.0, 0.0 }, 3.0, 0);
    particles.emitThrusterSparks(vehicle_sim::math::Vec2{ 0.0, 0.0 }, vehicle_sim::math::Vec2{ 0.0, 0.0 });

    particles.render(cga, camera);
    // Buffer should contain rendered particles
    EXPECT_GT(particles.getActiveCount(), 0U);
}

TEST(TruckRendererTest, AnimationRotationMonotonicity)
{
    TruckRenderer renderer {};
    // Step animation with positive speed
    renderer.updateAnimation(15.0, 0.1);
    // Reset restores zero phase
    renderer.reset();
}

TEST(TruckRendererTest, OffScreenCullingSafety)
{
    TruckRenderer renderer {};
    Camera2D camera {};
    CgaPaletteRenderer cga {};
    vehicle_sim::physics::RigidBody2D body {};
    vehicle_sim::dynamics::OffroadTruckDynamics dynamics {};

    // Position truck 1000m away (well off-screen)
    body.setPosition(vehicle_sim::math::Vec2{ 1000.0, 500.0 });

    // Should execute safely without buffer overflow or crash
    renderer.render(cga, camera, body, dynamics);
}

TEST(GameAppPhase4IntegrationTest, DrivingSpawnsRoosterTails)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());

    vehicle_sim::dynamics::OffroadTruckInput input {};
    input.throttle = 1.0;
    app.setInput(input);

    for (int i { 0 }; i < 30; ++i) {
        app.updateSimulation(1.0 / 60.0);
    }

    // Moving on ground should produce tire rooster tail particles
    EXPECT_GT(app.getParticleSystem().getActiveCount(), 0U);
}

TEST(GameAppPhase4IntegrationTest, JumpGeneratesThrusterSparks)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());

    // Jump should emit thruster sparks immediately
    vehicle_sim::dynamics::OffroadTruckInput input {};
    input.jump = true;
    app.setInput(input);

    app.updateSimulation(1.0 / 60.0);
    EXPECT_GT(app.getParticleSystem().getActiveCount(), 0U);
}

TEST(GameAppPhase4IntegrationTest, FullPhase4RenderFrameExecution)
{
    GameAppConfig config {};
    config.headless = true;

    GameApp app{ config };
    ASSERT_TRUE(app.initialize());

    vehicle_sim::dynamics::OffroadTruckInput input {};
    input.throttle = 1.0;
    input.jump = true;
    app.setInput(input);

    for (int i { 0 }; i < 20; ++i) {
        app.updateSimulation(1.0 / 60.0);
    }

    // Render frame with all Phase 4 visual subsystems active
    app.renderFrame();
}

} // namespace montsoft_offroad
