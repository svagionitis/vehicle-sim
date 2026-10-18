#include <gtest/gtest.h>

#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/GameApp.h"
#include "apps/MontsoftOffroad/SDLWrappers.h"

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

} // namespace montsoft_offroad
