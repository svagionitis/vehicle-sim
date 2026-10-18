#include "apps/MontsoftOffroad/ParallaxRenderer.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;
using vehicle_sim::terrain::Heightfield1D;

namespace {

struct BiomePalette {
    ColorRgba sky;
    ColorRgba sun;
    ColorRgba mesa;
    ColorRgba dune;
    ColorRgba track;
    ColorRgba soil;
};

BiomePalette getBiomePalette(int stageIndex) noexcept
{
    switch (stageIndex) {
    case 1: // Mojave Dunes
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },        // Sky: Black
            ColorRgba{ 255, 255, 255, 255 },  // Sun: White
            ColorRgba{ 255, 85, 255, 255 },   // Mesa: Magenta
            ColorRgba{ 85, 255, 255, 255 },   // Dune: Cyan
            ColorRgba{ 255, 255, 255, 255 },  // Track: White
            ColorRgba{ 85, 255, 255, 255 }    // Soil: Cyan
        };
    case 2: // Red Rock Arroyo
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },
            ColorRgba{ 255, 255, 85, 255 },   // Sun: Yellow
            ColorRgba{ 255, 85, 85, 255 },    // Mesa: Red
            ColorRgba{ 170, 85, 0, 255 },     // Dune: Brown
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 85, 85, 255 }
        };
    case 3: // Great Salt Flats
        return BiomePalette{
            ColorRgba{ 85, 255, 255, 255 },   // Sky: Cyan
            ColorRgba{ 255, 255, 255, 255 },  // Sun: White
            ColorRgba{ 255, 85, 255, 255 },   // Mesa: Magenta
            ColorRgba{ 255, 255, 255, 255 },  // Dune: White
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 85, 255, 255, 255 }
        };
    case 4: // Muddy Quagmire
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },
            ColorRgba{ 255, 85, 255, 255 },
            ColorRgba{ 85, 255, 85, 255 },    // Mesa: Green
            ColorRgba{ 170, 85, 0, 255 },     // Dune: Brown
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 170, 85, 0, 255 }
        };
    case 5: // Volcanic Badlands
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },
            ColorRgba{ 255, 85, 85, 255 },    // Sun: Red
            ColorRgba{ 255, 85, 255, 255 },   // Mesa: Magenta
            ColorRgba{ 255, 85, 85, 255 },    // Dune: Red
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 85, 85, 255 }
        };
    case 6: // Scrap Canyon
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 85, 255, 255 },
            ColorRgba{ 85, 255, 255, 255 },
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 85, 255, 255 }
        };
    case 7: // Alpine Ridge
        return BiomePalette{
            ColorRgba{ 85, 255, 255, 255 },   // Sky: Cyan
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 255, 255, 255 },  // Snow peaks: White
            ColorRgba{ 255, 85, 255, 255 },   // Ridge: Magenta
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 85, 255, 255, 255 }
        };
    case 8: // Secret Fortress
    default:
        return BiomePalette{
            ColorRgba{ 0, 0, 0, 255 },
            ColorRgba{ 255, 85, 85, 255 },
            ColorRgba{ 255, 85, 255, 255 },
            ColorRgba{ 85, 255, 255, 255 },
            ColorRgba{ 255, 255, 255, 255 },
            ColorRgba{ 255, 255, 255, 255 }
        };
    }
}
} // namespace

ParallaxRenderer::ParallaxRenderer() noexcept
{
}

void ParallaxRenderer::update(double dt) noexcept
{
    animTimer_ += dt;
}

void ParallaxRenderer::reset() noexcept
{
    animTimer_ = 0.0;
}

void ParallaxRenderer::render(
    CgaPaletteRenderer& cga,
    const Camera2D& camera,
    const Heightfield1D& terrain,
    int stageIndex) noexcept
{
    renderSky(cga, stageIndex);
    renderSun(cga, stageIndex);
    renderDistantMesas(cga, camera, stageIndex);
    renderMidgroundDunes(cga, camera, stageIndex);
    renderForegroundTrack(cga, camera, terrain, stageIndex);
}

void ParallaxRenderer::renderSky(CgaPaletteRenderer& cga, int stageIndex) noexcept
{
    const BiomePalette pal{ getBiomePalette(stageIndex) };
    cga.clear(pal.sky);

    // Subtle dithered horizontal scanlines near the upper horizon
    for (int y { 0 }; y < 50; y += 4) {
        for (int x { (y % 8 == 0) ? 0 : 2 }; x < CgaPaletteRenderer::kNativeWidth; x += 4) {
            cga.setPixel(x, y, pal.dune);
        }
    }
}

void ParallaxRenderer::renderSun(CgaPaletteRenderer& cga, int stageIndex) noexcept
{
    const BiomePalette pal{ getBiomePalette(stageIndex) };
    constexpr int sunCenterX { 245 };
    constexpr int sunCenterY { 36 };
    constexpr int sunRadius { 11 };

    for (int dy { -sunRadius }; dy <= sunRadius; ++dy) {
        for (int dx { -sunRadius }; dx <= sunRadius; ++dx) {
            if (dx * dx + dy * dy <= sunRadius * sunRadius) {
                cga.setPixel(sunCenterX + dx, sunCenterY + dy, pal.sun);
            }
        }
    }

    // Corona ray markers
    cga.drawLine(sunCenterX - 16, sunCenterY, sunCenterX - 13, sunCenterY, pal.sun);
    cga.drawLine(sunCenterX + 13, sunCenterY, sunCenterX + 16, sunCenterY, pal.sun);
    cga.drawLine(sunCenterX, sunCenterY - 16, sunCenterX, sunCenterY - 13, pal.sun);
    cga.drawLine(sunCenterX, sunCenterY + 13, sunCenterX, sunCenterY + 16, pal.sun);
}

void ParallaxRenderer::renderDistantMesas(
    CgaPaletteRenderer& cga, const Camera2D& camera, int stageIndex) noexcept
{
    const BiomePalette pal{ getBiomePalette(stageIndex) };
    constexpr double kMesaParallaxScale { 0.06 };
    const double offsetX { camera.getPosition().x * kMesaParallaxScale * camera.getConfig().pixelsPerMeter };

    for (int x { 0 }; x < CgaPaletteRenderer::kNativeWidth; ++x) {
        const double worldCol { static_cast<double>(x) + offsetX };

        // Flat-topped mesa profile
        double rawProfile { 68.0 + 18.0 * std::sin(worldCol * 0.018) + 10.0 * std::cos(worldCol * 0.045) };
        if (stageIndex != 7) { // Clamp flat plateaus except on alpine peaks
            rawProfile = std::min(rawProfile, 82.0);
        }

        const int topY { std::clamp(static_cast<int>(rawProfile), 30, 150) };
        for (int y { topY }; y < CgaPaletteRenderer::kNativeHeight; ++y) {
            cga.setPixel(x, y, pal.mesa);
        }
    }
}

void ParallaxRenderer::renderMidgroundDunes(
    CgaPaletteRenderer& cga, const Camera2D& camera, int stageIndex) noexcept
{
    const BiomePalette pal{ getBiomePalette(stageIndex) };
    constexpr double kDuneParallaxScale { 0.25 };
    const double offsetX { camera.getPosition().x * kDuneParallaxScale * camera.getConfig().pixelsPerMeter };

    for (int x { 0 }; x < CgaPaletteRenderer::kNativeWidth; ++x) {
        const double worldCol { static_cast<double>(x) + offsetX };

        const double duneWave { 108.0 + 14.0 * std::sin(worldCol * 0.032) + 7.0 * std::sin(worldCol * 0.075) };
        const int topY { std::clamp(static_cast<int>(duneWave), 60, 160) };

        for (int y { topY }; y < CgaPaletteRenderer::kNativeHeight; ++y) {
            cga.setPixel(x, y, pal.dune);
        }
    }

    // Render silhouette props spaced along midground
    constexpr int propInterval { 140 };
    const int startPropIdx { static_cast<int>(std::floor((offsetX - 40.0) / propInterval)) };
    const int endPropIdx { static_cast<int>(std::ceil((offsetX + CgaPaletteRenderer::kNativeWidth + 40.0) / propInterval)) };

    for (int idx { startPropIdx }; idx <= endPropIdx; ++idx) {
        const int propScreenX { static_cast<int>(static_cast<double>(idx * propInterval) - offsetX) };
        if (propScreenX >= -20 && propScreenX < CgaPaletteRenderer::kNativeWidth + 20) {
            const double worldCol { static_cast<double>(propScreenX) + offsetX };
            const int groundY { static_cast<int>(108.0 + 14.0 * std::sin(worldCol * 0.032) + 7.0 * std::sin(worldCol * 0.075)) };
            const int propType { std::abs(idx) % 3 };
            renderSilhouetteProp(cga, propScreenX, groundY, propType, pal.mesa);
        }
    }
}

void ParallaxRenderer::renderSilhouetteProp(
    CgaPaletteRenderer& cga, int screenX, int screenGroundY, int propType, ColorRgba color) noexcept
{
    if (propType == 0) {
        // Saguaro Cactus
        cga.fillRect(screenX, screenGroundY - 22, 3, 22, color);
        cga.fillRect(screenX - 5, screenGroundY - 16, 5, 2, color);
        cga.fillRect(screenX - 5, screenGroundY - 20, 2, 6, color);
        cga.fillRect(screenX + 3, screenGroundY - 12, 5, 2, color);
        cga.fillRect(screenX + 6, screenGroundY - 18, 2, 8, color);
    } else if (propType == 1) {
        // Oil Derrick
        cga.drawLine(screenX - 6, screenGroundY, screenX - 2, screenGroundY - 26, color);
        cga.drawLine(screenX + 6, screenGroundY, screenX + 2, screenGroundY - 26, color);
        cga.drawLine(screenX - 2, screenGroundY - 26, screenX + 2, screenGroundY - 26, color);
        cga.drawLine(screenX - 4, screenGroundY - 14, screenX + 4, screenGroundY - 14, color);
        cga.drawLine(screenX - 5, screenGroundY - 5, screenX + 5, screenGroundY - 5, color);
        cga.fillRect(screenX - 1, screenGroundY - 29, 2, 3, color);
    } else {
        // Telegraph Pole
        cga.fillRect(screenX, screenGroundY - 24, 2, 24, color);
        cga.drawLine(screenX - 5, screenGroundY - 20, screenX + 6, screenGroundY - 20, color);
        cga.drawLine(screenX - 4, screenGroundY - 14, screenX + 5, screenGroundY - 14, color);
    }
}

void ParallaxRenderer::renderForegroundTrack(
    CgaPaletteRenderer& cga,
    const Camera2D& camera,
    const Heightfield1D& terrain,
    int stageIndex) noexcept
{
    const BiomePalette pal{ getBiomePalette(stageIndex) };

    for (int col { 0 }; col < CgaPaletteRenderer::kNativeWidth; ++col) {
        const double worldX { camera.screenToWorld(Vec2{ static_cast<double>(col), 0.0 }).x };
        const double worldY { terrain.getHeight(worldX) };
        const Vec2 screenPt { camera.worldToScreen(Vec2{ worldX, worldY }) };

        const int surfaceY { static_cast<int>(std::round(screenPt.y)) };
        const int clampedY { std::clamp(surfaceY, 0, CgaPaletteRenderer::kNativeHeight - 1) };

        // Surface edge pixel
        cga.setPixel(col, clampedY, pal.track);

        // Solid soil strata extending all the way down to y = 199 to eliminate buffer smearing
        for (int y { clampedY + 1 }; y < CgaPaletteRenderer::kNativeHeight; ++y) {
            // Authentic dithered strata bands
            if ((y - clampedY) <= 3) {
                cga.setPixel(col, y, pal.track);
            } else if (((col + y) % 4 == 0) || ((col - y) % 8 == 0)) {
                cga.setPixel(col, y, pal.soil);
            } else {
                cga.setPixel(col, y, pal.mesa);
            }
        }
    }
}

} // namespace montsoft_offroad
