#include "ParallaxRenderer.h"

#include <algorithm>
#include <cmath>

namespace earth_patrol {

ParallaxRenderer::ParallaxRenderer() noexcept
{
}

void ParallaxRenderer::update(double dt) noexcept
{
    cloudDrift_ += dt * 4.0;
}

void ParallaxRenderer::render(SDL_Renderer* renderer, const Camera2D& camera, int biomeIndex) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const int width { camera.getConfig().viewportWidth };
    const int height { camera.getConfig().viewportHeight };

    renderSky(renderer, width, height, biomeIndex);
    renderClouds(renderer, camera, width, biomeIndex);
    renderMountains(renderer, camera, width, height, biomeIndex);
    renderHills(renderer, camera, width, height, biomeIndex);
}

void ParallaxRenderer::reset() noexcept
{
    cloudDrift_ = 0.0;
}

void ParallaxRenderer::renderSky(SDL_Renderer* renderer, int width, int height, int biomeIndex) noexcept
{
    const int bandCount { 16 };
    const int bandHeight { (height + bandCount - 1) / bandCount };

    for (int i { 0 }; i < bandCount; ++i) {
        const double t { static_cast<double>(i) / static_cast<double>(bandCount - 1) };
        Uint8 r { 0 };
        Uint8 g { 0 };
        Uint8 b { 0 };

        if (biomeIndex == 1) { // Desert Badlands
            r = static_cast<Uint8>(40.0 + (140.0 * t));
            g = static_cast<Uint8>(25.0 + (75.0 * t));
            b = static_cast<Uint8>(20.0 + (25.0 * t));
        } else if (biomeIndex == 2) { // Alpine Forest Pass
            r = static_cast<Uint8>(25.0 + (105.0 * t));
            g = static_cast<Uint8>(55.0 + (125.0 * t));
            b = static_cast<Uint8>(95.0 + (125.0 * t));
        } else { // Highway Ruins (Default)
            r = static_cast<Uint8>(20.0 + (60.0 * t));
            g = static_cast<Uint8>(26.0 + (35.0 * t));
            b = static_cast<Uint8>(42.0 + (15.0 * t));
        }

        SDL_SetRenderDrawColor(renderer, r, g, b, 255);
        const int yStart { i * bandHeight };
        const int actualHeight { (i == (bandCount - 1)) ? (height - yStart) : (bandHeight + 1) };
        SDL_Rect bandRect { 0, yStart, width, actualHeight };
        SDL_RenderFillRect(renderer, &bandRect);
    }
}

void ParallaxRenderer::renderClouds(SDL_Renderer* renderer, const Camera2D& camera, int width, int biomeIndex) noexcept
{
    const double camX { camera.getPosition().x };
    const double ppm { camera.getConfig().pixelsPerMeter };
    const double cloudScroll { (camX * 0.02 * ppm) - cloudDrift_ };

    // Draw 4 cloud puffs
    struct CloudDef {
        double baseX { 0.0 };
        int y { 0 };
        int w { 0 };
        int h { 0 };
    };

    const CloudDef clouds[] { CloudDef { 120.0, 45, 160, 36 }, CloudDef { 480.0, 75, 210, 42 },
        CloudDef { 860.0, 35, 180, 38 }, CloudDef { 1200.0, 65, 230, 46 } };

    for (const auto& c : clouds) {
        double screenX { std::fmod(c.baseX - cloudScroll, static_cast<double>(width + 300)) };
        if (screenX < -200.0) {
            screenX += static_cast<double>(width + 300);
        }

        const int x { static_cast<int>(screenX) - 100 };

        if (biomeIndex == 1) {
            SDL_SetRenderDrawColor(renderer, 220, 170, 140, 160);
        } else if (biomeIndex == 2) {
            SDL_SetRenderDrawColor(renderer, 230, 240, 255, 180);
        } else {
            SDL_SetRenderDrawColor(renderer, 150, 155, 175, 140);
        }

        SDL_Rect puffA { x, c.y, c.w, c.h };
        SDL_Rect puffB { x + 25, c.y - 12, c.w - 50, c.h };
        SDL_RenderFillRect(renderer, &puffA);
        SDL_RenderFillRect(renderer, &puffB);
    }
}

void ParallaxRenderer::renderMountains(
    SDL_Renderer* renderer, const Camera2D& camera, int width, int height, int biomeIndex) noexcept
{
    const double camX { camera.getPosition().x };
    const double ppm { camera.getConfig().pixelsPerMeter };
    const double scrollOffset { camX * 0.08 * ppm };
    const int baselineY { height * 3 / 4 - 30 };

    if (biomeIndex == 1) { // Desert Buttes & Mesas
        SDL_SetRenderDrawColor(renderer, 95, 48, 32, 255);
    } else if (biomeIndex == 2) { // Alpine Jagged Peaks
        SDL_SetRenderDrawColor(renderer, 48, 70, 98, 255);
    } else { // Industrial skyline / distant ridge
        SDL_SetRenderDrawColor(renderer, 38, 42, 60, 255);
    }

    constexpr int stepPx { 6 };

    for (int screenX { 0 }; screenX <= width; screenX += stepPx) {
        const double worldDist { (static_cast<double>(screenX) + scrollOffset) * 0.012 };
        double ridgeY { 0.0 };

        if (biomeIndex == 1) { // Flat-topped mesas
            ridgeY = std::sin(worldDist) * 35.0 + std::cos(worldDist * 2.7) * 15.0;
        } else if (biomeIndex == 2) { // Sharp alpine crests
            ridgeY = std::abs(std::sin(worldDist * 1.5)) * 65.0 + std::sin(worldDist * 3.1) * 20.0;
        } else { // Undulating ridge
            ridgeY = std::sin(worldDist * 1.2) * 45.0 + std::cos(worldDist * 0.7) * 25.0;
        }

        const int currentY { baselineY - static_cast<int>(std::max(10.0, ridgeY)) };
        const int colHeight { std::max(0, height - currentY) };

        SDL_Rect mountainCol { screenX, currentY, stepPx, colHeight };
        SDL_RenderFillRect(renderer, &mountainCol);

        // Alpine snow cap highlights
        if (biomeIndex == 2 && ridgeY > 48.0) {
            SDL_SetRenderDrawColor(renderer, 195, 215, 235, 255);
            SDL_Rect snowCap { screenX, currentY, stepPx, 6 };
            SDL_RenderFillRect(renderer, &snowCap);
            SDL_SetRenderDrawColor(renderer, 48, 70, 98, 255);
        }
    }
}

void ParallaxRenderer::renderHills(
    SDL_Renderer* renderer, const Camera2D& camera, int width, int height, int biomeIndex) noexcept
{
    const double camX { camera.getPosition().x };
    const double ppm { camera.getConfig().pixelsPerMeter };
    const double scrollOffset { camX * 0.28 * ppm };
    const int baselineY { height * 3 / 4 };

    if (biomeIndex == 1) { // Red sandstone hills
        SDL_SetRenderDrawColor(renderer, 125, 68, 42, 255);
    } else if (biomeIndex == 2) { // Dark pine forest foothills
        SDL_SetRenderDrawColor(renderer, 35, 76, 45, 255);
    } else { // Industrial embankments
        SDL_SetRenderDrawColor(renderer, 44, 52, 68, 255);
    }

    constexpr int stepPx { 6 };

    for (int screenX { 0 }; screenX <= width; screenX += stepPx) {
        const double worldDist { (static_cast<double>(screenX) + scrollOffset) * 0.024 };
        const double hillHeight { std::sin(worldDist) * 28.0 + std::cos(worldDist * 1.8) * 14.0 + 35.0 };
        const int currentY { baselineY - static_cast<int>(hillHeight) };
        const int colHeight { std::max(0, height - currentY) };

        SDL_Rect hillCol { screenX, currentY, stepPx, colHeight };
        SDL_RenderFillRect(renderer, &hillCol);
    }
}

} // namespace earth_patrol
