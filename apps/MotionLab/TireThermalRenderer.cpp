#include "apps/MotionLab/TireThermalRenderer.h"
#include "apps/MotionLab/TelemetryHud.h"
#include "apps/MotionLab/WindVectorFieldRenderer.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace motion_lab {

namespace {
constexpr double kPi { 3.14159265358979323846 };
} // namespace

void TireThermalRenderer::toggleHud() noexcept
{
    mHudVisible = !mHudVisible;
}

void TireThermalRenderer::setHudVisible(bool visible) noexcept
{
    mHudVisible = visible;
}

bool TireThermalRenderer::isHudVisible() const noexcept
{
    return mHudVisible;
}

SDL_Color TireThermalRenderer::getTemperatureColor(double tempCelsius) noexcept
{
    if (tempCelsius < 50.0) {
        // Cold Phosphor Cyan
        return SDL_Color { 0, 229, 255, 255 };
    }
    if (tempCelsius < 75.0) {
        // Warmup Light Green
        return SDL_Color { 118, 255, 3, 255 };
    }
    if (tempCelsius <= 95.0) {
        // Optimal Neon Green
        return SDL_Color { 0, 230, 118, 255 };
    }
    if (tempCelsius <= 115.0) {
        // Warning Amber
        return SDL_Color { 255, 234, 0, 255 };
    }
    // Overheat / Blistering Orange-Red
    return SDL_Color { 255, 61, 0, 255 };
}

SDL_Point TireThermalRenderer::computeHudCardPos(int screenWidth,
                                                int screenHeight,
                                                bool circuitActive,
                                                bool advancedHudActive,
                                                bool windHudActive,
                                                bool provingGroundActive) noexcept
{
    constexpr int cardW { kCardWidth };
    constexpr int cardH { kCardHeight };

    auto intersects = [](int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2) noexcept {
        const bool noOverlap = (x1 + w1 <= x2) || (x2 + w2 <= x1) || (y1 + h1 <= y2) || (y2 + h2 <= y1);
        return !noOverlap;
    };

    auto collidesWithAny = [&](int cx, int cy) noexcept -> bool {
        if (cx < 0 || cx + cardW > screenWidth || cy < 0 || cy + cardH > screenHeight) {
            return true;
        }

        // 1. Live Telemetry text (top-left)
        constexpr int textX { 24 };
        constexpr int textY { 20 };
        constexpr int textW { 180 };
        constexpr int textH { 180 };
        if (intersects(cx, cy, cardW, cardH, textX, textY, textW, textH)) {
            return true;
        }

        // 2. Input HUD (bottom-right)
        const int inputX { screenWidth - 264 };
        const int inputY { screenHeight - 64 };
        constexpr int inputW { 250 };
        constexpr int inputH { 20 };
        if (intersects(cx, cy, cardW, cardH, inputX, inputY, inputW, inputH)) {
            return true;
        }

        // 3. Circuit Radar (top-right when circuit active)
        if (circuitActive) {
            const int radarX { screenWidth - 170 };
            constexpr int radarY { 30 };
            constexpr int radarW { 140 };
            constexpr int radarH { 140 };
            if (intersects(cx, cy, cardW, cardH, radarX, radarY, radarW, radarH)) {
                return true;
            }
        }

        // 4. Advanced Telemetry HUD (right sidebar)
        if (advancedHudActive) {
            const int advX { screenWidth - 228 - 16 };
            const int advY { circuitActive ? 170 : 20 };
            constexpr int advW { 228 };
            constexpr int advH { 232 };
            if (intersects(cx, cy, cardW, cardH, advX, advY, advW, advH)) {
                return true;
            }
        }

        // 5. Wind HUD (if active)
        if (windHudActive) {
            const auto wPos = WindVectorFieldRenderer::computeHudCardPos(
                screenWidth, screenHeight, circuitActive, advancedHudActive);
            if (intersects(cx, cy, cardW, cardH, wPos.x, wPos.y,
                           WindVectorFieldRenderer::kCardWidth, WindVectorFieldRenderer::kCardHeight)) {
                return true;
            }
        }

        // 6. Proving Ground HUD (Slalom / Moose / Skidpad at bottom-left)
        if (provingGroundActive) {
            constexpr int pgX { 10 };
            const int pgY { screenHeight - 155 };
            constexpr int pgW { 220 };
            constexpr int pgH { 145 };
            if (intersects(cx, cy, cardW, cardH, pgX, pgY, pgW, pgH)) {
                return true;
            }
        }

        return false;
    };

    // 1. Preferred primary position: Left sidebar directly below Live Metrics text
    if (!collidesWithAny(24, 205)) {
        return SDL_Point { 24, 205 };
    }

    // 2. Preferred secondary position: Right sidebar
    const int rightX { screenWidth - cardW - 16 };
    if (!collidesWithAny(rightX, 260)) {
        return SDL_Point { rightX, 260 };
    }
    if (!collidesWithAny(rightX, 408)) {
        return SDL_Point { rightX, 408 };
    }
    if (!collidesWithAny(rightX, 176)) {
        return SDL_Point { rightX, 176 };
    }
    if (!collidesWithAny(rightX, 20)) {
        return SDL_Point { rightX, 20 };
    }

    // 3. Preferred tertiary position: Top-center
    const int midX { (screenWidth - cardW) / 2 };
    if (!collidesWithAny(midX, 20)) {
        return SDL_Point { midX, 20 };
    }

    // 4. Lower left sidebar
    if (!collidesWithAny(24, 385)) {
        return SDL_Point { 24, 385 };
    }

    return SDL_Point { 24, std::clamp(205, 0, std::max(0, screenHeight - cardH)) };
}

void TireThermalRenderer::renderSmoke(SDL_Renderer* renderer,
                                      const TireSmokeParticleBuffer& smokeBuffer,
                                      const MotionLabCamera& camera) const noexcept
{
    if (renderer == nullptr) {
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    const int vpW { camera.getConfig().viewportWidth };
    const int vpH { camera.getConfig().viewportHeight };
    const double zoom { camera.getZoom() };

    for (const auto& particle : smokeBuffer.getParticles()) {
        if (particle.age >= particle.maxLifetime) {
            continue;
        }

        const auto screenPos = camera.worldToScreen(particle.position);
        const int cx { static_cast<int>(std::round(screenPos.x)) };
        const int cy { static_cast<int>(std::round(screenPos.y)) };

        const double radiusPx { particle.radius * zoom };
        const int r { static_cast<int>(std::round(radiusPx)) };

        if (cx < -r || cx > vpW + r || cy < -r || cy > vpH + r) {
            continue;
        }

        const double ageRatio { particle.age / particle.maxLifetime };
        const double alphaFrac { particle.initialAlpha * (1.0 - ageRatio * ageRatio) };
        const Uint8 alpha { static_cast<Uint8>(std::clamp(alphaFrac * 210.0, 0.0, 255.0)) };

        // 8-segment wireframe loop
        SDL_SetRenderDrawColor(renderer, 180, 220, 255, alpha);

        constexpr int kSegments { 8 };
        int prevX { 0 };
        int prevY { 0 };
        for (int i { 0 }; i <= kSegments; ++i) {
            const double angle { static_cast<double>(i) * (2.0 * kPi / static_cast<double>(kSegments)) };
            const int px { cx + static_cast<int>(std::round(std::cos(angle) * radiusPx)) };
            const int py { cy + static_cast<int>(std::round(std::sin(angle) * radiusPx)) };

            if (i > 0) {
                SDL_RenderDrawLine(renderer, prevX, prevY, px, py);
            }
            prevX = px;
            prevY = py;
        }

        // Inner core loop if sufficiently expanded
        if (radiusPx > 8.0) {
            const double innerR { radiusPx * 0.5 };
            const Uint8 innerAlpha { static_cast<Uint8>(alpha * 0.6) };
            SDL_SetRenderDrawColor(renderer, 220, 240, 255, innerAlpha);

            for (int i { 0 }; i <= kSegments; ++i) {
                const double angle { static_cast<double>(i) * (2.0 * kPi / static_cast<double>(kSegments)) };
                const int px { cx + static_cast<int>(std::round(std::cos(angle) * innerR)) };
                const int py { cy + static_cast<int>(std::round(std::sin(angle) * innerR)) };

                if (i > 0) {
                    SDL_RenderDrawLine(renderer, prevX, prevY, px, py);
                }
                prevX = px;
                prevY = py;
            }
        }
    }
}

void TireThermalRenderer::renderHudCard(SDL_Renderer* renderer,
                                       const TireThermalCoordinator& coordinator,
                                       int originX,
                                       int originY) const noexcept
{
    if (renderer == nullptr || !mHudVisible) {
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    // Card background
    const SDL_Rect bgRect { originX, originY, kCardWidth, kCardHeight };
    SDL_SetRenderDrawColor(renderer, 0, 15, 25, 225);
    SDL_RenderFillRect(renderer, &bgRect);

    // Border
    SDL_SetRenderDrawColor(renderer, 0, 229, 255, 240);
    SDL_RenderDrawRect(renderer, &bgRect);

    // Header Title
    TelemetryHud::drawText(renderer, "TIRE THERMALS", originX + 12, originY + 8, 1, SDL_Color { 0, 229, 255, 255 });

    // Environment info
    std::string envStr { "AIR:" + std::to_string(static_cast<int>(coordinator.getAmbientAirTemp())) + "C"
                       + " ROAD:" + std::to_string(static_cast<int>(coordinator.getTrackRoadTemp())) + "C" };
    TelemetryHud::drawText(renderer, envStr, originX + 115, originY + 8, 1, SDL_Color { 160, 200, 220, 220 });

    // Separator line
    SDL_SetRenderDrawColor(renderer, 0, 100, 140, 180);
    SDL_RenderDrawLine(renderer, originX + 8, originY + 22, originX + kCardWidth - 8, originY + 22);

    if (!coordinator.isWheeledVehicle()) {
        TelemetryHud::drawText(renderer, "NON-WHEELED VEHICLE", originX + 38, originY + 70, 1, SDL_Color { 140, 170, 190, 220 });
        TelemetryHud::drawText(renderer, "[THERMAL BYPASS]", originX + 52, originY + 90, 1, SDL_Color { 100, 140, 160, 180 });
        return;
    }

    // 4 Wheels: FL, FR, RL, RR
    struct WheelLayout {
        const char* name;
        int boxX;
        int boxY;
        std::size_t index;
    };

    const WheelLayout layouts[4] = {
        { "FL", originX + 12, originY + 30, 0 },
        { "FR", originX + 128, originY + 30, 1 },
        { "RL", originX + 12, originY + 85, 2 },
        { "RR", originX + 128, originY + 85, 3 }
    };

    bool anyOverheat { false };

    for (const auto& item : layouts) {
        const auto& state = coordinator.getWheelState(item.index);
        const double temp { state.surfaceTemp };
        const double pressure { state.inflationPressure };
        const SDL_Color color { getTemperatureColor(temp) };

        if (state.isOverheated) {
            anyOverheat = true;
        }

        // Draw wheel identifier and metrics
        std::string label { item.name };
        label += ": " + std::to_string(static_cast<int>(std::round(temp))) + "C";
        TelemetryHud::drawText(renderer, label, item.boxX, item.boxY, 1, color);

        std::string presStr { std::to_string(static_cast<int>(pressure)) + "."
                            + std::to_string(static_cast<int>(std::abs(pressure * 10.0)) % 10) + "b" };
        TelemetryHud::drawText(renderer, presStr, item.boxX + 68, item.boxY, 1, SDL_Color { 180, 210, 225, 220 });

        // Segmented LED temperature progress bar
        constexpr int barW { 95 };
        constexpr int barH { 6 };
        const SDL_Rect barBg { item.boxX, item.boxY + 14, barW, barH };
        SDL_SetRenderDrawColor(renderer, 20, 40, 50, 200);
        SDL_RenderFillRect(renderer, &barBg);

        const double fillFrac { std::clamp((temp - 20.0) / 110.0, 0.0, 1.0) };
        const int fillW { static_cast<int>(std::round(fillFrac * barW)) };
        if (fillW > 0) {
            const SDL_Rect barFill { item.boxX, item.boxY + 14, fillW, barH };
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 240);
            SDL_RenderFillRect(renderer, &barFill);
        }

        // Grip multiplier display
        const int gripPct { static_cast<int>(std::round(state.effectiveMuMultiplier * 100.0)) };
        std::string gripStr { "GRIP:" + std::to_string(gripPct) + "%" };
        TelemetryHud::drawText(renderer, gripStr, item.boxX, item.boxY + 24, 1, SDL_Color { 150, 180, 200, 200 });
    }

    // Chassis center wireframe icon
    const int midX { originX + kCardWidth / 2 };
    const int midY { originY + 70 };
    SDL_SetRenderDrawColor(renderer, 0, 180, 220, 150);
    const SDL_Rect chassisRect { midX - 8, midY - 18, 16, 36 };
    SDL_RenderDrawRect(renderer, &chassisRect);

    // Overheat warning banner
    if (anyOverheat) {
        SDL_SetRenderDrawColor(renderer, 255, 50, 0, 220);
        const SDL_Rect alertRect { originX + 16, originY + 144, kCardWidth - 32, 18 };
        SDL_RenderDrawRect(renderer, &alertRect);
        TelemetryHud::drawText(renderer, "ALERT: OVERHEAT GRIP DROP", originX + 24, originY + 148, 1, SDL_Color { 255, 80, 20, 255 });
    } else {
        TelemetryHud::drawText(renderer, "STATUS: NOMINAL THERMAL GRIP", originX + 22, originY + 148, 1, SDL_Color { 0, 230, 118, 220 });
    }
}

} // namespace motion_lab
