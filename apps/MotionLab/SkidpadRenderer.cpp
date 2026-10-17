#include "apps/MotionLab/SkidpadRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "apps/MotionLab/TelemetryHud.h"
#include "apps/MotionLab/WireframeRenderer.h"

namespace motion_lab {

namespace {

constexpr double kPi { 3.14159265358979323846 };
constexpr double kTwoPi { 2.0 * kPi };

void drawFilledCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color color) noexcept {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int dy { -radius }; dy <= radius; ++dy) {
        const int dx { static_cast<int>(std::sqrt(static_cast<double>(radius * radius - dy * dy))) };
        SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

void drawRing(SDL_Renderer* renderer, int cx, int cy, int rMin, int rMax, SDL_Color color) noexcept {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int dy { -rMax }; dy <= rMax; ++dy) {
        const int dxMax { static_cast<int>(std::sqrt(static_cast<double>(rMax * rMax - dy * dy))) };
        const int dxMin { (std::abs(dy) <= rMin)
            ? static_cast<int>(std::sqrt(static_cast<double>(rMin * rMin - dy * dy)))
            : 0 };

        if (dxMin > 0) {
            SDL_RenderDrawLine(renderer, cx - dxMax, cy + dy, cx - dxMin, cy + dy);
            SDL_RenderDrawLine(renderer, cx + dxMin, cy + dy, cx + dxMax, cy + dy);
        } else {
            SDL_RenderDrawLine(renderer, cx - dxMax, cy + dy, cx + dxMax, cy + dy);
        }
    }
}

} // namespace

void SkidpadRenderer::renderCourse(SDL_Renderer* renderer,
                                  const MotionLabCamera& camera,
                                  const SkidpadCoordinator& coordinator) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    drawConcentricRings(renderer, camera, coordinator);
    drawRadialSpokes(renderer, camera, coordinator);
    drawStagingChute(renderer, camera, coordinator);
    drawDynamicGuide(renderer, camera, coordinator);
    drawPylons(renderer, camera, coordinator);
}

void SkidpadRenderer::drawConcentricRings(SDL_Renderer* renderer,
                                          const MotionLabCamera& camera,
                                          const SkidpadCoordinator& coordinator) noexcept {
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    const double targetR { coordinator.getTargetRadius() };
    const double halfW { coordinator.getCorridorWidth() * 0.5 };
    const double rInner { targetR - halfW };
    const double rOuter { targetR + halfW };

    constexpr int kSegments { 72 };
    const SDL_Color cyanBoundary { 0, 220, 255, 180 };
    const SDL_Color tealCenterline { 0, 255, 180, 90 };

    // Draw inner, centerline, and outer circular boundaries
    for (int i { 0 }; i < kSegments; ++i) {
        const double a1 { (static_cast<double>(i) / static_cast<double>(kSegments)) * kTwoPi };
        const double a2 { (static_cast<double>(i + 1) / static_cast<double>(kSegments)) * kTwoPi };

        // Inner circle segment
        const vehicle_sim::math::Vec2 in1 { rInner * std::cos(a1), rInner * std::sin(a1) };
        const vehicle_sim::math::Vec2 in2 { rInner * std::cos(a2), rInner * std::sin(a2) };
        const vehicle_sim::math::Vec2 sIn1 { camera.worldToScreen(in1) };
        const vehicle_sim::math::Vec2 sIn2 { camera.worldToScreen(in2) };
        WireframeRenderer::drawLine(renderer,
                                    static_cast<int>(std::round(sIn1.x)), static_cast<int>(std::round(sIn1.y)),
                                    static_cast<int>(std::round(sIn2.x)), static_cast<int>(std::round(sIn2.y)),
                                    cyanBoundary);

        // Outer circle segment
        const vehicle_sim::math::Vec2 out1 { rOuter * std::cos(a1), rOuter * std::sin(a1) };
        const vehicle_sim::math::Vec2 out2 { rOuter * std::cos(a2), rOuter * std::sin(a2) };
        const vehicle_sim::math::Vec2 sOut1 { camera.worldToScreen(out1) };
        const vehicle_sim::math::Vec2 sOut2 { camera.worldToScreen(out2) };
        WireframeRenderer::drawLine(renderer,
                                    static_cast<int>(std::round(sOut1.x)), static_cast<int>(std::round(sOut1.y)),
                                    static_cast<int>(std::round(sOut2.x)), static_cast<int>(std::round(sOut2.y)),
                                    cyanBoundary);

        // Dashed Centerline segment
        if (i % 2 == 0) {
            const vehicle_sim::math::Vec2 mid1 { targetR * std::cos(a1), targetR * std::sin(a1) };
            const vehicle_sim::math::Vec2 mid2 { targetR * std::cos(a2), targetR * std::sin(a2) };
            const vehicle_sim::math::Vec2 sMid1 { camera.worldToScreen(mid1) };
            const vehicle_sim::math::Vec2 sMid2 { camera.worldToScreen(mid2) };
            WireframeRenderer::drawLine(renderer,
                                        static_cast<int>(std::round(sMid1.x)), static_cast<int>(std::round(sMid1.y)),
                                        static_cast<int>(std::round(sMid2.x)), static_cast<int>(std::round(sMid2.y)),
                                        tealCenterline);
        }
    }

    // Start/Finish Gate at theta = 0 (+x axis)
    const vehicle_sim::math::Vec2 startIn { rInner, 0.0 };
    const vehicle_sim::math::Vec2 startOut { rOuter, 0.0 };
    const vehicle_sim::math::Vec2 sStartIn { camera.worldToScreen(startIn) };
    const vehicle_sim::math::Vec2 sStartOut { camera.worldToScreen(startOut) };
    const SDL_Color greenGate { 0, 255, 120, 255 };
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sStartIn.x)), static_cast<int>(std::round(sStartIn.y)),
                                static_cast<int>(std::round(sStartOut.x)), static_cast<int>(std::round(sStartOut.y)),
                                greenGate);
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sStartIn.x)), static_cast<int>(std::round(sStartIn.y)) + 1,
                                static_cast<int>(std::round(sStartOut.x)), static_cast<int>(std::round(sStartOut.y)) + 1,
                                greenGate);

    // Cardinal sector splits at 90, 180, 270 deg
    const SDL_Color goldGate { 255, 220, 0, 180 };
    const std::array<double, 3> cardinalAngles { { 0.5 * kPi, kPi, 1.5 * kPi } };
    for (double ang : cardinalAngles) {
        const vehicle_sim::math::Vec2 ptIn { rInner * std::cos(ang), rInner * std::sin(ang) };
        const vehicle_sim::math::Vec2 ptOut { rOuter * std::cos(ang), rOuter * std::sin(ang) };
        const vehicle_sim::math::Vec2 sPtIn { camera.worldToScreen(ptIn) };
        const vehicle_sim::math::Vec2 sPtOut { camera.worldToScreen(ptOut) };
        WireframeRenderer::drawLine(renderer,
                                    static_cast<int>(std::round(sPtIn.x)), static_cast<int>(std::round(sPtIn.y)),
                                    static_cast<int>(std::round(sPtOut.x)), static_cast<int>(std::round(sPtOut.y)),
                                    goldGate);
    }
}

void SkidpadRenderer::drawRadialSpokes(SDL_Renderer* renderer,
                                       const MotionLabCamera& camera,
                                       const SkidpadCoordinator& coordinator) noexcept {
    const double targetR { coordinator.getTargetRadius() };
    const double halfW { coordinator.getCorridorWidth() * 0.5 };
    const double rInner { targetR - halfW };
    const double rOuter { targetR + halfW };

    const SDL_Color spokeColor { 0, 180, 220, 60 };
    constexpr int kSpokeCount { 12 }; // every 30 degrees

    for (int i { 0 }; i < kSpokeCount; ++i) {
        const double ang { (static_cast<double>(i) / static_cast<double>(kSpokeCount)) * kTwoPi };
        const vehicle_sim::math::Vec2 ptIn { rInner * std::cos(ang), rInner * std::sin(ang) };
        const vehicle_sim::math::Vec2 ptOut { rOuter * std::cos(ang), rOuter * std::sin(ang) };
        const vehicle_sim::math::Vec2 sPtIn { camera.worldToScreen(ptIn) };
        const vehicle_sim::math::Vec2 sPtOut { camera.worldToScreen(ptOut) };
        WireframeRenderer::drawLine(renderer,
                                    static_cast<int>(std::round(sPtIn.x)), static_cast<int>(std::round(sPtIn.y)),
                                    static_cast<int>(std::round(sPtOut.x)), static_cast<int>(std::round(sPtOut.y)),
                                    spokeColor);
    }
}

void SkidpadRenderer::drawStagingChute(SDL_Renderer* renderer,
                                       const MotionLabCamera& camera,
                                       const SkidpadCoordinator& coordinator) noexcept {
    const double targetR { coordinator.getTargetRadius() };
    const double halfW { coordinator.getCorridorWidth() * 0.5 };
    const double chuteLen { coordinator.getStagingLength() };

    const SDL_Color steelColor { 120, 140, 170, 140 };
    const SDL_Color greenGate { 0, 255, 120, 255 };

    // Left and right boundary lines of staging corridor at y = -targetR
    const vehicle_sim::math::Vec2 leftStart { -chuteLen, -targetR + halfW };
    const vehicle_sim::math::Vec2 leftEnd { 0.0, -targetR + halfW };
    const vehicle_sim::math::Vec2 rightStart { -chuteLen, -targetR - halfW };
    const vehicle_sim::math::Vec2 rightEnd { 0.0, -targetR - halfW };

    const vehicle_sim::math::Vec2 sL1 { camera.worldToScreen(leftStart) };
    const vehicle_sim::math::Vec2 sL2 { camera.worldToScreen(leftEnd) };
    const vehicle_sim::math::Vec2 sR1 { camera.worldToScreen(rightStart) };
    const vehicle_sim::math::Vec2 sR2 { camera.worldToScreen(rightEnd) };

    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sL1.x)), static_cast<int>(std::round(sL1.y)),
                                static_cast<int>(std::round(sL2.x)), static_cast<int>(std::round(sL2.y)),
                                steelColor);
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sR1.x)), static_cast<int>(std::round(sR1.y)),
                                static_cast<int>(std::round(sR2.x)), static_cast<int>(std::round(sR2.y)),
                                steelColor);

    // Staging start gate at x = -chuteLen
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sL1.x)), static_cast<int>(std::round(sL1.y)),
                                static_cast<int>(std::round(sR1.x)), static_cast<int>(std::round(sR1.y)),
                                greenGate);
}

void SkidpadRenderer::drawPylons(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const SkidpadCoordinator& coordinator) noexcept {
    const auto& pylons = coordinator.getPylons();
    const std::size_t count = coordinator.getActivePylonCount();

    for (std::size_t i { 0U }; i < count; ++i) {
        drawPylon(renderer, camera, pylons[i]);
    }
}

void SkidpadRenderer::drawPylon(SDL_Renderer* renderer,
                                const MotionLabCamera& camera,
                                const SkidpadPylon& pylon) noexcept {
    const vehicle_sim::math::Vec2 centerScreen { camera.worldToScreen(pylon.position) };
    const int cx { static_cast<int>(std::round(centerScreen.x)) };
    const int cy { static_cast<int>(std::round(centerScreen.y)) };

    const vehicle_sim::math::Vec2 edgeScreen { camera.worldToScreen(pylon.position + vehicle_sim::math::Vec2 { pylon.radius, 0.0 }) };
    const int screenRadius { std::max(10, static_cast<int>(std::round(std::abs(edgeScreen.x - centerScreen.x)))) };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    // Heavy rubber base plate
    const int baseHalf { static_cast<int>(screenRadius * 1.3) };
    const SDL_Rect baseRect { cx - baseHalf, cy - baseHalf, baseHalf * 2, baseHalf * 2 };
    SDL_SetRenderDrawColor(renderer, 20, 24, 30, 220);
    SDL_RenderFillRect(renderer, &baseRect);

    if (pylon.wasHit) {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 140, 0, 255);
    }
    SDL_RenderDrawRect(renderer, &baseRect);

    // Orange cone body
    const SDL_Color orangeColor = pylon.wasHit
        ? SDL_Color { 230, 40, 40, 240 }
        : SDL_Color { 255, 120, 0, 245 };
    drawFilledCircle(renderer, cx, cy, screenRadius, orangeColor);

    // Reflective white collar
    const int whiteRMin { static_cast<int>(screenRadius * 0.45) };
    const int whiteRMax { static_cast<int>(screenRadius * 0.72) };
    const SDL_Color whiteColor { 250, 250, 255, 255 };
    drawRing(renderer, cx, cy, whiteRMin, whiteRMax, whiteColor);
}

void SkidpadRenderer::drawDynamicGuide(SDL_Renderer* renderer,
                                       const MotionLabCamera& camera,
                                       const SkidpadCoordinator& coordinator) noexcept {
    (void)renderer;
    (void)camera;
    (void)coordinator;
}

void SkidpadRenderer::renderHud(SDL_Renderer* renderer,
                                const SkidpadCoordinator& coordinator,
                                int screenX,
                                int screenY) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    const int width { 310 };
    const int height { 145 };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 10, 16, 24, 220);
    const SDL_Rect bgRect { screenX, screenY, width, height };
    SDL_RenderFillRect(renderer, &bgRect);

    const SDL_Color borderColor { 0, 220, 255, 255 };
    SDL_SetRenderDrawColor(renderer, borderColor.r, borderColor.g, borderColor.b, borderColor.a);
    SDL_RenderDrawRect(renderer, &bgRect);

    const auto& metrics = coordinator.getMetrics();
    char buf[64] { };

    // Title & State
    const char* stateStr = "STAGING";
    if (coordinator.getState() == SkidpadState::Entering) {
        stateStr = "ENTERING";
    } else if (coordinator.getState() == SkidpadState::TrackingOrbit) {
        stateStr = metrics.isSteadyState ? "STEADY-STATE" : "ORBITING";
    } else if (coordinator.getState() == SkidpadState::OffCourse) {
        stateStr = "OFF-COURSE";
    }

    std::snprintf(buf, sizeof(buf), "SKIDPAD R:%.0fm [%s]", metrics.currentRadius, stateStr);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 10, 1, borderColor);

    // Direction & Radius Error
    const char* dirStr = "UNKNOWN";
    if (metrics.direction == SkidpadDirection::CounterClockwise) {
        dirStr = "CCW (LEFT)";
    } else if (metrics.direction == SkidpadDirection::Clockwise) {
        dirStr = "CW (RIGHT)";
    }
    std::snprintf(buf, sizeof(buf), "DIR: %s  ERR: %+.2fm", dirStr, metrics.radiusError);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 28, 1, SDL_Color { 220, 240, 255, 255 });

    // Lateral G & Speed
    std::snprintf(buf, sizeof(buf), "LAT G: %.2f G (PEAK: %.2f G)", metrics.instantaneousLatAcc, metrics.peakLatAcc);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 44, 1, SDL_Color { 255, 230, 40, 255 });

    // Official SAE Lap G & Current Lap
    std::snprintf(buf, sizeof(buf), "SAE G: %.2f G | LAP: %.2fs", metrics.lapAverageLatAcc, metrics.currentLapTime);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 62, 1, SDL_Color { 200, 255, 200, 255 });

    // Best Laps
    std::snprintf(buf, sizeof(buf), "BEST CCW: %.2fs | CW: %.2fs",
                  metrics.bestCcwLapTime, metrics.bestCwLapTime);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 78, 1, SDL_Color { 220, 240, 255, 255 });

    // Handling Classification Badge & Kus
    const char* balanceStr = "UNDERSTEER";
    SDL_Color balanceColor { 80, 255, 140, 255 }; // green
    if (metrics.balance == HandlingClassification::Oversteer) {
        balanceStr = "OVERSTEER";
        balanceColor = SDL_Color { 255, 80, 200, 255 }; // magenta
    } else if (metrics.balance == HandlingClassification::Neutral) {
        balanceStr = "NEUTRAL";
        balanceColor = SDL_Color { 255, 220, 0, 255 }; // gold
    }

    std::snprintf(buf, sizeof(buf), "[%s] Kus: %+4.1f deg/G", balanceStr, metrics.understeerGradient);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 96, 1, balanceColor);

    // Hits & Laps
    std::snprintf(buf, sizeof(buf), "LAPS: %u | CONE HITS: %u", metrics.completedLaps, metrics.hitCount);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 114, 1,
                           (metrics.hitCount > 0U) ? SDL_Color { 255, 80, 80, 255 } : SDL_Color { 140, 200, 255, 255 });
}

} // namespace motion_lab
