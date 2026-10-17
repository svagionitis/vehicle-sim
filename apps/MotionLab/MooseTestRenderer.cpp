#include "apps/MotionLab/MooseTestRenderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "apps/MotionLab/TelemetryHud.h"
#include "apps/MotionLab/WireframeRenderer.h"

namespace motion_lab {

namespace {

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

void MooseTestRenderer::renderCourse(SDL_Renderer* renderer,
                                     const MotionLabCamera& camera,
                                     const MooseTestCoordinator& coordinator) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    drawBoundaries(renderer, camera, coordinator);
    drawTrajectoryRoute(renderer, camera, coordinator);
    drawGates(renderer, camera, coordinator);
    drawChevrons(renderer, camera, coordinator);

    const auto& cones = coordinator.getCones();
    for (const auto& cone : cones) {
        drawPylon(renderer, camera, cone);
    }
}

void MooseTestRenderer::drawPylon(SDL_Renderer* renderer,
                                  const MotionLabCamera& camera,
                                  const MooseTestCone& cone) noexcept {
    const vehicle_sim::math::Vec2 centerScreen { camera.worldToScreen(cone.position) };
    const int cx { static_cast<int>(std::round(centerScreen.x)) };
    const int cy { static_cast<int>(std::round(centerScreen.y)) };

    const vehicle_sim::math::Vec2 edgeScreen { camera.worldToScreen(cone.position + vehicle_sim::math::Vec2 { cone.radius, 0.0 }) };
    const int screenRadius { std::max(12, static_cast<int>(std::round(std::abs(edgeScreen.x - centerScreen.x)))) };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    // 1. Heavy base pad
    const int baseHalf { static_cast<int>(screenRadius * 1.30) };
    const SDL_Rect baseRect { cx - baseHalf, cy - baseHalf, baseHalf * 2, baseHalf * 2 };

    SDL_SetRenderDrawColor(renderer, 20, 24, 30, 230);
    SDL_RenderFillRect(renderer, &baseRect);

    if (cone.wasHit) {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 140, 0, 255);
    }
    SDL_RenderDrawRect(renderer, &baseRect);

    // 2. Cone base disk
    const SDL_Color diskColor = cone.wasHit
        ? SDL_Color { 240, 40, 40, 240 }
        : SDL_Color { 255, 120, 0, 245 };
    drawFilledCircle(renderer, cx, cy, screenRadius, diskColor);

    // 3. Reflective white safety collar
    const int whiteRMin { static_cast<int>(screenRadius * 0.45) };
    const int whiteRMax { static_cast<int>(screenRadius * 0.70) };
    const SDL_Color whiteColor { 250, 250, 255, 255 };
    drawRing(renderer, cx, cy, whiteRMin, whiteRMax, whiteColor);

    // 4. Core apex
    const int coreRadius { static_cast<int>(screenRadius * 0.38) };
    const SDL_Color coreColor = cone.wasHit
        ? SDL_Color { 255, 80, 80, 255 }
        : SDL_Color { 255, 160, 20, 255 };
    drawFilledCircle(renderer, cx, cy, coreRadius, coreColor);

    // 5. Impact topple displacement indicator
    if (cone.wasHit && cone.toppleAngle > 0.1) {
        const int tipDist { static_cast<int>(screenRadius * 2.5) };
        const int tipX { cx + static_cast<int>(std::round(cone.toppleDir.x * tipDist)) };
        const int tipY { cy - static_cast<int>(std::round(cone.toppleDir.y * tipDist)) };

        const SDL_Color hitRayColor { 255, 60, 60, 220 };
        WireframeRenderer::drawLine(renderer, cx, cy, tipX, tipY, hitRayColor);
        drawFilledCircle(renderer, tipX, tipY, 4, SDL_Color { 255, 20, 20, 255 });
    }
}

void MooseTestRenderer::drawGates(SDL_Renderer* renderer,
                                  const MotionLabCamera& camera,
                                  const MooseTestCoordinator& coordinator) noexcept {
    const auto& sections = coordinator.getSections();
    const double w1 { sections[0].width };
    const double w5 { sections[4].width };

    // 1. Entry Gate at x = 0.0m (Green)
    const vehicle_sim::math::Vec2 startTop { 0.0, +w1 * 0.5 + 0.6 };
    const vehicle_sim::math::Vec2 startBot { 0.0, -w1 * 0.5 - 0.6 };
    const vehicle_sim::math::Vec2 sTopScr { camera.worldToScreen(startTop) };
    const vehicle_sim::math::Vec2 sBotScr { camera.worldToScreen(startBot) };

    const SDL_Color greenGate { 40, 255, 120, 255 };
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(sTopScr.x)),
                                static_cast<int>(std::round(sTopScr.y)),
                                static_cast<int>(std::round(sBotScr.x)),
                                static_cast<int>(std::round(sBotScr.y)),
                                greenGate);

    // 2. Apex Midpoint Gate at x = 31.0m (Cyan)
    const double w3 { sections[2].width };
    const vehicle_sim::math::Vec2 apexTop { 31.0, 3.5 + w3 * 0.5 + 0.5 };
    const vehicle_sim::math::Vec2 apexBot { 31.0, 3.5 - w3 * 0.5 - 0.5 };
    const vehicle_sim::math::Vec2 aTopScr { camera.worldToScreen(apexTop) };
    const vehicle_sim::math::Vec2 aBotScr { camera.worldToScreen(apexBot) };

    const SDL_Color cyanGate { 0, 230, 255, 200 };
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(aTopScr.x)),
                                static_cast<int>(std::round(aTopScr.y)),
                                static_cast<int>(std::round(aBotScr.x)),
                                static_cast<int>(std::round(aBotScr.y)),
                                cyanGate);

    // 3. Finish Gate at x = 61.0m (Gold)
    const vehicle_sim::math::Vec2 finishTop { 61.0, +w5 * 0.5 + 0.6 };
    const vehicle_sim::math::Vec2 finishBot { 61.0, -w5 * 0.5 - 0.6 };
    const vehicle_sim::math::Vec2 fTopScr { camera.worldToScreen(finishTop) };
    const vehicle_sim::math::Vec2 fBotScr { camera.worldToScreen(finishBot) };

    const SDL_Color goldGate { 255, 220, 40, 255 };
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(fTopScr.x)),
                                static_cast<int>(std::round(fTopScr.y)),
                                static_cast<int>(std::round(fBotScr.x)),
                                static_cast<int>(std::round(fBotScr.y)),
                                goldGate);
}

void MooseTestRenderer::drawBoundaries(SDL_Renderer* renderer,
                                      const MotionLabCamera& camera,
                                      const MooseTestCoordinator& coordinator) noexcept {
    const auto& cones = coordinator.getCones();
    const SDL_Color boundaryColor { 100, 140, 200, 140 };

    auto drawSegment = [&](std::size_t c1, std::size_t c2) {
        if (c1 < cones.size() && c2 < cones.size()) {
            const vehicle_sim::math::Vec2 p1 { camera.worldToScreen(cones[c1].position) };
            const vehicle_sim::math::Vec2 p2 { camera.worldToScreen(cones[c2].position) };
            WireframeRenderer::drawLine(renderer,
                                        static_cast<int>(std::round(p1.x)),
                                static_cast<int>(std::round(p1.y)),
                                static_cast<int>(std::round(p2.x)),
                                static_cast<int>(std::round(p2.y)),
                                boundaryColor);
        }
    };

    // Section 1 Left Boundary: Cones 0 -> 2 -> 4
    drawSegment(0, 2);
    drawSegment(2, 4);

    // Section 1 Right Boundary: Cones 1 -> 3 -> 5
    drawSegment(1, 3);
    drawSegment(3, 5);

    // Transition Section 2: Cones 5 -> 6, Cones 4 -> 8
    drawSegment(5, 6);
    drawSegment(4, 8);

    // Section 3 Left Boundary: Cones 8 -> 10 -> 12
    drawSegment(8, 10);
    drawSegment(10, 12);

    // Section 3 Right Boundary: Cones 9 -> 11 -> 13
    drawSegment(9, 11);
    drawSegment(11, 13);

    // Transition Section 4: Cones 12 -> 14 -> 16, Cones 13 -> 17
    drawSegment(12, 14);
    drawSegment(14, 16);
    drawSegment(13, 17);

    // Section 5 Left Boundary: Cones 16 -> 18 -> 20 -> 22
    drawSegment(16, 18);
    drawSegment(18, 20);
    drawSegment(20, 22);

    // Section 5 Right Boundary: Cones 17 -> 19 -> 21 -> 23
    drawSegment(17, 19);
    drawSegment(19, 21);
    drawSegment(21, 23);
}

void MooseTestRenderer::drawChevrons(SDL_Renderer* renderer,
                                     const MotionLabCamera& camera,
                                     const MooseTestCoordinator& coordinator) noexcept {
    if (coordinator.getState() != MooseTestState::InManeuver) {
        return;
    }

    const SDL_Color chevColor { 0, 240, 255, 200 };

    // Section 2 Left Evasion Chevron (around x = 18m, y = 1.7m)
    const vehicle_sim::math::Vec2 c1 { camera.worldToScreen(vehicle_sim::math::Vec2 { 18.0, 1.7 }) };
    const int px1 { static_cast<int>(std::round(c1.x)) };
    const int py1 { static_cast<int>(std::round(c1.y)) };
    WireframeRenderer::drawLine(renderer, px1 - 10, py1 + 10, px1 + 5, py1, chevColor);
    WireframeRenderer::drawLine(renderer, px1 - 10, py1 - 10, px1 + 5, py1, chevColor);

    // Section 4 Right Return Chevron (around x = 42m, y = 1.7m)
    const vehicle_sim::math::Vec2 c2 { camera.worldToScreen(vehicle_sim::math::Vec2 { 42.0, 1.7 }) };
    const int px2 { static_cast<int>(std::round(c2.x)) };
    const int py2 { static_cast<int>(std::round(c2.y)) };
    WireframeRenderer::drawLine(renderer, px2 - 5, py2, px2 + 10, py2 + 10, chevColor);
    WireframeRenderer::drawLine(renderer, px2 - 5, py2, px2 + 10, py2 - 10, chevColor);
}

void MooseTestRenderer::drawTrajectoryRoute(SDL_Renderer* renderer,
                                            const MotionLabCamera& camera,
                                            const MooseTestCoordinator& coordinator) noexcept {
    (void)coordinator;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    constexpr double kMaxShift { 3.5 };
    constexpr double kStartX { -30.0 }; // Staging spawn coordinate
    constexpr double kEndX { 75.0 };    // Run-off braking zone coordinate
    constexpr double kDashLength { 1.2 };
    constexpr double kGapLength { 0.6 };
    constexpr double kCycleLength { kDashLength + kGapLength };
    constexpr double kSubStep { 0.2 };

    auto computeRouteY = [](double x) noexcept -> double {
        if (x <= 12.0) {
            // Staging approach and Section 1 Entry Corridor along centerline
            return 0.0;
        }
        if (x < 25.5) {
            // Shortest direct diagonal chord across Section 2 with cubic Hermite fillets (dy = +3.5m)
            const double u { (x - 12.0) / 13.5 };
            return kMaxShift * (3.0 * u * u - 2.0 * u * u * u);
        }
        if (x <= 36.5) {
            // Section 3: Evasion Corridor along shifted lane (y = +3.5m)
            return kMaxShift;
        }
        if (x < 49.0) {
            // Shortest direct diagonal chord across Section 4 with cubic Hermite fillets (dy = -3.5m)
            const double v { (x - 36.5) / 12.5 };
            return kMaxShift * (1.0 - (3.0 * v * v - 2.0 * v * v * v));
        }
        // Section 5 & Run-off: Exit Corridor along centerline
        return 0.0;
    };

    auto drawBoldLine = [&](int x1, int y1, int x2, int y2, SDL_Color color) noexcept {
        WireframeRenderer::drawLine(renderer, x1, y1, x2, y2, color);
        WireframeRenderer::drawLine(renderer, x1 + 1, y1, x2 + 1, y2, color);
        WireframeRenderer::drawLine(renderer, x1 - 1, y1, x2 - 1, y2, color);
        WireframeRenderer::drawLine(renderer, x1, y1 + 1, x2, y2 + 1, color);
        WireframeRenderer::drawLine(renderer, x1, y1 - 1, x2, y2 - 1, color);
    };

    const SDL_Color routeColor { 0, 255, 240, 255 };
    const SDL_Color glowColor { 0, 180, 255, 180 };
    const SDL_Color beadCoreColor { 220, 255, 255, 255 };

    // 1. Draw bold 3-pixel-wide dashed double lane-change trajectory route and waypoint beads
    for (double dashStart { kStartX }; dashStart < kEndX; dashStart += kCycleLength) {
        const double dashEnd { std::min(dashStart + kDashLength, kEndX) };

        double currX { dashStart };
        while (currX < dashEnd) {
            const double nextX { std::min(currX + kSubStep, dashEnd) };

            const vehicle_sim::math::Vec2 p1World { currX, computeRouteY(currX) };
            const vehicle_sim::math::Vec2 p2World { nextX, computeRouteY(nextX) };

            const vehicle_sim::math::Vec2 p1Screen { camera.worldToScreen(p1World) };
            const vehicle_sim::math::Vec2 p2Screen { camera.worldToScreen(p2World) };

            drawBoldLine(static_cast<int>(std::round(p1Screen.x)),
                         static_cast<int>(std::round(p1Screen.y)),
                         static_cast<int>(std::round(p2Screen.x)),
                         static_cast<int>(std::round(p2Screen.y)),
                         routeColor);

            currX = nextX;
        }

        // Prominent glowing waypoint bead at center of each dash
        const double midDashX { 0.5 * (dashStart + dashEnd) };
        const vehicle_sim::math::Vec2 midWorld { midDashX, computeRouteY(midDashX) };
        const vehicle_sim::math::Vec2 midScreen { camera.worldToScreen(midWorld) };
        const int bx { static_cast<int>(std::round(midScreen.x)) };
        const int by { static_cast<int>(std::round(midScreen.y)) };

        drawFilledCircle(renderer, bx, by, 4, glowColor);
        drawFilledCircle(renderer, bx, by, 2, beadCoreColor);
    }

    // 2. Bold directional guidance chevrons across key maneuver sections
    const SDL_Color chevronColor { 100, 255, 250, 255 };
    const std::array<double, 8> guideStations { -20.0, -8.0, 6.0, 18.75, 31.0, 42.75, 55.0, 68.0 };

    for (const double stationX : guideStations) {
        const double yPos { computeRouteY(stationX) };
        const double yPosAhead { computeRouteY(stationX + 0.6) };

        const vehicle_sim::math::Vec2 pMidWorld { stationX, yPos };
        const vehicle_sim::math::Vec2 pAheadWorld { stationX + 0.6, yPosAhead };

        const vehicle_sim::math::Vec2 pMidScr { camera.worldToScreen(pMidWorld) };
        const vehicle_sim::math::Vec2 pAheadScr { camera.worldToScreen(pAheadWorld) };

        vehicle_sim::math::Vec2 dir { pAheadScr.x - pMidScr.x, pAheadScr.y - pMidScr.y };
        const double len { std::sqrt(dir.x * dir.x + dir.y * dir.y) };
        if (len > 0.001) {
            dir.x /= len;
            dir.y /= len;
            const vehicle_sim::math::Vec2 normal { -dir.y, dir.x };

            const int mx { static_cast<int>(std::round(pMidScr.x)) };
            const int my { static_cast<int>(std::round(pMidScr.y)) };

            const int wing1X { static_cast<int>(std::round(pMidScr.x - dir.x * 8.0 + normal.x * 6.5)) };
            const int wing1Y { static_cast<int>(std::round(pMidScr.y - dir.y * 8.0 + normal.y * 6.5)) };
            const int wing2X { static_cast<int>(std::round(pMidScr.x - dir.x * 8.0 - normal.x * 6.5)) };
            const int wing2Y { static_cast<int>(std::round(pMidScr.y - dir.y * 8.0 - normal.y * 6.5)) };

            drawBoldLine(wing1X, wing1Y, mx, my, chevronColor);
            drawBoldLine(wing2X, wing2Y, mx, my, chevronColor);
        }
    }
}

void MooseTestRenderer::renderHud(SDL_Renderer* renderer,
                                  const MooseTestCoordinator& coordinator,
                                  int screenX,
                                  int screenY) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    const int width { 330 };
    const int height { 145 };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 10, 16, 24, 225);
    const SDL_Rect bgRect { screenX, screenY, width, height };
    SDL_RenderFillRect(renderer, &bgRect);

    const SDL_Color borderColor { 0, 200, 255, 255 };
    SDL_SetRenderDrawColor(renderer, borderColor.r, borderColor.g, borderColor.b, borderColor.a);
    SDL_RenderDrawRect(renderer, &bgRect);

    const auto& metrics = coordinator.getMetrics();
    char buf[80] { };

    // Title & State
    const char* stateStr = "IDLE";
    if (coordinator.getState() == MooseTestState::Approaching) {
        stateStr = "APPROACHING";
    } else if (coordinator.getState() == MooseTestState::InManeuver) {
        stateStr = "IN MANEUVER";
    } else if (coordinator.getState() == MooseTestState::Completed) {
        stateStr = "COMPLETED";
    } else if (coordinator.getState() == MooseTestState::FailedOffCourse) {
        stateStr = "SPUN OUT";
    }

    std::snprintf(buf, sizeof(buf), "ISO 3888-2 MOOSE TEST [%s]", stateStr);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 10, 1, borderColor);

    // Entry Speed & Target
    std::snprintf(buf, sizeof(buf), "ENTRY: %.1f km/h  (TARGET: %.1f km/h)",
                  metrics.entrySpeed, coordinator.getTargetSpeed());
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 28, 1, SDL_Color { 220, 240, 255, 255 });

    // Apex & Exit Speed
    std::snprintf(buf, sizeof(buf), "APEX: %.1f km/h | EXIT: %.1f km/h",
                  metrics.evasionSpeed, metrics.exitSpeed);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 44, 1, SDL_Color { 140, 200, 255, 255 });

    // Speed Drop & Lateral G
    std::snprintf(buf, sizeof(buf), "DROP: %.1f km/h | PEAK: %.2f G (%.1f d/s)",
                  metrics.speedDrop, metrics.peakLatAcc, metrics.peakYawRate);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 62, 1, SDL_Color { 220, 240, 255, 255 });

    // Faults & Cones
    const char* throttleStr = metrics.throttleFault ? "VIOLATION" : "CUT (OK)";
    std::snprintf(buf, sizeof(buf), "CONES HIT: %u | THROTTLE: %s", metrics.hitCount, throttleStr);
    const SDL_Color statusColor = (metrics.hitCount > 0U || metrics.throttleFault)
        ? SDL_Color { 255, 80, 80, 255 }
        : SDL_Color { 80, 255, 140, 255 };
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 80, 1, statusColor);

    // Result & Best Pass
    const char* resultStr = "PENDING";
    if (metrics.result == MooseRunResult::CleanPass) {
        resultStr = "CLEAN PASS";
    } else if (metrics.result == MooseRunResult::FailedConeHit) {
        resultStr = "FAILED (CONE HIT)";
    } else if (metrics.result == MooseRunResult::Disqualified) {
        resultStr = "DISQUALIFIED (THROTTLE/BRAKE)";
    } else if (metrics.result == MooseRunResult::FailedSpinOut) {
        resultStr = "FAILED (SPIN-OUT)";
    }

    std::snprintf(buf, sizeof(buf), "RESULT: %s", resultStr);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 102, 1,
                           (metrics.result == MooseRunResult::CleanPass) ? SDL_Color { 40, 255, 120, 255 } : SDL_Color { 255, 200, 80, 255 });

    std::snprintf(buf, sizeof(buf), "MAX CLEAN PASS: %.1f km/h",
                  metrics.hasValidBest ? metrics.bestPassSpeed : 0.0);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 120, 1, SDL_Color { 255, 220, 40, 255 });
}

} // namespace motion_lab
