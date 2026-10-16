#include "apps/MotionLab/SlalomCourseRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

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

void SlalomCourseRenderer::renderCourse(SDL_Renderer* renderer,
                                        const MotionLabCamera& camera,
                                        const SlalomCourseCoordinator& coordinator) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    drawGates(renderer, camera, coordinator);
    drawWeaveGuides(renderer, camera, coordinator);

    const auto& pylons = coordinator.getPylons();
    for (const auto& pylon : pylons) {
        drawPylon(renderer, camera, pylon);
    }
}

void SlalomCourseRenderer::drawPylon(SDL_Renderer* renderer,
                                     const MotionLabCamera& camera,
                                     const SlalomPylon& pylon) noexcept {
    const vehicle_sim::math::Vec2 centerScreen { camera.worldToScreen(pylon.position) };
    const int cx { static_cast<int>(std::round(centerScreen.x)) };
    const int cy { static_cast<int>(std::round(centerScreen.y)) };

    // Determine screen pixel radius from camera zoom, clamped to minimum 14px for high visibility
    const vehicle_sim::math::Vec2 edgeScreen { camera.worldToScreen(pylon.position + vehicle_sim::math::Vec2 { pylon.radius, 0.0 }) };
    const int screenRadius { std::max(14, static_cast<int>(std::round(std::abs(edgeScreen.x - centerScreen.x)))) };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    // 1. Heavy square base plate (Rubber foundation pad)
    const int baseHalf { static_cast<int>(screenRadius * 1.35) };
    const SDL_Rect baseRect { cx - baseHalf, cy - baseHalf, baseHalf * 2, baseHalf * 2 };

    // Dark rubber fill
    SDL_SetRenderDrawColor(renderer, 20, 24, 30, 230);
    SDL_RenderFillRect(renderer, &baseRect);

    // High-contrast base boundary
    if (pylon.wasHit) {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 160, 0, 255);
    }
    SDL_RenderDrawRect(renderer, &baseRect);

    // 2. Outer fluorescent orange cone base disk
    const SDL_Color orangeColor = pylon.wasHit
        ? SDL_Color { 230, 40, 40, 240 }
        : SDL_Color { 255, 120, 0, 245 };
    drawFilledCircle(renderer, cx, cy, screenRadius, orangeColor);

    // 3. Reflective white safety collar band (Classic high-visibility traffic cone design)
    const int whiteRMin { static_cast<int>(screenRadius * 0.45) };
    const int whiteRMax { static_cast<int>(screenRadius * 0.72) };
    const SDL_Color whiteColor { 250, 250, 255, 255 };
    drawRing(renderer, cx, cy, whiteRMin, whiteRMax, whiteColor);

    // 4. Inner orange core
    const int coreRadius { static_cast<int>(screenRadius * 0.40) };
    drawFilledCircle(renderer, cx, cy, coreRadius, orangeColor);

    // 5. Bright apex center highlight / conical tip
    const int tipRadius { std::max(3, static_cast<int>(screenRadius * 0.18)) };
    const SDL_Color tipColor = pylon.wasHit
        ? SDL_Color { 255, 150, 150, 255 }
        : SDL_Color { 255, 240, 80, 255 };
    drawFilledCircle(renderer, cx, cy, tipRadius, tipColor);

    // 6. Upright 3D perspective wireframe generator ribs or toppled tilt silhouette
    if (pylon.wasHit) {
        // Draw toppled displacement silhouette
        const vehicle_sim::math::Vec2 apexWorld {
            pylon.position + pylon.toppleDir * (pylon.radius * 2.5)
        };
        const vehicle_sim::math::Vec2 apexScreen { camera.worldToScreen(apexWorld) };
        const int ax { static_cast<int>(std::round(apexScreen.x)) };
        const int ay { static_cast<int>(std::round(apexScreen.y)) };

        const SDL_Color toppleColor { 255, 70, 70, 255 };
        WireframeRenderer::drawLine(renderer, cx - screenRadius, cy, ax, ay, toppleColor);
        WireframeRenderer::drawLine(renderer, cx + screenRadius, cy, ax, ay, toppleColor);
        WireframeRenderer::drawLine(renderer, cx, cy - screenRadius, ax, ay, toppleColor);
        WireframeRenderer::drawLine(renderer, cx, cy + screenRadius, ax, ay, toppleColor);
        drawFilledCircle(renderer, ax, ay, tipRadius + 1, toppleColor);
    } else {
        // Elevation apex tip for 3D wireframe effect
        const vehicle_sim::math::Vec2 apexWorld { pylon.position.x, pylon.position.y + pylon.radius * 1.5 };
        const vehicle_sim::math::Vec2 apexScreen { camera.worldToScreen(apexWorld) };
        const int ax { static_cast<int>(std::round(apexScreen.x)) };
        const int ay { static_cast<int>(std::round(apexScreen.y)) };

        const SDL_Color ribColor { 255, 220, 80, 180 };
        WireframeRenderer::drawLine(renderer, cx - screenRadius, cy, ax, ay, ribColor);
        WireframeRenderer::drawLine(renderer, cx + screenRadius, cy, ax, ay, ribColor);
        WireframeRenderer::drawLine(renderer, cx, cy - screenRadius, ax, ay, ribColor);
        WireframeRenderer::drawLine(renderer, cx, cy + screenRadius, ax, ay, ribColor);
    }

    // 7. Floating cone number badge pill above pylon
    char numBuf[16] { };
    std::snprintf(numBuf, sizeof(numBuf), "%u", pylon.coneIndex);

    const int badgeW { (pylon.coneIndex >= 10U) ? 26 : 20 };
    const int badgeH { 13 };
    const int badgeX { cx - (badgeW / 2) };
    const int badgeY { cy - baseHalf - 18 };

    const SDL_Rect badgeRect { badgeX, badgeY, badgeW, badgeH };
    SDL_SetRenderDrawColor(renderer, 10, 15, 25, 230);
    SDL_RenderFillRect(renderer, &badgeRect);

    if (pylon.wasHit) {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
        SDL_RenderDrawRect(renderer, &badgeRect);
        TelemetryHud::drawText(renderer, numBuf, badgeX + 4, badgeY + 2, 1, SDL_Color { 255, 80, 80, 255 });
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 180, 0, 255);
        SDL_RenderDrawRect(renderer, &badgeRect);
        TelemetryHud::drawText(renderer, numBuf, badgeX + 4, badgeY + 2, 1, SDL_Color { 255, 240, 100, 255 });
    }
}

void SlalomCourseRenderer::drawGates(SDL_Renderer* renderer,
                                     const MotionLabCamera& camera,
                                     const SlalomCourseCoordinator& coordinator) noexcept {
    (void)coordinator;

    // Corridor bounds (y = -5.0m and y = +5.0m) with high-visibility dashed borders
    const SDL_Color corridorColor { 120, 120, 150, 180 };
    for (double x { -15.0 }; x < 115.0; x += 3.0) {
        const vehicle_sim::math::Vec2 sTop1 { camera.worldToScreen(vehicle_sim::math::Vec2 { x, 5.0 }) };
        const vehicle_sim::math::Vec2 sTop2 { camera.worldToScreen(vehicle_sim::math::Vec2 { x + 1.8, 5.0 }) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sTop1.x), static_cast<int>(sTop1.y),
                                    static_cast<int>(sTop2.x), static_cast<int>(sTop2.y), corridorColor);

        const vehicle_sim::math::Vec2 sBot1 { camera.worldToScreen(vehicle_sim::math::Vec2 { x, -5.0 }) };
        const vehicle_sim::math::Vec2 sBot2 { camera.worldToScreen(vehicle_sim::math::Vec2 { x + 1.8, -5.0 }) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sBot1.x), static_cast<int>(sBot1.y),
                                    static_cast<int>(sBot2.x), static_cast<int>(sBot2.y), corridorColor);
    }

    // Start Gate (x = 0.0m) - Double Neon Green line with banners
    const SDL_Color startColor { 40, 255, 120, 255 };
    const vehicle_sim::math::Vec2 startTop { camera.worldToScreen(vehicle_sim::math::Vec2 { 0.0, 5.0 }) };
    const vehicle_sim::math::Vec2 startBot { camera.worldToScreen(vehicle_sim::math::Vec2 { 0.0, -5.0 }) };
    WireframeRenderer::drawLine(renderer, static_cast<int>(startTop.x), static_cast<int>(startTop.y),
                                static_cast<int>(startBot.x), static_cast<int>(startBot.y), startColor);
    WireframeRenderer::drawLine(renderer, static_cast<int>(startTop.x) - 1, static_cast<int>(startTop.y),
                                static_cast<int>(startBot.x) - 1, static_cast<int>(startBot.y), startColor);

    // Start Gate banner label
    const vehicle_sim::math::Vec2 startLblPos { camera.worldToScreen(vehicle_sim::math::Vec2 { 0.0, 5.6 }) };
    TelemetryHud::drawText(renderer, "START", static_cast<int>(startLblPos.x) - 18, static_cast<int>(startLblPos.y), 1, startColor);

    // Finish Gate (x = 110.0m) - Double Golden Yellow Checkered line
    const SDL_Color finishColor { 255, 220, 40, 255 };
    const vehicle_sim::math::Vec2 finTop { camera.worldToScreen(vehicle_sim::math::Vec2 { 110.0, 5.0 }) };
    const vehicle_sim::math::Vec2 finBot { camera.worldToScreen(vehicle_sim::math::Vec2 { 110.0, -5.0 }) };
    WireframeRenderer::drawLine(renderer, static_cast<int>(finTop.x), static_cast<int>(finTop.y),
                                static_cast<int>(finBot.x), static_cast<int>(finBot.y), finishColor);
    WireframeRenderer::drawLine(renderer, static_cast<int>(finTop.x) + 1, static_cast<int>(finTop.y),
                                static_cast<int>(finBot.x) + 1, static_cast<int>(finBot.y), finishColor);

    // Finish Gate banner label
    const vehicle_sim::math::Vec2 finLblPos { camera.worldToScreen(vehicle_sim::math::Vec2 { 110.0, 5.6 }) };
    TelemetryHud::drawText(renderer, "FINISH", static_cast<int>(finLblPos.x) - 20, static_cast<int>(finLblPos.y), 1, finishColor);

    // Sector 1 Split (x = 35.0m) and Sector 2 Split (x = 75.0m)
    const SDL_Color splitColor { 80, 180, 255, 180 };
    for (double y { -5.0 }; y < 5.0; y += 1.0) {
        const vehicle_sim::math::Vec2 sec1P1 { camera.worldToScreen(vehicle_sim::math::Vec2 { 35.0, y }) };
        const vehicle_sim::math::Vec2 sec1P2 { camera.worldToScreen(vehicle_sim::math::Vec2 { 35.0, y + 0.6 }) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sec1P1.x), static_cast<int>(sec1P1.y),
                                    static_cast<int>(sec1P2.x), static_cast<int>(sec1P2.y), splitColor);

        const vehicle_sim::math::Vec2 sec2P1 { camera.worldToScreen(vehicle_sim::math::Vec2 { 75.0, y }) };
        const vehicle_sim::math::Vec2 sec2P2 { camera.worldToScreen(vehicle_sim::math::Vec2 { 75.0, y + 0.6 }) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sec2P1.x), static_cast<int>(sec2P1.y),
                                    static_cast<int>(sec2P2.x), static_cast<int>(sec2P2.y), splitColor);
    }

    const vehicle_sim::math::Vec2 s1LblPos { camera.worldToScreen(vehicle_sim::math::Vec2 { 35.0, 5.4 }) };
    TelemetryHud::drawText(renderer, "S1", static_cast<int>(s1LblPos.x) - 8, static_cast<int>(s1LblPos.y), 1, splitColor);

    const vehicle_sim::math::Vec2 s2LblPos { camera.worldToScreen(vehicle_sim::math::Vec2 { 75.0, 5.4 }) };
    TelemetryHud::drawText(renderer, "S2", static_cast<int>(s2LblPos.x) - 8, static_cast<int>(s2LblPos.y), 1, splitColor);
}

void SlalomCourseRenderer::drawWeaveGuides(SDL_Renderer* renderer,
                                          const MotionLabCamera& camera,
                                          const SlalomCourseCoordinator& coordinator) noexcept {
    // 1. Draw persistent alternating weave passage markers on the ground for all cones
    const auto& pylons = coordinator.getPylons();
    for (std::size_t i { 0U }; i < pylons.size(); ++i) {
        const double weaveSign { (i % 2U == 0U) ? 1.0 : -1.0 };
        const vehicle_sim::math::Vec2 guideWorld {
            pylons[i].position.x,
            pylons[i].position.y + weaveSign * 2.4
        };
        const vehicle_sim::math::Vec2 guideScreen { camera.worldToScreen(guideWorld) };
        const int gx { static_cast<int>(std::round(guideScreen.x)) };
        const int gy { static_cast<int>(std::round(guideScreen.y)) };

        const SDL_Color passGuideColor { 0, 220, 255, 140 };
        const int chevronDir { (weaveSign > 0.0) ? -1 : 1 }; // Screen Y is inverted
        WireframeRenderer::drawLine(renderer, gx - 8, gy + chevronDir * 8, gx, gy, passGuideColor);
        WireframeRenderer::drawLine(renderer, gx + 8, gy + chevronDir * 8, gx, gy, passGuideColor);
    }

    // 2. Dynamic pulsating highlight for the next upcoming active target pylon
    if (coordinator.getState() == SlalomState::InProgress) {
        const std::uint32_t nextCone = coordinator.getNextConeIndex();
        if (nextCone >= 1U && nextCone <= pylons.size()) {
            const auto& targetPylon = pylons[nextCone - 1U];
            const double weaveSign = coordinator.getNextWeaveSign();
            const vehicle_sim::math::Vec2 targetSidePos {
                targetPylon.position.x,
                targetPylon.position.y + weaveSign * 2.4
            };

            const vehicle_sim::math::Vec2 screenPt { camera.worldToScreen(targetSidePos) };
            const SDL_Color activeColor { 255, 230, 40, 255 };

            const int px { static_cast<int>(std::round(screenPt.x)) };
            const int py { static_cast<int>(std::round(screenPt.y)) };
            const int dir { (weaveSign > 0.0) ? -1 : 1 };

            // Bold active chevron indicator
            WireframeRenderer::drawLine(renderer, px - 12, py + dir * 12, px, py, activeColor);
            WireframeRenderer::drawLine(renderer, px + 12, py + dir * 12, px, py, activeColor);
            WireframeRenderer::drawLine(renderer, px - 12, py + dir * 13, px, py + 1, activeColor);
            WireframeRenderer::drawLine(renderer, px + 12, py + dir * 13, px, py + 1, activeColor);
        }
    }
}

void SlalomCourseRenderer::renderHud(SDL_Renderer* renderer,
                                     const SlalomCourseCoordinator& coordinator,
                                     int screenX,
                                     int screenY) noexcept {
    if (!coordinator.isEnabled()) {
        return;
    }

    const int width { 290 };
    const int height { 135 };

    // Semi-transparent background
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 10, 16, 24, 220);
    const SDL_Rect bgRect { screenX, screenY, width, height };
    SDL_RenderFillRect(renderer, &bgRect);

    // High-visibility border
    const SDL_Color borderColor { 255, 140, 0, 255 };
    SDL_SetRenderDrawColor(renderer, borderColor.r, borderColor.g, borderColor.b, borderColor.a);
    SDL_RenderDrawRect(renderer, &bgRect);

    const auto& metrics = coordinator.getMetrics();
    char buf[64] { };

    // Title & State
    const char* stateStr = "IDLE";
    if (coordinator.getState() == SlalomState::InProgress) {
        stateStr = "RUNNING";
    } else if (coordinator.getState() == SlalomState::Finished) {
        stateStr = "FINISHED";
    }
    std::snprintf(buf, sizeof(buf), "100M SLALOM [%s]", stateStr);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 10, 1, borderColor);

    // Elapsed & Adjusted Time
    std::snprintf(buf, sizeof(buf), "TIME: %.2fs  (+%.1fs PEN)", metrics.elapsedTime, metrics.penaltyTime);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 28, 1, SDL_Color { 220, 240, 255, 255 });

    std::snprintf(buf, sizeof(buf), "FINAL: %.2fs (BEST: %.2fs)",
                  metrics.finalTime, metrics.hasValidBest ? metrics.bestTime : 0.0);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 44, 1, SDL_Color { 220, 240, 255, 255 });

    // Hits & Splits
    std::snprintf(buf, sizeof(buf), "HITS: %u  MISSED: %u", metrics.hitCount, metrics.missedGates);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 62, 1,
                           (metrics.hitCount > 0U) ? SDL_Color { 255, 80, 80, 255 } : SDL_Color { 80, 255, 140, 255 });

    std::snprintf(buf, sizeof(buf), "S1: %.2fs | S2: %.2fs | S3: %.2fs",
                  metrics.sector1Time, metrics.sector2Time, metrics.sector3Time);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 80, 1, SDL_Color { 140, 200, 255, 255 });

    // Target Weave Prompt
    const char* weavePrompt = (coordinator.getNextWeaveSign() > 0.0) ? "NEXT: WEAVE LEFT  <<<" : "NEXT: WEAVE RIGHT >>>";
    std::snprintf(buf, sizeof(buf), "CONE %u/10: %s", coordinator.getNextConeIndex(), weavePrompt);
    TelemetryHud::drawText(renderer, buf, screenX + 10, screenY + 104, 1, SDL_Color { 0, 240, 255, 255 });
}

} // namespace motion_lab
