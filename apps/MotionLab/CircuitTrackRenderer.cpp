#include "apps/MotionLab/CircuitTrackRenderer.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "apps/MotionLab/WireframeRenderer.h"

namespace motion_lab {

void CircuitTrackRenderer::renderTrack(SDL_Renderer* renderer,
                                      const MotionLabCamera& camera,
                                      const CircuitTrackCoordinator& coordinator) noexcept
{
    if (!coordinator.isEnabled()) {
        return;
    }

    const auto& track = coordinator.getTrack();
    drawRibbon(renderer, camera, track);
    drawKerbs(renderer, camera, track);
    drawGates(renderer, camera, track);
    drawHazards(renderer, camera, track);
}

void CircuitTrackRenderer::drawRibbon(SDL_Renderer* renderer,
                                     const MotionLabCamera& camera,
                                     const vehicle_sim::terrain::CircuitTrack2D& track) noexcept
{
    const std::size_t numPoints { track.centerline.size() };
    if (numPoints < 2U || track.leftBoundary.size() != numPoints || track.rightBoundary.size() != numPoints) {
        return;
    }

    const SDL_Color roadColor { 80, 220, 255, 255 };
    const SDL_Color centerColor { 40, 160, 200, 140 };
    const SDL_Color shoulderColor { 180, 140, 60, 120 };

    const double halfTrack { 0.5 * track.trackWidth };
    const double shoulderRatio { (halfTrack > 1e-4) ? (track.shoulderWidth / halfTrack) : 0.4 };

    for (std::size_t i { 0U }; i < numPoints; ++i) {
        const std::size_t nextIdx { (i + 1U) % numPoints };

        // Road left and right vector boundaries
        const vehicle_sim::math::Vec2 sL1 { camera.worldToScreen(track.leftBoundary[i]) };
        const vehicle_sim::math::Vec2 sL2 { camera.worldToScreen(track.leftBoundary[nextIdx]) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sL1.x), static_cast<int>(sL1.y),
                                    static_cast<int>(sL2.x), static_cast<int>(sL2.y), roadColor);

        const vehicle_sim::math::Vec2 sR1 { camera.worldToScreen(track.rightBoundary[i]) };
        const vehicle_sim::math::Vec2 sR2 { camera.worldToScreen(track.rightBoundary[nextIdx]) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sR1.x), static_cast<int>(sR1.y),
                                    static_cast<int>(sR2.x), static_cast<int>(sR2.y), roadColor);

        // Dashed centerline
        if ((i % 2U) == 0U) {
            const vehicle_sim::math::Vec2 sC1 { camera.worldToScreen(track.centerline[i]) };
            const vehicle_sim::math::Vec2 sC2 { camera.worldToScreen(track.centerline[nextIdx]) };
            WireframeRenderer::drawLine(renderer, static_cast<int>(sC1.x), static_cast<int>(sC1.y),
                                        static_cast<int>(sC2.x), static_cast<int>(sC2.y), centerColor);
        }

        // Gravel shoulder boundaries
        const vehicle_sim::math::Vec2 shL1 { track.leftBoundary[i] + (track.leftBoundary[i] - track.centerline[i]) * shoulderRatio };
        const vehicle_sim::math::Vec2 shL2 { track.leftBoundary[nextIdx] + (track.leftBoundary[nextIdx] - track.centerline[nextIdx]) * shoulderRatio };
        const vehicle_sim::math::Vec2 sShL1 { camera.worldToScreen(shL1) };
        const vehicle_sim::math::Vec2 sShL2 { camera.worldToScreen(shL2) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sShL1.x), static_cast<int>(sShL1.y),
                                    static_cast<int>(sShL2.x), static_cast<int>(sShL2.y), shoulderColor);

        const vehicle_sim::math::Vec2 shR1 { track.rightBoundary[i] + (track.rightBoundary[i] - track.centerline[i]) * shoulderRatio };
        const vehicle_sim::math::Vec2 shR2 { track.rightBoundary[nextIdx] + (track.rightBoundary[nextIdx] - track.centerline[nextIdx]) * shoulderRatio };
        const vehicle_sim::math::Vec2 sShR1 { camera.worldToScreen(shR1) };
        const vehicle_sim::math::Vec2 sShR2 { camera.worldToScreen(shR2) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sShR1.x), static_cast<int>(sShR1.y),
                                    static_cast<int>(sShR2.x), static_cast<int>(sShR2.y), shoulderColor);
    }
}

void CircuitTrackRenderer::drawKerbs(SDL_Renderer* renderer,
                                    const MotionLabCamera& camera,
                                    const vehicle_sim::terrain::CircuitTrack2D& track) noexcept
{
    const std::size_t numPoints { track.centerline.size() };
    if (numPoints < 2U || track.bankAngles.size() != numPoints) {
        return;
    }

    const SDL_Color redKerb { 230, 40, 40, 255 };
    const SDL_Color whiteKerb { 240, 240, 240, 255 };
    constexpr double kKerbWidth { 0.8 };

    for (std::size_t i { 0U }; i < numPoints; ++i) {
        const double bankAngle { track.bankAngles[i] };
        if (std::abs(bankAngle) < 0.03) {
            continue;
        }

        const SDL_Color curColor { (i % 2U == 0U) ? redKerb : whiteKerb };

        if (bankAngle > 0.0) {
            // Apex is on the inside left
            const vehicle_sim::math::Vec2 basePt { track.leftBoundary[i] };
            const vehicle_sim::math::Vec2 outward { (track.leftBoundary[i] - track.centerline[i]).normalized() };
            const vehicle_sim::math::Vec2 tipPt { basePt + outward * kKerbWidth };

            const vehicle_sim::math::Vec2 sBase { camera.worldToScreen(basePt) };
            const vehicle_sim::math::Vec2 sTip { camera.worldToScreen(tipPt) };
            WireframeRenderer::drawLine(renderer, static_cast<int>(sBase.x), static_cast<int>(sBase.y),
                                        static_cast<int>(sTip.x), static_cast<int>(sTip.y), curColor);
        } else {
            // Apex is on the inside right
            const vehicle_sim::math::Vec2 basePt { track.rightBoundary[i] };
            const vehicle_sim::math::Vec2 outward { (track.rightBoundary[i] - track.centerline[i]).normalized() };
            const vehicle_sim::math::Vec2 tipPt { basePt + outward * kKerbWidth };

            const vehicle_sim::math::Vec2 sBase { camera.worldToScreen(basePt) };
            const vehicle_sim::math::Vec2 sTip { camera.worldToScreen(tipPt) };
            WireframeRenderer::drawLine(renderer, static_cast<int>(sBase.x), static_cast<int>(sBase.y),
                                        static_cast<int>(sTip.x), static_cast<int>(sTip.y), curColor);
        }
    }
}

void CircuitTrackRenderer::drawGates(SDL_Renderer* renderer,
                                    const MotionLabCamera& camera,
                                    const vehicle_sim::terrain::CircuitTrack2D& track) noexcept
{
    const std::size_t numPoints { track.centerline.size() };
    if (numPoints < 6U || track.leftBoundary.size() != numPoints || track.rightBoundary.size() != numPoints) {
        return;
    }

    const auto drawTransverseGate = [&](std::size_t idx, SDL_Color color) {
        const vehicle_sim::math::Vec2 sL { camera.worldToScreen(track.leftBoundary[idx]) };
        const vehicle_sim::math::Vec2 sR { camera.worldToScreen(track.rightBoundary[idx]) };
        WireframeRenderer::drawLine(renderer, static_cast<int>(sL.x), static_cast<int>(sL.y),
                                    static_cast<int>(sR.x), static_cast<int>(sR.y), color);
    };

    // Start / Finish Line (White double-line)
    const SDL_Color finishColor { 255, 255, 255, 255 };
    drawTransverseGate(0U, finishColor);
    const vehicle_sim::math::Vec2 sF1 { camera.worldToScreen(track.startGridPosition) };
    const vehicle_sim::math::Vec2 offsetFwd {
        (track.centerline[1] - track.centerline[0]).normalized() * 1.0
    };
    const vehicle_sim::math::Vec2 sF2 { camera.worldToScreen(track.startGridPosition + offsetFwd) };
    WireframeRenderer::drawLine(renderer, static_cast<int>(sF1.x), static_cast<int>(sF1.y),
                                static_cast<int>(sF2.x), static_cast<int>(sF2.y), finishColor);

    // Sector 1 Gate (Magenta)
    const SDL_Color sector1Color { 255, 80, 220, 255 };
    drawTransverseGate(numPoints / 3U, sector1Color);

    // Sector 2 Gate (Yellow)
    const SDL_Color sector2Color { 255, 220, 40, 255 };
    drawTransverseGate((2U * numPoints) / 3U, sector2Color);
}

void CircuitTrackRenderer::renderRadar(SDL_Renderer* renderer,
                                      const CircuitTrackCoordinator& coordinator,
                                      const vehicle_sim::math::Vec2& vehiclePos,
                                      int screenX,
                                      int screenY,
                                      int radarWidth,
                                      int radarHeight) noexcept
{
    if (!coordinator.isEnabled()) {
        return;
    }

    const auto& track = coordinator.getTrack();
    const std::size_t numPoints { track.centerline.size() };
    if (numPoints < 2U) {
        return;
    }

    // Draw background bezel
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    const SDL_Rect box { screenX, screenY, radarWidth, radarHeight };
    SDL_SetRenderDrawColor(renderer, 10, 15, 25, 200);
    SDL_RenderFillRect(renderer, &box);
    SDL_SetRenderDrawColor(renderer, 60, 90, 130, 255);
    SDL_RenderDrawRect(renderer, &box);

    // Compute bounding box of track centerline
    double minX { track.centerline[0].x };
    double maxX { track.centerline[0].x };
    double minY { track.centerline[0].y };
    double maxY { track.centerline[0].y };

    for (const auto& pt : track.centerline) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
    }

    const double spanX { std::max(10.0, maxX - minX) };
    const double spanY { std::max(10.0, maxY - minY) };

    const double margin { 12.0 };
    const double scaleX { (static_cast<double>(radarWidth) - 2.0 * margin) / spanX };
    const double scaleY { (static_cast<double>(radarHeight) - 2.0 * margin) / spanY };
    const double scale { std::min(scaleX, scaleY) };

    const double midWorldX { 0.5 * (minX + maxX) };
    const double midWorldY { 0.5 * (minY + maxY) };
    const double radarMidX { static_cast<double>(screenX) + 0.5 * static_cast<double>(radarWidth) };
    const double radarMidY { static_cast<double>(screenY) + 0.5 * static_cast<double>(radarHeight) };

    const auto worldToRadar = [&](const vehicle_sim::math::Vec2& w) -> SDL_Point {
        const double rx { radarMidX + (w.x - midWorldX) * scale };
        const double ry { radarMidY - (w.y - midWorldY) * scale }; // Invert Y for screen
        return SDL_Point { static_cast<int>(std::round(rx)), static_cast<int>(std::round(ry)) };
    };

    // Draw track centerline polyline on radar
    const SDL_Color loopColor { 70, 160, 220, 220 };
    for (std::size_t i { 0U }; i < numPoints; ++i) {
        const std::size_t nextIdx { (i + 1U) % numPoints };
        const SDL_Point p1 { worldToRadar(track.centerline[i]) };
        const SDL_Point p2 { worldToRadar(track.centerline[nextIdx]) };
        WireframeRenderer::drawLine(renderer, p1.x, p1.y, p2.x, p2.y, loopColor);
    }

    // Draw Start / Finish tick
    const SDL_Point startPt { worldToRadar(track.startGridPosition) };
    const SDL_Rect startBox { startPt.x - 2, startPt.y - 2, 5, 5 };
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderFillRect(renderer, &startBox);

    // Draw player vehicle blip
    const SDL_Point vehPt { worldToRadar(vehiclePos) };
    const SDL_Rect vehBox { vehPt.x - 2, vehPt.y - 2, 5, 5 };
    SDL_SetRenderDrawColor(renderer, 50, 255, 80, 255);
    SDL_RenderFillRect(renderer, &vehBox);

    // Draw hazards on radar
    const SDL_Color radarHazardColor { 255, 180, 50, 220 };
    for (const auto& h : track.hazards) {
        const SDL_Point hp { worldToRadar(h.position) };
        const SDL_Rect hBox { hp.x - 1, hp.y - 1, 3, 3 };
        SDL_SetRenderDrawColor(renderer, radarHazardColor.r, radarHazardColor.g, radarHazardColor.b, radarHazardColor.a);
        SDL_RenderFillRect(renderer, &hBox);
    }
}

void CircuitTrackRenderer::drawHazards(SDL_Renderer* renderer,
                                       const MotionLabCamera& camera,
                                       const vehicle_sim::terrain::CircuitTrack2D& track) noexcept
{
    if (track.hazards.empty()) {
        return;
    }

    const double zoom { camera.getZoom() };

    const auto& camCfg = camera.getConfig();

    for (const auto& h : track.hazards) {
        const vehicle_sim::math::Vec2 scr { camera.worldToScreen(h.position) };
        const int px { static_cast<int>(std::round(scr.x)) };
        const int py { static_cast<int>(std::round(scr.y)) };

        if (px < -50 || px > (camCfg.viewportWidth + 50) ||
            py < -50 || py > (camCfg.viewportHeight + 50)) {
            continue;
        }

        const double scrR { std::max(2.0, h.boundingRadius * zoom) };
        const int r { static_cast<int>(std::round(scrR)) };

        switch (h.type) {
        case vehicle_sim::terrain::HazardType::TrafficCone: {
            const SDL_Color coneOrange { 255, 120, 20, 255 };
            const SDL_Color coneWhite { 255, 255, 255, 255 };
            WireframeRenderer::drawLine(renderer, px - r, py, px + r, py, coneOrange);
            WireframeRenderer::drawLine(renderer, px, py - r, px, py + r, coneOrange);
            WireframeRenderer::drawLine(renderer, px - r / 2, py - r / 2, px + r / 2, py + r / 2, coneOrange);
            WireframeRenderer::drawLine(renderer, px - r / 2, py + r / 2, px + r / 2, py - r / 2, coneOrange);
            const SDL_Rect core { px - 1, py - 1, 3, 3 };
            SDL_SetRenderDrawColor(renderer, coneWhite.r, coneWhite.g, coneWhite.b, coneWhite.a);
            SDL_RenderFillRect(renderer, &core);
            break;
        }
        case vehicle_sim::terrain::HazardType::OilSlick: {
            const SDL_Color oilColor { 150, 60, 180, 220 };
            const int rx { static_cast<int>(std::round(scrR * 1.2)) };
            const int ry { static_cast<int>(std::round(scrR * 0.8)) };
            WireframeRenderer::drawLine(renderer, px - rx, py, px, py - ry, oilColor);
            WireframeRenderer::drawLine(renderer, px, py - ry, px + rx, py, oilColor);
            WireframeRenderer::drawLine(renderer, px + rx, py, px, py + ry, oilColor);
            WireframeRenderer::drawLine(renderer, px, py + ry, px - rx, py, oilColor);
            break;
        }
        case vehicle_sim::terrain::HazardType::StandingPuddle: {
            const SDL_Color puddleColor { 40, 200, 255, 200 };
            WireframeRenderer::drawLine(renderer, px - r, py, px + r, py, puddleColor);
            WireframeRenderer::drawLine(renderer, px, py - r, px, py + r, puddleColor);
            WireframeRenderer::drawLine(renderer, px - r * 7 / 10, py - r * 7 / 10, px + r * 7 / 10, py + r * 7 / 10, puddleColor);
            WireframeRenderer::drawLine(renderer, px - r * 7 / 10, py + r * 7 / 10, px + r * 7 / 10, py - r * 7 / 10, puddleColor);
            break;
        }
        case vehicle_sim::terrain::HazardType::Rock: {
            const SDL_Color rockColor { 190, 190, 200, 255 };
            const int h1 { r * 8 / 10 };
            const int w1 { r * 6 / 10 };
            WireframeRenderer::drawLine(renderer, px - w1, py - h1, px + w1, py - h1, rockColor);
            WireframeRenderer::drawLine(renderer, px + w1, py - h1, px + r, py, rockColor);
            WireframeRenderer::drawLine(renderer, px + r, py, px + w1, py + h1, rockColor);
            WireframeRenderer::drawLine(renderer, px + w1, py + h1, px - w1, py + h1, rockColor);
            WireframeRenderer::drawLine(renderer, px - w1, py + h1, px - r, py, rockColor);
            WireframeRenderer::drawLine(renderer, px - r, py, px - w1, py - h1, rockColor);
            break;
        }
        case vehicle_sim::terrain::HazardType::BoostPad: {
            const SDL_Color boostColor { 50, 255, 140, 255 };
            const double dirX { std::cos(h.orientation) };
            const double dirY { std::sin(h.orientation) };
            const vehicle_sim::math::Vec2 tip { static_cast<double>(px) + dirX * scrR * 1.2, static_cast<double>(py) - dirY * scrR * 1.2 };
            const vehicle_sim::math::Vec2 tail { static_cast<double>(px) - dirX * scrR * 1.2, static_cast<double>(py) + dirY * scrR * 1.2 };
            WireframeRenderer::drawArrow(renderer, tail, tip, boostColor);
            break;
        }
        default:
            break;
        }
    }
}

} // namespace motion_lab
