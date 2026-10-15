#include "apps/MotionLab/WireframeRenderer.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

void WireframeRenderer::drawLine(SDL_Renderer* renderer,
                                 int x1, int y1,
                                 int x2, int y2,
                                 SDL_Color color) noexcept
{
    if (renderer == nullptr) {
        return;
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

void WireframeRenderer::drawArrow(SDL_Renderer* renderer,
                                 const vehicle_sim::math::Vec2& screenStart,
                                 const vehicle_sim::math::Vec2& screenEnd,
                                 SDL_Color color) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const int x1 { static_cast<int>(std::round(screenStart.x)) };
    const int y1 { static_cast<int>(std::round(screenStart.y)) };
    const int x2 { static_cast<int>(std::round(screenEnd.x)) };
    const int y2 { static_cast<int>(std::round(screenEnd.y)) };

    drawLine(renderer, x1, y1, x2, y2, color);

    const double angle { std::atan2(screenEnd.y - screenStart.y, screenEnd.x - screenStart.x) };
    constexpr double kHeadLen { 10.0 };
    constexpr double kHeadAngle { 0.4 };

    const int hx1 { static_cast<int>(std::round(screenEnd.x - kHeadLen * std::cos(angle - kHeadAngle))) };
    const int hy1 { static_cast<int>(std::round(screenEnd.y - kHeadLen * std::sin(angle - kHeadAngle))) };
    const int hx2 { static_cast<int>(std::round(screenEnd.x - kHeadLen * std::cos(angle + kHeadAngle))) };
    const int hy2 { static_cast<int>(std::round(screenEnd.y - kHeadLen * std::sin(angle + kHeadAngle))) };

    drawLine(renderer, x2, y2, hx1, hy1, color);
    drawLine(renderer, x2, y2, hx2, hy2, color);
}

namespace {
struct TerrainTheme {
    SDL_Color background { 8, 9, 10, 255 };
    SDL_Color grid { 41, 60, 70, 255 };
    SDL_Color skid { 242, 138, 114, 255 };
};

[[nodiscard]] TerrainTheme getTerrainTheme(vehicle_sim::terrain::MaterialType type) noexcept
{
    using vehicle_sim::terrain::MaterialType;
    switch (type) {
    case MaterialType::Ice:
        return { { 6, 12, 18, 255 }, { 60, 110, 140, 255 }, { 165, 216, 255, 255 } };
    case MaterialType::Sand:
        return { { 18, 14, 9, 255 }, { 115, 88, 52, 255 }, { 232, 176, 104, 255 } };
    case MaterialType::Mud:
        return { { 14, 10, 7, 255 }, { 95, 68, 45, 255 }, { 140, 92, 52, 255 } };
    case MaterialType::Grass:
        return { { 7, 16, 9, 255 }, { 45, 95, 55, 255 }, { 124, 179, 66, 255 } };
    case MaterialType::Gravel:
        return { { 12, 13, 14, 255 }, { 80, 85, 90, 255 }, { 176, 190, 197, 255 } };
    case MaterialType::Snow:
        return { { 11, 15, 20, 255 }, { 95, 115, 135, 255 }, { 215, 232, 245, 255 } };
    case MaterialType::Tarmac:
    default:
        return { { 8, 9, 10, 255 }, { 41, 60, 70, 255 }, { 242, 138, 114, 255 } };
    }
}
} // namespace

void WireframeRenderer::drawGrid(SDL_Renderer* renderer, const MotionLabCamera& camera, SDL_Color gridColor) noexcept
{
    const int screenW { camera.getConfig().viewportWidth };
    const int screenH { camera.getConfig().viewportHeight };
    const double zoom { camera.getZoom() };
    const auto& camPos { camera.getPosition() };

    const double left { camPos.x - static_cast<double>(screenW) / (2.0 * zoom) };
    const double right { camPos.x + static_cast<double>(screenW) / (2.0 * zoom) };
    const double bottom { camPos.y - static_cast<double>(screenH) / (2.0 * zoom) };
    const double top { camPos.y + static_cast<double>(screenH) / (2.0 * zoom) };

    SDL_SetRenderDrawColor(renderer, gridColor.r, gridColor.g, gridColor.b, gridColor.a);

    // Vertical grid lines every 5 meters
    const double startX { std::floor(left / 5.0) * 5.0 };
    for (double x { startX }; x <= right; x += 5.0) {
        const auto p1 = camera.worldToScreen({ x, bottom });
        const auto p2 = camera.worldToScreen({ x, top });
        SDL_RenderDrawLine(renderer,
                           static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
                           static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)));
    }

    // Horizontal grid lines every 5 meters
    const double startY { std::floor(bottom / 5.0) * 5.0 };
    for (double y { startY }; y <= top; y += 5.0) {
        const auto p1 = camera.worldToScreen({ left, y });
        const auto p2 = camera.worldToScreen({ right, y });
        SDL_RenderDrawLine(renderer,
                           static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
                           static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)));
    }
}

void WireframeRenderer::drawSkids(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const SkidRibbonBuffer& skids,
                                 SDL_Color skidBaseColor) noexcept
{
    const double lifetime { skids.getLifetime() };
    if (lifetime <= 0.0) {
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    for (const auto& seg : skids.getSegments()) {
        const double remaining { std::max(0.0, 1.0 - (seg.age / lifetime)) };
        const auto alpha { static_cast<std::uint8_t>(std::clamp(204.0 * remaining, 0.0, 255.0)) };

        const auto s1 = camera.worldToScreen(seg.from);
        const auto s2 = camera.worldToScreen(seg.to);

        const SDL_Color skidColor { skidBaseColor.r, skidBaseColor.g, skidBaseColor.b, alpha };
        drawLine(renderer,
                 static_cast<int>(std::round(s1.x)), static_cast<int>(std::round(s1.y)),
                 static_cast<int>(std::round(s2.x)), static_cast<int>(std::round(s2.y)),
                 skidColor);
    }
}

void WireframeRenderer::drawTrail(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const TrailParticleBuffer& trail) noexcept
{
    const double lifetime { trail.getLifetime() };
    if (lifetime <= 0.0) {
        return;
    }

    const int screenW { camera.getConfig().viewportWidth };
    const int screenH { camera.getConfig().viewportHeight };

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    for (const auto& dot : trail.getDots()) {
        const auto s = camera.worldToScreen(dot.position);
        if (s.x < -4.0 || s.x > screenW + 4.0 || s.y < -4.0 || s.y > screenH + 4.0) {
            continue;
        }

        const double remaining { std::max(0.0, 1.0 - (dot.age / lifetime)) };
        const auto alpha { static_cast<std::uint8_t>(std::clamp(153.0 * remaining * remaining, 0.0, 255.0)) };
        const int radius { std::max(1, static_cast<int>(std::round(0.8 + 1.4 * remaining))) };

        SDL_SetRenderDrawColor(renderer, 140, 230, 197, alpha); // #8ce6c5
        const int cx { static_cast<int>(std::round(s.x)) };
        const int cy { static_cast<int>(std::round(s.y)) };

        SDL_Rect rect { cx - radius, cy - radius, radius * 2 + 1, radius * 2 + 1 };
        SDL_RenderFillRect(renderer, &rect);
    }
}

void WireframeRenderer::drawBarrier(SDL_Renderer* renderer, const MotionLabCamera& camera) noexcept
{
    const auto bounds = camera.getBounds();
    const auto topLeft = camera.worldToScreen({ bounds.left, bounds.top });
    const auto bottomRight = camera.worldToScreen({ bounds.right, bounds.bottom });

    const SDL_Color barrierColor { 212, 171, 113, 255 }; // #d4ab71
    SDL_SetRenderDrawColor(renderer, barrierColor.r, barrierColor.g, barrierColor.b, barrierColor.a);

    SDL_Rect rect {
        static_cast<int>(std::round(topLeft.x)),
        static_cast<int>(std::round(topLeft.y)),
        static_cast<int>(std::round(bottomRight.x - topLeft.x)),
        static_cast<int>(std::round(bottomRight.y - topLeft.y))
    };
    SDL_RenderDrawRect(renderer, &rect);
}

void WireframeRenderer::drawCar(SDL_Renderer* renderer,
                               const MotionLabCamera& camera,
                               const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                               const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    // 1. Draw car wireframe perimeter & translucent interior fill
    const auto worldCorners = car.computeWorldPoints(body);
    std::array<vehicle_sim::math::Vec2, 4> screenCorners { };
    for (std::size_t i { 0 }; i < 4; ++i) {
        screenCorners[i] = camera.worldToScreen(worldCorners[i]);
    }

    // Translucent body fill
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 140, 230, 197, 25); // #8ce6c515

    // Wireframe perimeter lines (mint teal #8ce6c5)
    const SDL_Color mintColor { 140, 230, 197, 255 };
    for (std::size_t i { 0 }; i < 4; ++i) {
        const auto& p1 = screenCorners[i];
        const auto& p2 = screenCorners[(i + 1) % 4];
        drawLine(renderer,
                 static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
                 static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)),
                 mintColor);
    }

    // Front-to-rear interior cross line at x = 0.70m
    const auto crossA = camera.worldToScreen(body.toWorldPoint({ 0.7, -0.7 }));
    const auto crossB = camera.worldToScreen(body.toWorldPoint({ 0.7, 0.7 }));
    drawLine(renderer,
             static_cast<int>(std::round(crossA.x)), static_cast<int>(std::round(crossA.y)),
             static_cast<int>(std::round(crossB.x)), static_cast<int>(std::round(crossB.y)),
             mintColor);

    // 2. Draw 4 articulated wheels
    const SDL_Color handbrakeColor { 255, 223, 147, 255 }; // #ffdf93
    const bool isHandbrakeEngaged { car.getInput().handbrake || car.getRearGripLoss() > 0.05 };

    for (const auto& tire : car.getTires()) {
        const double steerAngle { tire.steerAngle };
        const vehicle_sim::math::Vec2 half {
            0.35 * std::cos(steerAngle),
            0.35 * std::sin(steerAngle)
        };

        const auto worldA = body.toWorldPoint(tire.position - half);
        const auto worldB = body.toWorldPoint(tire.position + half);

        const auto screenA = camera.worldToScreen(worldA);
        const auto screenB = camera.worldToScreen(worldB);

        const SDL_Color tireColor = (!tire.steered && isHandbrakeEngaged) ? handbrakeColor : mintColor;
        drawLine(renderer,
                 static_cast<int>(std::round(screenA.x)), static_cast<int>(std::round(screenA.y)),
                 static_cast<int>(std::round(screenB.x)), static_cast<int>(std::round(screenB.y)),
                 tireColor);
    }

    // 3. Draw Center of Mass Dot
    const auto centerScreen = camera.worldToScreen(body.getPosition());
    const int cx { static_cast<int>(std::round(centerScreen.x)) };
    const int cy { static_cast<int>(std::round(centerScreen.y)) };
    SDL_SetRenderDrawColor(renderer, mintColor.r, mintColor.g, mintColor.b, mintColor.a);
    SDL_Rect centerRect { cx - 2, cy - 2, 5, 5 };
    SDL_RenderFillRect(renderer, &centerRect);

    // 4. Draw Velocity Vector Arrow (if speed > 0.1 m/s)
    const auto velocity = body.getLinearVelocity();
    if (velocity.length() > 0.1) {
        const auto arrowEndWorld = body.getPosition() + velocity * 0.35;
        const auto arrowEndScreen = camera.worldToScreen(arrowEndWorld);
        const SDL_Color arrowColor { 101, 141, 160, 255 }; // #658da0
        drawArrow(renderer, centerScreen, arrowEndScreen, arrowColor);
    }
}

void WireframeRenderer::renderScene(SDL_Renderer* renderer,
                                   const MotionLabCamera& camera,
                                   const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                                   const vehicle_sim::physics::RigidBody2D& body,
                                   const SkidRibbonBuffer& skids,
                                   const TrailParticleBuffer& trail,
                                   bool barrierActive) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const auto theme = getTerrainTheme(car.getSurfaceMaterial().type);
    SDL_SetRenderDrawColor(renderer, theme.background.r, theme.background.g, theme.background.b, theme.background.a);
    SDL_RenderClear(renderer);

    drawGrid(renderer, camera, theme.grid);
    drawTrail(renderer, camera, trail);
    drawSkids(renderer, camera, skids, theme.skid);

    if (barrierActive) {
        drawBarrier(renderer, camera);
    }

    drawCar(renderer, camera, car, body);
}

} // namespace motion_lab
