#include "apps/MotionLab/WireframeRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_lab {

void WireframeRenderer::drawLine(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, SDL_Color color) noexcept
{
    if (renderer == nullptr) {
        return;
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

void WireframeRenderer::drawArrow(SDL_Renderer* renderer, const vehicle_sim::math::Vec2& screenStart,
    const vehicle_sim::math::Vec2& screenEnd, SDL_Color color) noexcept
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

    void drawLineLocal(SDL_Renderer* renderer, const MotionLabCamera& camera,
        const vehicle_sim::physics::RigidBody2D& body, const vehicle_sim::math::Vec2& localA,
        const vehicle_sim::math::Vec2& localB, SDL_Color color) noexcept
    {
        const auto worldA = body.toWorldPoint(localA);
        const auto worldB = body.toWorldPoint(localB);
        const auto screenA = camera.worldToScreen(worldA);
        const auto screenB = camera.worldToScreen(worldB);
        WireframeRenderer::drawLine(renderer, static_cast<int>(std::round(screenA.x)),
            static_cast<int>(std::round(screenA.y)), static_cast<int>(std::round(screenB.x)),
            static_cast<int>(std::round(screenB.y)), color);
    }

    void drawPolyLocal(SDL_Renderer* renderer, const MotionLabCamera& camera,
        const vehicle_sim::physics::RigidBody2D& body, const vehicle_sim::math::Vec2* points, std::size_t count,
        bool closed, SDL_Color color) noexcept
    {
        if (count < 2) {
            return;
        }
        for (std::size_t i { 0 }; i < count - 1; ++i) {
            drawLineLocal(renderer, camera, body, points[i], points[i + 1], color);
        }
        if (closed) {
            drawLineLocal(renderer, camera, body, points[count - 1], points[0], color);
        }
    }

    void drawComAndVelocity(SDL_Renderer* renderer, const MotionLabCamera& camera,
        const vehicle_sim::physics::RigidBody2D& body, SDL_Color dotColor) noexcept
    {
        const auto centerScreen = camera.worldToScreen(body.getPosition());
        const int cx { static_cast<int>(std::round(centerScreen.x)) };
        const int cy { static_cast<int>(std::round(centerScreen.y)) };
        SDL_SetRenderDrawColor(renderer, dotColor.r, dotColor.g, dotColor.b, dotColor.a);
        SDL_Rect centerRect { cx - 2, cy - 2, 5, 5 };
        SDL_RenderFillRect(renderer, &centerRect);

        const auto velocity = body.getLinearVelocity();
        if (velocity.length() > 0.1) {
            const auto arrowEndWorld = body.getPosition() + velocity * 0.35;
            const auto arrowEndScreen = camera.worldToScreen(arrowEndWorld);
            const SDL_Color arrowColor { 101, 141, 160, 255 };
            WireframeRenderer::drawArrow(renderer, centerScreen, arrowEndScreen, arrowColor);
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

    const double startX { std::floor(left / 5.0) * 5.0 };
    for (double x { startX }; x <= right; x += 5.0) {
        const auto p1 = camera.worldToScreen({ x, bottom });
        const auto p2 = camera.worldToScreen({ x, top });
        SDL_RenderDrawLine(renderer, static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
            static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)));
    }

    const double startY { std::floor(bottom / 5.0) * 5.0 };
    for (double y { startY }; y <= top; y += 5.0) {
        const auto p1 = camera.worldToScreen({ left, y });
        const auto p2 = camera.worldToScreen({ right, y });
        SDL_RenderDrawLine(renderer, static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
            static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)));
    }
}

void WireframeRenderer::drawSkids(SDL_Renderer* renderer, const MotionLabCamera& camera, const SkidRibbonBuffer& skids,
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
        drawLine(renderer, static_cast<int>(std::round(s1.x)), static_cast<int>(std::round(s1.y)),
            static_cast<int>(std::round(s2.x)), static_cast<int>(std::round(s2.y)), skidColor);
    }
}

void WireframeRenderer::drawTrail(
    SDL_Renderer* renderer, const MotionLabCamera& camera, const TrailParticleBuffer& trail) noexcept
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

        SDL_SetRenderDrawColor(renderer, 140, 230, 197, alpha);
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

    const SDL_Color barrierColor { 212, 171, 113, 255 };
    SDL_SetRenderDrawColor(renderer, barrierColor.r, barrierColor.g, barrierColor.b, barrierColor.a);

    SDL_Rect rect { static_cast<int>(std::round(topLeft.x)), static_cast<int>(std::round(topLeft.y)),
        static_cast<int>(std::round(bottomRight.x - topLeft.x)),
        static_cast<int>(std::round(bottomRight.y - topLeft.y)) };
    SDL_RenderDrawRect(renderer, &rect);
}

void WireframeRenderer::drawClassicCar(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::MotionLabCarDynamics& car, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const auto worldCorners = car.computeWorldPoints(body);
    std::array<vehicle_sim::math::Vec2, 4> screenCorners {};
    for (std::size_t i { 0 }; i < 4; ++i) {
        screenCorners[i] = camera.worldToScreen(worldCorners[i]);
    }

    const SDL_Color mintColor { 140, 230, 197, 255 };
    for (std::size_t i { 0 }; i < 4; ++i) {
        const auto& p1 = screenCorners[i];
        const auto& p2 = screenCorners[(i + 1) % 4];
        drawLine(renderer, static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
            static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)), mintColor);
    }

    const auto crossA = camera.worldToScreen(body.toWorldPoint({ 0.7, -0.7 }));
    const auto crossB = camera.worldToScreen(body.toWorldPoint({ 0.7, 0.7 }));
    drawLine(renderer, static_cast<int>(std::round(crossA.x)), static_cast<int>(std::round(crossA.y)),
        static_cast<int>(std::round(crossB.x)), static_cast<int>(std::round(crossB.y)), mintColor);

    const SDL_Color handbrakeColor { 255, 223, 147, 255 };
    const bool isHandbrakeEngaged { car.getInput().handbrake || car.getRearGripLoss() > 0.05 };

    for (const auto& tire : car.getTires()) {
        const double steerAngle { tire.steerAngle };
        const vehicle_sim::math::Vec2 half { 0.35 * std::cos(steerAngle), 0.35 * std::sin(steerAngle) };

        const auto worldA = body.toWorldPoint(tire.position - half);
        const auto worldB = body.toWorldPoint(tire.position + half);

        const auto screenA = camera.worldToScreen(worldA);
        const auto screenB = camera.worldToScreen(worldB);

        const SDL_Color tireColor = (!tire.steered && isHandbrakeEngaged) ? handbrakeColor : mintColor;
        drawLine(renderer, static_cast<int>(std::round(screenA.x)), static_cast<int>(std::round(screenA.y)),
            static_cast<int>(std::round(screenB.x)), static_cast<int>(std::round(screenB.y)), tireColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawRaceCar(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::RaceCarDynamics& raceCar, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color drsColor { 255, 223, 147, 255 };

    // 1. Tapered aerodynamic chassis & cockpit
    const vehicle_sim::math::Vec2 noseBody[] { { 2.2, 0.0 }, { 1.1, -0.32 }, { 0.3, -0.42 }, { -0.5, -0.42 },
        { -1.4, -0.32 }, { -1.8, -0.22 }, { -1.8, 0.22 }, { -1.4, 0.32 }, { -0.5, 0.42 }, { 0.3, 0.42 },
        { 1.1, 0.32 } };
    drawPolyLocal(renderer, camera, body, noseBody, sizeof(noseBody) / sizeof(noseBody[0]), true, mintColor);

    // Sidepods
    const vehicle_sim::math::Vec2 sidepodL[] { { 0.3, 0.42 }, { 0.3, 0.72 }, { -1.1, 0.70 }, { -1.2, 0.38 } };
    drawPolyLocal(renderer, camera, body, sidepodL, 4, false, mintColor);
    const vehicle_sim::math::Vec2 sidepodR[] { { 0.3, -0.42 }, { 0.3, -0.72 }, { -1.1, -0.70 }, { -1.2, -0.38 } };
    drawPolyLocal(renderer, camera, body, sidepodR, 4, false, mintColor);

    // Cockpit halo
    const vehicle_sim::math::Vec2 halo[] { { 0.25, -0.20 }, { 0.25, 0.20 }, { -0.35, 0.15 }, { -0.35, -0.15 } };
    drawPolyLocal(renderer, camera, body, halo, 4, true, mintColor);

    // Front Wing
    drawLineLocal(renderer, camera, body, { 1.9, -0.85 }, { 1.9, 0.85 }, mintColor);
    drawLineLocal(renderer, camera, body, { 1.7, 0.85 }, { 2.1, 0.85 }, mintColor);
    drawLineLocal(renderer, camera, body, { 1.7, -0.85 }, { 2.1, -0.85 }, mintColor);

    // Rear Wing & DRS flap
    const bool isDrsOpen { raceCar.getDrsAngleDeg() > 1.0 };
    const SDL_Color rearWingColor { isDrsOpen ? drsColor : mintColor };
    drawLineLocal(renderer, camera, body, { -1.8, -0.85 }, { -1.8, 0.85 }, rearWingColor);
    drawLineLocal(renderer, camera, body, { -1.6, 0.85 }, { -2.0, 0.85 }, rearWingColor);
    drawLineLocal(renderer, camera, body, { -1.6, -0.85 }, { -2.0, -0.85 }, rearWingColor);

    // 2. 4 articulated open wheels
    const auto& wheels = raceCar.getWheelStates();
    for (std::size_t i { 0 }; i < 4; ++i) {
        const double steerAngle { wheels[i].steerAngle };
        const vehicle_sim::math::Vec2 half { 0.38 * std::cos(steerAngle), 0.38 * std::sin(steerAngle) };
        const auto localPos = wheels[i].localOffset;
        const bool skidding { std::abs(wheels[i].slipAngle) > 0.15 || std::abs(wheels[i].slipRatio) > 0.15 };
        const SDL_Color wheelColor { skidding ? drsColor : mintColor };
        drawLineLocal(renderer, camera, body, localPos - half, localPos + half, wheelColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawRocket(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::RocketDynamics& rocket, const vehicle_sim::physics::RigidBody2D& body,
    double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color flameColor { 255, 140, 50, 255 };
    const SDL_Color flameCore { 255, 235, 120, 255 };

    // 1. Fuselage cylinder & pointed ogive fairing
    const vehicle_sim::math::Vec2 rocketHull[] { { -2.2, -0.75 }, { 1.5, -0.75 }, { 2.8, 0.0 }, { 1.5, 0.75 },
        { -2.2, 0.75 } };
    drawPolyLocal(renderer, camera, body, rocketHull, 5, true, mintColor);

    // Interstage divider line
    drawLineLocal(renderer, camera, body, { -0.3, -0.75 }, { -0.3, 0.75 }, mintColor);

    // Delta stabilizing fins
    const vehicle_sim::math::Vec2 finTop[] { { -1.0, 0.75 }, { -2.4, 1.45 }, { -2.2, 0.75 } };
    drawPolyLocal(renderer, camera, body, finTop, 3, false, mintColor);
    const vehicle_sim::math::Vec2 finBottom[] { { -1.0, -0.75 }, { -2.4, -1.45 }, { -2.2, -0.75 } };
    drawPolyLocal(renderer, camera, body, finBottom, 3, false, mintColor);

    // 2. Gimbaled thrust nozzle bell
    const double gimbal { rocket.getGimbalAngle() };
    const double cosG { std::cos(gimbal) };
    const double sinG { std::sin(gimbal) };
    const vehicle_sim::math::Vec2 bellMount { -2.2, 0.0 };

    const vehicle_sim::math::Vec2 bellLeft { bellMount.x - 0.65 * cosG - (-0.35) * sinG,
        bellMount.y - 0.65 * sinG + (-0.35) * cosG };
    const vehicle_sim::math::Vec2 bellRight { bellMount.x - 0.65 * cosG - 0.35 * sinG,
        bellMount.y - 0.65 * sinG + 0.35 * cosG };

    drawLineLocal(renderer, camera, body, bellMount, bellLeft, mintColor);
    drawLineLocal(renderer, camera, body, bellMount, bellRight, mintColor);
    drawLineLocal(renderer, camera, body, bellLeft, bellRight, mintColor);

    // 3. Dynamic thrust exhaust flame
    if (rocket.getThrustForce() > 10.0) {
        const double flicker { 0.4 * std::sin(animTimer * 50.0) + 0.2 * std::cos(animTimer * 30.0) };
        const double flameLen { 1.5 + flicker };
        const vehicle_sim::math::Vec2 flameTip { bellMount.x - (0.65 + flameLen) * cosG,
            bellMount.y - (0.65 + flameLen) * sinG };
        drawLineLocal(renderer, camera, body, bellLeft, flameTip, flameColor);
        drawLineLocal(renderer, camera, body, bellRight, flameTip, flameColor);
        drawLineLocal(renderer, camera, body, bellMount, flameTip, flameCore);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawHelicopter(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::HelicopterDynamics& helicopter, const vehicle_sim::physics::RigidBody2D& body,
    double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };

    // 1. Fuselage pod & cabin
    const vehicle_sim::math::Vec2 cabin[] { { 1.8, 0.0 }, { 1.1, 0.65 }, { -0.8, 0.55 }, { -0.8, -0.55 },
        { 1.1, -0.65 } };
    drawPolyLocal(renderer, camera, body, cabin, 5, true, mintColor);

    // Windshield glass line
    drawLineLocal(renderer, camera, body, { 0.6, -0.5 }, { 0.6, 0.5 }, mintColor);

    // Tail boom & stabilizer
    drawLineLocal(renderer, camera, body, { -0.8, 0.0 }, { -3.2, 0.0 }, mintColor);
    drawLineLocal(renderer, camera, body, { -3.0, 0.0 }, { -3.3, 0.65 }, mintColor);
    drawLineLocal(renderer, camera, body, { -2.4, -0.45 }, { -2.4, 0.45 }, mintColor);

    // Tail rotor
    const double tailAngle { animTimer * 60.0 };
    const vehicle_sim::math::Vec2 tailHub { -3.3, 0.65 };
    const vehicle_sim::math::Vec2 tailTip1 { tailHub.x + 0.3 * std::cos(tailAngle),
        tailHub.y + 0.3 * std::sin(tailAngle) };
    const vehicle_sim::math::Vec2 tailTip2 { tailHub.x - 0.3 * std::cos(tailAngle),
        tailHub.y - 0.3 * std::sin(tailAngle) };
    drawLineLocal(renderer, camera, body, tailTip1, tailTip2, mintColor);

    // Landing skids
    drawLineLocal(renderer, camera, body, { -1.1, 0.75 }, { 1.1, 0.75 }, mintColor);
    drawLineLocal(renderer, camera, body, { -0.5, 0.55 }, { -0.5, 0.75 }, mintColor);
    drawLineLocal(renderer, camera, body, { 0.5, 0.55 }, { 0.5, 0.75 }, mintColor);

    drawLineLocal(renderer, camera, body, { -1.1, -0.75 }, { 1.1, -0.75 }, mintColor);
    drawLineLocal(renderer, camera, body, { -0.5, -0.55 }, { -0.5, -0.75 }, mintColor);
    drawLineLocal(renderer, camera, body, { 0.5, -0.55 }, { 0.5, -0.75 }, mintColor);

    // 2. Spinning Main Rotor Blades (4 blades)
    const double rotorSpeed { helicopter.getRotorSpeed() };
    const double rotorAngle { animTimer * (rotorSpeed * 0.10472 + 5.0) };
    constexpr double kRotorRadius { 4.5 };

    for (int i { 0 }; i < 4; ++i) {
        const double a { rotorAngle + static_cast<double>(i) * (3.14159265358979323846 / 2.0) };
        const vehicle_sim::math::Vec2 bladeTip { kRotorRadius * std::cos(a), kRotorRadius * std::sin(a) };
        drawLineLocal(renderer, camera, body, { 0.0, 0.0 }, bladeTip, mintColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawHovercraft(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::HovercraftDynamics& hovercraft, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color rudderColor { 255, 223, 147, 255 };

    // 1. Perimeter cushion skirt
    const vehicle_sim::math::Vec2 skirt[] { { 2.0, 0.5 }, { 1.5, 1.05 }, { -1.5, 1.05 }, { -2.0, 0.6 }, { -2.0, -0.6 },
        { -1.5, -1.05 }, { 1.5, -1.05 }, { 2.0, -0.5 } };
    drawPolyLocal(renderer, camera, body, skirt, 8, true, mintColor);

    // Inner deck boundary
    const vehicle_sim::math::Vec2 innerDeck[] { { 1.6, 0.35 }, { 1.2, 0.8 }, { -1.2, 0.8 }, { -1.6, 0.45 },
        { -1.6, -0.45 }, { -1.2, -0.8 }, { 1.2, -0.8 }, { 1.6, -0.35 } };
    drawPolyLocal(renderer, camera, body, innerDeck, 8, true, mintColor);

    // Wheelhouse cabin
    const vehicle_sim::math::Vec2 cabin[] { { 0.8, -0.45 }, { 0.8, 0.45 }, { -0.5, 0.45 }, { -0.5, -0.45 } };
    drawPolyLocal(renderer, camera, body, cabin, 4, true, mintColor);

    // 2. Dual rear fan ducts and steerable rudders
    const double rudder { hovercraft.getRudderAngle() };
    const double cosR { std::cos(rudder) };
    const double sinR { std::sin(rudder) };

    const vehicle_sim::math::Vec2 ductL { -1.5, 0.55 };
    const vehicle_sim::math::Vec2 ductR { -1.5, -0.55 };

    // Fan duct rectangles
    drawLineLocal(
        renderer, camera, body, { ductL.x - 0.3, ductL.y - 0.3 }, { ductL.x + 0.3, ductL.y - 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductL.x + 0.3, ductL.y - 0.3 }, { ductL.x + 0.3, ductL.y + 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductL.x + 0.3, ductL.y + 0.3 }, { ductL.x - 0.3, ductL.y + 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductL.x - 0.3, ductL.y + 0.3 }, { ductL.x - 0.3, ductL.y - 0.3 }, mintColor);

    drawLineLocal(
        renderer, camera, body, { ductR.x - 0.3, ductR.y - 0.3 }, { ductR.x + 0.3, ductR.y - 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductR.x + 0.3, ductR.y - 0.3 }, { ductR.x + 0.3, ductR.y + 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductR.x + 0.3, ductR.y + 0.3 }, { ductR.x - 0.3, ductR.y + 0.3 }, mintColor);
    drawLineLocal(
        renderer, camera, body, { ductR.x - 0.3, ductR.y + 0.3 }, { ductR.x - 0.3, ductR.y - 0.3 }, mintColor);

    // Steerable rudder vanes
    const vehicle_sim::math::Vec2 rudderHalf { 0.25 * cosR, 0.25 * sinR };
    drawLineLocal(renderer, camera, body, ductL - rudderHalf, ductL + rudderHalf, rudderColor);
    drawLineLocal(renderer, camera, body, ductR - rudderHalf, ductR + rudderHalf, rudderColor);

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawTrackedTank(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::TrackedTankDynamics& tank, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color recoilColor { 255, 223, 147, 255 };

    // 1. Armored chassis hull
    const vehicle_sim::math::Vec2 hull[] { { 1.6, -0.85 }, { 2.1, 0.0 }, { 1.6, 0.85 }, { -1.8, 0.85 },
        { -1.8, -0.85 } };
    drawPolyLocal(renderer, camera, body, hull, 5, true, mintColor);

    // 2. Left and right continuous track belts with cleats
    const vehicle_sim::math::Vec2 trackL[] { { -2.1, 0.90 }, { 2.1, 0.90 }, { 2.1, 1.25 }, { -2.1, 1.25 } };
    drawPolyLocal(renderer, camera, body, trackL, 4, true, mintColor);
    for (double x { -1.8 }; x <= 1.8; x += 0.45) {
        drawLineLocal(renderer, camera, body, { x, 0.90 }, { x, 1.25 }, mintColor);
    }

    const vehicle_sim::math::Vec2 trackR[] { { -2.1, -1.25 }, { 2.1, -1.25 }, { 2.1, -0.90 }, { -2.1, -0.90 } };
    drawPolyLocal(renderer, camera, body, trackR, 4, true, mintColor);
    for (double x { -1.8 }; x <= 1.8; x += 0.45) {
        drawLineLocal(renderer, camera, body, { x, -1.25 }, { x, -0.90 }, mintColor);
    }

    // 3. Turret cupola & long cannon barrel
    const vehicle_sim::math::Vec2 turret[] { { 0.7, -0.45 }, { 0.7, 0.45 }, { -0.5, 0.55 }, { -0.6, -0.55 } };
    drawPolyLocal(renderer, camera, body, turret, 4, true, mintColor);

    const bool recoiling { tank.getTelemetry().isRecoiling };
    const double barrelTipX { recoiling ? 2.5 : 2.8 };

    drawLineLocal(renderer, camera, body, { 0.7, -0.07 }, { barrelTipX, -0.07 }, mintColor);
    drawLineLocal(renderer, camera, body, { 0.7, 0.07 }, { barrelTipX, 0.07 }, mintColor);
    drawLineLocal(renderer, camera, body, { barrelTipX, -0.14 }, { barrelTipX, 0.14 }, mintColor);

    if (recoiling) {
        drawLineLocal(renderer, camera, body, { barrelTipX, 0.0 }, { barrelTipX + 0.5, 0.2 }, recoilColor);
        drawLineLocal(renderer, camera, body, { barrelTipX, 0.0 }, { barrelTipX + 0.5, -0.2 }, recoilColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawAntiGravityRacer(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::AntiGravityRacerDynamics& antiGravity, const vehicle_sim::physics::RigidBody2D& body,
    double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color cyanColor { 101, 141, 160, 255 };
    const SDL_Color airbrakeColor { 255, 223, 147, 255 };

    // 1. Tapered aerodynamic fuselage & winglets
    const vehicle_sim::math::Vec2 hull[] { { 2.4, 0.0 }, { 1.2, 0.55 }, { -0.2, 0.95 }, { -1.8, 1.20 }, { -2.2, 0.80 },
        { -2.0, 0.35 }, { -2.0, -0.35 }, { -2.2, -0.80 }, { -1.8, -1.20 }, { -0.2, -0.95 }, { 1.2, -0.55 } };
    drawPolyLocal(renderer, camera, body, hull, 11, true, mintColor);

    // 2. Cockpit canopy
    const vehicle_sim::math::Vec2 canopy[] { { 0.8, 0.0 }, { 0.2, 0.25 }, { -0.6, 0.20 }, { -0.6, -0.20 },
        { 0.2, -0.25 } };
    drawPolyLocal(renderer, camera, body, canopy, 5, true, mintColor);

    // 3. Maglev repulsor field coils (pulsing)
    const double pulsePhase { std::sin(animTimer * 12.0) * 0.05 };
    const vehicle_sim::math::Vec2 coilFL[] { { 0.7 + pulsePhase, 0.35 }, { 0.7 + pulsePhase, 0.65 },
        { 0.2 + pulsePhase, 0.65 }, { 0.2 + pulsePhase, 0.35 } };
    drawPolyLocal(renderer, camera, body, coilFL, 4, true, cyanColor);

    const vehicle_sim::math::Vec2 coilFR[] { { 0.7 + pulsePhase, -0.35 }, { 0.7 + pulsePhase, -0.65 },
        { 0.2 + pulsePhase, -0.65 }, { 0.2 + pulsePhase, -0.35 } };
    drawPolyLocal(renderer, camera, body, coilFR, 4, true, cyanColor);

    // 4. Deployable differential airbrakes
    const auto& telem = antiGravity.getTelemetry();
    const bool leftActive { telem.airbrakeDrag > 10.0 && telem.netYawTorque > 100.0 };
    const bool rightActive { telem.airbrakeDrag > 10.0 && telem.netYawTorque < -100.0 };

    const SDL_Color leftColor = leftActive ? airbrakeColor : mintColor;
    const SDL_Color rightColor = rightActive ? airbrakeColor : mintColor;

    drawLineLocal(
        renderer, camera, body, { -0.8, 0.90 }, { leftActive ? -0.5 : -0.8, leftActive ? 1.45 : 1.15 }, leftColor);
    drawLineLocal(renderer, camera, body, { -0.8, -0.90 }, { rightActive ? -0.5 : -0.8, rightActive ? -1.45 : -1.15 },
        rightColor);

    // 5. Twin ion thrusters & animated exhaust plume
    drawLineLocal(renderer, camera, body, { -2.0, 0.30 }, { -2.2, 0.30 }, mintColor);
    drawLineLocal(renderer, camera, body, { -2.0, -0.30 }, { -2.2, -0.30 }, mintColor);

    if (telem.thrustForce > 500.0) {
        const double plumeLen { 0.4 + (telem.isBoostActive ? 0.8 : 0.3) * (0.8 + 0.2 * std::sin(animTimer * 40.0)) };
        const SDL_Color plumeCol { 165, 216, 255, 255 };
        drawLineLocal(renderer, camera, body, { -2.2, 0.30 }, { -2.2 - plumeLen, 0.30 }, plumeCol);
        drawLineLocal(renderer, camera, body, { -2.2, -0.30 }, { -2.2 - plumeLen, -0.30 }, plumeCol);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawDriftCar(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::DriftCarDynamics& driftCar, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color amberColor { 255, 223, 147, 255 };
    const SDL_Color hotTireColor { 242, 138, 114, 255 };

    // 1. Widebody sports coupe silhouette
    const vehicle_sim::math::Vec2 coupe[] { { 2.1, 0.75 }, { 2.2, 0.0 }, { 2.1, -0.75 }, { 1.4, -0.85 },
        { -1.3, -0.85 }, { -1.9, -0.80 }, { -2.1, -0.70 }, { -2.1, 0.70 }, { -1.9, 0.80 }, { -1.3, 0.85 },
        { 1.4, 0.85 } };
    drawPolyLocal(renderer, camera, body, coupe, 11, true, mintColor);

    // 2. Windshield, roof, and rear window
    const vehicle_sim::math::Vec2 greenhouse[] { { 0.7, 0.65 }, { -0.3, 0.65 }, { -1.1, 0.60 }, { -1.1, -0.60 },
        { -0.3, -0.65 }, { 0.7, -0.65 } };
    drawPolyLocal(renderer, camera, body, greenhouse, 6, true, mintColor);
    drawLineLocal(renderer, camera, body, { -0.3, 0.65 }, { -0.3, -0.65 }, mintColor);

    // 3. High-mount drift wing
    drawLineLocal(renderer, camera, body, { -2.0, -0.85 }, { -2.0, 0.85 }, amberColor);
    drawLineLocal(renderer, camera, body, { -1.9, 0.85 }, { -2.1, 0.85 }, amberColor);
    drawLineLocal(renderer, camera, body, { -1.9, -0.85 }, { -2.1, -0.85 }, amberColor);

    // 4. Wheels with extreme steering lock visualization
    const auto& wheels = driftCar.getWheelStates();
    const auto temps = driftCar.getTireTemperatures();

    for (std::size_t i { 0 }; i < 4; ++i) {
        const double steerAngle { wheels[i].steerAngle };
        const vehicle_sim::math::Vec2 half { 0.36 * std::cos(steerAngle), 0.36 * std::sin(steerAngle) };
        const auto localPos = wheels[i].localOffset;

        SDL_Color wheelCol = mintColor;
        if (temps[i] > 85.0 || std::abs(wheels[i].slipAngle) > 0.20) {
            wheelCol = temps[i] > 100.0 ? hotTireColor : amberColor;
        }

        drawLineLocal(renderer, camera, body, localPos - half, localPos + half, wheelCol);
        const vehicle_sim::math::Vec2 rimPerp { -0.10 * std::sin(steerAngle), 0.10 * std::cos(steerAngle) };
        drawLineLocal(renderer, camera, body, localPos - half - rimPerp, localPos - half + rimPerp, wheelCol);
        drawLineLocal(renderer, camera, body, localPos + half - rimPerp, localPos + half + rimPerp, wheelCol);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawMonsterTruck(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::MonsterCarDynamics& monsterCar, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color amberColor { 255, 223, 147, 255 };

    // 1. Truck chassis body and cab
    const vehicle_sim::math::Vec2 cab[] { { 2.0, 0.70 }, { 2.0, -0.70 }, { 0.6, -0.75 }, { -1.8, -0.75 },
        { -2.0, -0.65 }, { -2.0, 0.65 }, { -1.8, 0.75 }, { 0.6, 0.75 } };
    drawPolyLocal(renderer, camera, body, cab, 8, true, mintColor);

    // Windshield & roof
    const vehicle_sim::math::Vec2 roof[] { { 0.6, 0.60 }, { 0.6, -0.60 }, { -0.6, -0.60 }, { -0.6, 0.60 } };
    drawPolyLocal(renderer, camera, body, roof, 4, true, mintColor);

    // Bed rollbar
    drawLineLocal(renderer, camera, body, { -0.8, -0.65 }, { -1.6, 0.65 }, amberColor);
    drawLineLocal(renderer, camera, body, { -0.8, 0.65 }, { -1.6, -0.65 }, amberColor);

    // Front push bumper
    drawLineLocal(renderer, camera, body, { 2.3, -0.60 }, { 2.3, 0.60 }, amberColor);
    drawLineLocal(renderer, camera, body, { 2.0, -0.60 }, { 2.3, -0.60 }, amberColor);
    drawLineLocal(renderer, camera, body, { 2.0, 0.60 }, { 2.3, 0.60 }, amberColor);

    // 2. 4 massive oversized wheels with knobby chevron treads
    const double steerAngle { monsterCar.getInput().steer * 0.50 };
    const vehicle_sim::math::Vec2 wheelOffsets[] { { 1.25, 1.15 }, { 1.25, -1.15 }, { -1.25, 1.15 }, { -1.25, -1.15 } };

    for (std::size_t i { 0 }; i < 4; ++i) {
        const double steer = (i < 2) ? steerAngle : 0.0;
        const vehicle_sim::math::Vec2 forward { 0.50 * std::cos(steer), 0.50 * std::sin(steer) };
        const vehicle_sim::math::Vec2 lateral { -0.22 * std::sin(steer), 0.22 * std::cos(steer) };

        const auto center = wheelOffsets[i];
        const vehicle_sim::math::Vec2 tireBox[] { center + forward + lateral, center + forward - lateral,
            center - forward - lateral, center - forward + lateral };
        drawPolyLocal(renderer, camera, body, tireBox, 4, true, mintColor);

        // Suspension linkage from chassis to wheel
        drawLineLocal(renderer, camera, body, { center.x * 0.6, center.y * 0.4 }, center, amberColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawArticulatedTruck(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::ArticulatedTruckDynamics& truck,
    const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color amberColor { 255, 223, 147, 255 };
    const SDL_Color alertRed { 242, 138, 114, 255 };

    const auto& telem = truck.getTelemetry();

    // 1. Heavy tractor cab
    const vehicle_sim::math::Vec2 tractor[] { { 2.5, 1.15 }, { 2.5, -1.15 }, { 0.8, -1.20 }, { -2.4, -1.20 },
        { -2.4, 1.20 }, { 0.8, 1.20 } };
    drawPolyLocal(renderer, camera, body, tractor, 6, true, mintColor);

    // Tractor cab windshield
    drawLineLocal(renderer, camera, body, { 1.5, 1.0 }, { 1.5, -1.0 }, mintColor);
    drawLineLocal(renderer, camera, body, { 0.8, 1.0 }, { 0.8, -1.0 }, mintColor);

    // Tractor front steer wheels
    const double steer { telem.steerAngle };
    const vehicle_sim::math::Vec2 fwdSteer { 0.45 * std::cos(steer), 0.45 * std::sin(steer) };
    const vehicle_sim::math::Vec2 steerFL { 1.8, 1.25 };
    const vehicle_sim::math::Vec2 steerFR { 1.8, -1.25 };
    drawLineLocal(renderer, camera, body, steerFL - fwdSteer, steerFL + fwdSteer, mintColor);
    drawLineLocal(renderer, camera, body, steerFR - fwdSteer, steerFR + fwdSteer, mintColor);

    // Tractor dual rear drive axles
    const vehicle_sim::math::Vec2 tandemDx { 0.45, 0.0 };
    const vehicle_sim::math::Vec2 r1L { -1.3, 1.25 };
    const vehicle_sim::math::Vec2 r1R { -1.3, -1.25 };
    const vehicle_sim::math::Vec2 r2L { -2.0, 1.25 };
    const vehicle_sim::math::Vec2 r2R { -2.0, -1.25 };
    drawLineLocal(renderer, camera, body, r1L - tandemDx, r1L + tandemDx, mintColor);
    drawLineLocal(renderer, camera, body, r1R - tandemDx, r1R + tandemDx, mintColor);
    drawLineLocal(renderer, camera, body, r2L - tandemDx, r2L + tandemDx, mintColor);
    drawLineLocal(renderer, camera, body, r2R - tandemDx, r2R + tandemDx, mintColor);

    // 2. Fifth-wheel kingpin plate
    const vehicle_sim::math::Vec2 kingpinLocal { -1.8, 0.0 };
    const auto worldPin = body.toWorldPoint(kingpinLocal);
    const SDL_Color hitchColor
        = telem.isJackknifed ? alertRed : (std::abs(telem.hitchAngle) > 0.6 ? amberColor : mintColor);

    // 3. 53-ft Articulated Semi-Trailer (rendered in world space using trailer heading)
    const double trailerHeading { telem.trailerHeading };
    const vehicle_sim::math::Vec2 uTrailer { std::cos(trailerHeading), std::sin(trailerHeading) };
    const vehicle_sim::math::Vec2 vTrailer { -std::sin(trailerHeading), std::cos(trailerHeading) };

    const double trailerHalfW { 1.25 };
    const double trailerFrontDist { 1.2 };
    const double trailerRearDist { 10.2 };

    const vehicle_sim::math::Vec2 tCornersWorld[] { worldPin + uTrailer * trailerFrontDist + vTrailer * trailerHalfW,
        worldPin + uTrailer * trailerFrontDist - vTrailer * trailerHalfW,
        worldPin - uTrailer * trailerRearDist - vTrailer * trailerHalfW,
        worldPin - uTrailer * trailerRearDist + vTrailer * trailerHalfW };

    for (std::size_t i { 0 }; i < 4; ++i) {
        const auto p1 = camera.worldToScreen(tCornersWorld[i]);
        const auto p2 = camera.worldToScreen(tCornersWorld[(i + 1) % 4]);
        drawLine(renderer, static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
            static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)), hitchColor);
    }

    // Trailer rear tandem wheels
    const double bogieDist1 { 8.8 };
    const double bogieDist2 { 9.6 };
    const vehicle_sim::math::Vec2 bogie1L = worldPin - uTrailer * bogieDist1 + vTrailer * (trailerHalfW + 0.05);
    const vehicle_sim::math::Vec2 bogie1R = worldPin - uTrailer * bogieDist1 - vTrailer * (trailerHalfW + 0.05);
    const vehicle_sim::math::Vec2 bogie2L = worldPin - uTrailer * bogieDist2 + vTrailer * (trailerHalfW + 0.05);
    const vehicle_sim::math::Vec2 bogie2R = worldPin - uTrailer * bogieDist2 - vTrailer * (trailerHalfW + 0.05);

    const auto sB1L_A = camera.worldToScreen(bogie1L - uTrailer * 0.40);
    const auto sB1L_B = camera.worldToScreen(bogie1L + uTrailer * 0.40);
    drawLine(renderer, static_cast<int>(std::round(sB1L_A.x)), static_cast<int>(std::round(sB1L_A.y)),
        static_cast<int>(std::round(sB1L_B.x)), static_cast<int>(std::round(sB1L_B.y)), hitchColor);

    const auto sB1R_A = camera.worldToScreen(bogie1R - uTrailer * 0.40);
    const auto sB1R_B = camera.worldToScreen(bogie1R + uTrailer * 0.40);
    drawLine(renderer, static_cast<int>(std::round(sB1R_A.x)), static_cast<int>(std::round(sB1R_A.y)),
        static_cast<int>(std::round(sB1R_B.x)), static_cast<int>(std::round(sB1R_B.y)), hitchColor);

    const auto sB2L_A = camera.worldToScreen(bogie2L - uTrailer * 0.40);
    const auto sB2L_B = camera.worldToScreen(bogie2L + uTrailer * 0.40);
    drawLine(renderer, static_cast<int>(std::round(sB2L_A.x)), static_cast<int>(std::round(sB2L_A.y)),
        static_cast<int>(std::round(sB2L_B.x)), static_cast<int>(std::round(sB2L_B.y)), hitchColor);

    const auto sB2R_A = camera.worldToScreen(bogie2R - uTrailer * 0.40);
    const auto sB2R_B = camera.worldToScreen(bogie2R + uTrailer * 0.40);
    drawLine(renderer, static_cast<int>(std::round(sB2R_A.x)), static_cast<int>(std::round(sB2R_A.y)),
        static_cast<int>(std::round(sB2R_A.y)), static_cast<int>(std::round(sB2R_B.y)), hitchColor);

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawForklift(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::ForkliftDynamics& forklift, const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color amberColor { 255, 223, 147, 255 };
    const SDL_Color alertRed { 242, 138, 114, 255 };

    const bool tipped { forklift.isTippedOver() || forklift.getRolloverIndex() > 0.85 };
    const SDL_Color chassisCol = tipped ? alertRed : mintColor;

    // 1. Counterweight chassis hull
    const vehicle_sim::math::Vec2 chassis[] { { 0.9, 0.55 }, { 0.9, -0.55 }, { 0.2, -0.60 }, { -1.0, -0.60 },
        { -1.4, -0.40 }, { -1.4, 0.40 }, { -1.0, 0.60 }, { 0.2, 0.60 } };
    drawPolyLocal(renderer, camera, body, chassis, 8, true, chassisCol);

    // Operator overhead guard cage
    const vehicle_sim::math::Vec2 cage[] { { 0.3, 0.45 }, { 0.3, -0.45 }, { -0.7, -0.45 }, { -0.7, 0.45 } };
    drawPolyLocal(renderer, camera, body, cage, 4, true, chassisCol);

    // 2. Fixed front drive wheels
    drawLineLocal(renderer, camera, body, { 0.8, 0.65 }, { 0.8, 0.45 }, mintColor);
    drawLineLocal(renderer, camera, body, { 0.8, -0.65 }, { 0.8, -0.45 }, mintColor);

    // 3. Rear steering pivot wheel
    double steerAngle { 0.0 };
    if (std::abs(body.getAngularVelocity()) > 0.05) {
        steerAngle = std::clamp(body.getAngularVelocity() * 0.4, -1.45, 1.45);
    }
    const vehicle_sim::math::Vec2 steerDir { 0.30 * std::cos(steerAngle), 0.30 * std::sin(steerAngle) };
    const vehicle_sim::math::Vec2 pivotPos { -1.0, 0.0 };
    drawLineLocal(renderer, camera, body, pivotPos - steerDir, pivotPos + steerDir, amberColor);

    // 4. Front vertical mast & cantilevered load forks
    drawLineLocal(renderer, camera, body, { 0.95, -0.45 }, { 0.95, 0.45 }, amberColor);
    drawLineLocal(renderer, camera, body, { 1.05, -0.45 }, { 1.05, 0.45 }, amberColor);

    const double forkReach { 0.85 };
    drawLineLocal(renderer, camera, body, { 1.05, 0.25 }, { 1.05 + forkReach, 0.25 }, amberColor);
    drawLineLocal(renderer, camera, body, { 1.05, -0.25 }, { 1.05 + forkReach, -0.25 }, amberColor);
    drawLineLocal(renderer, camera, body, { 1.05 + forkReach, 0.23 }, { 1.05 + forkReach, 0.27 }, amberColor);
    drawLineLocal(renderer, camera, body, { 1.05 + forkReach, -0.23 }, { 1.05 + forkReach, -0.27 }, amberColor);

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawQuadcopterDrone(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::QuadcopterDroneDynamics& drone, const vehicle_sim::physics::RigidBody2D& body,
    double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color cyanColor { 101, 141, 160, 255 };
    const SDL_Color amberColor { 255, 223, 147, 255 };

    // 1. Central flight controller & battery pod
    const vehicle_sim::math::Vec2 pod[] { { 0.22, 0.12 }, { 0.22, -0.12 }, { -0.22, -0.12 }, { -0.22, 0.12 } };
    drawPolyLocal(renderer, camera, body, pod, 4, true, mintColor);

    // Forward direction arrow on top of pod
    drawLineLocal(renderer, camera, body, { 0.15, 0.0 }, { 0.05, 0.06 }, amberColor);
    drawLineLocal(renderer, camera, body, { 0.15, 0.0 }, { 0.05, -0.06 }, amberColor);

    // 2. 'X'-frame carbon fiber boom arms to 4 rotor hubs
    const vehicle_sim::math::Vec2 motorHubs[] {
        { 0.45, 0.45 }, // 0: FL
        { 0.45, -0.45 }, // 1: FR
        { -0.45, 0.45 }, // 2: RL
        { -0.45, -0.45 } // 3: RR
    };

    drawLineLocal(renderer, camera, body, { 0.15, 0.10 }, motorHubs[0], mintColor);
    drawLineLocal(renderer, camera, body, { 0.15, -0.10 }, motorHubs[1], mintColor);
    drawLineLocal(renderer, camera, body, { -0.15, 0.10 }, motorHubs[2], mintColor);
    drawLineLocal(renderer, camera, body, { -0.15, -0.10 }, motorHubs[3], mintColor);

    // 3. 4 spinning rotor blade pairs
    const auto& speeds = drone.getMotorSpeeds();
    const double rRotor { 0.30 };

    for (std::size_t i { 0 }; i < 4; ++i) {
        const double rotSpeed = (speeds[i] > 10.0) ? speeds[i] : (drone.isAirborne() ? 100.0 : 0.0);
        const double bladeAngle = animTimer * rotSpeed * ((i == 0 || i == 3) ? 1.0 : -1.0);

        const vehicle_sim::math::Vec2 b1 { rRotor * std::cos(bladeAngle), rRotor * std::sin(bladeAngle) };
        const vehicle_sim::math::Vec2 b2 { -rRotor * std::sin(bladeAngle), rRotor * std::cos(bladeAngle) };

        const auto hub = motorHubs[i];
        drawLineLocal(renderer, camera, body, hub - b1, hub + b1, cyanColor);
        drawLineLocal(renderer, camera, body, hub - b2, hub + b2, cyanColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::renderScene(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const vehicle_sim::dynamics::MotionLabCarDynamics& car, const vehicle_sim::physics::RigidBody2D& body,
    const SkidRibbonBuffer& skids, const TrailParticleBuffer& trail, bool barrierActive) noexcept
{
    VehicleRenderContext context {};
    context.type = MotionLabVehicleType::ClassicCar;
    context.terrain = car.getSurfaceMaterial().type;
    context.car = &car;
    renderScene(renderer, camera, context, body, skids, trail, barrierActive);
}

void WireframeRenderer::renderScene(SDL_Renderer* renderer, const MotionLabCamera& camera,
    const VehicleRenderContext& context, const vehicle_sim::physics::RigidBody2D& body, const SkidRibbonBuffer& skids,
    const TrailParticleBuffer& trail, bool barrierActive) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const auto theme = getTerrainTheme(context.terrain);
    SDL_SetRenderDrawColor(renderer, theme.background.r, theme.background.g, theme.background.b, theme.background.a);
    SDL_RenderClear(renderer);

    drawGrid(renderer, camera, theme.grid);
    drawTrail(renderer, camera, trail);
    drawSkids(renderer, camera, skids, theme.skid);

    if (barrierActive) {
        drawBarrier(renderer, camera);
    }

    switch (context.type) {
    case MotionLabVehicleType::ClassicCar:
        if (context.car != nullptr) {
            drawClassicCar(renderer, camera, *context.car, body);
        }
        break;
    case MotionLabVehicleType::RaceCar:
        if (context.raceCar != nullptr) {
            drawRaceCar(renderer, camera, *context.raceCar, body);
        }
        break;
    case MotionLabVehicleType::Rocket:
        if (context.rocket != nullptr) {
            drawRocket(renderer, camera, *context.rocket, body, context.animTimer);
        }
        break;
    case MotionLabVehicleType::Helicopter:
        if (context.helicopter != nullptr) {
            drawHelicopter(renderer, camera, *context.helicopter, body, context.animTimer);
        }
        break;
    case MotionLabVehicleType::Hovercraft:
        if (context.hovercraft != nullptr) {
            drawHovercraft(renderer, camera, *context.hovercraft, body);
        }
        break;
    case MotionLabVehicleType::TrackedTank:
        if (context.tank != nullptr) {
            drawTrackedTank(renderer, camera, *context.tank, body);
        }
        break;
    case MotionLabVehicleType::AntiGravity:
        if (context.antiGravity != nullptr) {
            drawAntiGravityRacer(renderer, camera, *context.antiGravity, body, context.animTimer);
        }
        break;
    case MotionLabVehicleType::DriftCar:
        if (context.driftCar != nullptr) {
            drawDriftCar(renderer, camera, *context.driftCar, body);
        }
        break;
    case MotionLabVehicleType::MonsterTruck:
        if (context.monsterCar != nullptr) {
            drawMonsterTruck(renderer, camera, *context.monsterCar, body);
        }
        break;
    case MotionLabVehicleType::SemiTruck:
        if (context.articulatedTruck != nullptr) {
            drawArticulatedTruck(renderer, camera, *context.articulatedTruck, body);
        }
        break;
    case MotionLabVehicleType::Forklift:
        if (context.forklift != nullptr) {
            drawForklift(renderer, camera, *context.forklift, body);
        }
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        if (context.quadcopter != nullptr) {
            drawQuadcopterDrone(renderer, camera, *context.quadcopter, body, context.animTimer);
        }
        break;
    }
}

} // namespace motion_lab
