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

void drawLineLocal(SDL_Renderer* renderer,
                   const MotionLabCamera& camera,
                   const vehicle_sim::physics::RigidBody2D& body,
                   const vehicle_sim::math::Vec2& localA,
                   const vehicle_sim::math::Vec2& localB,
                   SDL_Color color) noexcept
{
    const auto worldA = body.toWorldPoint(localA);
    const auto worldB = body.toWorldPoint(localB);
    const auto screenA = camera.worldToScreen(worldA);
    const auto screenB = camera.worldToScreen(worldB);
    WireframeRenderer::drawLine(renderer,
                                static_cast<int>(std::round(screenA.x)),
                                static_cast<int>(std::round(screenA.y)),
                                static_cast<int>(std::round(screenB.x)),
                                static_cast<int>(std::round(screenB.y)),
                                color);
}

void drawPolyLocal(SDL_Renderer* renderer,
                   const MotionLabCamera& camera,
                   const vehicle_sim::physics::RigidBody2D& body,
                   const vehicle_sim::math::Vec2* points,
                   std::size_t count,
                   bool closed,
                   SDL_Color color) noexcept
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

void drawComAndVelocity(SDL_Renderer* renderer,
                        const MotionLabCamera& camera,
                        const vehicle_sim::physics::RigidBody2D& body,
                        SDL_Color dotColor) noexcept
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
        SDL_RenderDrawLine(renderer,
                           static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
                           static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)));
    }

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

    SDL_Rect rect {
        static_cast<int>(std::round(topLeft.x)),
        static_cast<int>(std::round(topLeft.y)),
        static_cast<int>(std::round(bottomRight.x - topLeft.x)),
        static_cast<int>(std::round(bottomRight.y - topLeft.y))
    };
    SDL_RenderDrawRect(renderer, &rect);
}

void WireframeRenderer::drawClassicCar(SDL_Renderer* renderer,
                                      const MotionLabCamera& camera,
                                      const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                                      const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const auto worldCorners = car.computeWorldPoints(body);
    std::array<vehicle_sim::math::Vec2, 4> screenCorners { };
    for (std::size_t i { 0 }; i < 4; ++i) {
        screenCorners[i] = camera.worldToScreen(worldCorners[i]);
    }

    const SDL_Color mintColor { 140, 230, 197, 255 };
    for (std::size_t i { 0 }; i < 4; ++i) {
        const auto& p1 = screenCorners[i];
        const auto& p2 = screenCorners[(i + 1) % 4];
        drawLine(renderer,
                 static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
                 static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)),
                 mintColor);
    }

    const auto crossA = camera.worldToScreen(body.toWorldPoint({ 0.7, -0.7 }));
    const auto crossB = camera.worldToScreen(body.toWorldPoint({ 0.7, 0.7 }));
    drawLine(renderer,
             static_cast<int>(std::round(crossA.x)), static_cast<int>(std::round(crossA.y)),
             static_cast<int>(std::round(crossB.x)), static_cast<int>(std::round(crossB.y)),
             mintColor);

    const SDL_Color handbrakeColor { 255, 223, 147, 255 };
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

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawRaceCar(SDL_Renderer* renderer,
                                   const MotionLabCamera& camera,
                                   const vehicle_sim::dynamics::RaceCarDynamics& raceCar,
                                   const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color drsColor { 255, 223, 147, 255 };

    // 1. Tapered aerodynamic chassis & cockpit
    const vehicle_sim::math::Vec2 noseBody[] {
        { 2.2, 0.0 }, { 1.1, -0.32 }, { 0.3, -0.42 }, { -0.5, -0.42 },
        { -1.4, -0.32 }, { -1.8, -0.22 }, { -1.8, 0.22 }, { -1.4, 0.32 },
        { -0.5, 0.42 }, { 0.3, 0.42 }, { 1.1, 0.32 }
    };
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
        const vehicle_sim::math::Vec2 half {
            0.38 * std::cos(steerAngle),
            0.38 * std::sin(steerAngle)
        };
        const auto localPos = wheels[i].localOffset;
        const bool skidding { std::abs(wheels[i].slipAngle) > 0.15 || std::abs(wheels[i].slipRatio) > 0.15 };
        const SDL_Color wheelColor { skidding ? drsColor : mintColor };
        drawLineLocal(renderer, camera, body, localPos - half, localPos + half, wheelColor);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawRocket(SDL_Renderer* renderer,
                                  const MotionLabCamera& camera,
                                  const vehicle_sim::dynamics::RocketDynamics& rocket,
                                  const vehicle_sim::physics::RigidBody2D& body,
                                  double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color flameColor { 255, 140, 50, 255 };
    const SDL_Color flameCore { 255, 235, 120, 255 };

    // 1. Fuselage cylinder & pointed ogive fairing
    const vehicle_sim::math::Vec2 rocketHull[] {
        { -2.2, -0.75 }, { 1.5, -0.75 }, { 2.8, 0.0 },
        { 1.5, 0.75 }, { -2.2, 0.75 }
    };
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

    const vehicle_sim::math::Vec2 bellLeft {
        bellMount.x - 0.65 * cosG - (-0.35) * sinG,
        bellMount.y - 0.65 * sinG + (-0.35) * cosG
    };
    const vehicle_sim::math::Vec2 bellRight {
        bellMount.x - 0.65 * cosG - 0.35 * sinG,
        bellMount.y - 0.65 * sinG + 0.35 * cosG
    };

    drawLineLocal(renderer, camera, body, bellMount, bellLeft, mintColor);
    drawLineLocal(renderer, camera, body, bellMount, bellRight, mintColor);
    drawLineLocal(renderer, camera, body, bellLeft, bellRight, mintColor);

    // 3. Dynamic thrust exhaust flame
    if (rocket.getThrustForce() > 10.0) {
        const double flicker { 0.4 * std::sin(animTimer * 50.0) + 0.2 * std::cos(animTimer * 30.0) };
        const double flameLen { 1.5 + flicker };
        const vehicle_sim::math::Vec2 flameTip {
            bellMount.x - (0.65 + flameLen) * cosG,
            bellMount.y - (0.65 + flameLen) * sinG
        };
        drawLineLocal(renderer, camera, body, bellLeft, flameTip, flameColor);
        drawLineLocal(renderer, camera, body, bellRight, flameTip, flameColor);
        drawLineLocal(renderer, camera, body, bellMount, flameTip, flameCore);
    }

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawHelicopter(SDL_Renderer* renderer,
                                      const MotionLabCamera& camera,
                                      const vehicle_sim::dynamics::HelicopterDynamics& helicopter,
                                      const vehicle_sim::physics::RigidBody2D& body,
                                      double animTimer) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };

    // 1. Fuselage pod & cabin
    const vehicle_sim::math::Vec2 cabin[] {
        { 1.8, 0.0 }, { 1.1, 0.65 }, { -0.8, 0.55 },
        { -0.8, -0.55 }, { 1.1, -0.65 }
    };
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
    const vehicle_sim::math::Vec2 tailTip1 { tailHub.x + 0.3 * std::cos(tailAngle), tailHub.y + 0.3 * std::sin(tailAngle) };
    const vehicle_sim::math::Vec2 tailTip2 { tailHub.x - 0.3 * std::cos(tailAngle), tailHub.y - 0.3 * std::sin(tailAngle) };
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

void WireframeRenderer::drawHovercraft(SDL_Renderer* renderer,
                                      const MotionLabCamera& camera,
                                      const vehicle_sim::dynamics::HovercraftDynamics& hovercraft,
                                      const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color rudderColor { 255, 223, 147, 255 };

    // 1. Perimeter cushion skirt
    const vehicle_sim::math::Vec2 skirt[] {
        { 2.0, 0.5 }, { 1.5, 1.05 }, { -1.5, 1.05 }, { -2.0, 0.6 },
        { -2.0, -0.6 }, { -1.5, -1.05 }, { 1.5, -1.05 }, { 2.0, -0.5 }
    };
    drawPolyLocal(renderer, camera, body, skirt, 8, true, mintColor);

    // Inner deck boundary
    const vehicle_sim::math::Vec2 innerDeck[] {
        { 1.6, 0.35 }, { 1.2, 0.8 }, { -1.2, 0.8 }, { -1.6, 0.45 },
        { -1.6, -0.45 }, { -1.2, -0.8 }, { 1.2, -0.8 }, { 1.6, -0.35 }
    };
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
    drawLineLocal(renderer, camera, body, { ductL.x - 0.3, ductL.y - 0.3 }, { ductL.x + 0.3, ductL.y - 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductL.x + 0.3, ductL.y - 0.3 }, { ductL.x + 0.3, ductL.y + 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductL.x + 0.3, ductL.y + 0.3 }, { ductL.x - 0.3, ductL.y + 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductL.x - 0.3, ductL.y + 0.3 }, { ductL.x - 0.3, ductL.y - 0.3 }, mintColor);

    drawLineLocal(renderer, camera, body, { ductR.x - 0.3, ductR.y - 0.3 }, { ductR.x + 0.3, ductR.y - 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductR.x + 0.3, ductR.y - 0.3 }, { ductR.x + 0.3, ductR.y + 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductR.x + 0.3, ductR.y + 0.3 }, { ductR.x - 0.3, ductR.y + 0.3 }, mintColor);
    drawLineLocal(renderer, camera, body, { ductR.x - 0.3, ductR.y + 0.3 }, { ductR.x - 0.3, ductR.y - 0.3 }, mintColor);

    // Steerable rudder vanes
    const vehicle_sim::math::Vec2 rudderHalf { 0.25 * cosR, 0.25 * sinR };
    drawLineLocal(renderer, camera, body, ductL - rudderHalf, ductL + rudderHalf, rudderColor);
    drawLineLocal(renderer, camera, body, ductR - rudderHalf, ductR + rudderHalf, rudderColor);

    drawComAndVelocity(renderer, camera, body, mintColor);
}

void WireframeRenderer::drawTrackedTank(SDL_Renderer* renderer,
                                       const MotionLabCamera& camera,
                                       const vehicle_sim::dynamics::TrackedTankDynamics& tank,
                                       const vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const SDL_Color mintColor { 140, 230, 197, 255 };
    const SDL_Color recoilColor { 255, 223, 147, 255 };

    // 1. Armored chassis hull
    const vehicle_sim::math::Vec2 hull[] {
        { 1.6, -0.85 }, { 2.1, 0.0 }, { 1.6, 0.85 },
        { -1.8, 0.85 }, { -1.8, -0.85 }
    };
    drawPolyLocal(renderer, camera, body, hull, 5, true, mintColor);

    // 2. Left and right continuous track belts with cleats
    const vehicle_sim::math::Vec2 trackL[] {
        { -2.1, 0.90 }, { 2.1, 0.90 }, { 2.1, 1.25 }, { -2.1, 1.25 }
    };
    drawPolyLocal(renderer, camera, body, trackL, 4, true, mintColor);
    for (double x { -1.8 }; x <= 1.8; x += 0.45) {
        drawLineLocal(renderer, camera, body, { x, 0.90 }, { x, 1.25 }, mintColor);
    }

    const vehicle_sim::math::Vec2 trackR[] {
        { -2.1, -1.25 }, { 2.1, -1.25 }, { 2.1, -0.90 }, { -2.1, -0.90 }
    };
    drawPolyLocal(renderer, camera, body, trackR, 4, true, mintColor);
    for (double x { -1.8 }; x <= 1.8; x += 0.45) {
        drawLineLocal(renderer, camera, body, { x, -1.25 }, { x, -0.90 }, mintColor);
    }

    // 3. Turret cupola & long cannon barrel
    const vehicle_sim::math::Vec2 turret[] {
        { 0.7, -0.45 }, { 0.7, 0.45 }, { -0.5, 0.55 }, { -0.6, -0.55 }
    };
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

void WireframeRenderer::renderScene(SDL_Renderer* renderer,
                                   const MotionLabCamera& camera,
                                   const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                                   const vehicle_sim::physics::RigidBody2D& body,
                                   const SkidRibbonBuffer& skids,
                                   const TrailParticleBuffer& trail,
                                   bool barrierActive) noexcept
{
    VehicleRenderContext context { };
    context.type = MotionLabVehicleType::ClassicCar;
    context.terrain = car.getSurfaceMaterial().type;
    context.car = &car;
    renderScene(renderer, camera, context, body, skids, trail, barrierActive);
}

void WireframeRenderer::renderScene(SDL_Renderer* renderer,
                                   const MotionLabCamera& camera,
                                   const VehicleRenderContext& context,
                                   const vehicle_sim::physics::RigidBody2D& body,
                                   const SkidRibbonBuffer& skids,
                                   const TrailParticleBuffer& trail,
                                   bool barrierActive) noexcept
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
    }
}

} // namespace motion_lab
