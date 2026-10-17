#include "WindVectorFieldRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "apps/MotionLab/TelemetryHud.h"

namespace motion_lab {

namespace {

constexpr double kPi { 3.14159265358979323846 };
constexpr double kDegToRad { kPi / 180.0 };

const char* getPresetName(vehicle_sim::physics::WindEnvironmentPreset preset) noexcept
{
    using Preset = vehicle_sim::physics::WindEnvironmentPreset;
    switch (preset) {
    case Preset::Calm:
        return "CALM (0 M/S)";
    case Preset::LightBreeze:
        return "SEA BREEZE (5 M/S, E)";
    case Preset::CrosswindGale:
        return "CROSSWIND GALE (15 M/S, NW)";
    case Preset::SevereStorm:
        return "SEVERE STORM (25 M/S, W)";
    case Preset::Microburst:
        return "MICROBURST (38 M/S, S)";
    case Preset::DynamicStorm:
        return "DYNAMIC STORM (SHIFTING)";
    }
    return "UNKNOWN";
}

void drawCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color color) noexcept
{
    if (renderer == nullptr || radius <= 0) {
        return;
    }
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    constexpr int kSteps { 32 };
    constexpr double kAngleStep { 6.283185307179586 / static_cast<double>(kSteps) };
    int prevX { cx + radius };
    int prevY { cy };
    for (int i { 1 }; i <= kSteps; ++i) {
        const double angle { static_cast<double>(i) * kAngleStep };
        const int currX { cx + static_cast<int>(std::round(std::cos(angle) * radius)) };
        const int currY { cy + static_cast<int>(std::round(std::sin(angle) * radius)) };
        SDL_RenderDrawLine(renderer, prevX, prevY, currX, currY);
        prevX = currX;
        prevY = currY;
    }
}

} // namespace

WindVectorFieldRenderer::WindVectorFieldRenderer() noexcept
{
    mParticles.fill(StreamlineParticle { });
}

void WindVectorFieldRenderer::updateParticles(
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera,
    double dt,
    const vehicle_sim::physics::RigidBody2D* body) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    mAnimTimer += dt;

    const auto camTarget = camera.getPosition();
    const double zoom = std::max(camera.getZoom(), 1.0);
    const double viewHalfW = (camera.getConfig().viewportWidth * 0.5) / zoom + 20.0;
    const double viewHalfH = (camera.getConfig().viewportHeight * 0.5) / zoom + 20.0;

    std::size_t activeCount { 0U };
    for (std::size_t i { 0U }; i < mParticleCount; ++i) {
        auto& p = mParticles[i];
        p.lifeRemaining -= dt;
        if (p.lifeRemaining > 0.0) {
            // Shift history trail for smooth curved ribbons
            p.history[2] = p.history[1];
            p.history[1] = p.history[0];
            p.history[0] = p.position;

            const auto flow = coordinator.sampleWindAt(p.position);
            if (p.isWake) {
                p.position.x += (flow.x + p.wakeDrift.x) * dt;
                p.position.y += (flow.y + p.wakeDrift.y) * dt;
                p.wakeDrift.x *= 0.95;
                p.wakeDrift.y *= 0.95;
            } else {
                p.position.x += flow.x * dt;
                p.position.y += flow.y * dt;
            }

            // Check if still within camera bounds
            const double dx { std::abs(p.position.x - camTarget.x) };
            const double dy { std::abs(p.position.y - camTarget.y) };
            if (dx <= viewHalfW && dy <= viewHalfH) {
                if (activeCount != i) {
                    mParticles[activeCount] = p;
                }
                ++activeCount;
            }
        }
    }
    mParticleCount = activeCount;

    // Spawn vehicle wake turbulence particles when vehicle is in motion
    if (body != nullptr) {
        const auto bodyVel = body->getLinearVelocity();
        const double bodySpeed = bodyVel.length();
        if (bodySpeed > 2.0 && mParticleCount + 2U <= kMaxStreamlines) {
            const double heading = body->getOrientation();
            const double fwdX = std::cos(heading);
            const double fwdY = std::sin(heading);
            const double latX = -fwdY;
            const double latY = fwdX;

            constexpr double kHalfLen { 2.0 };
            constexpr double kHalfWid { 0.9 };
            const auto rearLeft = body->toWorldPoint({ -kHalfLen, kHalfWid });
            const auto rearRight = body->toWorldPoint({ -kHalfLen, -kHalfWid });

            // Port wake particle
            auto& pL = mParticles[mParticleCount++];
            pL.position = rearLeft;
            pL.history[0] = rearLeft;
            pL.history[1] = rearLeft;
            pL.history[2] = rearLeft;
            pL.wakeDrift = { -fwdX * bodySpeed * 0.35 + latX * 1.2, -fwdY * bodySpeed * 0.35 + latY * 1.2 };
            pL.totalLifetime = 1.0;
            pL.lifeRemaining = pL.totalLifetime;
            pL.intensity = 1.0f;
            pL.isWake = true;

            // Starboard wake particle
            auto& pR = mParticles[mParticleCount++];
            pR.position = rearRight;
            pR.history[0] = rearRight;
            pR.history[1] = rearRight;
            pR.history[2] = rearRight;
            pR.wakeDrift = { -fwdX * bodySpeed * 0.35 - latX * 1.2, -fwdY * bodySpeed * 0.35 - latY * 1.2 };
            pR.totalLifetime = 1.0;
            pR.lifeRemaining = pR.totalLifetime;
            pR.intensity = 1.0f;
            pR.isWake = true;
        }
    }

    // Spawn ambient wind streamline particles
    mSpawnTimer += dt;
    if (mSpawnTimer >= 0.04 && mParticleCount < kMaxStreamlines) {
        mSpawnTimer = 0.0;
        const auto snap = coordinator.getSnapshot();
        const double trueSpeed = snap.trueWindSpeed;
        if (trueSpeed > 0.1) {
            const std::size_t spawnBatch = std::min<std::size_t>(
                12U, kMaxStreamlines - mParticleCount);

            for (std::size_t s { 0U }; s < spawnBatch; ++s) {
                const double rx { ((static_cast<double>((activeCount * 37U + s * 97U) % 1000U) / 1000.0) - 0.5) * 2.0 };
                const double ry { ((static_cast<double>((activeCount * 53U + s * 131U) % 1000U) / 1000.0) - 0.5) * 2.0 };

                StreamlineParticle newP { };
                newP.position.x = camTarget.x + rx * viewHalfW;
                newP.position.y = camTarget.y + ry * viewHalfH;
                newP.history[0] = newP.position;
                newP.history[1] = newP.position;
                newP.history[2] = newP.position;
                newP.wakeDrift = { 0.0, 0.0 };
                newP.totalLifetime = 2.0;
                newP.lifeRemaining = newP.totalLifetime;
                newP.intensity = 1.0f;
                newP.isWake = false;

                mParticles[mParticleCount++] = newP;
            }
        }
    }
}

void WindVectorFieldRenderer::renderVectorField(
    SDL_Renderer* renderer,
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera,
    const vehicle_sim::physics::RigidBody2D* body) const noexcept
{
    if (renderer == nullptr || !mShowOverlay) {
        return;
    }

    const int vpW = camera.getConfig().viewportWidth;
    const int vpH = camera.getConfig().viewportHeight;
    if (vpW <= 0 || vpH <= 0) {
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    renderGridArrows(renderer, coordinator, camera);
    renderEddySwirls(renderer, coordinator, camera);
    renderStreamRibbons(renderer, coordinator, camera);

    if (body != nullptr) {
        renderChassisAero(renderer, coordinator, camera, *body);
    }
}

void WindVectorFieldRenderer::renderGridArrows(
    SDL_Renderer* renderer,
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera) const noexcept
{
    const int vpW = camera.getConfig().viewportWidth;
    const int vpH = camera.getConfig().viewportHeight;

    constexpr int kCols { 8 };
    constexpr int kRows { 6 };
    const int stepX = vpW / (kCols + 1);
    const int stepY = vpH / (kRows + 1);

    for (int r { 1 }; r <= kRows; ++r) {
        const int sy = r * stepY;
        for (int c { 1 }; c <= kCols; ++c) {
            const int sx = c * stepX;
            const auto worldPos = camera.screenToWorld({ static_cast<double>(sx), static_cast<double>(sy) });
            const auto windVel = coordinator.sampleWindAt(worldPos);
            const double speed = windVel.length();

            if (speed > 0.4) {
                const double lenPixels = std::clamp(6.0 + 3.0 * std::sqrt(speed), 6.0, 28.0);
                const double angle = std::atan2(-windVel.y, windVel.x);

                const int ex = sx + static_cast<int>(std::round(std::cos(angle) * lenPixels));
                const int ey = sy + static_cast<int>(std::round(std::sin(angle) * lenPixels));

                if (speed < 8.0) {
                    SDL_SetRenderDrawColor(renderer, 51, 255, 51, 120); // Neon Emerald
                } else if (speed < 20.0) {
                    SDL_SetRenderDrawColor(renderer, 255, 176, 0, 160); // Electric Amber
                } else {
                    SDL_SetRenderDrawColor(renderer, 255, 51, 51, 200); // Hazard Red
                }

                SDL_RenderDrawLine(renderer, sx, sy, ex, ey);

                constexpr double kHeadAngle { 0.45 };
                constexpr double kHeadLen { 5.0 };
                const int a1x = ex - static_cast<int>(std::round(std::cos(angle - kHeadAngle) * kHeadLen));
                const int a1y = ey - static_cast<int>(std::round(std::sin(angle - kHeadAngle) * kHeadLen));
                const int a2x = ex - static_cast<int>(std::round(std::cos(angle + kHeadAngle) * kHeadLen));
                const int a2y = ey - static_cast<int>(std::round(std::sin(angle + kHeadAngle) * kHeadLen));

                SDL_RenderDrawLine(renderer, ex, ey, a1x, a1y);
                SDL_RenderDrawLine(renderer, ex, ey, a2x, a2y);
            }
        }
    }
}

void WindVectorFieldRenderer::renderEddySwirls(
    SDL_Renderer* renderer,
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera) const noexcept
{
    const int vpW = camera.getConfig().viewportWidth;
    const int vpH = camera.getConfig().viewportHeight;

    constexpr int kCols { 7 };
    constexpr int kRows { 5 };
    const int stepX = vpW / (kCols + 1);
    const int stepY = vpH / (kRows + 1);
    constexpr double kDeltaH { 2.0 };

    for (int r { 1 }; r <= kRows; ++r) {
        const int sy = r * stepY;
        for (int c { 1 }; c <= kCols; ++c) {
            const int sx = c * stepX;
            const auto worldPos = camera.screenToWorld({ static_cast<double>(sx), static_cast<double>(sy) });

            const auto v0 = coordinator.sampleWindAt(worldPos);
            const auto vx = coordinator.sampleWindAt({ worldPos.x + kDeltaH, worldPos.y });
            const auto vy = coordinator.sampleWindAt({ worldPos.x, worldPos.y + kDeltaH });

            const double curl = (vx.y - v0.y) / kDeltaH - (vy.x - v0.x) / kDeltaH;
            const double absCurl = std::abs(curl);

            if (absCurl > 0.05) {
                const double signCurl = (curl >= 0.0) ? 1.0 : -1.0;
                const double spin = mAnimTimer * 2.8 * signCurl;
                const double radius = std::clamp(10.0 + absCurl * 40.0, 12.0, 24.0);
                const Uint8 alpha = static_cast<Uint8>(std::clamp(absCurl * 350.0, 70.0, 200.0));

                if (curl > 0.0) {
                    SDL_SetRenderDrawColor(renderer, 255, 176, 0, alpha);
                } else {
                    SDL_SetRenderDrawColor(renderer, 0, 229, 255, alpha);
                }

                constexpr int kSegments { 6 };
                int prevX { sx };
                int prevY { sy };
                for (int s { 0 }; s <= kSegments; ++s) {
                    const double frac = static_cast<double>(s) / static_cast<double>(kSegments);
                    const double r_curr = radius * (0.35 + 0.65 * frac);
                    const double angle = spin + frac * (kPi * 1.3) * signCurl;
                    const int px = sx + static_cast<int>(std::round(std::cos(angle) * r_curr));
                    const int py = sy - static_cast<int>(std::round(std::sin(angle) * r_curr));

                    if (s > 0) {
                        SDL_RenderDrawLine(renderer, prevX, prevY, px, py);
                    }
                    prevX = px;
                    prevY = py;
                }
            }
        }
    }
}

void WindVectorFieldRenderer::renderStreamRibbons(
    SDL_Renderer* renderer,
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera) const noexcept
{
    const int vpW = camera.getConfig().viewportWidth;
    const int vpH = camera.getConfig().viewportHeight;

    for (std::size_t i { 0U }; i < mParticleCount; ++i) {
        const auto& p = mParticles[i];

        const auto screenHead = camera.worldToScreen(p.position);
        const int hx = static_cast<int>(std::round(screenHead.x));
        const int hy = static_cast<int>(std::round(screenHead.y));

        if (hx < -50 || hx > vpW + 50 || hy < -50 || hy > vpH + 50) {
            continue;
        }

        const auto screenJ0 = camera.worldToScreen(p.history[0]);
        const auto screenJ1 = camera.worldToScreen(p.history[1]);
        const auto screenJ2 = camera.worldToScreen(p.history[2]);

        const int j0x = static_cast<int>(std::round(screenJ0.x));
        const int j0y = static_cast<int>(std::round(screenJ0.y));
        const int j1x = static_cast<int>(std::round(screenJ1.x));
        const int j1y = static_cast<int>(std::round(screenJ1.y));
        const int j2x = static_cast<int>(std::round(screenJ2.x));
        const int j2y = static_cast<int>(std::round(screenJ2.y));

        if (p.isWake) {
            SDL_SetRenderDrawColor(renderer, 240, 80, 20, 60);
            SDL_RenderDrawLine(renderer, j2x, j2y, j1x, j1y);

            SDL_SetRenderDrawColor(renderer, 255, 150, 30, 140);
            SDL_RenderDrawLine(renderer, j1x, j1y, j0x, j0y);

            SDL_SetRenderDrawColor(renderer, 255, 210, 60, 240);
            SDL_RenderDrawLine(renderer, j0x, j0y, hx, hy);

            const SDL_Rect headRect { hx - 1, hy - 1, 3, 3 };
            SDL_RenderFillRect(renderer, &headRect);
        } else {
            const auto flow = coordinator.sampleWindAt(p.position);
            const double spd = flow.length();

            Uint8 r_col { 0U };
            Uint8 g_col { 229U };
            Uint8 b_col { 255U };
            if (spd < 6.0) {
                r_col = 50U;
                g_col = 255U;
                b_col = 120U;
            } else if (spd > 18.0) {
                r_col = 255U;
                g_col = 200U;
                b_col = 80U;
            }

            SDL_SetRenderDrawColor(renderer, r_col, g_col, b_col, 50);
            SDL_RenderDrawLine(renderer, j2x, j2y, j1x, j1y);

            SDL_SetRenderDrawColor(renderer, r_col, g_col, b_col, 130);
            SDL_RenderDrawLine(renderer, j1x, j1y, j0x, j0y);

            SDL_SetRenderDrawColor(renderer, 220, 250, 255, 220);
            SDL_RenderDrawLine(renderer, j0x, j0y, hx, hy);

            const SDL_Rect headRect { hx - 1, hy - 1, 2, 2 };
            SDL_RenderFillRect(renderer, &headRect);
        }
    }
}

void WindVectorFieldRenderer::renderChassisAero(
    SDL_Renderer* renderer,
    const WindTurbulenceCoordinator& coordinator,
    const MotionLabCamera& camera,
    const vehicle_sim::physics::RigidBody2D& body) const noexcept
{
    const auto bodyPos = body.getPosition();
    const auto bodyVel = body.getLinearVelocity();

    const auto screenCenter = camera.worldToScreen(bodyPos);
    const int cx = static_cast<int>(std::round(screenCenter.x));
    const int cy = static_cast<int>(std::round(screenCenter.y));

    const int vpW = camera.getConfig().viewportWidth;
    const int vpH = camera.getConfig().viewportHeight;
    if (cx < -40 || cx > vpW + 40 || cy < -40 || cy > vpH + 40) {
        return;
    }

    const auto windVel = coordinator.sampleWindAt(bodyPos);
    const auto relAir = bodyVel - windVel;
    const double relSpeed = relAir.length();

    // 1. Apparent Relative Airflow arrow (incoming air direction onto car)
    if (relSpeed > 1.2) {
        const double inAngle = std::atan2(relAir.y, -relAir.x);
        const double arrowLen = std::clamp(16.0 + relSpeed * 0.9, 18.0, 50.0);
        const int sx = cx + static_cast<int>(std::round(std::cos(inAngle) * arrowLen));
        const int sy = cy - static_cast<int>(std::round(std::sin(inAngle) * arrowLen));

        SDL_SetRenderDrawColor(renderer, 0, 229, 255, 230);
        SDL_RenderDrawLine(renderer, sx, sy, cx, cy);

        constexpr double kHeadAngle { 0.45 };
        constexpr double kHeadLen { 6.0 };
        const int h1x = cx + static_cast<int>(std::round(std::cos(inAngle - kHeadAngle) * kHeadLen));
        const int h1y = cy - static_cast<int>(std::round(std::sin(inAngle - kHeadAngle) * kHeadLen));
        const int h2x = cx + static_cast<int>(std::round(std::cos(inAngle + kHeadAngle) * kHeadLen));
        const int h2y = cy - static_cast<int>(std::round(std::sin(inAngle + kHeadAngle) * kHeadLen));
        SDL_RenderDrawLine(renderer, cx, cy, h1x, h1y);
        SDL_RenderDrawLine(renderer, cx, cy, h2x, h2y);

        TelemetryHud::drawText(renderer, "AIR", sx - 8, sy - 10, 1, SDL_Color { 0, 229, 255, 220 });
    }

    // 2. Aerodynamic side force reaction arrow
    const auto snap = coordinator.getSnapshot();
    const double latForce = snap.lateralAeroForce;
    if (std::abs(latForce) > 40.0) {
        const double heading = body.getOrientation();
        const double signF = (latForce >= 0.0) ? 1.0 : -1.0;
        const double forceAngle = heading + (signF > 0.0 ? (kPi * 0.5) : (-kPi * 0.5));
        const double forceLen = std::clamp(12.0 + std::abs(latForce) * 0.035, 14.0, 44.0);

        const int fx = cx + static_cast<int>(std::round(std::cos(forceAngle) * forceLen));
        const int fy = cy - static_cast<int>(std::round(std::sin(forceAngle) * forceLen));

        SDL_SetRenderDrawColor(renderer, 255, 220, 40, 240);
        SDL_RenderDrawLine(renderer, cx, cy, fx, fy);

        constexpr double kHeadAngle { 0.45 };
        constexpr double kHeadLen { 6.0 };
        const int h1x = fx - static_cast<int>(std::round(std::cos(forceAngle - kHeadAngle) * kHeadLen));
        const int h1y = fy + static_cast<int>(std::round(std::sin(forceAngle - kHeadAngle) * kHeadLen));
        const int h2x = fx - static_cast<int>(std::round(std::cos(forceAngle + kHeadAngle) * kHeadLen));
        const int h2y = fy + static_cast<int>(std::round(std::sin(forceAngle + kHeadAngle) * kHeadLen));
        SDL_RenderDrawLine(renderer, fx, fy, h1x, h1y);
        SDL_RenderDrawLine(renderer, fx, fy, h2x, h2y);

        TelemetryHud::drawText(renderer, "F_AERO", fx + 3, fy - 5, 1, SDL_Color { 255, 220, 40, 220 });
    }
}

SDL_Point WindVectorFieldRenderer::computeHudCardPos(
    int screenWidth,
    int screenHeight,
    bool circuitActive,
    bool advancedHudActive) noexcept
{
    const int rightX { screenWidth - kCardWidth - 16 };
    SDL_Point pos { rightX, 20 };

    if (circuitActive) {
        if (advancedHudActive) {
            if (screenHeight < 750) {
                // Relocate to clean left column below live telemetry text
                pos.x = 24;
                pos.y = 205;
            } else {
                // Stack below Advanced Telemetry on right sidebar
                pos.x = rightX;
                pos.y = 416;
            }
        } else {
            // Stack below Circuit Radar on right sidebar
            pos.x = rightX;
            pos.y = 176;
        }
    } else {
        if (advancedHudActive) {
            if (screenHeight < 580) {
                pos.x = 24;
                pos.y = 205;
            } else {
                // Stack directly below Advanced Telemetry on right sidebar
                pos.x = rightX;
                pos.y = 260;
            }
        } else {
            // Top-right corner of screen
            pos.x = rightX;
            pos.y = 20;
        }
    }

    return pos;
}

void WindVectorFieldRenderer::renderHudCard(
    SDL_Renderer* renderer,
    const WindTelemetrySnapshot& snapshot,
    int originX, int originY) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    // 1. Semi-translucent dark card background
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 10, 16, 29, 210);
    const SDL_Rect bgRect { originX, originY, kCardWidth, kCardHeight };
    SDL_RenderFillRect(renderer, &bgRect);

    // 2. Outer border wireframe
    SDL_SetRenderDrawColor(renderer, 0, 229, 255, 230);
    SDL_RenderDrawRect(renderer, &bgRect);

    // Header
    const SDL_Color cyan { 0, 229, 255, 255 };
    const SDL_Color amber { 255, 176, 0, 255 };
    const SDL_Color white { 220, 230, 245, 255 };

    TelemetryHud::drawText(renderer, "WIND & TURBULENCE", originX + 12, originY + 8, 2, cyan);

    // Preset & status lines
    char buffer[64] { };
    std::snprintf(buffer, sizeof(buffer), "ENV: %s", getPresetName(snapshot.activePreset));
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 28, 1, amber);

    const double knots = snapshot.trueWindSpeed * 1.94384;
    std::snprintf(buffer, sizeof(buffer), "SPEED: %4.1f M/S (%4.1f KT)", snapshot.trueWindSpeed, knots);
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 42, 1, white);

    std::snprintf(buffer, sizeof(buffer), "HDG:   %3.0f DEG", snapshot.trueWindHeadingDeg);
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 56, 1, white);

    if (snapshot.gustPulseActive) {
        TelemetryHud::drawText(renderer, "GUST: ACTIVE PULSE [G]", originX + 12, originY + 70, 1, amber);
    } else {
        TelemetryHud::drawText(renderer, "GUST: READY (KEY: G)", originX + 12, originY + 70, 1, cyan);
    }

    // 3. Compass Rose Widget (left side)
    const int compassCx = originX + 50;
    const int compassCy = originY + 125;
    constexpr int compassR { 28 };

    drawCircle(renderer, compassCx, compassCy, compassR, cyan);
    SDL_SetRenderDrawColor(renderer, 0, 229, 255, 120);
    SDL_RenderDrawLine(renderer, compassCx, compassCy - compassR - 3, compassCx, compassCy + compassR + 3);
    SDL_RenderDrawLine(renderer, compassCx - compassR - 3, compassCy, compassCx + compassR + 3, compassCy);

    TelemetryHud::drawText(renderer, "N", compassCx - 3, compassCy - compassR - 10, 1, cyan);
    TelemetryHud::drawText(renderer, "S", compassCx - 3, compassCy + compassR + 2, 1, cyan);
    TelemetryHud::drawText(renderer, "W", compassCx - compassR - 10, compassCy - 3, 1, cyan);
    TelemetryHud::drawText(renderer, "E", compassCx + compassR + 2, compassCy - 3, 1, cyan);

    // True Wind Vector needle (Amber)
    if (snapshot.trueWindSpeed > 0.05) {
        const double trueWindRad = snapshot.trueWindHeadingDeg * kDegToRad;
        // Direction wind blows TOWARD on map
        const double flowAngle = kPi * 1.5 - trueWindRad;
        const int needleLen = 22;
        const int nx = compassCx + static_cast<int>(std::round(std::cos(flowAngle) * needleLen));
        const int ny = compassCy - static_cast<int>(std::round(std::sin(flowAngle) * needleLen));
        SDL_SetRenderDrawColor(renderer, 255, 176, 0, 255);
        SDL_RenderDrawLine(renderer, compassCx, compassCy, nx, ny);
    }

    // 4. Articulated Windsock Widget (right side)
    const int sockBaseX = originX + 130;
    const int sockBaseY = originY + 105;

    // Pole
    SDL_SetRenderDrawColor(renderer, 180, 190, 205, 255);
    SDL_RenderDrawLine(renderer, sockBaseX, sockBaseY, sockBaseX, sockBaseY + 45);
    SDL_RenderDrawLine(renderer, sockBaseX - 3, sockBaseY + 45, sockBaseX + 3, sockBaseY + 45);

    // Droop angle: 0 rad (straight down) to pi/2 rad (horizontal)
    const double droopFrac = std::clamp(snapshot.trueWindSpeed / 16.0, 0.05, 1.0);
    const double droopAngle = droopFrac * (kPi * 0.45); // Extends rightward

    const int segLen = 14;
    int currX = sockBaseX;
    int currY = sockBaseY + 4;

    for (int seg { 0 }; seg < 3; ++seg) {
        const double w1 = 7.0 - seg * 1.8;
        const double w2 = 7.0 - (seg + 1) * 1.8;

        const int nextX = currX + static_cast<int>(std::round(std::sin(droopAngle) * segLen));
        const int nextY = currY + static_cast<int>(std::round(std::cos(droopAngle) * segLen));

        // Alternating red/white retro bands
        if (seg % 2 == 0) {
            SDL_SetRenderDrawColor(renderer, 255, 60, 60, 255);
        } else {
            SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        }

        SDL_RenderDrawLine(renderer, currX, currY - static_cast<int>(w1), nextX, nextY - static_cast<int>(w2));
        SDL_RenderDrawLine(renderer, currX, currY + static_cast<int>(w1), nextX, nextY + static_cast<int>(w2));
        SDL_RenderDrawLine(renderer, nextX, nextY - static_cast<int>(w2), nextX, nextY + static_cast<int>(w2));

        currX = nextX;
        currY = nextY;
    }

    std::snprintf(buffer, sizeof(buffer), "EXT: %2.0f%%", droopFrac * 100.0);
    TelemetryHud::drawText(renderer, buffer, originX + 130, originY + 145, 1, amber);

    // 5. Aerodynamic Reactions Telemetry
    std::snprintf(buffer, sizeof(buffer), "REL SPEED: %4.1f M/S", snapshot.relativeAirspeed);
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 168, 1, cyan);

    std::snprintf(buffer, sizeof(buffer), "AERO SIDE: %5.0f N", std::abs(snapshot.lateralAeroForce));
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 182, 1, white);

    std::snprintf(buffer, sizeof(buffer), "YAW MOM:   %5.0f NM", std::abs(snapshot.yawOverturningMoment));
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 196, 1, white);

    std::snprintf(buffer, sizeof(buffer), "DOWNFORCE: %5.0f N", snapshot.downforceLoad);
    TelemetryHud::drawText(renderer, buffer, originX + 12, originY + 210, 1, white);

    // 6. Crosswind alert warning
    if (snapshot.crosswindAlert) {
        SDL_SetRenderDrawColor(renderer, 255, 30, 30, 240);
        const SDL_Rect alertRect { originX + 8, originY + 226, kCardWidth - 16, 16 };
        SDL_RenderFillRect(renderer, &alertRect);
        TelemetryHud::drawText(renderer, "! CROSSWIND ALERT !", originX + 22, originY + 230, 1, white);
    }
}

void WindVectorFieldRenderer::toggleOverlay() noexcept
{
    mShowOverlay = !mShowOverlay;
}

void WindVectorFieldRenderer::setOverlayEnabled(bool enabled) noexcept
{
    mShowOverlay = enabled;
}

bool WindVectorFieldRenderer::isOverlayEnabled() const noexcept
{
    return mShowOverlay;
}

} // namespace motion_lab
