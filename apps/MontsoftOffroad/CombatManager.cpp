#include "apps/MontsoftOffroad/CombatManager.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::dynamics::BlastEvent;
using vehicle_sim::dynamics::ProjectileState;
using vehicle_sim::dynamics::ProjectileType;
using vehicle_sim::math::Vec2;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::terrain::Heightfield1D;

CombatManager::CombatManager() noexcept
    : config_{}
{
    reset();
}

CombatManager::CombatManager(const CombatManagerConfig& config) noexcept
    : config_{ config }
{
    reset();
}

void CombatManager::reset() noexcept
{
    ballistics_.reset();
    rocketTimer_ = 0.0;
    mortarTimer_ = 0.0;
    nextShockwaveSlot_ = 0;

    for (auto& sw : shockwaves_) {
        sw.active = false;
        sw.age = 0.0;
    }
}

void CombatManager::updateCooldowns(double dt) noexcept
{
    if (rocketTimer_ > 0.0) {
        rocketTimer_ = std::max(0.0, rocketTimer_ - dt);
    }
    if (mortarTimer_ > 0.0) {
        mortarTimer_ = std::max(0.0, mortarTimer_ - dt);
    }
}

void CombatManager::handleFireInputs(
    const vehicle_sim::dynamics::OffroadTruckInput& input,
    RigidBody2D& truckBody,
    const vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics,
    Camera2D& camera
) noexcept
{
    static_cast<void>(truckDynamics);

    if (input.firePrimary) {
        if (fireRocket(truckBody)) {
            camera.addTrauma(0.18);
        }
    }

    if (input.fireSecondary) {
        if (fireMortar(truckBody)) {
            camera.addTrauma(0.25);
        }
    }
}

bool CombatManager::fireRocket(RigidBody2D& truckBody) noexcept
{
    if (!canFireRocket()) {
        return false;
    }

    const double heading { truckBody.getOrientation() };
    const double cosH { std::cos(heading) };
    const double sinH { std::sin(heading) };

    const Vec2 localOffset{ 1.20, 0.22 };
    const Vec2 rotatedOffset{
        localOffset.x * cosH - localOffset.y * sinH,
        localOffset.x * sinH + localOffset.y * cosH
    };
    const Vec2 muzzlePos { truckBody.getPosition() + rotatedOffset };

    ballistics_.fireRocket(muzzlePos, heading, truckBody.getLinearVelocity());

    // Reaction recoil impulse
    const Vec2 recoilDir{ -cosH, -sinH };
    truckBody.applyImpulseAtPoint(recoilDir * config_.rocketRecoilImpulse, muzzlePos);

    rocketTimer_ = config_.rocketCooldownTime;
    return true;
}

bool CombatManager::fireMortar(RigidBody2D& truckBody) noexcept
{
    if (!canFireMortar()) {
        return false;
    }

    const double heading { truckBody.getOrientation() };
    const double cosH { std::cos(heading) };
    const double sinH { std::sin(heading) };

    const Vec2 localOffset{ 0.0, 0.40 };
    const Vec2 rotatedOffset{
        localOffset.x * cosH - localOffset.y * sinH,
        localOffset.x * sinH + localOffset.y * cosH
    };
    const Vec2 muzzlePos { truckBody.getPosition() + rotatedOffset };

    ballistics_.fireMortar(muzzlePos, truckBody.getLinearVelocity());

    // Vertical downward reaction kick on suspension
    const Vec2 recoilDir{ 0.0, -1.0 };
    truckBody.applyImpulseAtPoint(recoilDir * config_.mortarRecoilImpulse, muzzlePos);

    mortarTimer_ = config_.mortarCooldownTime;
    return true;
}

void CombatManager::emitExhaustTrails(ParticleSystem& particleSystem) noexcept
{
    for (const auto& proj : ballistics_.getProjectiles()) {
        if (!proj.isAlive) {
            continue;
        }

        if (proj.type == ProjectileType::ForwardRocket) {
            const double cosH { std::cos(proj.heading) };
            const double sinH { std::sin(proj.heading) };
            const Vec2 nozzlePos { proj.position - Vec2{ cosH * 0.35, sinH * 0.35 } };
            particleSystem.emitRocketExhaust(nozzlePos, proj.velocity, proj.heading);
        } else if (proj.type == ProjectileType::AntiAirMortar) {
            particleSystem.emitMortarTrail(proj.position, proj.velocity);
        }
    }
}

void CombatManager::performSweepCollisions(std::vector<CombatTargetAabb>& targets) noexcept
{
    const auto& projectiles { ballistics_.getProjectiles() };
    for (std::size_t pIdx { 0 }; pIdx < projectiles.size(); ++pIdx) {
        const auto& proj { projectiles[pIdx] };
        if (!proj.isAlive) {
            continue;
        }

        for (auto& target : targets) {
            if (!target.isAlive) {
                continue;
            }

            // AABB containment hit test
            const bool hitX { proj.position.x >= target.minBounds.x && proj.position.x <= target.maxBounds.x };
            const bool hitY { proj.position.y >= target.minBounds.y && proj.position.y <= target.maxBounds.y };

            if (hitX && hitY) {
                const double dmg { (proj.type == ProjectileType::ForwardRocket)
                    ? ballistics_.getRocketConfig().baseDamage
                    : ballistics_.getMortarConfig().baseDamage };

                target.health -= dmg;
                if (target.health <= 0.0) {
                    target.isAlive = false;
                }

                ballistics_.detonate(pIdx);
                break;
            }
        }
    }
}

void CombatManager::spawnShockwave(const Vec2& epicenter, double radius) noexcept
{
    auto& sw { shockwaves_[nextShockwaveSlot_] };
    nextShockwaveSlot_ = (nextShockwaveSlot_ + 1U) % kMaxShockwaves;

    sw.worldEpicenter = epicenter;
    sw.currentRadius = 0.4;
    sw.maxRadius = radius * 1.15;
    sw.age = 0.0;
    sw.maxLifespan = 0.28;
    sw.color = ColorRgba{ 255, 255, 255, 255 };
    sw.active = true;
}

void CombatManager::updateShockwaves(double dt) noexcept
{
    for (auto& sw : shockwaves_) {
        if (!sw.active) {
            continue;
        }

        sw.age += dt;
        if (sw.age >= sw.maxLifespan) {
            sw.active = false;
            continue;
        }

        const double frac { sw.age / sw.maxLifespan };
        sw.currentRadius = sw.maxRadius * std::sqrt(frac);
    }
}

void CombatManager::processDetonations(
    ParticleSystem& particleSystem,
    Camera2D& camera,
    const std::vector<RigidBody2D*>& rigidBodies,
    std::size_t stageIndex
) noexcept
{
    const auto blasts { ballistics_.consumeBlasts() };
    for (const auto& blast : blasts) {
        // 1. Transfer explosive impulse to rigid bodies
        ballistics_.applyBlast(blast, rigidBodies);

        // 2. Radial pyrotechnic shrapnel burst
        particleSystem.emitExplosionShrapnel(blast.epicenter, blast.radius, 1.0, stageIndex);

        // 3. Animated CGA shockwave ring
        spawnShockwave(blast.epicenter, blast.radius);

        // 4. Proximity camera trauma shake
        const double dist { (camera.getPosition() - blast.epicenter).length() };
        if (dist < config_.blastTraumaDistance && config_.blastTraumaDistance > 0.0) {
            const double traumaScale { 1.0 - (dist / config_.blastTraumaDistance) };
            camera.addTrauma(traumaScale * 0.38);
        }
    }
}

void CombatManager::update(
    double dt,
    const Heightfield1D* terrain,
    ParticleSystem& particleSystem,
    Camera2D& camera,
    const std::vector<RigidBody2D*>& rigidBodies,
    std::vector<CombatTargetAabb>* targets,
    std::size_t stageIndex
) noexcept
{
    updateCooldowns(dt);

    if (targets != nullptr && !targets->empty()) {
        performSweepCollisions(*targets);
    }

    ballistics_.step(dt, terrain);
    emitExhaustTrails(particleSystem);
    processDetonations(particleSystem, camera, rigidBodies, stageIndex);
    updateShockwaves(dt);
}

void CombatManager::render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    // 1. Render active shockwave rings
    const double ppm { camera.getConfig().pixelsPerMeter };

    for (const auto& sw : shockwaves_) {
        if (!sw.active) {
            continue;
        }

        const Vec2 screenPos { camera.worldToScreen(sw.worldEpicenter) };
        const int cx { static_cast<int>(std::round(screenPos.x)) };
        const int cy { static_cast<int>(std::round(screenPos.y)) };
        const int rPx { static_cast<int>(std::round(sw.currentRadius * ppm)) };

        if (cx < -40 || cx >= CgaPaletteRenderer::kNativeWidth + 40 ||
            cy < -40 || cy >= CgaPaletteRenderer::kNativeHeight + 40) {
            continue;
        }

        // Draw dithered circle outline
        if (rPx <= 2) {
            renderer.fillRect(cx - 1, cy - 1, 3, 3, sw.color);
        } else {
            // Draw circle with stippled pixel parity
            int x { rPx };
            int y { 0 };
            int err { 0 };

            while (x >= y) {
                const auto setStippled = [&](int px, int py) noexcept {
                    if ((px + py) % 2 == 0) {
                        renderer.setPixel(px, py, sw.color);
                    }
                };

                setStippled(cx + x, cy + y);
                setStippled(cx + y, cy + x);
                setStippled(cx - y, cy + x);
                setStippled(cx - x, cy + y);
                setStippled(cx - x, cy - y);
                setStippled(cx - y, cy - x);
                setStippled(cx + y, cy - x);
                setStippled(cx + x, cy - y);

                if (err <= 0) {
                    y += 1;
                    err += 2 * y + 1;
                }
                if (err > 0) {
                    x -= 1;
                    err -= 2 * x + 1;
                }
            }
        }
    }

    // 2. Render active projectiles
    for (const auto& proj : ballistics_.getProjectiles()) {
        if (!proj.isAlive) {
            continue;
        }

        const Vec2 screenPos { camera.worldToScreen(proj.position) };
        const int px { static_cast<int>(std::round(screenPos.x)) };
        const int py { static_cast<int>(std::round(screenPos.y)) };

        if (px < -10 || px >= CgaPaletteRenderer::kNativeWidth + 10 ||
            py < -10 || py >= CgaPaletteRenderer::kNativeHeight + 10) {
            continue;
        }

        if (proj.type == ProjectileType::ForwardRocket) {
            // Directional Rocket Sprite (7x3 px along heading)
            const double cosH { std::cos(proj.heading) };
            const double sinH { std::sin(proj.heading) };

            const int noseX { static_cast<int>(std::round(screenPos.x + cosH * 3.5)) };
            const int noseY { static_cast<int>(std::round(screenPos.y - sinH * 3.5)) };
            const int tailX { static_cast<int>(std::round(screenPos.x - cosH * 2.5)) };
            const int tailY { static_cast<int>(std::round(screenPos.y + sinH * 2.5)) };

            // White warhead nose
            renderer.setPixel(noseX, noseY, ColorRgba{ 255, 255, 255, 255 });

            // Core fuselage body
            renderer.drawLine(tailX, tailY, noseX, noseY, ColorRgba{ 85, 255, 255, 255 });

            // Stabilizer fins at tail
            const double perpX { -sinH * 1.5 };
            const double perpY { -cosH * 1.5 };
            renderer.setPixel(static_cast<int>(std::round(tailX + perpX)), static_cast<int>(std::round(tailY + perpY)), ColorRgba{ 255, 85, 85, 255 });
            renderer.setPixel(static_cast<int>(std::round(tailX - perpX)), static_cast<int>(std::round(tailY - perpY)), ColorRgba{ 255, 85, 85, 255 });

            // Flickering engine flare behind tail
            const int flameX { static_cast<int>(std::round(tailX - cosH * 1.8)) };
            const int flameY { static_cast<int>(std::round(tailY + sinH * 1.8)) };
            renderer.setPixel(flameX, flameY, ColorRgba{ 255, 255, 85, 255 });
        } else if (proj.type == ProjectileType::AntiAirMortar) {
            // High-angle Anti-Air Mortar: 3x3 Flak diamond
            renderer.setPixel(px, py, ColorRgba{ 255, 255, 255, 255 });     // Hot core
            renderer.setPixel(px - 1, py, ColorRgba{ 85, 255, 255, 255 }); // Casing
            renderer.setPixel(px + 1, py, ColorRgba{ 85, 255, 255, 255 });
            renderer.setPixel(px, py - 1, ColorRgba{ 255, 255, 85, 255 });
            renderer.setPixel(px, py + 1, ColorRgba{ 255, 255, 85, 255 });
        }
    }
}

const vehicle_sim::dynamics::OffroadBallistics2D& CombatManager::getBallistics() const noexcept
{
    return ballistics_;
}

vehicle_sim::dynamics::OffroadBallistics2D& CombatManager::getBallistics() noexcept
{
    return ballistics_;
}

double CombatManager::getRocketCooldown() const noexcept
{
    return rocketTimer_;
}

double CombatManager::getMortarCooldown() const noexcept
{
    return mortarTimer_;
}

bool CombatManager::canFireRocket() const noexcept
{
    return rocketTimer_ <= 0.0;
}

bool CombatManager::canFireMortar() const noexcept
{
    return mortarTimer_ <= 0.0;
}

std::size_t CombatManager::getShockwaveCount() const noexcept
{
    std::size_t count { 0 };
    for (const auto& sw : shockwaves_) {
        if (sw.active) {
            ++count;
        }
    }
    return count;
}

const CombatManagerConfig& CombatManager::getConfig() const noexcept
{
    return config_;
}

} // namespace montsoft_offroad
