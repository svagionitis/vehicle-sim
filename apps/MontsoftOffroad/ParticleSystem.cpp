#include "apps/MontsoftOffroad/ParticleSystem.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;

ParticleSystem::ParticleSystem() noexcept
{
    reset();
}

double ParticleSystem::nextRandomDouble() noexcept
{
    rngState_ = rngState_ * 1664525U + 1013904223U;
    return static_cast<double>(rngState_ & 0x00FFFFFFU) / 16777216.0;
}

std::size_t ParticleSystem::allocateSlot() noexcept
{
    const std::size_t slot { nextSlot_ };
    nextSlot_ = (nextSlot_ + 1U) % kMaxParticles;

    if (!pool_[slot].active) {
        ++activeCount_;
    }
    return slot;
}

void ParticleSystem::reset() noexcept
{
    for (auto& p : pool_) {
        p.active = false;
        p.age = 0.0;
    }
    nextSlot_ = 0;
    activeCount_ = 0;
    rngState_ = 19880724U;
}

std::size_t ParticleSystem::getActiveCount() const noexcept
{
    return activeCount_;
}

ColorRgba ParticleSystem::getBiomeColor(std::size_t stageIndex, ParticleType type) noexcept
{
    if (type == ParticleType::ThrusterSpark) {
        const double r { nextRandomDouble() };
        if (r < 0.5) {
            return ColorRgba{ 255, 255, 255, 255 }; // Incandescent White
        }
        if (r < 0.8) {
            return ColorRgba{ 255, 255, 85, 255 };  // Hot Yellow
        }
        return ColorRgba{ 85, 255, 255, 255 };      // Cyan Plasma
    }

    // Biome-specific dust coloring
    switch (stageIndex % 8U) {
    case 0U: // Mojave Dunes - Warm Desert Sand
        return (nextRandomDouble() < 0.6)
            ? ColorRgba{ 255, 255, 85, 255 }  // Sand Yellow
            : ColorRgba{ 255, 255, 255, 255 }; // Sunlit Dust
    case 1U: // Red Rock Arroyo - Copper Sandstone
        return (nextRandomDouble() < 0.7)
            ? ColorRgba{ 255, 85, 85, 255 }   // Red Sand
            : ColorRgba{ 255, 85, 255, 255 };  // Terracotta Dust
    case 2U: // Great Salt Flats - White Salt Crust
        return (nextRandomDouble() < 0.8)
            ? ColorRgba{ 255, 255, 255, 255 } // Gleaming White Salt
            : ColorRgba{ 85, 255, 255, 255 };  // Salt Glint Cyan
    case 3U: // Muddy Quagmire - Silt & Marsh Mud
        return (nextRandomDouble() < 0.6)
            ? ColorRgba{ 85, 255, 85, 255 }   // Mossy Mud
            : ColorRgba{ 255, 85, 85, 255 };   // Dark Silt
    case 4U: // Volcanic Badlands - Basalt Soot & Embers
        return (nextRandomDouble() < 0.5)
            ? ColorRgba{ 255, 85, 85, 255 }   // Glowing Ember
            : ColorRgba{ 85, 85, 85, 255 };    // Dark Basalt Ash
    case 5U: // Scrap Canyon - Rust & Metal Shavings
        return (nextRandomDouble() < 0.6)
            ? ColorRgba{ 255, 85, 85, 255 }   // Rust Orange/Red
            : ColorRgba{ 255, 255, 255, 255 }; // Metal Glint
    case 6U: // Alpine Scree - Gravel & Slate
        return (nextRandomDouble() < 0.7)
            ? ColorRgba{ 85, 255, 255, 255 }  // Slate Cyan
            : ColorRgba{ 255, 255, 255, 255 }; // Rock Chips
    case 7U: // Secret Fortress - Concrete & Smog
    default:
        return (nextRandomDouble() < 0.5)
            ? ColorRgba{ 255, 255, 255, 255 } // Concrete Powder
            : ColorRgba{ 85, 255, 255, 255 };  // Electric Arc Dust
    }
}

void ParticleSystem::emitRoosterTail(
    const Vec2& contactWorldPos,
    const Vec2& chassisVelocity,
    double soilSinkage,
    std::size_t stageIndex
) noexcept
{
    const double speed { std::abs(chassisVelocity.x) };
    if (speed < 0.5) {
        return;
    }

    const std::size_t count { std::min<std::size_t>(4U, static_cast<std::size_t>(1.0 + speed * 0.2 + soilSinkage * 10.0)) };
    for (std::size_t i { 0 }; i < count; ++i) {
        const std::size_t slot { allocateSlot() };
        Particle& p { pool_[slot] };
        p.active = true;
        p.type = ParticleType::SandRoosterTail;
        p.age = 0.0;
        p.maxLifetime = 0.35 + nextRandomDouble() * 0.30;
        p.sizePx = 1U;

        const double jitterX { (nextRandomDouble() - 0.5) * 0.2 };
        const double jitterY { nextRandomDouble() * 0.1 };
        p.position = contactWorldPos + Vec2{ jitterX, jitterY };

        // Parabolic trajectory directed rearward relative to travel direction
        const double dir { chassisVelocity.x >= 0.0 ? -1.0 : 1.0 };
        const double vx { dir * speed * (0.35 + nextRandomDouble() * 0.40) };
        const double vy { (1.2 + nextRandomDouble() * 2.2) * std::sqrt(std::max(0.1, speed * 0.35)) };
        p.velocity = Vec2{ vx, vy };
        p.baseColor = getBiomeColor(stageIndex, ParticleType::SandRoosterTail);
    }
}

void ParticleSystem::emitLandingDust(
    const Vec2& contactWorldPos,
    double impactVelocity,
    std::size_t stageIndex
) noexcept
{
    const double impact { std::abs(impactVelocity) };
    if (impact < 1.5) {
        return;
    }

    const std::size_t count { std::min<std::size_t>(16U, static_cast<std::size_t>(6U + impact * 2.5)) };
    for (std::size_t i { 0 }; i < count; ++i) {
        const std::size_t slot { allocateSlot() };
        Particle& p { pool_[slot] };
        p.active = true;
        p.type = ParticleType::LandingDustPlume;
        p.age = 0.0;
        p.maxLifetime = 0.45 + nextRandomDouble() * 0.35;
        p.sizePx = 1U;

        const double side { (i % 2U == 0U) ? -1.0 : 1.0 };
        const double jitterX { (nextRandomDouble() - 0.5) * 0.3 };
        p.position = contactWorldPos + Vec2{ jitterX, 0.05 };

        // Radial billowing dispersion
        const double vx { side * (1.5 + nextRandomDouble() * 3.5) };
        const double vy { 0.6 + nextRandomDouble() * 1.8 };
        p.velocity = Vec2{ vx, vy };
        p.baseColor = getBiomeColor(stageIndex, ParticleType::LandingDustPlume);
    }
}

void ParticleSystem::emitThrusterSparks(
    const Vec2& thrusterWorldPos,
    const Vec2& chassisVelocity
) noexcept
{
    constexpr std::size_t kCount { 6U };
    for (std::size_t i { 0 }; i < kCount; ++i) {
        const std::size_t slot { allocateSlot() };
        Particle& p { pool_[slot] };
        p.active = true;
        p.type = ParticleType::ThrusterSpark;
        p.age = 0.0;
        p.maxLifetime = 0.15 + nextRandomDouble() * 0.15;
        p.sizePx = 1U;

        const double jitterX { (nextRandomDouble() - 0.5) * 0.4 };
        p.position = thrusterWorldPos + Vec2{ jitterX, -0.1 };

        // Fast downward exhaust blast with lateral spread
        const double vx { chassisVelocity.x * 0.3 + (nextRandomDouble() - 0.5) * 3.5 };
        const double vy { -(5.5 + nextRandomDouble() * 4.5) };
        p.velocity = Vec2{ vx, vy };
        p.baseColor = getBiomeColor(0U, ParticleType::ThrusterSpark);
    }
}

void ParticleSystem::emitRocketExhaust(
    const Vec2& nozzleWorldPos,
    const Vec2& rocketVelocity,
    double heading
) noexcept
{
    constexpr std::size_t kCount { 2U };
    const double cosH { std::cos(heading) };
    const double sinH { std::sin(heading) };

    for (std::size_t i { 0 }; i < kCount; ++i) {
        const std::size_t slot { allocateSlot() };
        Particle& p { pool_[slot] };
        p.active = true;
        p.type = ParticleType::RocketExhaust;
        p.age = 0.0;
        p.maxLifetime = 0.12 + nextRandomDouble() * 0.12;
        p.sizePx = 1U;

        const double jitter { (nextRandomDouble() - 0.5) * 0.15 };
        p.position = nozzleWorldPos + Vec2{ -sinH * jitter, cosH * jitter };

        const double exhaustSpeed { 4.0 + nextRandomDouble() * 5.0 };
        const double vx { rocketVelocity.x * 0.15 - cosH * exhaustSpeed + (nextRandomDouble() - 0.5) * 1.5 };
        const double vy { rocketVelocity.y * 0.15 - sinH * exhaustSpeed + (nextRandomDouble() - 0.5) * 1.5 };
        p.velocity = Vec2{ vx, vy };

        const double r { nextRandomDouble() };
        if (r < 0.45) {
            p.baseColor = ColorRgba{ 255, 255, 255, 255 }; // Hot white
        } else if (r < 0.75) {
            p.baseColor = ColorRgba{ 255, 255, 85, 255 };  // Hot yellow
        } else {
            p.baseColor = ColorRgba{ 255, 85, 85, 255 };   // Flame red
        }
    }
}

void ParticleSystem::emitMortarTrail(
    const Vec2& shellWorldPos,
    const Vec2& shellVelocity
) noexcept
{
    const std::size_t slot { allocateSlot() };
    Particle& p { pool_[slot] };
    p.active = true;
    p.type = ParticleType::RocketExhaust;
    p.age = 0.0;
    p.maxLifetime = 0.18 + nextRandomDouble() * 0.10;
    p.sizePx = 1U;

    p.position = shellWorldPos;
    p.velocity = shellVelocity * 0.08 + Vec2{ (nextRandomDouble() - 0.5) * 0.8, (nextRandomDouble() - 0.5) * 0.8 };
    p.baseColor = (nextRandomDouble() < 0.6)
        ? ColorRgba{ 255, 255, 255, 255 }
        : ColorRgba{ 85, 255, 255, 255 };
}

void ParticleSystem::emitExplosionShrapnel(
    const Vec2& epicenter,
    double radius,
    double intensity,
    std::size_t stageIndex
) noexcept
{
    const std::size_t count { std::min<std::size_t>(28U, static_cast<std::size_t>(16U + radius * 3.0 * intensity)) };
    for (std::size_t i { 0 }; i < count; ++i) {
        const std::size_t slot { allocateSlot() };
        Particle& p { pool_[slot] };
        p.active = true;
        p.type = ParticleType::ExplosionShrapnel;
        p.age = 0.0;
        p.maxLifetime = 0.25 + nextRandomDouble() * 0.35;
        p.sizePx = (nextRandomDouble() < 0.4) ? 2U : 1U;

        const double angle { (2.0 * 3.14159265358979323846 / static_cast<double>(count)) * static_cast<double>(i) + (nextRandomDouble() - 0.5) * 0.4 };
        const double speed { (8.0 + nextRandomDouble() * 16.0) * std::sqrt(std::max(0.5, intensity)) };
        const double jitterDist { nextRandomDouble() * 0.2 };

        p.position = epicenter + Vec2{ std::cos(angle) * jitterDist, std::sin(angle) * jitterDist };
        p.velocity = Vec2{ std::cos(angle) * speed, std::sin(angle) * speed + 2.0 };

        const double cr { nextRandomDouble() };
        if (cr < 0.35) {
            p.baseColor = ColorRgba{ 255, 255, 255, 255 }; // Hot core white
        } else if (cr < 0.70) {
            p.baseColor = ColorRgba{ 255, 255, 85, 255 };  // Yellow fireball
        } else {
            p.baseColor = getBiomeColor(stageIndex, ParticleType::ExplosionShrapnel);
        }
    }
}

void ParticleSystem::update(double dt) noexcept
{
    for (auto& p : pool_) {
        if (!p.active) {
            continue;
        }

        p.age += dt;
        if (p.age >= p.maxLifetime) {
            p.active = false;
            if (activeCount_ > 0U) {
                --activeCount_;
            }
            continue;
        }

        // Kinematic integration based on particle category
        double gravityY { -9.81 };
        double dragCoeff { 0.5 };

        if (p.type == ParticleType::LandingDustPlume) {
            gravityY = -0.4;   // Floats gently
            dragCoeff = 2.4;  // Strong air resistance halts lateral speed
            if (p.age > p.maxLifetime * 0.4) {
                p.sizePx = 2U; // Expands as it billows
            }
            if (p.age > p.maxLifetime * 0.75) {
                p.sizePx = 3U;
            }
        } else if (p.type == ParticleType::ThrusterSpark) {
            gravityY = -14.0;
            dragCoeff = 0.1;
        } else if (p.type == ParticleType::RocketExhaust) {
            gravityY = -0.2;
            dragCoeff = 3.5;
            if (p.age > p.maxLifetime * 0.5) {
                p.sizePx = 2U;
            }
        } else if (p.type == ParticleType::ExplosionShrapnel) {
            gravityY = -8.5;
            dragCoeff = 1.2;
            if (p.age > p.maxLifetime * 0.6) {
                p.sizePx = 2U;
            }
        }

        // Air drag force
        const double speed { std::sqrt(p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y) };
        const Vec2 dragForce { -dragCoeff * speed * p.velocity.x, -dragCoeff * speed * p.velocity.y };

        p.velocity.x += dragForce.x * dt;
        p.velocity.y += (gravityY + dragForce.y) * dt;

        p.position.x += p.velocity.x * dt;
        p.position.y += p.velocity.y * dt;
    }
}

void ParticleSystem::render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    for (const auto& p : pool_) {
        if (!p.active) {
            continue;
        }

        const Vec2 screenPos { camera.worldToScreen(p.position) };
        const int sx { static_cast<int>(std::round(screenPos.x)) };
        const int sy { static_cast<int>(std::round(screenPos.y)) };

        if (sx < 0 || sx >= CgaPaletteRenderer::kNativeWidth || sy < 0 || sy >= CgaPaletteRenderer::kNativeHeight) {
            continue;
        }

        if (p.sizePx <= 1U) {
            renderer.setPixel(sx, sy, p.baseColor);
        } else if (p.sizePx == 2U) {
            renderer.fillRect(sx, sy, 2, 2, p.baseColor);
        } else {
            // 3x3 stippled dither cross
            renderer.setPixel(sx + 1, sy, p.baseColor);
            renderer.setPixel(sx, sy + 1, p.baseColor);
            renderer.setPixel(sx + 1, sy + 1, p.baseColor);
            renderer.setPixel(sx + 2, sy + 1, p.baseColor);
            renderer.setPixel(sx + 1, sy + 2, p.baseColor);
        }
    }
}

} // namespace montsoft_offroad
