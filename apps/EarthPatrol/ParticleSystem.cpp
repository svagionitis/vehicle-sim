#include "apps/EarthPatrol/ParticleSystem.h"

#include <algorithm>
#include <cmath>

namespace earth_patrol {

namespace {
    // Deterministic pseudo-random number generator for particle velocities
    uint32_t gRndState { 0x12345678u };

    double nextRandom() noexcept
    {
        gRndState = (gRndState * 1664525u) + 1013904223u;
        return static_cast<double>(gRndState & 0x00FFFFFFu) / 16777216.0;
    }

    double nextRandomRange(double minVal, double maxVal) noexcept
    {
        return minVal + (nextRandom() * (maxVal - minVal));
    }
} // namespace

ParticleSystem::ParticleSystem() noexcept
{
    reset();
}

void ParticleSystem::reset() noexcept
{
    for (auto& p : pool_) {
        p.active = false;
        p.lifetime = 0.0;
    }
    nextSlot_ = 0;
    activeCount_ = 0;
}

int ParticleSystem::getActiveCount() const noexcept
{
    return activeCount_;
}

Particle& ParticleSystem::allocateParticle() noexcept
{
    auto& slot { pool_[nextSlot_] };
    if (!slot.active) {
        activeCount_++;
    }
    nextSlot_ = (nextSlot_ + 1) % kMaxPoolSize;
    return slot;
}

void ParticleSystem::update(double dt) noexcept
{
    activeCount_ = 0;

    for (auto& p : pool_) {
        if (!p.active) {
            continue;
        }

        p.lifetime -= dt;
        if (p.lifetime <= 0.0) {
            p.active = false;
            continue;
        }

        activeCount_++;

        // Apply physics integration
        p.position += p.velocity * dt;

        if (p.type == ParticleType::Dust) {
            // Drag deceleration and slight gravity
            p.velocity *= std::max(0.0, 1.0 - (2.5 * dt));
            p.velocity.y -= 1.8 * dt;
        } else if (p.type == ParticleType::Spark) {
            // Stronger downward gravity and drag
            p.velocity *= std::max(0.0, 1.0 - (1.2 * dt));
            p.velocity.y -= 7.5 * dt;
        } else if (p.type == ParticleType::Debris) {
            // Earth ballistic gravity
            p.velocity.y -= 9.81 * dt;
        } else if (p.type == ParticleType::Smoke) {
            // Buoyant upward drift
            p.velocity.y += 0.5 * dt;
            p.velocity.x *= std::max(0.0, 1.0 - (1.5 * dt));
        }
    }
}

void ParticleSystem::emitTireDust(const vehicle_sim::math::Vec2& pos, int biomeIndex, int count) noexcept
{
    SDL_Color dustColor { 140, 140, 145, 200 };
    if (biomeIndex == 1) { // Desert sand
        dustColor = SDL_Color { 210, 140, 70, 220 };
    } else if (biomeIndex == 2) { // Alpine loam
        dustColor = SDL_Color { 90, 85, 80, 210 };
    }

    for (int i { 0 }; i < count; ++i) {
        auto& p { allocateParticle() };
        p.active = true;
        p.type = ParticleType::Dust;
        p.position = pos + vehicle_sim::math::Vec2 { nextRandomRange(-0.15, 0.15), nextRandomRange(0.0, 0.1) };
        p.velocity = vehicle_sim::math::Vec2 { nextRandomRange(-2.8, -0.6), nextRandomRange(0.4, 2.0) };
        p.maxLifetime = nextRandomRange(0.3, 0.6);
        p.lifetime = p.maxLifetime;
        p.size = nextRandomRange(0.12, 0.22);
        p.color = dustColor;
    }
}

void ParticleSystem::emitThrusterSparks(const vehicle_sim::math::Vec2& pos, int count) noexcept
{
    for (int i { 0 }; i < count; ++i) {
        auto& p { allocateParticle() };
        p.active = true;
        p.type = ParticleType::Spark;
        p.position = pos + vehicle_sim::math::Vec2 { nextRandomRange(-0.1, 0.1), nextRandomRange(-0.05, 0.05) };
        p.velocity = vehicle_sim::math::Vec2 { nextRandomRange(-3.5, 0.5), nextRandomRange(-6.5, -2.5) };
        p.maxLifetime = nextRandomRange(0.15, 0.35);
        p.lifetime = p.maxLifetime;
        p.size = nextRandomRange(0.08, 0.14);
        p.color = SDL_Color { 255, static_cast<Uint8>(nextRandomRange(140.0, 220.0)), 30, 255 };
    }
}

void ParticleSystem::emitLandingDust(const vehicle_sim::math::Vec2& pos, int biomeIndex) noexcept
{
    SDL_Color dustColor { 150, 150, 155, 230 };
    if (biomeIndex == 1) {
        dustColor = SDL_Color { 215, 145, 75, 230 };
    } else if (biomeIndex == 2) {
        dustColor = SDL_Color { 100, 95, 90, 220 };
    }

    // Emit spray left and right
    for (int i { 0 }; i < 10; ++i) {
        auto& p { allocateParticle() };
        p.active = true;
        p.type = ParticleType::Dust;
        p.position = pos;
        const double dir { (i % 2 == 0) ? -1.0 : 1.0 };
        p.velocity = vehicle_sim::math::Vec2 {
            dir * nextRandomRange(1.5, 4.5),
            nextRandomRange(0.8, 3.2)
        };
        p.maxLifetime = nextRandomRange(0.4, 0.8);
        p.lifetime = p.maxLifetime;
        p.size = nextRandomRange(0.15, 0.30);
        p.color = dustColor;
    }
}

void ParticleSystem::emitCrashExplosion(const vehicle_sim::math::Vec2& pos) noexcept
{
    // Fire sparks
    for (int i { 0 }; i < 24; ++i) {
        auto& p { allocateParticle() };
        p.active = true;
        p.type = ParticleType::Spark;
        p.position = pos;
        const double angle { nextRandomRange(0.0, 6.2831853) };
        const double speed { nextRandomRange(3.0, 9.0) };
        p.velocity = vehicle_sim::math::Vec2 { std::cos(angle) * speed, std::sin(angle) * speed };
        p.maxLifetime = nextRandomRange(0.4, 1.0);
        p.lifetime = p.maxLifetime;
        p.size = nextRandomRange(0.12, 0.24);
        p.color = SDL_Color { 255, static_cast<Uint8>(nextRandomRange(100.0, 240.0)), 20, 255 };
    }

    // Heavy debris shrapnel
    for (int i { 0 }; i < 16; ++i) {
        auto& p { allocateParticle() };
        p.active = true;
        p.type = ParticleType::Debris;
        p.position = pos;
        p.velocity = vehicle_sim::math::Vec2 {
            nextRandomRange(-6.0, 6.0),
            nextRandomRange(2.0, 8.5)
        };
        p.maxLifetime = nextRandomRange(0.8, 1.4);
        p.lifetime = p.maxLifetime;
        p.size = nextRandomRange(0.20, 0.35);
        p.color = SDL_Color { 70, 75, 85, 255 };
    }
}

void ParticleSystem::render(SDL_Renderer* renderer, const Camera2D& camera) const noexcept
{
    if (renderer == nullptr || activeCount_ == 0) {
        return;
    }

    const double minX { camera.getVisibleMinX() - 1.0 };
    const double maxX { camera.getVisibleMaxX() + 1.0 };
    const double ppm { camera.getConfig().pixelsPerMeter };

    for (const auto& p : pool_) {
        if (!p.active || p.position.x < minX || p.position.x > maxX) {
            continue;
        }

        const auto screenPos { camera.worldToScreen(p.position) };
        const int pxSize { std::max(2, static_cast<int>(p.size * ppm)) };
        const int halfSize { pxSize / 2 };

        // Alpha fade out over lifetime
        const double alphaFrac { std::clamp(p.lifetime / p.maxLifetime, 0.0, 1.0) };
        const Uint8 alpha { static_cast<Uint8>(static_cast<double>(p.color.a) * alphaFrac) };

        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer, p.color.r, p.color.g, p.color.b, alpha);

        SDL_Rect rect { static_cast<int>(screenPos.x) - halfSize,
            static_cast<int>(screenPos.y) - halfSize, pxSize, pxSize };
        SDL_RenderFillRect(renderer, &rect);
    }
}

} // namespace earth_patrol
