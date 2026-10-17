#include "apps/MotionLab/TireSmokeParticleBuffer.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

TireSmokeParticleBuffer::TireSmokeParticleBuffer() noexcept
{
    mParticles.resize(kMaxParticles);
    clear();
}

void TireSmokeParticleBuffer::clear() noexcept
{
    for (auto& particle : mParticles) {
        particle.age = 999.0; // Mark expired
        particle.radius = 0.20;
    }
    mWriteIndex = 0;
    mActiveCount = 0;
}

void TireSmokeParticleBuffer::emit(const vehicle_sim::math::Vec2& origin,
                                   const vehicle_sim::math::Vec2& slipVelocity,
                                   double intensity) noexcept
{
    const double safeIntensity { std::clamp(intensity, 0.1, 1.0) };

    SmokeParticle& p { mParticles[mWriteIndex] };
    p.position = origin;
    // Initial drift along slip direction + slight dispersal
    p.velocity = slipVelocity * 0.35;
    p.radius = 0.15 + 0.15 * safeIntensity;
    p.age = 0.0;
    p.maxLifetime = 0.80 + 0.70 * safeIntensity;
    p.initialAlpha = 0.50 + 0.40 * safeIntensity;

    mWriteIndex = (mWriteIndex + 1) % kMaxParticles;
    if (mActiveCount < kMaxParticles) {
        ++mActiveCount;
    }
}

void TireSmokeParticleBuffer::update(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    std::size_t aliveCount { 0 };
    for (auto& particle : mParticles) {
        if (particle.age >= particle.maxLifetime) {
            continue;
        }

        particle.age += dt;
        if (particle.age < particle.maxLifetime) {
            ++aliveCount;
            // Linear position advancement with aerodynamic drag decay
            particle.position = particle.position + particle.velocity * dt;
            constexpr double dragFactor { 0.92 };
            particle.velocity = particle.velocity * dragFactor;

            // Radial thermal expansion
            constexpr double expansionRate { 0.45 };
            particle.radius += expansionRate * dt;
        }
    }
    mActiveCount = aliveCount;
}

const std::vector<SmokeParticle>& TireSmokeParticleBuffer::getParticles() const noexcept
{
    return mParticles;
}

std::size_t TireSmokeParticleBuffer::getActiveCount() const noexcept
{
    return mActiveCount;
}

} // namespace motion_lab
