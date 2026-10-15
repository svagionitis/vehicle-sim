#include "apps/MotionLab/TrailParticleBuffer.h"

#include <algorithm>

namespace motion_lab {

TrailParticleBuffer::TrailParticleBuffer(double lifetime) noexcept
    : mLifetime { std::max(0.0, lifetime) }
{
    mDots.reserve(kMaxDots);
}

void TrailParticleBuffer::clear() noexcept
{
    mDots.clear();
    mTrailClock = 0.0;
    mHasLastPosition = false;
}

void TrailParticleBuffer::setLifetime(double lifetime) noexcept
{
    mLifetime = std::max(0.0, lifetime);
    if (mLifetime <= 0.0) {
        clear();
    } else {
        mDots.erase(
            std::remove_if(mDots.begin(), mDots.end(),
                [this](const TrailDot& dot) noexcept {
                    return dot.age >= mLifetime;
                }),
            mDots.end());
    }
}

double TrailParticleBuffer::getLifetime() const noexcept
{
    return mLifetime;
}

const std::vector<TrailDot>& TrailParticleBuffer::getDots() const noexcept
{
    return mDots;
}

void TrailParticleBuffer::update(const vehicle_sim::math::Vec2& position, double dt) noexcept
{
    if (mLifetime <= 0.0 || dt <= 0.0) {
        return;
    }

    // Age existing dots and purge expired
    for (auto& dot : mDots) {
        dot.age += dt;
    }

    mDots.erase(
        std::remove_if(mDots.begin(), mDots.end(),
            [this](const TrailDot& dot) noexcept {
                return dot.age >= mLifetime;
            }),
        mDots.end());

    // Fixed 30 Hz emission clock (1 / 30 = 0.0333s)
    mTrailClock += dt;
    constexpr double kEmissionPeriod { 1.0 / 30.0 };

    if (mTrailClock >= kEmissionPeriod) {
        mTrailClock = std::fmod(mTrailClock, kEmissionPeriod);

        // Only emit if displaced more than 0.04m from last position
        if (!mHasLastPosition || (position - mLastPosition).lengthSquared() >= (0.04 * 0.04)) {
            mLastPosition = position;
            mHasLastPosition = true;

            if (mDots.size() >= kMaxDots) {
                mDots.erase(mDots.begin());
            }

            mDots.push_back(TrailDot { position, 0.0 });
        }
    }
}

} // namespace motion_lab
