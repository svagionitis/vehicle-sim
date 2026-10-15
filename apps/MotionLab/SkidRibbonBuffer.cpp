#include "apps/MotionLab/SkidRibbonBuffer.h"

#include <algorithm>

namespace motion_lab {

SkidRibbonBuffer::SkidRibbonBuffer(double lifetime) noexcept
    : mLifetime { std::max(0.1, lifetime) }
{
    mSegments.reserve(kMaxSegments);
}

void SkidRibbonBuffer::clear() noexcept
{
    mSegments.clear();
}

void SkidRibbonBuffer::setLifetime(double lifetime) noexcept
{
    mLifetime = std::max(0.1, lifetime);
}

double SkidRibbonBuffer::getLifetime() const noexcept
{
    return mLifetime;
}

const std::vector<SkidSegment>& SkidRibbonBuffer::getSegments() const noexcept
{
    return mSegments;
}

void SkidRibbonBuffer::update(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    for (auto& segment : mSegments) {
        segment.age += dt;
    }

    mSegments.erase(
        std::remove_if(mSegments.begin(), mSegments.end(),
            [this](const SkidSegment& seg) noexcept {
                return seg.age >= mLifetime;
            }),
        mSegments.end());
}

void SkidRibbonBuffer::addSegment(const vehicle_sim::math::Vec2& from,
                                 const vehicle_sim::math::Vec2& to) noexcept
{
    const double lengthSq { (to - from).lengthSquared() };
    if (lengthSq < 1e-6) {
        return;
    }

    if (mSegments.size() >= kMaxSegments) {
        mSegments.erase(mSegments.begin());
    }

    mSegments.push_back(SkidSegment { from, to, 0.0 });
}

} // namespace motion_lab
