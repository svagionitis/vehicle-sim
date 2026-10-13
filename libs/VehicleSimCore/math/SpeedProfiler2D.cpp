#include "libs/VehicleSimCore/math/SpeedProfiler2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

SpeedProfiler2D::SpeedProfiler2D(const SpeedProfileLimits& limits) noexcept
    : mLimits { limits }
{
}

void SpeedProfiler2D::setLimits(const SpeedProfileLimits& limits) noexcept
{
    mLimits = limits;
}

const SpeedProfileLimits& SpeedProfiler2D::getLimits() const noexcept
{
    return mLimits;
}

void SpeedProfiler2D::clear() noexcept
{
    mMilestones.clear();
    mTotalLength = 0.0;
    mIsClosed = false;
}

const std::vector<SpeedMilestone>& SpeedProfiler2D::getMilestones() const noexcept
{
    return mMilestones;
}

bool SpeedProfiler2D::buildProfile(const Spline2D& spline) noexcept
{
    clear();
    const double totalLength { spline.getTotalLength() };
    if (spline.getPointCount() < 2 || totalLength <= 0.05) {
        return false;
    }

    mTotalLength = totalLength;
    mIsClosed = spline.isClosed();

    const double step { std::clamp(mLimits.sampleStep, 0.1, 50.0) };
    const auto count = static_cast<std::size_t>(std::max(2.0, std::ceil(mTotalLength / step) + 1.0));
    mMilestones.reserve(count);

    const double cruiseSpeed { std::max(0.5, mLimits.maxCruiseSpeed) };
    const double latAccel { std::max(0.1, mLimits.maxLatAccel) };

    for (std::size_t i { 0 }; i < count; ++i) {
        const double dist { std::min(
            mTotalLength, static_cast<double>(i) * (mTotalLength / static_cast<double>(count - 1))) };
        const SplineSample2D sample { spline.evaluateAtDistance(dist) };

        const double absCurv { std::abs(sample.curvature) };
        double vMax { cruiseSpeed };
        if (absCurv > 1e-4) {
            const double cornerLimit { std::sqrt(latAccel / absCurv) };
            vMax = std::min(cruiseSpeed, cornerLimit);
        }

        SpeedMilestone milestone {};
        milestone.distance = dist;
        milestone.position = sample.point;
        milestone.curvature = sample.curvature;
        milestone.targetSpeed = vMax;
        mMilestones.push_back(milestone);
    }

    if (mMilestones.size() < 2) {
        return false;
    }

    const double maxBrake { std::max(0.1, mLimits.maxBrakingDecel) };
    const double maxAccel { std::max(0.1, mLimits.maxLongAccel) };
    const std::size_t n { mMilestones.size() };
    const std::size_t passes { mIsClosed ? 2U : 1U };

    // Backward deceleration sweeps
    for (std::size_t p { 0 }; p < passes; ++p) {
        for (std::size_t i { n - 1 }; i > 0; --i) {
            const double ds { mMilestones[i].distance - mMilestones[i - 1].distance };
            const double nextV { mMilestones[i].targetSpeed };
            const double maxAllowed { std::sqrt((nextV * nextV) + (2.0 * maxBrake * ds)) };
            if (mMilestones[i - 1].targetSpeed > maxAllowed) {
                mMilestones[i - 1].targetSpeed = maxAllowed;
            }
        }
        if (mIsClosed) {
            const double ds { mTotalLength - mMilestones[n - 1].distance + mMilestones[0].distance };
            const double nextV { mMilestones[0].targetSpeed };
            const double maxAllowed { std::sqrt((nextV * nextV) + (2.0 * maxBrake * ds)) };
            if (mMilestones[n - 1].targetSpeed > maxAllowed) {
                mMilestones[n - 1].targetSpeed = maxAllowed;
            }
        }
    }

    // Forward traction acceleration sweeps
    for (std::size_t p { 0 }; p < passes; ++p) {
        for (std::size_t i { 0 }; i < n - 1; ++i) {
            const double ds { mMilestones[i + 1].distance - mMilestones[i].distance };
            const double currV { mMilestones[i].targetSpeed };
            const double maxAllowed { std::sqrt((currV * currV) + (2.0 * maxAccel * ds)) };
            if (mMilestones[i + 1].targetSpeed > maxAllowed) {
                mMilestones[i + 1].targetSpeed = maxAllowed;
            }
        }
        if (mIsClosed) {
            const double ds { mTotalLength - mMilestones[n - 1].distance + mMilestones[0].distance };
            const double currV { mMilestones[n - 1].targetSpeed };
            const double maxAllowed { std::sqrt((currV * currV) + (2.0 * maxAccel * ds)) };
            if (mMilestones[0].targetSpeed > maxAllowed) {
                mMilestones[0].targetSpeed = maxAllowed;
            }
        }
    }

    return true;
}

double SpeedProfiler2D::getSpeedAt(double distance) const noexcept
{
    if (mMilestones.empty()) {
        return 0.0;
    }
    if (mMilestones.size() == 1 || mTotalLength <= 1e-4) {
        return mMilestones.front().targetSpeed;
    }

    double d { distance };
    if (mIsClosed) {
        d = std::fmod(d, mTotalLength);
        if (d < 0.0) {
            d += mTotalLength;
        }
    } else {
        d = std::clamp(d, 0.0, mTotalLength);
    }

    auto it = std::lower_bound(mMilestones.begin(), mMilestones.end(), d,
        [](const SpeedMilestone& m, double val) noexcept { return m.distance < val; });

    if (it == mMilestones.begin()) {
        return mMilestones.front().targetSpeed;
    }
    if (it == mMilestones.end()) {
        return mMilestones.back().targetSpeed;
    }

    const auto prev = it - 1;
    const double span { it->distance - prev->distance };
    if (span < 1e-9) {
        return it->targetSpeed;
    }

    const double t { (d - prev->distance) / span };
    return prev->targetSpeed + (t * (it->targetSpeed - prev->targetSpeed));
}

} // namespace vehicle_sim::math
