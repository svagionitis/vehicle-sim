#include "libs/VehicleSimCore/math/SlewRateLimiter.h"

#include <algorithm>

namespace vehicle_sim::math {

SlewRateLimiter::SlewRateLimiter(double maxRate) noexcept
    : mRisingRate { (maxRate > 0.0) ? maxRate : 0.0 }
    , mFallingRate { (maxRate > 0.0) ? maxRate : 0.0 }
{
}

SlewRateLimiter::SlewRateLimiter(double risingRate, double fallingRate) noexcept
    : mRisingRate { (risingRate > 0.0) ? risingRate : 0.0 }
    , mFallingRate { (fallingRate > 0.0) ? fallingRate : 0.0 }
{
}

double SlewRateLimiter::update(double target, double dt) noexcept
{
    if (dt <= 0.0) {
        return mCurrentVal;
    }

    if (!mInitialized) {
        mCurrentVal = target;
        mInitialized = true;
        return mCurrentVal;
    }

    const double maxRise { mRisingRate * dt };
    const double maxFall { mFallingRate * dt };

    if (target > mCurrentVal) {
        mCurrentVal = (mRisingRate > 0.0) ? std::min(mCurrentVal + maxRise, target) : target;
    } else if (target < mCurrentVal) {
        mCurrentVal = (mFallingRate > 0.0) ? std::max(mCurrentVal - maxFall, target) : target;
    } else {
        mCurrentVal = target;
    }

    return mCurrentVal;
}

void SlewRateLimiter::reset() noexcept
{
    mCurrentVal = 0.0;
    mInitialized = false;
}

void SlewRateLimiter::reset(double initialVal) noexcept
{
    mCurrentVal = initialVal;
    mInitialized = true;
}

void SlewRateLimiter::setRateLimit(double maxRate) noexcept
{
    mRisingRate = (maxRate > 0.0) ? maxRate : 0.0;
    mFallingRate = (maxRate > 0.0) ? maxRate : 0.0;
}

void SlewRateLimiter::setRateLimits(double risingRate, double fallingRate) noexcept
{
    mRisingRate = (risingRate > 0.0) ? risingRate : 0.0;
    mFallingRate = (fallingRate > 0.0) ? fallingRate : 0.0;
}

} // namespace vehicle_sim::math
