#include "libs/VehicleSimCore/math/LowPassFilter.h"

#include <algorithm>

namespace vehicle_sim::math {

LowPassFilter::LowPassFilter(double cutoffHz) noexcept
    : mCutoffHz { (cutoffHz > 0.0) ? cutoffHz : 0.0 }
{
}

double LowPassFilter::update(double input, double dt) noexcept
{
    if (dt <= 0.0) {
        return mValue;
    }

    if (!mInitialized) {
        mValue = input;
        mInitialized = true;
        return mValue;
    }

    if (mCutoffHz <= 0.0) {
        return mValue;
    }

    // Discrete exponential smoothing factor:
    // alpha = (2 * pi * fc * dt) / (1 + 2 * pi * fc * dt)
    const double omegaDt { kTwoPi * mCutoffHz * dt };
    const double alpha { std::clamp(omegaDt / (1.0 + omegaDt), 0.0, 1.0) };

    mValue += alpha * (input - mValue);
    return mValue;
}

void LowPassFilter::reset() noexcept
{
    mValue = 0.0;
    mInitialized = false;
}

void LowPassFilter::reset(double initialVal) noexcept
{
    mValue = initialVal;
    mInitialized = true;
}

void LowPassFilter::setCutoff(double cutoffHz) noexcept
{
    mCutoffHz = (cutoffHz > 0.0) ? cutoffHz : 0.0;
}

} // namespace vehicle_sim::math
