#include "FluidField2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::physics {

FluidField2D::FluidField2D(const FluidFieldConfig& config) noexcept
    : mConfig { config }
{
}

FluidSample2D FluidField2D::sample(const math::Vec2& position, double time) const noexcept
{
    FluidSample2D result { };
    result.velocity = getVelocity(position, time);
    result.density = getDensity(position);
    result.medium = isSubmerged(position) ? FluidMediumType::Water : FluidMediumType::Air;
    return result;
}

math::Vec2 FluidField2D::getVelocity(const math::Vec2& position, double time) const noexcept
{
    math::Vec2 baseVelocity { 0.0, 0.0 };

    if (isSubmerged(position)) {
        baseVelocity = mConfig.ambientCurrent;
        if (mConfig.enableCurrentDecay && (mConfig.currentDecayDepth > 1e-6)) {
            const double depth { mConfig.waterlineY - position.y };
            const double factor { std::exp(-depth / mConfig.currentDecayDepth) };
            baseVelocity = mConfig.ambientCurrent * factor;
        }
    } else {
        baseVelocity = mConfig.ambientWind;
        if (mConfig.enableWindShear && (mConfig.windShearRefHeight > 1e-6)) {
            const double altitude { position.y - mConfig.waterlineY };
            const double clampedAltitude { std::max(altitude, 0.1) };
            const double ratio { clampedAltitude / mConfig.windShearRefHeight };
            const double factor { std::pow(ratio, mConfig.windShearExponent) };
            baseVelocity = mConfig.ambientWind * factor;
        }
    }

    math::Vec2 gustVelocity { 0.0, 0.0 };

    for (const auto& gust : mDiscreteGusts) {
        if ((time >= gust.startTime) && (time <= (gust.startTime + gust.duration))
            && (gust.duration > 1e-9)) {
            const double tau { (time - gust.startTime) / gust.duration };
            const double oneMinusCos { 0.5 * (1.0 - std::cos(2.0 * kPi * tau)) };
            math::Vec2 dir { gust.direction };
            const double dirLenSq { dir.lengthSquared() };
            if (dirLenSq > 1e-12) {
                dir = dir.normalized();
            }
            gustVelocity += dir * (gust.amplitude * oneMinusCos);
        }
    }

    for (const auto& harmonic : mHarmonicGusts) {
        const double dotK { (harmonic.waveVector.x * position.x) + (harmonic.waveVector.y * position.y) };
        const double phaseArg { dotK - (harmonic.frequency * time) + harmonic.phase };
        const double sinVal { std::sin(phaseArg) };
        gustVelocity += harmonic.amplitude * sinVal;
    }

    return baseVelocity + gustVelocity;
}

double FluidField2D::getDensity(const math::Vec2& position) const noexcept
{
    return isSubmerged(position) ? mConfig.waterDensity : mConfig.airDensity;
}

bool FluidField2D::isSubmerged(const math::Vec2& position) const noexcept
{
    return position.y <= mConfig.waterlineY;
}

void FluidField2D::addDiscreteGust(const DiscreteGust2D& gust)
{
    mDiscreteGusts.push_back(gust);
}

void FluidField2D::addHarmonicGust(const HarmonicGust2D& gust)
{
    mHarmonicGusts.push_back(gust);
}

void FluidField2D::clearGusts() noexcept
{
    mDiscreteGusts.clear();
    mHarmonicGusts.clear();
}

const FluidFieldConfig& FluidField2D::getConfig() const noexcept
{
    return mConfig;
}

void FluidField2D::setConfig(const FluidFieldConfig& config) noexcept
{
    mConfig = config;
}

void FluidField2D::setAmbientWind(const math::Vec2& wind) noexcept
{
    mConfig.ambientWind = wind;
}

void FluidField2D::setAmbientCurrent(const math::Vec2& current) noexcept
{
    mConfig.ambientCurrent = current;
}

void FluidField2D::setWaterline(double waterlineY) noexcept
{
    mConfig.waterlineY = waterlineY;
}

std::size_t FluidField2D::getDiscreteGustCount() const noexcept
{
    return mDiscreteGusts.size();
}

std::size_t FluidField2D::getHarmonicGustCount() const noexcept
{
    return mHarmonicGusts.size();
}

} // namespace vehicle_sim::physics
