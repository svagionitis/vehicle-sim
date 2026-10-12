#include "libs/VehicleSimCore/terrain/FractalNoise.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

namespace {

    constexpr std::uint32_t MIN_OCTAVES { 1U };
    constexpr std::uint32_t MAX_OCTAVES { 16U };

} // namespace

FractalNoise::FractalNoise(const FractalConfig& config) noexcept
    : mConfig { config }
    , mNoise { config.seed }
{
    mConfig.octaves = std::clamp(mConfig.octaves, MIN_OCTAVES, MAX_OCTAVES);
}

const FractalConfig& FractalNoise::getConfig() const noexcept
{
    return mConfig;
}

void FractalNoise::setConfig(const FractalConfig& config) noexcept
{
    mConfig = config;
    mConfig.octaves = std::clamp(mConfig.octaves, MIN_OCTAVES, MAX_OCTAVES);
    mNoise.setSeed(mConfig.seed);
}

std::uint32_t FractalNoise::getSeed() const noexcept
{
    return mConfig.seed;
}

void FractalNoise::setSeed(std::uint32_t seed) noexcept
{
    mConfig.seed = seed;
    mNoise.setSeed(seed);
}

std::uint32_t FractalNoise::getOctaves() const noexcept
{
    return mConfig.octaves;
}

void FractalNoise::setOctaves(std::uint32_t octaves) noexcept
{
    mConfig.octaves = std::clamp(octaves, MIN_OCTAVES, MAX_OCTAVES);
}

double FractalNoise::getFrequency() const noexcept
{
    return mConfig.frequency;
}

void FractalNoise::setFrequency(double frequency) noexcept
{
    mConfig.frequency = frequency;
}

double FractalNoise::getAmplitude() const noexcept
{
    return mConfig.amplitude;
}

void FractalNoise::setAmplitude(double amplitude) noexcept
{
    mConfig.amplitude = amplitude;
}

double FractalNoise::getLacunarity() const noexcept
{
    return mConfig.lacunarity;
}

void FractalNoise::setLacunarity(double lacunarity) noexcept
{
    mConfig.lacunarity = lacunarity;
}

double FractalNoise::getPersistence() const noexcept
{
    return mConfig.persistence;
}

void FractalNoise::setPersistence(double persistence) noexcept
{
    mConfig.persistence = persistence;
}

FractalType FractalNoise::getType() const noexcept
{
    return mConfig.type;
}

void FractalNoise::setType(FractalType type) noexcept
{
    mConfig.type = type;
}

double FractalNoise::getMaxAmplitude() const noexcept
{
    double sum { 0.0 };
    double curAmp { mConfig.amplitude };
    const std::uint32_t octaves { std::clamp(mConfig.octaves, MIN_OCTAVES, MAX_OCTAVES) };
    for (std::uint32_t i { 0U }; i < octaves; ++i) {
        sum += curAmp;
        curAmp *= mConfig.persistence;
    }
    return sum;
}

double FractalNoise::sample1D(double x) const noexcept
{
    return sample1DWithDeriv(x).value;
}

NoiseSample1D FractalNoise::sample1DWithDeriv(double x) const noexcept
{
    NoiseSample1D total {};
    double currentFreq { mConfig.frequency };
    double currentAmp { mConfig.amplitude };

    const std::uint32_t octaves { std::clamp(mConfig.octaves, MIN_OCTAVES, MAX_OCTAVES) };
    for (std::uint32_t i { 0U }; i < octaves; ++i) {
        const NoiseSample1D s { mNoise.sample1DWithDeriv(x * currentFreq) };
        double val { s.value };
        double deriv { s.derivative };

        switch (mConfig.type) {
        case FractalType::Fbm:
            break;
        case FractalType::Billow: {
            const double sgn { val >= 0.0 ? 1.0 : -1.0 };
            val = (std::abs(val) * 2.0) - 1.0;
            deriv = 2.0 * sgn * deriv;
            break;
        }
        case FractalType::Ridged: {
            const double sgn { val >= 0.0 ? 1.0 : -1.0 };
            val = 1.0 - std::abs(val);
            deriv = -sgn * deriv;
            break;
        }
        }

        total.value += currentAmp * val;
        total.derivative += currentAmp * currentFreq * deriv;

        currentFreq *= mConfig.lacunarity;
        currentAmp *= mConfig.persistence;
    }

    return total;
}

double FractalNoise::sample2D(double x, double y) const noexcept
{
    return sample2DWithDeriv(x, y).value;
}

double FractalNoise::sample2D(const math::Vec2& point) const noexcept
{
    return sample2DWithDeriv(point.x, point.y).value;
}

NoiseSample2D FractalNoise::sample2DWithDeriv(const math::Vec2& point) const noexcept
{
    return sample2DWithDeriv(point.x, point.y);
}

NoiseSample2D FractalNoise::sample2DWithDeriv(double x, double y) const noexcept
{
    NoiseSample2D total {};
    double currentFreq { mConfig.frequency };
    double currentAmp { mConfig.amplitude };

    const std::uint32_t octaves { std::clamp(mConfig.octaves, MIN_OCTAVES, MAX_OCTAVES) };
    for (std::uint32_t i { 0U }; i < octaves; ++i) {
        const NoiseSample2D s { mNoise.sample2DWithDeriv(x * currentFreq, y * currentFreq) };
        double val { s.value };
        math::Vec2 grad { s.gradient };

        switch (mConfig.type) {
        case FractalType::Fbm:
            break;
        case FractalType::Billow: {
            const double sgn { val >= 0.0 ? 1.0 : -1.0 };
            val = (std::abs(val) * 2.0) - 1.0;
            grad = grad * (2.0 * sgn);
            break;
        }
        case FractalType::Ridged: {
            const double sgn { val >= 0.0 ? 1.0 : -1.0 };
            val = 1.0 - std::abs(val);
            grad = grad * (-sgn);
            break;
        }
        }

        total.value += currentAmp * val;
        total.gradient = total.gradient + grad * (currentAmp * currentFreq);

        currentFreq *= mConfig.lacunarity;
        currentAmp *= mConfig.persistence;
    }

    return total;
}

math::Vec2 FractalNoise::computeNormal1D(double x) const noexcept
{
    const double deriv { sample1DWithDeriv(x).derivative };
    const double len { std::hypot(1.0, deriv) };
    return math::Vec2 { -deriv / len, 1.0 / len };
}

math::Vec2 FractalNoise::computeTangent1D(double x) const noexcept
{
    const double deriv { sample1DWithDeriv(x).derivative };
    const double len { std::hypot(1.0, deriv) };
    return math::Vec2 { 1.0 / len, deriv / len };
}

double FractalNoise::computeSlopeAngle1D(double x) const noexcept
{
    const double deriv { sample1DWithDeriv(x).derivative };
    return std::atan(deriv);
}

} // namespace vehicle_sim::terrain
