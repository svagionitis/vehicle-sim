#include "WaveGrid1D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

WaveGrid1D::WaveGrid1D(const WaveConfig1D& config)
{
    initGrid(config);
}

void WaveGrid1D::initGrid(const WaveConfig1D& config)
{
    mConfig = config;
    const double validCellSize { std::max(0.01, config.cellSize) };
    const double validLength { std::max(validCellSize * 2.0, config.length) };
    mInvCellSize = 1.0 / validCellSize;

    mCellCount = static_cast<std::size_t>(std::ceil(validLength * mInvCellSize)) + 1U;
    mCellCount = std::max<std::size_t>(mCellCount, 3U);

    mElevationPrev.assign(mCellCount, 0.0);
    mElevationCurr.assign(mCellCount, 0.0);
    mElevationNext.assign(mCellCount, 0.0);
    mSpongeDamping.assign(mCellCount, 0.0);

    updateSpongeCoeffs();
}

void WaveGrid1D::updateSpongeCoeffs()
{
    const std::size_t spongeWidth { mConfig.spongeWidth };
    if (spongeWidth == 0U || spongeWidth * 2U >= mCellCount) {
        return;
    }

    const double maxSpongeDamping { 12.0 };

    if (mConfig.leftBoundary == WaveBoundaryType::Absorbing) {
        for (std::size_t i { 0U }; i < spongeWidth; ++i) {
            const double normDist { static_cast<double>(spongeWidth - i) / static_cast<double>(spongeWidth) };
            mSpongeDamping[i] = maxSpongeDamping * normDist * normDist;
        }
    }

    if (mConfig.rightBoundary == WaveBoundaryType::Absorbing) {
        for (std::size_t i { 0U }; i < spongeWidth; ++i) {
            const std::size_t cellIdx { mCellCount - 1U - i };
            const double normDist { static_cast<double>(spongeWidth - i) / static_cast<double>(spongeWidth) };
            mSpongeDamping[cellIdx] = maxSpongeDamping * normDist * normDist;
        }
    }
}

double WaveGrid1D::computeCourant(double dt) const noexcept
{
    return (mConfig.waveSpeed * dt) * mInvCellSize;
}

void WaveGrid1D::step(double dt) noexcept
{
    if (dt <= 0.0 || mCellCount < 3U) {
        return;
    }

    const double validWaveSpeed { std::max(0.1, mConfig.waveSpeed) };
    const double maxStableDt { 0.70 / (validWaveSpeed * mInvCellSize) };

    const std::size_t subStepCount { std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(dt / maxStableDt))) };
    const double subDt { dt / static_cast<double>(subStepCount) };

    mLastSubDt = subDt;
    for (std::size_t s { 0U }; s < subStepCount; ++s) {
        executeSubstep(subDt);
    }
}

void WaveGrid1D::executeSubstep(double dt) noexcept
{
    const double c2dt2 { mConfig.waveSpeed * mConfig.waveSpeed * dt * dt };
    const double invDx2 { mInvCellSize * mInvCellSize };

    for (std::size_t i { 1U }; i < mCellCount - 1U; ++i) {
        const double effectiveDamping { mConfig.damping + mSpongeDamping[i] };
        const double alpha { 0.5 * effectiveDamping * dt };

        const double laplacian { (mElevationCurr[i + 1U] - (2.0 * mElevationCurr[i]) + mElevationCurr[i - 1U])
            * invDx2 };

        const double numerator { (2.0 * mElevationCurr[i]) - ((1.0 - alpha) * mElevationPrev[i])
            + (c2dt2 * laplacian) };
        mElevationNext[i] = numerator / (1.0 + alpha);
    }

    applyBoundaries();

    mElevationPrev.swap(mElevationCurr);
    mElevationCurr.swap(mElevationNext);
}

void WaveGrid1D::applyBoundaries() noexcept
{
    if (mConfig.leftBoundary == WaveBoundaryType::Neumann) {
        mElevationNext[0U] = mElevationNext[1U];
    } else {
        mElevationNext[0U] = 0.0;
    }

    const std::size_t lastIdx { mCellCount - 1U };
    if (mConfig.rightBoundary == WaveBoundaryType::Neumann) {
        mElevationNext[lastIdx] = mElevationNext[lastIdx - 1U];
    } else {
        mElevationNext[lastIdx] = 0.0;
    }
}

void WaveGrid1D::injectImpulse(double x, double radius, double impulse) noexcept
{
    if (radius <= 0.0 || mCellCount < 3U) {
        return;
    }

    const double minX { x - radius };
    const double maxX { x + radius };

    const double startCoord { mConfig.originX };
    const double clampedMin { std::max(startCoord, minX) };
    const double clampedMax { std::min(startCoord + ((mCellCount - 1U) * mConfig.cellSize), maxX) };

    if (clampedMin > clampedMax) {
        return;
    }

    const std::size_t startIdx { static_cast<std::size_t>(
        std::max(0.0, std::floor((clampedMin - startCoord) * mInvCellSize))) };
    const std::size_t endIdx { std::min(
        mCellCount - 1U, static_cast<std::size_t>(std::ceil((clampedMax - startCoord) * mInvCellSize))) };

    const double invRadius { 1.0 / radius };
    constexpr double kPi { 3.14159265358979323846 };

    for (std::size_t i { startIdx }; i <= endIdx; ++i) {
        const double cellX { startCoord + (static_cast<double>(i) * mConfig.cellSize) };
        const double dist { std::abs(cellX - x) };
        if (dist < radius) {
            const double weight { 0.5 * (1.0 + std::cos(kPi * dist * invRadius)) };
            const double delta { impulse * weight };
            mElevationCurr[i] += delta;
            mElevationPrev[i] += delta;
        }
    }
}

WaveSample1D WaveGrid1D::getSample(double x) const noexcept
{
    WaveSample1D sample {};
    if (mCellCount < 2U) {
        sample.elevation = mConfig.waterlineY;
        return sample;
    }

    const double relX { x - mConfig.originX };
    const double clampedRelX { std::clamp(relX, 0.0, static_cast<double>(mCellCount - 1U) * mConfig.cellSize) };

    const double fracIdx { clampedRelX * mInvCellSize };
    const std::size_t idx { static_cast<std::size_t>(std::floor(fracIdx)) };
    const std::size_t nextIdx { std::min(mCellCount - 1U, idx + 1U) };
    const double t { fracIdx - static_cast<double>(idx) };

    const double eta0 { mElevationCurr[idx] };
    const double eta1 { mElevationCurr[nextIdx] };
    const double eta { ((1.0 - t) * eta0) + (t * eta1) };

    const double derivative { (eta1 - eta0) * mInvCellSize };
    const double slopeAngle { std::atan(derivative) };

    const double invNormLen { 1.0 / std::sqrt(1.0 + (derivative * derivative)) };
    const math::Vec2 normal { -derivative * invNormLen, invNormLen };

    const double prevEta0 { mElevationPrev[idx] };
    const double prevEta1 { mElevationPrev[nextIdx] };
    const double prevEta { ((1.0 - t) * prevEta0) + (t * prevEta1) };
    const double verticalSpeed { (eta - prevEta) / std::max(1e-4, mLastSubDt) };

    sample.elevation = mConfig.waterlineY + eta;
    sample.perturbation = eta;
    sample.derivative = derivative;
    sample.verticalSpeed = verticalSpeed;
    sample.slopeAngle = slopeAngle;
    sample.normal = normal;
    return sample;
}

double WaveGrid1D::getElevation(double x) const noexcept
{
    return getSample(x).elevation;
}

double WaveGrid1D::computeEnergy() const noexcept
{
    if (mCellCount < 2U) {
        return 0.0;
    }

    double totalEnergy { 0.0 };
    const double gravity { 9.80665 };
    const double density { 1000.0 };
    const double dx { mConfig.cellSize };
    const double invDt { 1.0 / std::max(1e-4, mLastSubDt) };

    for (std::size_t i { 0U }; i < mCellCount; ++i) {
        const double eta { mElevationCurr[i] };
        const double vel { (eta - mElevationPrev[i]) * invDt };
        const double potEnergy { 0.5 * density * gravity * eta * eta * dx };
        const double kinEnergy { 0.5 * density * vel * vel * dx };
        totalEnergy += potEnergy + kinEnergy;
    }

    return totalEnergy;
}

const WaveConfig1D& WaveGrid1D::getConfig() const noexcept
{
    return mConfig;
}

std::size_t WaveGrid1D::getCellCount() const noexcept
{
    return mCellCount;
}

void WaveGrid1D::reset() noexcept
{
    std::fill(mElevationPrev.begin(), mElevationPrev.end(), 0.0);
    std::fill(mElevationCurr.begin(), mElevationCurr.end(), 0.0);
    std::fill(mElevationNext.begin(), mElevationNext.end(), 0.0);
}

} // namespace vehicle_sim::terrain
