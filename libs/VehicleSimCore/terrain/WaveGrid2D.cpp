#include "WaveGrid2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

WaveGrid2D::WaveGrid2D(const WaveConfig2D& config)
{
    initGrid(config);
}

void WaveGrid2D::initGrid(const WaveConfig2D& config)
{
    mConfig = config;
    const double validCellSize { std::max(0.01, config.cellSize) };
    const double validSizeX { std::max(validCellSize * 2.0, config.size.x) };
    const double validSizeY { std::max(validCellSize * 2.0, config.size.y) };
    mInvCellSize = 1.0 / validCellSize;

    mWidth = static_cast<std::size_t>(std::ceil(validSizeX * mInvCellSize)) + 1U;
    mHeight = static_cast<std::size_t>(std::ceil(validSizeY * mInvCellSize)) + 1U;
    mWidth = std::max<std::size_t>(mWidth, 3U);
    mHeight = std::max<std::size_t>(mHeight, 3U);

    const std::size_t totalCells { mWidth * mHeight };
    mElevationPrev.assign(totalCells, 0.0);
    mElevationCurr.assign(totalCells, 0.0);
    mElevationNext.assign(totalCells, 0.0);
    mSpongeDamping.assign(totalCells, 0.0);

    updateSpongeLayer();
}

void WaveGrid2D::updateSpongeLayer()
{
    if (mConfig.boundaryType != WaveBoundaryType::Absorbing || mConfig.spongeWidth == 0U) {
        return;
    }

    const std::size_t spongeWidth { mConfig.spongeWidth };
    const double maxSpongeDamping { 14.0 };

    for (std::size_t y { 0U }; y < mHeight; ++y) {
        const std::size_t distY { std::min(y, mHeight - 1U - y) };
        for (std::size_t x { 0U }; x < mWidth; ++x) {
            const std::size_t distX { std::min(x, mWidth - 1U - x) };
            const std::size_t edgeDist { std::min(distX, distY) };
            if (edgeDist < spongeWidth) {
                const double normDist { static_cast<double>(spongeWidth - edgeDist)
                    / static_cast<double>(spongeWidth) };
                mSpongeDamping[getIndex(x, y)] = maxSpongeDamping * normDist * normDist;
            }
        }
    }
}

void WaveGrid2D::step(double dt) noexcept
{
    if (dt <= 0.0 || mWidth < 3U || mHeight < 3U) {
        return;
    }

    const double validWaveSpeed { std::max(0.1, mConfig.waveSpeed) };
    constexpr double kSqrt2 { 1.41421356237309504880 };
    const double maxStableDt { 0.70 / (validWaveSpeed * mInvCellSize * kSqrt2) };

    const std::size_t subStepCount { std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(dt / maxStableDt))) };
    const double subDt { dt / static_cast<double>(subStepCount) };

    mLastSubDt = subDt;
    for (std::size_t s { 0U }; s < subStepCount; ++s) {
        executeSubstep(subDt);
    }
}

void WaveGrid2D::executeSubstep(double dt) noexcept
{
    const double c2dt2 { mConfig.waveSpeed * mConfig.waveSpeed * dt * dt };
    const double invDx2 { mInvCellSize * mInvCellSize };

    if (mConfig.use9PointLaplacian) {
        const double invDx2Div6 { invDx2 / 6.0 };
        for (std::size_t y { 1U }; y < mHeight - 1U; ++y) {
            for (std::size_t x { 1U }; x < mWidth - 1U; ++x) {
                const std::size_t idx { getIndex(x, y) };
                const double effectiveDamping { mConfig.damping + mSpongeDamping[idx] };
                const double alpha { 0.5 * effectiveDamping * dt };

                const double orthogonalSum { mElevationCurr[getIndex(x + 1U, y)]
                    + mElevationCurr[getIndex(x - 1U, y)]
                    + mElevationCurr[getIndex(x, y + 1U)]
                    + mElevationCurr[getIndex(x, y - 1U)] };
                const double diagonalSum { mElevationCurr[getIndex(x + 1U, y + 1U)]
                    + mElevationCurr[getIndex(x - 1U, y + 1U)]
                    + mElevationCurr[getIndex(x + 1U, y - 1U)]
                    + mElevationCurr[getIndex(x - 1U, y - 1U)] };
                const double center { mElevationCurr[idx] };

                const double laplacian { ((4.0 * orthogonalSum) + diagonalSum - (20.0 * center)) * invDx2Div6 };

                const double numerator { (2.0 * center) - ((1.0 - alpha) * mElevationPrev[idx])
                    + (c2dt2 * laplacian) };
                mElevationNext[idx] = numerator / (1.0 + alpha);
            }
        }
    } else {
        for (std::size_t y { 1U }; y < mHeight - 1U; ++y) {
            for (std::size_t x { 1U }; x < mWidth - 1U; ++x) {
                const std::size_t idx { getIndex(x, y) };
                const double effectiveDamping { mConfig.damping + mSpongeDamping[idx] };
                const double alpha { 0.5 * effectiveDamping * dt };

                const double laplacian { (mElevationCurr[getIndex(x + 1U, y)]
                                            + mElevationCurr[getIndex(x - 1U, y)]
                                            + mElevationCurr[getIndex(x, y + 1U)]
                                            + mElevationCurr[getIndex(x, y - 1U)]
                                            - (4.0 * mElevationCurr[idx]))
                    * invDx2 };

                const double numerator { (2.0 * mElevationCurr[idx]) - ((1.0 - alpha) * mElevationPrev[idx])
                    + (c2dt2 * laplacian) };
                mElevationNext[idx] = numerator / (1.0 + alpha);
            }
        }
    }

    applyBoundaries();

    mElevationPrev.swap(mElevationCurr);
    mElevationCurr.swap(mElevationNext);
}

void WaveGrid2D::applyBoundaries() noexcept
{
    if (mConfig.boundaryType == WaveBoundaryType::Neumann) {
        for (std::size_t y { 0U }; y < mHeight; ++y) {
            mElevationNext[getIndex(0U, y)] = mElevationNext[getIndex(1U, y)];
            mElevationNext[getIndex(mWidth - 1U, y)] = mElevationNext[getIndex(mWidth - 2U, y)];
        }
        for (std::size_t x { 0U }; x < mWidth; ++x) {
            mElevationNext[getIndex(x, 0U)] = mElevationNext[getIndex(x, 1U)];
            mElevationNext[getIndex(x, mHeight - 1U)] = mElevationNext[getIndex(x, mHeight - 2U)];
        }
    } else {
        for (std::size_t y { 0U }; y < mHeight; ++y) {
            mElevationNext[getIndex(0U, y)] = 0.0;
            mElevationNext[getIndex(mWidth - 1U, y)] = 0.0;
        }
        for (std::size_t x { 0U }; x < mWidth; ++x) {
            mElevationNext[getIndex(x, 0U)] = 0.0;
            mElevationNext[getIndex(x, mHeight - 1U)] = 0.0;
        }
    }
}

void WaveGrid2D::injectImpulse(const math::Vec2& position, double radius, double impulse) noexcept
{
    if (radius <= 0.0 || mWidth < 3U || mHeight < 3U) {
        return;
    }

    const double minX { position.x - radius };
    const double maxX { position.x + radius };
    const double minY { position.y - radius };
    const double maxY { position.y + radius };

    const double startX { mConfig.origin.x };
    const double startY { mConfig.origin.y };
    const double boundMaxX { startX + ((mWidth - 1U) * mConfig.cellSize) };
    const double boundMaxY { startY + ((mHeight - 1U) * mConfig.cellSize) };

    const double clampedMinX { std::clamp(minX, startX, boundMaxX) };
    const double clampedMaxX { std::clamp(maxX, startX, boundMaxX) };
    const double clampedMinY { std::clamp(minY, startY, boundMaxY) };
    const double clampedMaxY { std::clamp(maxY, startY, boundMaxY) };

    if (clampedMinX > clampedMaxX || clampedMinY > clampedMaxY) {
        return;
    }

    const std::size_t startIdxX { static_cast<std::size_t>(
        std::max(0.0, std::floor((clampedMinX - startX) * mInvCellSize))) };
    const std::size_t endIdxX { std::min(
        mWidth - 1U, static_cast<std::size_t>(std::ceil((clampedMaxX - startX) * mInvCellSize))) };
    const std::size_t startIdxY { static_cast<std::size_t>(
        std::max(0.0, std::floor((clampedMinY - startY) * mInvCellSize))) };
    const std::size_t endIdxY { std::min(
        mHeight - 1U, static_cast<std::size_t>(std::ceil((clampedMaxY - startY) * mInvCellSize))) };

    const double invRadius { 1.0 / radius };
    constexpr double kPi { 3.14159265358979323846 };

    for (std::size_t y { startIdxY }; y <= endIdxY; ++y) {
        const double cellY { startY + (static_cast<double>(y) * mConfig.cellSize) };
        const double dy { cellY - position.y };
        for (std::size_t x { startIdxX }; x <= endIdxX; ++x) {
            const double cellX { startX + (static_cast<double>(x) * mConfig.cellSize) };
            const double dx { cellX - position.x };
            const double dist { std::sqrt((dx * dx) + (dy * dy)) };
            if (dist < radius) {
                const double weight { 0.5 * (1.0 + std::cos(kPi * dist * invRadius)) };
                const double delta { impulse * weight };
                const std::size_t cellIdx { getIndex(x, y) };
                mElevationCurr[cellIdx] += delta;
                mElevationPrev[cellIdx] += delta;
            }
        }
    }
}

void WaveGrid2D::applyWakeStamp(const WaterWakeStamp2D& stamp) noexcept
{
    const double speed { stamp.velocity.length() };
    if (speed < 0.1 && std::abs(stamp.verticalVelocity) < 0.05) {
        return;
    }

    const double halfLen { std::max(0.5, stamp.length * 0.5) };
    const double halfBeam { std::max(0.3, stamp.beam * 0.5) };
    const double boundingRadius { std::sqrt((halfLen * halfLen) + (halfBeam * halfBeam)) };

    const double minX { stamp.position.x - boundingRadius };
    const double maxX { stamp.position.x + boundingRadius };
    const double minY { stamp.position.y - boundingRadius };
    const double maxY { stamp.position.y + boundingRadius };

    const double startX { mConfig.origin.x };
    const double startY { mConfig.origin.y };
    const double boundMaxX { startX + ((mWidth - 1U) * mConfig.cellSize) };
    const double boundMaxY { startY + ((mHeight - 1U) * mConfig.cellSize) };

    const double clampedMinX { std::clamp(minX, startX, boundMaxX) };
    const double clampedMaxX { std::clamp(maxX, startX, boundMaxX) };
    const double clampedMinY { std::clamp(minY, startY, boundMaxY) };
    const double clampedMaxY { std::clamp(maxY, startY, boundMaxY) };

    if (clampedMinX > clampedMaxX || clampedMinY > clampedMaxY) {
        return;
    }

    const std::size_t startIdxX { static_cast<std::size_t>(
        std::max(0.0, std::floor((clampedMinX - startX) * mInvCellSize))) };
    const std::size_t endIdxX { std::min(
        mWidth - 1U, static_cast<std::size_t>(std::ceil((clampedMaxX - startX) * mInvCellSize))) };
    const std::size_t startIdxY { static_cast<std::size_t>(
        std::max(0.0, std::floor((clampedMinY - startY) * mInvCellSize))) };
    const std::size_t endIdxY { std::min(
        mHeight - 1U, static_cast<std::size_t>(std::ceil((clampedMaxY - startY) * mInvCellSize))) };

    const double cosH { std::cos(-stamp.heading) };
    const double sinH { std::sin(-stamp.heading) };

    const double draftRatio { std::clamp(stamp.draft / 1.0, 0.2, 1.5) };
    const double wakeImpulseBase { 0.04 * speed * draftRatio };
    const double heaveImpulse { -0.15 * stamp.verticalVelocity };

    for (std::size_t y { startIdxY }; y <= endIdxY; ++y) {
        const double cellY { startY + (static_cast<double>(y) * mConfig.cellSize) };
        const double dy { cellY - stamp.position.y };
        for (std::size_t x { startIdxX }; x <= endIdxX; ++x) {
            const double cellX { startX + (static_cast<double>(x) * mConfig.cellSize) };
            const double dx { cellX - stamp.position.x };

            // Rotate into local hull coordinates
            const double localX { (dx * cosH) - (dy * sinH) };
            const double localY { (dx * sinH) + (dy * cosH) };

            const double normX { std::abs(localX) / halfLen };
            const double normY { std::abs(localY) / halfBeam };

            if (normX <= 1.0 && normY <= 1.0) {
                const double weight { (1.0 - normX) * (1.0 - normY) };
                // Stern generates trough (-), bow generates crest (+)
                const double longitudinalShape { (localX > 0.0) ? 0.6 : -0.8 };
                const double impulse { (longitudinalShape * wakeImpulseBase) + heaveImpulse };
                mElevationPrev[getIndex(x, y)] -= impulse * weight;
            }
        }
    }
}

WaveSample2D WaveGrid2D::getSample(const math::Vec2& position) const noexcept
{
    WaveSample2D sample {};
    if (mWidth < 2U || mHeight < 2U) {
        sample.elevation = mConfig.waterlineZ;
        return sample;
    }

    const double relX { position.x - mConfig.origin.x };
    const double relY { position.y - mConfig.origin.y };
    const double maxRelX { static_cast<double>(mWidth - 1U) * mConfig.cellSize };
    const double maxRelY { static_cast<double>(mHeight - 1U) * mConfig.cellSize };

    const double clampedRelX { std::clamp(relX, 0.0, maxRelX) };
    const double clampedRelY { std::clamp(relY, 0.0, maxRelY) };

    const double fracX { clampedRelX * mInvCellSize };
    const double fracY { clampedRelY * mInvCellSize };

    const std::size_t x0 { static_cast<std::size_t>(std::floor(fracX)) };
    const std::size_t y0 { static_cast<std::size_t>(std::floor(fracY)) };
    const std::size_t x1 { std::min(mWidth - 1U, x0 + 1U) };
    const std::size_t y1 { std::min(mHeight - 1U, y0 + 1U) };

    const double tx { fracX - static_cast<double>(x0) };
    const double ty { fracY - static_cast<double>(y0) };

    const double h00 { mElevationCurr[getIndex(x0, y0)] };
    const double h10 { mElevationCurr[getIndex(x1, y0)] };
    const double h01 { mElevationCurr[getIndex(x0, y1)] };
    const double h11 { mElevationCurr[getIndex(x1, y1)] };

    const double eta { ((1.0 - tx) * ((1.0 - ty) * h00 + ty * h01))
        + (tx * ((1.0 - ty) * h10 + ty * h11)) };

    // Gradient dh/dx and dh/dy
    const double dhdx { (((1.0 - ty) * (h10 - h00)) + (ty * (h11 - h01))) * mInvCellSize };
    const double dhdy { (((1.0 - tx) * (h01 - h00)) + (tx * (h11 - h10))) * mInvCellSize };

    const double gradLenSq { (dhdx * dhdx) + (dhdy * dhdy) };
    const double slopeAngle { std::atan(std::sqrt(gradLenSq)) };

    const double invNormLen { 1.0 / std::sqrt(1.0 + gradLenSq) };
    const SurfaceNormal3D normal { -dhdx * invNormLen, -dhdy * invNormLen, invNormLen };

    const double prev00 { mElevationPrev[getIndex(x0, y0)] };
    const double prevEta { prev00 };
    const double verticalSpeed { (eta - prevEta) / std::max(1e-4, mLastSubDt) };

    sample.elevation = mConfig.waterlineZ + eta;
    sample.perturbation = eta;
    sample.gradient = math::Vec2 { dhdx, dhdy };
    sample.verticalSpeed = verticalSpeed;
    sample.slopeAngle = slopeAngle;
    sample.normal = normal;
    return sample;
}

double WaveGrid2D::getElevation(const math::Vec2& position) const noexcept
{
    return getSample(position).elevation;
}

double WaveGrid2D::computeEnergy() const noexcept
{
    if (mWidth < 2U || mHeight < 2U) {
        return 0.0;
    }

    double totalEnergy { 0.0 };
    const double gravity { 9.80665 };
    const double density { 1000.0 };
    const double cellArea { mConfig.cellSize * mConfig.cellSize };
    const double invDt { 1.0 / std::max(1e-4, mLastSubDt) };

    for (std::size_t i { 0U }; i < mElevationCurr.size(); ++i) {
        const double eta { mElevationCurr[i] };
        const double vel { (eta - mElevationPrev[i]) * invDt };
        const double potEnergy { 0.5 * density * gravity * eta * eta * cellArea };
        const double kinEnergy { 0.5 * density * vel * vel * cellArea };
        totalEnergy += potEnergy + kinEnergy;
    }

    return totalEnergy;
}

const WaveConfig2D& WaveGrid2D::getConfig() const noexcept
{
    return mConfig;
}

std::size_t WaveGrid2D::getWidth() const noexcept
{
    return mWidth;
}

std::size_t WaveGrid2D::getHeight() const noexcept
{
    return mHeight;
}

void WaveGrid2D::reset() noexcept
{
    std::fill(mElevationPrev.begin(), mElevationPrev.end(), 0.0);
    std::fill(mElevationCurr.begin(), mElevationCurr.end(), 0.0);
    std::fill(mElevationNext.begin(), mElevationNext.end(), 0.0);
}

} // namespace vehicle_sim::terrain
