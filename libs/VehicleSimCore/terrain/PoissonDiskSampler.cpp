#include "PoissonDiskSampler.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace vehicle_sim::terrain {

namespace {

    constexpr double kPi { 3.14159265358979323846 };
    constexpr double kTwoPi { 2.0 * kPi };
    constexpr double kInvSqrt2 { 0.70710678118654752440 };

    /// @class DeterministicRng64
    /// @brief Cross-platform 64-bit deterministic linear congruential pseudo-random generator.
    class DeterministicRng64 {
    public:
        explicit DeterministicRng64(std::uint32_t seed) noexcept
            : mState { (static_cast<std::uint64_t>(seed) != 0ULL) ? static_cast<std::uint64_t>(seed) : 1ULL }
        {
        }

        [[nodiscard]] std::uint64_t next() noexcept
        {
            mState = mState * 6364136223846793005ULL + 1442695040888963407ULL;
            return mState;
        }

        [[nodiscard]] double nextDouble(double minVal, double maxVal) noexcept
        {
            const double unit { static_cast<double>(next() >> 11U) / 9007199254740992.0 };
            return minVal + unit * (maxVal - minVal);
        }

        [[nodiscard]] std::size_t nextSizeT(std::size_t maxExclusive) noexcept
        {
            if (maxExclusive <= 1U) {
                return 0U;
            }
            return static_cast<std::size_t>(next() % maxExclusive);
        }

    private:
        std::uint64_t mState { 1ULL };
    };

} // namespace

PoissonDiskSampler::PoissonDiskSampler(const PoissonSamplerConfig& config) noexcept
    : mConfig { config }
{
}

std::vector<math::Vec2> PoissonDiskSampler::generate(const PointAcceptancePredicate& predicate) const
{
    const std::vector<math::Vec2> emptySeeds {};
    return generateWithSeedPoints(emptySeeds, predicate);
}

std::vector<math::Vec2> PoissonDiskSampler::generateWithSeedPoints(
    const std::vector<math::Vec2>& initialPoints, const PointAcceptancePredicate& predicate) const
{
    std::vector<math::Vec2> samples {};

    if (mConfig.minDistance <= 1e-4) {
        return samples;
    }

    const double cellSize { std::max(0.01, mConfig.minDistance * kInvSqrt2) };
    const double width { std::max(cellSize, mConfig.bounds.max.x - mConfig.bounds.min.x) };
    const double height { std::max(cellSize, mConfig.bounds.max.y - mConfig.bounds.min.y) };

    const std::size_t gridWidth { static_cast<std::size_t>(std::ceil(width / cellSize)) };
    const std::size_t gridHeight { static_cast<std::size_t>(std::ceil(height / cellSize)) };
    const std::size_t totalCells { gridWidth * gridHeight };

    if (totalCells == 0U) {
        return samples;
    }

    std::vector<std::int32_t> grid(totalCells, -1);
    std::vector<std::size_t> activeList {};
    DeterministicRng64 rng { mConfig.seed };

    auto getCellCoords = [&](const math::Vec2& pt) noexcept -> std::pair<std::int32_t, std::int32_t> {
        const double relX { pt.x - mConfig.bounds.min.x };
        const double relY { pt.y - mConfig.bounds.min.y };
        const std::int32_t cx { static_cast<std::int32_t>(relX / cellSize) };
        const std::int32_t cy { static_cast<std::int32_t>(relY / cellSize) };
        return { cx, cy };
    };

    auto isValidCandidate = [&](const math::Vec2& pt) -> bool {
        if (!mConfig.bounds.contains(pt)) {
            return false;
        }
        if (predicate && !predicate(pt)) {
            return false;
        }
        const auto [cx, cy] = getCellCoords(pt);
        if (cx < 0 || static_cast<std::size_t>(cx) >= gridWidth || cy < 0
            || static_cast<std::size_t>(cy) >= gridHeight) {
            return false;
        }
        const double minDistSq { mConfig.minDistance * mConfig.minDistance };
        const std::int32_t minX { std::max(0, cx - 2) };
        const std::int32_t maxX { std::min(static_cast<std::int32_t>(gridWidth) - 1, cx + 2) };
        const std::int32_t minY { std::max(0, cy - 2) };
        const std::int32_t maxY { std::min(static_cast<std::int32_t>(gridHeight) - 1, cy + 2) };

        for (std::int32_t y { minY }; y <= maxY; ++y) {
            for (std::int32_t x { minX }; x <= maxX; ++x) {
                const std::size_t cellIndex { static_cast<std::size_t>(y) * gridWidth + static_cast<std::size_t>(x) };
                const std::int32_t sampleIdx { grid[cellIndex] };
                if (sampleIdx >= 0) {
                    const math::Vec2 diff { samples[static_cast<std::size_t>(sampleIdx)] - pt };
                    if (diff.lengthSquared() < minDistSq) {
                        return false;
                    }
                }
            }
        }
        return true;
    };

    // 1. Insert valid initial seed points
    for (const math::Vec2& pt : initialPoints) {
        if (isValidCandidate(pt)) {
            const auto [cx, cy] = getCellCoords(pt);
            const std::size_t cellIndex { static_cast<std::size_t>(cy) * gridWidth + static_cast<std::size_t>(cx) };
            samples.push_back(pt);
            grid[cellIndex] = static_cast<std::int32_t>(samples.size() - 1U);
            activeList.push_back(samples.size() - 1U);
        }
    }

    // 2. If no valid seeds, generate an initial seed point
    if (activeList.empty()) {
        const math::Vec2 center { (mConfig.bounds.min + mConfig.bounds.max) * 0.5 };
        math::Vec2 firstPoint { center };
        bool foundInitial { false };

        if (isValidCandidate(center)) {
            firstPoint = center;
            foundInitial = true;
        } else {
            for (std::size_t attempt { 0U }; attempt < 500U; ++attempt) {
                const double rx { rng.nextDouble(mConfig.bounds.min.x, mConfig.bounds.max.x) };
                const double ry { rng.nextDouble(mConfig.bounds.min.y, mConfig.bounds.max.y) };
                const math::Vec2 candidate { rx, ry };
                if (isValidCandidate(candidate)) {
                    firstPoint = candidate;
                    foundInitial = true;
                    break;
                }
            }
        }

        if (foundInitial) {
            const auto [cx, cy] = getCellCoords(firstPoint);
            const std::size_t cellIndex { static_cast<std::size_t>(cy) * gridWidth + static_cast<std::size_t>(cx) };
            samples.push_back(firstPoint);
            grid[cellIndex] = static_cast<std::int32_t>(samples.size() - 1U);
            activeList.push_back(samples.size() - 1U);
        } else {
            return samples;
        }
    }

    // 3. Bridson Poisson-Disk expansion loop
    while (!activeList.empty()) {
        const std::size_t activeIdx { rng.nextSizeT(activeList.size()) };
        const math::Vec2 sourcePoint { samples[activeList[activeIdx]] };
        bool candidateFound { false };

        for (std::size_t attempt { 0U }; attempt < mConfig.maxAttempts; ++attempt) {
            const double u1 { rng.nextDouble(0.0, 1.0) };
            const double u2 { rng.nextDouble(0.0, 1.0) };
            const double angle { kTwoPi * u1 };
            // Uniform areal sampling in [r_min, 2*r_min]
            const double radius { mConfig.minDistance * std::sqrt(3.0 * u2 + 1.0) };
            const math::Vec2 candidate { sourcePoint.x + radius * std::cos(angle),
                sourcePoint.y + radius * std::sin(angle) };

            if (isValidCandidate(candidate)) {
                const auto [cx, cy] = getCellCoords(candidate);
                const std::size_t cellIndex { static_cast<std::size_t>(cy) * gridWidth + static_cast<std::size_t>(cx) };
                samples.push_back(candidate);
                grid[cellIndex] = static_cast<std::int32_t>(samples.size() - 1U);
                activeList.push_back(samples.size() - 1U);
                candidateFound = true;
                break;
            }
        }

        if (!candidateFound) {
            std::swap(activeList[activeIdx], activeList.back());
            activeList.pop_back();
        }
    }

    return samples;
}

const PoissonSamplerConfig& PoissonDiskSampler::getConfig() const noexcept
{
    return mConfig;
}

void PoissonDiskSampler::setConfig(const PoissonSamplerConfig& config) noexcept
{
    mConfig = config;
}

} // namespace vehicle_sim::terrain
