#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::terrain {

/// @struct PoissonSamplerConfig
/// @brief Configuration parameters for 2D Bridson Poisson-disk blue-noise sampling.
struct PoissonSamplerConfig {
    double minDistance { 4.0 }; ///< Minimum Euclidean distance r_min between any two samples in meters.
    std::size_t maxAttempts { 30U }; ///< Rejection candidate attempts (k) per active sample.
    math::AABB2D bounds { math::Vec2 { -100.0, -100.0 }, math::Vec2 { 100.0, 100.0 } }; ///< Bounding domain.
    std::uint32_t seed { 1337U }; ///< Deterministic PRNG seed.
};

/// @typedef PointAcceptancePredicate
/// @brief Callback query determining whether a candidate point falls within a valid placement domain.
using PointAcceptancePredicate = std::function<bool(const math::Vec2&)>;

/// @class PoissonDiskSampler
/// @brief Generates 2D blue-noise point distributions with guaranteed minimum distance in O(N) time.
/// @details Implements Bridson's Poisson-disk algorithm using a uniform acceleration grid.
class PoissonDiskSampler {
public:
    /// @brief Parameterized constructor configuring sampler domain and parameters.
    /// @param[in] config Sampling domain and distance configuration.
    explicit PoissonDiskSampler(const PoissonSamplerConfig& config) noexcept;

    /// @brief Generates blue-noise sample points within the configured bounding domain.
    /// @param[in] predicate Optional acceptance test for boundary or mask filtering.
    /// @return Vector of placed 2D Cartesian points satisfying minimum distance constraint.
    [[nodiscard]] std::vector<math::Vec2> generate(
        const PointAcceptancePredicate& predicate = nullptr) const;

    /// @brief Generates blue-noise samples seeded with pre-existing fixed anchor points.
    /// @param[in] initialPoints Pre-existing fixed coordinates to preserve.
    /// @param[in] predicate Optional acceptance test for boundary or mask filtering.
    /// @return Complete vector of sample points including initial points.
    [[nodiscard]] std::vector<math::Vec2> generateWithSeedPoints(
        const std::vector<math::Vec2>& initialPoints,
        const PointAcceptancePredicate& predicate = nullptr) const;

    /// @brief Gets active sampler configuration.
    /// @return Constant reference to configuration.
    [[nodiscard]] const PoissonSamplerConfig& getConfig() const noexcept;

    /// @brief Updates sampler configuration.
    /// @param[in] config New configuration settings.
    void setConfig(const PoissonSamplerConfig& config) noexcept;

private:
    PoissonSamplerConfig mConfig { };
};

} // namespace vehicle_sim::terrain
