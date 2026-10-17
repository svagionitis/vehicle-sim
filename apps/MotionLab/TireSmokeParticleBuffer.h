#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @struct SmokeParticle
/// @brief 2D world-space expanding vapor puff representing tire scrub smoke.
struct SmokeParticle {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };  ///< World position in meters.
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };  ///< Drift velocity in m/s.
    double radius { 0.20 };                         ///< Expanding puff radius in meters.
    double age { 0.0 };                            ///< Current particle age in seconds.
    double maxLifetime { 1.50 };                   ///< Total particle lifespan in seconds.
    double initialAlpha { 0.85 };                  ///< Initial opacity factor in range [0.0, 1.0].
};

/// @class TireSmokeParticleBuffer
/// @brief Zero-allocation circular buffer of retro vector wireframe smoke particles.
class TireSmokeParticleBuffer {
public:
    static constexpr std::size_t kMaxParticles { 512 };

    /// @brief Default constructor pre-allocating fixed memory pool.
    TireSmokeParticleBuffer() noexcept;

    /// @brief Emits a smoke puff at a specific wheel world location.
    /// @param[in] origin Wheel ground contact center in world coordinates.
    /// @param[in] slipVelocity Slip vector imparting initial puff drift.
    /// @param[in] intensity Heat and slip scaling factor in range [0.0, 1.0].
    void emit(const vehicle_sim::math::Vec2& origin,
              const vehicle_sim::math::Vec2& slipVelocity,
              double intensity) noexcept;

    /// @brief Updates particle expansion, drag deceleration, and aging.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Clears all active smoke particles.
    void clear() noexcept;

    /// @brief Retrieves list of active smoke particles.
    /// @return Const reference to vector of active SmokeParticle.
    [[nodiscard]] const std::vector<SmokeParticle>& getParticles() const noexcept;

    /// @brief Queries number of currently alive particles.
    /// @return Count of active particles.
    [[nodiscard]] std::size_t getActiveCount() const noexcept;

private:
    std::vector<SmokeParticle> mParticles {};
    std::size_t mWriteIndex { 0 };
    std::size_t mActiveCount { 0 };
};

} // namespace motion_lab
