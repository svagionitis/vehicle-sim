#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <SDL2/SDL.h>

#include "apps/EarthPatrol/Camera2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace earth_patrol {

/// @enum ParticleType
/// @brief Semantic category determining particle physics and rendering style.
enum class ParticleType : std::uint8_t {
    Dust, ///< Terrestrial soil or sand kicked up by rolling tires.
    Spark, ///< Incandescent exhaust or thruster spark.
    Smoke, ///< Diffuse lingering explosion or exhaust puff.
    Debris ///< Solid high-velocity shrapnel shard from vehicle crash.
};

/// @struct Particle
/// @brief State payload for an individual active particle.
struct Particle {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 }; ///< World position in meters.
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 }; ///< World velocity in m/s.
    double lifetime { 0.0 }; ///< Remaining active duration in seconds.
    double maxLifetime { 1.0 }; ///< Total lifespan duration in seconds.
    double size { 0.1 }; ///< Rendered dimension in meters.
    SDL_Color color { 255, 255, 255, 255 }; ///< Base tint color.
    ParticleType type { ParticleType::Dust }; ///< Visual classification.
    bool active { false }; ///< Active allocation flag in ring pool.
};

/// @class ParticleSystem
/// @brief Zero-allocation ring buffer particle coordinator for arcade visual polish.
/// @details Pre-allocates a fixed pool of particle slots and synthesizes ground dust,
/// thruster exhaust sparks, landing impacts, and explosive shrapnel without heap overhead.
class ParticleSystem {
public:
    static constexpr std::size_t kMaxPoolSize { 384 };

    /// @brief Default constructor initializing fixed particle pool.
    ParticleSystem() noexcept;

    /// @brief Advances active particles by timestep dt, applying drag and gravity.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Renders all visible active particles within the camera viewport.
    /// @param[in] renderer Active SDL hardware renderer pointer.
    /// @param[in] camera Active 2D camera providing screen projection transform.
    void render(SDL_Renderer* renderer, const Camera2D& camera) const noexcept;

    /// @brief Emits a burst of tire surface dust at wheel contact coordinates.
    /// @param[in] pos World contact coordinates.
    /// @param[in] biomeIndex Active terrestrial biome (0: Gray, 1: Sand, 2: Loam).
    /// @param[in] count Number of dust puffs to spawn.
    void emitTireDust(const vehicle_sim::math::Vec2& pos, int biomeIndex, int count = 2) noexcept;

    /// @brief Emits high-energy thruster sparks downward from vehicle exhaust.
    /// @param[in] pos World position of thruster nozzle.
    /// @param[in] count Number of sparks to spawn.
    void emitThrusterSparks(const vehicle_sim::math::Vec2& pos, int count = 4) noexcept;

    /// @brief Emits a bilateral ground shockwave puff on wheel impact after landing.
    /// @param[in] pos World ground contact point.
    /// @param[in] biomeIndex Active biome index.
    void emitLandingDust(const vehicle_sim::math::Vec2& pos, int biomeIndex) noexcept;

    /// @brief Emits omnidirectional debris fragments and fire sparks on rover crash.
    /// @param[in] pos World center of vehicle destruction.
    void emitCrashExplosion(const vehicle_sim::math::Vec2& pos) noexcept;

    /// @brief Deactivates all active particles in the pool.
    void reset() noexcept;

    /// @brief Gets total count of currently active particles.
    /// @return Active particle count.
    [[nodiscard]] int getActiveCount() const noexcept;

private:
    /// @brief Allocates an inactive slot or recycles the oldest particle.
    /// @return Mutable reference to allocated particle slot.
    [[nodiscard]] Particle& allocateParticle() noexcept;

    std::array<Particle, kMaxPoolSize> pool_ { };
    std::size_t nextSlot_ { 0 };
    int activeCount_ { 0 };
};

} // namespace earth_patrol
