#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace montsoft_offroad {

/// @enum ParticleType
/// @brief Classification of terramechanic and combat particle effects.
enum class ParticleType : std::uint8_t {
    SandRoosterTail,  ///< Grit sprayed by spinning off-road tires.
    LandingDustPlume, ///< Dense billowing cloud expanding on suspension impact.
    ThrusterSpark,    ///< Incandescent jet flare from jump thruster nozzles.
    RocketExhaust,    ///< Trailing propellant smoke behind active missiles.
    ExplosionShrapnel ///< High-speed fiery blast fragments.
};

/// @struct Particle
/// @brief Kinematic and rendering attributes of an individual particle in the pool.
struct Particle {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 }; ///< World position in meters.
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 }; ///< Velocity in m/s.
    double age { 0.0 };                            ///< Elapsed lifetime in seconds.
    double maxLifetime { 0.5 };                    ///< Total duration before recycling.
    ColorRgba baseColor { 255, 255, 255, 255 };    ///< Primary CGA render color.
    ParticleType type { ParticleType::SandRoosterTail }; ///< Effect classification.
    std::uint8_t sizePx { 1 };                     ///< Render footprint diameter in screen pixels.
    bool active { false };                         ///< Active slot flag in pool.
};

/// @class ParticleSystem
/// @brief High-performance, pool-allocated terramechanic particle engine.
/// @details Pre-allocates a fixed array of 1024 particle slots with zero heap allocations during
/// gameplay. Simulates aerodynamic drag, gravity, and biome-specific color selection.
class ParticleSystem final {
public:
    static constexpr std::size_t kMaxParticles { 1024 };

    /// @brief Default constructor initializing inactive particle pool.
    ParticleSystem() noexcept;

    /// @brief Destructor.
    ~ParticleSystem() noexcept = default;

    /// @brief Advances all active particles by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Rasterizes all active particles into the CGA virtual framebuffer.
    /// @param[in,out] renderer Target CGA palette renderer.
    /// @param[in] camera Camera tracker performing world-to-screen coordinate mapping.
    void render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;

    /// @brief Emits a spray of sand rooster-tail particles from a tire contact patch.
    /// @param[in] contactWorldPos Ground contact location in world coordinates.
    /// @param[in] chassisVelocity Current vehicle forward velocity in m/s.
    /// @param[in] soilSinkage Bekker soil penetration depth in meters.
    /// @param[in] stageIndex Active course stage determining biome color scheme.
    void emitRoosterTail(
        const vehicle_sim::math::Vec2& contactWorldPos,
        const vehicle_sim::math::Vec2& chassisVelocity,
        double soilSinkage,
        std::size_t stageIndex
    ) noexcept;

    /// @brief Emits a lateral billowing dust plume upon hard suspension touchdown.
    /// @param[in] contactWorldPos Ground contact location in world coordinates.
    /// @param[in] impactVelocity Strut compression velocity in m/s.
    /// @param[in] stageIndex Active course stage determining biome color scheme.
    void emitLandingDust(
        const vehicle_sim::math::Vec2& contactWorldPos,
        double impactVelocity,
        std::size_t stageIndex
    ) noexcept;

    /// @brief Emits high-velocity pyrotechnic sparks from undercarriage jump thrusters.
    /// @param[in] thrusterWorldPos Undercarriage nozzle position in world coordinates.
    /// @param[in] chassisVelocity Current vehicle velocity in m/s.
    void emitThrusterSparks(
        const vehicle_sim::math::Vec2& thrusterWorldPos,
        const vehicle_sim::math::Vec2& chassisVelocity
    ) noexcept;

    /// @brief Clears and deactivates all particles in the pool.
    void reset() noexcept;

    /// @brief Gets count of currently active particles in pool.
    /// @return Active particle count in [0, kMaxParticles].
    [[nodiscard]] std::size_t getActiveCount() const noexcept;

private:
    [[nodiscard]] std::size_t allocateSlot() noexcept;
    [[nodiscard]] double nextRandomDouble() noexcept;
    [[nodiscard]] ColorRgba getBiomeColor(std::size_t stageIndex, ParticleType type) noexcept;

    std::array<Particle, kMaxParticles> pool_ {};
    std::size_t nextSlot_ { 0 };
    std::size_t activeCount_ { 0 };
    std::uint32_t rngState_ { 19880724U };
};

} // namespace montsoft_offroad
