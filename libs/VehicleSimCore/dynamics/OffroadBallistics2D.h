#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum ProjectileType
/// @brief Classification of ballistic combat ordnance.
enum class ProjectileType : std::uint8_t {
    ForwardRocket,   ///< High-speed horizontal kinetic missile.
    AntiAirMortar    ///< High-angle parabolic anti-air flak shell.
};

/// @struct ProjectileConfig
/// @brief Physical and damage properties for a projectile class.
struct ProjectileConfig {
    double mass { 8.0 };                ///< Projectile mass in kg.
    double muzzleVelocity { 45.0 };     ///< Launch speed in m/s.
    double launchAngleOffset { 0.0 };   ///< Relative launch angle offset in radians.
    double dragCoefficient { 0.25 };    ///< Aerodynamic drag coefficient Cd.
    double crossSectionArea { 0.02 };   ///< Frontal area in m^2.
    double blastRadius { 2.5 };         ///< Explosive blast radius in meters.
    double blastMaxImpulse { 4500.0 };  ///< Peak explosive impulse at epicenter in N*s.
    double baseDamage { 50.0 };         ///< Damage points inflicted at epicenter.
    double maxLifespan { 4.0 };         ///< Maximum active flight time in seconds.
};

/// @struct ProjectileState
/// @brief Kinetic and visual state of an active in-flight projectile.
struct ProjectileState {
    ProjectileType type { ProjectileType::ForwardRocket };
    math::Vec2 position { 0.0, 0.0 };
    math::Vec2 velocity { 0.0, 0.0 };
    double heading { 0.0 };
    double age { 0.0 };
    bool isAlive { true };
};

/// @struct BlastEvent
/// @brief Telemetry of an explosion detonation for visual/sound triggering.
struct BlastEvent {
    math::Vec2 epicenter { 0.0, 0.0 };
    double radius { 2.5 };
    double damage { 50.0 };
    double maxImpulse { 4500.0 };
};

/// @class OffroadBallistics2D
/// @brief Ballistic trajectory integrator and explosion impulse coordinator.
/// @details Simulates physical projectile motion subject to aerodynamic drag and gravity,
/// detects continuous ground heightfield collisions, and applies radial blast impulses.
class OffroadBallistics2D final {
public:
    /// @brief Default constructor initializing default rocket and mortar configurations.
    OffroadBallistics2D() noexcept;

    /// @brief Destructor.
    ~OffroadBallistics2D() noexcept = default;

    /// @brief Advances all active projectiles by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] terrain Pointer to heightfield for ground collision checks (or nullptr).
    void step(double dt, const terrain::Heightfield1D* terrain) noexcept;

    /// @brief Spawns a forward kinetic missile.
    /// @param[in] origin Launch position in world coordinates.
    /// @param[in] heading Truck heading angle in radians.
    /// @param[in] baseVelocity Initial velocity of launcher vehicle in m/s.
    void fireRocket(const math::Vec2& origin, double heading, const math::Vec2& baseVelocity) noexcept;

    /// @brief Spawns a high-angle anti-air mortar shell.
    /// @param[in] origin Launch position in world coordinates.
    /// @param[in] baseVelocity Initial velocity of launcher vehicle in m/s.
    void fireMortar(const math::Vec2& origin, const math::Vec2& baseVelocity) noexcept;

    /// @brief Applies explosive blast impulses to nearby rigid bodies.
    /// @param[in] blast Detonation event parameters.
    /// @param[in,out] bodies Array of pointers to rigid bodies in scene.
    void applyBlast(const BlastEvent& blast, const std::vector<physics::RigidBody2D*>& bodies) noexcept;

    /// @brief Manually detonates a projectile at given index.
    /// @param[in] index Index of projectile in active collection.
    void detonate(std::size_t index) noexcept;

    /// @brief Detonates a projectile at given index at specified world impact point.
    /// @param[in] index Index of projectile in active collection.
    /// @param[in] impactPoint Epicenter coordinate of detonation.
    void detonateAt(std::size_t index, const math::Vec2& impactPoint) noexcept;

    /// @brief Gets constant reference to active projectiles.
    /// @return Vector of active ProjectileState instances.
    [[nodiscard]] const std::vector<ProjectileState>& getProjectiles() const noexcept;

    /// @brief Consumes and clears pending detonation events from current step.
    /// @return Vector of BlastEvent instances.
    [[nodiscard]] std::vector<BlastEvent> consumeBlasts() noexcept;

    /// @brief Gets count of active in-flight projectiles.
    /// @return Active projectile count.
    [[nodiscard]] std::size_t getActiveCount() const noexcept;

    /// @brief Clears all active projectiles and pending blast events.
    void reset() noexcept;

    /// @brief Gets rocket ballistic configuration.
    /// @return Constant reference to ProjectileConfig.
    [[nodiscard]] const ProjectileConfig& getRocketConfig() const noexcept;

    /// @brief Sets rocket ballistic configuration.
    /// @param[in] config Rocket parameters.
    void setRocketConfig(const ProjectileConfig& config) noexcept;

    /// @brief Gets mortar ballistic configuration.
    /// @return Constant reference to ProjectileConfig.
    [[nodiscard]] const ProjectileConfig& getMortarConfig() const noexcept;

    /// @brief Sets mortar ballistic configuration.
    /// @param[in] config Mortar parameters.
    void setMortarConfig(const ProjectileConfig& config) noexcept;

private:
    std::vector<ProjectileState> projectiles_ {};
    std::vector<BlastEvent> pendingBlasts_ {};
    ProjectileConfig rocketConfig_ {};
    ProjectileConfig mortarConfig_ {};
};

} // namespace vehicle_sim::dynamics
