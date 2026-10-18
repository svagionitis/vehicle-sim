#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/ParticleSystem.h"
#include "libs/VehicleSimCore/dynamics/OffroadBallistics2D.h"
#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace montsoft_offroad {

/// @enum CombatTargetType
/// @brief Classification of destructible entities in the combat grid.
enum class CombatTargetType : std::uint8_t {
    GroundScout,     ///< Fast motorcycle or scout buggy.
    GroundHeavy,     ///< Armored crawler or tank.
    AerialFighter,   ///< Strafing jet fighter.
    AerialSaucer,    ///< Hovering saucer or drone square.
    TerrainObstacle, ///< Destructible boulder or barrier.
    BossComponent    ///< Boss weak-point or turret pod.
};

/// @struct CombatTargetAabb
/// @brief Axis-aligned collision bounding box and health state for combat targets.
struct CombatTargetAabb {
    std::uint32_t targetId { 0U };
    CombatTargetType type { CombatTargetType::GroundScout };
    vehicle_sim::math::Vec2 minBounds { 0.0, 0.0 };
    vehicle_sim::math::Vec2 maxBounds { 0.0, 0.0 };
    double health { 100.0 };
    bool isAlive { true };
};

/// @struct ShockwaveRing
/// @brief Visual animated expanding blast ring for CGA shockwaves.
struct ShockwaveRing {
    vehicle_sim::math::Vec2 worldEpicenter { 0.0, 0.0 };
    double currentRadius { 0.0 };
    double maxRadius { 3.5 };
    double age { 0.0 };
    double maxLifespan { 0.30 };
    ColorRgba color { 255, 255, 255, 255 };
    bool active { false };
};

/// @struct CombatManagerConfig
/// @brief Operational and cadence settings for the vehicle combat subsystem.
struct CombatManagerConfig {
    double rocketCooldownTime { 0.25 };     ///< Minimum delay between rocket shots in seconds (4 Hz).
    double mortarCooldownTime { 0.45 };     ///< Minimum delay between mortar shots in seconds (~2.2 Hz).
    double rocketRecoilImpulse { 360.0 };   ///< Backward recoil impulse on truck chassis in N*s.
    double mortarRecoilImpulse { 220.0 };   ///< Downward recoil impulse on suspension in N*s.
    double blastTraumaDistance { 20.0 };    ///< Distance threshold for camera shake induction in meters.
};

/// @class CombatManager
/// @brief Coordinates weapon firing, projectile flight, collision sweeps, and detonations.
/// @details Links vehicle simulation input with VehicleSimCore ballistics, enforces weapon reload cadences,
/// emits propellant trails, resolves explosive blast impulses, and triggers camera trauma.
class CombatManager final {
public:
    static constexpr std::size_t kMaxShockwaves { 16 };

    /// @brief Default constructor using standard combat configuration.
    CombatManager() noexcept;

    /// @brief Parameterized constructor using custom configuration.
    /// @param[in] config Combat subsystem configuration settings.
    explicit CombatManager(const CombatManagerConfig& config) noexcept;

    /// @brief Destructor.
    ~CombatManager() noexcept = default;

    /// @brief Advances active combat simulation by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] terrain Pointer to continuous heightfield for ground collision checks.
    /// @param[in,out] particleSystem Mutable reference to particle pool for exhaust/shrapnel.
    /// @param[in,out] camera Mutable reference to 2D camera for blast trauma induction.
    /// @param[in] rigidBodies Collection of scene bodies receiving blast impulses.
    /// @param[in,out] targets Optional collection of combat targets for hit testing.
    /// @param[in] stageIndex Active course stage index for biome styling.
    void update(
        double dt,
        const vehicle_sim::terrain::Heightfield1D* terrain,
        ParticleSystem& particleSystem,
        Camera2D& camera,
        const std::vector<vehicle_sim::physics::RigidBody2D*>& rigidBodies,
        std::vector<CombatTargetAabb>* targets = nullptr,
        std::size_t stageIndex = 0U
    ) noexcept;

    /// @brief Processes driver weapon firing inputs and triggers launches when off cooldown.
    /// @param[in] input Active driver control input state.
    /// @param[in,out] truckBody Truck rigid body chassis receiving launch recoil forces.
    /// @param[in] truckDynamics Active truck dynamics state for axle and speed queries.
    /// @param[in,out] camera Mutable reference to camera for firing trauma.
    void handleFireInputs(
        const vehicle_sim::dynamics::OffroadTruckInput& input,
        vehicle_sim::physics::RigidBody2D& truckBody,
        const vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics,
        Camera2D& camera
    ) noexcept;

    /// @brief Spawns a forward kinetic missile along truck heading.
    /// @param[in,out] truckBody Vehicle chassis receiving launch reaction impulse.
    /// @return True if weapon was successfully discharged, false if on cooldown.
    bool fireRocket(vehicle_sim::physics::RigidBody2D& truckBody) noexcept;

    /// @brief Spawns a high-angle anti-air mortar shell.
    /// @param[in,out] truckBody Vehicle chassis receiving launch reaction impulse.
    /// @return True if weapon was successfully discharged, false if on cooldown.
    bool fireMortar(vehicle_sim::physics::RigidBody2D& truckBody) noexcept;

    /// @brief Rasterizes all active rockets, mortar shells, and shockwave rings.
    /// @param[in,out] renderer Target CGA palette renderer.
    /// @param[in] camera Camera tracker performing world-to-screen coordinate mapping.
    void render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;

    /// @brief Resets all weapon cooldowns, clears active projectiles and pending blasts.
    void reset() noexcept;

    /// @brief Gets underlying ballistics physics coordinator.
    /// @return Constant reference to OffroadBallistics2D.
    [[nodiscard]] const vehicle_sim::dynamics::OffroadBallistics2D& getBallistics() const noexcept;

    /// @brief Gets mutable underlying ballistics physics coordinator.
    /// @return Mutable reference to OffroadBallistics2D.
    [[nodiscard]] vehicle_sim::dynamics::OffroadBallistics2D& getBallistics() noexcept;

    /// @brief Gets remaining reload cooldown for primary forward rockets in seconds.
    /// @return Remaining cooldown time in seconds (0.0 means ready).
    [[nodiscard]] double getRocketCooldown() const noexcept;

    /// @brief Gets remaining reload cooldown for secondary anti-air mortars in seconds.
    /// @return Remaining cooldown time in seconds (0.0 means ready).
    [[nodiscard]] double getMortarCooldown() const noexcept;

    /// @brief Queries whether primary rocket is ready to fire.
    /// @return True if reload timer is zero.
    [[nodiscard]] bool canFireRocket() const noexcept;

    /// @brief Queries whether secondary mortar is ready to fire.
    /// @return True if reload timer is zero.
    [[nodiscard]] bool canFireMortar() const noexcept;

    /// @brief Gets count of active visual shockwave rings.
    /// @return Count of active shockwave instances.
    [[nodiscard]] std::size_t getShockwaveCount() const noexcept;

    /// @brief Gets combat configuration settings.
    /// @return Constant reference to CombatManagerConfig.
    [[nodiscard]] const CombatManagerConfig& getConfig() const noexcept;

private:
    void updateCooldowns(double dt) noexcept;
    void emitExhaustTrails(ParticleSystem& particleSystem) noexcept;
    void processDetonations(
        ParticleSystem& particleSystem,
        Camera2D& camera,
        const std::vector<vehicle_sim::physics::RigidBody2D*>& rigidBodies,
        std::size_t stageIndex
    ) noexcept;
    void performSweepCollisions(std::vector<CombatTargetAabb>& targets) noexcept;
    void spawnShockwave(const vehicle_sim::math::Vec2& epicenter, double radius) noexcept;
    void updateShockwaves(double dt) noexcept;

    CombatManagerConfig config_ {};
    vehicle_sim::dynamics::OffroadBallistics2D ballistics_ {};
    double rocketTimer_ { 0.0 };
    double mortarTimer_ { 0.0 };
    std::array<ShockwaveRing, kMaxShockwaves> shockwaves_ {};
    std::size_t nextShockwaveSlot_ { 0 };
};

} // namespace montsoft_offroad
