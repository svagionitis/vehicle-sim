#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/CombatManager.h"
#include "apps/MontsoftOffroad/ParticleSystem.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace montsoft_offroad {

/// @enum BossPhase
/// @brief Primary progression state of the Stage 8 boss encounter.
enum class BossPhase : std::uint8_t {
    Inactive,             ///< Player has not crossed arena threshold.
    MechHeadEntrance,     ///< Giant Mech Head emerges from ground bedrock.
    MechHeadCombat,       ///< Giant Mech Head actively fighting.
    MechHeadDefeated,     ///< Mech Head undergoing explosive destruction.
    CruiserDescent,       ///< Dreadnought Battlecruiser descending from sky.
    CruiserCombat,        ///< Dreadnought Battlecruiser actively fighting.
    CruiserDefeated,      ///< Dreadnought core overload and disintegration.
    VictoryCleared        ///< Boss fully neutralized; Stage 8 clear.
};

/// @enum MechHeadAttack
/// @brief Discrete combat attack modes for the Giant Mech Head.
enum class MechHeadAttack : std::uint8_t {
    Idle,                 ///< Stalking truck position, charging next attack.
    EyeLaserTelegraph,    ///< Eye shutters open with aiming reticle.
    EyeLaserDischarge,    ///< Dual continuous horizontal beams firing.
    MouthOpenExposed,     ///< Mouth hatch open, vulnerable to rockets.
    ClusterBombDischarge, ///< Vomiting rolling terrain bombs.
    SonicRoarWave         ///< Concussive suspension-compressing roar.
};

/// @struct BossSubComponent
/// @brief State and damage metrics for a destructible boss sub-assembly.
struct BossSubComponent {
    std::uint32_t targetId { 0U };
    vehicle_sim::math::Vec2 relativeOffset { 0.0, 0.0 };
    vehicle_sim::math::Vec2 dimensions { 2.0, 2.0 };
    double health { 100.0 };
    double maxHealth { 100.0 };
    double flashTimer { 0.0 };
    bool isExposed { true };
    bool isDestroyed { false };
};

/// @struct BossClusterBomb
/// @brief Rolling terrain explosive expelled by the Giant Mech Head.
struct BossClusterBomb {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };
    double age { 0.0 };
    double maxLifespan { 5.0 };
    double radius { 0.6 };
    bool active { false };
};

/// @struct BossLaserBeam
/// @brief High-energy horizontal beam emitted by Mech Head eye sensors.
struct BossLaserBeam {
    double originX { 0.0 };
    double targetX { 0.0 };
    double elevationY { 0.0 };
    double thickness { 0.35 };
    double duration { 0.0 };
    bool active { false };
};

/// @struct BossFlakShell
/// @brief Ballistic flak shell discharged by the Dreadnought Cruiser.
struct BossFlakShell {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };
    double age { 0.0 };
    double maxLifespan { 4.0 };
    double damage { 25.0 };
    bool active { false };
};

/// @struct BossControllerConfig
/// @brief Tuning parameters for arena geometry and boss health scaling.
struct BossControllerConfig {
    double arenaTriggerX { 435.0 };       ///< World X triggering arena entrance.
    double arenaMinX { 422.0 };           ///< Left boundary preventing backward retreat.
    double arenaMaxX { 495.0 };           ///< Right boundary past the boss anchor.
    double cameraMinX { 425.0 };          ///< Left viewport camera clamp in meters.
    double cameraMaxX { 485.0 };          ///< Right viewport camera clamp in meters.
    double mechHeadMaxHealth { 400.0 };   ///< Total vitality of Giant Mech Head.
    double cruiserTurretHealth { 150.0 }; ///< Vitality of each Dreadnought pod.
    double cruiserCoreHealth { 300.0 };   ///< Vitality of exposed Reactor Core.
};

/// @class BossController
/// @brief Orchestrates Stage 8 multi-phase boss encounters, attacks, and rendering.
/// @details Coordinates the Giant Mech Head and Dreadnought Battlecruiser duels, manages
/// camera containment, evaluates weak points, and drives retro CGA visual presentation.
class BossController final {
public:
    static constexpr std::uint32_t kTargetMechMouth { 1001U };
    static constexpr std::uint32_t kTargetMechEyes { 1002U };
    static constexpr std::uint32_t kTargetCruiserPort { 2001U };
    static constexpr std::uint32_t kTargetCruiserStarboard { 2002U };
    static constexpr std::uint32_t kTargetCruiserCore { 2003U };

    /// @brief Default constructor.
    BossController() noexcept;

    /// @brief Parameterized constructor using custom settings.
    /// @param[in] config Subsystem tuning configuration.
    explicit BossController(const BossControllerConfig& config) noexcept;

    /// @brief Destructor.
    ~BossController() noexcept = default;

    /// @brief Advances active boss state, attack timers, and kinematics.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] terrain Pointer to continuous terrain heightfield.
    /// @param[in,out] camera Mutable reference to 2D camera tracker.
    /// @param[in,out] truckBody Player truck chassis receiving physical impacts.
    /// @param[in,out] particleSystem Particle pool for detonations and sparks.
    void update(
        double dt,
        const vehicle_sim::terrain::Heightfield1D* terrain,
        Camera2D& camera,
        vehicle_sim::physics::RigidBody2D& truckBody,
        ParticleSystem& particleSystem
    ) noexcept;

    /// @brief Populates combat target list with vulnerable boss hitboxes.
    /// @param[out] targets Destination vector for active bounding boxes.
    void syncCombatTargets(std::vector<CombatTargetAabb>& targets) const noexcept;

    /// @brief Applies damage inflicted by player weapons back to boss components.
    /// @param[in] targets Modified target bounding boxes from CombatManager.
    /// @param[in,out] particleSystem Particle pool for explosion shrapnel.
    void applyCombatDamage(
        const std::vector<CombatTargetAabb>& targets,
        ParticleSystem& particleSystem
    ) noexcept;

    /// @brief Rasterizes boss entities, attack beams, and HUD vitality bar.
    /// @param[in,out] renderer Target CGA palette renderer.
    /// @param[in] camera Camera tracker performing coordinate mapping.
    void render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;

    /// @brief Resets boss encounter to initial inactive state.
    void reset() noexcept;

    /// @brief Triggers manual activation of boss encounter (for testing).
    /// @param[in] phase Starting boss phase.
    void triggerBossEncounter(BossPhase phase = BossPhase::MechHeadEntrance) noexcept;

    /// @brief Gets active boss phase.
    /// @return Current BossPhase enumeration value.
    [[nodiscard]] BossPhase getActivePhase() const noexcept;

    /// @brief Queries whether boss encounter is currently active.
    /// @return True if in any active entrance or combat state.
    [[nodiscard]] bool isBossActive() const noexcept;

    /// @brief Queries whether Stage 8 boss has been completely defeated.
    /// @return True if VictoryCleared state attained.
    [[nodiscard]] bool isBossDefeated() const noexcept;

    /// @brief Gets current health fraction of the active boss entity.
    /// @return Value in [0.0, 1.0].
    [[nodiscard]] double getHealthFraction() const noexcept;

    /// @brief Gets active configuration parameters.
    /// @return Constant reference to BossControllerConfig.
    [[nodiscard]] const BossControllerConfig& getConfig() const noexcept;

    /// @brief Gets world coordinate of Giant Mech Head anchor.
    /// @return 2D vector coordinate in meters.
    [[nodiscard]] const vehicle_sim::math::Vec2& getMechHeadPos() const noexcept;

    /// @brief Gets world coordinate of Dreadnought Cruiser center.
    /// @return 2D vector coordinate in meters.
    [[nodiscard]] const vehicle_sim::math::Vec2& getCruiserPos() const noexcept;

    /// @brief Queries whether the Dreadnought energy barrier is currently active.
    /// @return True if reactor core is shielded.
    [[nodiscard]] bool isShieldActive() const noexcept;

    /// @brief Gets current total remaining hit points of active boss.
    /// @return Remaining hit points.
    [[nodiscard]] double getRemainingBossHp() const noexcept;

private:
    void updateMechHead(double dt, const vehicle_sim::terrain::Heightfield1D* terrain, vehicle_sim::physics::RigidBody2D& truckBody, ParticleSystem& particleSystem, Camera2D& camera) noexcept;
    void updateCruiser(double dt, vehicle_sim::physics::RigidBody2D& truckBody, ParticleSystem& particleSystem, Camera2D& camera) noexcept;
    void enforceArenaBounds(vehicle_sim::physics::RigidBody2D& truckBody) noexcept;
    void fireEyeLasers() noexcept;
    void vomitClusterBombs() noexcept;
    void emitSonicRoar(vehicle_sim::physics::RigidBody2D& truckBody, Camera2D& camera) noexcept;
    void fireCruiserFlak() noexcept;

    void renderMechHead(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;
    void renderCruiser(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;
    void renderBombsAndBeams(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;
    void renderBossHud(CgaPaletteRenderer& renderer) const noexcept;

    BossControllerConfig config_ {};
    BossPhase currentPhase_ { BossPhase::Inactive };
    MechHeadAttack mechAttack_ { MechHeadAttack::Idle };

    vehicle_sim::math::Vec2 mechHeadPos_ { 480.0, 0.0 };
    vehicle_sim::math::Vec2 cruiserPos_ { 460.0, 22.0 };
    vehicle_sim::math::Vec2 cruiserVelocity_ { 0.0, 0.0 };

    double phaseTimer_ { 0.0 };
    double attackTimer_ { 0.0 };
    double flakTimer_ { 0.0 };
    double deathExplosionTimer_ { 0.0 };
    double hoverTime_ { 0.0 };

    BossSubComponent mechMouth_ {};
    BossSubComponent mechEyes_ {};
    BossSubComponent cruiserPort_ {};
    BossSubComponent cruiserStarboard_ {};
    BossSubComponent cruiserCore_ {};

    static constexpr std::size_t kMaxClusterBombs { 8 };
    static constexpr std::size_t kMaxFlakShells { 12 };
    std::array<BossClusterBomb, kMaxClusterBombs> clusterBombs_ {};
    std::array<BossLaserBeam, 2> eyeLasers_ {};
    std::array<BossFlakShell, kMaxFlakShells> flakShells_ {};

    bool arenaLocked_ { false };
    bool attackToggle_ { false };
};

} // namespace montsoft_offroad
