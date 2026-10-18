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

/// @enum EnemyType
/// @brief Classification of autonomous combat threats.
enum class EnemyType : std::uint8_t {
    ScoutDirtBike,     ///< Fast light ground motorcycle.
    RedChaseBuggy,     ///< Heavy aggressive ramming buggy.
    BuriedLandmine,    ///< Concealed ground proximity explosive.
    ArtilleryCrawler,  ///< Tracked armored vehicle firing mortar shells.
    StrafingJet,       ///< Sweeping high-speed aerial interceptor.
    HoveringSaucer,    ///< Medium-altitude saucer dropping energy bombs.
    BouncingDrone      ///< Sine-wave oscillating aerial drone square.
};

/// @struct EnemyConfig
/// @brief Physical and combat baseline properties for an enemy archetype.
struct EnemyConfig {
    double maxHealth { 50.0 };         ///< Total hit points.
    double baseSpeed { 15.0 };         ///< Nominal movement speed in m/s.
    double collisionRadius { 1.2 };    ///< Bounding radius for rough proximity.
    vehicle_sim::math::Vec2 dimensions { 2.0, 1.0 }; ///< Width and height for AABB.
    double attackCooldown { 2.0 };     ///< Delay between attack discharges in seconds.
    std::uint32_t scoreValue { 100U }; ///< Bounty points awarded upon destruction.
};

/// @struct EnemyProjectile
/// @brief Ordinance discharged by enemy combatants against the player.
struct EnemyProjectile {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };
    double damage { 20.0 };
    double blastRadius { 2.0 };
    double age { 0.0 };
    double maxLifespan { 4.0 };
    bool isBallistic { false };        ///< Subject to gravity if true.
    bool active { false };
};

/// @struct EnemyEntity
/// @brief Runtime state of an active autonomous hostile unit.
struct EnemyEntity {
    std::uint32_t id { 0U };
    EnemyType type { EnemyType::ScoutDirtBike };
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 };
    double orientation { 0.0 };        ///< Heading or pitch angle in radians.
    double health { 50.0 };
    double maxHealth { 50.0 };
    double attackTimer { 0.0 };
    double flashTimer { 0.0 };         ///< Hit-flash white highlight timer.
    double animTimer { 0.0 };          ///< Wheel or thruster animation clock.
    double sinePhase { 0.0 };          ///< Oscillation phase for drone squares.
    bool active { false };
};

/// @struct EnemyManagerConfig
/// @brief Tuning parameters for enemy formation spawning and culling.
struct EnemyManagerConfig {
    std::size_t maxEnemies { 32U };
    std::size_t maxProjectiles { 48U };
    double spawnLeadDistance { 35.0 };     ///< Spawn distance ahead of camera in meters.
    double spawnRearDistance { 22.0 };     ///< Spawn distance behind camera for buggies.
    double despawnDistance { 40.0 };      ///< Distance behind camera triggering cleanup.
};

/// @class EnemyManager
/// @brief Coordinates lifecycle, kinematics, AI behaviors, and rendering for all enemy units.
/// @details Links with CourseManager for milestone wave spawning, integrates with CombatManager
/// target lists, simulates hostile ordinance, and resolves player truck impacts.
class EnemyManager final {
public:
    /// @brief Default constructor.
    EnemyManager() noexcept;

    /// @brief Parameterized constructor using custom configuration.
    /// @param[in] config Subsystem tuning configuration.
    explicit EnemyManager(const EnemyManagerConfig& config) noexcept;

    /// @brief Destructor.
    ~EnemyManager() noexcept = default;

    /// @brief Advances enemy positions, AI timers, and weapon projectiles.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] terrain Pointer to continuous heightfield.
    /// @param[in] camera Camera tracker for frustum culling.
    /// @param[in,out] truckBody Player truck chassis receiving physical impacts.
    /// @param[in,out] particleSystem Particle pool for exhaust and explosion debris.
    void update(
        double dt,
        const vehicle_sim::terrain::Heightfield1D* terrain,
        const Camera2D& camera,
        vehicle_sim::physics::RigidBody2D& truckBody,
        ParticleSystem& particleSystem
    ) noexcept;

    /// @brief Spawns a single enemy entity at specified world coordinates.
    /// @param[in] type Archetype classification.
    /// @param[in] position Initial world coordinate.
    /// @param[in] initialVelocity Initial movement vector.
    /// @return True if successfully spawned, false if pool is full.
    bool spawnEnemy(
        EnemyType type,
        const vehicle_sim::math::Vec2& position,
        const vehicle_sim::math::Vec2& initialVelocity
    ) noexcept;

    /// @brief Spawns a coordinated attack formation based on course stage and milestone.
    /// @param[in] stageIndex Active course stage (0-7).
    /// @param[in] milestoneIndex Active milestone sector (0-4).
    /// @param[in] playerX Current horizontal coordinate of player truck.
    /// @param[in] terrain Pointer to continuous ground surface.
    void spawnFormation(
        std::size_t stageIndex,
        std::size_t milestoneIndex,
        double playerX,
        const vehicle_sim::terrain::Heightfield1D* terrain
    ) noexcept;

    /// @brief Populates combat target AABB collection for player weapon hit-testing.
    /// @param[out] targets Destination vector for active enemy bounding boxes.
    void syncCombatTargets(std::vector<CombatTargetAabb>& targets) const noexcept;

    /// @brief Applies damage inflicted by player weapons back to internal enemy entities.
    /// @param[in] targets Modified target bounding boxes from CombatManager.
    /// @param[in,out] particleSystem Particle pool for explosion shrapnel.
    /// @param[in] stageIndex Active stage for biome-specific debris colors.
    void applyCombatDamage(
        const std::vector<CombatTargetAabb>& targets,
        ParticleSystem& particleSystem,
        std::size_t stageIndex
    ) noexcept;

    /// @brief Rasterizes all active enemy units and incoming projectiles.
    /// @param[in,out] renderer Target CGA palette renderer.
    /// @param[in] camera Camera tracker performing world-to-screen coordinate mapping.
    void render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept;

    /// @brief Clears all active enemies, projectiles, and resets spawn clocks.
    void reset() noexcept;

    /// @brief Gets active enemy entity count.
    /// @return Number of currently active enemy units.
    [[nodiscard]] std::size_t getActiveCount() const noexcept;

    /// @brief Gets collection of all enemy entities.
    /// @return Vector of EnemyEntity instances.
    [[nodiscard]] const std::vector<EnemyEntity>& getEnemies() const noexcept;

    /// @brief Gets collection of all active enemy projectiles.
    /// @return Vector of EnemyProjectile instances.
    [[nodiscard]] const std::vector<EnemyProjectile>& getProjectiles() const noexcept;

    /// @brief Gets total accumulated score from destroyed enemies.
    /// @return Accumulated score points.
    [[nodiscard]] std::uint32_t getAccumulatedScore() const noexcept;

    /// @brief Queries whether player truck received fatal damage this tick.
    /// @return True if truck was destroyed by enemy attack.
    [[nodiscard]] bool checkTruckDestroyed() const noexcept;

    /// @brief Gets configuration settings.
    /// @return Constant reference to EnemyManagerConfig.
    [[nodiscard]] const EnemyManagerConfig& getConfig() const noexcept;

private:
    void updateGroundAi(EnemyEntity& enemy, double dt, const vehicle_sim::terrain::Heightfield1D* terrain, const vehicle_sim::math::Vec2& playerPos) noexcept;
    void updateAerialAi(EnemyEntity& enemy, double dt, const vehicle_sim::math::Vec2& playerPos) noexcept;
    void updateProjectiles(double dt, const vehicle_sim::terrain::Heightfield1D* terrain, vehicle_sim::physics::RigidBody2D& truckBody, ParticleSystem& particleSystem) noexcept;
    void resolveTruckContacts(vehicle_sim::physics::RigidBody2D& truckBody, ParticleSystem& particleSystem) noexcept;
    void fireEnemyBullet(const vehicle_sim::math::Vec2& origin, const vehicle_sim::math::Vec2& velocity, double damage) noexcept;
    void fireEnemyBomb(const vehicle_sim::math::Vec2& origin, const vehicle_sim::math::Vec2& velocity, double damage, double radius, bool isBallistic = false) noexcept;

    void renderDirtBike(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderChaseBuggy(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderLandmine(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderArtillery(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderStrafingJet(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderSaucer(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;
    void renderDroneSquare(CgaPaletteRenderer& renderer, const Camera2D& camera, const EnemyEntity& enemy) const noexcept;

    EnemyManagerConfig config_ {};
    std::vector<EnemyEntity> enemies_ {};
    std::vector<EnemyProjectile> projectiles_ {};
    std::array<EnemyConfig, 7> archetypeConfigs_ {};
    std::uint32_t nextEntityId_ { 1U };
    std::uint32_t accumulatedScore_ { 0U };
    bool truckDestroyed_ { false };
    double playerHealth_ { 100.0 };
    std::size_t lastSpawnedMilestone_ { 999U };
    std::size_t lastSpawnedStage_ { 999U };
};

} // namespace montsoft_offroad
