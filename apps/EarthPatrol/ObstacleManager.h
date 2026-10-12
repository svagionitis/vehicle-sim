#pragma once

#include <cstdint>
#include <vector>

#include <SDL2/SDL.h>

#include "apps/EarthPatrol/Camera2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {
class EarthRoverDynamics;
} // namespace vehicle_sim::dynamics

namespace vehicle_sim::terrain {
class Heightfield1D;
} // namespace vehicle_sim::terrain

namespace earth_patrol {

class CourseManager;

/// @enum ObstacleType
/// @brief Classification of surface hazards and roadblocks.
enum class ObstacleType : std::uint8_t {
    SmallRock, ///< Low rock bump that compresses suspension without fatal damage.
    LargeBoulder, ///< Heavy boulder that halts and crashes rover on direct chassis impact.
    FallenLog, ///< Low horizontal log trunk requiring timed clearance jump.
    ConcreteBarrier ///< Tall construction highway barrier causing fatal crash on collision.
};

/// @struct Obstacle
/// @brief Geometric definition and physical fatality attributes of a track hazard.
struct Obstacle {
    ObstacleType type { ObstacleType::SmallRock }; ///< Type of track obstacle.
    vehicle_sim::math::Vec2 worldPos { 0.0, 0.0 }; ///< Base contact position in world space.
    double width { 0.8 }; ///< Horizontal bounding dimension in meters.
    double height { 0.8 }; ///< Vertical bounding dimension in meters.
    bool fatal { false }; ///< True if direct collision destroys the rover.
    bool cleared { false }; ///< Flag indicating if rover passed without crashing.
};

/// @enum DynamicObstacleType
/// @brief Classification of moving terrestrial hazards.
enum class DynamicObstacleType : std::uint8_t {
    RollingBoulder, ///< Ground-conforming rolling rock with closing speed.
    PatrolDrone, ///< Oscillating roadwork drone with flashing beacon.
    TumblingDebris ///< Wind-driven hopping debris cluster.
};

/// @struct DynamicObstacle
/// @brief Moving obstacle with kinetic, rotational, and boundary state.
struct DynamicObstacle {
    DynamicObstacleType type { DynamicObstacleType::RollingBoulder }; ///< Hazard classification.
    vehicle_sim::math::Vec2 position { 0.0, 0.0 }; ///< Center position in world space.
    vehicle_sim::math::Vec2 velocity { 0.0, 0.0 }; ///< Velocity vector in meters per second.
    double radius { 0.40 }; ///< Physical and collision radius in meters.
    double rotationAngle { 0.0 }; ///< Current orientation angle in radians.
    double patrolMinX { 0.0 }; ///< Minimum boundary for patrol oscillation.
    double patrolMaxX { 0.0 }; ///< Maximum boundary for patrol oscillation.
    bool fatal { true }; ///< True if direct collision causes fatal crash.
    bool cleared { false }; ///< Flag indicating if rover passed without crashing.
};

/// @class ObstacleManager
/// @brief Manages placement, collision queries, and visual rasterization of course obstacles.
/// @details Spawns rocks, boulders, logs, and barriers at strategic intervals along the course,
/// updates dynamic moving hazards, and evaluates bounding volume intersection tests against rover.
class ObstacleManager {
public:
    /// @brief Default constructor.
    ObstacleManager() noexcept;

    /// @brief Generates obstacles adapted to course terrain elevation and active biome.
    /// @details Populates both static hazards and dynamic moving obstacles appropriate for the biome.
    /// @param[in] course Active course manager providing terrain elevation queries.
    /// @param[in] biomeIndex Active biome index.
    void generateObstacles(const CourseManager& course, int biomeIndex);

    /// @brief Updates kinematic positions and rotations of all dynamic obstacles.
    /// @details Conforms rolling and hopping hazards to the terrain surface elevation.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] terrain Heightfield terrain model.
    void updateDynamic(double dt, const vehicle_sim::terrain::Heightfield1D& terrain) noexcept;

    /// @brief Evaluates collision between rover and active obstacles or deep chasms.
    /// @details Checks static obstacles, dynamic hazards, and fatal drop thresholds.
    /// @param[in] body Rover chassis rigid body.
    /// @param[in] rover Rover dynamics coordinator.
    /// @param[out] outFatal True if collision causes fatal vehicle destruction.
    /// @return True if a collision was detected.
    [[nodiscard]] bool checkCollision(const vehicle_sim::physics::RigidBody2D& body,
        const vehicle_sim::dynamics::EarthRoverDynamics& rover, bool& outFatal) noexcept;

    /// @brief Evaluates collision specifically against dynamic obstacles.
    /// @details Performs circular bounding overlap tests against rover chassis AABB.
    /// @param[in] body Rover chassis rigid body.
    /// @param[out] outFatal True if collision causes fatal destruction.
    /// @return True if collision occurred.
    [[nodiscard]] bool checkDynamicColl(const vehicle_sim::physics::RigidBody2D& body,
        bool& outFatal) noexcept;

    /// @brief Renders visible obstacles in the camera viewport.
    /// @details Rasterizes static objects and rotated dynamic obstacles.
    /// @param[in] renderer Active SDL hardware renderer pointer.
    /// @param[in] camera 2D camera providing screen projection transform.
    void render(SDL_Renderer* renderer, const Camera2D& camera) const noexcept;

    /// @brief Resets cleared flags on all registered obstacles.
    /// @details Clears passing status on both static and dynamic obstacle collections.
    void reset() noexcept;

    /// @brief Gets total count of hazards successfully cleared without collision.
    /// @details Sums cleared counts across static and dynamic hazards.
    /// @return Cleared obstacle count.
    [[nodiscard]] int getClearedCount() const noexcept;

    /// @brief Gets collection of registered static obstacles.
    /// @details Provides immutable access to track hazards.
    /// @return Constant reference to vector of static obstacles.
    [[nodiscard]] const std::vector<Obstacle>& getObstacles() const noexcept;

    /// @brief Gets collection of registered dynamic obstacles.
    /// @details Provides immutable access to moving hazards.
    /// @return Constant reference to vector of dynamic obstacles.
    [[nodiscard]] const std::vector<DynamicObstacle>& getDynamicObstacles() const noexcept;

private:
    std::vector<Obstacle> obstacles_ { };
    std::vector<DynamicObstacle> dynamicObstacles_ { };
};

} // namespace earth_patrol
