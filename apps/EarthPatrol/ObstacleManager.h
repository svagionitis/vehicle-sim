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

/// @class ObstacleManager
/// @brief Manages placement, collision queries, and visual rasterization of course obstacles.
/// @details Spawns rocks, boulders, logs, and barriers at strategic intervals along the course,
/// and performs bounding volume intersection tests against rover chassis and wheels.
class ObstacleManager {
public:
    /// @brief Default constructor.
    ObstacleManager() noexcept;

    /// @brief Generates obstacles adapted to course terrain elevation and active biome.
    /// @param[in] course Active course manager providing terrain elevation queries.
    /// @param[in] biomeIndex Active biome index.
    void generateObstacles(const CourseManager& course, int biomeIndex);

    /// @brief Evaluates collision between rover and active obstacles or deep chasms.
    /// @param[in] body Rover chassis rigid body.
    /// @param[in] rover Rover dynamics coordinator.
    /// @param[out] outFatal True if collision causes fatal vehicle destruction.
    /// @return True if a collision was detected.
    [[nodiscard]] bool checkCollision(const vehicle_sim::physics::RigidBody2D& body,
        const vehicle_sim::dynamics::EarthRoverDynamics& rover, bool& outFatal) noexcept;

    /// @brief Renders visible obstacles in the camera viewport.
    /// @param[in] renderer Active SDL hardware renderer pointer.
    /// @param[in] camera 2D camera providing screen projection transform.
    void render(SDL_Renderer* renderer, const Camera2D& camera) const noexcept;

    /// @brief Resets cleared flags on all registered obstacles.
    void reset() noexcept;

    /// @brief Gets total count of hazards successfully cleared without collision.
    /// @return Cleared obstacle count.
    [[nodiscard]] int getClearedCount() const noexcept;

    /// @brief Gets collection of registered obstacles.
    /// @return Constant reference to vector of obstacles.
    [[nodiscard]] const std::vector<Obstacle>& getObstacles() const noexcept;

private:
    std::vector<Obstacle> obstacles_ { };
};

} // namespace earth_patrol
