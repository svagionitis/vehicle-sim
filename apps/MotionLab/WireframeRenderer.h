#pragma once

#include <SDL2/SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/MotionLabVehicleType.h"
#include "apps/MotionLab/SkidRibbonBuffer.h"
#include "apps/MotionLab/TrailParticleBuffer.h"
#include "libs/VehicleSimCore/dynamics/AntiGravityRacerDynamics.h"
#include "libs/VehicleSimCore/dynamics/ArticulatedTruckDynamics.h"
#include "libs/VehicleSimCore/dynamics/DriftCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/ForkliftDynamics.h"
#include "libs/VehicleSimCore/dynamics/HelicopterDynamics.h"
#include "libs/VehicleSimCore/dynamics/HovercraftDynamics.h"
#include "libs/VehicleSimCore/dynamics/MonsterCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/QuadcopterDroneDynamics.h"
#include "libs/VehicleSimCore/dynamics/RaceCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/RocketDynamics.h"
#include "libs/VehicleSimCore/dynamics/TrackedTankDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace motion_lab {

/// @struct VehicleRenderContext
/// @brief Aggregated vehicle references and animation states for wireframe rendering.
struct VehicleRenderContext {
    MotionLabVehicleType type { MotionLabVehicleType::ClassicCar }; ///< Target vehicle simulation model.
    vehicle_sim::terrain::MaterialType terrain { vehicle_sim::terrain::MaterialType::Tarmac }; ///< Active surface terrain.
    const vehicle_sim::dynamics::MotionLabCarDynamics* car { nullptr }; ///< Classic car dynamics pointer.
    const vehicle_sim::dynamics::RaceCarDynamics* raceCar { nullptr }; ///< Race car dynamics pointer.
    const vehicle_sim::dynamics::RocketDynamics* rocket { nullptr }; ///< Rocket dynamics pointer.
    const vehicle_sim::dynamics::HelicopterDynamics* helicopter { nullptr }; ///< Helicopter dynamics pointer.
    const vehicle_sim::dynamics::HovercraftDynamics* hovercraft { nullptr }; ///< Hovercraft dynamics pointer.
    const vehicle_sim::dynamics::TrackedTankDynamics* tank { nullptr }; ///< Tank dynamics pointer.
    const vehicle_sim::dynamics::AntiGravityRacerDynamics* antiGravity { nullptr }; ///< Anti-gravity racer pointer.
    const vehicle_sim::dynamics::DriftCarDynamics* driftCar { nullptr }; ///< Drift car dynamics pointer.
    const vehicle_sim::dynamics::MonsterCarDynamics* monsterCar { nullptr }; ///< Monster truck dynamics pointer.
    const vehicle_sim::dynamics::ArticulatedTruckDynamics* articulatedTruck { nullptr }; ///< Semi-truck dynamics pointer.
    const vehicle_sim::dynamics::ForkliftDynamics* forklift { nullptr }; ///< Forklift dynamics pointer.
    const vehicle_sim::dynamics::QuadcopterDroneDynamics* quadcopter { nullptr }; ///< Quadcopter drone pointer.
    double animTimer { 0.0 }; ///< Continuous animation timer in seconds.
};

/// @class WireframeRenderer
/// @brief Hardware-accelerated 2D vector renderer for Motion Lab simulation.
/// @details Draws infinite 5m coordinate grid, anti-aliased wireframe vehicles (classic car,
/// formula race car, rocket, helicopter, hovercraft, tracked tank), velocity vector arrows,
/// fading skidmarks, and arena barriers.
class WireframeRenderer {
public:
    /// @brief Default constructor.
    WireframeRenderer() noexcept = default;

    /// @brief Renders the complete simulation scene using generalized vehicle render context.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera tracker and world-to-screen projection.
    /// @param[in] context Active vehicle render context and dynamics pointers.
    /// @param[in] body Active vehicle chassis rigid body.
    /// @param[in] skids Active skidmark segment buffer.
    /// @param[in] trail Active breadcrumb trail dot buffer.
    /// @param[in] barrierActive True if arena barrier boundary is enabled.
    void renderScene(SDL_Renderer* renderer,
                     const MotionLabCamera& camera,
                     const VehicleRenderContext& context,
                     const vehicle_sim::physics::RigidBody2D& body,
                     const SkidRibbonBuffer& skids,
                     const TrailParticleBuffer& trail,
                     bool barrierActive) noexcept;

    /// @brief Backward-compatible scene render using classic car dynamics.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera tracker and world-to-screen projection.
    /// @param[in] car Active car dynamics coordinator.
    /// @param[in] body Active car chassis rigid body.
    /// @param[in] skids Active skidmark segment buffer.
    /// @param[in] trail Active breadcrumb trail dot buffer.
    /// @param[in] barrierActive True if arena barrier boundary is enabled.
    void renderScene(SDL_Renderer* renderer,
                     const MotionLabCamera& camera,
                     const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                     const vehicle_sim::physics::RigidBody2D& body,
                     const SkidRibbonBuffer& skids,
                     const TrailParticleBuffer& trail,
                     bool barrierActive) noexcept;

    /// @brief Draws a screen-space line with optional anti-aliasing / thickness.
    /// @param[in] renderer Active SDL renderer.
    /// @param[in] x1 Start pixel X coordinate.
    /// @param[in] y1 Start pixel Y coordinate.
    /// @param[in] x2 End pixel X coordinate.
    /// @param[in] y2 End pixel Y coordinate.
    /// @param[in] color Line RGBA color.
    static void drawLine(SDL_Renderer* renderer,
                         int x1, int y1,
                         int x2, int y2,
                         SDL_Color color) noexcept;

    /// @brief Draws a directional arrow from start to end coordinates.
    /// @param[in] renderer Active SDL renderer.
    /// @param[in] screenStart Arrow origin screen coordinates.
    /// @param[in] screenEnd Arrow tip screen coordinates.
    /// @param[in] color Arrow RGBA color.
    static void drawArrow(SDL_Renderer* renderer,
                          const vehicle_sim::math::Vec2& screenStart,
                          const vehicle_sim::math::Vec2& screenEnd,
                          SDL_Color color) noexcept;

private:
    void drawGrid(SDL_Renderer* renderer, const MotionLabCamera& camera, SDL_Color gridColor) noexcept;
    void drawSkids(SDL_Renderer* renderer, const MotionLabCamera& camera, const SkidRibbonBuffer& skids, SDL_Color skidColor) noexcept;
    void drawTrail(SDL_Renderer* renderer, const MotionLabCamera& camera, const TrailParticleBuffer& trail) noexcept;
    void drawBarrier(SDL_Renderer* renderer, const MotionLabCamera& camera) noexcept;

    void drawClassicCar(SDL_Renderer* renderer,
                        const MotionLabCamera& camera,
                        const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                        const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawRaceCar(SDL_Renderer* renderer,
                     const MotionLabCamera& camera,
                     const vehicle_sim::dynamics::RaceCarDynamics& raceCar,
                     const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawRocket(SDL_Renderer* renderer,
                    const MotionLabCamera& camera,
                    const vehicle_sim::dynamics::RocketDynamics& rocket,
                    const vehicle_sim::physics::RigidBody2D& body,
                    double animTimer) noexcept;

    void drawHelicopter(SDL_Renderer* renderer,
                        const MotionLabCamera& camera,
                        const vehicle_sim::dynamics::HelicopterDynamics& helicopter,
                        const vehicle_sim::physics::RigidBody2D& body,
                        double animTimer) noexcept;

    void drawHovercraft(SDL_Renderer* renderer,
                        const MotionLabCamera& camera,
                        const vehicle_sim::dynamics::HovercraftDynamics& hovercraft,
                        const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawTrackedTank(SDL_Renderer* renderer,
                         const MotionLabCamera& camera,
                         const vehicle_sim::dynamics::TrackedTankDynamics& tank,
                         const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawAntiGravityRacer(SDL_Renderer* renderer,
                              const MotionLabCamera& camera,
                              const vehicle_sim::dynamics::AntiGravityRacerDynamics& antiGravity,
                              const vehicle_sim::physics::RigidBody2D& body,
                              double animTimer) noexcept;

    void drawDriftCar(SDL_Renderer* renderer,
                      const MotionLabCamera& camera,
                      const vehicle_sim::dynamics::DriftCarDynamics& driftCar,
                      const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawMonsterTruck(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const vehicle_sim::dynamics::MonsterCarDynamics& monsterCar,
                          const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawArticulatedTruck(SDL_Renderer* renderer,
                              const MotionLabCamera& camera,
                              const vehicle_sim::dynamics::ArticulatedTruckDynamics& truck,
                              const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawForklift(SDL_Renderer* renderer,
                      const MotionLabCamera& camera,
                      const vehicle_sim::dynamics::ForkliftDynamics& forklift,
                      const vehicle_sim::physics::RigidBody2D& body) noexcept;

    void drawQuadcopterDrone(SDL_Renderer* renderer,
                             const MotionLabCamera& camera,
                             const vehicle_sim::dynamics::QuadcopterDroneDynamics& drone,
                             const vehicle_sim::physics::RigidBody2D& body,
                             double animTimer) noexcept;
};

} // namespace motion_lab
