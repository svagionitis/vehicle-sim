#pragma once

#include <SDL2/SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/SkidRibbonBuffer.h"
#include "apps/MotionLab/TrailParticleBuffer.h"
#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @class WireframeRenderer
/// @brief Hardware-accelerated 2D vector renderer for Motion Lab simulation.
/// @details Draws infinite 5m coordinate grid, anti-aliased wireframe car body,
/// steered tires, velocity vector arrows, fading skidmarks, and arena barriers.
class WireframeRenderer {
public:
    /// @brief Default constructor.
    WireframeRenderer() noexcept = default;

    /// @brief Renders the complete simulation scene into the SDL target renderer.
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
    static void drawLine(SDL_Renderer* renderer,
                         int x1, int y1,
                         int x2, int y2,
                         SDL_Color color) noexcept;

    /// @brief Draws a directional arrow from start to end coordinates.
    static void drawArrow(SDL_Renderer* renderer,
                          const vehicle_sim::math::Vec2& screenStart,
                          const vehicle_sim::math::Vec2& screenEnd,
                          SDL_Color color) noexcept;

private:
    void drawGrid(SDL_Renderer* renderer, const MotionLabCamera& camera, SDL_Color gridColor) noexcept;
    void drawSkids(SDL_Renderer* renderer, const MotionLabCamera& camera, const SkidRibbonBuffer& skids, SDL_Color skidColor) noexcept;
    void drawTrail(SDL_Renderer* renderer, const MotionLabCamera& camera, const TrailParticleBuffer& trail) noexcept;
    void drawBarrier(SDL_Renderer* renderer, const MotionLabCamera& camera) noexcept;
    void drawCar(SDL_Renderer* renderer,
                 const MotionLabCamera& camera,
                 const vehicle_sim::dynamics::MotionLabCarDynamics& car,
                 const vehicle_sim::physics::RigidBody2D& body) noexcept;
};

} // namespace motion_lab
