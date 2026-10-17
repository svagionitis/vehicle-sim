#pragma once

#include <SDL2/SDL.h>
#include "apps/MotionLab/MooseTestCoordinator.h"
#include "apps/MotionLab/MotionLabCamera.h"

namespace motion_lab {

/// @class MooseTestRenderer
/// @brief Vector wireframe renderer for ISO 3888-2 severe lane-change corridor, pylons, and HUD.
class MooseTestRenderer {
public:
    /// @brief Default constructor.
    MooseTestRenderer() noexcept = default;

    /// @brief Renders all course elements into the active SDL viewport.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera projection.
    /// @param[in] coordinator Active Moose Test coordinator.
    static void renderCourse(SDL_Renderer* renderer,
                             const MotionLabCamera& camera,
                             const MooseTestCoordinator& coordinator) noexcept;

    /// @brief Renders the dedicated Moose Test HUD card.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] coordinator Active Moose Test coordinator.
    /// @param[in] screenX Top-left X pixel coordinate.
    /// @param[in] screenY Top-left Y pixel coordinate.
    static void renderHud(SDL_Renderer* renderer,
                          const MooseTestCoordinator& coordinator,
                          int screenX,
                          int screenY) noexcept;

private:
    static void drawPylon(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const MooseTestCone& cone) noexcept;

    static void drawGates(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const MooseTestCoordinator& coordinator) noexcept;

    static void drawBoundaries(SDL_Renderer* renderer,
                               const MotionLabCamera& camera,
                               const MooseTestCoordinator& coordinator) noexcept;

    static void drawChevrons(SDL_Renderer* renderer,
                             const MotionLabCamera& camera,
                             const MooseTestCoordinator& coordinator) noexcept;

    static void drawTrajectoryRoute(SDL_Renderer* renderer,
                                    const MotionLabCamera& camera,
                                    const MooseTestCoordinator& coordinator) noexcept;
};

} // namespace motion_lab
