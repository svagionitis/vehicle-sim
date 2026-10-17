#pragma once

#include <SDL2/SDL.h>
#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/SkidpadCoordinator.h"

namespace motion_lab {

/// @class SkidpadRenderer
/// @brief Vector wireframe renderer for skidpad circles, radial spokes, pylons, and handling HUD.
class SkidpadRenderer {
public:
    /// @brief Default constructor.
    SkidpadRenderer() noexcept = default;

    /// @brief Renders all skidpad ground elements into the active SDL viewport.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera projection.
    /// @param[in] coordinator Active Skidpad coordinator.
    static void renderCourse(SDL_Renderer* renderer,
                             const MotionLabCamera& camera,
                             const SkidpadCoordinator& coordinator) noexcept;

    /// @brief Renders dedicated Skidpad HUD card and live handling graph.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] coordinator Active Skidpad coordinator.
    /// @param[in] screenX Top-left X pixel coordinate.
    /// @param[in] screenY Top-left Y pixel coordinate.
    static void renderHud(SDL_Renderer* renderer,
                          const SkidpadCoordinator& coordinator,
                          int screenX,
                          int screenY) noexcept;

private:
    static void drawConcentricRings(SDL_Renderer* renderer,
                                    const MotionLabCamera& camera,
                                    const SkidpadCoordinator& coordinator) noexcept;

    static void drawRadialSpokes(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const SkidpadCoordinator& coordinator) noexcept;

    static void drawStagingChute(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const SkidpadCoordinator& coordinator) noexcept;

    static void drawPylons(SDL_Renderer* renderer,
                           const MotionLabCamera& camera,
                           const SkidpadCoordinator& coordinator) noexcept;

    static void drawPylon(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const SkidpadPylon& pylon) noexcept;

    static void drawDynamicGuide(SDL_Renderer* renderer,
                                 const MotionLabCamera& camera,
                                 const SkidpadCoordinator& coordinator) noexcept;

    static void drawHandlingPlot(SDL_Renderer* renderer,
                                 const SkidpadMetrics& metrics,
                                 int plotX,
                                 int plotY,
                                 int width,
                                 int height) noexcept;
};

} // namespace motion_lab
