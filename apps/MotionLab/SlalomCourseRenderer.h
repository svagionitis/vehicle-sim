#pragma once

#include <SDL2/SDL.h>
#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/SlalomCourseCoordinator.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @class SlalomCourseRenderer
/// @brief Vector wireframe renderer for 100-meter slalom course, pylons, and HUD.
class SlalomCourseRenderer {
public:
    /// @brief Default constructor.
    SlalomCourseRenderer() noexcept = default;

    /// @brief Renders all course elements into the active SDL viewport.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera projection.
    /// @param[in] coordinator Active slalom course coordinator.
    static void renderCourse(SDL_Renderer* renderer,
                             const MotionLabCamera& camera,
                             const SlalomCourseCoordinator& coordinator) noexcept;

    /// @brief Renders the dedicated Slalom HUD card.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] coordinator Active slalom course coordinator.
    /// @param[in] screenX Top-left X pixel coordinate.
    /// @param[in] screenY Top-left Y pixel coordinate.
    static void renderHud(SDL_Renderer* renderer,
                          const SlalomCourseCoordinator& coordinator,
                          int screenX,
                          int screenY) noexcept;

private:
    static void drawPylon(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const SlalomPylon& pylon) noexcept;

    static void drawGates(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const SlalomCourseCoordinator& coordinator) noexcept;

    static void drawWeaveGuides(SDL_Renderer* renderer,
                                const MotionLabCamera& camera,
                                const SlalomCourseCoordinator& coordinator) noexcept;
};

} // namespace motion_lab
