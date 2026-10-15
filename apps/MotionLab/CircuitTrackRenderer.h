#pragma once

#include <SDL2/SDL.h>

#include "apps/MotionLab/CircuitTrackCoordinator.h"
#include "apps/MotionLab/MotionLabCamera.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @class CircuitTrackRenderer
/// @brief Renders procedural circuit track wireframe ribbon, kerbs, gates, and radar HUD.
class CircuitTrackRenderer {
public:
    /// @brief Default constructor.
    CircuitTrackRenderer() noexcept = default;

    /// @brief Renders track road boundaries, dashed centerline, kerbs, and timing gates.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] camera Active camera tracker and coordinate projection.
    /// @param[in] coordinator Circuit track simulation coordinator.
    static void renderTrack(SDL_Renderer* renderer,
                            const MotionLabCamera& camera,
                            const CircuitTrackCoordinator& coordinator) noexcept;

    /// @brief Renders HUD radar mini-map showing full circuit loop and player blip.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] coordinator Circuit track simulation coordinator.
    /// @param[in] vehiclePos World coordinates of player vehicle.
    /// @param[in] screenX Top-left X coordinate of radar on screen.
    /// @param[in] screenY Top-left Y coordinate of radar on screen.
    /// @param[in] radarWidth Width of radar box in pixels.
    /// @param[in] radarHeight Height of radar box in pixels.
    static void renderRadar(SDL_Renderer* renderer,
                            const CircuitTrackCoordinator& coordinator,
                            const vehicle_sim::math::Vec2& vehiclePos,
                            int screenX,
                            int screenY,
                            int radarWidth,
                            int radarHeight) noexcept;

private:
    static void drawRibbon(SDL_Renderer* renderer,
                           const MotionLabCamera& camera,
                           const vehicle_sim::terrain::CircuitTrack2D& track) noexcept;

    static void drawKerbs(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const vehicle_sim::terrain::CircuitTrack2D& track) noexcept;

    static void drawGates(SDL_Renderer* renderer,
                          const MotionLabCamera& camera,
                          const vehicle_sim::terrain::CircuitTrack2D& track) noexcept;
};

} // namespace motion_lab
