#pragma once

#include <array>
#include <cstddef>
#include <SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/WindTurbulenceCoordinator.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @struct StreamlineParticle
/// @brief Zero-allocation drifting visual wind streamline and wake ribbon particle.
struct StreamlineParticle {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };
    vehicle_sim::math::Vec2 history[3] { };
    vehicle_sim::math::Vec2 wakeDrift { 0.0, 0.0 };
    double lifeRemaining { 0.0 };
    double totalLifetime { 1.5 };
    float intensity { 1.0f };
    bool isWake { false };
};

/// @class WindVectorFieldRenderer
/// @brief High-performance wireframe renderer for wind vector fields, streamlines, and HUD windsock.
class WindVectorFieldRenderer {
public:
    static constexpr std::size_t kMaxStreamlines { 384U };
    static constexpr int kCardWidth { 228 };
    static constexpr int kCardHeight { 244 };

    WindVectorFieldRenderer() noexcept;

    /// @brief Updates positions, history ribbons, and lifetimes of streamline & wake particles.
    /// @param[in] coordinator Wind coordinator providing velocity queries.
    /// @param[in] camera Active viewport camera.
    /// @param[in] dt Frame render timestep in seconds.
    /// @param[in] body Optional vehicle rigid body for aerodynamic wake generation.
    void updateParticles(const WindTurbulenceCoordinator& coordinator,
                         const MotionLabCamera& camera,
                         double dt,
                         const vehicle_sim::physics::RigidBody2D* body = nullptr) noexcept;

    /// @brief Renders viewport wind vector grid, curved streamlines, and turbulence eddies.
    /// @param[in] renderer Target SDL accelerated renderer.
    /// @param[in] coordinator Wind coordinator for velocity vectors.
    /// @param[in] camera Viewport camera for world-to-screen transforms.
    /// @param[in] body Optional vehicle rigid body for apparent wind and force arrows.
    void renderVectorField(SDL_Renderer* renderer,
                           const WindTurbulenceCoordinator& coordinator,
                           const MotionLabCamera& camera,
                           const vehicle_sim::physics::RigidBody2D* body = nullptr) const noexcept;

    /// @brief Computes collision-free screen coordinates for Wind HUD card.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    /// @param[in] circuitActive Whether procedural circuit track mode is active.
    /// @param[in] advancedHudActive Whether advanced telemetry HUD is visible.
    /// @return Collision-free top-left screen position (x, y).
    [[nodiscard]] static SDL_Point computeHudCardPos(
        int screenWidth,
        int screenHeight,
        bool circuitActive,
        bool advancedHudActive) noexcept;

    /// @brief Renders the dedicated retro HUD windsock, compass rose, and aero force cards.
    /// @param[in] renderer Target SDL renderer.
    /// @param[in] snapshot Wind telemetry snapshot.
    /// @param[in] originX Top-left X coordinate on screen.
    /// @param[in] originY Top-left Y coordinate on screen.
    static void renderHudCard(SDL_Renderer* renderer,
                              const WindTelemetrySnapshot& snapshot,
                              int originX, int originY) noexcept;

    /// @brief Toggles visibility of viewport vector grid and streamlines.
    void toggleOverlay() noexcept;

    /// @brief Sets overlay enabled state explicitly.
    /// @param[in] enabled True to display streamline overlay.
    void setOverlayEnabled(bool enabled) noexcept;

    /// @brief Queries if viewport vector overlay is enabled.
    [[nodiscard]] bool isOverlayEnabled() const noexcept;

private:
    void renderGridArrows(SDL_Renderer* renderer,
                          const WindTurbulenceCoordinator& coordinator,
                          const MotionLabCamera& camera) const noexcept;

    void renderEddySwirls(SDL_Renderer* renderer,
                          const WindTurbulenceCoordinator& coordinator,
                          const MotionLabCamera& camera) const noexcept;

    void renderStreamRibbons(SDL_Renderer* renderer,
                             const WindTurbulenceCoordinator& coordinator,
                             const MotionLabCamera& camera) const noexcept;

    void renderChassisAero(SDL_Renderer* renderer,
                           const WindTurbulenceCoordinator& coordinator,
                           const MotionLabCamera& camera,
                           const vehicle_sim::physics::RigidBody2D& body) const noexcept;

    std::array<StreamlineParticle, kMaxStreamlines> mParticles { };
    std::size_t mParticleCount { 0U };
    bool mShowOverlay { true };
    double mSpawnTimer { 0.0 };
    double mAnimTimer { 0.0 };
};

} // namespace motion_lab
