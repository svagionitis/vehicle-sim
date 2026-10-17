#pragma once

#include <SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/TireSmokeParticleBuffer.h"
#include "apps/MotionLab/TireThermalCoordinator.h"

namespace motion_lab {

/// @class TireThermalRenderer
/// @brief Renders vector wireframe tire thermal smoke and 4-corner HUD card.
class TireThermalRenderer {
public:
    static constexpr int kCardWidth { 228 };
    static constexpr int kCardHeight { 170 };

    /// @brief Default constructor with HUD initially enabled.
    TireThermalRenderer() noexcept = default;

    /// @brief Renders world-space expanding wireframe smoke particles.
    /// @param[in] renderer Target SDL hardware renderer.
    /// @param[in] smokeBuffer Buffer containing active smoke particles.
    /// @param[in] camera Active viewport camera for projection.
    void renderSmoke(SDL_Renderer* renderer,
                     const TireSmokeParticleBuffer& smokeBuffer,
                     const MotionLabCamera& camera) const noexcept;

    /// @brief Renders the 4-corner tire thermal diagnostics HUD card.
    /// @param[in] renderer Target SDL hardware renderer.
    /// @param[in] coordinator Tire thermal coordinator providing states.
    /// @param[in] originX Top-left X screen coordinate.
    /// @param[in] originY Top-left Y screen coordinate.
    void renderHudCard(SDL_Renderer* renderer,
                       const TireThermalCoordinator& coordinator,
                       int originX,
                       int originY) const noexcept;

    /// @brief Computes collision-free screen placement for thermal HUD card.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    /// @param[in] circuitActive Whether procedural circuit track mode is active.
    /// @param[in] advancedHudActive Whether advanced telemetry HUD is visible.
    /// @param[in] windHudActive Whether wind HUD is visible.
    /// @param[in] provingGroundActive Whether a proving ground discipline HUD is visible.
    /// @return Collision-free top-left screen position (x, y).
    [[nodiscard]] static SDL_Point computeHudCardPos(int screenWidth,
                                                     int screenHeight,
                                                     bool circuitActive,
                                                     bool advancedHudActive,
                                                     bool windHudActive,
                                                     bool provingGroundActive = false) noexcept;

    /// @brief Toggles visibility of the tire thermal HUD card.
    void toggleHud() noexcept;

    /// @brief Sets HUD visibility state.
    /// @param[in] visible True to display thermal HUD card.
    void setHudVisible(bool visible) noexcept;

    /// @brief Queries if thermal HUD card is visible.
    /// @return True if HUD card is active.
    [[nodiscard]] bool isHudVisible() const noexcept;

private:
    bool mHudVisible { true };

    [[nodiscard]] static SDL_Color getTemperatureColor(double tempCelsius) noexcept;
};

} // namespace motion_lab
