#pragma once

#include <SDL2/SDL.h>

#include "apps/EarthPatrol/Camera2D.h"

namespace earth_patrol {

/// @class ParallaxRenderer
/// @brief Multi-layer terrestrial parallax background renderer for Earth biomes.
/// @details Renders atmospheric sky gradients, drifting clouds, distant mountain ridges,
/// and midground forest/hill silhouettes at depth-scaled parallax speeds.
class ParallaxRenderer {
public:
    /// @brief Default constructor initializing cloud drift offsets.
    ParallaxRenderer() noexcept;

    /// @brief Advances cloud animation offsets by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Renders the multi-tier parallax background for the active biome.
    /// @param[in] renderer Active SDL hardware renderer pointer.
    /// @param[in] camera 2D camera providing world viewport transform.
    /// @param[in] biomeIndex Active biome index (0: Highway, 1: Desert, 2: Alpine).
    void render(SDL_Renderer* renderer, const Camera2D& camera, int biomeIndex) noexcept;

    /// @brief Resets cloud drift offsets to initial state.
    void reset() noexcept;

private:
    /// @brief Renders the gradient atmospheric sky.
    void renderSky(SDL_Renderer* renderer, int width, int height, int biomeIndex) noexcept;

    /// @brief Renders drifting volumetric cloud silhouettes.
    void renderClouds(SDL_Renderer* renderer, const Camera2D& camera, int width, int biomeIndex) noexcept;

    /// @brief Renders distant mountain ridge crests (0.08x parallax).
    void renderMountains(
        SDL_Renderer* renderer, const Camera2D& camera, int width, int height, int biomeIndex) noexcept;

    /// @brief Renders midground rolling hills / forest ridges (0.28x parallax).
    void renderHills(SDL_Renderer* renderer, const Camera2D& camera, int width, int height, int biomeIndex) noexcept;

    double cloudDrift_ { 0.0 };
};

} // namespace earth_patrol
