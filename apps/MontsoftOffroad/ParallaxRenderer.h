#pragma once

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace montsoft_offroad {

/// @class ParallaxRenderer
/// @brief Multi-layer retro parallax backdrop renderer for 8 desert biomes.
/// @details Rasterizes 4-tier parallax scenery directly into CgaPaletteRenderer backbuffer:
/// Tier 0 (Celestial sun/sky), Tier 1 (Distant mesas), Tier 2 (Rolling dunes & props),
/// and Tier 3 (Foreground heightfield and soil strata).
class ParallaxRenderer final {
public:
    /// @brief Default constructor.
    ParallaxRenderer() noexcept;

    /// @brief Destructor.
    ~ParallaxRenderer() noexcept = default;

    ParallaxRenderer(const ParallaxRenderer&) = delete;
    ParallaxRenderer& operator=(const ParallaxRenderer&) = delete;
    ParallaxRenderer(ParallaxRenderer&&) noexcept = default;
    ParallaxRenderer& operator=(ParallaxRenderer&&) noexcept = default;

    /// @brief Advances atmospheric drifts and shimmer animations by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Renders complete 4-tier parallax scenery and foreground terrain.
    /// @param[in,out] cga Target CGA software rasterizer.
    /// @param[in] camera Viewport camera tracking the vehicle.
    /// @param[in] terrain Active stage 1D heightfield surface.
    /// @param[in] stageIndex Active stage number in [1, 8].
    void render(
        CgaPaletteRenderer& cga,
        const Camera2D& camera,
        const vehicle_sim::terrain::Heightfield1D& terrain,
        int stageIndex) noexcept;

    /// @brief Resets atmospheric animation timers.
    void reset() noexcept;

private:
    void renderSky(CgaPaletteRenderer& cga, int stageIndex) noexcept;
    void renderSun(CgaPaletteRenderer& cga, int stageIndex) noexcept;
    void renderDistantMesas(
        CgaPaletteRenderer& cga, const Camera2D& camera, int stageIndex) noexcept;
    void renderMidgroundDunes(
        CgaPaletteRenderer& cga, const Camera2D& camera, int stageIndex) noexcept;
    void renderForegroundTrack(
        CgaPaletteRenderer& cga,
        const Camera2D& camera,
        const vehicle_sim::terrain::Heightfield1D& terrain,
        int stageIndex) noexcept;
    void renderSilhouetteProp(
        CgaPaletteRenderer& cga, int screenX, int screenGroundY, int propType, ColorRgba color) noexcept;

    double animTimer_ { 0.0 };
};

} // namespace montsoft_offroad
