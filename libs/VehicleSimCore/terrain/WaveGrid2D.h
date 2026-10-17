#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/WaveGrid1D.h"

namespace vehicle_sim::terrain {

/// @struct WaveConfig2D
/// @brief Configuration settings for 2D shallow water wave basin.
struct WaveConfig2D {
    math::Vec2 origin { -50.0, -50.0 }; ///< Minimum corner coordinates in meters.
    math::Vec2 size { 100.0, 100.0 };   ///< Basin domain dimensions in meters.
    double cellSize { 0.5 };            ///< Spatial grid cell spacing dx = dy in meters.
    double waveSpeed { 5.0 };           ///< Base wave propagation celerity c in m/s.
    double damping { 0.12 };            ///< Viscous wave damping coefficient gamma in 1/s.
    double waterlineZ { 0.0 };          ///< Undisturbed baseline water elevation in meters.
    WaveBoundaryType boundaryType { WaveBoundaryType::Absorbing }; ///< Domain boundary policy.
    std::size_t spongeWidth { 12U };    ///< Outer perimeter cell count for absorbing sponge.
    bool use9PointLaplacian { true };   ///< Enable isotropic 9-point Laplacian stencil.
};

/// @struct WaveSample2D
/// @brief Point evaluation snapshot from a 2D wave grid.
struct WaveSample2D {
    double elevation { 0.0 };        ///< Total water elevation z = waterlineZ + eta in meters.
    double perturbation { 0.0 };     ///< Perturbation eta from baseline in meters.
    math::Vec2 gradient { 0.0, 0.0 };///< Planar spatial gradient (deta/dx, deta/dy).
    double verticalSpeed { 0.0 };    ///< Vertical wave velocity deta/dt in m/s.
    double slopeAngle { 0.0 };       ///< Surface inclination angle in radians.
    SurfaceNormal3D normal { 0.0, 0.0, 1.0 }; ///< 3D upward unit surface normal vector.
};

/// @struct WaterWakeStamp2D
/// @brief Hull footprint and kinematics for wake generation.
struct WaterWakeStamp2D {
    math::Vec2 position { 0.0, 0.0 };  ///< Center of hull contact in world meters.
    double heading { 0.0 };            ///< Hull heading angle in radians.
    double length { 5.0 };             ///< Hull length in meters.
    double beam { 2.0 };               ///< Hull beam width in meters.
    double draft { 0.6 };              ///< Submerged hull draft in meters.
    math::Vec2 velocity { 0.0, 0.0 };  ///< Planar velocity vector in m/s.
    double verticalVelocity { 0.0 };   ///< Vertical heave velocity in m/s.
    double displacement { 1500.0 };    ///< Hull displacement mass in kg.
};

/// @class WaveGrid2D
/// @brief 2D finite-difference shallow water wave propagation basin.
/// @details Solves the 2D damped wave PDE over a regular Cartesian grid using FDTD.
/// Supports 5-point and 9-point isotropic Laplacians, absorbing sponge layers,
/// and two-way vehicle hydrodynamics wake interactions.
class WaveGrid2D {
public:
    /// @brief Default constructor creating an uninitialized wave basin.
    WaveGrid2D() noexcept = default;

    /// @brief Parameterized constructor initializing 2D grid dimensions.
    /// @param[in] config 2D wave basin configuration settings.
    explicit WaveGrid2D(const WaveConfig2D& config);

    /// @brief Initializes 2D grid dimensions and precomputes sponge layer damping.
    /// @param[in] config 2D wave basin configuration settings.
    void initGrid(const WaveConfig2D& config);

    /// @brief Advances 2D wave field by timestep dt using adaptive sub-stepping.
    /// @param[in] dt Timestep duration in seconds.
    void step(double dt) noexcept;

    /// @brief Injects a localized vertical impulse at world coordinates (x, y).
    /// @param[in] position World coordinates of impulse center in meters.
    /// @param[in] radius Spatial radius of impulse footprint in meters.
    /// @param[in] impulse Perturbation magnitude in meters.
    void injectImpulse(const math::Vec2& position, double radius, double impulse) noexcept;

    /// @brief Stamps a dynamic vehicle wake footprint into the wave basin.
    /// @param[in] stamp Vehicle hull dimensions, load, and velocity footprint.
    void applyWakeStamp(const WaterWakeStamp2D& stamp) noexcept;

    /// @brief Evaluates 2D wave elevation and surface normal at coordinates (x, y).
    /// @param[in] position World coordinates in meters.
    /// @return WaveSample2D with elevation, gradient, vertical speed, and 3D normal.
    [[nodiscard]] WaveSample2D getSample(const math::Vec2& position) const noexcept;

    /// @brief Evaluates water elevation at world coordinates (x, y).
    /// @param[in] position World coordinates in meters.
    /// @return Total water surface elevation z in meters.
    [[nodiscard]] double getElevation(const math::Vec2& position) const noexcept;

    /// @brief Evaluates total fluid kinetic and potential energy in wave basin.
    /// @return Integrated mechanical wave energy in Joules.
    [[nodiscard]] double computeEnergy() const noexcept;

    /// @brief Gets active 2D wave basin configuration.
    /// @return Const reference to WaveConfig2D.
    [[nodiscard]] const WaveConfig2D& getConfig() const noexcept;

    /// @brief Gets column sample count along X axis.
    /// @return Column count.
    [[nodiscard]] std::size_t getWidth() const noexcept;

    /// @brief Gets row sample count along Y axis.
    /// @return Row count.
    [[nodiscard]] std::size_t getHeight() const noexcept;

    /// @brief Resets wave field to flat equilibrium waterline.
    void reset() noexcept;

private:
    WaveConfig2D mConfig {};
    std::size_t mWidth { 0U };
    std::size_t mHeight { 0U };
    double mInvCellSize { 0.0 };

    std::vector<double> mElevationPrev {};
    std::vector<double> mElevationCurr {};
    std::vector<double> mElevationNext {};
    std::vector<double> mSpongeDamping {};
    double mLastSubDt { 0.0166667 };

    [[nodiscard]] inline std::size_t getIndex(std::size_t x, std::size_t y) const noexcept
    {
        return (y * mWidth) + x;
    }

    void executeSubstep(double dt) noexcept;
    void applyBoundaries() noexcept;
    void updateSpongeLayer();
};

} // namespace vehicle_sim::terrain
