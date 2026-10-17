#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::terrain {

/// @enum WaveBoundaryType
/// @brief Boundary condition enforcement policy for wave grids.
enum class WaveBoundaryType : std::uint8_t {
    Dirichlet, ///< Fixed baseline water elevation (zero perturbation).
    Neumann,   ///< Specular reflection (zero normal derivative, solid quay wall).
    Absorbing  ///< Absorbing sponge layer with quadratic damping attenuation.
};

/// @struct WaveConfig1D
/// @brief Configuration settings for 1D shallow water wave grid.
struct WaveConfig1D {
    double originX { -100.0 };       ///< Minimum horizontal coordinate in meters.
    double length { 200.0 };         ///< Total horizontal span in meters.
    double cellSize { 0.5 };         ///< Spatial sample spacing dx in meters.
    double waveSpeed { 6.0 };        ///< Base shallow water wave celerity c in m/s.
    double damping { 0.15 };         ///< Viscous energy dissipation coefficient gamma in 1/s.
    double waterlineY { 0.0 };       ///< Undisturbed baseline water level in meters.
    WaveBoundaryType leftBoundary { WaveBoundaryType::Absorbing };  ///< Left boundary condition.
    WaveBoundaryType rightBoundary { WaveBoundaryType::Absorbing }; ///< Right boundary condition.
    std::size_t spongeWidth { 10U }; ///< Cell count for absorbing sponge layer.
};

/// @struct WaveSample1D
/// @brief Point evaluation snapshot from a 1D wave grid.
struct WaveSample1D {
    double elevation { 0.0 };        ///< Total water elevation h = waterlineY + eta in meters.
    double perturbation { 0.0 };     ///< Perturbation eta from baseline in meters.
    double derivative { 0.0 };       ///< Spatial gradient deta/dx.
    double verticalSpeed { 0.0 };    ///< Vertical wave velocity deta/dt in m/s.
    double slopeAngle { 0.0 };       ///< Surface tilt angle in radians.
    math::Vec2 normal { 0.0, 1.0 };  ///< Upward unit normal vector.
};

/// @class WaveGrid1D
/// @brief 1D finite-difference shallow water wave propagation grid.
/// @details Simulates real-time 1D surface wave elevation, propagation, reflection, and damping.
/// Maintains strict IEEE 754 determinism, pure C++17 compliance, and zero runtime heap allocations.
class WaveGrid1D {
public:
    /// @brief Default constructor creating an uninitialized wave grid.
    WaveGrid1D() noexcept = default;

    /// @brief Parameterized constructor initializing grid dimensions and configuration.
    /// @param[in] config Wave configuration settings.
    explicit WaveGrid1D(const WaveConfig1D& config);

    /// @brief Initializes grid buffers and parameters.
    /// @param[in] config Wave configuration settings.
    void initGrid(const WaveConfig1D& config);

    /// @brief Advances wave simulation by timestep dt using adaptive sub-stepping.
    /// @param[in] dt Timestep duration in seconds.
    void step(double dt) noexcept;

    /// @brief Injects a localized vertical displacement impulse into the wave field.
    /// @param[in] x World coordinate of impulse center in meters.
    /// @param[in] radius Spatial radius of impulse influence in meters.
    /// @param[in] impulse Velocity/elevation perturbation magnitude in meters.
    void injectImpulse(double x, double radius, double impulse) noexcept;

    /// @brief Evaluates wave elevation and surface state at horizontal coordinate x.
    /// @param[in] x World horizontal coordinate in meters.
    /// @return WaveSample1D containing elevation, gradient, slope, and normal.
    [[nodiscard]] WaveSample1D getSample(double x) const noexcept;

    /// @brief Evaluates wave elevation at horizontal coordinate x.
    /// @param[in] x World horizontal coordinate in meters.
    /// @return Total water surface elevation in meters.
    [[nodiscard]] double getElevation(double x) const noexcept;

    /// @brief Evaluates total mechanical wave energy (kinetic + potential) in Joules.
    /// @return Integrated mechanical wave energy.
    [[nodiscard]] double computeEnergy() const noexcept;

    /// @brief Gets active grid configuration.
    /// @return Const reference to WaveConfig1D.
    [[nodiscard]] const WaveConfig1D& getConfig() const noexcept;

    /// @brief Gets total cell count along 1D domain.
    /// @return Number of discrete spatial samples.
    [[nodiscard]] std::size_t getCellCount() const noexcept;

    /// @brief Resets wave field to undisturbed equilibrium waterline.
    void reset() noexcept;

    /// @brief Calculates Courant number for given timestep.
    /// @param[in] dt Candidate timestep in seconds.
    /// @return Dimensionless Courant number c * dt / dx.
    [[nodiscard]] double computeCourant(double dt) const noexcept;

private:
    WaveConfig1D mConfig {};
    std::size_t mCellCount { 0U };
    double mInvCellSize { 0.0 };

    std::vector<double> mElevationPrev {};
    std::vector<double> mElevationCurr {};
    std::vector<double> mElevationNext {};
    std::vector<double> mSpongeDamping {};
    double mLastSubDt { 0.0166667 };

    void executeSubstep(double dt) noexcept;
    void applyBoundaries() noexcept;
    void updateSpongeCoeffs();
};

} // namespace vehicle_sim::terrain
