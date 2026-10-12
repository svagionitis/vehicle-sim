#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @enum FluidMediumType
/// @brief Physical fluid domain category.
enum class FluidMediumType {
    Air,   ///< Gaseous atmospheric air domain.
    Water  ///< Liquid aqueous fresh or salt water domain.
};

/// @struct FluidSample2D
/// @brief Point sample of environmental fluid state.
/// @details Contains instantaneous fluid velocity, fluid mass density, and medium category.
struct FluidSample2D {
    math::Vec2 velocity { 0.0, 0.0 }; ///< Fluid flow velocity vector in m/s.
    double density { 1.225 };         ///< Fluid mass density in kg/m^3.
    FluidMediumType medium { FluidMediumType::Air }; ///< Active fluid medium type.
};

/// @struct DiscreteGust2D
/// @brief Configuration for a 1-Cosine discrete transient gust pulse.
/// @details Models standard FAA/MIL 1-cosine gust profiles with spatial direction and time limits.
struct DiscreteGust2D {
    math::Vec2 direction { 1.0, 0.0 }; ///< Unit direction of gust velocity perturbation.
    double amplitude { 10.0 };         ///< Peak gust velocity perturbation in m/s.
    double startTime { 0.0 };          ///< Simulation start time in seconds.
    double duration { 3.0 };           ///< Total pulse duration in seconds.
};

/// @struct HarmonicGust2D
/// @brief Harmonic spatial and temporal turbulence wave component.
struct HarmonicGust2D {
    math::Vec2 waveVector { 0.05, 0.0 }; ///< Spatial wave frequencies (kx, ky) in rad/m.
    math::Vec2 amplitude { 2.0, 0.0 };   ///< Gust velocity amplitude vector in m/s.
    double frequency { 0.5 };            ///< Temporal frequency omega in rad/s.
    double phase { 0.0 };                ///< Initial phase offset in radians.
};

/// @struct FluidFieldConfig
/// @brief Configuration settings for Environmental Fluid Vector Fields.
/// @details Defines ambient wind and current velocities, boundary elevation, shear exponents, and densities.
struct FluidFieldConfig {
    math::Vec2 ambientWind { 0.0, 0.0 };    ///< Uniform ambient atmospheric wind in m/s.
    math::Vec2 ambientCurrent { 0.0, 0.0 }; ///< Uniform ambient water current in m/s.
    double waterlineY { 0.0 };              ///< World Y waterline boundary elevation in meters.
    double airDensity { 1.225 };            ///< Ambient atmospheric air density in kg/m^3.
    double waterDensity { 1025.0 };         ///< Ambient sea water density in kg/m^3.

    double windShearRefHeight { 10.0 };     ///< Reference elevation for wind shear in meters.
    double windShearExponent { 0.143 };     ///< Power-law wind shear exponent alpha.
    double currentDecayDepth { 30.0 };      ///< Characteristic depth decay scale for water currents in meters.
    bool enableWindShear { false };         ///< Toggle for atmospheric wind shear profile.
    bool enableCurrentDecay { false };      ///< Toggle for subsurface water current decay profile.
};

/// @class FluidField2D
/// @brief Continuous 2D environmental fluid vector field coordinator.
/// @details Evaluates spatial and temporal fluid velocities and densities for atmospheric wind and ocean currents.
/// Conforms to IEEE 754 floating-point determinism and pure C++17 standards with zero external dependencies.
class FluidField2D {
public:
    /// @brief Mathematical constant Pi (3.14159265358979323846).
    static constexpr double kPi { 3.14159265358979323846 };

    /// @brief Default constructor using baseline atmospheric and marine settings.
    FluidField2D() noexcept = default;

    /// @brief Parameterized constructor using custom configuration parameters.
    /// @param[in] config Field configuration parameters.
    explicit FluidField2D(const FluidFieldConfig& config) noexcept;

    /// @brief Samples the fluid velocity and density at given position and time.
    /// @param[in] position World position vector in meters.
    /// @param[in] time Current simulation time in seconds.
    /// @return Sampled FluidSample2D containing velocity, density, and medium type.
    [[nodiscard]] FluidSample2D sample(const math::Vec2& position, double time) const noexcept;

    /// @brief Queries fluid velocity vector at given position and time.
    /// @param[in] position World position vector in meters.
    /// @param[in] time Current simulation time in seconds.
    /// @return 2D fluid velocity vector in m/s.
    [[nodiscard]] math::Vec2 getVelocity(const math::Vec2& position, double time) const noexcept;

    /// @brief Queries fluid density at given elevation.
    /// @param[in] position World position vector in meters.
    /// @return Fluid density in kg/m^3.
    [[nodiscard]] double getDensity(const math::Vec2& position) const noexcept;

    /// @brief Checks if given position is submerged below the waterline.
    /// @param[in] position World position vector in meters.
    /// @return True if y <= waterlineY.
    [[nodiscard]] bool isSubmerged(const math::Vec2& position) const noexcept;

    /// @brief Adds a 1-Cosine discrete gust pulse to the field.
    /// @param[in] gust Discrete gust descriptor.
    void addDiscreteGust(const DiscreteGust2D& gust);

    /// @brief Adds a continuous harmonic turbulence component.
    /// @param[in] gust Harmonic turbulence descriptor.
    void addHarmonicGust(const HarmonicGust2D& gust);

    /// @brief Clears all registered discrete and harmonic gusts.
    void clearGusts() noexcept;

    /// @brief Gets active field configuration parameters.
    /// @return Const reference to active FluidFieldConfig.
    [[nodiscard]] const FluidFieldConfig& getConfig() const noexcept;

    /// @brief Updates field configuration parameters.
    /// @param[in] config New configuration parameters.
    void setConfig(const FluidFieldConfig& config) noexcept;

    /// @brief Sets uniform ambient wind velocity.
    /// @param[in] wind Wind velocity vector in m/s.
    void setAmbientWind(const math::Vec2& wind) noexcept;

    /// @brief Sets uniform ambient water current velocity.
    /// @param[in] current Water current velocity vector in m/s.
    void setAmbientCurrent(const math::Vec2& current) noexcept;

    /// @brief Sets world waterline elevation.
    /// @param[in] waterlineY Waterline elevation in meters.
    void setWaterline(double waterlineY) noexcept;

    /// @brief Returns the count of registered discrete gusts.
    /// @return Number of discrete gusts.
    [[nodiscard]] std::size_t getDiscreteGustCount() const noexcept;

    /// @brief Returns the count of registered harmonic gusts.
    /// @return Number of harmonic gusts.
    [[nodiscard]] std::size_t getHarmonicGustCount() const noexcept;

private:
    FluidFieldConfig mConfig { };
    std::vector<DiscreteGust2D> mDiscreteGusts { };
    std::vector<HarmonicGust2D> mHarmonicGusts { };
};

} // namespace vehicle_sim::physics
