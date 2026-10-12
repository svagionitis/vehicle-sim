#pragma once

#include <cmath>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/GradientNoise.h"

namespace vehicle_sim::terrain {

/// @enum FractalType
/// @brief Procedural fractal noise synthesis variant.
enum class FractalType : std::uint8_t {
    Fbm, ///< Standard Fractal Brownian Motion (rolling natural terrain).
    Billow, ///< Absolute-value billow noise (dunes, mounds).
    Ridged ///< Inverted absolute-value ridged multifractal (mountain ridges, canyons).
};

/// @struct FractalConfig
/// @brief Configuration parameters for multi-octave fractal synthesis.
struct FractalConfig {
    std::uint32_t octaves { 4U }; ///< Number of frequency layers (octaves).
    double frequency { 1.0 }; ///< Base spatial frequency multiplier.
    double amplitude { 1.0 }; ///< Base noise amplitude scaling.
    double lacunarity { 2.0 }; ///< Frequency scaling multiplier per octave.
    double persistence { 0.5 }; ///< Amplitude decay multiplier per octave.
    std::uint32_t seed { 0U }; ///< Deterministic pseudo-random seed.
    FractalType type { FractalType::Fbm }; ///< Fractal synthesis algorithm.
};

/// @class FractalNoise
/// @brief Multi-octave fractal noise synthesizer supporting fBm, Billow, and Ridged variants.
/// @details Combines multiple octaves of deterministic gradient noise with analytical derivatives.
/// Suitable for procedural 1D elevation profiles and 2D heightmaps for vehicle physics simulation.
class FractalNoise {
public:
    /// @brief Default constructor with default configuration.
    FractalNoise() noexcept = default;

    /// @brief Parameterized constructor accepting a configuration.
    /// @param[in] config Fractal configuration parameters.
    explicit FractalNoise(const FractalConfig& config) noexcept;

    /// @brief Retrieves the current fractal configuration.
    /// @return Current FractalConfig.
    [[nodiscard]] const FractalConfig& getConfig() const noexcept;

    /// @brief Sets the fractal configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const FractalConfig& config) noexcept;

    /// @brief Retrieves the current seed value.
    /// @return Current pseudo-random seed.
    [[nodiscard]] std::uint32_t getSeed() const noexcept;

    /// @brief Sets a new seed value.
    /// @param[in] seed New pseudo-random seed.
    void setSeed(std::uint32_t seed) noexcept;

    /// @brief Retrieves the number of octaves.
    /// @return Octave count.
    [[nodiscard]] std::uint32_t getOctaves() const noexcept;

    /// @brief Sets the octave count.
    /// @param[in] octaves Number of octaves (clamped to range [1, 16]).
    void setOctaves(std::uint32_t octaves) noexcept;

    /// @brief Retrieves base frequency.
    /// @return Base spatial frequency.
    [[nodiscard]] double getFrequency() const noexcept;

    /// @brief Sets base frequency.
    /// @param[in] frequency Spatial frequency multiplier.
    void setFrequency(double frequency) noexcept;

    /// @brief Retrieves base amplitude.
    /// @return Base amplitude.
    [[nodiscard]] double getAmplitude() const noexcept;

    /// @brief Sets base amplitude.
    /// @param[in] amplitude Base amplitude scaling.
    void setAmplitude(double amplitude) noexcept;

    /// @brief Retrieves lacunarity.
    /// @return Frequency multiplier per octave.
    [[nodiscard]] double getLacunarity() const noexcept;

    /// @brief Sets lacunarity.
    /// @param[in] lacunarity Frequency multiplier per octave.
    void setLacunarity(double lacunarity) noexcept;

    /// @brief Retrieves persistence.
    /// @return Amplitude decay factor per octave.
    [[nodiscard]] double getPersistence() const noexcept;

    /// @brief Sets persistence.
    /// @param[in] persistence Amplitude decay factor per octave.
    void setPersistence(double persistence) noexcept;

    /// @brief Retrieves fractal synthesis type.
    /// @return Current FractalType enum value.
    [[nodiscard]] FractalType getType() const noexcept;

    /// @brief Sets fractal synthesis type.
    /// @param[in] type Target FractalType enum value.
    void setType(FractalType type) noexcept;

    /// @brief Computes theoretical maximum amplitude bound across all octaves.
    /// @return Sum of geometric series of octave amplitudes.
    [[nodiscard]] double getMaxAmplitude() const noexcept;

    /// @brief Evaluates 1D fractal noise scalar at coordinate x.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return Accumulated fractal scalar value.
    [[nodiscard]] double sample1D(double x) const noexcept;

    /// @brief Evaluates 1D fractal noise and its analytical derivative at x.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return NoiseSample1D containing accumulated value and exact derivative d/dx.
    [[nodiscard]] NoiseSample1D sample1DWithDeriv(double x) const noexcept;

    /// @brief Evaluates 2D fractal noise scalar at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate.
    /// @param[in] y Vertical spatial coordinate.
    /// @return Accumulated fractal scalar value.
    [[nodiscard]] double sample2D(double x, double y) const noexcept;

    /// @brief Evaluates 2D fractal noise scalar at vector position.
    /// @param[in] point Evaluation position vector.
    /// @return Accumulated fractal scalar value.
    [[nodiscard]] double sample2D(const math::Vec2& point) const noexcept;

    /// @brief Evaluates 2D fractal noise and analytical gradient at (x, y).
    /// @param[in] x Horizontal spatial coordinate.
    /// @param[in] y Vertical spatial coordinate.
    /// @return NoiseSample2D containing accumulated value and gradient vector.
    [[nodiscard]] NoiseSample2D sample2DWithDeriv(double x, double y) const noexcept;

    /// @brief Evaluates 2D fractal noise and analytical gradient at vector position.
    /// @param[in] point Evaluation position vector.
    /// @return NoiseSample2D containing accumulated value and gradient vector.
    [[nodiscard]] NoiseSample2D sample2DWithDeriv(const math::Vec2& point) const noexcept;

    /// @brief Computes upward unit normal vector of 1D terrain profile at x.
    /// @details Normal vector is perpendicular to the surface slope and points upward.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return Unit 2D upward normal vector.
    [[nodiscard]] math::Vec2 computeNormal1D(double x) const noexcept;

    /// @brief Computes forward unit tangent vector of 1D terrain profile at x.
    /// @details Tangent vector points along increasing x following the surface slope.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return Unit 2D forward tangent vector.
    [[nodiscard]] math::Vec2 computeTangent1D(double x) const noexcept;

    /// @brief Computes terrain slope inclination angle in radians at x.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return Slope angle in radians in range [-pi/2, pi/2].
    [[nodiscard]] double computeSlopeAngle1D(double x) const noexcept;

private:
    FractalConfig mConfig {};
    GradientNoise mNoise {};
};

} // namespace vehicle_sim::terrain
