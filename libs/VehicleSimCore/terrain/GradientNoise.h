#pragma once

#include <cmath>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::terrain {

/// @struct NoiseSample1D
/// @brief Result of a 1D gradient noise evaluation containing scalar and analytical derivative.
/// @details Encapsulates the interpolated scalar height and the exact spatial derivative d/dx.
struct NoiseSample1D {
    double value { 0.0 }; ///< Interpolated noise scalar.
    double derivative { 0.0 }; ///< First-order spatial derivative d/dx.
};

/// @struct NoiseSample2D
/// @brief Result of a 2D gradient noise evaluation containing scalar and analytical gradient.
/// @details Encapsulates the interpolated scalar value and the exact 2D spatial gradient (d/dx, d/dy).
struct NoiseSample2D {
    double value { 0.0 }; ///< Interpolated noise scalar.
    math::Vec2 gradient { 0.0, 0.0 }; ///< First-order spatial gradient vector.
};

/// @class GradientNoise
/// @brief Deterministic 1D and 2D Perlin gradient noise generator with analytical derivatives.
/// @details Evaluates smooth C2-continuous gradient noise using Ken Perlin's quintic blend polynomial
/// and a deterministic pseudo-random hash generator. Guarantees cross-platform bit-level repeatability
/// with zero dynamic allocations.
class GradientNoise {
public:
    /// @brief Default constructor with seed 0.
    constexpr GradientNoise() noexcept = default;

    /// @brief Parameterized constructor initializing pseudo-random seed.
    /// @param[in] seed Pseudo-random seed value.
    constexpr explicit GradientNoise(std::uint32_t seed) noexcept
        : mSeed { seed }
    {
    }

    /// @brief Retrieves the current seed value.
    /// @return Current pseudo-random seed.
    [[nodiscard]] constexpr std::uint32_t getSeed() const noexcept
    {
        return mSeed;
    }

    /// @brief Sets a new seed value.
    /// @param[in] seed New pseudo-random seed.
    void setSeed(std::uint32_t seed) noexcept
    {
        mSeed = seed;
    }

    /// @brief Evaluates 1D gradient noise scalar at coordinate x.
    /// @details Computes smooth interpolated noise value in range [-1.0, 1.0].
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return Noise scalar value.
    [[nodiscard]] double sample1D(double x) const noexcept;

    /// @brief Evaluates 1D gradient noise and its analytical derivative at x.
    /// @details Computes noise value and exact d/dx spatial derivative.
    /// @param[in] x Evaluation coordinate along the 1D axis.
    /// @return NoiseSample1D containing value and derivative.
    [[nodiscard]] NoiseSample1D sample1DWithDeriv(double x) const noexcept;

    /// @brief Evaluates 2D gradient noise scalar at coordinates (x, y).
    /// @details Computes smooth interpolated noise value in range [-1.0, 1.0].
    /// @param[in] x Horizontal spatial coordinate.
    /// @param[in] y Vertical spatial coordinate.
    /// @return Noise scalar value.
    [[nodiscard]] double sample2D(double x, double y) const noexcept;

    /// @brief Evaluates 2D gradient noise scalar at vector position.
    /// @param[in] point Evaluation position vector.
    /// @return Noise scalar value.
    [[nodiscard]] double sample2D(const math::Vec2& point) const noexcept;

    /// @brief Evaluates 2D gradient noise and analytical gradient at (x, y).
    /// @details Computes noise value and exact (d/dx, d/dy) spatial gradient vector.
    /// @param[in] x Horizontal spatial coordinate.
    /// @param[in] y Vertical spatial coordinate.
    /// @return NoiseSample2D containing value and gradient vector.
    [[nodiscard]] NoiseSample2D sample2DWithDeriv(double x, double y) const noexcept;

    /// @brief Evaluates 2D gradient noise and analytical gradient at vector position.
    /// @param[in] point Evaluation position vector.
    /// @return NoiseSample2D containing value and gradient vector.
    [[nodiscard]] NoiseSample2D sample2DWithDeriv(const math::Vec2& point) const noexcept;

    /// @brief Computes deterministic hash for 1D integer coordinate and seed.
    /// @param[in] x Integer grid coordinate.
    /// @param[in] seed Pseudo-random seed value.
    /// @return 32-bit pseudo-random hash value.
    [[nodiscard]] static constexpr std::uint32_t hash1D(std::int32_t x, std::uint32_t seed) noexcept
    {
        std::uint32_t h { static_cast<std::uint32_t>(x) ^ (seed * 0x9e3779b9U) };
        h ^= h >> 16U;
        h *= 0x45d9f3bU;
        h ^= h >> 16U;
        h *= 0x45d9f3bU;
        h ^= h >> 16U;
        return h;
    }

    /// @brief Computes deterministic hash for 2D integer coordinates and seed.
    /// @param[in] x Horizontal integer grid coordinate.
    /// @param[in] y Vertical integer grid coordinate.
    /// @param[in] seed Pseudo-random seed value.
    /// @return 32-bit pseudo-random hash value.
    [[nodiscard]] static constexpr std::uint32_t hash2D(std::int32_t x, std::int32_t y, std::uint32_t seed) noexcept
    {
        constexpr std::uint32_t prime1 { 0x9e3779b9U };
        constexpr std::uint32_t prime2 { 0x85ebca6bU };
        std::uint32_t h { (static_cast<std::uint32_t>(x) * prime1) ^ (static_cast<std::uint32_t>(y) * prime2) ^ seed };
        h ^= h >> 16U;
        h *= 0x45d9f3bU;
        h ^= h >> 16U;
        h *= 0x45d9f3bU;
        h ^= h >> 16U;
        return h;
    }

    /// @brief Computes Ken Perlin's quintic blend polynomial 6t^5 - 15t^4 + 10t^3.
    /// @param[in] t Fractional parameter in range [0, 1].
    /// @return Quintic blend weight in range [0, 1].
    [[nodiscard]] static constexpr double computeQuintic(double t) noexcept
    {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    /// @brief Computes first derivative of quintic polynomial 30t^4 - 60t^3 + 30t^2.
    /// @param[in] t Fractional parameter in range [0, 1].
    /// @return Derivative of quintic blend weight.
    [[nodiscard]] static constexpr double computeQuinticDeriv(double t) noexcept
    {
        const double tMinusOne { t - 1.0 };
        return 30.0 * t * t * tMinusOne * tMinusOne;
    }

private:
    std::uint32_t mSeed { 0U };
};

} // namespace vehicle_sim::terrain
