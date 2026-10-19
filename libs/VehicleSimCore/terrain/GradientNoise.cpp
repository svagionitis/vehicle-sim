#include "libs/VehicleSimCore/terrain/GradientNoise.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace vehicle_sim::terrain {

namespace {

    constexpr double INV_SQRT2 { 0.70710678118654752440 };
    constexpr double SCALE_1D { 2.0 };
    constexpr double SCALE_2D { 1.41421356237309504880 };

    constexpr std::array<math::Vec2, 8> GRADIENTS_2D { {
        { 1.0, 0.0 },
        { -1.0, 0.0 },
        { 0.0, 1.0 },
        { 0.0, -1.0 },
        { INV_SQRT2, INV_SQRT2 },
        { -INV_SQRT2, INV_SQRT2 },
        { INV_SQRT2, -INV_SQRT2 },
        { -INV_SQRT2, -INV_SQRT2 },
    } };

} // namespace

double GradientNoise::sample1D(double x) const noexcept
{
    return sample1DWithDeriv(x).value;
}

NoiseSample1D GradientNoise::sample1DWithDeriv(double x) const noexcept
{
    const double floorVal { std::floor(x) };
    const auto i0 { static_cast<std::int32_t>(floorVal) };
    const std::int32_t i1 { i0 + 1 };
    const double t { x - floorVal };

    const std::uint32_t h0 { hash1D(i0, mSeed) };
    const std::uint32_t h1 { hash1D(i1, mSeed) };

    const double g0 { (h0 & 1U) != 0U ? 1.0 : -1.0 };
    const double g1 { (h1 & 1U) != 0U ? 1.0 : -1.0 };

    const double p0 { g0 * t };
    const double p1 { g1 * (t - 1.0) };

    const double u { computeQuintic(t) };
    const double uPrime { computeQuinticDeriv(t) };

    NoiseSample1D result {};
    result.value = SCALE_1D * (p0 + u * (p1 - p0));
    result.derivative = SCALE_1D * (g0 + uPrime * (p1 - p0) + u * (g1 - g0));

    return result;
}

double GradientNoise::sample2D(double x, double y) const noexcept
{
    return sample2DWithDeriv(x, y).value;
}

double GradientNoise::sample2D(const math::Vec2& point) const noexcept
{
    return sample2DWithDeriv(point.x, point.y).value;
}

NoiseSample2D GradientNoise::sample2DWithDeriv(const math::Vec2& point) const noexcept
{
    return sample2DWithDeriv(point.x, point.y);
}

NoiseSample2D GradientNoise::sample2DWithDeriv(double x, double y) const noexcept
{
    const double floorX { std::floor(x) };
    const double floorY { std::floor(y) };
    const auto i0 { static_cast<std::int32_t>(floorX) };
    const auto j0 { static_cast<std::int32_t>(floorY) };
    const std::int32_t i1 { i0 + 1 };
    const std::int32_t j1 { j0 + 1 };

    const double ux { x - floorX };
    const double uy { y - floorY };

    const math::Vec2& g00 { GRADIENTS_2D[hash2D(i0, j0, mSeed) & 7U] };
    const math::Vec2& g10 { GRADIENTS_2D[hash2D(i1, j0, mSeed) & 7U] };
    const math::Vec2& g01 { GRADIENTS_2D[hash2D(i0, j1, mSeed) & 7U] };
    const math::Vec2& g11 { GRADIENTS_2D[hash2D(i1, j1, mSeed) & 7U] };

    const double s00 { g00.x * ux + g00.y * uy };
    const double s10 { g10.x * (ux - 1.0) + g10.y * uy };
    const double s01 { g01.x * ux + g01.y * (uy - 1.0) };
    const double s11 { g11.x * (ux - 1.0) + g11.y * (uy - 1.0) };

    const double fx { computeQuintic(ux) };
    const double fxPrime { computeQuinticDeriv(ux) };
    const double fy { computeQuintic(uy) };
    const double fyPrime { computeQuinticDeriv(uy) };

    const double k0 { s00 + fx * (s10 - s00) };
    const double k1 { s01 + fx * (s11 - s01) };
    const double val { k0 + fy * (k1 - k0) };

    const double dk0_dx { g00.x + fxPrime * (s10 - s00) + fx * (g10.x - g00.x) };
    const double dk1_dx { g01.x + fxPrime * (s11 - s01) + fx * (g11.x - g01.x) };
    const double dval_dx { dk0_dx + fy * (dk1_dx - dk0_dx) };

    const double dk0_dy { g00.y + fx * (g10.y - g00.y) };
    const double dk1_dy { g01.y + fx * (g11.y - g01.y) };
    const double dval_dy { dk0_dy + fy * (dk1_dy - dk0_dy) + fyPrime * (k1 - k0) };

    NoiseSample2D result {};
    result.value = SCALE_2D * val;
    result.gradient = math::Vec2 { SCALE_2D * dval_dx, SCALE_2D * dval_dy };

    return result;
}

} // namespace vehicle_sim::terrain
