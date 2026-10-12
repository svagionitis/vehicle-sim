#pragma once

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

/// @class MathUtils
/// @brief Collection of deterministic scalar mathematical primitives and interpolation utilities.
/// @details Provides standard linear and non-linear transitions, floating-point comparisons,
/// signal processing thresholds, and numerical safeguards targeting IEEE 754 compliance in C++17.
class MathUtils {
public:
    MathUtils() = delete;

    /// @brief Linearly interpolates between two scalar values.
    /// @details Computes (a * (1.0 - t)) + (b * t) monotonically with IEEE 754 accuracy.
    /// @param[in] a Starting value.
    /// @param[in] b Target value.
    /// @param[in] t Interpolation parameter.
    /// @return Interpolated scalar.
    [[nodiscard]] static constexpr double lerp(double a, double b, double t) noexcept
    {
        return (a * (1.0 - t)) + (b * t);
    }

    /// @brief Inverses linear interpolation to determine parameter t from value.
    /// @param[in] a Range lower boundary.
    /// @param[in] b Range upper boundary.
    /// @param[in] val Value between a and b.
    /// @return Normalized parameter t, or 0.0 if boundaries are degenerate.
    [[nodiscard]] static constexpr double inverseLerp(double a, double b, double val) noexcept
    {
        const double diff { b - a };
        if (std::abs(diff) <= 1e-12) {
            return 0.0;
        }
        return (val - a) / diff;
    }

    /// @brief Clamps a scalar value to the unit interval [0.0, 1.0].
    /// @param[in] val Input scalar.
    /// @return Clamped value within [0.0, 1.0].
    [[nodiscard]] static constexpr double clamp01(double val) noexcept
    {
        return std::clamp(val, 0.0, 1.0);
    }

    /// @brief Linearly remaps a value from one interval to another.
    /// @param[in] inMin Source interval minimum.
    /// @param[in] inMax Source interval maximum.
    /// @param[in] outMin Destination interval minimum.
    /// @param[in] outMax Destination interval maximum.
    /// @param[in] val Value in source interval.
    /// @return Remapped value in destination interval.
    [[nodiscard]] static constexpr double remap(
        double inMin, double inMax, double outMin, double outMax, double val) noexcept
    {
        const double t { inverseLerp(inMin, inMax, val) };
        return lerp(outMin, outMax, t);
    }

    /// @brief Evaluates Hermite cubic smoothstep interpolation between two edges.
    /// @details Returns 0.0 for x <= edge0, 1.0 for x >= edge1, and cubic S-curve 3t^2 - 2t^3 in between.
    /// @param[in] edge0 Lower edge threshold.
    /// @param[in] edge1 Upper edge threshold.
    /// @param[in] x Evaluation input.
    /// @return Smoothed value within [0.0, 1.0].
    [[nodiscard]] static double smoothstep(double edge0, double edge1, double x) noexcept;

    /// @brief Evaluates Ken Perlin C2-continuous smootherstep interpolation.
    /// @details Returns 6t^5 - 15t^4 + 10t^3 curve providing zero first and second derivatives at edges.
    /// @param[in] edge0 Lower edge threshold.
    /// @param[in] edge1 Upper edge threshold.
    /// @param[in] x Evaluation input.
    /// @return Smoothed value within [0.0, 1.0].
    [[nodiscard]] static double smootherstep(double edge0, double edge1, double x) noexcept;

    /// @brief Applies linear deadband filtering to an input signal.
    /// @details Suppresses sensor or controller noise below threshold and linearly scales output to maxVal.
    /// @param[in] val Raw input signal value.
    /// @param[in] threshold Absolute deadband threshold below which output is 0.0.
    /// @param[in] maxVal Maximum expected signal magnitude for rescaling (defaults to 1.0).
    /// @return Filtered and rescaled signal.
    [[nodiscard]] static double applyDeadband(double val, double threshold, double maxVal = 1.0) noexcept;

    /// @brief Returns the sign of a number as -1.0, 0.0, or +1.0.
    /// @param[in] val Input value.
    /// @return -1.0 if negative, +1.0 if positive, 0.0 if zero.
    [[nodiscard]] static constexpr double sign(double val) noexcept
    {
        if (val > 0.0) {
            return 1.0;
        }
        if (val < 0.0) {
            return -1.0;
        }
        return 0.0;
    }

    /// @brief Returns non-zero sign (-1.0 for negative, +1.0 for zero and positive).
    /// @param[in] val Input value.
    /// @return -1.0 if val < 0.0, else 1.0.
    [[nodiscard]] static constexpr double signNonZero(double val) noexcept
    {
        return (val < 0.0) ? -1.0 : 1.0;
    }

    /// @brief Compares two floating-point values within an absolute epsilon tolerance.
    /// @param[in] a First value.
    /// @param[in] b Second value.
    /// @param[in] eps Absolute tolerance threshold (default 1e-9).
    /// @return True if |a - b| <= eps.
    [[nodiscard]] static constexpr bool nearlyEqual(double a, double b, double eps = 1e-9) noexcept
    {
        return std::abs(a - b) <= eps;
    }

    /// @brief Checks if a floating-point value is within epsilon of zero.
    /// @param[in] a Value to test.
    /// @param[in] eps Absolute tolerance threshold (default 1e-9).
    /// @return True if |a| <= eps.
    [[nodiscard]] static constexpr bool nearlyZero(double a, double eps = 1e-9) noexcept
    {
        return std::abs(a) <= eps;
    }

    /// @brief Performs division guarded against divide-by-zero errors.
    /// @param[in] num Numerator.
    /// @param[in] denom Denominator.
    /// @param[in] fallback Fallback return value if denominator is near zero (default 0.0).
    /// @return num / denom if valid, otherwise fallback.
    [[nodiscard]] static constexpr double safeDivide(double num, double denom, double fallback = 0.0) noexcept
    {
        if (std::abs(denom) <= 1e-12) {
            return fallback;
        }
        return num / denom;
    }
};

} // namespace vehicle_sim::math
