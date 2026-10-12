#include "MathUtils.h"

namespace vehicle_sim::math {

double MathUtils::smoothstep(double edge0, double edge1, double x) noexcept
{
    const double diff { edge1 - edge0 };
    if (std::abs(diff) <= 1e-12) {
        return (x >= edge1) ? 1.0 : 0.0;
    }
    const double t { clamp01((x - edge0) / diff) };
    return t * t * (3.0 - (2.0 * t));
}

double MathUtils::smootherstep(double edge0, double edge1, double x) noexcept
{
    const double diff { edge1 - edge0 };
    if (std::abs(diff) <= 1e-12) {
        return (x >= edge1) ? 1.0 : 0.0;
    }
    const double t { clamp01((x - edge0) / diff) };
    return t * t * t * ((t * ((t * 6.0) - 15.0)) + 10.0);
}

double MathUtils::applyDeadband(double val, double threshold, double maxVal) noexcept
{
    if (threshold <= 0.0) {
        return val;
    }
    const double absVal { std::abs(val) };
    if (absVal <= threshold) {
        return 0.0;
    }
    if (maxVal <= threshold) {
        return 0.0;
    }
    const double signVal { (val > 0.0) ? 1.0 : -1.0 };
    const double normalizedMag { (absVal - threshold) / (maxVal - threshold) };
    return signVal * clamp01(normalizedMag) * maxVal;
}

} // namespace vehicle_sim::math
