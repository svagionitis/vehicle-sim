#include "Mat2.h"

namespace vehicle_sim::math {

[[nodiscard]] Mat2 Mat2::fromAngle(double angleRadians) noexcept
{
    const double cosA { std::cos(angleRadians) };
    const double sinA { std::sin(angleRadians) };
    return Mat2 { cosA, -sinA, sinA, cosA };
}

[[nodiscard]] std::optional<Mat2> Mat2::inverse(double epsilon) const noexcept
{
    const double determinant { det() };
    if (std::abs(determinant) <= epsilon) {
        return std::nullopt;
    }
    const double invDet { 1.0 / determinant };
    return Mat2 { m11 * invDet, -m01 * invDet, -m10 * invDet, m00 * invDet };
}

[[nodiscard]] bool Mat2::isApproxEqual(const Mat2& other, double epsilon) const noexcept
{
    return (std::abs(m00 - other.m00) <= epsilon) && (std::abs(m01 - other.m01) <= epsilon)
        && (std::abs(m10 - other.m10) <= epsilon) && (std::abs(m11 - other.m11) <= epsilon);
}

} // namespace vehicle_sim::math
