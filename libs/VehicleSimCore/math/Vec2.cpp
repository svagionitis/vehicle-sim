#include "Vec2.h"

namespace vehicle_sim::math {

[[nodiscard]] Vec2 Vec2::normalized(double eps) const noexcept
{
    const double len { length() };
    if (len <= eps) {
        return Vec2 { 0.0, 0.0 };
    }
    const double inv { 1.0 / len };
    return Vec2 { x * inv, y * inv };
}

void Vec2::normalize(double eps) noexcept
{
    *this = normalized(eps);
}

[[nodiscard]] Vec2 Vec2::rotated(double angleRadians) const noexcept
{
    const double cosA { std::cos(angleRadians) };
    const double sinA { std::sin(angleRadians) };
    return Vec2 { (x * cosA) - (y * sinA), (x * sinA) + (y * cosA) };
}

[[nodiscard]] bool Vec2::isApproxEqual(const Vec2& other, double epsilon) const noexcept
{
    return (std::abs(x - other.x) <= epsilon) && (std::abs(y - other.y) <= epsilon);
}

[[nodiscard]] bool Vec2::isZero(double epsilon) const noexcept
{
    return lengthSquared() <= (epsilon * epsilon);
}

} // namespace vehicle_sim::math
