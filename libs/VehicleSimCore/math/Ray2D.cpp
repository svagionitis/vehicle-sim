#include "Ray2D.h"

namespace vehicle_sim::math {

Ray2D::Ray2D(const Vec2& inOrigin, const Vec2& inDirection, double inMaxDist) noexcept
    : origin { inOrigin }
    , direction { inDirection }
    , maxDistance { inMaxDist }
{
    const double lengthSq { direction.lengthSquared() };
    if (lengthSq > 1e-18) {
        direction = direction.normalized();
    } else {
        direction = Vec2 { 1.0, 0.0 };
    }
}

Ray2D Ray2D::transformed(const Vec2& translation, double rotationRad) const noexcept
{
    const double cosTheta { std::cos(rotationRad) };
    const double sinTheta { std::sin(rotationRad) };

    const Vec2 rotDir { (direction.x * cosTheta) - (direction.y * sinTheta),
        (direction.x * sinTheta) + (direction.y * cosTheta) };

    const Vec2 rotOrigin { (origin.x * cosTheta) - (origin.y * sinTheta),
        (origin.x * sinTheta) + (origin.y * cosTheta) };

    return Ray2D { rotOrigin + translation, rotDir, maxDistance };
}

} // namespace vehicle_sim::math
