#include "Transform2D.h"

namespace vehicle_sim::math {

Transform2D::Transform2D(const Vec2& pos, double angleRadians) noexcept
    : position { pos }
    , rotation { Mat2::fromAngle(angleRadians) }
{
}

[[nodiscard]] double Transform2D::getAngle() const noexcept
{
    return std::atan2(rotation.m10, rotation.m00);
}

void Transform2D::setAngle(double angleRadians) noexcept
{
    rotation = Mat2::fromAngle(angleRadians);
}

[[nodiscard]] bool Transform2D::isApproxEqual(const Transform2D& other, double epsilon) const noexcept
{
    return position.isApproxEqual(other.position, epsilon) && rotation.isApproxEqual(other.rotation, epsilon);
}

} // namespace vehicle_sim::math
