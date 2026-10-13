#include "Circle2D.h"
#include "OBB2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

bool Circle2D::contains(const Vec2& point) const noexcept
{
    return (point - center).lengthSquared() <= (radius * radius);
}

Vec2 Circle2D::closestPoint(const Vec2& point) const noexcept
{
    const Vec2 delta { point - center };
    const double distSq { delta.lengthSquared() };

    if (distSq <= (radius * radius)) {
        return point;
    }

    const double dist { std::sqrt(distSq) };
    if (dist <= 1e-12) {
        return center + Vec2 { radius, 0.0 };
    }

    return center + (delta * (radius / dist));
}

AABB2D Circle2D::computeAABB() const noexcept
{
    const Vec2 rVec { radius, radius };
    return AABB2D { center - rVec, center + rVec };
}

bool Circle2D::intersects(const Circle2D& other) const noexcept
{
    const double rSum { radius + other.radius };
    return (other.center - center).lengthSquared() <= (rSum * rSum);
}

bool Circle2D::intersects(const AABB2D& aabb) const noexcept
{
    const double clampedX { std::clamp(center.x, aabb.min.x, aabb.max.x) };
    const double clampedY { std::clamp(center.y, aabb.min.y, aabb.max.y) };
    const Vec2 closest { clampedX, clampedY };

    return (closest - center).lengthSquared() <= (radius * radius);
}

bool Circle2D::intersects(const OBB2D& obb) const noexcept
{
    const Vec2 closest { obb.closestPoint(center) };
    return (closest - center).lengthSquared() <= (radius * radius);
}

std::optional<RayHit2D> Circle2D::raycast(const Ray2D& ray) const noexcept
{
    const Vec2 m { ray.origin - center };
    const double c { m.lengthSquared() - (radius * radius) };
    const double b { m.dot(ray.direction) };

    if (c > 0.0 && b > 0.0) {
        return std::nullopt;
    }

    const double discr { (b * b) - c };
    if (discr < 0.0) {
        return std::nullopt;
    }

    const double sqrtDiscr { std::sqrt(discr) };
    double t { -b - sqrtDiscr };

    if (t < 0.0) {
        t = -b + sqrtDiscr;
    }

    if (t < 0.0 || t > ray.maxDistance) {
        return std::nullopt;
    }

    const Vec2 hitPoint { ray.origin + (ray.direction * t) };
    Vec2 hitNormal { (hitPoint - center).normalized() };
    if (c < 0.0) {
        hitNormal = -hitNormal;
    }

    const double fraction { (ray.maxDistance > 0.0) ? (t / ray.maxDistance) : 0.0 };
    return RayHit2D { hitPoint, hitNormal, t, fraction };
}

} // namespace vehicle_sim::math
