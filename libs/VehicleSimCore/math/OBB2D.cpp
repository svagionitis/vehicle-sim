#include "OBB2D.h"

namespace vehicle_sim::math {

std::array<Vec2, 4> OBB2D::vertices() const noexcept
{
    const Vec2 ax { axisX() * halfExtents.x };
    const Vec2 ay { axisY() * halfExtents.y };

    return std::array<Vec2, 4> { center - ax - ay, center + ax - ay, center + ax + ay, center - ax + ay };
}

Vec2 OBB2D::toLocal(const Vec2& worldPoint) const noexcept
{
    const Vec2 diff { worldPoint - center };
    return Vec2 { diff.dot(axisX()), diff.dot(axisY()) };
}

Vec2 OBB2D::toWorld(const Vec2& localPoint) const noexcept
{
    return center + (axisX() * localPoint.x) + (axisY() * localPoint.y);
}

bool OBB2D::contains(const Vec2& point) const noexcept
{
    const Vec2 local { toLocal(point) };
    return (std::abs(local.x) <= halfExtents.x) && (std::abs(local.y) <= halfExtents.y);
}

Vec2 OBB2D::closestPoint(const Vec2& point) const noexcept
{
    const Vec2 local { toLocal(point) };
    const Vec2 clamped { std::clamp(local.x, -halfExtents.x, halfExtents.x),
        std::clamp(local.y, -halfExtents.y, halfExtents.y) };
    return toWorld(clamped);
}

AABB2D OBB2D::computeAABB() const noexcept
{
    const Vec2 ax { axisX() };
    const Vec2 ay { axisY() };

    const double rx { (halfExtents.x * std::abs(ax.x)) + (halfExtents.y * std::abs(ay.x)) };
    const double ry { (halfExtents.x * std::abs(ax.y)) + (halfExtents.y * std::abs(ay.y)) };
    const Vec2 rad { rx, ry };

    return AABB2D { center - rad, center + rad };
}

bool OBB2D::intersects(const OBB2D& other) const noexcept
{
    const Vec2 axes[4] { axisX(), axisY(), other.axisX(), other.axisY() };

    const Vec2 t { other.center - center };

    const auto overlapsOnAxis = [this, &other, &t](const Vec2& axis) noexcept -> bool {
        const double rA { (halfExtents.x * std::abs(axisX().dot(axis)))
            + (halfExtents.y * std::abs(axisY().dot(axis))) };
        const double rB { (other.halfExtents.x * std::abs(other.axisX().dot(axis)))
            + (other.halfExtents.y * std::abs(other.axisY().dot(axis))) };

        const double dist { std::abs(t.dot(axis)) };
        return dist <= (rA + rB);
    };

    for (const Vec2& axis : axes) {
        if (!overlapsOnAxis(axis)) {
            return false;
        }
    }

    return true;
}

bool OBB2D::intersects(const AABB2D& aabb) const noexcept
{
    return intersects(aabb.toOBB());
}

std::optional<RayHit2D> OBB2D::raycast(const Ray2D& ray) const noexcept
{
    const Vec2 locOrigin { toLocal(ray.origin) };
    const Vec2 locDir { ray.direction.dot(axisX()), ray.direction.dot(axisY()) };

    const Ray2D localRay { locOrigin, locDir, ray.maxDistance };
    const AABB2D localBox { -halfExtents, halfExtents };

    const std::optional<RayHit2D> localHit { localBox.raycast(localRay) };
    if (!localHit.has_value()) {
        return std::nullopt;
    }

    const Vec2 worldPoint { toWorld(localHit->point) };
    const Vec2 worldNormal { (axisX() * localHit->normal.x) + (axisY() * localHit->normal.y) };

    return RayHit2D { worldPoint, worldNormal, localHit->distance, localHit->fraction };
}

} // namespace vehicle_sim::math
