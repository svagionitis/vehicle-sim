#include "AABB2D.h"
#include "OBB2D.h"

namespace vehicle_sim::math {

AABB2D AABB2D::fromPoints(const Vec2* points, std::size_t count) noexcept
{
    if (points == nullptr || count == 0U) {
        return AABB2D { Vec2::zero(), Vec2::zero() };
    }

    Vec2 minVal { points[0] };
    Vec2 maxVal { points[0] };

    for (std::size_t i { 1U }; i < count; ++i) {
        minVal.x = std::min(minVal.x, points[i].x);
        minVal.y = std::min(minVal.y, points[i].y);
        maxVal.x = std::max(maxVal.x, points[i].x);
        maxVal.y = std::max(maxVal.y, points[i].y);
    }

    return AABB2D { minVal, maxVal };
}

void AABB2D::encapsulate(const Vec2& point) noexcept
{
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
}

void AABB2D::encapsulate(const AABB2D& other) noexcept
{
    min.x = std::min(min.x, other.min.x);
    min.y = std::min(min.y, other.min.y);
    max.x = std::max(max.x, other.max.x);
    max.y = std::max(max.y, other.max.y);
}

OBB2D AABB2D::toOBB() const noexcept
{
    return OBB2D { center(), halfExtents(), 0.0 };
}

std::optional<RayHit2D> AABB2D::raycast(const Ray2D& ray) const noexcept
{
    constexpr double kEpsilon { 1e-12 };
    double tMin { 0.0 };
    double tMax { ray.maxDistance };
    Vec2 hitNormal { 0.0, 0.0 };

    // X axis slab test
    if (std::abs(ray.direction.x) < kEpsilon) {
        if (ray.origin.x < min.x || ray.origin.x > max.x) {
            return std::nullopt;
        }
    } else {
        const double invD { 1.0 / ray.direction.x };
        double t0 { (min.x - ray.origin.x) * invD };
        double t1 { (max.x - ray.origin.x) * invD };
        Vec2 n0 { -1.0, 0.0 };
        Vec2 n1 { 1.0, 0.0 };

        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(n0, n1);
        }

        if (t0 > tMin) {
            tMin = t0;
            hitNormal = n0;
        }
        if (t1 < tMax) {
            tMax = t1;
        }

        if (tMin > tMax) {
            return std::nullopt;
        }
    }

    // Y axis slab test
    if (std::abs(ray.direction.y) < kEpsilon) {
        if (ray.origin.y < min.y || ray.origin.y > max.y) {
            return std::nullopt;
        }
    } else {
        const double invD { 1.0 / ray.direction.y };
        double t0 { (min.y - ray.origin.y) * invD };
        double t1 { (max.y - ray.origin.y) * invD };
        Vec2 n0 { 0.0, -1.0 };
        Vec2 n1 { 0.0, 1.0 };

        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(n0, n1);
        }

        if (t0 > tMin) {
            tMin = t0;
            hitNormal = n0;
        }
        if (t1 < tMax) {
            tMax = t1;
        }

        if (tMin > tMax) {
            return std::nullopt;
        }
    }

    // Origin is inside the box if hitNormal was not assigned (tMin was 0.0)
    if (hitNormal.lengthSquared() < 0.5) {
        hitNormal = -ray.direction;
    }

    const double fraction { (ray.maxDistance > 0.0 && !std::isinf(ray.maxDistance)) ? (tMin / ray.maxDistance) : 0.0 };

    return RayHit2D { ray.origin + (ray.direction * tMin), hitNormal, tMin, fraction };
}

} // namespace vehicle_sim::math
