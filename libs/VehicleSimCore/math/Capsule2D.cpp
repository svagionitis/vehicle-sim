#include "Capsule2D.h"
#include "Circle2D.h"
#include "OBB2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

namespace {

    [[nodiscard]] double segmentSegmentDistSq(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2) noexcept
    {
        const Vec2 d1 { p2 - p1 };
        const Vec2 d2 { q2 - q1 };
        const Vec2 r { p1 - q1 };

        const double a { d1.lengthSquared() };
        const double e { d2.lengthSquared() };
        const double f { d2.dot(r) };

        constexpr double kEps { 1e-12 };

        double s { 0.0 };
        double t { 0.0 };

        if (a <= kEps && e <= kEps) {
            return r.lengthSquared();
        }

        if (a <= kEps) {
            s = 0.0;
            t = std::clamp(f / e, 0.0, 1.0);
        } else {
            const double c { d1.dot(r) };
            if (e <= kEps) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else {
                const double b { d1.dot(d2) };
                const double denom { (a * e) - (b * b) };

                if (denom > kEps) {
                    s = std::clamp(((b * f) - (c * e)) / denom, 0.0, 1.0);
                } else {
                    s = 0.0;
                }

                t = (b * s) + f;
                if (t < 0.0) {
                    t = 0.0;
                    s = std::clamp(-c / a, 0.0, 1.0);
                } else if (t > e) {
                    t = 1.0;
                    s = std::clamp((b - c) / a, 0.0, 1.0);
                } else {
                    t /= e;
                }
            }
        }

        const Vec2 c1 { p1 + (d1 * s) };
        const Vec2 c2 { q1 + (d2 * t) };
        return (c1 - c2).lengthSquared();
    }

} // namespace

double Capsule2D::segmentLength() const noexcept
{
    return (p2 - p1).length();
}

Vec2 Capsule2D::closestSegmentPoint(const Vec2& point) const noexcept
{
    const Vec2 d { p2 - p1 };
    const double lenSq { d.lengthSquared() };

    if (lenSq <= 1e-12) {
        return p1;
    }

    const double t { std::clamp((point - p1).dot(d) / lenSq, 0.0, 1.0) };
    return p1 + (d * t);
}

Vec2 Capsule2D::closestPoint(const Vec2& point) const noexcept
{
    const Vec2 segPt { closestSegmentPoint(point) };
    const Vec2 delta { point - segPt };
    const double distSq { delta.lengthSquared() };

    if (distSq <= (radius * radius)) {
        return point;
    }

    const double dist { std::sqrt(distSq) };
    if (dist <= 1e-12) {
        return segPt + Vec2 { radius, 0.0 };
    }

    return segPt + (delta * (radius / dist));
}

bool Capsule2D::contains(const Vec2& point) const noexcept
{
    const Vec2 segPt { closestSegmentPoint(point) };
    return (point - segPt).lengthSquared() <= (radius * radius);
}

AABB2D Capsule2D::computeAABB() const noexcept
{
    const double minX { std::min(p1.x, p2.x) - radius };
    const double minY { std::min(p1.y, p2.y) - radius };
    const double maxX { std::max(p1.x, p2.x) + radius };
    const double maxY { std::max(p1.y, p2.y) + radius };

    return AABB2D { Vec2 { minX, minY }, Vec2 { maxX, maxY } };
}

bool Capsule2D::intersects(const Capsule2D& other) const noexcept
{
    const double distSq { segmentSegmentDistSq(p1, p2, other.p1, other.p2) };
    const double rSum { radius + other.radius };
    return distSq <= (rSum * rSum);
}

bool Capsule2D::intersects(const Circle2D& circle) const noexcept
{
    const Vec2 segPt { closestSegmentPoint(circle.center) };
    const double rSum { radius + circle.radius };
    return (circle.center - segPt).lengthSquared() <= (rSum * rSum);
}

bool Capsule2D::intersects(const AABB2D& aabb) const noexcept
{
    return intersects(aabb.toOBB());
}

bool Capsule2D::intersects(const OBB2D& obb) const noexcept
{
    const Vec2 locP1 { obb.toLocal(p1) };
    const Vec2 locP2 { obb.toLocal(p2) };
    const Vec2 locDelta { locP2 - locP1 };

    constexpr std::size_t kSamples { 8 };
    for (std::size_t i { 0 }; i <= kSamples; ++i) {
        const double t { static_cast<double>(i) / static_cast<double>(kSamples) };
        const Vec2 sampleLoc { locP1 + (locDelta * t) };
        const Vec2 clamped { std::clamp(sampleLoc.x, -obb.halfExtents.x, obb.halfExtents.x),
            std::clamp(sampleLoc.y, -obb.halfExtents.y, obb.halfExtents.y) };
        if ((sampleLoc - clamped).lengthSquared() <= (radius * radius)) {
            return true;
        }
    }

    return false;
}

std::optional<RayHit2D> Capsule2D::raycast(const Ray2D& ray) const noexcept
{
    std::optional<RayHit2D> bestHit { std::nullopt };

    const Circle2D cap1 { p1, radius };
    const std::optional<RayHit2D> hit1 { cap1.raycast(ray) };
    if (hit1.has_value()) {
        bestHit = hit1;
    }

    const Circle2D cap2 { p2, radius };
    const std::optional<RayHit2D> hit2 { cap2.raycast(ray) };
    if (hit2.has_value()) {
        if (!bestHit.has_value() || hit2->distance < bestHit->distance) {
            bestHit = hit2;
        }
    }

    const Vec2 seg { p2 - p1 };
    const double segLen { seg.length() };

    if (segLen > 1e-9) {
        const Vec2 axisX { seg / segLen };
        const Vec2 axisY { -axisX.y, axisX.x };
        const Vec2 center { (p1 + p2) * 0.5 };
        const Vec2 halfExtents { segLen * 0.5, radius };

        const double rot { std::atan2(axisX.y, axisX.x) };
        const OBB2D bodyBox { center, halfExtents, rot };
        const std::optional<RayHit2D> hitBox { bodyBox.raycast(ray) };

        if (hitBox.has_value()) {
            if (!bestHit.has_value() || hitBox->distance < bestHit->distance) {
                bestHit = hitBox;
            }
        }
    }

    return bestHit;
}

} // namespace vehicle_sim::math
