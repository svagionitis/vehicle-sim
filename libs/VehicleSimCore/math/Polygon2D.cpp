#include "Polygon2D.h"
#include "OBB2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

Polygon2D::Polygon2D(const Vec2* inVertices, std::size_t count) noexcept
{
    if (inVertices == nullptr) {
        mCount = 0;
        return;
    }

    mCount = std::min(count, kMaxVertices);
    for (std::size_t i { 0 }; i < mCount; ++i) {
        mVertices[i] = inVertices[i];
    }
}

Polygon2D Polygon2D::fromEllipse(
    const Vec2& center, double radiusA, double radiusB, double rotation, std::size_t segments) noexcept
{
    const std::size_t n { std::clamp(segments, std::size_t { 3 }, kMaxVertices) };
    std::array<Vec2, kMaxVertices> verts {};

    const double cosR { std::cos(rotation) };
    const double sinR { std::sin(rotation) };
    constexpr double kTwoPi { 6.28318530717958647692 };
    const double dTheta { kTwoPi / static_cast<double>(n) };

    for (std::size_t i { 0 }; i < n; ++i) {
        const double theta { static_cast<double>(i) * dTheta };
        const double lx { radiusA * std::cos(theta) };
        const double ly { radiusB * std::sin(theta) };

        const double wx { center.x + (lx * cosR) - (ly * sinR) };
        const double wy { center.y + (lx * sinR) + (ly * cosR) };
        verts[i] = Vec2 { wx, wy };
    }

    return Polygon2D { verts.data(), n };
}

Polygon2D Polygon2D::fromOBB(const Vec2& center, const Vec2& halfExtents, double rotation) noexcept
{
    const OBB2D obb { center, halfExtents, rotation };
    const std::array<Vec2, 4> v { obb.vertices() };
    return Polygon2D { v.data(), 4 };
}

std::size_t Polygon2D::vertexCount() const noexcept
{
    return mCount;
}

Vec2 Polygon2D::getVertex(std::size_t index) const noexcept
{
    if (mCount == 0) {
        return Vec2::zero();
    }
    return mVertices[index % mCount];
}

Vec2 Polygon2D::getEdgeNormal(std::size_t index) const noexcept
{
    if (mCount < 2) {
        return Vec2 { 0.0, 1.0 };
    }

    const Vec2 vCurrent { mVertices[index % mCount] };
    const Vec2 vNext { mVertices[(index + 1) % mCount] };
    const Vec2 edge { vNext - vCurrent };

    // Outward normal for CCW polygon: (edge.y, -edge.x)
    const Vec2 normal { edge.y, -edge.x };
    const double len { normal.length() };

    if (len <= 1e-12) {
        return Vec2 { 0.0, 1.0 };
    }

    return normal / len;
}

Vec2 Polygon2D::computeCentroid() const noexcept
{
    if (mCount == 0) {
        return Vec2::zero();
    }
    if (mCount == 1) {
        return mVertices[0];
    }
    if (mCount == 2) {
        return (mVertices[0] + mVertices[1]) * 0.5;
    }

    double signedArea { 0.0 };
    double cx { 0.0 };
    double cy { 0.0 };

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2& v0 { mVertices[i] };
        const Vec2& v1 { mVertices[(i + 1) % mCount] };
        const double a { (v0.x * v1.y) - (v1.x * v0.y) };

        signedArea += a;
        cx += (v0.x + v1.x) * a;
        cy += (v0.y + v1.y) * a;
    }

    signedArea *= 0.5;
    if (std::abs(signedArea) <= 1e-12) {
        Vec2 sum { 0.0, 0.0 };
        for (std::size_t i { 0 }; i < mCount; ++i) {
            sum += mVertices[i];
        }
        return sum / static_cast<double>(mCount);
    }

    const double factor { 1.0 / (6.0 * signedArea) };
    return Vec2 { cx * factor, cy * factor };
}

double Polygon2D::computeArea() const noexcept
{
    if (mCount < 3) {
        return 0.0;
    }

    double area { 0.0 };
    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2& v0 { mVertices[i] };
        const Vec2& v1 { mVertices[(i + 1) % mCount] };
        area += (v0.x * v1.y) - (v1.x * v0.y);
    }

    return std::abs(area) * 0.5;
}

AABB2D Polygon2D::computeAABB() const noexcept
{
    if (mCount == 0) {
        return AABB2D { Vec2::zero(), Vec2::zero() };
    }

    double minX { mVertices[0].x };
    double minY { mVertices[0].y };
    double maxX { mVertices[0].x };
    double maxY { mVertices[0].y };

    for (std::size_t i { 1 }; i < mCount; ++i) {
        minX = std::min(minX, mVertices[i].x);
        minY = std::min(minY, mVertices[i].y);
        maxX = std::max(maxX, mVertices[i].x);
        maxY = std::max(maxY, mVertices[i].y);
    }

    return AABB2D { Vec2 { minX, minY }, Vec2 { maxX, maxY } };
}

bool Polygon2D::contains(const Vec2& point) const noexcept
{
    if (mCount < 3) {
        return false;
    }

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2 normal { getEdgeNormal(i) };
        const Vec2 toPoint { point - mVertices[i] };
        if (toPoint.dot(normal) > 1e-9) {
            return false;
        }
    }

    return true;
}

Vec2 Polygon2D::closestPoint(const Vec2& point) const noexcept
{
    if (mCount == 0) {
        return Vec2::zero();
    }
    if (mCount == 1) {
        return mVertices[0];
    }
    if (contains(point)) {
        return point;
    }

    double bestDistSq { 1e30 };
    Vec2 bestPt { mVertices[0] };

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2 v0 { mVertices[i] };
        const Vec2 v1 { mVertices[(i + 1) % mCount] };
        const Vec2 edge { v1 - v0 };
        const double edgeLenSq { edge.lengthSquared() };

        Vec2 candidate { v0 };
        if (edgeLenSq > 1e-12) {
            const double t { std::clamp((point - v0).dot(edge) / edgeLenSq, 0.0, 1.0) };
            candidate = v0 + (edge * t);
        }

        const double distSq { (point - candidate).lengthSquared() };
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestPt = candidate;
        }
    }

    return bestPt;
}

bool Polygon2D::intersects(const Polygon2D& other) const noexcept
{
    if (mCount < 3 || other.mCount < 3) {
        return false;
    }

    const auto project = [](const Polygon2D& poly, const Vec2& axis) {
        double minProj { poly.mVertices[0].dot(axis) };
        double maxProj { minProj };
        for (std::size_t i { 1 }; i < poly.mCount; ++i) {
            const double p { poly.mVertices[i].dot(axis) };
            minProj = std::min(minProj, p);
            maxProj = std::max(maxProj, p);
        }
        return std::pair<double, double> { minProj, maxProj };
    };

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2 axis { getEdgeNormal(i) };
        const auto [minA, maxA] = project(*this, axis);
        const auto [minB, maxB] = project(other, axis);

        if (maxA < minB || maxB < minA) {
            return false;
        }
    }

    for (std::size_t i { 0 }; i < other.mCount; ++i) {
        const Vec2 axis { other.getEdgeNormal(i) };
        const auto [minA, maxA] = project(*this, axis);
        const auto [minB, maxB] = project(other, axis);

        if (maxA < minB || maxB < minA) {
            return false;
        }
    }

    return true;
}

std::optional<RayHit2D> Polygon2D::raycast(const Ray2D& ray) const noexcept
{
    if (mCount < 3) {
        return std::nullopt;
    }

    double tNear { 0.0 };
    double tFar { ray.maxDistance };
    Vec2 hitNormal { 0.0, 0.0 };

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const Vec2 v { mVertices[i] };
        const Vec2 n { getEdgeNormal(i) };

        const double denom { n.dot(ray.direction) };
        const double numer { n.dot(v - ray.origin) };

        if (std::abs(denom) < 1e-12) {
            // Ray parallel to edge
            if (numer < 0.0) {
                return std::nullopt;
            }
        } else {
            const double t { numer / denom };
            if (denom < 0.0) {
                // Ray entering polygon
                if (t > tNear) {
                    tNear = t;
                    hitNormal = n;
                }
            } else {
                // Ray exiting polygon
                if (t < tFar) {
                    tFar = t;
                }
            }

            if (tNear > tFar) {
                return std::nullopt;
            }
        }
    }

    if (tNear < 0.0 || tNear > ray.maxDistance) {
        return std::nullopt;
    }

    const Vec2 hitPoint { ray.origin + (ray.direction * tNear) };
    const double fraction { (ray.maxDistance > 0.0) ? (tNear / ray.maxDistance) : 0.0 };
    return RayHit2D { hitPoint, hitNormal, tNear, fraction };
}

} // namespace vehicle_sim::math
