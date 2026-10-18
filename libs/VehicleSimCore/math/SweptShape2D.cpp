#include "SweptShape2D.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vehicle_sim::math {

bool SweptShape2D::solveQuadratic(double a, double b, double c, double& outRoot) noexcept
{
    constexpr double kEpsilon { 1e-12 };
    if (std::abs(a) < kEpsilon) {
        if (std::abs(b) < kEpsilon) {
            return false;
        }
        const double t { -c / b };
        if (t >= 0.0) {
            outRoot = t;
            return true;
        }
        return false;
    }

    const double discriminant { (b * b) - (4.0 * a * c) };
    if (discriminant < 0.0) {
        return false;
    }

    const double sqrtDisc { std::sqrt(std::max(0.0, discriminant)) };
    double t1 { (-b - sqrtDisc) / (2.0 * a) };
    double t2 { (-b + sqrtDisc) / (2.0 * a) };
    if (t1 > t2) {
        std::swap(t1, t2);
    }

    if (t1 >= 0.0) {
        outRoot = t1;
        return true;
    }
    if (t2 >= 0.0) {
        outRoot = t2;
        return true;
    }

    return false;
}

bool SweptShape2D::solveSweptCircleCircle(const Circle2D& a, const Vec2& velA, const Circle2D& b,
    const Vec2& velB, double maxTime, double& outT, Vec2& outNormal, Vec2& outPoint) noexcept
{
    const Vec2 deltaC { a.center - b.center };
    const double rTotal { a.radius + b.radius };

    const double initDistSq { deltaC.lengthSquared() };
    if (initDistSq <= (rTotal * rTotal)) {
        outT = 0.0;
        const double dist { std::sqrt(std::max(0.0, initDistSq)) };
        if (dist > 1e-9) {
            outNormal = (b.center - a.center) * (1.0 / dist);
        } else {
            outNormal = Vec2 { 0.0, 1.0 };
        }
        outPoint = a.center + (outNormal * a.radius);
        return true;
    }

    const Vec2 deltaV { velA - velB };
    const double quadA { deltaV.lengthSquared() };
    const double quadB { 2.0 * deltaC.dot(deltaV) };
    const double quadC { initDistSq - (rTotal * rTotal) };

    double root { 0.0 };
    if (!solveQuadratic(quadA, quadB, quadC, root)) {
        return false;
    }

    if (root < 0.0 || root > maxTime) {
        return false;
    }

    outT = root;
    const Vec2 posA { a.center + (velA * root) };
    const Vec2 posB { b.center + (velB * root) };
    const Vec2 diff { posB - posA };
    const double dist { diff.length() };
    if (dist > 1e-9) {
        outNormal = diff * (1.0 / dist);
    } else {
        outNormal = Vec2 { 0.0, 1.0 };
    }
    outPoint = posA + (outNormal * a.radius);
    return true;
}

bool SweptShape2D::solveSweptCircleSegment(const Circle2D& circle, const Vec2& vel, const Vec2& p1,
    const Vec2& p2, double maxTime, double& outT, Vec2& outNormal, Vec2& outPoint) noexcept
{
    const Vec2 segDir { p2 - p1 };
    const double segLen { segDir.length() };
    if (segLen < 1e-9) {
        const Circle2D pointCircle { p1, 0.0 };
        return solveSweptCircleCircle(
            circle, vel, pointCircle, Vec2::zero(), maxTime, outT, outNormal, outPoint);
    }

    const Vec2 segU { segDir * (1.0 / segLen) };
    Vec2 segN { -segU.y, segU.x };

    // Orient normal towards circle
    if ((circle.center - p1).dot(segN) < 0.0) {
        segN = -segN;
    }

    bool hitFound { false };
    double earliestT { maxTime + 1.0 };
    Vec2 bestNormal { 0.0, 1.0 };
    Vec2 bestPoint { 0.0, 0.0 };

    // 1. Slab test against infinite segment line offset by circle.radius
    const double vn { vel.dot(segN) };
    const double d0 { (circle.center - p1).dot(segN) };

    if (d0 <= circle.radius) {
        // Initial touch/overlap against line: check segment projection
        const double s { (circle.center - p1).dot(segU) };
        if (s >= 0.0 && s <= segLen) {
            outT = 0.0;
            outNormal = -segN; // Point from circle towards segment
            outPoint = p1 + (segU * s);
            return true;
        }
    } else if (vn < -1e-9) {
        const double tLine { (circle.radius - d0) / vn };
        if (tLine >= 0.0 && tLine <= maxTime) {
            const Vec2 cAtT { circle.center + (vel * tLine) };
            const double s { (cAtT - p1).dot(segU) };
            if (s >= 0.0 && s <= segLen) {
                earliestT = tLine;
                bestNormal = -segN;
                bestPoint = p1 + (segU * s);
                hitFound = true;
            }
        }
    }

    // 2. Test endpoints p1 and p2 as point circles
    const Circle2D circ1 { p1, 0.0 };
    double tP1 { 0.0 };
    Vec2 nP1 { 0.0, 0.0 };
    Vec2 ptP1 { 0.0, 0.0 };
    if (solveSweptCircleCircle(circle, vel, circ1, Vec2::zero(), maxTime, tP1, nP1, ptP1)) {
        if (tP1 < earliestT) {
            earliestT = tP1;
            bestNormal = nP1;
            bestPoint = ptP1;
            hitFound = true;
        }
    }

    const Circle2D circ2 { p2, 0.0 };
    double tP2 { 0.0 };
    Vec2 nP2 { 0.0, 0.0 };
    Vec2 ptP2 { 0.0, 0.0 };
    if (solveSweptCircleCircle(circle, vel, circ2, Vec2::zero(), maxTime, tP2, nP2, ptP2)) {
        if (tP2 < earliestT) {
            earliestT = tP2;
            bestNormal = nP2;
            bestPoint = ptP2;
            hitFound = true;
        }
    }

    if (hitFound && earliestT <= maxTime) {
        outT = earliestT;
        outNormal = bestNormal;
        outPoint = bestPoint;
        return true;
    }

    return false;
}

bool SweptShape2D::solveSweptCircleOBB(const Circle2D& circle, const Vec2& velCircle,
    const OBB2D& obb, const Vec2& velOBB, double maxTime, double& outT, Vec2& outNormal,
    Vec2& outPoint) noexcept
{
    // Transform into local frame of OBB
    const Vec2 relVel { velCircle - velOBB };
    const double cosAngle { std::cos(-obb.rotation) };
    const double sinAngle { std::sin(-obb.rotation) };

    const auto toLocal = [&](const Vec2& worldPt) noexcept -> Vec2 {
        const Vec2 rel { worldPt - obb.center };
        return Vec2 { (rel.x * cosAngle) - (rel.y * sinAngle),
            (rel.x * sinAngle) + (rel.y * cosAngle) };
    };

    const auto toWorldDir = [&](const Vec2& localDir) noexcept -> Vec2 {
        const double c { std::cos(obb.rotation) };
        const double s { std::sin(obb.rotation) };
        return Vec2 { (localDir.x * c) - (localDir.y * s), (localDir.x * s) + (localDir.y * c) };
    };

    const auto toWorldPos = [&](const Vec2& localPt) noexcept -> Vec2 {
        return obb.center + toWorldDir(localPt);
    };

    const Circle2D localCircle { toLocal(circle.center), circle.radius };
    const Vec2 localVel { (relVel.x * cosAngle) - (relVel.y * sinAngle),
        (relVel.x * sinAngle) + (relVel.y * cosAngle) };

    const Vec2 e { obb.halfExtents };
    const Vec2 p0 { -e.x, -e.y };
    const Vec2 p1 { e.x, -e.y };
    const Vec2 p2 { e.x, e.y };
    const Vec2 p3 { -e.x, e.y };

    double earliestT { maxTime + 1.0 };
    Vec2 bestNormal { 0.0, 1.0 };
    Vec2 bestPoint { 0.0, 0.0 };
    bool hitFound { false };

    const std::array<std::pair<Vec2, Vec2>, 4> edges { { { p0, p1 }, { p1, p2 }, { p2, p3 },
        { p3, p0 } } };

    for (const auto& edge : edges) {
        double tHit { 0.0 };
        Vec2 nHit { 0.0, 0.0 };
        Vec2 ptHit { 0.0, 0.0 };
        if (solveSweptCircleSegment(localCircle, localVel, edge.first, edge.second, maxTime, tHit,
                nHit, ptHit)) {
            if (tHit < earliestT) {
                earliestT = tHit;
                bestNormal = nHit;
                bestPoint = ptHit;
                hitFound = true;
            }
        }
    }

    if (hitFound && earliestT <= maxTime) {
        outT = earliestT;
        outNormal = toWorldDir(bestNormal);
        outPoint = toWorldPos(bestPoint);
        return true;
    }

    return false;
}

bool SweptShape2D::solveSweptAABB(const AABB2D& a, const Vec2& velA, const AABB2D& b,
    const Vec2& velB, double maxTime, double& outT, Vec2& outNormal) noexcept
{
    // Expand box B by extents of box A (Minkowski sum)
    const Vec2 extA { (a.max - a.min) * 0.5 };
    const Vec2 extB { (b.max - b.min) * 0.5 };
    const Vec2 centerA { (a.min + a.max) * 0.5 };
    const Vec2 centerB { (b.min + b.max) * 0.5 };

    const Vec2 totalExt { extA + extB };
    const Vec2 minBound { centerB - totalExt };
    const Vec2 maxBound { centerB + totalExt };

    // Check initial overlap
    if (centerA.x >= minBound.x && centerA.x <= maxBound.x && centerA.y >= minBound.y
        && centerA.y <= maxBound.y) {
        outT = 0.0;
        outNormal = (centerB - centerA).lengthSquared() > 1e-9 ? (centerB - centerA).normalized()
                                                               : Vec2 { 0.0, 1.0 };
        return true;
    }

    const Vec2 relVel { velA - velB };

    double tEntryX { -std::numeric_limits<double>::infinity() };
    double tExitX { std::numeric_limits<double>::infinity() };
    double normalX { 0.0 };

    if (std::abs(relVel.x) > 1e-9) {
        double t1 { (minBound.x - centerA.x) / relVel.x };
        double t2 { (maxBound.x - centerA.x) / relVel.x };
        double signX { -1.0 };
        if (t1 > t2) {
            std::swap(t1, t2);
            signX = 1.0;
        }
        tEntryX = t1;
        tExitX = t2;
        normalX = signX;
    } else {
        if (centerA.x < minBound.x || centerA.x > maxBound.x) {
            return false;
        }
    }

    double tEntryY { -std::numeric_limits<double>::infinity() };
    double tExitY { std::numeric_limits<double>::infinity() };
    double normalY { 0.0 };

    if (std::abs(relVel.y) > 1e-9) {
        double t1 { (minBound.y - centerA.y) / relVel.y };
        double t2 { (maxBound.y - centerA.y) / relVel.y };
        double signY { -1.0 };
        if (t1 > t2) {
            std::swap(t1, t2);
            signY = 1.0;
        }
        tEntryY = t1;
        tExitY = t2;
        normalY = signY;
    } else {
        if (centerA.y < minBound.y || centerA.y > maxBound.y) {
            return false;
        }
    }

    const double tEntry { std::max(tEntryX, tEntryY) };
    const double tExit { std::min(tExitX, tExitY) };

    if (tEntry > tExit || (tEntryX < 0.0 && tEntryY < 0.0) || tEntry > maxTime) {
        return false;
    }

    outT = std::max(0.0, tEntry);
    if (tEntryX > tEntryY) {
        outNormal = Vec2 { normalX, 0.0 };
    } else {
        outNormal = Vec2 { 0.0, normalY };
    }

    return true;
}

} // namespace vehicle_sim::math
