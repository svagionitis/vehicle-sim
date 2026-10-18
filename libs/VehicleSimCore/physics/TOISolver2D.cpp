#include "TOISolver2D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "libs/VehicleSimCore/math/SweptShape2D.h"

namespace vehicle_sim::physics {

namespace {

[[nodiscard]] double evaluatePolygonSeparation(const math::Polygon2D& polyA,
    const math::Polygon2D& polyB, math::Vec2& outNormal, math::Vec2& outPoint) noexcept
{
    const std::size_t countA { polyA.vertexCount() };
    const std::size_t countB { polyB.vertexCount() };

    double maxSeparation { -std::numeric_limits<double>::infinity() };
    math::Vec2 bestNormal { 0.0, 1.0 };
    math::Vec2 bestPoint { 0.0, 0.0 };

    // Test axes from polyA
    for (std::size_t i { 0 }; i < countA; ++i) {
        const math::Vec2 v1 { polyA.getVertex(i) };
        const math::Vec2 v2 { polyA.getVertex((i + 1) % countA) };
        const math::Vec2 edge { v2 - v1 };
        math::Vec2 normal { -edge.y, edge.x };
        const double len { normal.length() };
        if (len < 1e-9) {
            continue;
        }
        normal = normal * (1.0 / len);

        double minB { std::numeric_limits<double>::infinity() };
        math::Vec2 minPtB { 0.0, 0.0 };
        for (std::size_t j { 0 }; j < countB; ++j) {
            const math::Vec2 vB { polyB.getVertex(j) };
            const double proj { vB.dot(normal) };
            if (proj < minB) {
                minB = proj;
                minPtB = vB;
            }
        }

        double maxA { -std::numeric_limits<double>::infinity() };
        for (std::size_t j { 0 }; j < countA; ++j) {
            const double proj { polyA.getVertex(j).dot(normal) };
            if (proj > maxA) {
                maxA = proj;
            }
        }

        const double separation { minB - maxA };
        if (separation > maxSeparation) {
            maxSeparation = separation;
            bestNormal = normal;
            bestPoint = minPtB;
        }
    }

    // Test axes from polyB
    for (std::size_t i { 0 }; i < countB; ++i) {
        const math::Vec2 v1 { polyB.getVertex(i) };
        const math::Vec2 v2 { polyB.getVertex((i + 1) % countB) };
        const math::Vec2 edge { v2 - v1 };
        math::Vec2 normal { -edge.y, edge.x };
        const double len { normal.length() };
        if (len < 1e-9) {
            continue;
        }
        normal = normal * (1.0 / len);

        double minA { std::numeric_limits<double>::infinity() };
        math::Vec2 minPtA { 0.0, 0.0 };
        for (std::size_t j { 0 }; j < countA; ++j) {
            const math::Vec2 vA { polyA.getVertex(j) };
            const double proj { vA.dot(normal) };
            if (proj < minA) {
                minA = proj;
                minPtA = vA;
            }
        }

        double maxB { -std::numeric_limits<double>::infinity() };
        for (std::size_t j { 0 }; j < countB; ++j) {
            const double proj { polyB.getVertex(j).dot(normal) };
            if (proj > maxB) {
                maxB = proj;
            }
        }

        const double separation { minA - maxB };
        if (separation > maxSeparation) {
            maxSeparation = separation;
            bestNormal = -normal; // Orient from A towards B
            bestPoint = minPtA;
        }
    }

    outNormal = bestNormal;
    outPoint = bestPoint;
    return maxSeparation;
}

} // namespace

double TOISolver2D::computeBoundRadius(const Collider2D& collider) noexcept
{
    const ColliderType type { collider.getType() };
    if (type == ColliderType::Circle) {
        return collider.toWorldCircle().radius;
    }
    if (type == ColliderType::OBB) {
        return collider.toWorldOBB().halfExtents.length();
    }
    if (type == ColliderType::Capsule) {
        const math::Capsule2D cap { collider.toWorldCapsule() };
        return ((cap.p2 - cap.p1).length() * 0.5) + cap.radius;
    }
    if (type == ColliderType::Polygon) {
        const math::Polygon2D poly { collider.toWorldPolygon() };
        const math::Vec2 centroid { poly.computeCentroid() };
        double maxDistSq { 0.0 };
        for (std::size_t i { 0 }; i < poly.vertexCount(); ++i) {
            maxDistSq = std::max(maxDistSq, (poly.getVertex(i) - centroid).lengthSquared());
        }
        return std::sqrt(std::max(0.0, maxDistSq));
    }
    return 1.0;
}

TOIOutput TOISolver2D::solveCircleCircleTOI(const math::Circle2D& a, const math::Vec2& velA,
    const math::Circle2D& b, const math::Vec2& velB, const TOIInput& input) noexcept
{
    TOIOutput output {};
    double tHit { 0.0 };
    math::Vec2 nHit { 0.0, 1.0 };
    math::Vec2 ptHit { 0.0, 0.0 };

    if (math::SweptShape2D::solveSweptCircleCircle(
            a, velA, b, velB, input.dt, tHit, nHit, ptHit)) {
        if (tHit <= 1e-9) {
            output.state = TOIState::Overlapped;
            output.time = 0.0;
        } else {
            output.state = TOIState::Touching;
            output.time = tHit / input.dt;
        }
        output.point = ptHit;
        output.normal = nHit;
        output.iterations = 1;
        return output;
    }

    output.state = TOIState::Separated;
    output.time = 1.0;
    return output;
}

TOIOutput TOISolver2D::solveCircleOBBTOI(const math::Circle2D& circle, const math::Vec2& circleVel,
    const math::OBB2D& obb, const math::Vec2& obbVel, const TOIInput& input) noexcept
{
    TOIOutput output {};
    double tHit { 0.0 };
    math::Vec2 nHit { 0.0, 1.0 };
    math::Vec2 ptHit { 0.0, 0.0 };

    if (math::SweptShape2D::solveSweptCircleOBB(
            circle, circleVel, obb, obbVel, input.dt, tHit, nHit, ptHit)) {
        if (tHit <= 1e-9) {
            output.state = TOIState::Overlapped;
            output.time = 0.0;
        } else {
            output.state = TOIState::Touching;
            output.time = tHit / input.dt;
        }
        output.point = ptHit;
        output.normal = nHit;
        output.iterations = 1;
        return output;
    }

    output.state = TOIState::Separated;
    output.time = 1.0;
    return output;
}

TOIOutput TOISolver2D::solveOBBOBBTOI(const math::OBB2D& a, const math::Vec2& velA, double omegaA,
    const math::OBB2D& b, const math::Vec2& velB, double omegaB, const TOIInput& input) noexcept
{
    const math::Polygon2D polyA { math::Polygon2D::fromOBB(a.center, a.halfExtents, a.rotation) };
    const math::Polygon2D polyB { math::Polygon2D::fromOBB(b.center, b.halfExtents, b.rotation) };
    return solvePolygonTOI(polyA, velA, omegaA, polyB, velB, omegaB, input);
}

TOIOutput TOISolver2D::solvePolygonTOI(const math::Polygon2D& a, const math::Vec2& velA, double omegaA,
    const math::Polygon2D& b, const math::Vec2& velB, double omegaB, const TOIInput& input) noexcept
{
    TOIOutput output {};
    const double radiusA { computeBoundRadius(Collider2D { a }) };
    const double radiusB { computeBoundRadius(Collider2D { b }) };

    double tCurrent { 0.0 };
    std::size_t iter { 0 };

    while (tCurrent < input.dt && iter < input.maxIterations) {
        ++iter;

        // Form transformed polygons at tCurrent
        const math::Vec2 deltaPosA { velA * tCurrent };
        const double deltaRotA { omegaA * tCurrent };
        const math::Transform2D tfA { deltaPosA, deltaRotA };
        std::array<math::Vec2, math::Polygon2D::kMaxVertices> transA {};
        for (std::size_t i { 0 }; i < a.vertexCount(); ++i) {
            transA[i] = tfA.transformPoint(a.getVertex(i));
        }
        const math::Polygon2D curA { transA.data(), a.vertexCount() };

        const math::Vec2 deltaPosB { velB * tCurrent };
        const double deltaRotB { omegaB * tCurrent };
        const math::Transform2D tfB { deltaPosB, deltaRotB };
        std::array<math::Vec2, math::Polygon2D::kMaxVertices> transB {};
        for (std::size_t i { 0 }; i < b.vertexCount(); ++i) {
            transB[i] = tfB.transformPoint(b.getVertex(i));
        }
        const math::Polygon2D curB { transB.data(), b.vertexCount() };

        math::Vec2 normal { 0.0, 1.0 };
        math::Vec2 point { 0.0, 0.0 };
        const double separation { evaluatePolygonSeparation(curA, curB, normal, point) };

        if (separation <= 0.0) {
            output.state = (tCurrent <= 1e-9) ? TOIState::Overlapped : TOIState::Touching;
            output.time = tCurrent / input.dt;
            output.point = point;
            output.normal = normal;
            output.iterations = iter;
            return output;
        }

        if (separation <= input.tolerance) {
            output.state = TOIState::Touching;
            output.time = tCurrent / input.dt;
            output.point = point;
            output.normal = normal;
            output.iterations = iter;
            return output;
        }

        const double relVelNormal { (velB - velA).dot(normal) };
        const double vBound { -relVelNormal + (std::abs(omegaA) * radiusA) + (std::abs(omegaB) * radiusB) };

        if (vBound <= 1e-6) {
            output.state = TOIState::Separated;
            output.time = 1.0;
            output.iterations = iter;
            return output;
        }

        const double dtSafe { (separation - input.tolerance) / vBound };
        const double advance { std::max(1e-5, dtSafe) };
        tCurrent += advance;
    }

    output.state = TOIState::Separated;
    output.time = 1.0;
    output.iterations = iter;
    return output;
}

TOIOutput TOISolver2D::solveTerrainTOI(
    const Collider2D& collider, const terrain::Heightfield1D& terrain, const TOIInput& input) noexcept
{
    TOIOutput output {};
    math::Vec2 vel { 0.0, 0.0 };
    if (collider.getBody() != nullptr) {
        vel = collider.getBody()->getLinearVelocity();
    }

    if (collider.getType() == ColliderType::Circle) {
        const math::Circle2D circ { collider.toWorldCircle() };
        const double x0 { circ.center.x };
        const double x1 { circ.center.x + (vel.x * input.dt) };

        // Test swept circle against terrain profile segments between x0 and x1
        const double minX { std::min(x0, x1) - circ.radius };
        const double maxX { std::max(x0, x1) + circ.radius };

        double earliestT { input.dt + 1.0 };
        math::Vec2 bestNormal { 0.0, 1.0 };
        math::Vec2 bestPoint { 0.0, 0.0 };
        bool hit { false };

        constexpr double kStep { 0.25 };
        for (double x { minX }; x <= maxX; x += kStep) {
            const double nextX { std::min(maxX, x + kStep) };
            const math::Vec2 p1 { x, terrain.getHeight(x) };
            const math::Vec2 p2 { nextX, terrain.getHeight(nextX) };

            double tSeg { 0.0 };
            math::Vec2 nSeg { 0.0, 1.0 };
            math::Vec2 ptSeg { 0.0, 0.0 };
            if (math::SweptShape2D::solveSweptCircleSegment(
                    circ, vel, p1, p2, input.dt, tSeg, nSeg, ptSeg)) {
                if (tSeg < earliestT) {
                    earliestT = tSeg;
                    bestNormal = nSeg;
                    bestPoint = ptSeg;
                    hit = true;
                }
            }
        }

        if (hit && earliestT <= input.dt) {
            output.state = (earliestT <= 1e-9) ? TOIState::Overlapped : TOIState::Touching;
            output.time = earliestT / input.dt;
            output.normal = bestNormal;
            output.point = bestPoint;
            output.iterations = 1;
            return output;
        }
    }

    output.state = TOIState::Separated;
    output.time = 1.0;
    return output;
}

TOIOutput TOISolver2D::solveColliderTOI(
    const Collider2D& colA, const Collider2D& colB, const TOIInput& input) noexcept
{
    math::Vec2 velA { 0.0, 0.0 };
    double omegaA { 0.0 };
    if (colA.getBody() != nullptr) {
        velA = colA.getBody()->getLinearVelocity();
        omegaA = colA.getBody()->getAngularVelocity();
    }

    math::Vec2 velB { 0.0, 0.0 };
    double omegaB { 0.0 };
    if (colB.getBody() != nullptr) {
        velB = colB.getBody()->getLinearVelocity();
        omegaB = colB.getBody()->getAngularVelocity();
    }

    const ColliderType typeA { colA.getType() };
    const ColliderType typeB { colB.getType() };

    // Circle vs Circle
    if (typeA == ColliderType::Circle && typeB == ColliderType::Circle) {
        return solveCircleCircleTOI(colA.toWorldCircle(), velA, colB.toWorldCircle(), velB, input);
    }

    // Circle vs OBB
    if (typeA == ColliderType::Circle && typeB == ColliderType::OBB) {
        return solveCircleOBBTOI(colA.toWorldCircle(), velA, colB.toWorldOBB(), velB, input);
    }
    if (typeA == ColliderType::OBB && typeB == ColliderType::Circle) {
        TOIOutput res { solveCircleOBBTOI(colB.toWorldCircle(), velB, colA.toWorldOBB(), velA, input) };
        res.normal = -res.normal;
        return res;
    }

    // OBB vs OBB
    if (typeA == ColliderType::OBB && typeB == ColliderType::OBB) {
        return solveOBBOBBTOI(
            colA.toWorldOBB(), velA, omegaA, colB.toWorldOBB(), velB, omegaB, input);
    }

    // Default to general convex polygon TOI
    return solvePolygonTOI(
        colA.toWorldPolygon(), velA, omegaA, colB.toWorldPolygon(), velB, omegaB, input);
}

} // namespace vehicle_sim::physics
