#include "Narrowphase2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::physics {

namespace {

    struct ClipVertex {
        math::Vec2 v { 0.0, 0.0 };
    };

    std::size_t clipSegmentToLine(
        const ClipVertex in[2], ClipVertex out[2], const math::Vec2& normal, double offset) noexcept
    {
        std::size_t numOut { 0 };

        const double d0 { normal.dot(in[0].v) - offset };
        const double d1 { normal.dot(in[1].v) - offset };

        if (d0 <= 0.0) {
            out[numOut++] = in[0];
        }
        if (d1 <= 0.0) {
            out[numOut++] = in[1];
        }

        if (d0 * d1 < 0.0 && numOut < 2) {
            const double alpha { d0 / (d0 - d1) };
            out[numOut++].v = in[0].v + ((in[1].v - in[0].v) * alpha);
        }

        return numOut;
    }

} // namespace

double Narrowphase2D::combineRestitution(double eA, double eB) noexcept
{
    const double a { std::clamp(eA, 0.0, 1.0) };
    const double b { std::clamp(eB, 0.0, 1.0) };
    return std::sqrt(a * b);
}

double Narrowphase2D::combineFriction(double muA, double muB) noexcept
{
    const double a { std::max(0.0, muA) };
    const double b { std::max(0.0, muB) };
    return std::sqrt(a * b);
}

bool Narrowphase2D::detectCircleCircle(const math::Circle2D& a, const math::Circle2D& b, const ColliderMaterial& matA,
    const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Vec2 delta { b.center - a.center };
    const double distSq { delta.lengthSquared() };
    const double rSum { a.radius + b.radius };

    if (distSq >= (rSum * rSum)) {
        return false;
    }

    const double dist { std::sqrt(distSq) };
    math::Vec2 normal { 0.0, 1.0 };
    if (dist > 1e-12) {
        normal = delta / dist;
    }

    const double penetration { rSum - dist };
    const math::Vec2 contactPt { a.center + (normal * (a.radius - (0.5 * penetration))) };

    Contact2D c {};
    c.point = contactPt;
    c.normal = normal; // Points A -> B
    c.penetration = penetration;
    c.restitution = combineRestitution(matA.restitution, matB.restitution);
    c.friction = combineFriction(matA.friction, matB.friction);

    outManifold.addContact(c);
    return true;
}

bool Narrowphase2D::detectCircleOBB(const math::Circle2D& circle, const math::OBB2D& obb, const ColliderMaterial& matA,
    const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Vec2 cLoc { obb.toLocal(circle.center) };
    const math::Vec2 clampedLoc { std::clamp(cLoc.x, -obb.halfExtents.x, obb.halfExtents.x),
        std::clamp(cLoc.y, -obb.halfExtents.y, obb.halfExtents.y) };

    const math::Vec2 deltaLoc { cLoc - clampedLoc };
    const double distSq { deltaLoc.lengthSquared() };

    math::Vec2 normalWorld { 0.0, 1.0 };
    math::Vec2 contactWorld { 0.0, 0.0 };
    double penetration { 0.0 };

    if (distSq > 1e-12) {
        const double dist { std::sqrt(distSq) };
        if (dist >= circle.radius) {
            return false;
        }

        penetration = circle.radius - dist;
        // deltaLoc points from box toward circle; normal should point from circle toward box
        const math::Vec2 nLoc { -deltaLoc / dist };
        normalWorld = (obb.axisX() * nLoc.x) + (obb.axisY() * nLoc.y);
        contactWorld = obb.toWorld(clampedLoc);
    } else {
        // Circle center is inside OBB
        const double dxPlus { obb.halfExtents.x - cLoc.x };
        const double dxMinus { obb.halfExtents.x + cLoc.x };
        const double dyPlus { obb.halfExtents.y - cLoc.y };
        const double dyMinus { obb.halfExtents.y + cLoc.y };

        double minOverlap { dxPlus };
        math::Vec2 nLoc { 1.0, 0.0 };

        if (dxMinus < minOverlap) {
            minOverlap = dxMinus;
            nLoc = math::Vec2 { -1.0, 0.0 };
        }
        if (dyPlus < minOverlap) {
            minOverlap = dyPlus;
            nLoc = math::Vec2 { 0.0, 1.0 };
        }
        if (dyMinus < minOverlap) {
            minOverlap = dyMinus;
            nLoc = math::Vec2 { 0.0, -1.0 };
        }

        penetration = circle.radius + minOverlap;
        normalWorld = (obb.axisX() * nLoc.x) + (obb.axisY() * nLoc.y);
        contactWorld = circle.center + (normalWorld * circle.radius);
    }

    Contact2D c {};
    c.point = contactWorld;
    c.normal = normalWorld; // Points from circle to box
    c.penetration = penetration;
    c.restitution = combineRestitution(matA.restitution, matB.restitution);
    c.friction = combineFriction(matA.friction, matB.friction);

    outManifold.addContact(c);
    return true;
}

bool Narrowphase2D::detectCapsuleCircle(const math::Capsule2D& cap, const math::Circle2D& circ,
    const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Vec2 segPt { cap.closestSegmentPoint(circ.center) };
    const math::Circle2D virtualCirc { segPt, cap.radius };
    return detectCircleCircle(virtualCirc, circ, matA, matB, outManifold);
}

bool Narrowphase2D::detectCapsuleCapsule(const math::Capsule2D& a, const math::Capsule2D& b,
    const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Vec2 d1 { a.p2 - a.p1 };
    const math::Vec2 d2 { b.p2 - b.p1 };
    const math::Vec2 r { a.p1 - b.p1 };

    const double l1Sq { d1.lengthSquared() };
    const double l2Sq { d2.lengthSquared() };
    const double f { d2.dot(r) };

    constexpr double kEps { 1e-12 };
    double s { 0.0 };
    double t { 0.0 };

    if (l1Sq <= kEps && l2Sq <= kEps) {
        s = 0.0;
        t = 0.0;
    } else if (l1Sq <= kEps) {
        s = 0.0;
        t = std::clamp(f / l2Sq, 0.0, 1.0);
    } else {
        const double c { d1.dot(r) };
        if (l2Sq <= kEps) {
            t = 0.0;
            s = std::clamp(-c / l1Sq, 0.0, 1.0);
        } else {
            const double bDot { d1.dot(d2) };
            const double denom { (l1Sq * l2Sq) - (bDot * bDot) };

            if (denom > kEps) {
                s = std::clamp(((bDot * f) - (c * l2Sq)) / denom, 0.0, 1.0);
            } else {
                s = 0.0;
            }

            t = (bDot * s) + f;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / l1Sq, 0.0, 1.0);
            } else if (t > l2Sq) {
                t = 1.0;
                s = std::clamp((bDot - c) / l1Sq, 0.0, 1.0);
            } else {
                t /= l2Sq;
            }
        }
    }

    const math::Vec2 ptA { a.p1 + (d1 * s) };
    const math::Vec2 ptB { b.p1 + (d2 * t) };

    const math::Circle2D circA { ptA, a.radius };
    const math::Circle2D circB { ptB, b.radius };

    return detectCircleCircle(circA, circB, matA, matB, outManifold);
}

bool Narrowphase2D::detectCapsuleOBB(const math::Capsule2D& cap, const math::OBB2D& obb, const ColliderMaterial& matA,
    const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Polygon2D polyOBB { math::Polygon2D::fromOBB(obb.center, obb.halfExtents, obb.rotation) };

    const math::Vec2 closestOnOBB { polyOBB.closestPoint(cap.p1) };
    const math::Vec2 segPt { cap.closestSegmentPoint(closestOnOBB) };

    const math::Circle2D virtCircle { segPt, cap.radius };
    return detectCircleOBB(virtCircle, obb, matA, matB, outManifold);
}

bool Narrowphase2D::detectPolygonPolygon(const math::Polygon2D& a, const math::Polygon2D& b,
    const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const std::size_t countA { a.vertexCount() };
    const std::size_t countB { b.vertexCount() };

    if (countA < 3 || countB < 3) {
        return false;
    }

    double minPenetration { 1e30 };
    math::Vec2 bestAxis { 0.0, 1.0 };
    bool axisFromA { true };

    const auto testAxes = [&](const math::Polygon2D& poly1, const math::Polygon2D& poly2, bool isA) {
        const std::size_t n { poly1.vertexCount() };
        for (std::size_t i { 0 }; i < n; ++i) {
            const math::Vec2 normal { poly1.getEdgeNormal(i) };

            double min1 { poly1.getVertex(0).dot(normal) };
            double max1 { min1 };
            for (std::size_t j { 1 }; j < poly1.vertexCount(); ++j) {
                const double p { poly1.getVertex(j).dot(normal) };
                min1 = std::min(min1, p);
                max1 = std::max(max1, p);
            }

            double min2 { poly2.getVertex(0).dot(normal) };
            double max2 { min2 };
            for (std::size_t j { 1 }; j < poly2.vertexCount(); ++j) {
                const double p { poly2.getVertex(j).dot(normal) };
                min2 = std::min(min2, p);
                max2 = std::max(max2, p);
            }

            if (max1 < min2 || max2 < min1) {
                return false;
            }

            const double overlap { std::min(max1 - min2, max2 - min1) };
            if (overlap < minPenetration) {
                minPenetration = overlap;
                bestAxis = normal;
                axisFromA = isA;
            }
        }
        return true;
    };

    if (!testAxes(a, b, true) || !testAxes(b, a, false)) {
        return false;
    }

    const math::Vec2 centerA { a.computeCentroid() };
    const math::Vec2 centerB { b.computeCentroid() };
    if (bestAxis.dot(centerB - centerA) < 0.0) {
        bestAxis = -bestAxis;
    }

    const math::Polygon2D& refPoly { axisFromA ? a : b };
    const math::Polygon2D& incPoly { axisFromA ? b : a };

    std::size_t bestEdgeIndex { 0 };
    double bestDot { -1e30 };
    const math::Vec2 refNormal { axisFromA ? bestAxis : -bestAxis };

    for (std::size_t i { 0 }; i < refPoly.vertexCount(); ++i) {
        const double d { refPoly.getEdgeNormal(i).dot(refNormal) };
        if (d > bestDot) {
            bestDot = d;
            bestEdgeIndex = i;
        }
    }

    const math::Vec2 refV0 { refPoly.getVertex(bestEdgeIndex) };
    const math::Vec2 refV1 { refPoly.getVertex(bestEdgeIndex + 1) };
    const math::Vec2 refEdge { (refV1 - refV0).normalized() };
    const math::Vec2 refEdgeNorm { refPoly.getEdgeNormal(bestEdgeIndex) };

    std::size_t incEdgeIndex { 0 };
    double minDot { 1e30 };
    for (std::size_t i { 0 }; i < incPoly.vertexCount(); ++i) {
        const double d { incPoly.getEdgeNormal(i).dot(refEdgeNorm) };
        if (d < minDot) {
            minDot = d;
            incEdgeIndex = i;
        }
    }

    ClipVertex incVertices[2] { { incPoly.getVertex(incEdgeIndex) }, { incPoly.getVertex(incEdgeIndex + 1) } };

    ClipVertex clipped[2] {};
    const double sideOffset1 { refEdge.dot(refV0) };
    std::size_t numClipped { clipSegmentToLine(incVertices, clipped, -refEdge, -sideOffset1) };
    if (numClipped < 2) {
        return false;
    }

    ClipVertex clipped2[2] {};
    const double sideOffset2 { refEdge.dot(refV1) };
    numClipped = clipSegmentToLine(clipped, clipped2, refEdge, sideOffset2);
    if (numClipped < 2) {
        return false;
    }

    const double frontOffset { refEdgeNorm.dot(refV0) };
    const double restitution { combineRestitution(matA.restitution, matB.restitution) };
    const double friction { combineFriction(matA.friction, matB.friction) };

    for (std::size_t i { 0 }; i < numClipped; ++i) {
        const double depth { refEdgeNorm.dot(clipped2[i].v) - frontOffset };
        if (depth <= 0.0) {
            Contact2D c {};
            c.point = clipped2[i].v;
            c.normal = bestAxis; // Points from A to B
            c.penetration = -depth;
            c.restitution = restitution;
            c.friction = friction;
            outManifold.addContact(c);
        }
    }

    return outManifold.contactCount > 0;
}

bool Narrowphase2D::detectOBBOBB(const math::OBB2D& a, const math::OBB2D& b, const ColliderMaterial& matA,
    const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept
{
    const math::Polygon2D polyA { math::Polygon2D::fromOBB(a.center, a.halfExtents, a.rotation) };
    const math::Polygon2D polyB { math::Polygon2D::fromOBB(b.center, b.halfExtents, b.rotation) };
    return detectPolygonPolygon(polyA, polyB, matA, matB, outManifold);
}

bool Narrowphase2D::detectCompound(const CompoundShape2D& comp, const math::Transform2D& compTf,
    const Collider2D& other, ContactManifold2D& outManifold) noexcept
{
    bool hit { false };
    const std::size_t childCount { comp.getChildCount() };

    for (std::size_t i { 0 }; i < childCount; ++i) {
        const CompoundChild2D& child { comp.getChild(i) };
        const math::Transform2D childTf { compTf * child.localTransform };

        std::visit(
            [&](const auto& s) {
                using T = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<T, math::Circle2D>) {
                    const math::Circle2D worldCirc { childTf.transformPoint(s.center), s.radius };
                    const Collider2D childCol { worldCirc };
                    hit = detectColliders(childCol, other, outManifold) || hit;
                } else if constexpr (std::is_same_v<T, math::OBB2D>) {
                    const math::OBB2D worldOBB { childTf.transformPoint(s.center), s.halfExtents,
                        childTf.getAngle() + s.rotation };
                    const Collider2D childCol { worldOBB };
                    hit = detectColliders(childCol, other, outManifold) || hit;
                } else if constexpr (std::is_same_v<T, math::Capsule2D>) {
                    const math::Capsule2D worldCap { childTf.transformPoint(s.p1), childTf.transformPoint(s.p2),
                        s.radius };
                    const Collider2D childCol { worldCap };
                    hit = detectColliders(childCol, other, outManifold) || hit;
                } else if constexpr (std::is_same_v<T, math::Polygon2D>) {
                    const std::size_t n { s.vertexCount() };
                    std::array<math::Vec2, math::Polygon2D::kMaxVertices> wVerts {};
                    for (std::size_t v { 0 }; v < n; ++v) {
                        wVerts[v] = childTf.transformPoint(s.getVertex(v));
                    }
                    const math::Polygon2D worldPoly { wVerts.data(), n };
                    const Collider2D childCol { worldPoly };
                    hit = detectColliders(childCol, other, outManifold) || hit;
                }
            },
            child.shape);
    }

    return hit;
}

bool Narrowphase2D::detectColliders(
    const Collider2D& colA, const Collider2D& colB, ContactManifold2D& manifold) noexcept
{
    manifold.clear();
    manifold.bodyA = colA.getBody();
    manifold.bodyB = colB.getBody();

    if (!colA.canCollideWith(colB)) {
        return false;
    }

    const ColliderType typeA { colA.getType() };
    const ColliderType typeB { colB.getType() };
    const ColliderMaterial& matA { colA.getMaterial() };
    const ColliderMaterial& matB { colB.getMaterial() };

    if (typeA == ColliderType::Compound) {
        const auto* comp = std::get_if<CompoundShape2D>(&colA.getShape());
        if (comp != nullptr) {
            const math::Transform2D tf { (colA.getBody() != nullptr) ? colA.getBody()->getTransform()
                                                                     : math::Transform2D::identity() };
            return detectCompound(*comp, tf, colB, manifold);
        }
        return false;
    }

    if (typeB == ColliderType::Compound) {
        const auto* comp = std::get_if<CompoundShape2D>(&colB.getShape());
        if (comp != nullptr) {
            const math::Transform2D tf { (colB.getBody() != nullptr) ? colB.getBody()->getTransform()
                                                                     : math::Transform2D::identity() };
            ContactManifold2D revManifold {};
            const bool hit { detectCompound(*comp, tf, colA, revManifold) };
            if (hit) {
                // Reverse normal from B->A to A->B
                for (std::size_t i { 0 }; i < revManifold.contactCount; ++i) {
                    Contact2D c { revManifold.contacts[i] };
                    c.normal = -c.normal;
                    manifold.addContact(c);
                }
                return true;
            }
        }
        return false;
    }

    if (typeA == ColliderType::Circle && typeB == ColliderType::Circle) {
        return detectCircleCircle(colA.toWorldCircle(), colB.toWorldCircle(), matA, matB, manifold);
    }

    if (typeA == ColliderType::Circle && typeB == ColliderType::OBB) {
        return detectCircleOBB(colA.toWorldCircle(), colB.toWorldOBB(), matA, matB, manifold);
    }

    if (typeA == ColliderType::OBB && typeB == ColliderType::Circle) {
        ContactManifold2D revManifold {};
        const bool hit { detectCircleOBB(colB.toWorldCircle(), colA.toWorldOBB(), matB, matA, revManifold) };
        if (hit) {
            for (std::size_t i { 0 }; i < revManifold.contactCount; ++i) {
                Contact2D c { revManifold.contacts[i] };
                c.normal = -c.normal;
                manifold.addContact(c);
            }
            return true;
        }
        return false;
    }

    if (typeA == ColliderType::Capsule && typeB == ColliderType::Circle) {
        return detectCapsuleCircle(colA.toWorldCapsule(), colB.toWorldCircle(), matA, matB, manifold);
    }

    if (typeA == ColliderType::Circle && typeB == ColliderType::Capsule) {
        ContactManifold2D revManifold {};
        const bool hit { detectCapsuleCircle(colB.toWorldCapsule(), colA.toWorldCircle(), matB, matA, revManifold) };
        if (hit) {
            for (std::size_t i { 0 }; i < revManifold.contactCount; ++i) {
                Contact2D c { revManifold.contacts[i] };
                c.normal = -c.normal;
                manifold.addContact(c);
            }
            return true;
        }
        return false;
    }

    if (typeA == ColliderType::Capsule && typeB == ColliderType::Capsule) {
        return detectCapsuleCapsule(colA.toWorldCapsule(), colB.toWorldCapsule(), matA, matB, manifold);
    }

    if (typeA == ColliderType::Capsule && typeB == ColliderType::OBB) {
        return detectCapsuleOBB(colA.toWorldCapsule(), colB.toWorldOBB(), matA, matB, manifold);
    }

    if (typeA == ColliderType::OBB && typeB == ColliderType::Capsule) {
        ContactManifold2D revManifold {};
        const bool hit { detectCapsuleOBB(colB.toWorldCapsule(), colA.toWorldOBB(), matB, matA, revManifold) };
        if (hit) {
            for (std::size_t i { 0 }; i < revManifold.contactCount; ++i) {
                Contact2D c { revManifold.contacts[i] };
                c.normal = -c.normal;
                manifold.addContact(c);
            }
            return true;
        }
        return false;
    }

    if (typeA == ColliderType::OBB && typeB == ColliderType::OBB) {
        return detectOBBOBB(colA.toWorldOBB(), colB.toWorldOBB(), matA, matB, manifold);
    }

    // Default to general polygon SAT
    return detectPolygonPolygon(colA.toWorldPolygon(), colB.toWorldPolygon(), matA, matB, manifold);
}

bool Narrowphase2D::detectTerrain(
    const Collider2D& collider, const terrain::Heightfield1D& terrain, ContactManifold2D& outManifold) noexcept
{
    outManifold.clear();
    outManifold.bodyA = collider.getBody();
    outManifold.bodyB = nullptr; // Barrier

    const ColliderType type { collider.getType() };
    const ColliderMaterial& mat { collider.getMaterial() };

    if (type == ColliderType::Circle) {
        const math::Circle2D circ { collider.toWorldCircle() };
        const double groundY { terrain.getHeight(circ.center.x) };
        const double penetration { (groundY + circ.radius) - circ.center.y };

        if (penetration > 0.0) {
            const math::Vec2 groundNormal { terrain.getNormal(circ.center.x) };
            Contact2D c {};
            c.point = math::Vec2 { circ.center.x, groundY };
            c.normal = -groundNormal; // Points from body towards ground barrier
            c.penetration = penetration;
            c.restitution = mat.restitution;
            c.friction = mat.friction;
            outManifold.addContact(c);
            return true;
        }
        return false;
    }

    if (type == ColliderType::Capsule) {
        const math::Capsule2D cap { collider.toWorldCapsule() };
        const math::Vec2 mid { (cap.p1 + cap.p2) * 0.5 };
        const std::array<math::Vec2, 3> samplePts { cap.p1, mid, cap.p2 };

        bool hit { false };
        for (const math::Vec2& pt : samplePts) {
            const double groundY { terrain.getHeight(pt.x) };
            const double penetration { (groundY + cap.radius) - pt.y };
            if (penetration > 0.0) {
                const math::Vec2 groundNormal { terrain.getNormal(pt.x) };
                Contact2D c {};
                c.point = math::Vec2 { pt.x, groundY };
                c.normal = -groundNormal;
                c.penetration = penetration;
                c.restitution = mat.restitution;
                c.friction = mat.friction;
                outManifold.addContact(c);
                hit = true;
                if (outManifold.contactCount >= 2) {
                    break;
                }
            }
        }
        return hit;
    }

    // OBB and Polygon: sample vertices
    const math::Polygon2D poly { collider.toWorldPolygon() };
    const std::size_t vertexCount { poly.vertexCount() };

    struct Candidate {
        math::Vec2 point;
        double penetration;
        math::Vec2 normal;
    };
    std::array<Candidate, math::Polygon2D::kMaxVertices> candidates {};
    std::size_t numCandidates { 0 };

    for (std::size_t i { 0 }; i < vertexCount; ++i) {
        const math::Vec2 v { poly.getVertex(i) };
        const double groundY { terrain.getHeight(v.x) };
        const double penetration { groundY - v.y };

        if (penetration > 0.0) {
            candidates[numCandidates++] = { v, penetration, -terrain.getNormal(v.x) };
        }
    }

    if (numCandidates == 0) {
        return false;
    }

    // Sort descending by penetration to take deepest contact points
    std::sort(candidates.begin(), candidates.begin() + numCandidates,
        [](const Candidate& a, const Candidate& b) { return a.penetration > b.penetration; });

    const std::size_t contactsToTake { std::min(numCandidates, std::size_t { 2 }) };
    for (std::size_t i { 0 }; i < contactsToTake; ++i) {
        Contact2D c {};
        c.point = candidates[i].point;
        c.normal = candidates[i].normal;
        c.penetration = candidates[i].penetration;
        c.restitution = mat.restitution;
        c.friction = mat.friction;
        outManifold.addContact(c);
    }

    return true;
}

} // namespace vehicle_sim::physics
