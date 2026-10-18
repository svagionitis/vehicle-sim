#include "SpeculativeContact2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::physics {

bool SpeculativeContact2D::generateContacts(const Collider2D& colA, const Collider2D& colB,
    double dt, const SpeculativeSettings& settings, ContactManifold2D& outManifold) noexcept
{
    if (colA.isSensor() || colB.isSensor()) {
        return false;
    }

    RigidBody2D* bodyA { colA.getBody() };
    RigidBody2D* bodyB { colB.getBody() };

    // At least one body must be dynamic
    if ((bodyA == nullptr || bodyA->isStatic()) && (bodyB == nullptr || bodyB->isStatic())) {
        return false;
    }

    const math::Vec2 velA { (bodyA != nullptr) ? bodyA->getLinearVelocity() : math::Vec2::zero() };
    const math::Vec2 velB { (bodyB != nullptr) ? bodyB->getLinearVelocity() : math::Vec2::zero() };
    const math::Vec2 relVel { velB - velA };

    const double relSpeed { relVel.length() };
    if (relSpeed < 1e-4) {
        return false;
    }

    const ColliderType typeA { colA.getType() };
    const ColliderType typeB { colB.getType() };

    double distance { -1.0 };
    math::Vec2 normal { 0.0, 1.0 }; // Pointing from A towards B
    math::Vec2 contactPt { 0.0, 0.0 };

    if (typeA == ColliderType::Circle && typeB == ColliderType::Circle) {
        const math::Circle2D circA { colA.toWorldCircle() };
        const math::Circle2D circB { colB.toWorldCircle() };
        const math::Vec2 diff { circB.center - circA.center };
        const double centerDist { diff.length() };
        const double rSum { circA.radius + circB.radius };
        distance = centerDist - rSum;

        if (centerDist > 1e-9) {
            normal = diff * (1.0 / centerDist);
        } else {
            normal = math::Vec2 { 0.0, 1.0 };
        }
        contactPt = circA.center + (normal * circA.radius);
    } else if (typeA == ColliderType::Circle && typeB == ColliderType::OBB) {
        const math::Circle2D circ { colA.toWorldCircle() };
        const math::OBB2D obb { colB.toWorldOBB() };
        const math::Vec2 closestOnOBB { obb.closestPoint(circ.center) };
        const math::Vec2 diff { closestOnOBB - circ.center };
        const double diffLen { diff.length() };
        distance = diffLen - circ.radius;

        if (diffLen > 1e-9) {
            normal = diff * (1.0 / diffLen);
        } else {
            normal = (obb.center - circ.center).lengthSquared() > 1e-9
                ? (obb.center - circ.center).normalized()
                : math::Vec2 { 0.0, 1.0 };
        }
        contactPt = closestOnOBB;
    } else if (typeA == ColliderType::OBB && typeB == ColliderType::Circle) {
        const math::OBB2D obb { colA.toWorldOBB() };
        const math::Circle2D circ { colB.toWorldCircle() };
        const math::Vec2 closestOnOBB { obb.closestPoint(circ.center) };
        const math::Vec2 diff { circ.center - closestOnOBB };
        const double diffLen { diff.length() };
        distance = diffLen - circ.radius;

        if (diffLen > 1e-9) {
            normal = diff * (1.0 / diffLen);
        } else {
            normal = (circ.center - obb.center).lengthSquared() > 1e-9
                ? (circ.center - obb.center).normalized()
                : math::Vec2 { 0.0, 1.0 };
        }
        contactPt = closestOnOBB;
    } else {
        // Fallback for general boxes / polygons: distance between centers minus estimated extents
        const math::AABB2D aabbA { colA.computeWorldAABB() };
        const math::AABB2D aabbB { colB.computeWorldAABB() };

        const math::Vec2 cA { aabbA.center() };
        const math::Vec2 cB { aabbB.center() };
        const math::Vec2 extA { aabbA.extents() * 0.5 };
        const math::Vec2 extB { aabbB.extents() * 0.5 };

        const double dx { std::abs(cB.x - cA.x) - (extA.x + extB.x) };
        const double dy { std::abs(cB.y - cA.y) - (extA.y + extB.y) };

        if (dx > dy) {
            distance = dx;
            normal = (cB.x > cA.x) ? math::Vec2 { 1.0, 0.0 } : math::Vec2 { -1.0, 0.0 };
            contactPt = math::Vec2 { (cB.x > cA.x) ? aabbB.min.x : aabbB.max.x, cB.y };
        } else {
            distance = dy;
            normal = (cB.y > cA.y) ? math::Vec2 { 0.0, 1.0 } : math::Vec2 { 0.0, -1.0 };
            contactPt = math::Vec2 { cB.x, (cB.y > cA.y) ? aabbB.min.y : aabbB.max.y };
        }
    }

    // Only process separated pairs within speculative distance
    if (distance <= 0.0 || distance > settings.maxDistance) {
        return false;
    }

    // Evaluate closing velocity along normal
    const double normalVel { relVel.dot(normal) };
    if (normalVel >= -1e-4) {
        return false; // Moving apart or stationary along normal
    }

    // Voronoi angle threshold: avoid tangential ghost collisions
    if (settings.enableVoronoiClip) {
        const double cosApproach { -normalVel / relSpeed };
        if (cosApproach < settings.angleThreshold) {
            return false;
        }
    }

    // Check if relative velocity will close the distance within time step dt
    const double distanceClosed { -normalVel * dt };
    if (distanceClosed < (distance - 1e-4)) {
        return false; // Will not touch during this time step
    }

    // Emit speculative contact
    outManifold.clear();
    outManifold.bodyA = bodyA;
    outManifold.bodyB = bodyB;

    Contact2D c {};
    c.point = contactPt;
    c.normal = normal;
    c.penetration = 0.0;
    c.targetVelocity = -distance / dt;
    c.isSpeculative = true;
    c.restitution = 0.0;
    c.friction = 0.0;

    return outManifold.addContact(c);
}

} // namespace vehicle_sim::physics
