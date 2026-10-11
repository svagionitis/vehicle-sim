#include "ContactSolver2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::physics {

bool ContactSolver2D::resolve(RigidBody2D& bodyA, RigidBody2D* bodyB, const Contact2D& contact) noexcept
{
    const math::Vec2 velA { bodyA.getVelocityAtPoint(contact.point) };
    const math::Vec2 velB { (bodyB != nullptr) ? bodyB->getVelocityAtPoint(contact.point) : math::Vec2::zero() };
    const math::Vec2 relVel { velB - velA };

    const double normalVel { relVel.dot(contact.normal) };
    if (normalVel >= 0.0) {
        return false;
    }

    const math::Vec2 rA { contact.point - bodyA.getPosition() };
    const double rACrossN { math::cross(rA, contact.normal) };

    double invMassB { 0.0 };
    double invInertiaB { 0.0 };
    double rBCrossN { 0.0 };

    if (bodyB != nullptr) {
        invMassB = bodyB->getInverseMass();
        invInertiaB = bodyB->getInverseInertia();
        const math::Vec2 rB { contact.point - bodyB->getPosition() };
        rBCrossN = math::cross(rB, contact.normal);
    }

    const double kn { bodyA.getInverseMass() + invMassB + (rACrossN * rACrossN * bodyA.getInverseInertia())
        + (rBCrossN * rBCrossN * invInertiaB) };

    if (kn <= 1e-12) {
        return false;
    }

    const double restitution { std::clamp(contact.restitution, 0.0, 1.0) };
    const double jn { -(1.0 + restitution) * normalVel / kn };
    const math::Vec2 normalImpulse { contact.normal * jn };

    bodyA.applyImpulseAtPoint(-normalImpulse, contact.point);
    if (bodyB != nullptr) {
        bodyB->applyImpulseAtPoint(normalImpulse, contact.point);
    }

    if (contact.friction > 0.0) {
        const math::Vec2 tangent { contact.normal.perpendicular() };
        const double tangentVel { relVel.dot(tangent) };

        const double rACrossT { math::cross(rA, tangent) };
        double rBCrossT { 0.0 };
        if (bodyB != nullptr) {
            const math::Vec2 rB { contact.point - bodyB->getPosition() };
            rBCrossT = math::cross(rB, tangent);
        }

        const double kt { bodyA.getInverseMass() + invMassB + (rACrossT * rACrossT * bodyA.getInverseInertia())
            + (rBCrossT * rBCrossT * invInertiaB) };

        if (kt > 1e-12) {
            const double jt { -tangentVel / kt };
            const double maxFriction { contact.friction * jn };
            const double clampedJt { std::clamp(jt, -maxFriction, maxFriction) };
            const math::Vec2 frictionImpulse { tangent * clampedJt };

            bodyA.applyImpulseAtPoint(-frictionImpulse, contact.point);
            if (bodyB != nullptr) {
                bodyB->applyImpulseAtPoint(frictionImpulse, contact.point);
            }
        }
    }

    if (contact.penetration > 0.0001) {
        const double totalInvMass { bodyA.getInverseMass() + invMassB };
        if (totalInvMass > 1e-12) {
            constexpr double kBaumgarte { 0.4 };
            const double correctionMag { (contact.penetration / totalInvMass) * kBaumgarte };
            const math::Vec2 correction { contact.normal * correctionMag };
            bodyA.setPosition(bodyA.getPosition() - (correction * bodyA.getInverseMass()));
            if (bodyB != nullptr) {
                bodyB->setPosition(bodyB->getPosition() + (correction * invMassB));
            }
        }
    }

    return true;
}

} // namespace vehicle_sim::physics
