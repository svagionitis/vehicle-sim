#include "Integrator2D.h"

#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"

namespace vehicle_sim::physics {

double Integrator2D::normalizeAngle(double angleRadians) noexcept
{
    return math::Angle2D::normalizeRad(angleRadians);
}

void Integrator2D::integrate(RigidBody2D& body, double dt, const math::Vec2& gravity) noexcept
{
    if (dt <= 0.0 || body.isStatic()) {
        return;
    }

    if (body.isKinematic()) {
        const math::Vec2 newPos { body.getPosition() + (body.getLinearVelocity() * dt) };
        body.setPosition(newPos);
        const double newAngle { normalizeAngle(body.getOrientation() + (body.getAngularVelocity() * dt)) };
        body.setOrientation(newAngle);
        body.clearForces();
        return;
    }

    // Dynamic body: compute accelerations
    const math::Vec2 linearAccel { (body.getAccumulatedForce() * body.getInverseMass())
        + (gravity * body.getGravityScale()) };
    const double angularAccel { body.getAccumulatedTorque() * body.getInverseInertia() };

    // Update velocities (first stage of Symplectic Euler)
    math::Vec2 newVel { body.getLinearVelocity() + (linearAccel * dt) };
    double newOmega { body.getAngularVelocity() + (angularAccel * dt) };

    // Apply damping via unconditionally stable implicit formulation
    if (body.getLinearDamping() > 0.0) {
        const double linDampingCoeff { 1.0 / (1.0 + (body.getLinearDamping() * dt)) };
        newVel *= linDampingCoeff;
    }

    if (body.getAngularDamping() > 0.0) {
        const double angDampingCoeff { 1.0 / (1.0 + (body.getAngularDamping() * dt)) };
        newOmega *= angDampingCoeff;
    }

    body.setLinearVelocity(newVel);
    body.setAngularVelocity(newOmega);

    // Update position and orientation using newly updated velocities (Symplectic step)
    const math::Vec2 newPos { body.getPosition() + (newVel * dt) };
    body.setPosition(newPos);

    const double newOrientation { normalizeAngle(body.getOrientation() + (newOmega * dt)) };
    body.setOrientation(newOrientation);

    // Clear force and torque accumulators for next frame
    body.clearForces();
}

} // namespace vehicle_sim::physics
