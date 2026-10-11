#include "PhysicsWorld2D.h"

#include <algorithm>

namespace vehicle_sim::physics {

PhysicsWorld2D::PhysicsWorld2D(const math::Vec2& gravity) noexcept
    : mGravity { gravity }
{
}

math::Vec2 PhysicsWorld2D::getGravity() const noexcept
{
    return mGravity;
}

void PhysicsWorld2D::setGravity(const math::Vec2& gravity) noexcept
{
    mGravity = gravity;
}

void PhysicsWorld2D::addBody(RigidBody2D& body)
{
    const auto it = std::find(mBodies.begin(), mBodies.end(), &body);
    if (it == mBodies.end()) {
        mBodies.push_back(&body);
    }
}

void PhysicsWorld2D::removeBody(RigidBody2D& body) noexcept
{
    const auto it = std::remove(mBodies.begin(), mBodies.end(), &body);
    mBodies.erase(it, mBodies.end());
}

void PhysicsWorld2D::clearBodies() noexcept
{
    mBodies.clear();
}

std::size_t PhysicsWorld2D::getBodyCount() const noexcept
{
    return mBodies.size();
}

void PhysicsWorld2D::step(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    for (auto* body : mBodies) {
        if (body != nullptr) {
            Integrator2D::integrate(*body, dt, mGravity);
        }
    }
}

void PhysicsWorld2D::stepBody(RigidBody2D& body, double dt) const noexcept
{
    Integrator2D::integrate(body, dt, mGravity);
}

bool PhysicsWorld2D::resolveCollision(RigidBody2D& bodyA, RigidBody2D* bodyB, const Contact2D& contact) const noexcept
{
    return ContactSolver2D::resolve(bodyA, bodyB, contact);
}

} // namespace vehicle_sim::physics
