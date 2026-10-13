#include "PhysicsWorld2D.h"

#include <algorithm>

#include "Collider2D.h"
#include "ContactManifold2D.h"
#include "Narrowphase2D.h"
#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

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

void PhysicsWorld2D::addJoint(Joint2D& joint)
{
    const auto it = std::find(mJoints.begin(), mJoints.end(), &joint);
    if (it == mJoints.end()) {
        mJoints.push_back(&joint);
    }
}

void PhysicsWorld2D::removeJoint(Joint2D& joint) noexcept
{
    const auto it = std::remove(mJoints.begin(), mJoints.end(), &joint);
    mJoints.erase(it, mJoints.end());
}

void PhysicsWorld2D::clearJoints() noexcept
{
    mJoints.clear();
}

std::size_t PhysicsWorld2D::getJointCount() const noexcept
{
    return mJoints.size();
}

std::size_t PhysicsWorld2D::getVelocityIterations() const noexcept
{
    return mVelocityIterations;
}

void PhysicsWorld2D::setVelocityIterations(std::size_t iterations) noexcept
{
    mVelocityIterations = (iterations > 0) ? iterations : 1;
}

std::size_t PhysicsWorld2D::getPositionIterations() const noexcept
{
    return mPositionIterations;
}

void PhysicsWorld2D::setPositionIterations(std::size_t iterations) noexcept
{
    mPositionIterations = (iterations > 0) ? iterations : 1;
}

void PhysicsWorld2D::step(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    if (mJoints.empty() && mColliders.empty() && mTerrain == nullptr) {
        for (auto* body : mBodies) {
            if (body != nullptr) {
                Integrator2D::integrate(*body, dt, mGravity);
            }
        }
        return;
    }

    // 1. Velocity prediction pass
    for (auto* body : mBodies) {
        if (body == nullptr || body->isStatic()) {
            continue;
        }

        if (body->isKinematic()) {
            body->clearForces();
            continue;
        }

        const math::Vec2 linearAccel { (body->getAccumulatedForce() * body->getInverseMass())
            + (mGravity * body->getGravityScale()) };
        const double angularAccel { body->getAccumulatedTorque() * body->getInverseInertia() };

        math::Vec2 newVel { body->getLinearVelocity() + (linearAccel * dt) };
        double newOmega { body->getAngularVelocity() + (angularAccel * dt) };

        if (body->getLinearDamping() > 0.0) {
            const double linDampingCoeff { 1.0 / (1.0 + (body->getLinearDamping() * dt)) };
            newVel *= linDampingCoeff;
        }

        if (body->getAngularDamping() > 0.0) {
            const double angDampingCoeff { 1.0 / (1.0 + (body->getAngularDamping() * dt)) };
            newOmega *= angDampingCoeff;
        }

        body->setLinearVelocity(newVel);
        body->setAngularVelocity(newOmega);
        body->clearForces();
    }

    // 2. Joint pre-step initialization (warm starting & mass matrix assembly)
    for (auto* joint : mJoints) {
        if (joint != nullptr) {
            joint->initVelocityConstraints(dt);
        }
    }

    // 3. Iterative velocity constraint solve
    for (std::size_t i = 0; i < mVelocityIterations; ++i) {
        for (auto* joint : mJoints) {
            if (joint != nullptr) {
                joint->solveVelocityConstraints();
            }
        }
    }

    // 4. Narrowphase collision detection & contact impulse resolution
    if (!mColliders.empty()) {
        const std::size_t numColliders { mColliders.size() };
        ContactManifold2D manifold {};

        for (std::size_t i { 0 }; i < numColliders; ++i) {
            Collider2D* colA { mColliders[i] };
            if (colA == nullptr) {
                continue;
            }

            const math::AABB2D aabbA { colA->computeWorldAABB() };

            for (std::size_t j { i + 1 }; j < numColliders; ++j) {
                Collider2D* colB { mColliders[j] };
                if (colB == nullptr) {
                    continue;
                }

                if (colA->getBody() != nullptr && colA->getBody() == colB->getBody()) {
                    continue;
                }

                if (aabbA.intersects(colB->computeWorldAABB())) {
                    if (Narrowphase2D::detectColliders(*colA, *colB, manifold)) {
                        if (!colA->isSensor() && !colB->isSensor()) {
                            if (manifold.bodyA != nullptr) {
                                for (std::size_t c { 0 }; c < manifold.contactCount; ++c) {
                                    static_cast<void>(ContactSolver2D::resolve(
                                        *manifold.bodyA, manifold.bodyB, manifold.contacts[c]));
                                }
                            }
                        }
                    }
                }
            }

            if (mTerrain != nullptr && colA->getBody() != nullptr && !colA->isSensor()) {
                if (Narrowphase2D::detectTerrain(*colA, *mTerrain, manifold)) {
                    for (std::size_t c { 0 }; c < manifold.contactCount; ++c) {
                        static_cast<void>(ContactSolver2D::resolve(*manifold.bodyA, nullptr, manifold.contacts[c]));
                    }
                }
            }
        }
    }

    // 5. Symplectic position integration
    for (auto* body : mBodies) {
        if (body == nullptr || body->isStatic()) {
            continue;
        }

        const math::Vec2 newPos { body->getPosition() + (body->getLinearVelocity() * dt) };
        body->setPosition(newPos);

        const double newAngle { math::Angle2D::normalizeRad(
            body->getOrientation() + (body->getAngularVelocity() * dt)) };
        body->setOrientation(newAngle);
    }

    // 6. Iterative position constraint solve
    for (std::size_t i = 0; i < mPositionIterations; ++i) {
        bool allSatisfied { true };
        for (auto* joint : mJoints) {
            if (joint != nullptr) {
                const bool satisfied = joint->solvePositionConstraints();
                allSatisfied = allSatisfied && satisfied;
            }
        }
        if (allSatisfied) {
            break;
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

void PhysicsWorld2D::setFluidField(const FluidField2D* field) noexcept
{
    mFluidField = field;
}

const FluidField2D* PhysicsWorld2D::getFluidField() const noexcept
{
    return mFluidField;
}

void PhysicsWorld2D::addCollider(Collider2D& collider)
{
    const auto it = std::find(mColliders.begin(), mColliders.end(), &collider);
    if (it == mColliders.end()) {
        mColliders.push_back(&collider);
    }
}

void PhysicsWorld2D::removeCollider(Collider2D& collider) noexcept
{
    const auto it = std::remove(mColliders.begin(), mColliders.end(), &collider);
    mColliders.erase(it, mColliders.end());
}

void PhysicsWorld2D::clearColliders() noexcept
{
    mColliders.clear();
}

std::size_t PhysicsWorld2D::getColliderCount() const noexcept
{
    return mColliders.size();
}

void PhysicsWorld2D::setTerrain(const terrain::Heightfield1D* terrain) noexcept
{
    mTerrain = terrain;
}

const terrain::Heightfield1D* PhysicsWorld2D::getTerrain() const noexcept
{
    return mTerrain;
}

} // namespace vehicle_sim::physics
