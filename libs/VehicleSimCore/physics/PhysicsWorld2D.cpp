#include "PhysicsWorld2D.h"

#include <algorithm>

#include "Collider2D.h"
#include "ContactManifold2D.h"
#include "Narrowphase2D.h"
#include "SpeculativeContact2D.h"
#include "TOISolver2D.h"
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
        if (joint != nullptr && !joint->isBroken()) {
            joint->initVelocityConstraints(dt);
        }
    }

    // 3. Iterative velocity constraint solve
    for (std::size_t i = 0; i < mVelocityIterations; ++i) {
        for (auto* joint : mJoints) {
            if (joint != nullptr && !joint->isBroken()) {
                joint->solveVelocityConstraints();
            }
        }
    }

    // 4. Narrowphase collision detection & contact impulse resolution
    if (!mColliders.empty()) {
        // 4a. Synchronize broadphase proxies with predictive displacement
        for (auto* col : mColliders) {
            if (col == nullptr) {
                continue;
            }
            const math::AABB2D tightAABB { col->computeWorldAABB() };
            if (col->getBroadphaseProxyId() == kNullBVHNode) {
                const std::int32_t proxyId { mBroadphaseTree.createProxy(tightAABB, col) };
                col->setBroadphaseProxyId(proxyId);
            } else {
                math::Vec2 displacement { 0.0, 0.0 };
                if (col->getBody() != nullptr && !col->getBody()->isStatic()) {
                    displacement = col->getBody()->getLinearVelocity() * dt;
                }
                static_cast<void>(mBroadphaseTree.moveProxy(col->getBroadphaseProxyId(), tightAABB, displacement));
            }
        }

        // 4b. Discover candidate overlapping collider pairs via DynamicBVH2D
        mPairBuffer.clear();
        mBroadphaseTree.queryPairs(mPairBuffer);

        ContactManifold2D manifold {};
        for (const auto& pair : mPairBuffer) {
            Collider2D* colA { pair.colliderA };
            Collider2D* colB { pair.colliderB };
            if (colA == nullptr || colB == nullptr) {
                continue;
            }

            if (Narrowphase2D::detectColliders(*colA, *colB, manifold)) {
                if (!colA->isSensor() && !colB->isSensor() && manifold.bodyA != nullptr) {
                    for (std::size_t c { 0 }; c < manifold.contactCount; ++c) {
                        static_cast<void>(ContactSolver2D::resolve(
                            *manifold.bodyA, manifold.bodyB, manifold.contacts[c]));
                    }
                }
            } else if (mSpeculativeEnabled) {
                if (SpeculativeContact2D::generateContacts(
                        *colA, *colB, dt, mSpeculativeSettings, manifold)) {
                    if (manifold.bodyA != nullptr) {
                        for (std::size_t c { 0 }; c < manifold.contactCount; ++c) {
                            static_cast<void>(ContactSolver2D::resolve(
                                *manifold.bodyA, manifold.bodyB, manifold.contacts[c]));
                        }
                    }
                }
            }
        }

        // 4c. Continuous terrain collisions
        if (mTerrain != nullptr) {
            for (auto* colA : mColliders) {
                if (colA != nullptr && colA->getBody() != nullptr && !colA->isSensor()) {
                    if (Narrowphase2D::detectTerrain(*colA, *mTerrain, manifold)) {
                        for (std::size_t c { 0 }; c < manifold.contactCount; ++c) {
                            static_cast<void>(ContactSolver2D::resolve(*manifold.bodyA, nullptr, manifold.contacts[c]));
                        }
                    }
                }
            }
        }
    }

    // 5. Symplectic position integration with Continuous Collision Detection (CCD)
    for (auto* body : mBodies) {
        if (body == nullptr || body->isStatic()) {
            continue;
        }

        if (!mCCDEnabled || !body->isBullet()) {
            const math::Vec2 newPos { body->getPosition() + (body->getLinearVelocity() * dt) };
            body->setPosition(newPos);

            const double newAngle { math::Angle2D::normalizeRad(
                body->getOrientation() + (body->getAngularVelocity() * dt)) };
            body->setOrientation(newAngle);
            continue;
        }

        // Bullet body CCD sub-stepping
        double timeRemaining { dt };
        std::size_t subStep { 0 };

        while (timeRemaining > 1e-6 && subStep < mMaxCCDSubsteps) {
            ++subStep;

            TOIOutput earliestTOI {};
            earliestTOI.time = 1.0;
            earliestTOI.state = TOIState::Separated;
            Collider2D* hitCollider { nullptr };

            for (auto* colA : mColliders) {
                if (colA == nullptr || colA->getBody() != body) {
                    continue;
                }

                for (auto* colB : mColliders) {
                    if (colB == nullptr || colB->getBody() == body || colB->isSensor()) {
                        continue;
                    }
                    if (!colA->canCollideWith(*colB)) {
                        continue;
                    }

                    const TOIInput subInput { timeRemaining, 1e-4, 16 };
                    const TOIOutput res { TOISolver2D::solveColliderTOI(*colA, *colB, subInput) };
                    if ((res.state == TOIState::Touching || res.state == TOIState::Overlapped)
                        && res.time < earliestTOI.time) {
                        earliestTOI = res;
                        hitCollider = colB;
                    }
                }

                if (mTerrain != nullptr) {
                    const TOIInput subInput { timeRemaining, 1e-4, 16 };
                    const TOIOutput res { TOISolver2D::solveTerrainTOI(*colA, *mTerrain, subInput) };
                    if ((res.state == TOIState::Touching || res.state == TOIState::Overlapped)
                        && res.time < earliestTOI.time) {
                        earliestTOI = res;
                        hitCollider = nullptr;
                    }
                }
            }

            if (earliestTOI.state == TOIState::Touching || earliestTOI.state == TOIState::Overlapped) {
                const double tAdvance { std::max(0.0, earliestTOI.time * timeRemaining) };
                body->setPosition(body->getPosition() + (body->getLinearVelocity() * tAdvance));
                body->setOrientation(math::Angle2D::normalizeRad(
                    body->getOrientation() + (body->getAngularVelocity() * tAdvance)));

                Contact2D impactContact {};
                impactContact.point = earliestTOI.point;
                impactContact.normal = earliestTOI.normal;
                impactContact.penetration = 0.0;
                impactContact.restitution = 0.2;
                impactContact.friction = 0.4;
                RigidBody2D* otherBody { (hitCollider != nullptr) ? hitCollider->getBody() : nullptr };
                static_cast<void>(ContactSolver2D::resolve(*body, otherBody, impactContact));

                timeRemaining -= tAdvance;
            } else {
                body->setPosition(body->getPosition() + (body->getLinearVelocity() * timeRemaining));
                body->setOrientation(math::Angle2D::normalizeRad(
                    body->getOrientation() + (body->getAngularVelocity() * timeRemaining)));
                timeRemaining = 0.0;
            }
        }
    }

    // 6. Iterative position constraint solve
    for (std::size_t i = 0; i < mPositionIterations; ++i) {
        bool allSatisfied { true };
        for (auto* joint : mJoints) {
            if (joint != nullptr && !joint->isBroken()) {
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
        const math::AABB2D aabb { collider.computeWorldAABB() };
        const std::int32_t proxyId { mBroadphaseTree.createProxy(aabb, &collider) };
        collider.setBroadphaseProxyId(proxyId);
    }
}

void PhysicsWorld2D::removeCollider(Collider2D& collider) noexcept
{
    const auto it = std::remove(mColliders.begin(), mColliders.end(), &collider);
    if (it != mColliders.end()) {
        const std::int32_t proxyId { collider.getBroadphaseProxyId() };
        if (proxyId != kNullBVHNode) {
            mBroadphaseTree.destroyProxy(proxyId);
            collider.setBroadphaseProxyId(kNullBVHNode);
        }
        mColliders.erase(it, mColliders.end());
    }
}

void PhysicsWorld2D::clearColliders() noexcept
{
    for (auto* col : mColliders) {
        if (col != nullptr) {
            col->setBroadphaseProxyId(kNullBVHNode);
        }
    }
    mBroadphaseTree.clear();
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

bool PhysicsWorld2D::raycast(
    const math::Ray2D& ray, math::RayHit2D& outHit, Collider2D*& outCollider) const noexcept
{
    outCollider = nullptr;
    if (mColliders.empty()) {
        return false;
    }

    bool hitFound { false };
    double closestDistance { ray.maxDistance };
    math::RayHit2D bestHit {};

    mBroadphaseTree.raycast(ray, [&](const math::Ray2D& r, std::int32_t proxyId) -> double {
        auto* col { static_cast<Collider2D*>(mBroadphaseTree.getUserData(proxyId)) };
        if (col == nullptr) {
            return 1.0;
        }

        const auto shapeHit { col->raycast(r) };
        if (shapeHit.has_value() && shapeHit->distance < closestDistance) {
            hitFound = true;
            closestDistance = shapeHit->distance;
            bestHit = *shapeHit;
            outCollider = col;
            return (ray.maxDistance > 0.0) ? (closestDistance / ray.maxDistance) : 0.0;
        }

        return (ray.maxDistance > 0.0) ? (closestDistance / ray.maxDistance) : 1.0;
    });

    if (hitFound) {
        outHit = bestHit;
        return true;
    }

    return false;
}

const DynamicBVH2D& PhysicsWorld2D::getBroadphaseTree() const noexcept
{
    return mBroadphaseTree;
}

DynamicBVH2D& PhysicsWorld2D::getBroadphaseTree() noexcept
{
    return mBroadphaseTree;
}

void PhysicsWorld2D::setCCDEnabled(bool enabled) noexcept
{
    mCCDEnabled = enabled;
}

bool PhysicsWorld2D::isCCDEnabled() const noexcept
{
    return mCCDEnabled;
}

void PhysicsWorld2D::setSpeculativeEnabled(bool enabled) noexcept
{
    mSpeculativeEnabled = enabled;
}

bool PhysicsWorld2D::isSpeculativeEnabled() const noexcept
{
    return mSpeculativeEnabled;
}

void PhysicsWorld2D::setMaxCCDSubsteps(std::size_t substeps) noexcept
{
    mMaxCCDSubsteps = std::max(std::size_t { 1 }, substeps);
}

std::size_t PhysicsWorld2D::getMaxCCDSubsteps() const noexcept
{
    return mMaxCCDSubsteps;
}

void PhysicsWorld2D::setSpeculativeSettings(const SpeculativeSettings& settings) noexcept
{
    mSpeculativeSettings = settings;
}

const SpeculativeSettings& PhysicsWorld2D::getSpeculativeSettings() const noexcept
{
    return mSpeculativeSettings;
}

} // namespace vehicle_sim::physics
