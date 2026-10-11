#include "RigidBody2D.h"

#include <algorithm>

namespace vehicle_sim::physics {

RigidBody2D::RigidBody2D() noexcept
{
    updateInertialCache();
}

RigidBody2D::RigidBody2D(const RigidBodyConfig& config) noexcept
    : mPosition { config.position }
    , mOrientation { config.orientation }
    , mMass { std::max(0.0, config.mass) }
    , mInertia { std::max(0.0, config.inertia) }
    , mLinearDamping { std::max(0.0, config.linearDamping) }
    , mAngularDamping { std::max(0.0, config.angularDamping) }
    , mGravityScale { config.gravityScale }
    , mBodyType { config.bodyType }
{
    updateInertialCache();
}

void RigidBody2D::updateInertialCache() noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mInvMass = (mMass > 0.0) ? (1.0 / mMass) : 0.0;
        mInvInertia = (mInertia > 0.0) ? (1.0 / mInertia) : 0.0;
    } else {
        mInvMass = 0.0;
        mInvInertia = 0.0;
    }
}

bool RigidBody2D::isDynamic() const noexcept
{
    return mBodyType == BodyType::Dynamic;
}

bool RigidBody2D::isStatic() const noexcept
{
    return mBodyType == BodyType::Static;
}

bool RigidBody2D::isKinematic() const noexcept
{
    return mBodyType == BodyType::Kinematic;
}

BodyType RigidBody2D::getBodyType() const noexcept
{
    return mBodyType;
}

void RigidBody2D::setBodyType(BodyType type) noexcept
{
    mBodyType = type;
    if (mBodyType == BodyType::Static) {
        mLinearVelocity = math::Vec2::zero();
        mAngularVelocity = 0.0;
        mAccumulatedForce = math::Vec2::zero();
        mAccumulatedTorque = 0.0;
    }
    updateInertialCache();
}

math::Vec2 RigidBody2D::getPosition() const noexcept
{
    return mPosition;
}

void RigidBody2D::setPosition(const math::Vec2& pos) noexcept
{
    mPosition = pos;
}

math::Vec2 RigidBody2D::getLinearVelocity() const noexcept
{
    return mLinearVelocity;
}

void RigidBody2D::setLinearVelocity(const math::Vec2& vel) noexcept
{
    if (mBodyType != BodyType::Static) {
        mLinearVelocity = vel;
    }
}

double RigidBody2D::getOrientation() const noexcept
{
    return mOrientation;
}

void RigidBody2D::setOrientation(double angleRadians) noexcept
{
    mOrientation = angleRadians;
}

double RigidBody2D::getAngularVelocity() const noexcept
{
    return mAngularVelocity;
}

void RigidBody2D::setAngularVelocity(double omega) noexcept
{
    if (mBodyType != BodyType::Static) {
        mAngularVelocity = omega;
    }
}

double RigidBody2D::getMass() const noexcept
{
    return mMass;
}

void RigidBody2D::setMass(double massKg) noexcept
{
    mMass = std::max(0.0, massKg);
    updateInertialCache();
}

double RigidBody2D::getInverseMass() const noexcept
{
    return mInvMass;
}

double RigidBody2D::getInertia() const noexcept
{
    return mInertia;
}

void RigidBody2D::setInertia(double inertiaVal) noexcept
{
    mInertia = std::max(0.0, inertiaVal);
    updateInertialCache();
}

double RigidBody2D::getInverseInertia() const noexcept
{
    return mInvInertia;
}

double RigidBody2D::getLinearDamping() const noexcept
{
    return mLinearDamping;
}

void RigidBody2D::setLinearDamping(double damping) noexcept
{
    mLinearDamping = std::max(0.0, damping);
}

double RigidBody2D::getAngularDamping() const noexcept
{
    return mAngularDamping;
}

void RigidBody2D::setAngularDamping(double damping) noexcept
{
    mAngularDamping = std::max(0.0, damping);
}

double RigidBody2D::getGravityScale() const noexcept
{
    return mGravityScale;
}

void RigidBody2D::setGravityScale(double scale) noexcept
{
    mGravityScale = scale;
}

math::Transform2D RigidBody2D::getTransform() const noexcept
{
    return math::Transform2D { mPosition, mOrientation };
}

math::Vec2 RigidBody2D::getAccumulatedForce() const noexcept
{
    return mAccumulatedForce;
}

double RigidBody2D::getAccumulatedTorque() const noexcept
{
    return mAccumulatedTorque;
}

void RigidBody2D::applyForce(const math::Vec2& forceWorld) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mAccumulatedForce += forceWorld;
    }
}

void RigidBody2D::applyForceAtPoint(const math::Vec2& forceWorld, const math::Vec2& pointWorld) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mAccumulatedForce += forceWorld;
        const math::Vec2 r { pointWorld - mPosition };
        mAccumulatedTorque += math::cross(r, forceWorld);
    }
}

void RigidBody2D::applyForceAtLocalPoint(const math::Vec2& forceWorld, const math::Vec2& pointLocal) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        const math::Vec2 rWorld { toWorldVec(pointLocal) };
        applyForceAtPoint(forceWorld, mPosition + rWorld);
    }
}

void RigidBody2D::applyTorque(double torqueNm) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mAccumulatedTorque += torqueNm;
    }
}

void RigidBody2D::applyLinearImpulse(const math::Vec2& impulseWorld) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mLinearVelocity += impulseWorld * mInvMass;
    }
}

void RigidBody2D::applyAngularImpulse(double impulseNmSec) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mAngularVelocity += impulseNmSec * mInvInertia;
    }
}

void RigidBody2D::applyImpulseAtPoint(const math::Vec2& impulseWorld, const math::Vec2& pointWorld) noexcept
{
    if (mBodyType == BodyType::Dynamic) {
        mLinearVelocity += impulseWorld * mInvMass;
        const math::Vec2 r { pointWorld - mPosition };
        mAngularVelocity += math::cross(r, impulseWorld) * mInvInertia;
    }
}

void RigidBody2D::clearForces() noexcept
{
    mAccumulatedForce = math::Vec2::zero();
    mAccumulatedTorque = 0.0;
}

math::Vec2 RigidBody2D::getVelocityAtPoint(const math::Vec2& pointWorld) const noexcept
{
    const math::Vec2 r { pointWorld - mPosition };
    return mLinearVelocity + math::cross(mAngularVelocity, r);
}

math::Vec2 RigidBody2D::getVelocityAtLocalPoint(const math::Vec2& pointLocal) const noexcept
{
    const math::Vec2 rWorld { toWorldVec(pointLocal) };
    return mLinearVelocity + math::cross(mAngularVelocity, rWorld);
}

math::Vec2 RigidBody2D::toWorldPoint(const math::Vec2& pointLocal) const noexcept
{
    return getTransform().transformPoint(pointLocal);
}

math::Vec2 RigidBody2D::toLocalPoint(const math::Vec2& pointWorld) const noexcept
{
    return getTransform().inverseTransformPoint(pointWorld);
}

math::Vec2 RigidBody2D::toWorldVec(const math::Vec2& vecLocal) const noexcept
{
    return getTransform().transformVector(vecLocal);
}

math::Vec2 RigidBody2D::toLocalVec(const math::Vec2& vecWorld) const noexcept
{
    return getTransform().inverseTransformVector(vecWorld);
}

double RigidBody2D::getTranslationalEnergy() const noexcept
{
    return 0.5 * mMass * mLinearVelocity.lengthSquared();
}

double RigidBody2D::getRotationalEnergy() const noexcept
{
    return 0.5 * mInertia * mAngularVelocity * mAngularVelocity;
}

double RigidBody2D::getKineticEnergy() const noexcept
{
    return getTranslationalEnergy() + getRotationalEnergy();
}

math::Vec2 RigidBody2D::getLinearMomentum() const noexcept
{
    return mLinearVelocity * mMass;
}

double RigidBody2D::getAngularMomentum() const noexcept
{
    return mInertia * mAngularVelocity;
}

} // namespace vehicle_sim::physics
