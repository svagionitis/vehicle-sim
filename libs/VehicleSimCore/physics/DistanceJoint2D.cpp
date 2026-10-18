#include "DistanceJoint2D.h"

#include <algorithm>
#include <cmath>

#include "RigidBody2D.h"

namespace vehicle_sim::physics {

namespace {
    inline constexpr double kPi { 3.14159265358979323846 };
    inline constexpr double kEpsilon { 1e-9 };
    inline constexpr double kLengthEpsilon { 1e-6 };
    inline constexpr double kMaxPositionCorrection { 0.2 };
    inline constexpr double kPositionTolerance { 0.005 };
} // namespace

DistanceJoint2D::DistanceJoint2D(const DistanceJointConfig& config) noexcept
    : Joint2D(JointType::Distance, config)
    , mTargetDistance { config.targetDistance }
    , mMinDistance { config.minDistance }
    , mMaxDistance { config.maxDistance }
    , mFrequencyHz { config.frequencyHz }
    , mDampingRatio { config.dampingRatio }
{
    if (mTargetDistance < 0.0) {
        const math::Vec2 pA { getWorldAnchorA() };
        const math::Vec2 pB { getWorldAnchorB() };
        const double d { (pB - pA).length() };
        mTargetDistance = (d > kLengthEpsilon) ? d : 1.0;
    }
}

double DistanceJoint2D::getTargetDistance() const noexcept
{
    return mTargetDistance;
}

void DistanceJoint2D::setTargetDistance(double distance) noexcept
{
    mTargetDistance = (distance >= 0.0) ? distance : 0.0;
}

double DistanceJoint2D::getCurrentDistance() const noexcept
{
    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    return (pB - pA).length();
}

double DistanceJoint2D::getMinDistance() const noexcept
{
    return mMinDistance;
}

void DistanceJoint2D::setMinDistance(double distance) noexcept
{
    mMinDistance = (distance >= 0.0) ? distance : 0.0;
}

double DistanceJoint2D::getMaxDistance() const noexcept
{
    return mMaxDistance;
}

void DistanceJoint2D::setMaxDistance(double distance) noexcept
{
    mMaxDistance = (distance >= 0.0) ? distance : 0.0;
}

double DistanceJoint2D::getFrequencyHz() const noexcept
{
    return mFrequencyHz;
}

void DistanceJoint2D::setFrequencyHz(double hz) noexcept
{
    mFrequencyHz = (hz >= 0.0) ? hz : 0.0;
}

double DistanceJoint2D::getDampingRatio() const noexcept
{
    return mDampingRatio;
}

void DistanceJoint2D::setDampingRatio(double ratio) noexcept
{
    mDampingRatio = (ratio >= 0.0) ? ratio : 0.0;
}

void DistanceJoint2D::initVelocityConstraints(double dt) noexcept
{
    if (dt <= 0.0 || mBodyA == nullptr || mBodyB == nullptr) {
        return;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    mArmA = pA - mBodyA->getPosition();
    mArmB = pB - mBodyB->getPosition();

    const math::Vec2 delta { pB - pA };
    const double length { delta.length() };

    if (length > kLengthEpsilon) {
        mUnitSeparation = delta * (1.0 / length);
    } else {
        mUnitSeparation = math::Vec2 { 1.0, 0.0 };
    }

    const double crA { math::cross(mArmA, mUnitSeparation) };
    const double crB { math::cross(mArmB, mUnitSeparation) };

    const double invMassA { mBodyA->getInverseMass() };
    const double invInertiaA { mBodyA->getInverseInertia() };
    const double invMassB { mBodyB->getInverseMass() };
    const double invInertiaB { mBodyB->getInverseInertia() };

    const double k { invMassA + invMassB + (invInertiaA * crA * crA) + (invInertiaB * crB * crB) };

    const double error { length - mTargetDistance };

    if (mFrequencyHz > 0.0) {
        const double omega { 2.0 * kPi * mFrequencyHz };
        const double mass { (k > kEpsilon) ? (1.0 / k) : 0.0 };
        const double d { 2.0 * mass * mDampingRatio * omega };
        const double springK { mass * omega * omega };

        mGamma = 1.0 / (dt * (d + (dt * springK)));
        const double beta { (dt * springK) / (d + (dt * springK)) };
        mBias = (beta / dt) * error;

        const double kSoft { k + mGamma };
        mEffectiveMass = (kSoft > kEpsilon) ? (1.0 / kSoft) : 0.0;
    } else {
        mGamma = 0.0;
        constexpr double baumgarte { 0.2 };
        mBias = std::clamp((baumgarte / dt) * error, -4.0, 4.0);
        mEffectiveMass = (k > kEpsilon) ? (1.0 / k) : 0.0;
    }

    // Warm starting: apply cached impulse
    const math::Vec2 impulse { mUnitSeparation * mImpulse };
    mBodyA->applyImpulseAtPoint(-impulse, pA);
    mBodyB->applyImpulseAtPoint(impulse, pB);
}

void DistanceJoint2D::solveVelocityConstraints() noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr || mEffectiveMass <= 0.0) {
        return;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 vA { mBodyA->getVelocityAtPoint(pA) };
    const math::Vec2 vB { mBodyB->getVelocityAtPoint(pB) };

    const double cdot { mUnitSeparation.dot(vB - vA) };
    const double deltaImpulse { -mEffectiveMass * (cdot + mBias + (mGamma * mImpulse)) };
    mImpulse += deltaImpulse;

    const math::Vec2 impulse { mUnitSeparation * deltaImpulse };
    mBodyA->applyImpulseAtPoint(-impulse, pA);
    mBodyB->applyImpulseAtPoint(impulse, pB);
}

bool DistanceJoint2D::solvePositionConstraints() noexcept
{
    if (mFrequencyHz > 0.0 || mBodyA == nullptr || mBodyB == nullptr) {
        return true;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 delta { pB - pA };
    const double length { delta.length() };

    math::Vec2 u { 1.0, 0.0 };
    if (length > kLengthEpsilon) {
        u = delta * (1.0 / length);
    }

    const double error { length - mTargetDistance };
    if (std::abs(error) < kPositionTolerance) {
        return true;
    }

    const double clampedError { std::clamp(error, -kMaxPositionCorrection, kMaxPositionCorrection) };

    const math::Vec2 rA { pA - mBodyA->getPosition() };
    const math::Vec2 rB { pB - mBodyB->getPosition() };
    const double crA { math::cross(rA, u) };
    const double crB { math::cross(rB, u) };

    const double invMassA { mBodyA->getInverseMass() };
    const double invInertiaA { mBodyA->getInverseInertia() };
    const double invMassB { mBodyB->getInverseMass() };
    const double invInertiaB { mBodyB->getInverseInertia() };

    const double k { invMassA + invMassB + (invInertiaA * crA * crA) + (invInertiaB * crB * crB) };
    if (k <= kEpsilon) {
        return true;
    }

    const double impulseScalar { -clampedError / k };
    const math::Vec2 impulse { u * impulseScalar };

    if (!mBodyA->isStatic()) {
        mBodyA->setPosition(mBodyA->getPosition() - (impulse * invMassA));
        mBodyA->setOrientation(mBodyA->getOrientation() - (invInertiaA * math::cross(rA, impulse)));
    }

    if (!mBodyB->isStatic()) {
        mBodyB->setPosition(mBodyB->getPosition() + (impulse * invMassB));
        mBodyB->setOrientation(mBodyB->getOrientation() + (invInertiaB * math::cross(rB, impulse)));
    }

    return std::abs(error) < kPositionTolerance;
}

math::Vec2 DistanceJoint2D::getReactionForce(double invDt) const noexcept
{
    return mUnitSeparation * (mImpulse * invDt);
}

double DistanceJoint2D::getReactionTorque(double /*invDt*/) const noexcept
{
    return 0.0;
}

JointSnapshotData DistanceJoint2D::captureState() const noexcept
{
    JointSnapshotData data {};
    data.linearImpulse = math::Vec2 { mImpulse, 0.0 };
    data.angularImpulse = 0.0;
    data.motorImpulse = 0.0;
    data.limitImpulse = 0.0;
    data.isBroken = false;
    return data;
}

void DistanceJoint2D::restoreState(const JointSnapshotData& state) noexcept
{
    mImpulse = state.linearImpulse.x;
}

} // namespace vehicle_sim::physics
