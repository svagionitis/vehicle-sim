#include "RevoluteJoint2D.h"

#include <algorithm>
#include <cmath>

#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Angle2D.h"

namespace vehicle_sim::physics {

namespace {
    inline constexpr double kEpsilon { 1e-9 };
    inline constexpr double kMaxPositionCorrection { 0.2 };
    inline constexpr double kPositionTolerance { 0.005 };
    inline constexpr double kAngularSlop { 0.035 }; // ~2 degrees
} // namespace

RevoluteJoint2D::RevoluteJoint2D(const RevoluteJointConfig& config) noexcept
    : Joint2D(JointType::Revolute, config)
    , mReferenceAngle { config.referenceAngle }
    , mEnableLimit { config.enableLimit }
    , mLowerAngle { config.lowerAngle }
    , mUpperAngle { config.upperAngle }
    , mEnableMotor { config.enableMotor }
    , mMotorSpeed { config.motorSpeed }
    , mMaxMotorTorque { config.maxMotorTorque }
{
    if (mReferenceAngle == 0.0 && mBodyA != nullptr && mBodyB != nullptr) {
        mReferenceAngle = math::Angle2D::normalizeRad(mBodyB->getOrientation() - mBodyA->getOrientation());
    }
}

double RevoluteJoint2D::getJointAngle() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    return math::Angle2D::normalizeRad(mBodyB->getOrientation() - mBodyA->getOrientation() - mReferenceAngle);
}

double RevoluteJoint2D::getJointSpeed() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    return mBodyB->getAngularVelocity() - mBodyA->getAngularVelocity();
}

bool RevoluteJoint2D::isLimitEnabled() const noexcept
{
    return mEnableLimit;
}

void RevoluteJoint2D::enableLimit(bool flag) noexcept
{
    mEnableLimit = flag;
}

double RevoluteJoint2D::getLowerLimit() const noexcept
{
    return mLowerAngle;
}

double RevoluteJoint2D::getUpperLimit() const noexcept
{
    return mUpperAngle;
}

void RevoluteJoint2D::setLimits(double lower, double upper) noexcept
{
    mLowerAngle = lower;
    mUpperAngle = upper;
}

bool RevoluteJoint2D::isMotorEnabled() const noexcept
{
    return mEnableMotor;
}

void RevoluteJoint2D::enableMotor(bool flag) noexcept
{
    mEnableMotor = flag;
}

double RevoluteJoint2D::getMotorSpeed() const noexcept
{
    return mMotorSpeed;
}

void RevoluteJoint2D::setMotorSpeed(double speedRadPerSec) noexcept
{
    mMotorSpeed = speedRadPerSec;
}

double RevoluteJoint2D::getMaxMotorTorque() const noexcept
{
    return mMaxMotorTorque;
}

void RevoluteJoint2D::setMaxMotorTorque(double torqueNm) noexcept
{
    mMaxMotorTorque = (torqueNm >= 0.0) ? torqueNm : 0.0;
}

double RevoluteJoint2D::getMotorTorque(double invDt) const noexcept
{
    return mMotorImpulse * invDt;
}

void RevoluteJoint2D::initVelocityConstraints(double dt) noexcept
{
    if (dt <= 0.0 || mBodyA == nullptr || mBodyB == nullptr) {
        return;
    }

    mDt = dt;

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    mArmA = pA - mBodyA->getPosition();
    mArmB = pB - mBodyB->getPosition();

    const double invMassA { mBodyA->getInverseMass() };
    const double invInertiaA { mBodyA->getInverseInertia() };
    const double invMassB { mBodyB->getInverseMass() };
    const double invInertiaB { mBodyB->getInverseInertia() };

    // Point constraint effective mass matrix K
    const double k11 { invMassA + invMassB + (invInertiaA * mArmA.y * mArmA.y) + (invInertiaB * mArmB.y * mArmB.y) };
    const double k12 { -(invInertiaA * mArmA.x * mArmA.y) - (invInertiaB * mArmB.x * mArmB.y) };
    const double k22 { invMassA + invMassB + (invInertiaA * mArmA.x * mArmA.x) + (invInertiaB * mArmB.x * mArmB.x) };

    const math::Mat2 kMat { k11, k12, k12, k22 };
    const auto invK = kMat.inverse(kEpsilon);
    if (invK.has_value()) {
        mEffectiveMass = *invK;
    } else {
        mEffectiveMass = math::Mat2::zero();
    }

    // Angular mass
    const double invInertiaSum { invInertiaA + invInertiaB };
    if (invInertiaSum > kEpsilon) {
        mAngularMass = 1.0 / invInertiaSum;
    } else {
        mAngularMass = 0.0;
    }

    // Determine limit state
    if (!mEnableLimit) {
        mLimitState = LimitState::Inactive;
        mLimitImpulse = 0.0;
    } else {
        const double jointAngle { getJointAngle() };
        if (std::abs(mUpperAngle - mLowerAngle) < 1e-4) {
            mLimitState = LimitState::Equal;
        } else if (jointAngle <= mLowerAngle) {
            if (mLimitState != LimitState::AtLower) {
                mLimitImpulse = 0.0;
            }
            mLimitState = LimitState::AtLower;
        } else if (jointAngle >= mUpperAngle) {
            if (mLimitState != LimitState::AtUpper) {
                mLimitImpulse = 0.0;
            }
            mLimitState = LimitState::AtUpper;
        } else {
            mLimitState = LimitState::Inactive;
            mLimitImpulse = 0.0;
        }
    }

    if (!mEnableMotor) {
        mMotorImpulse = 0.0;
    }

    // Baumgarte position bias for point constraint
    constexpr double baumgarte { 0.2 };
    const math::Vec2 positionError { pB - pA };
    mPointBias = math::Vec2 { std::clamp((baumgarte / dt) * positionError.x, -4.0, 4.0),
        std::clamp((baumgarte / dt) * positionError.y, -4.0, 4.0) };

    // Warm starting: apply cached point and angular impulses
    mBodyA->applyImpulseAtPoint(-mPointImpulse, pA);
    mBodyB->applyImpulseAtPoint(mPointImpulse, pB);

    const double totalAngularImpulse { mMotorImpulse + mLimitImpulse };
    mBodyA->applyAngularImpulse(-totalAngularImpulse);
    mBodyB->applyAngularImpulse(totalAngularImpulse);
}

void RevoluteJoint2D::solveVelocityConstraints() noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return;
    }

    // 1. Solve motor constraint
    if (mEnableMotor && mLimitState != LimitState::Equal) {
        const double cdot { (mBodyB->getAngularVelocity() - mBodyA->getAngularVelocity()) - mMotorSpeed };
        double impulse { -mAngularMass * cdot };
        const double oldImpulse { mMotorImpulse };
        const double maxImpulse { mMaxMotorTorque * mDt };
        mMotorImpulse = std::clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = mMotorImpulse - oldImpulse;

        mBodyA->applyAngularImpulse(-impulse);
        mBodyB->applyAngularImpulse(impulse);
    }

    // 2. Solve limit constraint
    if (mEnableLimit && mLimitState != LimitState::Inactive) {
        const double cdot { mBodyB->getAngularVelocity() - mBodyA->getAngularVelocity() };
        const double impulse { -mAngularMass * cdot };

        if (mLimitState == LimitState::Equal) {
            mLimitImpulse += impulse;
            mBodyA->applyAngularImpulse(-impulse);
            mBodyB->applyAngularImpulse(impulse);
        } else if (mLimitState == LimitState::AtLower) {
            const double oldImpulse { mLimitImpulse };
            mLimitImpulse = std::max(oldImpulse + impulse, 0.0);
            const double deltaImpulse { mLimitImpulse - oldImpulse };
            mBodyA->applyAngularImpulse(-deltaImpulse);
            mBodyB->applyAngularImpulse(deltaImpulse);
        } else if (mLimitState == LimitState::AtUpper) {
            const double oldImpulse { mLimitImpulse };
            mLimitImpulse = std::min(oldImpulse + impulse, 0.0);
            const double deltaImpulse { mLimitImpulse - oldImpulse };
            mBodyA->applyAngularImpulse(-deltaImpulse);
            mBodyB->applyAngularImpulse(deltaImpulse);
        }
    }

    // 3. Solve point constraint
    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 vA { mBodyA->getVelocityAtPoint(pA) };
    const math::Vec2 vB { mBodyB->getVelocityAtPoint(pB) };
    const math::Vec2 cdot { (vB - vA) + mPointBias };

    const math::Vec2 deltaImpulse { mEffectiveMass * (-cdot) };
    mPointImpulse += deltaImpulse;

    mBodyA->applyImpulseAtPoint(-deltaImpulse, pA);
    mBodyB->applyImpulseAtPoint(deltaImpulse, pB);
}

bool RevoluteJoint2D::solvePositionConstraints() noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return true;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 positionError { pB - pA };
    const double linearError { positionError.length() };

    math::Vec2 clampedError { positionError };
    if (linearError > kMaxPositionCorrection) {
        clampedError = positionError * (kMaxPositionCorrection / linearError);
    }

    const math::Vec2 rA { pA - mBodyA->getPosition() };
    const math::Vec2 rB { pB - mBodyB->getPosition() };

    const double invMassA { mBodyA->getInverseMass() };
    const double invInertiaA { mBodyA->getInverseInertia() };
    const double invMassB { mBodyB->getInverseMass() };
    const double invInertiaB { mBodyB->getInverseInertia() };

    const double k11 { invMassA + invMassB + (invInertiaA * rA.y * rA.y) + (invInertiaB * rB.y * rB.y) };
    const double k12 { -(invInertiaA * rA.x * rA.y) - (invInertiaB * rB.x * rB.y) };
    const double k22 { invMassA + invMassB + (invInertiaA * rA.x * rA.x) + (invInertiaB * rB.x * rB.x) };

    const math::Mat2 kMat { k11, k12, k12, k22 };
    const auto invK = kMat.inverse(kEpsilon);

    if (invK.has_value()) {
        const math::Vec2 impulse { (*invK) * (-clampedError) };

        if (!mBodyA->isStatic()) {
            mBodyA->setPosition(mBodyA->getPosition() - (impulse * invMassA));
            mBodyA->setOrientation(mBodyA->getOrientation() - (invInertiaA * math::cross(rA, impulse)));
        }

        if (!mBodyB->isStatic()) {
            mBodyB->setPosition(mBodyB->getPosition() + (impulse * invMassB));
            mBodyB->setOrientation(mBodyB->getOrientation() + (invInertiaB * math::cross(rB, impulse)));
        }
    }

    // Angular limit position correction
    bool angularSatisfied { true };
    if (mEnableLimit && mLimitState != LimitState::Inactive) {
        const double jointAngle { getJointAngle() };
        double angularError { 0.0 };

        if (mLimitState == LimitState::Equal) {
            angularError = std::clamp(jointAngle - mLowerAngle, -kAngularSlop, kAngularSlop);
        } else if (mLimitState == LimitState::AtLower) {
            const double diff { jointAngle - mLowerAngle };
            if (diff < -kAngularSlop) {
                angularError = std::clamp(diff + kAngularSlop, -kAngularSlop, 0.0);
            }
        } else if (mLimitState == LimitState::AtUpper) {
            const double diff { jointAngle - mUpperAngle };
            if (diff > kAngularSlop) {
                angularError = std::clamp(diff - kAngularSlop, 0.0, kAngularSlop);
            }
        }

        if (std::abs(angularError) > 1e-4 && mAngularMass > kEpsilon) {
            const double angularImpulse { -mAngularMass * angularError };
            if (!mBodyA->isStatic()) {
                mBodyA->setOrientation(mBodyA->getOrientation() - (invInertiaA * angularImpulse));
            }
            if (!mBodyB->isStatic()) {
                mBodyB->setOrientation(mBodyB->getOrientation() + (invInertiaB * angularImpulse));
            }
            angularSatisfied = false;
        }
    }

    return (linearError < kPositionTolerance) && angularSatisfied;
}

math::Vec2 RevoluteJoint2D::getReactionForce(double invDt) const noexcept
{
    return mPointImpulse * invDt;
}

double RevoluteJoint2D::getReactionTorque(double invDt) const noexcept
{
    return (mMotorImpulse + mLimitImpulse) * invDt;
}

JointSnapshotData RevoluteJoint2D::captureState() const noexcept
{
    JointSnapshotData data {};
    data.linearImpulse = mPointImpulse;
    data.angularImpulse = 0.0;
    data.motorImpulse = mMotorImpulse;
    data.limitImpulse = mLimitImpulse;
    data.isBroken = false;
    return data;
}

void RevoluteJoint2D::restoreState(const JointSnapshotData& state) noexcept
{
    mPointImpulse = state.linearImpulse;
    mMotorImpulse = state.motorImpulse;
    mLimitImpulse = state.limitImpulse;
}

} // namespace vehicle_sim::physics
