#include "WheelJoint2D.h"

#include <algorithm>
#include <cmath>

#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Angle2D.h"

namespace vehicle_sim::physics {

namespace {

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kLinearSlop { 0.005 };
inline constexpr double kMaxLinearCorrection { 0.2 };
inline constexpr double kDetTolerance { 1e-12 };

} // namespace

WheelJoint2D::WheelJoint2D(const WheelJointConfig& config) noexcept
    : Joint2D(JointType::Wheel, config)
    , mLocalAxisA { config.localAxisA.lengthSquared() > 1e-12 ? config.localAxisA.normalized()
                                                              : math::Vec2 { 0.0, 1.0 } }
    , mFrequencyHz { std::max(0.0, config.frequencyHz) }
    , mDampingRatio { std::max(0.0, config.dampingRatio) }
    , mEnableLimit { config.enableLimit }
    , mLowerTranslation { config.lowerTranslation }
    , mUpperTranslation { config.upperTranslation }
    , mEnableMotor { config.enableMotor }
    , mMotorSpeed { config.motorSpeed }
    , mMaxMotorTorque { std::max(0.0, config.maxMotorTorque) }
{
}

double WheelJoint2D::getJointTranslation() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    const math::Vec2 pA { mBodyA->toWorldPoint(mLocalAnchorA) };
    const math::Vec2 pB { mBodyB->toWorldPoint(mLocalAnchorB) };
    const math::Vec2 axis { mBodyA->toWorldVec(mLocalAxisA).normalized() };
    return (pB - pA).dot(axis);
}

double WheelJoint2D::getJointLinearSpeed() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    const math::Vec2 axis { mBodyA->toWorldVec(mLocalAxisA).normalized() };
    const math::Vec2 rA { mBodyA->toWorldVec(mLocalAnchorA) };
    const math::Vec2 rB { mBodyB->toWorldVec(mLocalAnchorB) };
    const math::Vec2 pA { mBodyA->getPosition() + rA };
    const math::Vec2 pB { mBodyB->getPosition() + rB };
    const math::Vec2 d { pB - pA };

    const math::Vec2 vA { mBodyA->getLinearVelocity() };
    const double wA { mBodyA->getAngularVelocity() };
    const math::Vec2 vB { mBodyB->getLinearVelocity() };
    const double wB { mBodyB->getAngularVelocity() };

    const double sA1 { (rA + d).cross(axis) };
    const double sB1 { rB.cross(axis) };

    return axis.dot(vB - vA) + (sB1 * wB) - (sA1 * wA);
}

double WheelJoint2D::getJointAngularSpeed() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    return mBodyB->getAngularVelocity() - mBodyA->getAngularVelocity();
}

double WheelJoint2D::getFrequencyHz() const noexcept
{
    return mFrequencyHz;
}

void WheelJoint2D::setFrequencyHz(double hz) noexcept
{
    mFrequencyHz = std::max(0.0, hz);
}

double WheelJoint2D::getDampingRatio() const noexcept
{
    return mDampingRatio;
}

void WheelJoint2D::setDampingRatio(double ratio) noexcept
{
    mDampingRatio = std::max(0.0, ratio);
}

bool WheelJoint2D::isLimitEnabled() const noexcept
{
    return mEnableLimit;
}

void WheelJoint2D::enableLimit(bool flag) noexcept
{
    mEnableLimit = flag;
}

double WheelJoint2D::getLowerLimit() const noexcept
{
    return mLowerTranslation;
}

double WheelJoint2D::getUpperLimit() const noexcept
{
    return mUpperTranslation;
}

void WheelJoint2D::setLimits(double lower, double upper) noexcept
{
    mLowerTranslation = lower;
    mUpperTranslation = upper;
}

bool WheelJoint2D::isMotorEnabled() const noexcept
{
    return mEnableMotor;
}

void WheelJoint2D::enableMotor(bool flag) noexcept
{
    mEnableMotor = flag;
}

double WheelJoint2D::getMotorSpeed() const noexcept
{
    return mMotorSpeed;
}

void WheelJoint2D::setMotorSpeed(double speed) noexcept
{
    mMotorSpeed = speed;
}

double WheelJoint2D::getMaxMotorTorque() const noexcept
{
    return mMaxMotorTorque;
}

void WheelJoint2D::setMaxMotorTorque(double torque) noexcept
{
    mMaxMotorTorque = std::max(0.0, torque);
}

double WheelJoint2D::getMotorTorque(double invDt) const noexcept
{
    return mMotorImpulse * invDt;
}

void WheelJoint2D::initVelocityConstraints(double dt) noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr || dt <= 0.0) {
        return;
    }

    mDt = dt;
    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    mAxis = mBodyA->toWorldVec(mLocalAxisA).normalized();
    mPerpAxis = math::Vec2 { -mAxis.y, mAxis.x };

    mArmA = mBodyA->toWorldVec(mLocalAnchorA);
    mArmB = mBodyB->toWorldVec(mLocalAnchorB);
    mDelta = (mBodyB->getPosition() + mArmB) - (mBodyA->getPosition() + mArmA);

    mArmA1 = (mArmA + mDelta).cross(mAxis);
    mArmB1 = mArmB.cross(mAxis);
    mArmA2 = (mArmA + mDelta).cross(mPerpAxis);
    mArmB2 = mArmB.cross(mPerpAxis);

    // 1. Perpendicular mass
    const double kPerp { mA + mB + (iA * mArmA2 * mArmA2) + (iB * mArmB2 * mArmB2) };
    mPerpMass = (kPerp > kDetTolerance) ? (1.0 / kPerp) : 0.0;

    // 2. Axial spring mass & regularized Baumgarte parameters
    const double kAxial { mA + mB + (iA * mArmA1 * mArmA1) + (iB * mArmB1 * mArmB1) };
    mGamma = 0.0;
    mSpringBias = 0.0;

    if (mFrequencyHz > 0.0 && kAxial > kDetTolerance) {
        const double mEff { 1.0 / kAxial };
        const double omega { 2.0 * kPi * mFrequencyHz };
        const double d { 2.0 * mEff * mDampingRatio * omega };
        const double k { mEff * omega * omega };

        mGamma = 1.0 / (dt * (d + (dt * k)));
        const double beta { (dt * k) / (d + (dt * k)) };
        mAxialMass = 1.0 / (kAxial + mGamma);

        const double translation { mDelta.dot(mAxis) };
        mSpringBias = (beta / dt) * translation;
    } else {
        mAxialMass = (kAxial > kDetTolerance) ? (1.0 / kAxial) : 0.0;
    }

    // 3. Motor mass
    const double kMotor { iA + iB };
    mMotorMass = (kMotor > kDetTolerance) ? (1.0 / kMotor) : 0.0;

    if (!mEnableMotor) {
        mMotorImpulse = 0.0;
    }

    // Warm start: Apply previous impulses
    const math::Vec2 p { (mPerpAxis * mPerpImpulse) + (mAxis * mAxialImpulse) };
    const double lA { (mArmA + mDelta).cross(p) + mMotorImpulse };
    const double lB { mArmB.cross(p) + mMotorImpulse };

    mBodyA->setLinearVelocity(mBodyA->getLinearVelocity() - (p * mA));
    mBodyA->setAngularVelocity(mBodyA->getAngularVelocity() - (iA * lA));

    mBodyB->setLinearVelocity(mBodyB->getLinearVelocity() + (p * mB));
    mBodyB->setAngularVelocity(mBodyB->getAngularVelocity() + (iB * lB));
}

void WheelJoint2D::solveVelocityConstraints() noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return;
    }

    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    math::Vec2 vA { mBodyA->getLinearVelocity() };
    double wA { mBodyA->getAngularVelocity() };
    math::Vec2 vB { mBodyB->getLinearVelocity() };
    double wB { mBodyB->getAngularVelocity() };

    // 1. Solve wheel hub drive/brake motor
    if (mEnableMotor && mMotorMass > 0.0) {
        const double cDot { (wB - wA) - mMotorSpeed };
        const double impulse { -mMotorMass * cDot };
        const double oldImpulse { mMotorImpulse };
        const double maxImpulse { mMaxMotorTorque * mDt };
        mMotorImpulse = std::clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
        const double dImpulse { mMotorImpulse - oldImpulse };

        wA -= iA * dImpulse;
        wB += iB * dImpulse;
    }

    // 2. Solve axial spring-damper constraint
    if (mAxialMass > 0.0) {
        const double vRel { mAxis.dot(vB - vA) + (mArmB1 * wB) - (mArmA1 * wA) };
        const double cDot { vRel + mSpringBias + (mGamma * mAxialImpulse) };
        double impulse { -mAxialMass * cDot };

        if (mEnableLimit) {
            const double translation { mDelta.dot(mAxis) };
            const double oldImpulse { mAxialImpulse };
            if (translation <= mLowerTranslation) {
                mAxialImpulse = std::max(0.0, oldImpulse + impulse);
                impulse = mAxialImpulse - oldImpulse;
            } else if (translation >= mUpperTranslation) {
                mAxialImpulse = std::min(0.0, oldImpulse + impulse);
                impulse = mAxialImpulse - oldImpulse;
            } else {
                mAxialImpulse += impulse;
            }
        } else {
            mAxialImpulse += impulse;
        }

        const math::Vec2 p { mAxis * impulse };
        vA -= p * mA;
        wA -= iA * ((mArmA + mDelta).cross(p));
        vB += p * mB;
        wB += iB * (mArmB.cross(p));
    }

    // 3. Solve perpendicular lock constraint
    if (mPerpMass > 0.0) {
        const double vPerp { mPerpAxis.dot(vB - vA) + (mArmB2 * wB) - (mArmA2 * wA) };
        const double impulse { -mPerpMass * vPerp };
        mPerpImpulse += impulse;

        const math::Vec2 p { mPerpAxis * impulse };
        vA -= p * mA;
        wA -= iA * ((mArmA + mDelta).cross(p));
        vB += p * mB;
        wB += iB * (mArmB.cross(p));
    }

    mBodyA->setLinearVelocity(vA);
    mBodyA->setAngularVelocity(wA);
    mBodyB->setLinearVelocity(vB);
    mBodyB->setAngularVelocity(wB);
}

bool WheelJoint2D::solvePositionConstraints() noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return true;
    }

    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    const math::Vec2 axis { mBodyA->toWorldVec(mLocalAxisA).normalized() };
    const math::Vec2 perpAxis { -axis.y, axis.x };

    const math::Vec2 rA { mBodyA->toWorldVec(mLocalAnchorA) };
    const math::Vec2 rB { mBodyB->toWorldVec(mLocalAnchorB) };
    const math::Vec2 d { (mBodyB->getPosition() + rB) - (mBodyA->getPosition() + rA) };

    const double cPerp { d.dot(perpAxis) };
    const double linearError { std::abs(cPerp) };

    const double sA2 { (rA + d).cross(perpAxis) };
    const double sB2 { rB.cross(perpAxis) };
    const double kPerp { mA + mB + (iA * sA2 * sA2) + (iB * sB2 * sB2) };

    if (kPerp > kDetTolerance) {
        const double clampedC { std::clamp(cPerp, -kMaxLinearCorrection, kMaxLinearCorrection) };
        const double impulse { -clampedC / kPerp };
        const math::Vec2 p { perpAxis * impulse };

        mBodyA->setPosition(mBodyA->getPosition() - (p * mA));
        mBodyA->setOrientation(
            math::Angle2D::normalizeRad(mBodyA->getOrientation() - (iA * (rA + d).cross(p))));

        mBodyB->setPosition(mBodyB->getPosition() + (p * mB));
        mBodyB->setOrientation(
            math::Angle2D::normalizeRad(mBodyB->getOrientation() + (iB * rB.cross(p))));
    }

    return linearError <= kLinearSlop;
}

math::Vec2 WheelJoint2D::getReactionForce(double invDt) const noexcept
{
    return ((mPerpAxis * mPerpImpulse) + (mAxis * mAxialImpulse)) * invDt;
}

double WheelJoint2D::getReactionTorque(double invDt) const noexcept
{
    return mMotorImpulse * invDt;
}

JointSnapshotData WheelJoint2D::captureState() const noexcept
{
    JointSnapshotData data {};
    data.linearImpulse = math::Vec2 { mPerpImpulse, mAxialImpulse };
    data.angularImpulse = 0.0;
    data.motorImpulse = mMotorImpulse;
    data.limitImpulse = 0.0;
    data.isBroken = false;
    return data;
}

void WheelJoint2D::restoreState(const JointSnapshotData& state) noexcept
{
    mPerpImpulse = state.linearImpulse.x;
    mAxialImpulse = state.linearImpulse.y;
    mMotorImpulse = state.motorImpulse;
}

} // namespace vehicle_sim::physics
