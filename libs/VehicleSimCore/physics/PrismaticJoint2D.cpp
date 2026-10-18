#include "PrismaticJoint2D.h"

#include <algorithm>
#include <cmath>

#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Angle2D.h"

namespace vehicle_sim::physics {

namespace {

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kLinearSlop { 0.005 };
inline constexpr double kAngularSlop { 2.0 / 180.0 * kPi };
inline constexpr double kMaxLinearCorrection { 0.2 };
inline constexpr double kMaxAngularCorrection { 8.0 / 180.0 * kPi };
inline constexpr double kDetTolerance { 1e-12 };

} // namespace

PrismaticJoint2D::PrismaticJoint2D(const PrismaticJointConfig& config) noexcept
    : Joint2D(JointType::Prismatic, config)
    , mLocalAxisA { config.localAxisA.lengthSquared() > 1e-12 ? config.localAxisA.normalized()
                                                              : math::Vec2 { 1.0, 0.0 } }
    , mReferenceAngle { config.referenceAngle }
    , mEnableLimit { config.enableLimit }
    , mLowerTranslation { config.lowerTranslation }
    , mUpperTranslation { config.upperTranslation }
    , mEnableMotor { config.enableMotor }
    , mMotorSpeed { config.motorSpeed }
    , mMaxMotorForce { std::max(0.0, config.maxMotorForce) }
{
}

double PrismaticJoint2D::getJointTranslation() const noexcept
{
    if (mBodyA == nullptr || mBodyB == nullptr) {
        return 0.0;
    }
    const math::Vec2 pA { mBodyA->toWorldPoint(mLocalAnchorA) };
    const math::Vec2 pB { mBodyB->toWorldPoint(mLocalAnchorB) };
    const math::Vec2 axis { mBodyA->toWorldVec(mLocalAxisA).normalized() };
    return (pB - pA).dot(axis);
}

double PrismaticJoint2D::getJointSpeed() const noexcept
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

bool PrismaticJoint2D::isLimitEnabled() const noexcept
{
    return mEnableLimit;
}

void PrismaticJoint2D::enableLimit(bool flag) noexcept
{
    mEnableLimit = flag;
}

double PrismaticJoint2D::getLowerLimit() const noexcept
{
    return mLowerTranslation;
}

double PrismaticJoint2D::getUpperLimit() const noexcept
{
    return mUpperTranslation;
}

void PrismaticJoint2D::setLimits(double lower, double upper) noexcept
{
    mLowerTranslation = lower;
    mUpperTranslation = upper;
}

bool PrismaticJoint2D::isMotorEnabled() const noexcept
{
    return mEnableMotor;
}

void PrismaticJoint2D::enableMotor(bool flag) noexcept
{
    mEnableMotor = flag;
}

double PrismaticJoint2D::getMotorSpeed() const noexcept
{
    return mMotorSpeed;
}

void PrismaticJoint2D::setMotorSpeed(double speed) noexcept
{
    mMotorSpeed = speed;
}

double PrismaticJoint2D::getMaxMotorForce() const noexcept
{
    return mMaxMotorForce;
}

void PrismaticJoint2D::setMaxMotorForce(double force) noexcept
{
    mMaxMotorForce = std::max(0.0, force);
}

double PrismaticJoint2D::getMotorForce(double invDt) const noexcept
{
    return mMotorImpulse * invDt;
}

void PrismaticJoint2D::initVelocityConstraints(double dt) noexcept
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

    // K matrix for perpendicular and angular constraints (2x2)
    const double k11 { mA + mB + (iA * mArmA2 * mArmA2) + (iB * mArmB2 * mArmB2) };
    const double k12 { (iA * mArmA2) + (iB * mArmB2) };
    const double k22 { iA + iB };

    const double detK { (k11 * k22) - (k12 * k12) };
    if (std::abs(detK) > kDetTolerance) {
        const double invDet { 1.0 / detK };
        mEffectiveMass.m00 = k22 * invDet;
        mEffectiveMass.m01 = -k12 * invDet;
        mEffectiveMass.m10 = -k12 * invDet;
        mEffectiveMass.m11 = k11 * invDet;
    } else {
        mEffectiveMass = math::Mat2 {};
    }

    // Axial mass for motor and limits along axis
    const double kAxial { mA + mB + (iA * mArmA1 * mArmA1) + (iB * mArmB1 * mArmB1) };
    mAxialMass = (kAxial > kDetTolerance) ? (1.0 / kAxial) : 0.0;

    if (!mEnableMotor) {
        mMotorImpulse = 0.0;
    }
    if (!mEnableLimit) {
        mLimitImpulse = 0.0;
    }

    // Warm-start: Apply cached impulses
    const math::Vec2 p { (mPerpAxis * mImpulse.x) + (mAxis * (mMotorImpulse + mLimitImpulse)) };
    const double lA { (mArmA + mDelta).cross(p) + mImpulse.y };
    const double lB { mArmB.cross(p) + mImpulse.y };

    mBodyA->setLinearVelocity(mBodyA->getLinearVelocity() - (p * mA));
    mBodyA->setAngularVelocity(mBodyA->getAngularVelocity() - (iA * lA));

    mBodyB->setLinearVelocity(mBodyB->getLinearVelocity() + (p * mB));
    mBodyB->setAngularVelocity(mBodyB->getAngularVelocity() + (iB * lB));
}

void PrismaticJoint2D::solveVelocityConstraints() noexcept
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

    // 1. Solve linear motor constraint along mAxis
    if (mEnableMotor && mAxialMass > 0.0) {
        const double vRel { mAxis.dot(vB - vA) + (mArmB1 * wB) - (mArmA1 * wA) };
        const double cDot { vRel - mMotorSpeed };
        const double impulse { -mAxialMass * cDot };
        const double oldImpulse { mMotorImpulse };
        const double maxImpulse { mMaxMotorForce * mDt };
        mMotorImpulse = std::clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
        const double dImpulse { mMotorImpulse - oldImpulse };

        const math::Vec2 p { mAxis * dImpulse };
        vA -= p * mA;
        wA -= iA * ((mArmA + mDelta).cross(p));
        vB += p * mB;
        wB += iB * (mArmB.cross(p));
    }

    // 2. Solve translation limits along mAxis
    if (mEnableLimit && mAxialMass > 0.0) {
        const double translation { mDelta.dot(mAxis) };
        const double vRel { mAxis.dot(vB - vA) + (mArmB1 * wB) - (mArmA1 * wA) };

        if (std::abs(mUpperTranslation - mLowerTranslation) < 2.0 * kLinearSlop) {
            // Equal limits: Bilateral locking
            const double impulse { -mAxialMass * vRel };
            mLimitImpulse += impulse;
            const math::Vec2 p { mAxis * impulse };
            vA -= p * mA;
            wA -= iA * ((mArmA + mDelta).cross(p));
            vB += p * mB;
            wB += iB * (mArmB.cross(p));
        } else if (translation <= mLowerTranslation) {
            // Lower limit reached: impulse must be positive (push apart)
            const double impulse { -mAxialMass * vRel };
            const double oldImpulse { mLimitImpulse };
            mLimitImpulse = std::max(0.0, oldImpulse + impulse);
            const double dImpulse { mLimitImpulse - oldImpulse };
            const math::Vec2 p { mAxis * dImpulse };
            vA -= p * mA;
            wA -= iA * ((mArmA + mDelta).cross(p));
            vB += p * mB;
            wB += iB * (mArmB.cross(p));
        } else if (translation >= mUpperTranslation) {
            // Upper limit reached: impulse must be negative (pull together)
            const double impulse { -mAxialMass * vRel };
            const double oldImpulse { mLimitImpulse };
            mLimitImpulse = std::min(0.0, oldImpulse + impulse);
            const double dImpulse { mLimitImpulse - oldImpulse };
            const math::Vec2 p { mAxis * dImpulse };
            vA -= p * mA;
            wA -= iA * ((mArmA + mDelta).cross(p));
            vB += p * mB;
            wB += iB * (mArmB.cross(p));
        }
    }

    // 3. Solve 2-DOF perpendicular and angular constraint
    const double cDot1 { mPerpAxis.dot(vB - vA) + (mArmB2 * wB) - (mArmA2 * wA) };
    const double cDot2 { wB - wA };
    const math::Vec2 cDot { cDot1, cDot2 };

    const math::Vec2 dImpulse {
        -((mEffectiveMass.m00 * cDot.x) + (mEffectiveMass.m01 * cDot.y)),
        -((mEffectiveMass.m10 * cDot.x) + (mEffectiveMass.m11 * cDot.y))
    };

    mImpulse += dImpulse;

    const math::Vec2 p { mPerpAxis * dImpulse.x };
    const double lA { (mArmA + mDelta).cross(p) + dImpulse.y };
    const double lB { mArmB.cross(p) + dImpulse.y };

    vA -= p * mA;
    wA -= iA * lA;
    vB += p * mB;
    wB += iB * lB;

    mBodyA->setLinearVelocity(vA);
    mBodyA->setAngularVelocity(wA);
    mBodyB->setLinearVelocity(vB);
    mBodyB->setAngularVelocity(wB);
}

bool PrismaticJoint2D::solvePositionConstraints() noexcept
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

    const double c1 { d.dot(perpAxis) };
    const double c2 {
        math::Angle2D::deltaAngle(mBodyA->getOrientation() + mReferenceAngle, mBodyB->getOrientation())
    };

    const double linearError { std::abs(c1) };
    const double angularError { std::abs(c2) };

    const double sA2 { (rA + d).cross(perpAxis) };
    const double sB2 { rB.cross(perpAxis) };

    const double k11 { mA + mB + (iA * sA2 * sA2) + (iB * sB2 * sB2) };
    const double k12 { (iA * sA2) + (iB * sB2) };
    const double k22 { iA + iB };

    const double detK { (k11 * k22) - (k12 * k12) };
    math::Vec2 impulse { 0.0, 0.0 };
    if (std::abs(detK) > kDetTolerance) {
        const double clampedC1 { std::clamp(c1, -kMaxLinearCorrection, kMaxLinearCorrection) };
        const double clampedC2 { std::clamp(c2, -kMaxAngularCorrection, kMaxAngularCorrection) };
        const double invDet { 1.0 / detK };
        impulse.x = -invDet * ((k22 * clampedC1) - (k12 * clampedC2));
        impulse.y = -invDet * ((-k12 * clampedC1) + (k11 * clampedC2));
    }

    const math::Vec2 p { perpAxis * impulse.x };
    const double lA { (rA + d).cross(p) + impulse.y };
    const double lB { rB.cross(p) + impulse.y };

    mBodyA->setPosition(mBodyA->getPosition() - (p * mA));
    mBodyA->setOrientation(math::Angle2D::normalizeRad(mBodyA->getOrientation() - (iA * lA)));

    mBodyB->setPosition(mBodyB->getPosition() + (p * mB));
    mBodyB->setOrientation(math::Angle2D::normalizeRad(mBodyB->getOrientation() + (iB * lB)));

    // Limit position correction along axis
    double limitError { 0.0 };
    if (mEnableLimit) {
        const math::Vec2 rA_lim { mBodyA->toWorldVec(mLocalAnchorA) };
        const math::Vec2 rB_lim { mBodyB->toWorldVec(mLocalAnchorB) };
        const math::Vec2 d_lim { (mBodyB->getPosition() + rB_lim) - (mBodyA->getPosition() + rA_lim) };
        const double translation { d_lim.dot(axis) };

        double cAxial { 0.0 };
        if (std::abs(mUpperTranslation - mLowerTranslation) < 2.0 * kLinearSlop) {
            cAxial = std::clamp(translation - mLowerTranslation, -kMaxLinearCorrection, kMaxLinearCorrection);
            limitError = std::abs(translation - mLowerTranslation);
        } else if (translation <= mLowerTranslation) {
            cAxial = std::clamp(
                translation - mLowerTranslation + kLinearSlop, -kMaxLinearCorrection, 0.0);
            limitError = mLowerTranslation - translation;
        } else if (translation >= mUpperTranslation) {
            cAxial = std::clamp(
                translation - mUpperTranslation - kLinearSlop, 0.0, kMaxLinearCorrection);
            limitError = translation - mUpperTranslation;
        }

        if (std::abs(cAxial) > 1e-6) {
            const double sA1 { (rA_lim + d_lim).cross(axis) };
            const double sB1 { rB_lim.cross(axis) };
            const double kAxial { mA + mB + (iA * sA1 * sA1) + (iB * sB1 * sB1) };
            if (kAxial > kDetTolerance) {
                const double dImpulse { -cAxial / kAxial };
                const math::Vec2 pAxial { axis * dImpulse };
                mBodyA->setPosition(mBodyA->getPosition() - (pAxial * mA));
                mBodyA->setOrientation(
                    math::Angle2D::normalizeRad(mBodyA->getOrientation() - (iA * (rA_lim + d_lim).cross(pAxial))));

                mBodyB->setPosition(mBodyB->getPosition() + (pAxial * mB));
                mBodyB->setOrientation(
                    math::Angle2D::normalizeRad(mBodyB->getOrientation() + (iB * rB_lim.cross(pAxial))));
            }
        }
    }

    return (linearError <= kLinearSlop) && (angularError <= kAngularSlop) && (limitError <= kLinearSlop);
}

math::Vec2 PrismaticJoint2D::getReactionForce(double invDt) const noexcept
{
    return ((mPerpAxis * mImpulse.x) + (mAxis * (mMotorImpulse + mLimitImpulse))) * invDt;
}

double PrismaticJoint2D::getReactionTorque(double invDt) const noexcept
{
    return mImpulse.y * invDt;
}

} // namespace vehicle_sim::physics
