#include "WeldJoint2D.h"

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

WeldJoint2D::WeldJoint2D(const WeldJointConfig& config) noexcept
    : Joint2D(JointType::Weld, config)
    , mReferenceAngle { config.referenceAngle }
    , mFrequencyHz { std::max(0.0, config.frequencyHz) }
    , mDampingRatio { std::max(0.0, config.dampingRatio) }
    , mBreakForce { std::max(0.0, config.breakForce) }
    , mBreakTorque { std::max(0.0, config.breakTorque) }
{
}

double WeldJoint2D::getReferenceAngle() const noexcept
{
    return mReferenceAngle;
}

double WeldJoint2D::getFrequencyHz() const noexcept
{
    return mFrequencyHz;
}

void WeldJoint2D::setFrequencyHz(double hz) noexcept
{
    mFrequencyHz = std::max(0.0, hz);
}

double WeldJoint2D::getDampingRatio() const noexcept
{
    return mDampingRatio;
}

void WeldJoint2D::setDampingRatio(double ratio) noexcept
{
    mDampingRatio = std::max(0.0, ratio);
}

bool WeldJoint2D::isBroken() const noexcept
{
    return mBroken;
}

double WeldJoint2D::getBreakForce() const noexcept
{
    return mBreakForce;
}

void WeldJoint2D::setBreakForce(double force) noexcept
{
    mBreakForce = std::max(0.0, force);
}

double WeldJoint2D::getBreakTorque() const noexcept
{
    return mBreakTorque;
}

void WeldJoint2D::setBreakTorque(double torque) noexcept
{
    mBreakTorque = std::max(0.0, torque);
}

void WeldJoint2D::setOnBreakCallback(std::function<void(WeldJoint2D&)> callback) noexcept
{
    mOnBreakCallback = std::move(callback);
}

void WeldJoint2D::initVelocityConstraints(double dt) noexcept
{
    if (mBroken || mBodyA == nullptr || mBodyB == nullptr || dt <= 0.0) {
        return;
    }

    mDt = dt;
    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    mArmA = mBodyA->toWorldVec(mLocalAnchorA);
    mArmB = mBodyB->toWorldVec(mLocalAnchorB);

    // K11 = (mA + mB) * I2 + iA * [rA_skew]^2 + iB * [rB_skew]^2
    mMassK11.m00 = mA + mB + (iA * mArmA.y * mArmA.y) + (iB * mArmB.y * mArmB.y);
    mMassK11.m01 = (-iA * mArmA.x * mArmA.y) - (iB * mArmB.x * mArmB.y);
    mMassK11.m10 = mMassK11.m01;
    mMassK11.m11 = mA + mB + (iA * mArmA.x * mArmA.x) + (iB * mArmB.x * mArmB.x);

    // K12 = [ -iA*rAy - iB*rBy, iA*rAx + iB*rBx ]^T
    mMassK12.x = (-iA * mArmA.y) - (iB * mArmB.y);
    mMassK12.y = (iA * mArmA.x) + (iB * mArmB.x);

    // K22 = iA + iB
    mMassK22 = iA + iB;

    mBias = 0.0;
    mGamma = 0.0;

    if (mFrequencyHz > 0.0 && mMassK22 > 0.0) {
        const double omega { 2.0 * kPi * mFrequencyHz };
        const double d { 2.0 * (1.0 / mMassK22) * mDampingRatio * omega };
        const double k { (1.0 / mMassK22) * omega * omega };

        mGamma = 1.0 / (dt * (d + (dt * k)));
        const double beta { (dt * k) / (d + (dt * k)) };
        mMassK22 += mGamma;

        const double angleDiff {
            math::Angle2D::deltaAngle(mBodyA->getOrientation() + mReferenceAngle, mBodyB->getOrientation())
        };
        mBias = (beta / dt) * angleDiff;
    }

    // Warm start: Apply previous linear and angular impulses
    const math::Vec2 p { mLinearImpulse };
    const double l { mAngularImpulse };

    mBodyA->setLinearVelocity(mBodyA->getLinearVelocity() - (p * mA));
    mBodyA->setAngularVelocity(mBodyA->getAngularVelocity() - (iA * (mArmA.cross(p) + l)));

    mBodyB->setLinearVelocity(mBodyB->getLinearVelocity() + (p * mB));
    mBodyB->setAngularVelocity(mBodyB->getAngularVelocity() + (iB * (mArmB.cross(p) + l)));
}

void WeldJoint2D::solveVelocityConstraints() noexcept
{
    if (mBroken || mBodyA == nullptr || mBodyB == nullptr) {
        return;
    }

    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    const math::Vec2 vA { mBodyA->getLinearVelocity() };
    const double wA { mBodyA->getAngularVelocity() };
    const math::Vec2 vB { mBodyB->getLinearVelocity() };
    const double wB { mBodyB->getAngularVelocity() };

    // Point velocity error C_dot1 = vB + wB x rB - vA - wA x rA
    const math::Vec2 vPointA { vA + math::Vec2 { -wA * mArmA.y, wA * mArmA.x } };
    const math::Vec2 vPointB { vB + math::Vec2 { -wB * mArmB.y, wB * mArmB.x } };
    const math::Vec2 cDot1 { vPointB - vPointA };

    // Angular velocity error C_dot2 = wB - wA + bias + gamma * angularImpulse
    const double cDot2 { (wB - wA) + mBias + (mGamma * mAngularImpulse) };

    math::Vec2 dP { 0.0, 0.0 };
    double dL { 0.0 };

    if (mMassK22 > kDetTolerance) {
        // Schur complement: S = K11 - (1 / K22) * K12 * K12^T
        const double invK22 { 1.0 / mMassK22 };
        math::Mat2 S {};
        S.m00 = mMassK11.m00 - (invK22 * mMassK12.x * mMassK12.x);
        S.m01 = mMassK11.m01 - (invK22 * mMassK12.x * mMassK12.y);
        S.m10 = S.m01;
        S.m11 = mMassK11.m11 - (invK22 * mMassK12.y * mMassK12.y);

        // Right-hand side for linear impulse: b = -cDot1 + (cDot2 / K22) * K12
        const math::Vec2 b { -cDot1 + (mMassK12 * (cDot2 * invK22)) };
        const double detS { (S.m00 * S.m11) - (S.m01 * S.m10) };

        if (std::abs(detS) > kDetTolerance) {
            const double invDet { 1.0 / detS };
            dP.x = invDet * ((S.m11 * b.x) - (S.m01 * b.y));
            dP.y = invDet * ((-S.m10 * b.x) + (S.m00 * b.y));
        }

        // dL = (-cDot2 - K12 . dP) / K22
        dL = invK22 * (-cDot2 - mMassK12.dot(dP));
    } else {
        // Only point coincidence
        const double detK11 { (mMassK11.m00 * mMassK11.m11) - (mMassK11.m01 * mMassK11.m10) };
        if (std::abs(detK11) > kDetTolerance) {
            const double invDet { 1.0 / detK11 };
            dP.x = invDet * ((mMassK11.m11 * -cDot1.x) - (mMassK11.m01 * -cDot1.y));
            dP.y = invDet * ((-mMassK11.m10 * -cDot1.x) + (mMassK11.m00 * -cDot1.y));
        }
    }

    mLinearImpulse += dP;
    mAngularImpulse += dL;

    mBodyA->setLinearVelocity(mBodyA->getLinearVelocity() - (dP * mA));
    mBodyA->setAngularVelocity(mBodyA->getAngularVelocity() - (iA * (mArmA.cross(dP) + dL)));

    mBodyB->setLinearVelocity(mBodyB->getLinearVelocity() + (dP * mB));
    mBodyB->setAngularVelocity(mBodyB->getAngularVelocity() + (iB * (mArmB.cross(dP) + dL)));

    // Rupture check
    if (mDt > 0.0 && (mBreakForce > 0.0 || mBreakTorque > 0.0)) {
        const double currentForce { mLinearImpulse.length() / mDt };
        const double currentTorque { std::abs(mAngularImpulse) / mDt };

        bool shouldBreak { false };
        if (mBreakForce > 0.0 && currentForce >= mBreakForce) {
            shouldBreak = true;
        }
        if (mBreakTorque > 0.0 && currentTorque >= mBreakTorque) {
            shouldBreak = true;
        }

        if (shouldBreak) {
            mBroken = true;
            mLinearImpulse = math::Vec2 { 0.0, 0.0 };
            mAngularImpulse = 0.0;
            if (mOnBreakCallback) {
                mOnBreakCallback(*this);
            }
        }
    }
}

bool WeldJoint2D::solvePositionConstraints() noexcept
{
    if (mBroken || mBodyA == nullptr || mBodyB == nullptr) {
        return true;
    }

    const double mA { mBodyA->getInverseMass() };
    const double mB { mBodyB->getInverseMass() };
    const double iA { mBodyA->getInverseInertia() };
    const double iB { mBodyB->getInverseInertia() };

    const math::Vec2 rA { mBodyA->toWorldVec(mLocalAnchorA) };
    const math::Vec2 rB { mBodyB->toWorldVec(mLocalAnchorB) };

    const math::Vec2 pA { mBodyA->getPosition() + rA };
    const math::Vec2 pB { mBodyB->getPosition() + rB };
    const math::Vec2 c1 { pB - pA };

    double c2 { 0.0 };
    if (mFrequencyHz <= 0.0) {
        c2 = math::Angle2D::deltaAngle(mBodyA->getOrientation() + mReferenceAngle, mBodyB->getOrientation());
    }

    const double positionError { c1.length() };
    const double angularError { std::abs(c2) };

    // Formulate position K matrix
    math::Mat2 K11 {};
    K11.m00 = mA + mB + (iA * rA.y * rA.y) + (iB * rB.y * rB.y);
    K11.m01 = (-iA * rA.x * rA.y) - (iB * rB.x * rB.y);
    K11.m10 = K11.m01;
    K11.m11 = mA + mB + (iA * rA.x * rA.x) + (iB * rB.x * rB.x);

    const math::Vec2 K12 { (-iA * rA.y) - (iB * rB.y), (iA * rA.x) + (iB * rB.x) };
    const double K22 { iA + iB };

    math::Vec2 dP { 0.0, 0.0 };
    double dL { 0.0 };

    if (mFrequencyHz <= 0.0 && K22 > kDetTolerance) {
        const double invK22 { 1.0 / K22 };
        math::Mat2 S {};
        S.m00 = K11.m00 - (invK22 * K12.x * K12.x);
        S.m01 = K11.m01 - (invK22 * K12.x * K12.y);
        S.m10 = S.m01;
        S.m11 = K11.m11 - (invK22 * K12.y * K12.y);

        const math::Vec2 clampedC1 { (c1.length() > kMaxLinearCorrection)
                ? (c1.normalized() * kMaxLinearCorrection)
                : c1 };
        const double clampedC2 { std::clamp(c2, -kMaxAngularCorrection, kMaxAngularCorrection) };

        const math::Vec2 b { -clampedC1 + (K12 * (clampedC2 * invK22)) };
        const double detS { (S.m00 * S.m11) - (S.m01 * S.m10) };

        if (std::abs(detS) > kDetTolerance) {
            const double invDet { 1.0 / detS };
            dP.x = invDet * ((S.m11 * b.x) - (S.m01 * b.y));
            dP.y = invDet * ((-S.m10 * b.x) + (S.m00 * b.y));
        }
        dL = invK22 * (-clampedC2 - K12.dot(dP));
    } else {
        const math::Vec2 clampedC1 { (c1.length() > kMaxLinearCorrection)
                ? (c1.normalized() * kMaxLinearCorrection)
                : c1 };
        const double detK11 { (K11.m00 * K11.m11) - (K11.m01 * K11.m10) };
        if (std::abs(detK11) > kDetTolerance) {
            const double invDet { 1.0 / detK11 };
            dP.x = invDet * ((K11.m11 * -clampedC1.x) - (K11.m01 * -clampedC1.y));
            dP.y = invDet * ((-K11.m10 * -clampedC1.x) + (K11.m00 * -clampedC1.y));
        }
    }

    mBodyA->setPosition(mBodyA->getPosition() - (dP * mA));
    mBodyA->setOrientation(math::Angle2D::normalizeRad(mBodyA->getOrientation() - (iA * (rA.cross(dP) + dL))));

    mBodyB->setPosition(mBodyB->getPosition() + (dP * mB));
    mBodyB->setOrientation(math::Angle2D::normalizeRad(mBodyB->getOrientation() + (iB * (rB.cross(dP) + dL))));

    return (positionError <= kLinearSlop) && (angularError <= kAngularSlop);
}

math::Vec2 WeldJoint2D::getReactionForce(double invDt) const noexcept
{
    return mLinearImpulse * invDt;
}

double WeldJoint2D::getReactionTorque(double invDt) const noexcept
{
    return mAngularImpulse * invDt;
}

JointSnapshotData WeldJoint2D::captureState() const noexcept
{
    JointSnapshotData data {};
    data.linearImpulse = mLinearImpulse;
    data.angularImpulse = mAngularImpulse;
    data.motorImpulse = 0.0;
    data.limitImpulse = 0.0;
    data.isBroken = mBroken;
    return data;
}

void WeldJoint2D::restoreState(const JointSnapshotData& state) noexcept
{
    mLinearImpulse = state.linearImpulse;
    mAngularImpulse = state.angularImpulse;
    mBroken = state.isBroken;
}

} // namespace vehicle_sim::physics
