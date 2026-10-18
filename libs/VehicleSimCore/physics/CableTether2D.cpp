#include "CableTether2D.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "RigidBody2D.h"

namespace vehicle_sim::physics {

namespace {
    inline constexpr double kPi { 3.14159265358979323846 };
    inline constexpr double kEpsilon { 1e-9 };
    inline constexpr double kLengthEpsilon { 1e-6 };
    inline constexpr double kSlop { 0.005 };
    inline constexpr double kMaxPositionCorrection { 0.2 };
    inline constexpr double kFreeWheelFriction { 50.0 };
    inline constexpr double kDrumDamping { 200.0 };
    inline constexpr double kMaxFreeWheelSpeed { 10.0 };
} // namespace

CableTether2D::CableTether2D(const CableTetherConfig& config) noexcept
    : Joint2D(JointType::Cable, config)
    , mRestLength { config.restLength }
    , mMinRestLength { (config.minRestLength > 0.0) ? config.minRestLength : 0.05 }
    , mMaxRestLength { (config.maxRestLength > mMinRestLength) ? config.maxRestLength : mMinRestLength }
    , mFrequencyHz { (config.frequencyHz >= 0.0) ? config.frequencyHz : 0.0 }
    , mDampingRatio { (config.dampingRatio >= 0.0) ? config.dampingRatio : 0.0 }
    , mBreakTension { (config.breakTension >= 0.0) ? config.breakTension : 0.0 }
    , mMaxPullForce { (config.maxPullForce >= 0.0) ? config.maxPullForce : 0.0 }
    , mSpoolSpeed { config.spoolSpeed }
    , mWinchMode { config.winchMode }
{
    if (mRestLength <= 0.0) {
        const double d { getCurrentLength() };
        mRestLength = (d > mMinRestLength) ? d : mMinRestLength;
    }
    mRestLength = std::clamp(mRestLength, mMinRestLength, mMaxRestLength);
}

double CableTether2D::getRestLength() const noexcept
{
    return mRestLength;
}

void CableTether2D::setRestLength(double length) noexcept
{
    mRestLength = std::clamp(length, mMinRestLength, mMaxRestLength);
}

double CableTether2D::getCurrentLength() const noexcept
{
    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    return (pB - pA).length();
}

double CableTether2D::getMinRestLength() const noexcept
{
    return mMinRestLength;
}

void CableTether2D::setMinRestLength(double length) noexcept
{
    mMinRestLength = (length > 0.0) ? length : 0.05;
    if (mMaxRestLength < mMinRestLength) {
        mMaxRestLength = mMinRestLength;
    }
    mRestLength = std::clamp(mRestLength, mMinRestLength, mMaxRestLength);
}

double CableTether2D::getMaxRestLength() const noexcept
{
    return mMaxRestLength;
}

void CableTether2D::setMaxRestLength(double length) noexcept
{
    mMaxRestLength = (length >= mMinRestLength) ? length : mMinRestLength;
    mRestLength = std::clamp(mRestLength, mMinRestLength, mMaxRestLength);
}

bool CableTether2D::isSlack() const noexcept
{
    return !isTaut();
}

bool CableTether2D::isTaut() const noexcept
{
    return (getCurrentLength() >= (mRestLength - kSlop)) || (mCurrentTension > kEpsilon);
}

bool CableTether2D::isBroken() const noexcept
{
    return mBroken;
}

double CableTether2D::getBreakTension() const noexcept
{
    return mBreakTension;
}

void CableTether2D::setBreakTension(double tension) noexcept
{
    mBreakTension = (tension >= 0.0) ? tension : 0.0;
}

double CableTether2D::getTension() const noexcept
{
    return mCurrentTension;
}

double CableTether2D::getStrain() const noexcept
{
    const double d { getCurrentLength() };
    if (d <= mRestLength || mRestLength < kLengthEpsilon) {
        return 0.0;
    }
    return (d - mRestLength) / mRestLength;
}

double CableTether2D::getStrainEnergy() const noexcept
{
    if (isSlack() || mBroken) {
        return 0.0;
    }
    const double deltaL { getCurrentLength() - mRestLength };
    if (mFrequencyHz > 0.0 && mEffectiveMass > 0.0) {
        const double omega { 2.0 * kPi * mFrequencyHz };
        const double k { mEffectiveMass * omega * omega };
        return 0.5 * k * deltaL * deltaL;
    }
    return 0.5 * mCurrentTension * deltaL;
}

WinchMode CableTether2D::getWinchMode() const noexcept
{
    return mWinchMode;
}

void CableTether2D::setWinchMode(WinchMode mode) noexcept
{
    mWinchMode = mode;
}

double CableTether2D::getSpoolSpeed() const noexcept
{
    return mSpoolSpeed;
}

void CableTether2D::setSpoolSpeed(double speed) noexcept
{
    mSpoolSpeed = speed;
}

double CableTether2D::getMaxPullForce() const noexcept
{
    return mMaxPullForce;
}

void CableTether2D::setMaxPullForce(double force) noexcept
{
    mMaxPullForce = (force >= 0.0) ? force : 0.0;
}

double CableTether2D::getFrequencyHz() const noexcept
{
    return mFrequencyHz;
}

void CableTether2D::setFrequencyHz(double hz) noexcept
{
    mFrequencyHz = (hz >= 0.0) ? hz : 0.0;
}

double CableTether2D::getDampingRatio() const noexcept
{
    return mDampingRatio;
}

void CableTether2D::setDampingRatio(double ratio) noexcept
{
    mDampingRatio = (ratio >= 0.0) ? ratio : 0.0;
}

void CableTether2D::setOnSnapCallback(std::function<void(CableTether2D&, double)> callback) noexcept
{
    mOnSnapCallback = std::move(callback);
}

void CableTether2D::updateWinchSpool(double dt) noexcept
{
    if (dt <= 0.0 || mBroken) {
        return;
    }

    double spoolRate { 0.0 };
    switch (mWinchMode) {
    case WinchMode::Locked:
        spoolRate = 0.0;
        break;
    case WinchMode::HaulIn: {
        const double commandedRate { -std::abs(mSpoolSpeed) };
        double loadFactor { 1.0 };
        if (mMaxPullForce > 0.0 && mCurrentTension > 0.0) {
            loadFactor = std::clamp(1.0 - (mCurrentTension / mMaxPullForce), 0.0, 1.0);
        }
        spoolRate = commandedRate * loadFactor;
        break;
    }
    case WinchMode::Payout:
        spoolRate = std::abs(mSpoolSpeed);
        break;
    case WinchMode::FreeWheel: {
        if (mCurrentTension > kFreeWheelFriction) {
            spoolRate = (mCurrentTension - kFreeWheelFriction) / kDrumDamping;
            spoolRate = std::clamp(spoolRate, 0.0, kMaxFreeWheelSpeed);
        } else {
            spoolRate = 0.0;
        }
        break;
    }
    }

    mRestLength = std::clamp(mRestLength + (spoolRate * dt), mMinRestLength, mMaxRestLength);
}

void CableTether2D::sampleCatenary(std::size_t numPoints,
                                  const math::Vec2& gravity,
                                  std::vector<math::Vec2>& outPoints) const
{
    if (numPoints < 2) {
        return;
    }

    outPoints.clear();
    outPoints.reserve(numPoints);

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 delta { pB - pA };
    const double d { delta.length() };

    math::Vec2 sagUnit { 0.0, -1.0 };
    const double gLen { gravity.length() };
    if (gLen > kEpsilon) {
        sagUnit = gravity * (1.0 / gLen);
    }

    double hSag { 0.0 };
    if (d < mRestLength && mRestLength > kLengthEpsilon) {
        const double diff { mRestLength - d };
        hSag = 0.5 * std::sqrt(1.5 * mRestLength * diff);
    }

    const double invSeg { 1.0 / static_cast<double>(numPoints - 1) };
    for (std::size_t i { 0 }; i < numPoints; ++i) {
        const double s { static_cast<double>(i) * invSeg };
        const math::Vec2 straightPos { (pA * (1.0 - s)) + (pB * s) };
        const double sagCurve { 4.0 * hSag * s * (1.0 - s) };
        outPoints.push_back(straightPos + (sagUnit * sagCurve));
    }
}

void CableTether2D::initVelocityConstraints(double dt) noexcept
{
    if (dt <= 0.0 || mBroken) {
        return;
    }

    mDt = dt;
    updateWinchSpool(dt);

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 delta { pB - pA };
    const double length { delta.length() };

    if (length > kLengthEpsilon) {
        mUnitSeparation = delta * (1.0 / length);
    } else {
        mUnitSeparation = math::Vec2 { 1.0, 0.0 };
    }

    mArmA = (mBodyA != nullptr) ? (pA - mBodyA->getPosition()) : math::Vec2 { 0.0, 0.0 };
    mArmB = (mBodyB != nullptr) ? (pB - mBodyB->getPosition()) : math::Vec2 { 0.0, 0.0 };

    const double crA { (mBodyA != nullptr) ? math::cross(mArmA, mUnitSeparation) : 0.0 };
    const double crB { (mBodyB != nullptr) ? math::cross(mArmB, mUnitSeparation) : 0.0 };

    const double invMassA { (mBodyA != nullptr) ? mBodyA->getInverseMass() : 0.0 };
    const double invInertiaA { (mBodyA != nullptr) ? mBodyA->getInverseInertia() : 0.0 };
    const double invMassB { (mBodyB != nullptr) ? mBodyB->getInverseMass() : 0.0 };
    const double invInertiaB { (mBodyB != nullptr) ? mBodyB->getInverseInertia() : 0.0 };

    const double k { invMassA + invMassB + (invInertiaA * crA * crA) + (invInertiaB * crB * crB) };

    const double error { length - mRestLength };

    if (length > (mRestLength - kSlop)) {
        if (mFrequencyHz > 0.0) {
            const double omega { 2.0 * kPi * mFrequencyHz };
            const double mass { (k > kEpsilon) ? (1.0 / k) : 0.0 };
            const double dAmp { 2.0 * mass * mDampingRatio * omega };
            const double springK { mass * omega * omega };

            mGamma = 1.0 / (dt * (dAmp + (dt * springK)));
            const double beta { (dt * springK) / (dAmp + (dt * springK)) };
            mBias = (beta / dt) * std::max(0.0, error);

            const double kSoft { k + mGamma };
            mEffectiveMass = (kSoft > kEpsilon) ? (1.0 / kSoft) : 0.0;
        } else {
            mGamma = 0.0;
            mBias = 0.0;
            mEffectiveMass = (k > kEpsilon) ? (1.0 / k) : 0.0;
        }

        // Warm starting: apply cached positive tension impulse
        // Pull Body A toward B (+u) and Body B toward A (-u)
        const math::Vec2 impulse { mUnitSeparation * mImpulse };
        if (mBodyA != nullptr && !mBodyA->isStatic()) {
            mBodyA->applyImpulseAtPoint(impulse, pA);
        }
        if (mBodyB != nullptr && !mBodyB->isStatic()) {
            mBodyB->applyImpulseAtPoint(-impulse, pB);
        }
    } else {
        // Cable is genuinely slack: zero cached impulse and disable bias
        mEffectiveMass = (k > kEpsilon) ? (1.0 / k) : 0.0;
        mImpulse = 0.0;
        mBias = 0.0;
        mGamma = 0.0;
        mCurrentTension = 0.0;
    }
}

void CableTether2D::solveVelocityConstraints() noexcept
{
    if (mBroken || mEffectiveMass <= 0.0) {
        return;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 vA { (mBodyA != nullptr) ? mBodyA->getVelocityAtPoint(pA) : math::Vec2 { 0.0, 0.0 } };
    const math::Vec2 vB { (mBodyB != nullptr) ? mBodyB->getVelocityAtPoint(pB) : math::Vec2 { 0.0, 0.0 } };

    // Relative separation velocity along unit separation u (positive when separating)
    const double crA { (mBodyA != nullptr) ? math::cross(mArmA, mUnitSeparation) : 0.0 };
    const double crB { (mBodyB != nullptr) ? math::cross(mArmB, mUnitSeparation) : 0.0 };
    const double omegaA { (mBodyA != nullptr) ? mBodyA->getAngularVelocity() : 0.0 };
    const double omegaB { (mBodyB != nullptr) ? mBodyB->getAngularVelocity() : 0.0 };

    const double cdot { mUnitSeparation.dot(vB - vA) + (crB * omegaB) - (crA * omegaA) };

    // When cdot > 0 (separating) or error > 0 (mBias > 0), tension impulse increases
    const double deltaImpulse { mEffectiveMass * (cdot + mBias - (mGamma * mImpulse)) };

    // Unilateral clamping: cable can only pull (mImpulse >= 0.0)
    const double oldImpulse { mImpulse };
    mImpulse = std::max(0.0, mImpulse + deltaImpulse);
    const double clampedDelta { mImpulse - oldImpulse };

    // Pull Body A toward B (+u) and Body B toward A (-u)
    const math::Vec2 impulse { mUnitSeparation * clampedDelta };
    if (mBodyA != nullptr && !mBodyA->isStatic()) {
        mBodyA->applyImpulseAtPoint(impulse, pA);
    }
    if (mBodyB != nullptr && !mBodyB->isStatic()) {
        mBodyB->applyImpulseAtPoint(-impulse, pB);
    }

    mCurrentTension = (mDt > kEpsilon) ? (mImpulse / mDt) : 0.0;

    // Check rupture threshold
    if (mBreakTension > 0.0 && mDt > kEpsilon) {
        const double estTension { mImpulse / mDt };
        if (estTension >= mBreakTension) {
            mBroken = true;
            mImpulse = 0.0;
            mCurrentTension = 0.0;
            if (mOnSnapCallback) {
                mOnSnapCallback(*this, estTension);
            }
        }
    }
}

bool CableTether2D::solvePositionConstraints() noexcept
{
    if (mBroken || mFrequencyHz > 0.0) {
        return true;
    }

    const math::Vec2 pA { getWorldAnchorA() };
    const math::Vec2 pB { getWorldAnchorB() };
    const math::Vec2 delta { pB - pA };
    const double length { delta.length() };

    // If slack, position constraint is satisfied
    if (length <= mRestLength) {
        return true;
    }

    math::Vec2 u { 1.0, 0.0 };
    if (length > kLengthEpsilon) {
        u = delta * (1.0 / length);
    }

    const double error { length - mRestLength };
    if (error <= kSlop) {
        return true;
    }

    const double correction { std::clamp(error - kSlop, 0.0, kMaxPositionCorrection) };

    const double invMassA { (mBodyA != nullptr) ? mBodyA->getInverseMass() : 0.0 };
    const double invInertiaA { (mBodyA != nullptr) ? mBodyA->getInverseInertia() : 0.0 };
    const double invMassB { (mBodyB != nullptr) ? mBodyB->getInverseMass() : 0.0 };
    const double invInertiaB { (mBodyB != nullptr) ? mBodyB->getInverseInertia() : 0.0 };

    const math::Vec2 rA { (mBodyA != nullptr) ? (pA - mBodyA->getPosition()) : math::Vec2 { 0.0, 0.0 } };
    const math::Vec2 rB { (mBodyB != nullptr) ? (pB - mBodyB->getPosition()) : math::Vec2 { 0.0, 0.0 } };

    const double crA { math::cross(rA, u) };
    const double crB { math::cross(rB, u) };

    const double k { invMassA + invMassB + (invInertiaA * crA * crA) + (invInertiaB * crB * crB) };
    if (k <= kEpsilon) {
        return true;
    }

    const double p { correction / k };
    const math::Vec2 impulse { u * p };

    // Pull Body A toward B (+impulse) and Body B toward A (-impulse)
    if (mBodyA != nullptr && !mBodyA->isStatic()) {
        mBodyA->setPosition(mBodyA->getPosition() + (impulse * invMassA));
        mBodyA->setOrientation(mBodyA->getOrientation() + (invInertiaA * math::cross(rA, impulse)));
    }

    if (mBodyB != nullptr && !mBodyB->isStatic()) {
        mBodyB->setPosition(mBodyB->getPosition() - (impulse * invMassB));
        mBodyB->setOrientation(mBodyB->getOrientation() - (invInertiaB * math::cross(rB, impulse)));
    }

    return error <= (kSlop * 2.0);
}

math::Vec2 CableTether2D::getReactionForce(double invDt) const noexcept
{
    // Force on Body B is toward Body A (-u)
    return -mUnitSeparation * (mImpulse * invDt);
}

double CableTether2D::getReactionTorque(double invDt) const noexcept
{
    (void)invDt;
    return 0.0;
}

JointSnapshotData CableTether2D::captureState() const noexcept
{
    JointSnapshotData data {};
    data.linearImpulse = mUnitSeparation * mImpulse;
    data.motorImpulse = mRestLength;
    data.limitImpulse = mCurrentTension;
    data.isBroken = mBroken;
    return data;
}

void CableTether2D::restoreState(const JointSnapshotData& state) noexcept
{
    mImpulse = state.linearImpulse.length();
    if (state.motorImpulse > 0.0) {
        mRestLength = state.motorImpulse;
    }
    mCurrentTension = state.limitImpulse;
    mBroken = state.isBroken;
}

} // namespace vehicle_sim::physics
