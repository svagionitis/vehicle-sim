#include "apps/MotionLab/SkidpadCoordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_lab {

namespace {
    constexpr double kPi { 3.14159265358979323846 };
    constexpr double kTwoPi { 2.0 * kPi };
    constexpr double kGravityAcc { 9.80665 };
    constexpr double kRadToDeg { 180.0 / kPi };
} // namespace

SkidpadCoordinator::SkidpadCoordinator() noexcept
{
    rebuildCourseGeometry();
}

void SkidpadCoordinator::setRadiusPreset(SkidpadRadiusPreset preset) noexcept
{
    mPreset = preset;
    const double targetR { getTargetRadius() };
    const double swept { std::pow(0.75 * mVehicleLength, 2.0) / (2.0 * targetR) };
    mCorridorWidth = std::max(3.50, mVehicleWidth + swept + 1.20);
    rebuildCourseGeometry();
}

SkidpadRadiusPreset SkidpadCoordinator::getRadiusPreset() const noexcept
{
    return mPreset;
}

double SkidpadCoordinator::getTargetRadius() const noexcept
{
    return (mPreset == SkidpadRadiusPreset::Radius15m) ? kRadius15m : kRadius30m;
}

void SkidpadCoordinator::stepPhysics(
    vehicle_sim::physics::RigidBody2D& body, double steerAngle, double wheelbase, double dt) noexcept
{
    if (!mEnabled || dt <= 0.0) {
        return;
    }

    const vehicle_sim::math::Vec2 currPos { body.getPosition() };
    const vehicle_sim::math::Vec2 vel { body.getLinearVelocity() };
    const double speed { vel.length() };
    const double yawRate { body.getAngularVelocity() };
    const double targetR { getTargetRadius() };

    const double distFromCenter { currPos.length() };
    const double radiusErr { distFromCenter - targetR };

    // Update operational state
    if (mState == SkidpadState::Staging) {
        if (speed > 0.5 && currPos.x > (-mStagingLength * 0.5)) {
            mState = SkidpadState::Entering;
        }
    } else if (mState == SkidpadState::Entering) {
        if (std::abs(radiusErr) <= (mCorridorWidth * 0.5 + 0.5)) {
            mState = SkidpadState::TrackingOrbit;
        }
    } else if (mState == SkidpadState::TrackingOrbit) {
        if (std::abs(radiusErr) > (mCorridorWidth * 0.5 + 1.5)) {
            mState = SkidpadState::OffCourse;
        }
    } else if (mState == SkidpadState::OffCourse) {
        if (std::abs(radiusErr) <= (mCorridorWidth * 0.5)) {
            mState = SkidpadState::TrackingOrbit;
        }
    }

    // Azimuthal tracking & direction determination
    const double currTheta { std::atan2(currPos.y, currPos.x) };
    if (distFromCenter > 1.0) {
        double dTheta { currTheta - mLastTheta };
        while (dTheta > kPi) {
            dTheta -= kTwoPi;
        }
        while (dTheta < -kPi) {
            dTheta += kTwoPi;
        }

        if (std::abs(dTheta) > 1e-4) {
            if (dTheta > 0.0) {
                mMetrics.direction = SkidpadDirection::CounterClockwise;
            } else {
                mMetrics.direction = SkidpadDirection::Clockwise;
            }
            mAccumulatedAngle += dTheta;
        }

        evaluateOrbitGate(mLastTheta, currTheta, dt);
    }

    // Kinematic & Dynamic Lateral Acceleration
    const double ayKin { (speed * speed) / targetR };
    const double latG { ayKin / kGravityAcc };

    mMetrics.currentRadius = targetR;
    mMetrics.instantaneousSpeed = speed * 3.6; // km/h
    mMetrics.instantaneousLatAcc = latG;
    if (latG > mMetrics.peakLatAcc) {
        mMetrics.peakLatAcc = latG;
    }
    mMetrics.radiusError = radiusErr;
    mMetrics.steerAngleDeg = steerAngle * kRadToDeg;
    mMetrics.currentLapTime += dt;

    updateSteadyState(steerAngle, wheelbase, dt);
    checkPylonCollisions(body);

    mLastPos = currPos;
    mLastTheta = currTheta;
    mLastSpeed = speed;
    mLastYawRate = yawRate;
}

void SkidpadCoordinator::evaluateOrbitGate(double prevTheta, double currTheta, double dt) noexcept
{
    (void)dt;
    if (std::abs(mAccumulatedAngle) >= kPi) {
        mGateArmed = true;
    }

    // Check crossing of theta = 0 (+x axis)
    const bool crossedCcw { (prevTheta < 0.0 && currTheta >= 0.0 && prevTheta > -1.5) };
    const bool crossedCw { (prevTheta > 0.0 && currTheta <= 0.0 && prevTheta < 1.5) };

    if (mGateArmed && (crossedCcw || crossedCw)) {
        const double lapTime { mMetrics.currentLapTime };
        if (lapTime >= 2.0) {
            const double targetR { getTargetRadius() };
            // Official SAE J266 formula: 4 * pi^2 * R / (g * T^2)
            const double lapG { (4.0 * kPi * kPi * targetR) / (kGravityAcc * lapTime * lapTime) };
            mMetrics.lapAverageLatAcc = lapG;
            mMetrics.completedLaps++;
            mMetrics.hasValidLap = true;

            if (mMetrics.direction == SkidpadDirection::CounterClockwise) {
                if (mMetrics.bestCcwLapTime <= 0.0 || lapTime < mMetrics.bestCcwLapTime) {
                    mMetrics.bestCcwLapTime = lapTime;
                }
            } else if (mMetrics.direction == SkidpadDirection::Clockwise) {
                if (mMetrics.bestCwLapTime <= 0.0 || lapTime < mMetrics.bestCwLapTime) {
                    mMetrics.bestCwLapTime = lapTime;
                }
            }
        }

        mMetrics.currentLapTime = 0.0;
        mAccumulatedAngle = 0.0;
        mGateArmed = false;
    }
}

void SkidpadCoordinator::updateSteadyState(double steerAngle, double wheelbase, double dt) noexcept
{
    const double speedAccel { std::abs(mMetrics.instantaneousSpeed / 3.6 - mLastSpeed) / dt };
    const double yawAccel {
        std::abs(mMetrics.instantaneousLatAcc - (mLastSpeed * mLastSpeed / (getTargetRadius() * kGravityAcc))) / dt
    };
    const double radiusErrAbs { std::abs(mMetrics.radiusError) };

    const bool steadyConditions { (radiusErrAbs <= 1.0) && (speedAccel <= 0.5) && (yawAccel <= 0.2)
        && (mState == SkidpadState::TrackingOrbit) && (mMetrics.instantaneousSpeed > 10.0) };

    if (steadyConditions) {
        mSteadyStateTimer += dt;
        if (mSteadyStateTimer >= 0.35) {
            mMetrics.isSteadyState = true;
            const double targetR { getTargetRadius() };
            const double delta0 { std::abs(wheelbase / targetR) };
            const double deltaAbs { std::abs(steerAngle) };

            if (mMetrics.instantaneousLatAcc > 0.05) {
                const double deltaDiff { deltaAbs - delta0 };
                const double kusRad { deltaDiff / mMetrics.instantaneousLatAcc };
                const double kusDeg { kusRad * kRadToDeg };
                mMetrics.understeerGradient = kusDeg;

                if (kusDeg > 0.5) {
                    mMetrics.balance = HandlingClassification::Understeer;
                } else if (kusDeg < -0.5) {
                    mMetrics.balance = HandlingClassification::Oversteer;
                } else {
                    mMetrics.balance = HandlingClassification::Neutral;
                }
            }
        }
    } else {
        mSteadyStateTimer = 0.0;
        mMetrics.isSteadyState = false;
    }
}

void SkidpadCoordinator::checkPylonCollisions(vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const vehicle_sim::math::Vec2 center { body.getPosition() };
    const double angle { body.getOrientation() };
    const double cosA { std::cos(angle) };
    const double sinA { std::sin(angle) };

    const double hx { mVehicleLength * 0.5 };
    const double hy { mVehicleWidth * 0.5 };
    const vehicle_sim::math::Vec2 vel { body.getLinearVelocity() };

    for (std::size_t i { 0U }; i < mActivePylonCount; ++i) {
        auto& pylon = mPylons[i];
        const vehicle_sim::math::Vec2 d { pylon.position - center };
        const double xLocal { d.x * cosA + d.y * sinA };
        const double yLocal { -d.x * sinA + d.y * cosA };

        const double xClamped { std::clamp(xLocal, -hx, hx) };
        const double yClamped { std::clamp(yLocal, -hy, hy) };

        const double dx { xLocal - xClamped };
        const double dy { yLocal - yClamped };
        const double distSq { dx * dx + dy * dy };

        if (distSq <= (pylon.radius * pylon.radius)) {
            if (!pylon.wasHit) {
                pylon.wasHit = true;
                pylon.toppleAngle = 1.5707963267948966; // 90 deg tilt
                if (vel.lengthSquared() > 0.01) {
                    pylon.toppleDir = vel.normalized();
                }
                mMetrics.hitCount++;
            }
        }
    }
}

void SkidpadCoordinator::resetToStaging(vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const double targetR { getTargetRadius() };
    const vehicle_sim::math::Vec2 stagingPos { -mStagingLength, -targetR };

    body.setPosition(stagingPos);
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    body.setAngularVelocity(0.0);
    body.setOrientation(0.0);

    mState = SkidpadState::Staging;
    mLastPos = stagingPos;
    mLastTheta = std::atan2(-targetR, -mStagingLength);
    mAccumulatedAngle = 0.0;
    mSteadyStateTimer = 0.0;
    mLastSpeed = 0.0;
    mLastYawRate = 0.0;
    mGateArmed = false;

    mMetrics.currentRadius = targetR;
    mMetrics.instantaneousSpeed = 0.0;
    mMetrics.instantaneousLatAcc = 0.0;
    mMetrics.currentLapTime = 0.0;
    mMetrics.radiusError = 0.0;
    mMetrics.steerAngleDeg = 0.0;
    mMetrics.understeerGradient = 0.0;
    mMetrics.hitCount = 0U;
    mMetrics.direction = SkidpadDirection::Unknown;
    mMetrics.balance = HandlingClassification::Understeer;
    mMetrics.isSteadyState = false;

    for (std::size_t i { 0U }; i < mActivePylonCount; ++i) {
        mPylons[i].wasHit = false;
        mPylons[i].toppleAngle = 0.0;
        mPylons[i].toppleDir = vehicle_sim::math::Vec2 { 1.0, 0.0 };
    }
}

void SkidpadCoordinator::toggleEnabled() noexcept
{
    mEnabled = !mEnabled;
}

void SkidpadCoordinator::setEnabled(bool enabled) noexcept
{
    mEnabled = enabled;
}

bool SkidpadCoordinator::isEnabled() const noexcept
{
    return mEnabled;
}

SkidpadState SkidpadCoordinator::getState() const noexcept
{
    return mState;
}

const SkidpadMetrics& SkidpadCoordinator::getMetrics() const noexcept
{
    return mMetrics;
}

const std::array<SkidpadPylon, SkidpadCoordinator::kMaxPylons>& SkidpadCoordinator::getPylons() const noexcept
{
    return mPylons;
}

std::size_t SkidpadCoordinator::getActivePylonCount() const noexcept
{
    return mActivePylonCount;
}

void SkidpadCoordinator::rebuildCourseGeometry() noexcept
{
    const double targetR { getTargetRadius() };
    const double halfW { mCorridorWidth * 0.5 };
    const double rInner { targetR - halfW };
    const double rOuter { targetR + halfW };

    std::size_t nInner { 0U };
    std::size_t nOuter { 0U };

    if (mPreset == SkidpadRadiusPreset::Radius15m) {
        nInner = 16U;
        nOuter = 20U;
    } else {
        nInner = 32U;
        nOuter = 36U;
    }

    mActivePylonCount = nInner + nOuter;
    std::size_t coneIdx { 0U };

    // Place inner ring
    for (std::size_t i { 0U }; i < nInner; ++i) {
        const double frac { static_cast<double>(i) / static_cast<double>(nInner) };
        const double angle { frac * kTwoPi };
        mPylons[coneIdx]
            = SkidpadPylon { vehicle_sim::math::Vec2 { rInner * std::cos(angle), rInner * std::sin(angle) }, 0.4,
                  vehicle_sim::math::Vec2 { 1.0, 0.0 }, 0.0, static_cast<std::uint32_t>(coneIdx), false, false };
        coneIdx++;
    }

    // Place outer ring
    for (std::size_t i { 0U }; i < nOuter; ++i) {
        const double frac { static_cast<double>(i) / static_cast<double>(nOuter) };
        const double angle { frac * kTwoPi };
        mPylons[coneIdx]
            = SkidpadPylon { vehicle_sim::math::Vec2 { rOuter * std::cos(angle), rOuter * std::sin(angle) }, 0.4,
                  vehicle_sim::math::Vec2 { 1.0, 0.0 }, 0.0, static_cast<std::uint32_t>(coneIdx), true, false };
        coneIdx++;
    }
}

void SkidpadCoordinator::updateDimensions(double vehicleWidth, double vehicleLength) noexcept
{
    mVehicleWidth = std::max(0.5, vehicleWidth);
    mVehicleLength = std::max(1.0, vehicleLength);

    const double targetR { getTargetRadius() };
    const double swept { std::pow(0.75 * mVehicleLength, 2.0) / (2.0 * targetR) };
    mCorridorWidth = std::max(3.50, mVehicleWidth + swept + 1.20);
    mStagingLength = std::max(40.0, 5.0 * mVehicleLength);

    rebuildCourseGeometry();
}

double SkidpadCoordinator::getCorridorWidth() const noexcept
{
    return mCorridorWidth;
}

double SkidpadCoordinator::getStagingLength() const noexcept
{
    return mStagingLength;
}

double SkidpadCoordinator::getVehicleWidth() const noexcept
{
    return mVehicleWidth;
}

double SkidpadCoordinator::getVehicleLength() const noexcept
{
    return mVehicleLength;
}

} // namespace motion_lab
