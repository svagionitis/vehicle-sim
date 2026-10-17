#include "apps/MotionLab/MooseTestCoordinator.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

MooseTestCoordinator::MooseTestCoordinator() noexcept {
    recomputeBounds();
}

void MooseTestCoordinator::updateDimensions(double vehicleWidth, double vehicleLength) noexcept {
    mVehicleWidth = std::max(vehicleWidth, 0.5);
    mVehicleLength = std::max(vehicleLength, 0.5);
    recomputeBounds();
}

void MooseTestCoordinator::recomputeBounds() noexcept {
    // ISO 3888-2 section width formulas
    const double w1 { 1.1 * mVehicleWidth + 0.25 };
    const double w3 { mVehicleWidth + 1.0 };
    const double w5 { std::max(1.3 * mVehicleWidth + 0.25, 3.0) };

    mSections[0] = MooseTestSection { 0.0, 12.0, w1, 0.0, 1U };
    mSections[1] = MooseTestSection { 12.0, 25.5, w1, 0.0, 2U };
    mSections[2] = MooseTestSection { 25.5, 36.5, w3, 3.5, 3U };
    mSections[3] = MooseTestSection { 36.5, 49.0, w5, 0.0, 4U };
    mSections[4] = MooseTestSection { 49.0, 61.0, w5, 0.0, 5U };

    const double halfW1 { w1 * 0.5 + 0.4 };
    const double halfW3 { w3 * 0.5 + 0.4 };
    const double halfW5 { w5 * 0.5 + 0.4 };

    // Section 1: 6 cones (x = 0, 6, 12m)
    mCones[0] = MooseTestCone { vehicle_sim::math::Vec2 { 0.0, +halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 0U, 1U, false };
    mCones[1] = MooseTestCone { vehicle_sim::math::Vec2 { 0.0, -halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 1U, 1U, false };
    mCones[2] = MooseTestCone { vehicle_sim::math::Vec2 { 6.0, +halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 2U, 1U, false };
    mCones[3] = MooseTestCone { vehicle_sim::math::Vec2 { 6.0, -halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 3U, 1U, false };
    mCones[4] = MooseTestCone { vehicle_sim::math::Vec2 { 12.0, +halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 4U, 1U, false };
    mCones[5] = MooseTestCone { vehicle_sim::math::Vec2 { 12.0, -halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 5U, 1U, false };

    // Section 2: 2 transition guide cones
    mCones[6] = MooseTestCone { vehicle_sim::math::Vec2 { 18.75, -halfW1 }, 0.4, { 1.0, 0.0 }, 0.0, 6U, 2U, false };
    mCones[7] = MooseTestCone { vehicle_sim::math::Vec2 { 25.5, 3.5 - halfW3 - 0.5 }, 0.4, { 1.0, 0.0 }, 0.0, 7U, 2U, false };

    // Section 3: 6 cones (x = 25.5, 31.0, 36.5m centered at y = +3.5m)
    mCones[8]  = MooseTestCone { vehicle_sim::math::Vec2 { 25.5, 3.5 + halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 8U, 3U, false };
    mCones[9]  = MooseTestCone { vehicle_sim::math::Vec2 { 25.5, 3.5 - halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 9U, 3U, false };
    mCones[10] = MooseTestCone { vehicle_sim::math::Vec2 { 31.0, 3.5 + halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 10U, 3U, false };
    mCones[11] = MooseTestCone { vehicle_sim::math::Vec2 { 31.0, 3.5 - halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 11U, 3U, false };
    mCones[12] = MooseTestCone { vehicle_sim::math::Vec2 { 36.5, 3.5 + halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 12U, 3U, false };
    mCones[13] = MooseTestCone { vehicle_sim::math::Vec2 { 36.5, 3.5 - halfW3 }, 0.4, { 1.0, 0.0 }, 0.0, 13U, 3U, false };

    // Section 4: 2 transition guide cones
    mCones[14] = MooseTestCone { vehicle_sim::math::Vec2 { 36.5, 3.5 + halfW3 + 0.5 }, 0.4, { 1.0, 0.0 }, 0.0, 14U, 4U, false };
    mCones[15] = MooseTestCone { vehicle_sim::math::Vec2 { 42.75, +halfW5 + 0.5 }, 0.4, { 1.0, 0.0 }, 0.0, 15U, 4U, false };

    // Section 5: 8 cones (x = 49, 53, 57, 61m centered at y = 0.0m)
    mCones[16] = MooseTestCone { vehicle_sim::math::Vec2 { 49.0, +halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 16U, 5U, false };
    mCones[17] = MooseTestCone { vehicle_sim::math::Vec2 { 49.0, -halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 17U, 5U, false };
    mCones[18] = MooseTestCone { vehicle_sim::math::Vec2 { 53.0, +halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 18U, 5U, false };
    mCones[19] = MooseTestCone { vehicle_sim::math::Vec2 { 53.0, -halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 19U, 5U, false };
    mCones[20] = MooseTestCone { vehicle_sim::math::Vec2 { 57.0, +halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 20U, 5U, false };
    mCones[21] = MooseTestCone { vehicle_sim::math::Vec2 { 57.0, -halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 21U, 5U, false };
    mCones[22] = MooseTestCone { vehicle_sim::math::Vec2 { 61.0, +halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 22U, 5U, false };
    mCones[23] = MooseTestCone { vehicle_sim::math::Vec2 { 61.0, -halfW5 }, 0.4, { 1.0, 0.0 }, 0.0, 23U, 5U, false };
}

void MooseTestCoordinator::stepPhysics(vehicle_sim::physics::RigidBody2D& body,
                                      double throttleInput,
                                      double brakeInput,
                                      double dt) noexcept {
    if (!mEnabled) {
        return;
    }

    if (mState == MooseTestState::IdleAtStaging || mState == MooseTestState::Approaching) {
        updateIdle(body);
    }

    if (mState == MooseTestState::InManeuver) {
        updateRunning(body, throttleInput, brakeInput, dt);
    }

    mLastPos = body.getPosition();
}

void MooseTestCoordinator::updateIdle(const vehicle_sim::physics::RigidBody2D& body) noexcept {
    const double currX { body.getPosition().x };
    const double speedMs { body.getLinearVelocity().length() };

    if (mState == MooseTestState::IdleAtStaging && currX > kStagingX + 1.0 && speedMs > 1.0) {
        mState = MooseTestState::Approaching;
    }

    // Crossing start line
    if (mLastPos.x < kStartX && currX >= kStartX) {
        if (speedMs > 1.0) { // Arming speed > 3.6 km/h
            mState = MooseTestState::InManeuver;
            mPassedEntry = true;
            mMetrics.entrySpeed = speedMs * 3.6;
            mMetrics.elapsedTime = 0.0;
            mMetrics.result = MooseRunResult::Pending;
        }
    }
}

void MooseTestCoordinator::updateRunning(vehicle_sim::physics::RigidBody2D& body,
                                         double throttleInput,
                                         double brakeInput,
                                         double dt) noexcept {
    mMetrics.elapsedTime += dt;

    const double speedMs { body.getLinearVelocity().length() };
    const double yawRateRad { std::abs(body.getAngularVelocity()) };
    const double latAccG { (speedMs * yawRateRad) / 9.80665 };
    if (latAccG > mMetrics.peakLatAcc) {
        mMetrics.peakLatAcc = latAccG;
    }

    const double yawRateDeg { yawRateRad * 57.29577951308232 };
    if (yawRateDeg > mMetrics.peakYawRate) {
        mMetrics.peakYawRate = yawRateDeg;
    }

    const double currX { body.getPosition().x };

    // Throttle fault check: throttle must be cut after Section 1 (x > 12m)
    if (currX > 12.0 && currX < kFinishX && throttleInput > 0.05) {
        mMetrics.throttleFault = true;
        if (mMetrics.result != MooseRunResult::FailedConeHit) {
            mMetrics.result = MooseRunResult::Disqualified;
        }
    }

    // Brake fault check: service brake must not be applied inside corridor
    if (currX >= kStartX && currX <= kFinishX && brakeInput > 0.05) {
        mMetrics.brakeFault = true;
        if (mMetrics.result != MooseRunResult::FailedConeHit) {
            mMetrics.result = MooseRunResult::Disqualified;
        }
    }

    // Heading stability & spin-out detection
    double heading { body.getOrientation() };
    while (heading > 3.141592653589793) {
        heading -= 6.283185307179586;
    }
    while (heading < -3.141592653589793) {
        heading += 6.283185307179586;
    }
    if (std::abs(heading) > 1.5707963267948966) {
        mMetrics.result = MooseRunResult::FailedSpinOut;
        mState = MooseTestState::FailedOffCourse;
    }

    checkCollisions(body);
    checkGates(mLastPos, body.getPosition(), body);
}

void MooseTestCoordinator::checkCollisions(const vehicle_sim::physics::RigidBody2D& body) noexcept {
    const vehicle_sim::math::Vec2 vehPos { body.getPosition() };
    const double heading { body.getOrientation() };
    const vehicle_sim::math::Vec2 uLong { std::cos(heading), std::sin(heading) };
    const vehicle_sim::math::Vec2 uLat  { -std::sin(heading), std::cos(heading) };

    const double halfLen { mVehicleLength * 0.5 };
    const double halfWid { mVehicleWidth * 0.5 };

    for (auto& cone : mCones) {
        const vehicle_sim::math::Vec2 delta { cone.position - vehPos };
        const double xLocal { delta.x * uLong.x + delta.y * uLong.y };
        const double yLocal { delta.x * uLat.x + delta.y * uLat.y };

        const double xClamp { std::clamp(xLocal, -halfLen, halfLen) };
        const double yClamp { std::clamp(yLocal, -halfWid, halfWid) };

        const double dx { xLocal - xClamp };
        const double dy { yLocal - yClamp };
        const double distSq { dx * dx + dy * dy };

        if (distSq <= cone.radius * cone.radius) {
            if (!cone.wasHit) {
                cone.wasHit = true;
                cone.toppleAngle = 1.5707963267948966;

                const vehicle_sim::math::Vec2 vel { body.getLinearVelocity() };
                const double vLen { vel.length() };
                if (vLen > 0.1) {
                    cone.toppleDir = vel / vLen;
                } else {
                    cone.toppleDir = uLong;
                }

                mMetrics.hitCount++;
                mMetrics.result = MooseRunResult::FailedConeHit;
            }
        }
    }
}

void MooseTestCoordinator::checkGates(const vehicle_sim::math::Vec2& prevPos,
                                      const vehicle_sim::math::Vec2& currPos,
                                      const vehicle_sim::physics::RigidBody2D& body) noexcept {
    // Evasion midpoint split (x = 31.0m)
    if (!mPassedMid && prevPos.x < kEvasionMidX && currPos.x >= kEvasionMidX) {
        mPassedMid = true;
        mMetrics.evasionSpeed = body.getLinearVelocity().length() * 3.6;
    }

    // Finish line (x = 61.0m)
    if (!mPassedFinish && prevPos.x < kFinishX && currPos.x >= kFinishX) {
        mPassedFinish = true;
        mState = MooseTestState::Completed;
        mMetrics.exitSpeed = body.getLinearVelocity().length() * 3.6;
        mMetrics.speedDrop = mMetrics.entrySpeed - mMetrics.exitSpeed;

        if (mMetrics.hitCount == 0U && !mMetrics.throttleFault && !mMetrics.brakeFault) {
            mMetrics.result = MooseRunResult::CleanPass;
            if (!mMetrics.hasValidBest || mMetrics.entrySpeed > mMetrics.bestPassSpeed) {
                mMetrics.bestPassSpeed = mMetrics.entrySpeed;
                mMetrics.hasValidBest = true;
            }
        } else if (mMetrics.hitCount > 0U) {
            mMetrics.result = MooseRunResult::FailedConeHit;
        } else if (mMetrics.throttleFault || mMetrics.brakeFault) {
            mMetrics.result = MooseRunResult::Disqualified;
        }
    }
}

void MooseTestCoordinator::resetCourse(vehicle_sim::physics::RigidBody2D& body) noexcept {
    body.setPosition(vehicle_sim::math::Vec2 { kStagingX, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    body.setAngularVelocity(0.0);
    body.setOrientation(0.0);

    mLastPos = vehicle_sim::math::Vec2 { kStagingX, 0.0 };
    mState = MooseTestState::IdleAtStaging;
    mPassedEntry = false;
    mPassedMid = false;
    mPassedFinish = false;

    mMetrics.entrySpeed = 0.0;
    mMetrics.evasionSpeed = 0.0;
    mMetrics.exitSpeed = 0.0;
    mMetrics.speedDrop = 0.0;
    mMetrics.peakLatAcc = 0.0;
    mMetrics.peakYawRate = 0.0;
    mMetrics.elapsedTime = 0.0;
    mMetrics.hitCount = 0U;
    mMetrics.result = MooseRunResult::Pending;
    mMetrics.throttleFault = false;
    mMetrics.brakeFault = false;

    for (auto& cone : mCones) {
        cone.wasHit = false;
        cone.toppleAngle = 0.0;
    }
}

void MooseTestCoordinator::toggleEnabled() noexcept {
    mEnabled = !mEnabled;
}

void MooseTestCoordinator::setEnabled(bool enabled) noexcept {
    mEnabled = enabled;
}

bool MooseTestCoordinator::isEnabled() const noexcept {
    return mEnabled;
}

MooseTestState MooseTestCoordinator::getState() const noexcept {
    return mState;
}

const MooseTestMetrics& MooseTestCoordinator::getMetrics() const noexcept {
    return mMetrics;
}

const std::array<MooseTestCone, MooseTestCoordinator::kConeCount>&
MooseTestCoordinator::getCones() const noexcept {
    return mCones;
}

const std::array<MooseTestSection, MooseTestCoordinator::kSectionCount>&
MooseTestCoordinator::getSections() const noexcept {
    return mSections;
}

double MooseTestCoordinator::getVehicleWidth() const noexcept {
    return mVehicleWidth;
}

double MooseTestCoordinator::getTargetSpeed() const noexcept {
    return mTargetSpeedKmH;
}

void MooseTestCoordinator::setTargetSpeed(double speedKmH) noexcept {
    mTargetSpeedKmH = std::max(speedKmH, 10.0);
}

} // namespace motion_lab
