#include "apps/MotionLab/SlalomCourseCoordinator.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

SlalomCourseCoordinator::SlalomCourseCoordinator() noexcept {
    recomputeLayout();
}

void SlalomCourseCoordinator::updateDimensions(double vehicleWidth, double vehicleLength) noexcept {
    mVehicleWidth = std::max(vehicleWidth, 0.5);
    mVehicleLength = std::max(vehicleLength, 0.5);
    mPylonSpacing = std::max(10.0, 2.5 * mVehicleLength);
    recomputeLayout();
}

void SlalomCourseCoordinator::recomputeLayout() noexcept {
    for (std::size_t i { 0U }; i < kPylonCount; ++i) {
        mPylons[i].position = vehicle_sim::math::Vec2 {
            static_cast<double>(i + 1U) * mPylonSpacing,
            0.0
        };
        mPylons[i].radius = 0.4;
        mPylons[i].toppleDir = vehicle_sim::math::Vec2 { 1.0, 0.0 };
        mPylons[i].toppleAngle = 0.0;
        mPylons[i].coneIndex = static_cast<std::uint32_t>(i + 1U);
        mPylons[i].wasHit = false;
        mPylons[i].passed = false;
        mPylons[i].correctSide = true;
    }
}

void SlalomCourseCoordinator::stepPhysics(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept {
    if (!mEnabled) {
        return;
    }

    if (mState == SlalomState::IdleAtStart) {
        updateIdle(body);
    }
    if (mState == SlalomState::InProgress) {
        updateRunning(body, dt);
    }

    mLastPos = body.getPosition();
}

void SlalomCourseCoordinator::updateIdle(const vehicle_sim::physics::RigidBody2D& body) noexcept {
    const vehicle_sim::math::Vec2 currPos { body.getPosition() };
    const vehicle_sim::math::Vec2 vel { body.getLinearVelocity() };
    const double speed { vel.length() };

    // Trigger start gate when crossing forward at sufficient speed
    if (mLastPos.x < kStartX && currPos.x >= kStartX && speed >= kArmingSpeed) {
        mState = SlalomState::InProgress;
        mMetrics.elapsedTime = 0.0;
        mMetrics.penaltyTime = 0.0;
        mMetrics.finalTime = 0.0;
        mMetrics.sector1Time = 0.0;
        mMetrics.sector2Time = 0.0;
        mMetrics.sector3Time = 0.0;
        mMetrics.hitCount = 0U;
        mMetrics.missedGates = 0U;
        mSec1Recorded = false;
        mSec2Recorded = false;
        mNextConeIdx = 0U;

        for (auto& pylon : mPylons) {
            pylon.wasHit = false;
            pylon.passed = false;
            pylon.correctSide = true;
            pylon.toppleAngle = 0.0;
            pylon.toppleDir = vehicle_sim::math::Vec2 { 1.0, 0.0 };
        }
    }
}

void SlalomCourseCoordinator::updateRunning(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept {
    mMetrics.elapsedTime += dt;
    mMetrics.finalTime = mMetrics.elapsedTime + mMetrics.penaltyTime;

    checkCollisions(body);
    checkGates(mLastPos, body.getPosition());
}

void SlalomCourseCoordinator::checkCollisions(const vehicle_sim::physics::RigidBody2D& body) noexcept {
    const vehicle_sim::math::Vec2 center { body.getPosition() };
    const double angle { body.getOrientation() };
    const double cosA { std::cos(angle) };
    const double sinA { std::sin(angle) };

    const double hx { mVehicleLength * 0.5 };
    const double hy { mVehicleWidth * 0.5 };

    const vehicle_sim::math::Vec2 vel { body.getLinearVelocity() };

    for (auto& pylon : mPylons) {
        const vehicle_sim::math::Vec2 d { pylon.position - center };
        // Transform to local coordinates
        const double xLocal { d.x * cosA + d.y * sinA };
        const double yLocal { -d.x * sinA + d.y * cosA };

        // Clamp to OBB extents
        const double xClamped { std::clamp(xLocal, -hx, hx) };
        const double yClamped { std::clamp(yLocal, -hy, hy) };

        const double dx { xLocal - xClamped };
        const double dy { yLocal - yClamped };
        const double distSq { dx * dx + dy * dy };

        if (distSq <= (pylon.radius * pylon.radius)) {
            if (!pylon.wasHit) {
                pylon.wasHit = true;
                pylon.toppleAngle = 1.5707963267948966; // 90 degrees
                if (vel.lengthSquared() > 0.01) {
                    pylon.toppleDir = vel.normalized();
                }
                mMetrics.hitCount++;
                mMetrics.penaltyTime = static_cast<double>(mMetrics.hitCount) * kPenaltyPerHit;
                mMetrics.finalTime = mMetrics.elapsedTime + mMetrics.penaltyTime;
            }
        }
    }
}

void SlalomCourseCoordinator::checkGates(const vehicle_sim::math::Vec2& prevPos,
                                        const vehicle_sim::math::Vec2& currPos) noexcept {
    // Check Sector 1 split (midway between Cone 3 and Cone 4: 3.5 * mPylonSpacing)
    if (!mSec1Recorded && currPos.x >= getSector1X()) {
        mSec1Recorded = true;
        mMetrics.sector1Time = mMetrics.elapsedTime;
    }

    // Check Sector 2 split (midway between Cone 7 and Cone 8: 7.5 * mPylonSpacing)
    if (!mSec2Recorded && currPos.x >= getSector2X()) {
        mSec2Recorded = true;
        mMetrics.sector2Time = mMetrics.elapsedTime - mMetrics.sector1Time;
    }

    // Check Finish gate (11.0 * mPylonSpacing)
    if (currPos.x >= getFinishX()) {
        mState = SlalomState::Finished;
        mMetrics.sector3Time = mMetrics.elapsedTime - (mMetrics.sector1Time + mMetrics.sector2Time);
        mMetrics.finalTime = mMetrics.elapsedTime + mMetrics.penaltyTime;

        if (!mMetrics.hasValidBest || mMetrics.finalTime < mMetrics.bestTime) {
            mMetrics.bestTime = mMetrics.finalTime;
            mMetrics.hasValidBest = true;
        }
    }

    // Check each pylon weave plane crossing
    for (std::size_t i { 0U }; i < kPylonCount; ++i) {
        auto& pylon { mPylons[i] };
        if (!pylon.passed && prevPos.x < pylon.position.x && currPos.x >= pylon.position.x) {
            pylon.passed = true;
            const double denom { currPos.x - prevPos.x };
            const double alpha { (std::abs(denom) > 1e-6) ? ((pylon.position.x - prevPos.x) / denom) : 0.0 };
            const double yCross { prevPos.y + alpha * (currPos.y - prevPos.y) };

            // Cone 1 (i=0) expects y > 0 (Left), Cone 2 (i=1) expects y < 0 (Right)
            const double expectedSign { (i % 2U == 0U) ? 1.0 : -1.0 };
            if ((expectedSign > 0.0 && yCross < 0.0) || (expectedSign < 0.0 && yCross > 0.0)) {
                pylon.correctSide = false;
                mMetrics.missedGates++;
            }

            if (mNextConeIdx == i) {
                mNextConeIdx = std::min(i + 1U, kPylonCount - 1U);
            }
        }
    }
}

void SlalomCourseCoordinator::resetCourse(vehicle_sim::physics::RigidBody2D& body) noexcept {
    const double stagingX { getStagingX() };
    body.setPosition(vehicle_sim::math::Vec2 { stagingX, 0.0 });
    body.setOrientation(0.0);
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    body.setAngularVelocity(0.0);
    body.clearForces();

    mLastPos = vehicle_sim::math::Vec2 { stagingX, 0.0 };
    mState = SlalomState::IdleAtStart;
    mMetrics.elapsedTime = 0.0;
    mMetrics.penaltyTime = 0.0;
    mMetrics.finalTime = 0.0;
    mMetrics.sector1Time = 0.0;
    mMetrics.sector2Time = 0.0;
    mMetrics.sector3Time = 0.0;
    mMetrics.hitCount = 0U;
    mMetrics.missedGates = 0U;
    mSec1Recorded = false;
    mSec2Recorded = false;
    mNextConeIdx = 0U;

    for (auto& pylon : mPylons) {
        pylon.wasHit = false;
        pylon.passed = false;
        pylon.correctSide = true;
        pylon.toppleAngle = 0.0;
        pylon.toppleDir = vehicle_sim::math::Vec2 { 1.0, 0.0 };
    }
}

void SlalomCourseCoordinator::toggleEnabled() noexcept {
    mEnabled = !mEnabled;
}

void SlalomCourseCoordinator::setEnabled(bool enabled) noexcept {
    mEnabled = enabled;
}

bool SlalomCourseCoordinator::isEnabled() const noexcept {
    return mEnabled;
}

SlalomState SlalomCourseCoordinator::getState() const noexcept {
    return mState;
}

const SlalomMetrics& SlalomCourseCoordinator::getMetrics() const noexcept {
    return mMetrics;
}

const std::array<SlalomPylon, SlalomCourseCoordinator::kPylonCount>& SlalomCourseCoordinator::getPylons() const noexcept {
    return mPylons;
}

double SlalomCourseCoordinator::getNextWeaveSign() const noexcept {
    return (mNextConeIdx % 2U == 0U) ? 1.0 : -1.0;
}

std::uint32_t SlalomCourseCoordinator::getNextConeIndex() const noexcept {
    return static_cast<std::uint32_t>(mNextConeIdx + 1U);
}

double SlalomCourseCoordinator::getPylonSpacing() const noexcept {
    return mPylonSpacing;
}

double SlalomCourseCoordinator::getVehicleWidth() const noexcept {
    return mVehicleWidth;
}

double SlalomCourseCoordinator::getVehicleLength() const noexcept {
    return mVehicleLength;
}

double SlalomCourseCoordinator::getFinishX() const noexcept {
    return 11.0 * mPylonSpacing;
}

double SlalomCourseCoordinator::getSector1X() const noexcept {
    return 3.5 * mPylonSpacing;
}

double SlalomCourseCoordinator::getSector2X() const noexcept {
    return 7.5 * mPylonSpacing;
}

double SlalomCourseCoordinator::getStagingX() const noexcept {
    return -1.5 * mPylonSpacing;
}

double SlalomCourseCoordinator::getGateWidth() const noexcept {
    return std::max(10.0, 4.0 * mVehicleWidth);
}

double SlalomCourseCoordinator::getApexOffset() const noexcept {
    return 0.40 + (mVehicleWidth * 0.5) + 0.15;
}

} // namespace motion_lab
