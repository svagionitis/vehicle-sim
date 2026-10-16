#include "apps/MotionLab/SlalomCourseCoordinator.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

SlalomCourseCoordinator::SlalomCourseCoordinator() noexcept {
    for (std::size_t i { 0U }; i < kPylonCount; ++i) {
        mPylons[i].position = vehicle_sim::math::Vec2 {
            static_cast<double>(i + 1U) * kPylonSpacing,
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

    // Approximate vehicle chassis extents (half-length = 2.0m, half-width = 1.0m)
    constexpr double hx { 2.0 };
    constexpr double hy { 1.0 };

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
    // Check Sector 1 split
    if (!mSec1Recorded && currPos.x >= kSector1X) {
        mSec1Recorded = true;
        mMetrics.sector1Time = mMetrics.elapsedTime;
    }

    // Check Sector 2 split
    if (!mSec2Recorded && currPos.x >= kSector2X) {
        mSec2Recorded = true;
        mMetrics.sector2Time = mMetrics.elapsedTime - mMetrics.sector1Time;
    }

    // Check Finish gate
    if (currPos.x >= kFinishX) {
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
    body.setPosition(vehicle_sim::math::Vec2 { -15.0, 0.0 });
    body.setOrientation(0.0);
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    body.setAngularVelocity(0.0);
    body.clearForces();

    mLastPos = vehicle_sim::math::Vec2 { -15.0, 0.0 };
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

} // namespace motion_lab
