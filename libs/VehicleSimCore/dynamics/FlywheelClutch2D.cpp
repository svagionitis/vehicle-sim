#include "libs/VehicleSimCore/dynamics/FlywheelClutch2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

namespace {
constexpr double kPi { 3.14159265358979323846 };
constexpr double kRpmToRadPerSec { (2.0 * kPi) / 60.0 };
} // namespace

FlywheelClutch2D::FlywheelClutch2D() noexcept
    : FlywheelClutch2D(ClutchConfig {}) {
}

FlywheelClutch2D::FlywheelClutch2D(const ClutchConfig& config) noexcept
    : config_ { config }
    , engineAngVel_ { 900.0 * kRpmToRadPerSec }
    , transAngVel_ { 0.0 }
    , currentPedal_ { 0.0 }
    , lastState_ {} {
}

ClutchState FlywheelClutch2D::step(double engineTorque,
                                   double loadTorque,
                                   double engineInertia,
                                   double transInertia,
                                   double pedalCommand,
                                   double dt) noexcept {
    ClutchState state {};

    if (dt <= 0.0) {
        state = lastState_;
        return state;
    }

    const double clampedCmd { std::clamp(pedalCommand, 0.0, 1.0) };
    const double maxPedalDelta { config_.engagementRate * dt };
    currentPedal_ += std::clamp(clampedCmd - currentPedal_, -maxPedalDelta, maxPedalDelta);

    const double lockRatio { std::clamp(1.0 - currentPedal_, 0.0, 1.0) };
    const double capacity { lockRatio * config_.maxClutchTorque };

    const double safeEngineInertia { std::max(0.01, engineInertia) };
    const double safeTransInertia { std::max(0.01, transInertia) };
    const double totalInertia { safeEngineInertia + safeTransInertia };

    const double stickTorque { ((safeTransInertia * engineTorque) + (safeEngineInertia * loadTorque)) / totalInertia };
    const double slipVel { engineAngVel_ - transAngVel_ };

    const bool canLock { (lockRatio > 0.05) &&
                         (std::abs(slipVel) <= config_.slipThreshold) &&
                         (std::abs(stickTorque) <= capacity) };

    double transferredTorque { 0.0 };

    if (canLock) {
        const double commonAcc { (engineTorque - loadTorque) / totalInertia };
        engineAngVel_ += commonAcc * dt;
        transAngVel_ = engineAngVel_;
        transferredTorque = stickTorque;

        state.isLocked = true;
        state.isSlipping = false;
    } else {
        const double safeThreshold { std::max(0.05, config_.slipThreshold) };
        transferredTorque = capacity * std::tanh(slipVel / safeThreshold);

        const double engineAcc { (engineTorque - transferredTorque) / safeEngineInertia };
        const double transAcc { (transferredTorque - loadTorque) / safeTransInertia };

        engineAngVel_ += engineAcc * dt;
        transAngVel_ += transAcc * dt;

        state.isLocked = false;
        state.isSlipping = (capacity > 1e-4) && (std::abs(slipVel) > 0.05);
    }

    engineAngVel_ = std::max(0.0, engineAngVel_);

    state.transferredTorque = transferredTorque;
    state.slipVelocity = engineAngVel_ - transAngVel_;
    state.clutchPedal = currentPedal_;
    state.lockRatio = lockRatio;

    lastState_ = state;
    return state;
}

void FlywheelClutch2D::reset(double initialEngineRpm) noexcept {
    engineAngVel_ = std::max(0.0, initialEngineRpm * kRpmToRadPerSec);
    transAngVel_ = 0.0;
    currentPedal_ = 0.0;
    lastState_ = ClutchState {};
}

double FlywheelClutch2D::getEngineSpeed() const noexcept {
    return engineAngVel_;
}

double FlywheelClutch2D::getTransSpeed() const noexcept {
    return transAngVel_;
}

void FlywheelClutch2D::setTransSpeed(double speed) noexcept {
    transAngVel_ = speed;
}

const ClutchState& FlywheelClutch2D::getState() const noexcept {
    return lastState_;
}

} // namespace vehicle_sim::dynamics
