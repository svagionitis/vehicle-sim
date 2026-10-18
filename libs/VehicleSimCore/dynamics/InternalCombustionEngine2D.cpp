#include "libs/VehicleSimCore/dynamics/InternalCombustionEngine2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

namespace {
constexpr double kPi { 3.14159265358979323846 };
constexpr double kRadPerSecToRpm { 60.0 / (2.0 * kPi) };
constexpr double kRpmToRadPerSec { (2.0 * kPi) / 60.0 };
} // namespace

InternalCombustionEngine2D::InternalCombustionEngine2D() noexcept
    : InternalCombustionEngine2D(ICEConfig {}) {
}

InternalCombustionEngine2D::InternalCombustionEngine2D(const ICEConfig& config) noexcept
    : config_ { config }
    , currentRpm_ { config.idleRpm }
    , currentAngVel_ { config.idleRpm * kRpmToRadPerSec }
    , effectiveThrottle_ { 0.0 }
    , idleIntegral_ { 0.0 }
    , isRunning_ { true }
    , isStalled_ { false }
    , isFuelCut_ { false } {
}

EngineOutput InternalCombustionEngine2D::step(double throttle, double shaftSpeed, double dt) noexcept {
    EngineOutput out {};

    if (dt <= 0.0) {
        out.angularVelocity = currentAngVel_;
        out.rpm = currentRpm_;
        out.torque = 0.0;
        out.power = 0.0;
        out.throttleLagged = effectiveThrottle_;
        out.isRedline = isFuelCut_;
        out.isStalled = isStalled_;
        return out;
    }

    currentAngVel_ = std::max(0.0, shaftSpeed);
    currentRpm_ = currentAngVel_ * kRadPerSecToRpm;

    if (isRunning_ && (currentRpm_ < config_.stallRpm)) {
        isStalled_ = true;
        isRunning_ = false;
    }

    const double clampedThrottle { std::clamp(throttle, 0.0, 1.0) };
    const double alpha { (config_.intakeLag > 1e-4) ? (dt / (config_.intakeLag + dt)) : 1.0 };
    effectiveThrottle_ += alpha * (clampedThrottle - effectiveThrottle_);

    if (currentRpm_ >= config_.redlineRpm) {
        isFuelCut_ = true;
    } else if (currentRpm_ <= (config_.redlineRpm - config_.hysteresisRpm)) {
        isFuelCut_ = false;
    }

    double taperFactor { 1.0 };
    if (!isFuelCut_ && (currentRpm_ > config_.softCutRpm) && (config_.redlineRpm > config_.softCutRpm)) {
        const double frac { (currentRpm_ - config_.softCutRpm) / (config_.redlineRpm - config_.softCutRpm) };
        taperFactor = 0.5 * (1.0 + std::cos(kPi * std::clamp(frac, 0.0, 1.0)));
    }

    double grossTorque { 0.0 };
    double idleTorque { 0.0 };

    if (isRunning_ && !isStalled_) {
        if (!isFuelCut_) {
            grossTorque = effectiveThrottle_ * computeWotTorque(currentRpm_) * taperFactor;
        }

        const double idleErr { config_.idleRpm - currentRpm_ };
        idleIntegral_ += idleErr * dt;
        idleIntegral_ = std::clamp(idleIntegral_, 0.0, 500.0);
        idleTorque = std::max(0.0, (config_.idleKp * idleErr) + (config_.idleKi * idleIntegral_));
    }

    const double frictionTorque { computeFriction(currentAngVel_) };
    const double netTorque { (grossTorque + idleTorque) - frictionTorque };

    out.torque = netTorque;
    out.power = netTorque * currentAngVel_;
    out.angularVelocity = currentAngVel_;
    out.rpm = currentRpm_;
    out.throttleLagged = effectiveThrottle_;
    out.isRedline = isFuelCut_ || (currentRpm_ >= config_.redlineRpm);
    out.isStalled = isStalled_;

    return out;
}

void InternalCombustionEngine2D::reset() noexcept {
    currentRpm_ = config_.idleRpm;
    currentAngVel_ = config_.idleRpm * kRpmToRadPerSec;
    effectiveThrottle_ = 0.0;
    idleIntegral_ = 0.0;
    isRunning_ = true;
    isStalled_ = false;
    isFuelCut_ = false;
}

void InternalCombustionEngine2D::start() noexcept {
    isRunning_ = true;
    isStalled_ = false;
    isFuelCut_ = false;
    currentRpm_ = config_.idleRpm;
    currentAngVel_ = config_.idleRpm * kRpmToRadPerSec;
    idleIntegral_ = 0.0;
}

void InternalCombustionEngine2D::stop() noexcept {
    isRunning_ = false;
}

EngineType InternalCombustionEngine2D::getEngineType() const noexcept {
    return EngineType::InternalCombustion;
}

bool InternalCombustionEngine2D::isRunning() const noexcept {
    return isRunning_ && !isStalled_;
}

double InternalCombustionEngine2D::getPeakTorque() const noexcept {
    return config_.peakTorque;
}

double InternalCombustionEngine2D::getRedlineRpm() const noexcept {
    return config_.redlineRpm;
}

double InternalCombustionEngine2D::computeWotTorque(double rpm) const noexcept {
    if (config_.peakTorqueRpm <= 1.0) {
        return config_.peakTorque;
    }

    const double normRpm { rpm / config_.peakTorqueRpm };
    const double norm2 { normRpm * normRpm };
    const double norm3 { norm2 * normRpm };

    const double poly { config_.torquePoly[0] +
                        (config_.torquePoly[1] * normRpm) +
                        (config_.torquePoly[2] * norm2) +
                        (config_.torquePoly[3] * norm3) };

    return std::max(0.0, config_.peakTorque * poly);
}

double InternalCombustionEngine2D::computeFriction(double angVel) const noexcept {
    const double absSpeed { std::abs(angVel) };
    return config_.frictionCoeff0 +
           (config_.frictionCoeff1 * absSpeed) +
           (config_.frictionCoeff2 * absSpeed * absSpeed);
}

double InternalCombustionEngine2D::getCurrentRpm() const noexcept {
    return currentRpm_;
}

double InternalCombustionEngine2D::getCurrentAngVel() const noexcept {
    return currentAngVel_;
}

bool InternalCombustionEngine2D::isStalled() const noexcept {
    return isStalled_;
}

bool InternalCombustionEngine2D::isFuelCut() const noexcept {
    return isFuelCut_;
}

} // namespace vehicle_sim::dynamics
