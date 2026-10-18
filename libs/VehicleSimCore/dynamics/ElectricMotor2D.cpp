#include "libs/VehicleSimCore/dynamics/ElectricMotor2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

namespace {
constexpr double kPi { 3.14159265358979323846 };
constexpr double kRadPerSecToRpm { 60.0 / (2.0 * kPi) };
constexpr double kRpmToRadPerSec { (2.0 * kPi) / 60.0 };
} // namespace

ElectricMotor2D::ElectricMotor2D() noexcept
    : ElectricMotor2D(ElectricMotorConfig {}) {
}

ElectricMotor2D::ElectricMotor2D(const ElectricMotorConfig& config) noexcept
    : config_ { config }
    , currentTorque_ { 0.0 }
    , currentSpeed_ { 0.0 }
    , currentRpm_ { 0.0 }
    , isEnabled_ { true } {
}

EngineOutput ElectricMotor2D::step(double throttle, double shaftSpeed, double dt) noexcept {
    EngineOutput out {};

    currentSpeed_ = shaftSpeed;
    currentRpm_ = std::abs(currentSpeed_) * kRadPerSecToRpm;

    if (dt <= 0.0 || !isEnabled_) {
        out.angularVelocity = currentSpeed_;
        out.rpm = currentRpm_;
        out.torque = 0.0;
        out.power = 0.0;
        out.throttleLagged = 0.0;
        out.isRedline = currentRpm_ >= config_.maxSpeedRpm;
        out.isStalled = false;
        return out;
    }

    const double clampedThrottle { std::clamp(throttle, -1.0, 1.0) };
    double targetTorque { 0.0 };

    if (clampedThrottle >= 0.0) {
        targetTorque = clampedThrottle * computeMaxTorque(currentRpm_);
    } else {
        const double regenCmd { -clampedThrottle };
        targetTorque = computeRegen(currentRpm_, regenCmd);
    }

    const double alpha { (config_.electricalLag > 1e-4) ? (dt / (config_.electricalLag + dt)) : 1.0 };
    currentTorque_ += alpha * (targetTorque - currentTorque_);

    out.torque = currentTorque_;
    out.power = currentTorque_ * currentSpeed_;
    out.angularVelocity = currentSpeed_;
    out.rpm = currentRpm_;
    out.throttleLagged = clampedThrottle;
    out.isRedline = currentRpm_ >= config_.maxSpeedRpm;
    out.isStalled = false;

    return out;
}

void ElectricMotor2D::reset() noexcept {
    currentTorque_ = 0.0;
    currentSpeed_ = 0.0;
    currentRpm_ = 0.0;
    isEnabled_ = true;
}

void ElectricMotor2D::start() noexcept {
    isEnabled_ = true;
}

void ElectricMotor2D::stop() noexcept {
    isEnabled_ = false;
    currentTorque_ = 0.0;
}

EngineType ElectricMotor2D::getEngineType() const noexcept {
    return EngineType::ElectricMotor;
}

bool ElectricMotor2D::isRunning() const noexcept {
    return isEnabled_;
}

double ElectricMotor2D::getPeakTorque() const noexcept {
    return config_.maxTorque;
}

double ElectricMotor2D::getRedlineRpm() const noexcept {
    return config_.maxSpeedRpm;
}

double ElectricMotor2D::computeMaxTorque(double rpm) const noexcept {
    const double absRpm { std::abs(rpm) };
    double torque { 0.0 };

    if (absRpm <= config_.baseSpeedRpm) {
        torque = config_.maxTorque;
    } else if (absRpm <= config_.maxSpeedRpm) {
        const double safeBase { (config_.baseSpeedRpm > 1.0) ? config_.baseSpeedRpm : 1.0 };
        torque = config_.maxTorque * (safeBase / std::max(absRpm, 1.0));
    } else {
        const double overspeed { absRpm - config_.maxSpeedRpm };
        constexpr double kCutoffMargin { 500.0 };
        if (overspeed < kCutoffMargin) {
            const double safeBase { (config_.baseSpeedRpm > 1.0) ? config_.baseSpeedRpm : 1.0 };
            const double baseTorque { config_.maxTorque * (safeBase / config_.maxSpeedRpm) };
            const double frac { overspeed / kCutoffMargin };
            torque = baseTorque * 0.5 * (1.0 + std::cos(kPi * frac));
        } else {
            torque = 0.0;
        }
    }

    const double angVel { absRpm * kRpmToRadPerSec };
    if (angVel > 1e-4) {
        const double powerLimitedTorque { config_.maxPower / angVel };
        torque = std::min(torque, powerLimitedTorque);
    }

    return std::max(0.0, torque);
}

double ElectricMotor2D::computeRegen(double rpm, double brakeCmd) const noexcept {
    const double absRpm { std::abs(rpm) };
    const double clampedCmd { std::clamp(brakeCmd, 0.0, 1.0) };

    if (clampedCmd <= 0.0 || absRpm < 1.0) {
        return 0.0;
    }

    // Smooth speed taper down to 0 below 50 RPM to prevent creeping in reverse
    const double speedTaper { std::clamp(absRpm / 50.0, 0.0, 1.0) };

    const double angVel { absRpm * kRpmToRadPerSec };
    double maxRegen { config_.maxRegenTorque };
    if (angVel > 1e-4) {
        const double powerLimit { config_.maxPower / angVel };
        maxRegen = std::min(maxRegen, powerLimit);
    }

    return -1.0 * clampedCmd * maxRegen * speedTaper * config_.regenEfficiency;
}

double ElectricMotor2D::getCurrentTorque() const noexcept {
    return currentTorque_;
}

double ElectricMotor2D::getCurrentSpeed() const noexcept {
    return currentSpeed_;
}

} // namespace vehicle_sim::dynamics
