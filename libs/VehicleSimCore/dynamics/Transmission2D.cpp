#include "libs/VehicleSimCore/dynamics/Transmission2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

Transmission2D::Transmission2D() noexcept
    : Transmission2D(TransmissionConfig {}) {
}

Transmission2D::Transmission2D(const TransmissionConfig& config) noexcept
    : config_ { config }
    , currentGear_ { 1 }
    , targetGear_ { 1 }
    , shiftState_ { ShiftState::InGear }
    , shiftTimer_ { 0.0 } {
}

TransmissionOutput Transmission2D::step(double clutchTorque,
                                        double propShaftSpeed,
                                        double engineRpm,
                                        double dt) noexcept {
    TransmissionOutput out {};

    if (config_.mode == TransmissionMode::Automatic && shiftState_ == ShiftState::InGear) {
        const auto fwdGears { static_cast<std::int32_t>(config_.forwardGears) };
        if (currentGear_ > 0 && currentGear_ < fwdGears && engineRpm >= config_.upshiftRpm) {
            shiftUp();
        } else if (currentGear_ > 1 && engineRpm <= config_.downshiftRpm) {
            shiftDown();
        }
    }

    const double totalShiftTime { std::max(0.01, config_.shiftDuration) };
    const double tDisengage { 0.20 * totalShiftTime };
    const double tNeutral { 0.20 * totalShiftTime };
    const double tSync { 0.30 * totalShiftTime };
    const double tEngage { 0.30 * totalShiftTime };

    double torqueScale { 1.0 };
    bool isCut { false };
    bool isBlip { false };

    if (shiftState_ != ShiftState::InGear && dt > 0.0) {
        shiftTimer_ += dt;
        isCut = true;

        switch (shiftState_) {
            case ShiftState::Disengaging:
                torqueScale = std::clamp(1.0 - (shiftTimer_ / tDisengage), 0.0, 1.0);
                if (shiftTimer_ >= tDisengage) {
                    shiftTimer_ = 0.0;
                    shiftState_ = ShiftState::InNeutral;
                }
                break;

            case ShiftState::InNeutral:
                torqueScale = 0.0;
                if (shiftTimer_ >= tNeutral) {
                    shiftTimer_ = 0.0;
                    currentGear_ = targetGear_;
                    shiftState_ = ShiftState::Synchronizing;
                }
                break;

            case ShiftState::Synchronizing:
                torqueScale = 0.0;
                isBlip = (targetGear_ > 0) && (targetGear_ < currentGear_);
                if (shiftTimer_ >= tSync) {
                    shiftTimer_ = 0.0;
                    shiftState_ = ShiftState::Engaging;
                }
                break;

            case ShiftState::Engaging:
                torqueScale = std::clamp(shiftTimer_ / tEngage, 0.0, 1.0);
                if (shiftTimer_ >= tEngage) {
                    shiftTimer_ = 0.0;
                    shiftState_ = ShiftState::InGear;
                    torqueScale = 1.0;
                    isCut = false;
                }
                break;

            default:
                shiftState_ = ShiftState::InGear;
                torqueScale = 1.0;
                isCut = false;
                break;
        }
    }

    const double activeRatio { getRatio(currentGear_) };
    const double totalRatio { activeRatio * config_.finalDriveRatio };

    out.currentRatio = activeRatio;
    out.activeGear = currentGear_;
    out.shiftPhase = shiftState_;
    out.shiftProgress = (shiftState_ == ShiftState::InGear) ? 1.0 : std::clamp(shiftTimer_ / totalShiftTime, 0.0, 1.0);
    out.isTorqueCut = isCut;
    out.isRevBlip = isBlip;
    out.propTorque = clutchTorque * totalRatio * config_.efficiency * torqueScale;
    out.inputSpeed = propShaftSpeed * totalRatio;

    return out;
}

void Transmission2D::reset(std::int32_t initialGear) noexcept {
    currentGear_ = initialGear;
    targetGear_ = initialGear;
    shiftState_ = ShiftState::InGear;
    shiftTimer_ = 0.0;
}

bool Transmission2D::shiftUp() noexcept {
    if (shiftState_ != ShiftState::InGear) {
        return false;
    }

    const auto maxGear { static_cast<std::int32_t>(config_.forwardGears) };
    if (currentGear_ < maxGear) {
        targetGear_ = (currentGear_ == -1) ? 0 : (currentGear_ + 1);
        shiftState_ = ShiftState::Disengaging;
        shiftTimer_ = 0.0;
        return true;
    }
    return false;
}

bool Transmission2D::shiftDown() noexcept {
    if (shiftState_ != ShiftState::InGear) {
        return false;
    }

    if (currentGear_ > -1) {
        targetGear_ = currentGear_ - 1;
        shiftState_ = ShiftState::Disengaging;
        shiftTimer_ = 0.0;
        return true;
    }
    return false;
}

bool Transmission2D::selectGear(std::int32_t gear) noexcept {
    const auto maxGear { static_cast<std::int32_t>(config_.forwardGears) };
    if (gear < -1 || gear > maxGear) {
        return false;
    }
    if (gear == currentGear_ && shiftState_ == ShiftState::InGear) {
        return true;
    }

    targetGear_ = gear;
    shiftState_ = ShiftState::Disengaging;
    shiftTimer_ = 0.0;
    return true;
}

void Transmission2D::setMode(TransmissionMode mode) noexcept {
    config_.mode = mode;
}

std::int32_t Transmission2D::getGear() const noexcept {
    return currentGear_;
}

bool Transmission2D::isShifting() const noexcept {
    return shiftState_ != ShiftState::InGear;
}

double Transmission2D::getRatio(std::int32_t gear) const noexcept {
    if (gear == 0) {
        return 0.0;
    }
    if (gear == -1) {
        return config_.reverseRatio;
    }
    const auto maxGear { static_cast<std::int32_t>(config_.forwardGears) };
    if (gear >= 1 && gear <= maxGear) {
        const auto idx { static_cast<std::size_t>(gear - 1) };
        if (idx < config_.gearRatios.size()) {
            return config_.gearRatios[idx];
        }
    }
    return 0.0;
}

double Transmission2D::getCurrentRatio() const noexcept {
    return getRatio(currentGear_);
}

} // namespace vehicle_sim::dynamics
