#include "libs/VehicleSimCore/dynamics/Differential2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

Differential2D::Differential2D() noexcept
    : Differential2D(DifferentialConfig {}) {
}

Differential2D::Differential2D(const DifferentialConfig& config) noexcept
    : config_ { config } {
}

DifferentialOutput Differential2D::step(double inputTorque,
                                        double leftSpeed,
                                        double rightSpeed,
                                        double yawBiasCommand) const noexcept {
    DifferentialOutput out {};

    const double netInput { inputTorque * config_.finalDriveEfficiency };
    const double baseTorque { 0.5 * netInput };
    const double deltaSpeed { leftSpeed - rightSpeed };
    const double carrierSpeed { 0.5 * (leftSpeed + rightSpeed) };

    out.carrierSpeed = carrierSpeed;
    out.deltaSpeed = deltaSpeed;

    double leftTorque { baseTorque };
    double rightTorque { baseTorque };
    double lockTorque { 0.0 };

    switch (config_.type) {
        case DifferentialType::Open:
            leftTorque = baseTorque;
            rightTorque = baseTorque;
            lockTorque = 0.0;
            break;

        case DifferentialType::LimitedSlipViscous: {
            lockTorque = config_.viscousStiffness * deltaSpeed;
            leftTorque = baseTorque - lockTorque;
            rightTorque = baseTorque + lockTorque;
            break;
        }

        case DifferentialType::LimitedSlipSalisbury: {
            const double rampRatio { (netInput >= 0.0) ? config_.driveRampRatio : config_.coastRampRatio };
            const double maxLock { config_.preloadTorque + (rampRatio * std::abs(netInput)) };
            const double tanhSlip { std::tanh(deltaSpeed / 0.5) };
            lockTorque = maxLock * tanhSlip;
            leftTorque = baseTorque - lockTorque;
            rightTorque = baseTorque + lockTorque;
            break;
        }

        case DifferentialType::Torsen: {
            const double safeBias { std::max(1.0, config_.torsenBiasRatio) };
            if (std::abs(deltaSpeed) < 0.05) {
                leftTorque = baseTorque;
                rightTorque = baseTorque;
                lockTorque = 0.0;
            } else if (deltaSpeed > 0.0) {
                // Left wheel spinning faster -> right wheel receives high torque
                const double highRatio { safeBias / (1.0 + safeBias) };
                const double lowRatio { 1.0 / (1.0 + safeBias) };
                leftTorque = netInput * lowRatio;
                rightTorque = netInput * highRatio;
                lockTorque = (highRatio - 0.5) * netInput;
            } else {
                // Right wheel spinning faster -> left wheel receives high torque
                const double highRatio { safeBias / (1.0 + safeBias) };
                const double lowRatio { 1.0 / (1.0 + safeBias) };
                leftTorque = netInput * highRatio;
                rightTorque = netInput * lowRatio;
                lockTorque = (highRatio - 0.5) * netInput;
            }
            break;
        }

        case DifferentialType::LockedSpool: {
            // High spring-damping locking torque forcing wheel speed synchronization
            constexpr double kSpoolStiffness { 2500.0 };
            lockTorque = kSpoolStiffness * std::clamp(deltaSpeed, -2.0, 2.0);
            leftTorque = baseTorque - lockTorque;
            rightTorque = baseTorque + lockTorque;
            break;
        }

        case DifferentialType::ActiveTorqueVectoring: {
            const double bias { std::clamp(yawBiasCommand, -1.0, 1.0) };
            lockTorque = baseTorque * bias;
            leftTorque = baseTorque - lockTorque;
            rightTorque = baseTorque + lockTorque;
            break;
        }

        default:
            leftTorque = baseTorque;
            rightTorque = baseTorque;
            lockTorque = 0.0;
            break;
    }

    out.leftTorque = leftTorque;
    out.rightTorque = rightTorque;
    out.lockingTorque = lockTorque;

    return out;
}

void Differential2D::setType(DifferentialType type) noexcept {
    config_.type = type;
}

DifferentialType Differential2D::getType() const noexcept {
    return config_.type;
}

const DifferentialConfig& Differential2D::getConfig() const noexcept {
    return config_;
}

void Differential2D::setConfig(const DifferentialConfig& config) noexcept {
    config_ = config;
}

} // namespace vehicle_sim::dynamics
