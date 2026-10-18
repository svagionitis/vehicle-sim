#include "libs/VehicleSimCore/dynamics/Powertrain2D.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/InternalCombustionEngine2D.h"

namespace vehicle_sim::dynamics {

Powertrain2D::Powertrain2D() noexcept
    : engine_ { std::make_unique<InternalCombustionEngine2D>() }
    , clutch_ {}
    , transmission_ {}
    , differential_ {}
    , config_ {}
    , lastState_ {} {
}

Powertrain2D::Powertrain2D(std::unique_ptr<IEngine2D> engine,
                           const ClutchConfig& clutchConfig,
                           const TransmissionConfig& transConfig,
                           const DifferentialConfig& diffConfig,
                           const PowertrainConfig& powertrainConfig) noexcept
    : engine_ { std::move(engine) }
    , clutch_ { clutchConfig }
    , transmission_ { transConfig }
    , differential_ { diffConfig }
    , config_ { powertrainConfig }
    , lastState_ {} {
    if (!engine_) {
        engine_ = std::make_unique<InternalCombustionEngine2D>();
    }
}

PowertrainState Powertrain2D::step(double throttle,
                                   double brake,
                                   double clutchPedal,
                                   double leftWheelSpeed,
                                   double rightWheelSpeed,
                                   double dt) noexcept {
    PowertrainState state {};

    if (dt <= 0.0) {
        state = lastState_;
        return state;
    }

    const double driveWheelSpeed { 0.5 * (leftWheelSpeed + rightWheelSpeed) };
    const double currentRatio { transmission_.getCurrentRatio() };

    double effectiveThrottle { throttle };
    if (transmission_.isShifting()) {
        effectiveThrottle = 0.0;
    }

    if (engine_->getEngineType() == EngineType::ElectricMotor && brake > 0.05) {
        effectiveThrottle = -brake;
    }

    const double engineInertia { 0.20 };
    const double transInertia { 0.05 + (0.01 * std::abs(currentRatio)) };

    const EngineOutput engOut { engine_->step(effectiveThrottle, clutch_.getEngineSpeed(), dt) };

    const double loadTorque { 0.05 * clutch_.getTransSpeed() };
    const ClutchState clutchState { clutch_.step(engOut.torque,
                                                 loadTorque,
                                                 engineInertia,
                                                 transInertia,
                                                 clutchPedal,
                                                 dt) };

    const TransmissionOutput transOut { transmission_.step(clutchState.transferredTorque,
                                                           driveWheelSpeed,
                                                           engOut.rpm,
                                                           dt) };

    const DifferentialOutput diffOut { differential_.step(transOut.propTorque,
                                                          leftWheelSpeed,
                                                          rightWheelSpeed,
                                                          0.0) };

    state.engineRpm = engOut.rpm;
    state.engineTorque = engOut.torque;
    state.clutchTorque = clutchState.transferredTorque;
    state.propTorque = transOut.propTorque;
    state.leftWheelTorque = diffOut.leftTorque;
    state.rightWheelTorque = diffOut.rightTorque;
    state.leftWheelSpeed = leftWheelSpeed;
    state.rightWheelSpeed = rightWheelSpeed;
    state.activeGear = transOut.activeGear;
    state.isShifting = transmission_.isShifting();
    state.isRedline = engOut.isRedline;
    state.isStalled = engOut.isStalled;
    state.isClutchLocked = clutchState.isLocked;

    lastState_ = state;
    return state;
}

void Powertrain2D::reset() noexcept {
    if (engine_) {
        engine_->reset();
    }
    clutch_.reset();
    transmission_.reset(1);
    lastState_ = PowertrainState {};
}

bool Powertrain2D::shiftUp() noexcept {
    return transmission_.shiftUp();
}

bool Powertrain2D::shiftDown() noexcept {
    return transmission_.shiftDown();
}

bool Powertrain2D::selectGear(std::int32_t gear) noexcept {
    return transmission_.selectGear(gear);
}

void Powertrain2D::setEngine(std::unique_ptr<IEngine2D> newEngine) noexcept {
    if (newEngine) {
        engine_ = std::move(newEngine);
    }
}

IEngine2D& Powertrain2D::getEngine() noexcept {
    return *engine_;
}

Transmission2D& Powertrain2D::getTransmission() noexcept {
    return transmission_;
}

Differential2D& Powertrain2D::getDifferential() noexcept {
    return differential_;
}

const PowertrainState& Powertrain2D::getState() const noexcept {
    return lastState_;
}

DrivetrainLayout Powertrain2D::getLayout() const noexcept {
    return config_.layout;
}

void Powertrain2D::setLayout(DrivetrainLayout layout) noexcept {
    config_.layout = layout;
}

} // namespace vehicle_sim::dynamics
