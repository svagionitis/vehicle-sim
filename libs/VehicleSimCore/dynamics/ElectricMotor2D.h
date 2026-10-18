#pragma once

#include "libs/VehicleSimCore/dynamics/IEngine2D.h"

namespace vehicle_sim::dynamics {

/// @struct ElectricMotorConfig
/// @brief Electromagnetic and inverter parameters for an electric traction motor.
struct ElectricMotorConfig {
    double maxTorque { 600.0 }; ///< Maximum constant torque in N*m.
    double baseSpeedRpm { 4500.0 }; ///< Base speed transitioning to constant power in RPM.
    double maxSpeedRpm { 16000.0 }; ///< Maximum operating rotor speed in RPM.
    double maxPower { 280000.0 }; ///< Maximum continuous power in Watts (280 kW).
    double maxRegenTorque { 350.0 }; ///< Maximum regenerative braking counter-torque in N*m.
    double regenEfficiency { 0.88 }; ///< Kinetic to electrical recovery efficiency in [0..1].
    double electricalLag { 0.012 }; ///< Inverter current response lag in seconds.
    double rotorInertia { 0.065 }; ///< Rotor moment of inertia in kg*m^2.
};

/// @class ElectricMotor2D
/// @brief Simulation of permanent-magnet or AC induction vehicle traction motor.
class ElectricMotor2D final : public IEngine2D {
public:
    /// @brief Default constructor using baseline EV parameters.
    ElectricMotor2D() noexcept;

    /// @brief Parameterized constructor using custom EV motor configuration.
    /// @param[in] config Motor electromagnetic and inverter parameters.
    explicit ElectricMotor2D(const ElectricMotorConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~ElectricMotor2D() noexcept override = default;

    /// @brief Computes generated torque and advances internal motor state.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    /// @param[in] shaftSpeed Rotor rotational speed in rad/s.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated engine output state.
    [[nodiscard]] EngineOutput step(double throttle, double shaftSpeed, double dt) noexcept override;

    /// @brief Resets motor state.
    void reset() noexcept override;

    /// @brief Enables the motor inverter.
    void start() noexcept override;

    /// @brief Disables the motor inverter.
    void stop() noexcept override;

    /// @brief Queries engine technology classification.
    /// @return EngineType::ElectricMotor.
    [[nodiscard]] EngineType getEngineType() const noexcept override;

    /// @brief Queries whether inverter is enabled.
    /// @return True if inverter is enabled.
    [[nodiscard]] bool isRunning() const noexcept override;

    /// @brief Queries maximum rated torque.
    /// @return Peak stall torque in N*m.
    [[nodiscard]] double getPeakTorque() const noexcept override;

    /// @brief Queries maximum rated speed.
    /// @return Redline speed in RPM.
    [[nodiscard]] double getRedlineRpm() const noexcept override;

    /// @brief Computes maximum motor torque available at given speed.
    /// @param[in] rpm Rotor speed in revolutions per minute.
    /// @return Maximum forward motor torque in N*m.
    [[nodiscard]] double computeMaxTorque(double rpm) const noexcept;

    /// @brief Computes available regenerative braking torque.
    /// @param[in] rpm Rotor speed in revolutions per minute.
    /// @param[in] brakeCmd Driver brake pedal demand [0, 1].
    /// @return Regenerative counter-torque in N*m.
    [[nodiscard]] double computeRegen(double rpm, double brakeCmd) const noexcept;

    /// @brief Queries current filtered motor torque in N*m.
    /// @return Motor torque in N*m.
    [[nodiscard]] double getCurrentTorque() const noexcept;

    /// @brief Queries current rotor speed in rad/s.
    /// @return Rotor speed in rad/s.
    [[nodiscard]] double getCurrentSpeed() const noexcept;

private:
    ElectricMotorConfig config_ {};
    double currentTorque_ { 0.0 };
    double currentSpeed_ { 0.0 };
    double currentRpm_ { 0.0 };
    bool isEnabled_ { true };
};

} // namespace vehicle_sim::dynamics
