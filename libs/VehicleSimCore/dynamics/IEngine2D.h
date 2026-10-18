#pragma once

#include <cstdint>

namespace vehicle_sim::dynamics {

/// @enum EngineType
/// @brief Prime mover propulsion technology classification.
enum class EngineType : std::uint8_t {
    InternalCombustion = 0U, ///< Fuel-burning piston or rotary engine.
    ElectricMotor = 1U       ///< Pure electric inverter-driven motor.
};

/// @struct EngineOutput
/// @brief Dynamic instantaneous output metrics from a prime mover.
struct EngineOutput {
    double torque { 0.0 }; ///< Crankshaft / rotor output torque in N*m.
    double power { 0.0 }; ///< Mechanical power in Watts.
    double angularVelocity { 0.0 }; ///< Shaft speed in rad/s.
    double rpm { 0.0 }; ///< Shaft speed in revolutions per minute.
    double throttleLagged { 0.0 }; ///< Effective internal throttle position in [0, 1].
    bool isRedline { false }; ///< True if engine is at or above rev limit.
    bool isStalled { false }; ///< True if engine has dropped below stall threshold.
};

/// @class IEngine2D
/// @brief Abstract interface for vehicle propulsion prime movers.
class IEngine2D {
public:
    virtual ~IEngine2D() noexcept = default;

    /// @brief Computes generated torque and advances internal engine state.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    /// @param[in] shaftSpeed Shaft rotational speed in rad/s.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated engine output state.
    [[nodiscard]] virtual EngineOutput step(double throttle, double shaftSpeed, double dt) noexcept = 0;

    /// @brief Resets engine states to initial idle conditions.
    virtual void reset() noexcept = 0;

    /// @brief Starts the engine from a resting or stalled state.
    virtual void start() noexcept = 0;

    /// @brief Cuts ignition or disables inverter.
    virtual void stop() noexcept = 0;

    /// @brief Queries engine technology classification.
    /// @return EngineType classification enum.
    [[nodiscard]] virtual EngineType getEngineType() const noexcept = 0;

    /// @brief Queries whether engine is currently running.
    /// @return True if running and not stalled.
    [[nodiscard]] virtual bool isRunning() const noexcept = 0;

    /// @brief Queries maximum rated torque.
    /// @return Peak torque in N*m.
    [[nodiscard]] virtual double getPeakTorque() const noexcept = 0;

    /// @brief Queries maximum rated speed.
    /// @return Redline speed in RPM.
    [[nodiscard]] virtual double getRedlineRpm() const noexcept = 0;
};

} // namespace vehicle_sim::dynamics
