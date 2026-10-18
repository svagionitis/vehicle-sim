#pragma once

#include <array>
#include <cstddef>

#include "libs/VehicleSimCore/dynamics/IEngine2D.h"

namespace vehicle_sim::dynamics {

/// @struct ICEConfig
/// @brief Physical, thermodynamic, and mechanical parameters for an ICE.
struct ICEConfig {
    double peakTorque { 450.0 }; ///< Peak engine torque in N*m.
    double peakTorqueRpm { 4800.0 }; ///< RPM at which peak torque occurs.
    double idleRpm { 900.0 }; ///< Target idle governor speed in RPM.
    double stallRpm { 450.0 }; ///< Engine stall threshold in RPM.
    double redlineRpm { 7200.0 }; ///< Fuel-cut rev limit in RPM.
    double softCutRpm { 7000.0 }; ///< Soft limiter onset in RPM.
    double hysteresisRpm { 200.0 }; ///< Fuel re-engagement hysteresis in RPM.
    double inertia { 0.22 }; ///< Flywheel and crankshaft rotational inertia in kg*m^2.
    double intakeLag { 0.08 }; ///< Throttle response lag time constant in seconds.
    double frictionCoeff0 { 15.0 }; ///< Coulomb engine friction in N*m.
    double frictionCoeff1 { 0.035 }; ///< Viscous friction in N*m*s/rad.
    double frictionCoeff2 { 0.0001 }; ///< Aerodynamic pumping loss in N*m*s^2/rad^2.
    double idleKp { 0.4 }; ///< Idle governor proportional gain.
    double idleKi { 0.15 }; ///< Idle governor integral gain.
    std::array<double, 4> torquePoly { 0.35, 1.30, -0.65, 0.00 }; ///< Normalized cubic curve.
};

/// @class InternalCombustionEngine2D
/// @brief Detailed 2D simulation of a piston or rotary combustion engine.
class InternalCombustionEngine2D final : public IEngine2D {
public:
    /// @brief Default constructor using baseline parameters.
    InternalCombustionEngine2D() noexcept;

    /// @brief Parameterized constructor using custom ICE configuration.
    /// @param[in] config Engine physical and torque parameters.
    explicit InternalCombustionEngine2D(const ICEConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~InternalCombustionEngine2D() noexcept override = default;

    /// @brief Computes generated torque and advances internal engine state.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    /// @param[in] shaftSpeed Shaft rotational speed in rad/s.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated engine output state.
    [[nodiscard]] EngineOutput step(double throttle, double shaftSpeed, double dt) noexcept override;

    /// @brief Resets engine states to initial idle conditions.
    void reset() noexcept override;

    /// @brief Starts the engine from a resting or stalled state.
    void start() noexcept override;

    /// @brief Cuts ignition to stop the engine.
    void stop() noexcept override;

    /// @brief Queries engine technology classification.
    /// @return EngineType::InternalCombustion.
    [[nodiscard]] EngineType getEngineType() const noexcept override;

    /// @brief Queries whether engine is currently running.
    /// @return True if running and not stalled.
    [[nodiscard]] bool isRunning() const noexcept override;

    /// @brief Queries maximum rated torque.
    /// @return Peak torque in N*m.
    [[nodiscard]] double getPeakTorque() const noexcept override;

    /// @brief Queries maximum rated speed.
    /// @return Redline speed in RPM.
    [[nodiscard]] double getRedlineRpm() const noexcept override;

    /// @brief Computes wide-open throttle indicated torque at a given RPM.
    /// @param[in] rpm Engine speed in revolutions per minute.
    /// @return Gross combustion torque in N*m.
    [[nodiscard]] double computeWotTorque(double rpm) const noexcept;

    /// @brief Computes mechanical and pumping friction losses at a given speed.
    /// @param[in] angVel Angular velocity in rad/s.
    /// @return Resisting friction torque in N*m.
    [[nodiscard]] double computeFriction(double angVel) const noexcept;

    /// @brief Queries current engine crankshaft speed in RPM.
    /// @return Shaft speed in RPM.
    [[nodiscard]] double getCurrentRpm() const noexcept;

    /// @brief Queries current engine angular velocity in rad/s.
    /// @return Shaft speed in rad/s.
    [[nodiscard]] double getCurrentAngVel() const noexcept;

    /// @brief Queries whether engine is stalled.
    /// @return True if stalled.
    [[nodiscard]] bool isStalled() const noexcept;

    /// @brief Queries whether rev-limiter fuel cut is active.
    /// @return True if fuel cut is active.
    [[nodiscard]] bool isFuelCut() const noexcept;

private:
    ICEConfig config_ {};
    double currentRpm_ { 900.0 };
    double currentAngVel_ { 94.24777960769379 };
    double effectiveThrottle_ { 0.0 };
    double idleIntegral_ { 0.0 };
    bool isRunning_ { true };
    bool isStalled_ { false };
    bool isFuelCut_ { false };
};

} // namespace vehicle_sim::dynamics
