#pragma once

#include <array>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/PIDController.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct QuadcopterConfig
/// @brief Physical, propulsion, and controller parameters for QuadcopterDroneDynamics.
struct QuadcopterConfig {
    double mass { 1.5 }; ///< All-up drone mass including battery in kg.
    double inertia { 0.025 }; ///< Pitch moment of inertia in kg*m^2.
    double armLength { 0.225 }; ///< Distance d from COG to each rotor hub in meters.
    double thrustCoeff { 2.9e-5 }; ///< Rotor thrust coefficient k_t in N/(rad/s)^2.
    double torqueCoeff { 1.1e-6 }; ///< Rotor aerodynamic drag torque coefficient k_q in N*m/(rad/s)^2.
    double maxMotorSpeed { 2400.0 }; ///< Maximum rotor speed in rad/s (~23,000 RPM).
    double motorTimeConstant { 0.02 }; ///< ESC motor spool-up time constant tau in seconds.
    double linearDragCoeff { 0.18 }; ///< Translational aerodynamic drag coefficient in N/(m/s)^2.
    double angularDamping { 0.08 }; ///< Aerodynamic pitch angular rate damping in N*m*s/rad.
    math::PIDGains pitchGains { 6.0, 0.2, 0.45 }; ///< Proportional, integral, derivative pitch gains.
};

/// @class QuadcopterDroneDynamics
/// @brief 4-rotor aerial drone dynamics coordinator.
/// @details Simulates an 'X' frame multi-rotor quadcopter with differential thrust mixing,
/// ESC spool-up time constants, gyroscopic anti-torque cancellation, and closed-loop pitch PID stabilization.
class QuadcopterDroneDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor with baseline racing drone parameters.
    QuadcopterDroneDynamics() noexcept;

    /// @brief Parameterized constructor using custom drone configuration.
    /// @param[in] config Drone physical dimensions, motor constants, and PID gains.
    explicit QuadcopterDroneDynamics(const QuadcopterConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~QuadcopterDroneDynamics() noexcept override = default;

    /// @brief Advances simulation step, applying thrust and aerodynamic forces.
    /// @param[in,out] body Target rigid body representing the drone frame.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets PID integrators, motor speeds, and target commands.
    void reset() noexcept override;

    /// @brief Queries whether the drone is currently airborne.
    /// @return True if drone is armed and producing thrust above ground.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets normalized collective throttle input.
    /// @param[in] throttle Normalized throttle in range [0.0, 1.0].
    void setThrottle(double throttle) noexcept;

    /// @brief Sets commanded target pitch angle in radians.
    /// @param[in] pitch Target pitch angle in radians (negative = pitch forward).
    void setPitchAngle(double pitch) noexcept;

    /// @brief Sets motor arming state.
    /// @param[in] armed True to arm motors, false to disarm and shut down thrust.
    void setArmed(bool armed) noexcept;

    /// @brief Retrieves current angular velocities for all 4 motors.
    /// @return Array of motor speeds in rad/s [FL, FR, RL, RR].
    [[nodiscard]] const std::array<double, 4>& getMotorSpeeds() const noexcept;

    /// @brief Retrieves total current aerodynamic thrust in Newtons.
    /// @return Total upward thrust magnitude in Newtons.
    [[nodiscard]] double getTotalThrust() const noexcept;

    /// @brief Queries net yaw reactionary torque in N*m.
    /// @return Reactionary torque scalar in N*m.
    [[nodiscard]] double getYawTorque() const noexcept;

    /// @brief Queries active vehicle configuration.
    /// @return Const reference to QuadcopterConfig.
    [[nodiscard]] const QuadcopterConfig& getConfig() const noexcept;

private:
    QuadcopterConfig mConfig {};
    math::PIDController mPitchController {};
    std::array<double, 4> mMotorSpeeds {};
    std::array<double, 4> mTargetSpeeds {};

    double mThrottle { 0.0 };
    double mTargetPitch { 0.0 };
    double mTotalThrust { 0.0 };
    double mYawTorque { 0.0 };
    bool mArmed { false };
    bool mIsAirborne { false };

    void updateMotorSpeeds(double dt) noexcept;
    void evaluateMixer(double collectiveThrottle, double pitchCommand) noexcept;
};

} // namespace vehicle_sim::dynamics
