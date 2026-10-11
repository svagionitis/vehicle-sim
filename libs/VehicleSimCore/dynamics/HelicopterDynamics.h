#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct HelicopterConfig
/// @brief Configuration parameters for 2D VTOL helicopter and rotorcraft dynamics.
/// @details Defines structural mass, main rotor aerodynamics, collective thrust limits,
/// cyclic pitch authority, tail rotor anti-torque capacity, in-ground effect, and autorotation.
struct HelicopterConfig {
    double mass { 2200.0 }; ///< Helicopter structural and fuel mass in kg.
    double rotorRadius { 5.2 }; ///< Main rotor disc radius in meters.
    double maxRotorRpm { 420.0 }; ///< Nominal operating main rotor speed in RPM.
    double rotorSpoolRate { 120.0 }; ///< Rotor acceleration spool rate in RPM/s.

    double maxCollectiveThrust { 32000.0 }; ///< Peak main rotor vertical lift capacity in Newtons.
    double cyclicPitchAuthority { 8500.0 }; ///< Fore/aft cyclic stick pitch tilt torque in N*m.
    double tailRotorMaxTorque { 4500.0 }; ///< Anti-torque tail rotor maximum authority in N*m.
    double mainReactionTorque { 3200.0 }; ///< Reactive counter-torque from main rotor rotation in N*m.

    double inGroundEffectHeight { 6.0 }; ///< Ground clearance ceiling for In-Ground Effect (IGE) in meters.
    double igeMaxBoostRatio { 0.25 }; ///< Peak IGE thrust augmentation ratio at surface.
    double etlSpeedThreshold { 8.0 }; ///< Effective Translational Lift (ETL) onset airspeed in m/s.

    double fuselageDrag { 16.0 }; ///< Fuselage parasite aerodynamic drag coefficient in N/(m/s)^2.
    double angularDamping { 2800.0 }; ///< Aerodynamic fuselage yaw/pitch damping moment in N*m*s.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity acceleration vector in m/s^2.
};

/// @struct HelicopterInput
/// @brief Pilot flight control commands for helicopter dynamics.
struct HelicopterInput {
    double collective { 0.0 }; ///< Main rotor collective pitch lift command in [0.0, 1.0].
    double cyclicPitch { 0.0 }; ///< Fore/aft cyclic stick command in [-1.0, 1.0] (+1 nose down forward).
    double rudder { 0.0 }; ///< Anti-torque tail rotor pedal command in [-1.0, 1.0].
    bool enginePower { true }; ///< Turboshaft engine master ignition flag (false = autorotation).
    double groundDistance { 10.0 }; ///< Radar altimeter terrain clearance in meters.
};

/// @struct HelicopterTelemetry
/// @brief Diagnostic snapshot of internal helicopter rotorcraft dynamics states.
struct HelicopterTelemetry {
    double rotorSpeed { 0.0 }; ///< Current main rotor rotational speed in RPM.
    double rotorThrust { 0.0 }; ///< Current main rotor total lift thrust in Newtons.
    double tailTorque { 0.0 }; ///< Current anti-torque tail rotor moment in N*m.
    double forwardSpeed { 0.0 }; ///< Forward longitudinal flight speed in m/s.
    double climbRate { 0.0 }; ///< Vertical climb or descent velocity in m/s.
    bool isGroundEffect { false }; ///< True if within In-Ground Effect (IGE) rotor cushion.
    bool isTranslationalLift { false }; ///< True if above Effective Translational Lift (ETL) speed.
    bool isAutorotating { false }; ///< True if descending without engine power under autorotation.
    double altitude { 0.0 }; ///< Current terrain clearance altitude in meters.
};

/// @class HelicopterDynamics
/// @brief 2D VTOL helicopter rotorcraft dynamics coordinator.
/// @details Models main rotor collective lift, fore/aft cyclic pitch tilting, tail rotor
/// anti-torque compensation, ground effect (IGE) cushion, translational lift (ETL), and autorotation.
class HelicopterDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline helicopter configuration.
    HelicopterDynamics() noexcept;

    /// @brief Parameterized constructor with custom helicopter configuration.
    /// @param[in] config Helicopter configuration parameters.
    explicit HelicopterDynamics(const HelicopterConfig& config) noexcept;

    /// @brief Advances helicopter simulation step by timestep dt.
    /// @details Computes rotor RPM, collective thrust with IGE/ETL augmentation, cyclic tilt,
    /// tail rotor anti-torque, and applies accumulated forces to rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and spool state.
    void reset() noexcept override;

    /// @brief Queries whether the helicopter is airborne.
    /// @return True if ground clearance exceeds minimum landing gear contact height.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active helicopter configuration.
    /// @return Const reference to active HelicopterConfig.
    [[nodiscard]] const HelicopterConfig& getConfig() const noexcept;

    /// @brief Updates active helicopter configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const HelicopterConfig& config) noexcept;

    /// @brief Gets current pilot control inputs.
    /// @return Const reference to active HelicopterInput.
    [[nodiscard]] const HelicopterInput& getInput() const noexcept;

    /// @brief Sets pilot flight control inputs.
    /// @param[in] input New pilot control commands.
    void setInput(const HelicopterInput& input) noexcept;

    /// @brief Gets diagnostic telemetry snapshot.
    /// @return Const reference to HelicopterTelemetry.
    [[nodiscard]] const HelicopterTelemetry& getTelemetry() const noexcept;

    /// @brief Queries whether the helicopter is descending in autorotation mode.
    /// @return True if engine is off and rotor is windmilling from upward airflow.
    [[nodiscard]] bool isAutorotating() const noexcept;

    /// @brief Gets current main rotor rotational speed.
    /// @return Rotor speed in RPM.
    [[nodiscard]] double getRotorSpeed() const noexcept;

    /// @brief Gets forward longitudinal airspeed.
    /// @return Speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    HelicopterConfig mConfig { };
    HelicopterInput mInput { };
    HelicopterTelemetry mTelemetry { };
    double mRotorRpm { 0.0 };
};

} // namespace vehicle_sim::dynamics
