#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct RocketConfig
/// @brief Configuration parameters for 2D rocket and space vehicle dynamics.
/// @details Defines structural mass, initial propellant capacity, geometry dimensions,
/// thruster capacity, gimbal bounds, RCS performance, and environmental constants.
struct RocketConfig {
    double dryMass { 1500.0 }; ///< Structural dry mass in kg.
    double initialFuelMass { 3500.0 }; ///< Initial propellant fuel mass in kg.
    double length { 6.0 }; ///< Total vehicle length in meters.
    double radius { 0.8 }; ///< Cylindrical hull radius in meters.

    double maxThrust { 65000.0 }; ///< Maximum main engine thrust in Newtons.
    double specificImpulse { 310.0 }; ///< Specific impulse (Isp) in seconds.
    double standardGravity { 9.80665 }; ///< Standard acceleration of gravity g0 in m/s^2.

    double maxGimbalAngle { 0.174533 }; ///< Maximum gimbal deflection angle in radians (~10 deg).
    double gimbalSpeed { 1.5 }; ///< Gimbal deflection slew rate in rad/s.
    math::Vec2 thrusterOffset { -3.0, 0.0 }; ///< Nozzle mount location in local chassis coordinates.

    double rcsTorque { 1500.0 }; ///< Attitude thruster pair maximum rotational torque in N*m.
    double rcsThrust { 800.0 }; ///< Translational RCS thruster force capacity in Newtons.
    double rcsFuelRate { 0.5 }; ///< RCS propellant consumption rate at full duty in kg/s.
    double sasDamping { 600.0 }; ///< SAS active rate damping coefficient in N*m*s.

    double dragCoefficient { 0.3 }; ///< Aerodynamic drag coefficient (Cd).
    double crossSectionArea { 2.0 }; ///< Frontal aerodynamic reference area in m^2.
    double atmosphericDensity { 0.0 }; ///< Ambient atmospheric density in kg/m^3 (0.0 in space vacuum).
    math::Vec2 gravity { 0.0, 0.0 }; ///< Ambient gravity vector in m/s^2 (0.0 in deep space).
};

/// @struct RocketInput
/// @brief Semantic pilot control commands for rocket dynamics.
struct RocketInput {
    double throttle { 0.0 }; ///< Main engine throttle in [0.0, 1.0].
    double gimbal { 0.0 }; ///< Commanded gimbal deflection in [-1.0, 1.0].
    double rcs { 0.0 }; ///< Attitude rotational RCS command in [-1.0, 1.0].
    math::Vec2 rcsTranslate { 0.0, 0.0 }; ///< Translational RCS command in [-1.0, 1.0] per axis.
    bool sasEnabled { true }; ///< Stability Augmentation System active rate damping flag.
};

/// @struct RocketTelemetry
/// @brief Diagnostic snapshot of internal rocket dynamics and propulsion states.
struct RocketTelemetry {
    double fuelMass { 0.0 }; ///< Remaining propellant fuel mass in kg.
    double totalMass { 0.0 }; ///< Current vehicle mass (dry mass + fuel) in kg.
    double inertia { 0.0 }; ///< Current scalar moment of inertia in kg*m^2.
    double currentThrust { 0.0 }; ///< Current main engine thrust output in Newtons.
    double gimbalAngle { 0.0 }; ///< Current main thruster gimbal angle in radians.
    double massFlowRate { 0.0 }; ///< Current propellant burn rate in kg/s.
    double rcsTorque { 0.0 }; ///< Current RCS rotational torque output in N*m.
    math::Vec2 rcsForce { 0.0, 0.0 }; ///< Current translational RCS force in Newtons.
    math::Vec2 aeroDrag { 0.0, 0.0 }; ///< Aerodynamic drag force vector in Newtons.
    bool isFlameout { false }; ///< True if propellant is depleted and engine cannot burn.
};

/// @class RocketDynamics
/// @brief 2D rocket and spacecraft dynamics coordinator.
/// @details Simulates main engine thrust vectoring with gimbal angle, offset thrust point
/// generating angular torque, RCS attitude control thrusters, mass depletion over time
/// with variable moment of inertia, and environmental drag/gravity.
class RocketDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline rocket configuration.
    RocketDynamics() noexcept;

    /// @brief Parameterized constructor with custom rocket configuration.
    /// @param[in] config Rocket configuration parameters.
    explicit RocketDynamics(const RocketConfig& config) noexcept;

    /// @brief Advances rocket simulation by timestep dt.
    /// @details Depletes propellant, updates body mass and inertia, computes vector thrust,
    /// RCS attitude torque, SAS stabilization, and applies accumulated forces.
    /// @param[in,out] body Target rigid body receiving forces and updated inertial state.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control and flight states.
    /// @details Clears gimbal deflection and pilot inputs without altering remaining fuel.
    void reset() noexcept override;

    /// @brief Queries whether the rocket is airborne or in space flight.
    /// @return Always true for rocket flight.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active rocket configuration.
    /// @return Const reference to active RocketConfig.
    [[nodiscard]] const RocketConfig& getConfig() const noexcept;

    /// @brief Sets rocket configuration parameters.
    /// @param[in] config New configuration parameters.
    void setConfig(const RocketConfig& config) noexcept;

    /// @brief Gets active pilot control inputs.
    /// @return Const reference to current RocketInput.
    [[nodiscard]] const RocketInput& getInput() const noexcept;

    /// @brief Sets pilot control inputs.
    /// @param[in] input New pilot commands.
    void setInput(const RocketInput& input) noexcept;

    /// @brief Gets real-time diagnostic telemetry.
    /// @return Const reference to latest RocketTelemetry snapshot.
    [[nodiscard]] const RocketTelemetry& getTelemetry() const noexcept;

    /// @brief Gets remaining propellant mass in kg.
    /// @return Fuel mass in kilograms.
    [[nodiscard]] double getFuelMass() const noexcept;

    /// @brief Manually sets remaining propellant mass in kg.
    /// @param[in] massKg Non-negative propellant mass.
    void setFuelMass(double massKg) noexcept;

    /// @brief Gets structural dry mass in kg.
    /// @return Dry mass in kilograms.
    [[nodiscard]] double getDryMass() const noexcept;

    /// @brief Gets current combined total mass in kg.
    /// @return Total mass in kilograms.
    [[nodiscard]] double getTotalMass() const noexcept;

    /// @brief Gets current dynamic moment of inertia in kg*m^2.
    /// @return Rotational inertia.
    [[nodiscard]] double getInertia() const noexcept;

    /// @brief Gets normalized remaining fuel ratio in [0.0, 1.0].
    /// @return Fraction of fuel remaining.
    [[nodiscard]] double getFuelRatio() const noexcept;

    /// @brief Queries whether propellant tank is empty.
    /// @return True if fuel mass is zero.
    [[nodiscard]] bool isFuelEmpty() const noexcept;

    /// @brief Gets current main engine gimbal deflection in radians.
    /// @return Gimbal angle in radians.
    [[nodiscard]] double getGimbalAngle() const noexcept;

    /// @brief Sets main engine gimbal angle in radians.
    /// @param[in] angleRad Gimbal deflection angle in radians.
    void setGimbalAngle(double angleRad) noexcept;

    /// @brief Gets current main engine thrust force in Newtons.
    /// @return Thrust force scalar.
    [[nodiscard]] double getThrustForce() const noexcept;

    /// @brief Queries whether Stability Augmentation System is active.
    /// @return True if SAS is enabled.
    [[nodiscard]] bool isSasEnabled() const noexcept;

    /// @brief Enables or disables Stability Augmentation System.
    /// @param[in] enabled SAS activation state.
    void setSasEnabled(bool enabled) noexcept;

    /// @brief Refuels propellant tank to initial capacity.
    void refuel() noexcept;

private:
    /// @brief Computes scalar rotational inertia from mass and dimensions.
    /// @param[in] currentMass Current total vehicle mass in kg.
    /// @return Moment of inertia in kg*m^2.
    [[nodiscard]] double computeInertia(double currentMass) const noexcept;

    RocketConfig mConfig { };
    RocketInput mInput { };
    RocketTelemetry mTelemetry { };

    double mFuelMass { 3500.0 };
    double mGimbalAngle { 0.0 };
};

} // namespace vehicle_sim::dynamics
