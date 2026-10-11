#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct AeroPlaneConfig
/// @brief Configuration parameters for 2D fixed-wing aircraft aerodynamic flight dynamics.
/// @details Defines wing area, wingspan, air density, propulsion capacity, lift slope,
/// parasitic and induced drag coefficients, stall angle, control authority, and environmental gravity.
struct AeroPlaneConfig {
    double mass { 1800.0 }; ///< Aircraft structural and payload mass in kg.
    double wingArea { 16.0 }; ///< Wing planform reference surface area in m^2.
    double wingspan { 10.0 }; ///< Wing lateral span tip-to-tip in meters.
    double airDensity { 1.225 }; ///< Ambient atmospheric air density in kg/m^3 (sea level standard).
    double maxThrust { 14000.0 }; ///< Maximum propeller or jet forward thrust in Newtons.

    double liftSlope { 5.5 }; ///< Lift curve slope dCL/dAlpha in radians^-1.
    double zeroLiftDrag { 0.025 }; ///< Parasitic zero-lift drag coefficient (CD0).
    double inducedDragFactor { 0.045 }; ///< Induced drag factor k = 1 / (pi * e * AR).
    double stallAngle { 0.28 }; ///< Critical stall angle of attack in radians (~16 degrees).

    double flapLiftCoeff { 0.5 }; ///< Additional lift coefficient contribution from full flaps.
    double flapDragCoeff { 0.04 }; ///< Additional drag coefficient contribution from full flaps.
    double brakeDragCoeff { 0.08 }; ///< Additional drag coefficient contribution from airbrakes.

    double pitchAuthority { 6000.0 }; ///< Maximum elevator pitch torque authority in N*m.
    double pitchDamping { 1500.0 }; ///< Aerodynamic pitch rate damping moment in N*m*s.
    double weathercockStability { 8000.0 }; ///< Aerodynamic restoring moment against alpha in N*m/rad.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity acceleration vector in m/s^2.
};

/// @struct AeroPlaneInput
/// @brief Pilot flight control commands for fixed-wing aircraft dynamics.
struct AeroPlaneInput {
    double throttle { 0.0 }; ///< Engine forward thrust throttle in [0.0, 1.0].
    double elevator { 0.0 }; ///< Pitch control elevator deflection in [-1.0, 1.0] (+1 pitch up).
    double flaps { 0.0 }; ///< Trailing-edge high-lift flap deployment in [0.0, 1.0].
    double airbrake { 0.0 }; ///< Deployable aerodynamic airbrake command in [0.0, 1.0].
};

/// @struct AeroPlaneTelemetry
/// @brief Diagnostic snapshot of aerodynamic flight performance and airflow states.
struct AeroPlaneTelemetry {
    double airspeed { 0.0 }; ///< True airspeed relative to ambient air mass in m/s.
    double dynamicPressure { 0.0 }; ///< Flight dynamic pressure q = 0.5 * rho * V^2 in Pascals.
    double angleOfAttack { 0.0 }; ///< Angle of attack alpha in radians.
    double liftForce { 0.0 }; ///< Total aerodynamic lift force magnitude in Newtons.
    double dragForce { 0.0 }; ///< Total aerodynamic drag force magnitude in Newtons.
    double thrustForce { 0.0 }; ///< Propulsive forward thrust force in Newtons.
    double liftCoeff { 0.0 }; ///< Active dimensionless lift coefficient (CL).
    double dragCoeff { 0.0 }; ///< Active dimensionless drag coefficient (CD).
    bool isStalled { false }; ///< True if angle of attack exceeds critical stall boundary.
    double climbRate { 0.0 }; ///< Vertical climb or descent velocity in m/s.
    double forwardVelocity { 0.0 }; ///< Axial forward velocity along fuselage in m/s.
};

/// @class AerodynamicPlaneDynamics
/// @brief 2D fixed-wing aerodynamic flight dynamics coordinator.
/// @details Models true airspeed, dynamic pressure, angle of attack, lift/drag polar curves,
/// dynamic aerodynamic stalls, elevator control authority, weathercock stability, and gravity.
class AerodynamicPlaneDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline aircraft configuration.
    AerodynamicPlaneDynamics() noexcept;

    /// @brief Parameterized constructor with custom aircraft configuration.
    /// @param[in] config Aircraft flight configuration parameters.
    explicit AerodynamicPlaneDynamics(const AeroPlaneConfig& config) noexcept;

    /// @brief Advances aerodynamic simulation step by timestep dt.
    /// @details Computes airspeed, angle of attack, lift, drag, thrust, pitch authority,
    /// and applies accumulated aerodynamic forces and moments to target rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and flight telemetry states.
    void reset() noexcept override;

    /// @brief Queries whether the aircraft is airborne.
    /// @return Always true for active aerodynamic flight simulation.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active aircraft configuration.
    /// @return Const reference to active AeroPlaneConfig.
    [[nodiscard]] const AeroPlaneConfig& getConfig() const noexcept;

    /// @brief Updates active aircraft configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const AeroPlaneConfig& config) noexcept;

    /// @brief Gets current pilot control inputs.
    /// @return Const reference to active AeroPlaneInput.
    [[nodiscard]] const AeroPlaneInput& getInput() const noexcept;

    /// @brief Sets pilot flight control inputs.
    /// @param[in] input New pilot control commands.
    void setInput(const AeroPlaneInput& input) noexcept;

    /// @brief Gets diagnostic flight telemetry snapshot.
    /// @return Const reference to AeroPlaneTelemetry.
    [[nodiscard]] const AeroPlaneTelemetry& getTelemetry() const noexcept;

    /// @brief Gets true airspeed.
    /// @return Airspeed scalar magnitude in m/s.
    [[nodiscard]] double getAirspeed() const noexcept;

    /// @brief Gets current aerodynamic angle of attack.
    /// @return Angle of attack in radians.
    [[nodiscard]] double getAngleOfAttack() const noexcept;

    /// @brief Queries whether the aircraft wings are aerodynamically stalled.
    /// @return True if in post-stall flow separation regime.
    [[nodiscard]] bool isStalled() const noexcept;

    /// @brief Gets forward longitudinal velocity along fuselage axis.
    /// @return Forward speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    AeroPlaneConfig mConfig { };
    AeroPlaneInput mInput { };
    AeroPlaneTelemetry mTelemetry { };
};

} // namespace vehicle_sim::dynamics
