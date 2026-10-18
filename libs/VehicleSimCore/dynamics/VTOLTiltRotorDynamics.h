#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/MathUtils.h"
#include "libs/VehicleSimCore/math/SlewRateLimiter.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum VTOLFlightMode
/// @brief Current operational flight mode based on nacelle conversion angle.
enum class VTOLFlightMode : unsigned char {
    Hover = 0U,         ///< Nacelles vertical (75 deg to 90 deg); pure rotor lift.
    ConversionForward,  ///< Nacelles transitioning from hover to airplane cruise.
    ConversionAft,      ///< Nacelles transitioning from airplane cruise to hover.
    AirplaneCruise      ///< Nacelles horizontal (0 deg to 15 deg); fixed-wing flight.
};

/// @struct VTOLTiltRotorConfig
/// @brief Configuration parameters for 2D tilt-rotor VTOL aircraft dynamics.
/// @details Defines structural mass, wing geometry, proprotor performance,
/// nacelle conversion slew rates, corridor safety boundaries, and control authority.
struct VTOLTiltRotorConfig {
    double mass { 6800.0 }; ///< Aircraft gross takeoff mass in kg.
    double wingArea { 32.0 }; ///< Wing planform reference surface area in m^2.
    double wingspan { 15.0 }; ///< Wing lateral span tip-to-tip in meters.
    double airDensity { 1.225 }; ///< Ambient atmospheric air density in kg/m^3.

    // Proprotor propulsion parameters
    double maxTotalThrust { 85000.0 }; ///< Total peak proprotor thrust capacity in Newtons.
    double ratedRotorRpm { 450.0 }; ///< Nominal operating rotor rotational speed in RPM.
    double rotorSpoolRate { 150.0 }; ///< Rotor acceleration spool rate in RPM/s.
    double proprotorRadius { 3.5 }; ///< Proprotor blade disc radius in meters.

    // Nacelle tilt kinematics
    double maxTiltSlewRate { 0.14 }; ///< Maximum nacelle conversion rate in rad/s (~8.0 deg/s).
    double minNacelleAngle { 0.0 }; ///< Minimum cruise nacelle angle in radians (0.0 rad = 0 deg).
    double maxNacelleAngle { 1.5707963 }; ///< Maximum hover nacelle angle in radians (pi/2 = 90 deg).

    // Aerodynamic wing parameters
    double liftSlope { 5.2 }; ///< Lift curve slope dCL/dAlpha in rad^-1.
    double zeroLiftDrag { 0.028 }; ///< Parasitic zero-lift drag coefficient (CD0).
    double inducedDragFactor { 0.048 }; ///< Induced drag factor k = 1 / (pi * e * AR).
    double stallAngle { 0.26 }; ///< Critical wing stall angle of attack in radians (~15 deg).
    double flapLiftCoeff { 0.45 }; ///< Additional lift coefficient from full flap deflection.
    double flapDragCoeff { 0.05 }; ///< Additional drag coefficient from full flap deflection.

    // Rotor / Wing interference
    double downloadFactor { 0.09 }; ///< Hover wing download fraction of rotor thrust (5% - 10%).
    double igeHeight { 8.0 }; ///< Ground clearance ceiling for In-Ground Effect (IGE) in meters.
    double igeMaxBoost { 0.20 }; ///< Peak IGE thrust augmentation ratio at surface.
    double etlSpeedThreshold { 9.0 }; ///< Effective Translational Lift (ETL) onset airspeed in m/s.

    // Flight corridor envelope protection
    double stallAirspeed { 38.0 }; ///< Clean wing aerodynamic stall airspeed in m/s (~137 km/h).
    double maxHoverAirspeed { 45.0 }; ///< Maximum safe airspeed with nacelles vertical in m/s.
    double maxCruiseAirspeed { 145.0 }; ///< Maximum level flight cruising airspeed in m/s.

    // Flight control authority and moments
    double cyclicPitchAuth { 18000.0 }; ///< Peak hover cyclic pitch torque authority in N*m.
    double elevatorPitchAuth { 24000.0 }; ///< Peak cruise elevator pitch torque authority in N*m.
    double pitchDamping { 6500.0 }; ///< Aerodynamic pitch rate damping moment in N*m*s.
    double weathercockStab { 14000.0 }; ///< Aerodynamic pitch restoring moment in N*m/rad.
    double nacelleMass { 1200.0 }; ///< Total mass of tilting nacelle assemblies in kg.
    double nacelleMastLength { 1.8 }; ///< Pivot-to-propeller hub distance in meters.

    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravitational acceleration vector in m/s^2.
};

/// @struct VTOLTiltRotorInput
/// @brief Pilot flight control commands for tilt-rotor VTOL dynamics.
struct VTOLTiltRotorInput {
    double throttle { 0.0 }; ///< Engine thrust / collective command in [0.0, 1.0].
    double pitchStick { 0.0 }; ///< Longitudinal pitch stick in [-1.0, 1.0] (+1 pitch up).
    double targetNacelleAngle { 1.5707963 }; ///< Desired nacelle tilt in rad [0, pi/2] (pi/2 = hover).
    double flaps { 0.0 }; ///< Trailing-edge flap deflection command in [0.0, 1.0].
    double groundDistance { 15.0 }; ///< Radar altimeter terrain clearance in meters.
    bool enginePower { true }; ///< Master engine turbine drive flag.
};

/// @struct VTOLTiltRotorTelemetry
/// @brief Diagnostic snapshot of internal tilt-rotor dynamics states.
struct VTOLTiltRotorTelemetry {
    double airspeed { 0.0 }; ///< True airspeed relative to air mass in m/s.
    double forwardVelocity { 0.0 }; ///< Longitudinal velocity along fuselage axis in m/s.
    double climbRate { 0.0 }; ///< Vertical climb or descent velocity in m/s.
    double dynamicPressure { 0.0 }; ///< Dynamic pressure q = 0.5 * rho * V^2 in Pa.
    double angleOfAttack { 0.0 }; ///< Aerodynamic angle of attack alpha in radians.

    double nacelleAngleRad { 1.5707963 }; ///< Active nacelle tilt angle in radians.
    double nacelleAngleDeg { 90.0 }; ///< Active nacelle tilt angle in degrees.
    double tiltRateDegPerSec { 0.0 }; ///< Instantaneous conversion rate in deg/s.
    VTOLFlightMode flightMode { VTOLFlightMode::Hover }; ///< Active flight regime.

    double rotorThrust { 0.0 }; ///< Total proprotor thrust force magnitude in Newtons.
    double rotorRpm { 0.0 }; ///< Current proprotor rotational speed in RPM.
    double wingLift { 0.0 }; ///< Fixed-wing aerodynamic lift force in Newtons.
    double wingDrag { 0.0 }; ///< Fixed-wing aerodynamic drag force in Newtons.
    double wingDownload { 0.0 }; ///< Downward vertical drag on wing from rotor slipstream in N.

    double minCorridorSpeed { 0.0 }; ///< Minimum safe airspeed for current nacelle angle in m/s.
    double maxCorridorSpeed { 0.0 }; ///< Maximum safe airspeed for current nacelle angle in m/s.
    bool isCorridorSafe { true }; ///< True if operating inside safe conversion envelope.
    bool isWingStalled { false }; ///< True if wing angle of attack exceeds stall boundary.
    bool isGroundEffect { false }; ///< True if within in-ground effect cushion.
};

/// @class VTOLTiltRotorDynamics
/// @brief 2D Tilt-Rotor VTOL Aircraft Dynamics Coordinator.
/// @details Simulates smooth conversion between helicopter hover mode and fixed-wing airplane cruise,
/// proprotor thrust vectoring, wing download drag, conversion corridor protection, and control cross-fading.
class VTOLTiltRotorDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor initializing default tilt-rotor configuration.
    VTOLTiltRotorDynamics() noexcept;

    /// @brief Parameterized constructor with custom aircraft configuration.
    /// @param[in] config Aircraft physical configuration parameters.
    explicit VTOLTiltRotorDynamics(const VTOLTiltRotorConfig& config) noexcept;

    /// @brief Advances tilt-rotor flight dynamics simulation step by timestep dt.
    /// @details Evaluates nacelle tilt slew rate, proprotor thrust vectoring, wing aerodynamics,
    /// conversion corridor protection, control surface mixing, and applies net forces/torques.
    /// @param[in,out] body Target rigid body receiving accumulated forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs, rotor spool, and conversion actuators.
    void reset() noexcept override;

    /// @brief Queries whether the aircraft is airborne.
    /// @return True if ground clearance exceeds minimum landing gear contact height.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active tilt-rotor aircraft configuration.
    /// @return Const reference to active VTOLTiltRotorConfig.
    [[nodiscard]] const VTOLTiltRotorConfig& getConfig() const noexcept;

    /// @brief Updates active aircraft configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const VTOLTiltRotorConfig& config) noexcept;

    /// @brief Gets current pilot control inputs.
    /// @return Const reference to active VTOLTiltRotorInput.
    [[nodiscard]] const VTOLTiltRotorInput& getInput() const noexcept;

    /// @brief Sets pilot flight control inputs.
    /// @param[in] input New pilot control commands.
    void setInput(const VTOLTiltRotorInput& input) noexcept;

    /// @brief Gets diagnostic flight telemetry snapshot.
    /// @return Const reference to VTOLTiltRotorTelemetry.
    [[nodiscard]] const VTOLTiltRotorTelemetry& getTelemetry() const noexcept;

    /// @brief Gets active nacelle conversion angle.
    /// @return Nacelle angle in radians (0.0 = cruise, pi/2 = hover).
    [[nodiscard]] double getNacelleAngle() const noexcept;

    /// @brief Gets active nacelle conversion angle in degrees.
    /// @return Nacelle angle in degrees (0.0 = cruise, 90.0 = hover).
    [[nodiscard]] double getNacelleDeg() const noexcept;

    /// @brief Gets true flight airspeed.
    /// @return Airspeed scalar magnitude in m/s.
    [[nodiscard]] double getAirspeed() const noexcept;

    /// @brief Gets total net proprotor thrust.
    /// @return Net thrust force in Newtons.
    [[nodiscard]] double getRotorThrust() const noexcept;

    /// @brief Gets aerodynamic wing lift force.
    /// @return Wing lift force in Newtons.
    [[nodiscard]] double getWingLift() const noexcept;

    /// @brief Queries whether the wings are aerodynamically stalled.
    /// @return True if in post-stall flow separation regime.
    [[nodiscard]] bool isStalled() const noexcept;

    /// @brief Queries whether the aircraft is operating within the safe conversion corridor.
    /// @return True if airspeed is bounded between minimum and maximum conversion corridor limits.
    [[nodiscard]] bool isCorridorSafe() const noexcept;

    /// @brief Gets current flight operational mode.
    /// @return Active VTOLFlightMode enum.
    [[nodiscard]] VTOLFlightMode getFlightMode() const noexcept;

private:
    VTOLTiltRotorConfig mConfig { };
    VTOLTiltRotorInput mInput { };
    VTOLTiltRotorTelemetry mTelemetry { };

    math::SlewRateLimiter mTiltLimiter { };
    double mRotorRpm { 0.0 };
    double mActiveNacelleAngle { 1.5707963 };
};

} // namespace vehicle_sim::dynamics
