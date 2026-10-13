#pragma once

#include <cmath>
#include <cstdint>

#include "libs/VehicleSimCore/math/SlewRateLimiter.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/FluidDrag2D.h"
#include "libs/VehicleSimCore/physics/FluidField2D.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct GroundEffectAeroConfig
/// @brief Physical and geometric parameters for ground effect downforce and active DRS flaps.
/// @details Defines reference aerodynamic areas, Venturi tunnel geometries, stall thresholds,
/// pitch rake sensitivity, center-of-pressure offsets, and DRS actuation limits.
struct GroundEffectAeroConfig {
    // Reference Geometry & Baseline Coefficients
    double referenceArea { 2.2 }; ///< Aerodynamic frontal/planform reference area in m^2.
    double baseDragCoeff { 0.38 }; ///< Baseline zero-lift chassis drag coefficient (Cd0).
    double baseWingLiftCoeff { 0.85 }; ///< Baseline free-stream wing downforce coefficient (Cl0,wing).
    double baseGroundLiftCoeff { 1.20 }; ///< Baseline underbody ground effect lift coefficient (Cl0,ground).
    double inducedDragFactor { 0.045 }; ///< Lift-induced drag scaling factor (k_induced * Cl^2).

    // Ground Effect & Venturi Parameters
    double venturiFactor { 0.15 }; ///< Venturi throat expansion factor in meters (k_venturi).
    double stallRideHeight { 0.025 }; ///< Critical ride height below which stall begins in meters (h_stall).
    double minChokeRideHeight { 0.005 }; ///< Ride height at total flow choking in meters (h_min).
    double stallResidualRatio { 0.10 }; ///< Residual suction fraction during total stall (beta_stall).
    double rakeSensitivity { 0.35 }; ///< Diffuser suction sensitivity to chassis rake (k_rake).

    // Center of Pressure & Pitch Moment
    double staticCpOffset { -0.15 }; ///< Static longitudinal offset of CoP from CG in meters (negative = rearward).
    double pitchCpMigrationRate { 0.40 }; ///< CoP migration rate per radian of pitch in m/rad.
    double drsCpMigrationRate { 0.10 }; ///< Forward CoP shift under full DRS activation in meters.

    // Active DRS Parameters
    double maxFlapAngleDeg { 25.0 }; ///< Maximum DRS flap opening angle in degrees.
    double flapOpenSpeedDegPerSec { 125.0 }; ///< Slew rate for opening flap in deg/s (full open in 0.2s).
    double flapCloseSpeedDegPerSec { 250.0 }; ///< Slew rate for closing flap in deg/s (full close in 0.1s).
    double maxDragReduction { 0.30 }; ///< Maximum drag reduction fraction when fully opened (eta_drs).
    double maxDownforceShedding { 0.22 }; ///< Wing downforce reduction fraction when opened (xi_drs).

    // Safety Interlock Thresholds
    double brakeDisengageThreshold { 0.05 }; ///< Brake pedal threshold triggering immediate DRS closure.
    double steerDisengageThreshold { 0.35 }; ///< Absolute steering demand threshold triggering closure.
    double lateralGDisengageThreshold { 1.5 }; ///< Lateral acceleration threshold in Gs triggering closure.
};

/// @struct GroundEffectAeroInput
/// @brief Per-tick runtime state and control inputs passed to the aerodynamic coordinator.
struct GroundEffectAeroInput {
    double frontRideHeight { 0.045 }; ///< Measured ground clearance at front axle in meters.
    double rearRideHeight { 0.050 }; ///< Measured ground clearance at rear axle in meters.
    double pitchAngle { 0.0 }; ///< Chassis pitch orientation in radians.
    double steerDemand { 0.0 }; ///< Normalized steering command in [-1.0, 1.0].
    double brakeDemand { 0.0 }; ///< Normalized braking command in [0.0, 1.0].
    double lateralAccelerationG { 0.0 }; ///< Measured lateral acceleration in Gs.
    bool isAirborne { false }; ///< True if vehicle is detached from ground plane.
    bool requestDrs { false }; ///< Driver/AI commanded DRS activation request.
};

/// @struct GroundEffectAeroOutput
/// @brief Evaluated aerodynamic forces, moments, and diagnostic telemetry states.
struct GroundEffectAeroOutput {
    math::Vec2 totalForceWorld { 0.0, 0.0 }; ///< Combined aero force vector in world coordinates in Newtons.
    double downforce { 0.0 }; ///< Net downward aerodynamic force magnitude in Newtons.
    double totalDrag { 0.0 }; ///< Net retarding drag force magnitude in Newtons.
    double pitchMoment { 0.0 }; ///< Net aerodynamic pitching trim moment in N*m.
    double effectiveCd { 0.0 }; ///< Modulated drag coefficient including DRS and induced drag.
    double effectiveCl { 0.0 }; ///< Modulated downforce coefficient including ground effect and stall.
    double flapAngleDeg { 0.0 }; ///< Current physical DRS flap angle in degrees.
    double centerOfPressure { 0.0 }; ///< Current longitudinal CoP position relative to CG in meters.
    bool isDrsActive { false }; ///< True if DRS flap is deployed (> 1.0 deg).
    bool isDrsInhibited { false }; ///< True if safety interlock is overriding driver DRS command.
    bool isStalled { false }; ///< True if underbody Venturi is actively in aerodynamic stall.
};

/// @class GroundEffectAero2D
/// @brief Deterministic 2D ground effect downforce and active DRS aerodynamic coordinator.
/// @details Evaluates Venturi underbody downforce scaling, smooth flow choking and stall collapse,
/// pitch rake modulation, migrating center-of-pressure moments, and slew-rate limited active DRS flaps.
/// Conforms to IEEE 754 determinism, pure ISO C++17, and MISRA C++:2023 zero-allocation standards.
class GroundEffectAero2D {
public:
    /// @brief Default constructor using baseline sports/racing configuration.
    GroundEffectAero2D() noexcept;

    /// @brief Parameterized constructor with custom aerodynamic configuration.
    /// @param[in] config Aerodynamic configuration parameters.
    explicit GroundEffectAero2D(const GroundEffectAeroConfig& config) noexcept;

    /// @brief Resets transient flap actuators, historical states, and safety flags.
    void reset() noexcept;

    /// @brief Updates aerodynamic state and computes forces across timestep dt.
    /// @param[in] input Runtime vehicle kinematics, ride heights, and pilot controls.
    /// @param[in] bodyLinearVel World linear velocity vector of vehicle in m/s.
    /// @param[in] bodyHeading Vehicle heading angle in radians.
    /// @param[in] fluidSample Environmental fluid sample (density and wind velocity).
    /// @param[in] dt Simulation timestep duration in seconds.
    /// @return Evaluated aerodynamic output struct.
    [[nodiscard]] GroundEffectAeroOutput update(const GroundEffectAeroInput& input, const math::Vec2& bodyLinearVel,
        double bodyHeading, const physics::FluidSample2D& fluidSample, double dt) noexcept;

    /// @brief Computes and applies aerodynamic forces directly to target rigid body.
    /// @param[in,out] body Target rigid body receiving forces and pitching moments.
    /// @param[in] input Runtime kinematics and pilot commands.
    /// @param[in] fluidSample Environmental fluid sample.
    /// @param[in] dt Simulation timestep in seconds.
    /// @return Evaluated aerodynamic telemetry output.
    GroundEffectAeroOutput applyForces(physics::RigidBody2D& body, const GroundEffectAeroInput& input,
        const physics::FluidSample2D& fluidSample, double dt) noexcept;

    /// @brief Returns the active aerodynamic configuration.
    /// @return Reference to configuration parameters.
    [[nodiscard]] const GroundEffectAeroConfig& getConfig() const noexcept;

    /// @brief Updates the aerodynamic configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const GroundEffectAeroConfig& config) noexcept;

    /// @brief Queries current physical DRS flap angle.
    /// @return Actuated flap angle in degrees.
    [[nodiscard]] double getFlapAngle() const noexcept;

    /// @brief Queries whether the underbody is currently stalled.
    /// @return True if ride height is below critical stall threshold.
    [[nodiscard]] bool isStalled() const noexcept;

    /// @brief Queries whether DRS is currently deployed.
    /// @return True if flap angle exceeds 1.0 degree.
    [[nodiscard]] bool isDrsActive() const noexcept;

private:
    /// @brief Evaluates smooth cubic Hermite stall decay factor.
    /// @param[in] rideHeight Current mean ground clearance in meters.
    /// @return Attenuation scalar in [stallResidualRatio, 1.0].
    [[nodiscard]] double computeStall(double rideHeight) const noexcept;

    /// @brief Evaluates safety interlock conditions.
    /// @param[in] input Pilot inputs and lateral acceleration state.
    /// @return True if DRS deployment is permitted.
    [[nodiscard]] bool checkInterlock(const GroundEffectAeroInput& input) const noexcept;

    GroundEffectAeroConfig mConfig {};
    math::SlewRateLimiter mFlapLimiter {};
    GroundEffectAeroOutput mLastOutput {};
    bool mIsStalled { false };
    bool mIsDrsInhibited { false };
};

} // namespace vehicle_sim::dynamics
