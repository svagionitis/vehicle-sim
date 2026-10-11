#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct ArticulatedTruckConfig
/// @brief Configuration parameters for 2D articulated truck and semi-trailer dynamics.
/// @details Defines tractor cab mass, trailer mass, fifth-wheel kingpin coupling offset,
/// trailer wheelbase length, steering geometry, service and independent trailer brakes,
/// hitch angular limits, and jackknife instability thresholds.
struct ArticulatedTruckConfig {
    double tractorMass { 7500.0 }; ///< Tractor cab mass in kg.
    double trailerMass { 24000.0 }; ///< Semi-trailer cargo mass in kg.
    double tractorWheelbase { 3.8 }; ///< Distance from front steer axle to rear drive tandem in meters.
    double kingpinOffset { -1.8 }; ///< Fifth-wheel hitch mount position along tractor longitudinal axis in meters.
    double trailerLength { 11.0 }; ///< Effective distance from kingpin to trailer rear axles in meters.

    double maxSteerAngle { 0.55 }; ///< Maximum front wheel steer deflection in radians (~31.5 deg).
    double steerSlewRate { 2.2 }; ///< Front steering slew rate in rad/s.

    double maxDriveForce { 44000.0 }; ///< Peak drivetrain propulsion force in Newtons.
    double maxTractorBrakeForce { 50000.0 }; ///< Maximum tractor service braking force in Newtons.
    double maxTrailerBrakeForce { 68000.0 }; ///< Maximum trailer axle braking force in Newtons.

    double maxHitchAngle { 1.48 }; ///< Maximum physical fifth-wheel articulation angle in radians (~85 deg).
    double jackknifeThresholdAngle { 0.95 }; ///< Articulation angle triggering jackknife instability warning in radians (~54.5 deg).
    double hitchFrictionDamping { 14000.0 }; ///< Fifth-wheel plate rotational friction damping in N*m*s.

    double tractorCorneringStiffness { 38000.0 }; ///< Tractor axle tire lateral grip stiffness in N*s/m.
    double trailerCorneringStiffness { 48000.0 }; ///< Trailer axle tire lateral grip stiffness in N*s/m.
    double rollingResistanceCoeff { 220.0 }; ///< Tire rolling resistance coefficient in N/(m/s).
    double aerodynamicDragCoeff { 28.0 }; ///< Aerodynamic frontal form drag coefficient in N/(m/s)^2.
};

/// @struct ArticulatedTruckInput
/// @brief Driver control commands for articulated truck dynamics.
struct ArticulatedTruckInput {
    double throttle { 0.0 }; ///< Commanded engine throttle in [-1.0, 1.0].
    double steer { 0.0 }; ///< Commanded front steering wheel input in [-1.0, 1.0].
    double serviceBrake { 0.0 }; ///< Combined tractor and trailer foot brake in [0.0, 1.0].
    double trailerBrake { 0.0 }; ///< Independent trailer trolley valve brake in [0.0, 1.0].
};

/// @struct ArticulatedTruckTelemetry
/// @brief Diagnostic snapshot of articulated truck and trailer dynamics states.
struct ArticulatedTruckTelemetry {
    double speed { 0.0 }; ///< Tractor absolute speed in m/s.
    double forwardSpeed { 0.0 }; ///< Tractor longitudinal forward velocity in m/s.
    double steerAngle { 0.0 }; ///< Current front steering angle in radians.
    double hitchAngle { 0.0 }; ///< Relative fifth-wheel articulation angle in radians.
    double trailerHeading { 0.0 }; ///< World space trailer orientation in radians.
    double jackknifeSeverity { 0.0 }; ///< Jackknife risk severity ratio in [0.0, 1.0].
    double tractorBrakeForce { 0.0 }; ///< Total applied tractor braking force in Newtons.
    double trailerBrakeForce { 0.0 }; ///< Total applied trailer braking force in Newtons.
    double totalMass { 0.0 }; ///< Combined tractor and trailer mass in kg.
    double offTrackingDistance { 0.0 }; ///< Lateral offset between tractor front path and trailer rear path in meters.
    bool isJackknifed { false }; ///< True if articulation angle exceeded jackknife threshold.
};

/// @class ArticulatedTruckDynamics
/// @brief 2D articulated truck and semi-trailer dynamics coordinator.
/// @details Simulates fifth-wheel hitch rotational kinematics, coupled trailer inertia,
/// differential tractor/trailer braking, jackknife instability on low friction or hard reverse,
/// and independent trailer hand-brake tension stabilization.
class ArticulatedTruckDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline articulated truck configuration.
    ArticulatedTruckDynamics() noexcept;

    /// @brief Parameterized constructor with custom articulated truck configuration.
    /// @param[in] config Articulated truck configuration parameters.
    explicit ArticulatedTruckDynamics(const ArticulatedTruckConfig& config) noexcept;

    /// @brief Advances articulated truck simulation step by timestep dt.
    /// @details Computes tractor and trailer kinematic coupling, fifth-wheel constraint forces,
    /// tire cornering forces, drivetrain propulsion, and jackknife dynamics.
    /// @param[in,out] body Target rigid body representing the tractor chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs, hitch angle, and telemetry states.
    void reset() noexcept override;

    /// @brief Queries whether the articulated vehicle is airborne.
    /// @return Always false for ground transport.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active articulated truck configuration.
    /// @return Const reference to active ArticulatedTruckConfig.
    [[nodiscard]] const ArticulatedTruckConfig& getConfig() const noexcept;

    /// @brief Updates active articulated truck configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const ArticulatedTruckConfig& config) noexcept;

    /// @brief Gets current driver control inputs.
    /// @return Const reference to active ArticulatedTruckInput.
    [[nodiscard]] const ArticulatedTruckInput& getInput() const noexcept;

    /// @brief Sets driver control inputs.
    /// @param[in] input New driver control commands.
    void setInput(const ArticulatedTruckInput& input) noexcept;

    /// @brief Gets diagnostic telemetry snapshot.
    /// @return Const reference to ArticulatedTruckTelemetry.
    [[nodiscard]] const ArticulatedTruckTelemetry& getTelemetry() const noexcept;

    /// @brief Gets relative fifth-wheel articulation hitch angle in radians.
    /// @return Hitch angle in radians.
    [[nodiscard]] double getHitchAngle() const noexcept;

    /// @brief Sets relative fifth-wheel articulation hitch angle in radians.
    /// @param[in] angle Articulation angle in radians.
    void setHitchAngle(double angle) noexcept;

    /// @brief Gets absolute trailer orientation in world space in radians.
    /// @return Trailer heading in radians.
    [[nodiscard]] double getTrailerHeading() const noexcept;

    /// @brief Queries whether the vehicle is in an active jackknife state.
    /// @return True if hitch angle exceeds jackknife threshold.
    [[nodiscard]] bool isJackknifed() const noexcept;

    /// @brief Gets active front wheel steering angle in radians.
    /// @return Steer angle in radians.
    [[nodiscard]] double getSteerAngle() const noexcept;

    /// @brief Gets tractor forward speed along heading.
    /// @return Speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    ArticulatedTruckConfig mConfig { };
    ArticulatedTruckInput mInput { };
    ArticulatedTruckTelemetry mTelemetry { };
    double mSteerAngle { 0.0 };
    double mHitchAngle { 0.0 };
    double mTrailerHeading { 0.0 };
};

} // namespace vehicle_sim::dynamics
