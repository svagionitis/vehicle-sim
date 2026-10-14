#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct BicycleConfig
/// @brief Physical, geometric, and Whipple stability parameters for BicycleDynamics.
struct BicycleConfig {
    double totalMass { 85.0 }; ///< Combined mass of bicycle frame, wheels, and rider in kg.
    double riderMass { 72.0 }; ///< Active rider mass in kg.
    double forkMass { 4.2 }; ///< Mass of front fork, handlebars, and front wheel assembly in kg.
    double wheelbase { 1.02 }; ///< Wheelbase L_wb between wheel contact patches in meters.
    double distFrontAxle { 0.62 }; ///< Longitudinal distance from static COG to front axle in meters.
    double cogHeight { 0.95 }; ///< Height of composite COG above ground in meters.

    // Geometry & Caster Trail
    double headTubeAngleRad { 1.27 }; ///< Steering axis rake angle lambda (~73 degrees from horizontal).
    double forkRakeOffset { 0.045 }; ///< Fork forward offset f_rake in meters.
    double frontWheelRadius { 0.34 }; ///< Front tire rolling radius in meters (700c wheel).
    double rearWheelRadius { 0.34 }; ///< Rear tire rolling radius in meters.
    double frontWheelSpinInertia { 0.14 }; ///< Front wheel spin polar moment of inertia I_yy,f in kg*m^2.
    double rearWheelSpinInertia { 0.16 }; ///< Rear wheel spin polar moment of inertia I_yy,r in kg*m^2.

    // Tire Cornering & Camber Properties
    double frontCorneringStiffness { 420.0 }; ///< Front tire cornering stiffness C_alpha in N/rad.
    double rearCorneringStiffness { 480.0 }; ///< Rear tire cornering stiffness C_alpha in N/rad.
    double frontCamberStiffness { 65.0 }; ///< Front round-profile tire camber thrust stiffness C_gamma in N/rad.
    double rearCamberStiffness { 72.0 }; ///< Rear round-profile tire camber thrust stiffness C_gamma in N/rad.
    double tireFrictionCoeff { 0.95 }; ///< Maximum tire-asphalt friction coefficient.

    // Whipple Inertia Matrix Elements (M_phi_phi, M_phi_delta, M_delta_delta)
    double massRollInertia { 9.8 }; ///< Composite roll moment of inertia M_phi_phi in kg*m^2.
    double massCrossInertia { 0.38 }; ///< Roll-steer cross product of inertia M_phi_delta in kg*m^2.
    double massSteerInertia { 0.28 }; ///< Steering assembly moment of inertia M_delta_delta in kg*m^2.

    double steerDamping { 2.4 }; ///< Steering headset viscous damping in N*m*s/rad.
    double maxSteerAngle { 0.70 }; ///< Maximum mechanical handlebar steering lock in radians (~40 deg).
    double maxDriveForce { 650.0 }; ///< Peak pedaling tractive chain drive force in Newtons.
    double maxFrontBrakeForce { 950.0 }; ///< Front hydraulic disc brake maximum force in Newtons.
    double maxRearBrakeForce { 600.0 }; ///< Rear hydraulic disc brake maximum force in Newtons.
};

/// @struct BicycleInput
/// @brief Rider control commands for Whipple single-track bicycle dynamics.
struct BicycleInput {
    double pedalingThrottle { 0.0 }; ///< Pedaling drive force command in range [0.0, 1.0].
    double frontBrake { 0.0 }; ///< Front handbrake lever command in range [0.0, 1.0].
    double rearBrake { 0.0 }; ///< Rear handbrake lever command in range [0.0, 1.0].
    double steerTorque { 0.0 }; ///< Rider handlebar steer torque command in N*m (+ right, - left).
    double riderLeanRoll { 0.0 }; ///< Rider upper body lean command in range [-1.0, 1.0] (+ right, - left).
};

/// @struct BicycleTelemetry
/// @brief Diagnostic telemetry snapshot for Whipple single-track bicycle dynamics.
struct BicycleTelemetry {
    double forwardSpeed { 0.0 }; ///< Forward longitudinal velocity in m/s.
    double rollAngle { 0.0 }; ///< Lean / roll angle phi in radians (+ right, - left).
    double rollRate { 0.0 }; ///< Roll angular velocity in rad/s.
    double steerAngle { 0.0 }; ///< Handlebar steer angle delta in radians (+ right, - left).
    double steerRate { 0.0 }; ///< Steer angular velocity in rad/s.

    double mechanicalTrail { 0.0 }; ///< Effective mechanical trail c_trail in meters.
    double frontWheelGyroscopicTorque { 0.0 }; ///< Gyroscopic precession torque from front wheel in N*m.
    double casterRestoringTorque { 0.0 }; ///< Self-aligning torque from caster trail in N*m.

    double frontNormalLoad { 0.0 }; ///< Dynamic vertical load on front tire in Newtons.
    double rearNormalLoad { 0.0 }; ///< Dynamic vertical load on rear tire in Newtons.
    double frontLateralForce { 0.0 }; ///< Total lateral contact patch force on front tire in Newtons.
    double rearLateralForce { 0.0 }; ///< Total lateral contact patch force on rear tire in Newtons.
    double frontCamberThrust { 0.0 }; ///< Camber thrust component on front tire in Newtons.
    double rearCamberThrust { 0.0 }; ///< Camber thrust component on rear tire in Newtons.

    bool isCounterSteering { false }; ///< True when steer angle opposes roll lean direction.
    bool isSelfStable { false }; ///< True when forward speed falls within hands-free stability window.
    bool isCapsizeMode { false }; ///< True when operating above critical capsize velocity.
    bool isAirborne { false }; ///< True when both tires are detached from ground.
};

/// @class BicycleDynamics
/// @brief Benchmark Whipple single-track bicycle dynamics coordinator.
/// @details Implements 2-DOF roll-steer equations of motion, counter-steering initiation,
/// gyroscopic precession self-stabilization, mechanical caster trail, and rounded tire camber thrust.
class BicycleDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using road racing bicycle configuration.
    BicycleDynamics() noexcept;

    /// @brief Parameterized constructor using custom bicycle configuration.
    /// @param[in] config Physical and geometric parameters.
    explicit BicycleDynamics(const BicycleConfig& config) noexcept;

    /// @brief Advances simulation step, integrating 2-DOF roll-steer equations.
    /// @param[in,out] body Target rigid body representing bicycle chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient dynamics, roll angle, and steer angle states.
    void reset() noexcept override;

    /// @brief Queries whether bicycle is airborne.
    /// @return True if both tires have lost contact with the ground.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active bicycle configuration.
    /// @return Const reference to BicycleConfig.
    [[nodiscard]] const BicycleConfig& getConfig() const noexcept;

    /// @brief Updates active bicycle configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const BicycleConfig& config) noexcept;

    /// @brief Gets current rider control inputs.
    /// @return Const reference to BicycleInput.
    [[nodiscard]] const BicycleInput& getInput() const noexcept;

    /// @brief Sets current rider control inputs.
    /// @param[in] input New rider commands.
    void setInput(const BicycleInput& input) noexcept;

    /// @brief Gets current diagnostic telemetry.
    /// @return Const reference to BicycleTelemetry.
    [[nodiscard]] const BicycleTelemetry& getTelemetry() const noexcept;

private:
    BicycleConfig m_config{};
    BicycleInput m_input{};
    BicycleTelemetry m_telemetry{};

    double m_rollAngleRad{ 0.0 };
    double m_rollRateRadPerSec{ 0.0 };
    double m_steerAngleRad{ 0.0 };
    double m_steerRateRadPerSec{ 0.0 };
    double m_lastForwardSpeed{ 0.0 };
    bool m_isAirborne{ false };
};

} // namespace vehicle_sim::dynamics
