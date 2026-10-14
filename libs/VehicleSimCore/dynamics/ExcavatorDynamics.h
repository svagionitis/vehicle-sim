#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct ExcavatorArmConfig
/// @brief Geometric and inertial parameters of the 3-DOF articulated excavator arm.
struct ExcavatorArmConfig {
    double boomLength { 5.7 }; ///< Length of boom link L1 in meters.
    double stickLength { 2.9 }; ///< Length of stick/arm link L2 in meters.
    double bucketRadius { 1.4 }; ///< Cutting radius of bucket link L3 in meters.
    double boomMass { 3200.0 }; ///< Boom structural mass in kg.
    double stickMass { 1600.0 }; ///< Stick structural mass in kg.
    double bucketMass { 950.0 }; ///< Empty bucket mass in kg.
    double maxBucketPayload { 3000.0 }; ///< Maximum payload capacity of soil in bucket in kg.

    double boomMinAngle { -0.35 }; ///< Boom minimum pitch angle in radians (~ -20 deg).
    double boomMaxAngle { 1.22 }; ///< Boom maximum pitch angle in radians (~ 70 deg).
    double stickMinAngle { 0.52 }; ///< Stick minimum dipper angle in radians (~ 30 deg).
    double stickMaxAngle { 2.79 }; ///< Stick maximum dipper angle in radians (~ 160 deg).
    double bucketMinAngle { -0.78 }; ///< Bucket minimum curl angle in radians (~ -45 deg).
    double bucketMaxAngle { 2.35 }; ///< Bucket maximum curl angle in radians (~ 135 deg).

    double maxBoomSpeed { 0.45 }; ///< Maximum boom angular rate in rad/s.
    double maxStickSpeed { 0.55 }; ///< Maximum stick angular rate in rad/s.
    double maxBucketSpeed { 0.75 }; ///< Maximum bucket curl rate in rad/s.
    double maxDiggingForce { 150000.0 }; ///< Maximum bucket cylinder breakout force in Newtons (~150 kN).
};

/// @struct ExcavatorConfig
/// @brief Physical configuration parameters for heavy dual-track crawler excavator.
struct ExcavatorConfig {
    double undercarriageMass { 12000.0 }; ///< Lower crawler chassis mass in kg.
    double superstructureMass { 14000.0 }; ///< Upper rotating cabin and counterweight mass in kg.
    double trackGauge { 2.6 }; ///< Lateral center-to-center track width B in meters.
    double trackContactLength { 3.8 }; ///< Ground contact length Lc of crawler tracks in meters.
    double maxTrackForce { 95000.0 }; ///< Maximum tractive force per crawler track in Newtons.
    double maxTrackSpeed { 1.5 }; ///< Maximum crawler travel speed in m/s (~5.4 km/h).
    double trackFriction { 0.85 }; ///< Track-to-terrain friction coefficient.
    double groundScrubTorque { 45000.0 }; ///< Ground shearing scrubbing torque in N*m.

    double slewInertia { 35000.0 }; ///< Superstructure moment of inertia I_upper in kg*m^2.
    double maxSlewTorque { 65000.0 }; ///< Maximum hydraulic slew motor torque in N*m.
    double maxSlewSpeed { 1.15 }; ///< Maximum slew rotation velocity in rad/s (~11 rpm).
    double slewDamping { 12000.0 }; ///< Viscous damping of slew ring bearing in N*m*s.

    ExcavatorArmConfig arm {}; ///< Kinematic parameters of 3-DOF articulated arm.
};

/// @struct ExcavatorInput
/// @brief Operator control inputs for crawler travel, slew ring, and articulated arm.
struct ExcavatorInput {
    double leftTrack { 0.0 }; ///< Commanded left crawler track throttle in [-1.0, 1.0].
    double rightTrack { 0.0 }; ///< Commanded right crawler track throttle in [-1.0, 1.0].
    double slewCommand { 0.0 }; ///< Commanded upper superstructure slew motor in [-1.0, 1.0].
    double boomCommand { 0.0 }; ///< Commanded boom elevation rate in [-1.0, 1.0].
    double stickCommand { 0.0 }; ///< Commanded stick dipper rate in [-1.0, 1.0].
    double bucketCommand { 0.0 }; ///< Commanded bucket curl rate in [-1.0, 1.0].
    bool slewBrakeActive { false }; ///< Hydraulic slew holding brake lock engagement.
};

/// @struct ExcavatorTelemetry
/// @brief Diagnostic snapshot of excavator kinematics, hydraulics, and tipping margins.
struct ExcavatorTelemetry {
    double forwardSpeed { 0.0 }; ///< Undercarriage forward speed in m/s.
    double yawRate { 0.0 }; ///< Undercarriage skid-steer yaw rate in rad/s.
    double slewAngle { 0.0 }; ///< Superstructure slew orientation relative to chassis in radians.
    double slewSpeed { 0.0 }; ///< Superstructure rotational rate in rad/s.
    double boomAngle { 0.0 }; ///< Boom joint angle theta1 in radians.
    double stickAngle { 0.0 }; ///< Stick joint angle theta2 in radians.
    double bucketAngle { 0.0 }; ///< Bucket joint angle theta3 in radians.
    math::Vec2 tcpPosition { 0.0, 0.0 }; ///< Bucket Tool Center Point position (reach, height) in meters.
    math::Vec2 tcpVelocity { 0.0, 0.0 }; ///< Bucket Tool Center Point linear velocity vector in m/s.
    math::Vec2 digForce { 0.0, 0.0 }; ///< Applied external soil digging reaction force in Newtons.
    double tippingMargin { 1.0 }; ///< Safety tipping margin [1.0 = fully balanced, 0.0 = track lift-off].
    double tippingIndex { 0.0 }; ///< Dynamic tipping index ratio (|M_overturn| / M_restoring).
    bool isTippedOver { false }; ///< True if machine has exceeded stability fulcrum tipping boundary.
    bool isArmStalled { false }; ///< True if digging resistance exceeds hydraulic cylinder capacity.
};

/// @class ExcavatorDynamics
/// @brief Heavy dual-track hydraulic excavator dynamics coordinator.
/// @details Simulates crawler undercarriage skid-steer kinematics with terrain scrubbing,
/// 360-degree continuous upper cabin slew bearing with dynamic inertia shifting,
/// planar 3-DOF articulated hydraulic arm forward kinematics with analytical Jacobian,
/// external bucket soil reaction forces, and machine tipping stability margins.
class ExcavatorDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline excavator configuration.
    ExcavatorDynamics() noexcept;

    /// @brief Parameterized constructor with custom excavator configuration.
    /// @param[in] config Machine geometry, mass, slew, and arm configurations.
    explicit ExcavatorDynamics(const ExcavatorConfig& config) noexcept;

    /// @brief Advances simulation step, applying forces and moments to rigid body.
    /// @param[in,out] body Target rigid body representing excavator lower chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets operator commands and transient actuator states.
    void reset() noexcept override;

    /// @brief Queries whether the vehicle is airborne or tipped over.
    /// @return True if machine is detached from ground or tipped over.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active excavator configuration.
    /// @return Const reference to active ExcavatorConfig.
    [[nodiscard]] const ExcavatorConfig& getConfig() const noexcept;

    /// @brief Updates active excavator configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const ExcavatorConfig& config) noexcept;

    /// @brief Gets current operator control inputs.
    /// @return Const reference to active ExcavatorInput.
    [[nodiscard]] const ExcavatorInput& getInput() const noexcept;

    /// @brief Sets current operator control inputs.
    /// @param[in] input New operator commands.
    void setInput(const ExcavatorInput& input) noexcept;

    /// @brief Gets current diagnostic telemetry.
    /// @return Const reference to active ExcavatorTelemetry.
    [[nodiscard]] const ExcavatorTelemetry& getTelemetry() const noexcept;

    /// @brief Sets external soil digging resistance force applied to bucket tip.
    /// @param[in] force External force vector in world coordinates in Newtons.
    void setDigForce(const math::Vec2& force) noexcept;

    /// @brief Sets payload mass currently held within the bucket.
    /// @param[in] payloadMass Soil/rock payload mass in kg.
    void setBucketPayload(double payloadMass) noexcept;

    /// @brief Queries current bucket Tool Center Point in upper carriage local frame.
    /// @return 2D position vector (reach forward, height above ground) in meters.
    [[nodiscard]] math::Vec2 getTcpPosition() const noexcept;

    /// @brief Queries current superstructure slew angle relative to chassis.
    /// @return Slew angle in radians.
    [[nodiscard]] double getSlewAngle() const noexcept;

    /// @brief Queries whether machine is currently tipping over fulcrum edges.
    /// @return True if tipping index >= 1.0.
    [[nodiscard]] bool isTippedOver() const noexcept;

    /// @brief Sets initial arm joint angles.
    /// @param[in] boom Boom angle in radians.
    /// @param[in] stick Stick angle in radians.
    /// @param[in] bucket Bucket angle in radians.
    void setArmJointAngles(double boom, double stick, double bucket) noexcept;

private:
    ExcavatorConfig m_config {};
    ExcavatorInput m_input {};
    ExcavatorTelemetry m_telemetry {};

    double m_slewAngle { 0.0 };
    double m_slewVelocity { 0.0 };
    double m_boomAngle { 0.35 };
    double m_stickAngle { 1.57 };
    double m_bucketAngle { 0.78 };
    double m_bucketPayload { 0.0 };
    math::Vec2 m_digForceWorld { 0.0, 0.0 };
    bool m_isTippedOver { false };

    void updateUndercarriage(physics::RigidBody2D& body, double dt) noexcept;
    void updateSlewRing(double dt) noexcept;
    void updateArmKinematics(double dt) noexcept;
    void evaluateTippingMargin() noexcept;
};

} // namespace vehicle_sim::dynamics
