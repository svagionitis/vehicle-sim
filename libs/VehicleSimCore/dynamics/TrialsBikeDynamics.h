#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct TrialsBikeConfig
/// @brief Configuration parameters for 2-wheel trials motorcycle dynamics.
/// @details Defines chassis and rider mass, wheelbase dimensions, suspension compliance,
/// drive and braking torque capacities, rider longitudinal CG shift bounds, and in-air angular momentum.
struct TrialsBikeConfig {
    double mass { 110.0 }; ///< Motorcycle frame and engine dry mass in kg.
    double riderMass { 75.0 }; ///< Active rider body mass in kg.
    double wheelbase { 1.35 }; ///< Distance between front and rear wheel axles in meters.
    math::Vec2 frontWheelOffset { 0.675, -0.35 }; ///< Front axle rest position in local frame in meters.
    math::Vec2 rearWheelOffset { -0.675, -0.35 }; ///< Rear axle rest position in local frame in meters.
    double wheelRadius { 0.35 }; ///< Tire radius in meters.

    double maxDriveTorque { 280.0 }; ///< Peak engine torque delivered to rear driven wheel in N*m.
    double frontBrakeTorque { 450.0 }; ///< Front hydraulic disc brake maximum capacity in N*m.
    double rearBrakeTorque { 350.0 }; ///< Rear hydraulic disc brake maximum capacity in N*m.

    double suspensionStiffness { 18000.0 }; ///< Telescopic fork and rear shock spring rate in N/m.
    double suspensionDamping { 1600.0 }; ///< Suspension damping coefficient in N*s/m.

    double riderMaxLeanX { 0.40 }; ///< Maximum rider center-of-mass longitudinal displacement in meters.
    double airPitchTorque { 220.0 }; ///< In-air gyroscopic pitch torque reaction from wheel spin in N*m.
    double surfaceFriction { 1.1 }; ///< Tire-ground contact friction coefficient.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity acceleration vector in m/s^2.
};

/// @struct TrialsBikeInput
/// @brief Rider control commands for trials motorcycle dynamics.
struct TrialsBikeInput {
    double throttle { 0.0 }; ///< Rear wheel engine throttle in [0.0, 1.0].
    double frontBrake { 0.0 }; ///< Front wheel brake lever in [0.0, 1.0].
    double rearBrake { 0.0 }; ///< Rear wheel brake pedal in [0.0, 1.0].
    double riderLean { 0.0 }; ///< Rider weight shift command in [-1.0, 1.0] (+1 forward, -1 back).
    double groundDistanceFront { 0.35 }; ///< Front tire ground surface clearance in meters.
    double groundDistanceRear { 0.35 }; ///< Rear tire ground surface clearance in meters.
};

/// @struct TrialsBikeTelemetry
/// @brief Diagnostic snapshot of internal 2-wheel trials motorcycle states.
struct TrialsBikeTelemetry {
    double forwardSpeed { 0.0 }; ///< Chassis forward longitudinal velocity in m/s.
    double frontNormalForce { 0.0 }; ///< Ground normal reaction force on front tire in Newtons.
    double rearNormalForce { 0.0 }; ///< Ground normal reaction force on rear tire in Newtons.
    bool frontContact { true }; ///< True if front tire is in compression contact with ground.
    bool rearContact { true }; ///< True if rear tire is in compression contact with ground.
    bool isWheelie { false }; ///< True if performing a rear-wheel wheelie (front wheel off ground).
    bool isStoppie { false }; ///< True if performing a front-wheel stoppie (rear wheel off ground).
    bool isAirborne { false }; ///< True if both wheels are completely detached from ground.
    double riderCgShift { 0.0 }; ///< Active longitudinal center-of-mass displacement in meters.
};

/// @class TrialsBikeDynamics
/// @brief 2-wheel trials motorcycle dynamics coordinator.
/// @details Simulates independent front/rear suspension normal forces, dynamic rider CG shift
/// inducing wheelies and stoppies, traction/braking forces, and in-air wheel gyroscopic reaction.
class TrialsBikeDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline trials bike configuration.
    TrialsBikeDynamics() noexcept;

    /// @brief Parameterized constructor with custom trials bike configuration.
    /// @param[in] config Trials motorcycle configuration parameters.
    explicit TrialsBikeDynamics(const TrialsBikeConfig& config) noexcept;

    /// @brief Advances motorcycle simulation step by timestep dt.
    /// @details Computes rider lean moment, suspension contact forces, drive/brake traction,
    /// in-air wheel gyroscopic reactions, and applies accumulated forces to rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and contact telemetry.
    void reset() noexcept override;

    /// @brief Queries whether the motorcycle is airborne.
    /// @return True if both front and rear tires are off the ground.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active motorcycle configuration.
    /// @return Const reference to active TrialsBikeConfig.
    [[nodiscard]] const TrialsBikeConfig& getConfig() const noexcept;

    /// @brief Updates active motorcycle configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const TrialsBikeConfig& config) noexcept;

    /// @brief Gets current rider control inputs.
    /// @return Const reference to active TrialsBikeInput.
    [[nodiscard]] const TrialsBikeInput& getInput() const noexcept;

    /// @brief Sets rider control inputs.
    /// @param[in] input New rider control commands.
    void setInput(const TrialsBikeInput& input) noexcept;

    /// @brief Gets diagnostic telemetry snapshot.
    /// @return Const reference to TrialsBikeTelemetry.
    [[nodiscard]] const TrialsBikeTelemetry& getTelemetry() const noexcept;

    /// @brief Queries whether bike is currently performing a wheelie.
    /// @return True if rear wheel has contact and front wheel is airborne.
    [[nodiscard]] bool isWheelie() const noexcept;

    /// @brief Queries whether bike is currently performing a stoppie.
    /// @return True if front wheel has contact and rear wheel is airborne.
    [[nodiscard]] bool isStoppie() const noexcept;

    /// @brief Gets forward longitudinal velocity.
    /// @return Speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    TrialsBikeConfig mConfig { };
    TrialsBikeInput mInput { };
    TrialsBikeTelemetry mTelemetry { };
};

} // namespace vehicle_sim::dynamics
