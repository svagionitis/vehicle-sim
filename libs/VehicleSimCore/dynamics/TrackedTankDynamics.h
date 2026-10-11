#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct TrackedTankConfig
/// @brief Configuration parameters for tracked vehicle skid-steer dynamics.
/// @details Defines track geometry, maximum drive forces, track velocity bounds,
/// skid scrubbing friction resistance, turret recoil impulse, and surface traction.
struct TrackedTankConfig {
    double mass { 12000.0 }; ///< Chassis structural mass in kg.
    double trackWidth { 2.4 }; ///< Lateral tread center-to-center gauge in meters.
    double maxTrackForce { 40000.0 }; ///< Peak tractive force capacity per track in Newtons.
    double maxTrackSpeed { 14.0 }; ///< Peak linear track velocity in m/s.
    double driveStiffness { 12000.0 }; ///< Tractive slip-to-force stiffness coefficient in N*s/m.
    double lateralGrip { 35000.0 }; ///< Lateral skid-resisting friction stiffness in N*s/m.
    double maxLateralForce { 60000.0 }; ///< Peak lateral friction force limit in Newtons.
    double skidTorque { 15000.0 }; ///< Skid-steer ground scrubbing resistance torque in N*m.
    double linearDrag { 45.0 }; ///< Aerodynamic/mechanical rolling drag coefficient in N/(m/s)^2.
    double angularDamping { 2500.0 }; ///< Rotational damping moment in N*m*s.

    double recoilForce { 85000.0 }; ///< Turret main gun recoil force impulse magnitude in Newtons.
    double recoilDuration { 0.08 }; ///< Recoil force pulse duration in seconds.
    math::Vec2 turretOffset { 0.2, 0.0 }; ///< Local mount position of turret pivot in meters.

    double surfaceFriction { 1.0 }; ///< Terrain friction multiplier in [0.0, 2.0].
};

/// @struct TrackedTankInput
/// @brief Pilot and gunner control commands for tracked vehicle dynamics.
struct TrackedTankInput {
    double leftTrack { 0.0 }; ///< Commanded left track throttle in [-1.0, 1.0].
    double rightTrack { 0.0 }; ///< Commanded right track throttle in [-1.0, 1.0].
    bool fireRecoil { false }; ///< Recoil trigger command activating cannon recoil pulse.
    double turretAngle { 0.0 }; ///< Turret orientation relative to chassis heading in radians.
};

/// @struct TrackedTankTelemetry
/// @brief Diagnostic snapshot of internal tracked vehicle dynamics states.
struct TrackedTankTelemetry {
    double leftTrackSpeed { 0.0 }; ///< Current kinematic linear velocity of left track in m/s.
    double rightTrackSpeed { 0.0 }; ///< Current kinematic linear velocity of right track in m/s.
    double leftTrackForce { 0.0 }; ///< Current longitudinal tractive force of left track in Newtons.
    double rightTrackForce { 0.0 }; ///< Current longitudinal tractive force of right track in Newtons.
    double skidTorque { 0.0 }; ///< Current ground scrubbing resistive torque in N*m.
    double lateralForce { 0.0 }; ///< Current lateral tire/tread shear friction force in Newtons.
    math::Vec2 recoilForce { 0.0, 0.0 }; ///< Current active cannon recoil force vector in Newtons.
    double recoilTimer { 0.0 }; ///< Remaining active recoil pulse duration in seconds.
    bool isRecoiling { false }; ///< True while cannon recoil force pulse is actively firing.
    double forwardSpeed { 0.0 }; ///< Chassis forward longitudinal velocity in m/s.
    double yawRate { 0.0 }; ///< Chassis angular yaw velocity in rad/s.
};

/// @class TrackedTankDynamics
/// @brief 2D tracked vehicle dynamics coordinator implementing differential skid-steer.
/// @details Models independent dual-track velocity kinematics, ground scrubbing friction torque,
/// zero-radius neutral turning, lateral tread shear limits, and off-center turret cannon recoil.
class TrackedTankDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline tank configuration.
    TrackedTankDynamics() noexcept;

    /// @brief Parameterized constructor with custom tank configuration.
    /// @param[in] config Tracked vehicle configuration parameters.
    explicit TrackedTankDynamics(const TrackedTankConfig& config) noexcept;

    /// @brief Advances tank simulation step by timestep dt.
    /// @details Translates dual-tread throttles into tractive forces, computes skid scrubbing,
    /// applies cannon recoil pulses, and updates accumulated forces on rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and recoil states.
    void reset() noexcept override;

    /// @brief Queries whether the tank is airborne.
    /// @return True if tank is airborne, false when in contact with ground.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active tank configuration.
    /// @return Const reference to active TrackedTankConfig.
    [[nodiscard]] const TrackedTankConfig& getConfig() const noexcept;

    /// @brief Updates active tank configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const TrackedTankConfig& config) noexcept;

    /// @brief Gets current pilot control inputs.
    /// @return Const reference to active TrackedTankInput.
    [[nodiscard]] const TrackedTankInput& getInput() const noexcept;

    /// @brief Sets pilot control inputs.
    /// @param[in] input New pilot control commands.
    void setInput(const TrackedTankInput& input) noexcept;

    /// @brief Triggers an instantaneous cannon fire recoil impulse.
    void fireRecoil() noexcept;

    /// @brief Sets terrain surface traction multiplier.
    /// @param[in] friction Surface friction multiplier.
    void setSurfaceFriction(double friction) noexcept;

    /// @brief Gets diagnostic telemetry snapshot.
    /// @return Const reference to TrackedTankTelemetry.
    [[nodiscard]] const TrackedTankTelemetry& getTelemetry() const noexcept;

    /// @brief Gets forward longitudinal velocity.
    /// @return Chassis forward speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Gets angular yaw rate.
    /// @return Chassis angular velocity in rad/s.
    [[nodiscard]] double getYawRate() const noexcept;

private:
    TrackedTankConfig mConfig { };
    TrackedTankInput mInput { };
    TrackedTankTelemetry mTelemetry { };
    double mRecoilTimer { 0.0 };
    double mRecoilAngle { 0.0 };
};

} // namespace vehicle_sim::dynamics
