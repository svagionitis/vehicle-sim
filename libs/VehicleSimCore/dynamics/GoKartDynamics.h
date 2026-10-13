#pragma once

#include <array>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct GoKartConfig
/// @brief Physical, frame flex, and steering parameters for GoKartDynamics.
struct GoKartConfig {
    double mass { 165.0 }; ///< Total kart mass including driver in kg.
    double inertia { 45.0 }; ///< Yaw moment of inertia in kg*m^2.
    double wheelbase { 1.04 }; ///< Wheelbase L in meters.
    double cogDistFront { 0.62 }; ///< Longitudinal distance from COG to front axle in meters.
    double trackWidthFront { 1.15 }; ///< Front track width in meters.
    double trackWidthRear { 1.38 }; ///< Rear track width in meters.
    double cogHeight { 0.22 }; ///< Low center of gravity height in meters.
    double maxSteerAngle { 0.48 }; ///< Maximum front spindle steer angle in radians (~27.5 deg).
    double maxDriveForce { 2200.0 }; ///< Maximum 2-stroke/4-stroke rear axle drive force in Newtons.
    double maxBrakeForce { 3800.0 }; ///< Maximum rear-axle hydraulic disc brake force in Newtons.
    double casterAngleRad { 0.24 }; ///< Aggressive kingpin caster angle in radians (~14 degrees).
    double kpiAngleRad { 0.21 }; ///< Kingpin inclination (KPI) angle in radians (~12 degrees).
    double jackingFactor { 1.35 }; ///< Frame flex caster jacking leverage multiplier.

    PacejkaConfig frontTireConfig {}; ///< Front kart slick tire configuration.
    PacejkaConfig rearTireConfig {}; ///< Rear wide kart slick tire configuration.
};

/// @struct GoKartWheelState
/// @brief Dynamic load and contact state for an individual go-kart wheel.
struct GoKartWheelState {
    double normalLoad { 0.0 }; ///< Dynamic vertical normal load in Newtons.
    double slipRatio { 0.0 }; ///< Longitudinal slip ratio kappa.
    double slipAngle { 0.0 }; ///< Lateral slip angle alpha in radians.
    double tractiveForce { 0.0 }; ///< Longitudinal force in Newtons.
    double corneringForce { 0.0 }; ///< Lateral force in Newtons.
    double steerAngle { 0.0 }; ///< Steer angle relative to chassis in radians.
    bool isLifted { false }; ///< True if wheel has unweighted and lifted off track.
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Body-local position offset.
};

/// @class GoKartDynamics
/// @brief Lightweight sprint go-kart vehicle dynamics coordinator.
/// @details Simulates solid spool rear drive axle kinematics (no differential),
/// kingpin caster jacking inside rear wheel lift-off (tripod cornering), and
/// direct 1:1 steering response.
class GoKartDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using standard competition sprint kart parameters.
    GoKartDynamics() noexcept;

    /// @brief Parameterized constructor using custom go-kart configuration.
    /// @param[in] config Kart mass, geometric, and frame flex parameters.
    explicit GoKartDynamics(const GoKartConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~GoKartDynamics() noexcept override = default;

    /// @brief Advances go-kart simulation step, applying forces to rigid body.
    /// @param[in,out] body Target rigid body representing go-kart chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states, speeds, and controllers.
    void reset() noexcept override;

    /// @brief Queries whether the kart is airborne.
    /// @return True if all wheels have lost ground contact.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets driver throttle pedal input.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    void setThrottle(double throttle) noexcept;

    /// @brief Sets driver brake pedal input.
    /// @param[in] brake Normalized brake command in range [0.0, 1.0].
    void setBrake(double brake) noexcept;

    /// @brief Sets direct 1:1 driver steering command.
    /// @param[in] steer Normalized steering command in range [-1.0, 1.0] (positive = left).
    void setSteer(double steer) noexcept;

    /// @brief Sets airborne status override.
    /// @param[in] airborne True if kart has launched off ground.
    void setAirborne(bool airborne) noexcept;

    /// @brief Queries whether the inside rear wheel has lifted off the track.
    /// @return True if tripod cornering state is active.
    [[nodiscard]] bool isInnerWheelLifted() const noexcept;

    /// @brief Retrieves the instantaneous rear solid spool axle binding scrub torque.
    /// @return Opposing yaw resistance torque in N*m (0.0 when inner wheel is lifted).
    [[nodiscard]] double getAxleScrubTorque() const noexcept;

    /// @brief Retrieves the caster jacking vertical load transfer delta in Newtons.
    /// @return Jacking force delta transferring load off inside rear wheel.
    [[nodiscard]] double getJackingForce() const noexcept;

    /// @brief Retrieves state of all four wheels (FL, FR, RL, RR).
    /// @return Const reference to 4-element array of wheel states.
    [[nodiscard]] const std::array<GoKartWheelState, 4>& getWheelStates() const noexcept;

    /// @brief Queries active go-kart configuration.
    /// @return Const reference to GoKartConfig.
    [[nodiscard]] const GoKartConfig& getConfig() const noexcept;

private:
    GoKartConfig mConfig {};
    std::array<PacejkaTireModel, 4> mTires {};
    std::array<GoKartWheelState, 4> mWheels {};

    double mThrottle { 0.0 };
    double mBrake { 0.0 };
    double mSteer { 0.0 };
    double mAxleScrubTorque { 0.0 };
    double mCurrentJackingForce { 0.0 };
    bool mIsInnerWheelLifted { false };
    bool mIsAirborne { false };

    void setupWheelOffsets() noexcept;
    void computeJackingLoads(double ay) noexcept;
};

} // namespace vehicle_sim::dynamics
