#pragma once

#include <array>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/GroundEffectAero2D.h"
#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct RaceCarConfig
/// @brief Physical and aerodynamic configuration parameters for RaceCarDynamics.
struct RaceCarConfig {
    double mass { 750.0 }; ///< Chassis mass in kg.
    double inertia { 900.0 }; ///< Yaw moment of inertia in kg*m^2.
    double wheelbase { 3.2 }; ///< Total wheelbase L = (cogDistFront + cogDistRear) in meters.
    double cogDistFront { 1.7 }; ///< Longitudinal distance a from COG to front axle in meters.
    double trackWidth { 1.6 }; ///< Lateral track width T in meters.
    double cogHeight { 0.32 }; ///< Center of gravity height above ground in meters.
    double rollStiffnessFront { 0.55 }; ///< Front roll stiffness distribution ratio [0..1].
    double brakeBiasFront { 0.58 }; ///< Front brake torque distribution ratio [0..1].
    double maxDriveForce { 9500.0 }; ///< Maximum tractive force in Newtons.
    double maxBrakeForce { 18000.0 }; ///< Maximum total braking force in Newtons.
    double maxSteerAngle { 0.45 }; ///< Maximum front wheel steer angle in radians (~25.8 deg).
    double rollingResistance { 0.015 }; ///< Tire rolling resistance coefficient.
    PacejkaConfig tireConfig {}; ///< Pacejka Magic Formula tire configuration.
    GroundEffectAeroConfig aeroConfig {}; ///< Ground effect and active DRS configuration.
};

/// @struct RaceCarWheelState
/// @brief Diagnostic telemetry and dynamic state for an individual racecar wheel.
struct RaceCarWheelState {
    double normalLoad { 0.0 }; ///< Dynamic vertical normal load F_z in Newtons.
    double slipRatio { 0.0 }; ///< Longitudinal slip ratio kappa.
    double slipAngle { 0.0 }; ///< Lateral cornering slip angle alpha in radians.
    double tractiveForce { 0.0 }; ///< Longitudinal force in wheel heading frame in Newtons.
    double corneringForce { 0.0 }; ///< Lateral force in wheel lateral frame in Newtons.
    double steerAngle { 0.0 }; ///< Steer angle in radians relative to chassis heading.
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Body-local coordinate offset.
};

/// @class RaceCarDynamics
/// @brief High-performance motorsport closed-circuit vehicle dynamics coordinator.
/// @details Couples 4-wheel dynamic load transfer, Ackermann steering, Pacejka combined
/// slip friction ellipses, underbody ground effect Venturi downforce, and active DRS flaps.
class RaceCarDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline formula-style racecar parameters.
    RaceCarDynamics() noexcept;

    /// @brief Parameterized constructor using custom racecar configuration.
    /// @param[in] config Vehicle mass, geometric, and aerodynamic parameters.
    explicit RaceCarDynamics(const RaceCarConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~RaceCarDynamics() noexcept override = default;

    /// @brief Advances racecar simulation step, applying forces to rigid body.
    /// @param[in,out] body Target rigid body representing racecar chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states, speeds, and controllers.
    void reset() noexcept override;

    /// @brief Queries whether the vehicle is airborne.
    /// @return True if all wheels have zero normal load or vehicle is airborne.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets driver throttle input.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    void setThrottle(double throttle) noexcept;

    /// @brief Sets driver service brake input.
    /// @param[in] brake Normalized brake command in range [0.0, 1.0].
    void setBrake(double brake) noexcept;

    /// @brief Sets driver steering command.
    /// @param[in] steer Normalized steering command in range [-1.0, 1.0] (positive = left).
    void setSteer(double steer) noexcept;

    /// @brief Sets active DRS flap actuation request.
    /// @param[in] active True to command DRS opening on straights.
    void setDrsActive(bool active) noexcept;

    /// @brief Sets measured underbody ground clearance beneath front and rear axles.
    /// @param[in] frontHeight Front axle ride height in meters.
    /// @param[in] rearHeight Rear axle ride height in meters.
    void setRideHeight(double frontHeight, double rearHeight) noexcept;

    /// @brief Sets airborne status override.
    /// @param[in] airborne True if vehicle has launched off ground.
    void setAirborne(bool airborne) noexcept;

    /// @brief Retrieves state of all four wheels (FL, FR, RL, RR).
    /// @return Reference to 4-element array of wheel states.
    [[nodiscard]] const std::array<RaceCarWheelState, 4>& getWheelStates() const noexcept;

    /// @brief Retrieves current total aerodynamic downforce in Newtons.
    /// @return Downforce magnitude in Newtons.
    [[nodiscard]] double getDownforce() const noexcept;

    /// @brief Queries current active DRS flap opening angle in degrees.
    /// @return Flap angle in degrees.
    [[nodiscard]] double getDrsAngleDeg() const noexcept;

    /// @brief Queries active vehicle configuration.
    /// @return Const reference to RaceCarConfig.
    [[nodiscard]] const RaceCarConfig& getConfig() const noexcept;

private:
    RaceCarConfig mConfig {};
    GroundEffectAero2D mAero {};
    std::array<PacejkaTireModel, 4> mTires {};
    std::array<RaceCarWheelState, 4> mWheels {};

    double mThrottle { 0.0 };
    double mBrake { 0.0 };
    double mSteer { 0.0 };
    double mFrontRideHeight { 0.045 };
    double mRearRideHeight { 0.050 };
    double mPrevLongVel { 0.0 };
    double mPrevLatVel { 0.0 };
    double mCurrentDownforce { 0.0 };
    double mCurrentDrsAngle { 0.0 };
    bool mDrsRequested { false };
    bool mIsAirborne { false };

    void setupWheelOffsets() noexcept;
    void computeWheelLoads(double ax, double ay, double downforce) noexcept;
};

} // namespace vehicle_sim::dynamics
