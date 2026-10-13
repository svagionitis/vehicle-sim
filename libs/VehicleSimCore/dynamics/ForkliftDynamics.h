#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct ForkliftConfig
/// @brief Physical, geometric, and stability configuration parameters for ForkliftDynamics.
struct ForkliftConfig {
    double chassisMass { 3500.0 }; ///< Unladen chassis and counterweight mass in kg.
    double chassisInertia { 4200.0 }; ///< Chassis yaw moment of inertia in kg*m^2.
    double wheelbase { 1.8 }; ///< Longitudinal wheelbase L (front axle to rear pivot) in m.
    double frontTrackWidth { 1.1 }; ///< Front drive axle track width in meters.
    double chassisCogDistFront { 0.8 }; ///< Distance from front axle to unladen COG in meters.
    double chassisCogHeight { 0.55 }; ///< Unladen chassis COG elevation above ground in meters.
    double forkDistFront { 0.5 }; ///< Cantilever distance of fork load ahead of front axle in m.
    double baseForkHeight { 0.15 }; ///< Ground clearance of carriage at lowest mast position in m.
    double maxMastHeight { 4.5 }; ///< Maximum hydraulic mast lift height in meters.
    double maxSteerAngle { 1.48 }; ///< Maximum rear wheel steer angle in radians (~85 deg).
    double maxDriveForce { 9000.0 }; ///< Maximum front axle tractive force in Newtons.
    double maxBrakeForce { 15000.0 }; ///< Maximum front axle braking force in Newtons.
    double maxSpeed { 5.5 }; ///< Factory speed governor in m/s (~20 km/h).
    double lateralGripCoeff { 18000.0 }; ///< Tire lateral cornering stiffness per axle in N/rad.
};

/// @class ForkliftDynamics
/// @brief Rear-wheel steer industrial counterbalanced forklift dynamics coordinator.
/// @details Simulates rear-wheel pivot steering up to +/- 90 degrees, dynamic mast height
/// Center of Gravity shifting, stability triangle margins, and rollover indices.
class ForkliftDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor with baseline industrial forklift parameters.
    ForkliftDynamics() noexcept;

    /// @brief Parameterized constructor using custom forklift configuration.
    /// @param[in] config Vehicle geometry, mass, and mast configurations.
    explicit ForkliftDynamics(const ForkliftConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~ForkliftDynamics() noexcept override = default;

    /// @brief Advances simulation step, applying traction and steering forces.
    /// @param[in,out] body Target rigid body representing forklift chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets operator commands and internal states.
    void reset() noexcept override;

    /// @brief Queries whether vehicle has lost ground contact or tipped over.
    /// @return True if vehicle is tipped over or airborne.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets accelerator pedal input.
    /// @param[in] throttle Normalized throttle command in range [0.0, 1.0].
    void setThrottle(double throttle) noexcept;

    /// @brief Sets service brake pedal input.
    /// @param[in] brake Normalized brake command in range [0.0, 1.0].
    void setBrake(double brake) noexcept;

    /// @brief Sets rear steering command.
    /// @param[in] steer Normalized steering command in range [-1.0, 1.0] (positive = left).
    void setSteer(double steer) noexcept;

    /// @brief Sets hydraulic mast extension height.
    /// @param[in] height Mast height in meters [0.0, maxMastHeight].
    void setMastHeight(double height) noexcept;

    /// @brief Sets payload mass currently carried on the forks.
    /// @param[in] mass Payload mass in kg.
    void setPayloadMass(double mass) noexcept;

    /// @brief Sets reverse drive toggle.
    /// @param[in] reverse True for reverse gear, false for forward.
    void setReverse(bool reverse) noexcept;

    /// @brief Retrieves the current dynamic lateral rollover index.
    /// @return Rollover index [0.0 = centered, 1.0 = wheel lift-off/rollover].
    [[nodiscard]] double getRolloverIndex() const noexcept;

    /// @brief Retrieves the normal force on the rear steering pivot.
    /// @return Vertical load in Newtons (<= 0 implies forward tipping).
    [[nodiscard]] double getRearPivotLoad() const noexcept;

    /// @brief Queries current composite Center of Gravity height.
    /// @return Composite COG elevation above ground in meters.
    [[nodiscard]] double getCompositeCogHeight() const noexcept;

    /// @brief Queries whether vehicle has experienced forward or lateral tip-over.
    /// @return True if tipped over beyond the stability triangle.
    [[nodiscard]] bool isTippedOver() const noexcept;

    /// @brief Queries active vehicle configuration.
    /// @return Const reference to ForkliftConfig.
    [[nodiscard]] const ForkliftConfig& getConfig() const noexcept;

private:
    ForkliftConfig mConfig {};
    double mThrottle { 0.0 };
    double mBrake { 0.0 };
    double mSteer { 0.0 };
    double mMastHeight { 0.0 };
    double mPayloadMass { 0.0 };
    double mRolloverIndex { 0.0 };
    double mRearPivotLoad { 0.0 };
    double mCompositeCogHeight { 0.55 };
    double mCompositeCogDistFront { 0.8 };
    double mPrevLongVel { 0.0 };
    bool mReverse { false };
    bool mIsTippedOver { false };
    bool mIsAirborne { false };

    void updateCenterOfGravity() noexcept;
    void evaluateStability(double ax, double ay) noexcept;
};

} // namespace vehicle_sim::dynamics
