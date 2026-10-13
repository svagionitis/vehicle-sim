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

/// @struct DriftCarConfig
/// @brief Physical, steering, and thermal configuration for DriftCarDynamics.
struct DriftCarConfig {
    double mass { 1250.0 }; ///< Chassis mass in kg.
    double inertia { 1800.0 }; ///< Yaw moment of inertia in kg*m^2.
    double wheelbase { 2.55 }; ///< Wheelbase L in meters.
    double cogDistFront { 1.35 }; ///< Distance from COG to front axle in meters.
    double trackWidth { 1.58 }; ///< Track width T in meters.
    double cogHeight { 0.45 }; ///< Center of gravity height in meters.
    double maxSteerAngle { 1.05 }; ///< Maximum front steer angle in radians (~60 degrees).
    double maxDriveForce { 8500.0 }; ///< Maximum rear-wheel tractive drive force in Newtons.
    double maxFootBrakeForce { 12000.0 }; ///< Maximum foot service brake force in Newtons.
    double maxHandbrakeForce { 16000.0 }; ///< Maximum hydraulic handbrake locking force in Newtons.
    double casterAngleRad { 0.14 }; ///< Front caster angle in radians (~8 degrees).
    double mechanicalTrail { 0.025 }; ///< Front mechanical trail lever arm in meters.
    double clutchKickTorque { 3500.0 }; ///< Instantaneous clutch kick torque surge in N*m.
    double clutchKickDuration { 0.20 }; ///< Duration of clutch kick impulse in seconds.

    // Thermal tire parameters
    double ambientTemperature { 25.0 }; ///< Ambient air temperature in Celsius.
    double optimalTireTemp { 85.0 }; ///< Optimal tire grip temperature in Celsius.
    double thermalCapacity { 1200.0 }; ///< Contact patch thermal capacity in J/K.
    double coolingRate { 0.08 }; ///< Convective cooling rate per second.
    double thermalFrictionDecay { 0.35 }; ///< Maximum friction degradation ratio when overheated.
    double tempScaleFactor { 40.0 }; ///< Temperature sensitivity scaling in Celsius.

    PacejkaConfig frontTireConfig {}; ///< High-grip front tire Pacejka settings.
    PacejkaConfig rearTireConfig {}; ///< Drift-compound rear tire Pacejka settings.
};

/// @struct DriftCarWheelState
/// @brief Diagnostic and thermal state for an individual drift car wheel.
struct DriftCarWheelState {
    double normalLoad { 0.0 }; ///< Dynamic vertical normal load in Newtons.
    double slipRatio { 0.0 }; ///< Longitudinal slip ratio kappa.
    double slipAngle { 0.0 }; ///< Lateral slip angle alpha in radians.
    double tractiveForce { 0.0 }; ///< Longitudinal force in Newtons.
    double corneringForce { 0.0 }; ///< Lateral force in Newtons.
    double temperature { 25.0 }; ///< Current contact patch temperature in Celsius.
    double effectiveMu { 1.0 }; ///< Thermal-degraded friction coefficient multiplier.
    double steerAngle { 0.0 }; ///< Steer angle relative to chassis in radians.
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Wheel offset in chassis body frame.
};

/// @class DriftCarDynamics
/// @brief Specialized drift motorsport vehicle dynamics coordinator.
/// @details Simulates extreme slip-angle oversteer equilibrium, pneumatic trail
/// self-aligning torque, hydraulic handbrake rear lockup, clutch kick transients,
/// and tire thermal friction degradation.
class DriftCarDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using standard sports car drift configuration.
    DriftCarDynamics() noexcept;

    /// @brief Parameterized constructor using custom drift configuration.
    /// @param[in] config Vehicle geometry, mass, and thermal parameters.
    explicit DriftCarDynamics(const DriftCarConfig& config) noexcept;

    /// @brief Virtual destructor ensuring clean polymorphic cleanup.
    ~DriftCarDynamics() noexcept override = default;

    /// @brief Advances drift car simulation step, applying forces to rigid body.
    /// @param[in,out] body Target rigid body representing drift car chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states, slip memories, and tire temperatures.
    void reset() noexcept override;

    /// @brief Queries whether the vehicle is airborne.
    /// @return True if all wheels have lost ground contact.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets driver throttle pedal command.
    /// @param[in] throttle Normalized throttle input in range [0.0, 1.0].
    void setThrottle(double throttle) noexcept;

    /// @brief Sets driver service foot brake pedal command.
    /// @param[in] brake Normalized foot brake input in range [0.0, 1.0].
    void setBrake(double brake) noexcept;

    /// @brief Sets driver steering wheel command.
    /// @param[in] steer Normalized steering command in range [-1.0, 1.0] (positive = left).
    void setSteer(double steer) noexcept;

    /// @brief Sets hydraulic handbrake (e-brake) actuation level.
    /// @param[in] handbrake Normalized handbrake input in range [0.0, 1.0].
    void setHandbrake(double handbrake) noexcept;

    /// @brief Triggers an instantaneous clutch kick impulse.
    void triggerClutchKick() noexcept;

    /// @brief Sets airborne status override.
    /// @param[in] airborne True if vehicle has launched off ground.
    void setAirborne(bool airborne) noexcept;

    /// @brief Retrieves current chassis body drift slip angle (beta) in radians.
    /// @return Drift angle beta = atan2(lateral_velocity, forward_velocity).
    [[nodiscard]] double getDriftAngle() const noexcept;

    /// @brief Queries front caster self-aligning counter-steering torque in N*m.
    /// @return Aligning torque tending to steer wheels into the skid.
    [[nodiscard]] double getAligningTorque() const noexcept;

    /// @brief Checks if vehicle is currently in a sustained drift state.
    /// @return True if drift angle exceeds threshold and rear wheels are sliding.
    [[nodiscard]] bool isDrifting() const noexcept;

    /// @brief Retrieves current contact patch temperatures for all four wheels.
    /// @return Array of temperatures in Celsius (FL, FR, RL, RR).
    [[nodiscard]] std::array<double, 4> getTireTemperatures() const noexcept;

    /// @brief Retrieves state of all four wheels (FL, FR, RL, RR).
    /// @return Const reference to 4-element array of wheel states.
    [[nodiscard]] const std::array<DriftCarWheelState, 4>& getWheelStates() const noexcept;

    /// @brief Queries active vehicle configuration.
    /// @return Const reference to DriftCarConfig.
    [[nodiscard]] const DriftCarConfig& getConfig() const noexcept;

private:
    DriftCarConfig mConfig {};
    std::array<PacejkaTireModel, 4> mTires {};
    std::array<DriftCarWheelState, 4> mWheels {};

    double mThrottle { 0.0 };
    double mBrake { 0.0 };
    double mSteer { 0.0 };
    double mHandbrake { 0.0 };
    double mClutchKickTimer { 0.0 };
    double mCurrentDriftAngle { 0.0 };
    double mCurrentAlignTorque { 0.0 };
    bool mIsAirborne { false };

    void setupWheelOffsets() noexcept;
    void updateThermalModel(double dt) noexcept;
    void computeWheelLoads(double ax, double ay) noexcept;
};

} // namespace vehicle_sim::dynamics
