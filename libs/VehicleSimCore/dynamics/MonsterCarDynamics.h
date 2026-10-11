#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum MonsterDriveType
/// @brief Powertrain wheel distribution classification for semi-realistic dynamics.
enum class MonsterDriveType {
    FrontWheelDrive, ///< Power delivered strictly to front wheels.
    RearWheelDrive, ///< Power delivered strictly to rear wheels.
    AllWheelDrive ///< Power distributed equally across all four wheels.
};

/// @struct EngineConfig
/// @brief Internal combustion engine torque curve and speed limits.
struct EngineConfig {
    double idleRPM { 1000.0 }; ///< Minimum engine rotational idle speed in RPM.
    double redlineRPM { 6500.0 }; ///< Maximum engine rotational redline speed in RPM.
    double peakTorqueRPM { 4400.0 }; ///< RPM at which peak engine torque is developed.
    double peakTorque { 450.0 }; ///< Peak engine torque in Newton-meters (N*m).
    double upshiftRPM { 5500.0 }; ///< Automatic transmission upshift threshold in RPM.
    double downshiftRPM { 2200.0 }; ///< Automatic transmission downshift threshold in RPM.
};

/// @struct TransmissionConfig
/// @brief Gearbox ratios, final drive differential, and transmission efficiency.
struct TransmissionConfig {
    double differentialRatio { 3.42 }; ///< Final drive differential reduction ratio.
    double transmissionEfficiency { 0.75 }; ///< Mechanical efficiency factor in [0.0, 1.0].
    double reverseRatio { -2.90 }; ///< Gear ratio for reverse gear.
    std::vector<double> gearRatios { 2.66, 1.78, 1.30, 1.00, 0.74, 0.50 }; ///< Forward ratios.
    bool isAutomatic { true }; ///< True if automatic transmission shifting is enabled.
};

/// @struct MonsterCarConfig
/// @brief Mechanical, aerodynamic, and tire parameters for semi-realistic dynamics.
struct MonsterCarConfig {
    double distCgToFront { 1.25 }; ///< Longitudinal distance b from CG to front axle in meters.
    double distCgToRear { 1.25 }; ///< Longitudinal distance c from CG to rear axle in meters.
    double trackWidth { 1.5 }; ///< Transverse distance between left and right wheels in meters.
    double cgHeight { 0.55 }; ///< Vertical center of gravity height h in meters.
    double wheelRadius { 0.34 }; ///< Effective loaded tire rolling radius R_w in meters.
    double gravity { 9.81 }; ///< Acceleration due to gravity in m/s^2.
    double mass { 1500.0 }; ///< Total vehicle mass in kg for weight transfer calculations.

    MonsterDriveType driveType { MonsterDriveType::RearWheelDrive }; ///< Driven wheels.

    double maxSteerAngle { 0.55 }; ///< Maximum steering angle in radians (~31.5 deg).
    double steerSpeed { 6.0 }; ///< Steering angle rate of change in rad/s.

    double frontCorneringStiffness { 35000.0 }; ///< Front cornering stiffness C_alpha in N/rad.
    double rearCorneringStiffness { 38000.0 }; ///< Rear cornering stiffness C_alpha in N/rad.
    double tireFrictionCoefficient { 1.0 }; ///< Nominal tire friction coefficient mu.

    double airDragCoefficient { 0.38 }; ///< Combined aerodynamic drag coefficient C_air in kg/m.
    double rollingResistanceCoeff { 12.0 }; ///< Rolling resistance coefficient C_rr in N/(m/s).

    double maxBrakeForce { 10000.0 }; ///< Total service braking force capacity in Newtons.
    double handbrakeBrakeForce { 5000.0 }; ///< Rear axle handbrake braking force in Newtons.

    double lowSpeedCutoff { 0.5 }; ///< Threshold in m/s below which slip angle is blended.

    double jumpVerticalSpeed { 5.0 }; ///< Initial vertical jump velocity in m/s.
    double gravityZ { 9.81 }; ///< Vertical flight gravity acceleration in m/s^2.

    EngineConfig engine { }; ///< Powertrain engine configuration.
    TransmissionConfig transmission { }; ///< Powertrain gearbox configuration.
};

/// @struct MonsterCarInput
/// @brief Semantic driver control inputs for the semi-realistic vehicle coordinator.
struct MonsterCarInput {
    double throttle { 0.0 }; ///< Throttle pedal position in [0.0, 1.0] (or [-1.0, 1.0]).
    double steer { 0.0 }; ///< Steering wheel input in [-1.0, 1.0] (positive = left).
    double brake { 0.0 }; ///< Service brake pedal position in [0.0, 1.0].
    double clutch { 0.0 }; ///< Clutch pedal position in [0.0, 1.0] (0.0 = engaged).
    bool handbrake { false }; ///< True if handbrake is actively applied.
};

/// @struct MonsterWheelTelemetry
/// @brief Diagnostic telemetry snapshot for an individual semi-realistic wheel.
struct MonsterWheelTelemetry {
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Local chassis offset (x=longitudinal, y=lateral).
    double steerAngle { 0.0 }; ///< Steering angle relative to chassis forward in radians.
    math::Vec2 worldPosition { 0.0, 0.0 }; ///< Contact point in world space.
    math::Vec2 velocity { 0.0, 0.0 }; ///< Contact point velocity in world space.
    double longitudinalVelocity { 0.0 }; ///< Velocity along wheel forward axis in m/s.
    double lateralVelocity { 0.0 }; ///< Velocity along wheel transverse axis in m/s.
    double slipAngle { 0.0 }; ///< Tire slip angle alpha in radians.
    double normalLoad { 0.0 }; ///< Normal load force F_z acting on tire in Newtons.
    double lateralForce { 0.0 }; ///< Lateral cornering force F_lat in Newtons.
    double longitudinalForce { 0.0 }; ///< Longitudinal traction/braking force in Newtons.
    math::Vec2 totalForce { 0.0, 0.0 }; ///< Resultant wheel force vector in world coordinates.
    double surfaceGrip { 1.0 }; ///< Surface friction multiplier at contact patch.
    bool isDriven { false }; ///< True if powertrain delivers drive torque to this wheel.
    bool isSteering { false }; ///< True if wheel articulates with steering inputs.
    bool isSaturated { false }; ///< True if tire force reached friction circle limit.
};

/// @class MonsterCarDynamics
/// @brief Semi-realistic vehicle dynamics coordinator based on Marco Monster's model.
/// @details Simulates dynamic longitudinal weight transfer, engine torque curve, multi-gear
/// transmission, aerodynamic drag, rolling resistance, slip angles, and Coulomb friction circles.
class MonsterCarDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kWheelCount { 4 };
    static constexpr std::size_t kFrontLeft { 0 };
    static constexpr std::size_t kFrontRight { 1 };
    static constexpr std::size_t kRearLeft { 2 };
    static constexpr std::size_t kRearRight { 3 };

    /// @brief Default constructor with baseline sedan configuration.
    MonsterCarDynamics() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Vehicle configuration parameters.
    explicit MonsterCarDynamics(const MonsterCarConfig& config) noexcept;

    /// @brief Destructor.
    ~MonsterCarDynamics() noexcept override = default;

    /// @brief Advances semi-realistic car dynamics simulation by timestep dt.
    /// @param[in,out] body Target rigid body to receive powertrain, tire, and aero forces.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states.
    /// @details Clears velocities, steer angle, jump height, gear to 1, and telemetry.
    void reset() noexcept override;

    /// @brief Queries whether vehicle is currently airborne.
    /// @return True if vehicle vertical height > 0.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets semantic driver control inputs.
    /// @param[in] input Control input commands.
    void setInput(const MonsterCarInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Active MonsterCarInput reference.
    [[nodiscard]] const MonsterCarInput& getInput() const noexcept;

    /// @brief Gets vehicle configuration.
    /// @return Active MonsterCarConfig reference.
    [[nodiscard]] const MonsterCarConfig& getConfig() const noexcept;

    /// @brief Sets vehicle configuration and recalculates wheel geometry.
    /// @param[in] config New configuration parameters.
    void setConfig(const MonsterCarConfig& config) noexcept;

    /// @brief Manually selects current transmission gear.
    /// @param[in] gear Selected gear (-1 for reverse, 0 for neutral, >= 1 for forward).
    void setGear(std::int32_t gear) noexcept;

    /// @brief Gets active transmission gear.
    /// @return Current gear index (-1 = reverse, 0 = neutral, 1..N = forward).
    [[nodiscard]] std::int32_t getGear() const noexcept;

    /// @brief Shifts transmission to next higher forward gear.
    void shiftUp() noexcept;

    /// @brief Shifts transmission to next lower gear.
    void shiftDown() noexcept;

    /// @brief Enables or disables automatic transmission shifting.
    /// @param[in] automatic True to enable automatic shifting.
    void setAutomatic(bool automatic) noexcept;

    /// @brief Queries whether automatic transmission shifting is active.
    /// @return True if automatic transmission is enabled.
    [[nodiscard]] bool isAutomatic() const noexcept;

    /// @brief Gets current rotational engine speed in RPM.
    /// @return Rotational speed in revolutions per minute.
    [[nodiscard]] double getEngineRPM() const noexcept;

    /// @brief Gets current output engine torque in Newton-meters.
    /// @return Engine torque in N*m.
    [[nodiscard]] double getEngineTorque() const noexcept;

    /// @brief Gets total drive force delivered to the driven wheels in Newtons.
    /// @return Total powertrain drive force in Newtons.
    [[nodiscard]] double getDriveForce() const noexcept;

    /// @brief Gets current dynamic normal load on front axle in Newtons.
    /// @return Total front axle normal load W_f in Newtons.
    [[nodiscard]] double getFrontAxleWeight() const noexcept;

    /// @brief Gets current dynamic normal load on rear axle in Newtons.
    /// @return Total rear axle normal load W_r in Newtons.
    [[nodiscard]] double getRearAxleWeight() const noexcept;

    /// @brief Gets current front wheel steering angle in radians.
    /// @return Steering angle in radians.
    [[nodiscard]] double getSteerAngle() const noexcept;

    /// @brief Gets current forward speed in m/s.
    /// @return Longitudinal forward speed scalar in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Gets current chassis longitudinal acceleration in m/s^2.
    /// @return Longitudinal acceleration in m/s^2.
    [[nodiscard]] double getAcceleration() const noexcept;

    /// @brief Queries whether vehicle is experiencing rear lateral slip saturation.
    /// @return True if rear wheels are saturated or slip angle exceeds drift threshold.
    [[nodiscard]] bool isDrifting() const noexcept;

    /// @brief Gets diagnostic telemetry for a specific wheel.
    /// @param[in] index Wheel index in [0, 3].
    /// @return MonsterWheelTelemetry reference.
    [[nodiscard]] const MonsterWheelTelemetry& getWheelTelemetry(std::size_t index) const noexcept;

    /// @brief Sets global terrain friction and rolling drag modifiers.
    /// @param[in] gripMultiplier Ground grip factor (1.0 = tarmac, 0.7 = dirt, 0.2 = ice).
    /// @param[in] dragMultiplier Rolling resistance factor.
    void setSurfaceModifier(double gripMultiplier, double dragMultiplier = 1.0) noexcept;

    /// @brief Sets individual wheel terrain grip modifier.
    /// @param[in] wheelIndex Wheel index in [0, 3].
    /// @param[in] gripMultiplier Wheel friction scale factor.
    void setWheelSurfaceModifier(std::size_t wheelIndex, double gripMultiplier) noexcept;

    /// @brief Triggers an airborne jump impulse.
    /// @param[in] verticalSpeed Initial vertical velocity in m/s (0 uses config speed).
    void triggerJump(double verticalSpeed = 0.0) noexcept;

    /// @brief Gets vertical flight height above ground in meters.
    /// @return Flight altitude in meters.
    [[nodiscard]] double getFlightHeight() const noexcept;

private:
    void updateWheelLayout() noexcept;
    void updateSteering(double dt) noexcept;
    void updateFlight(double dt) noexcept;
    void updateTransmission(double dt) noexcept;
    [[nodiscard]] double calculateTorque(double rpm) const noexcept;

    MonsterCarConfig mConfig { };
    MonsterCarInput mInput { };

    std::array<MonsterWheelTelemetry, kWheelCount> mWheels { };
    std::array<double, kWheelCount> mWheelGripModifiers { 1.0, 1.0, 1.0, 1.0 };

    double mGlobalGripModifier { 1.0 };
    double mGlobalDragModifier { 1.0 };

    std::int32_t mCurrentGear { 1 };
    double mSteerAngle { 0.0 };
    double mEngineRPM { 1000.0 };
    double mEngineTorque { 0.0 };
    double mDriveForce { 0.0 };
    double mFrontAxleWeight { 0.0 };
    double mRearAxleWeight { 0.0 };
    double mSpeed { 0.0 };
    double mPrevSpeed { 0.0 };
    double mAcceleration { 0.0 };
    double mFlightZ { 0.0 };
    double mFlightVz { 0.0 };
    bool mHasSpeedSample { false };
};

} // namespace vehicle_sim::dynamics
