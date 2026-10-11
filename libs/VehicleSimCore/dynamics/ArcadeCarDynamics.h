#pragma once

#include <array>
#include <cstddef>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum DriveType
/// @brief Powertrain wheel distribution classification.
enum class DriveType {
    FrontWheelDrive, ///< Power delivered strictly to front wheels.
    RearWheelDrive, ///< Power delivered strictly to rear wheels.
    AllWheelDrive ///< Power distributed equally across all four wheels.
};

/// @enum HandbrakeState
/// @brief Handbrake engagement state.
enum class HandbrakeState {
    Released, ///< Handbrake disengaged.
    Engaged ///< Handbrake applied, scaling rear lateral grip and braking.
};

/// @enum NitroState
/// @brief Nitro boost ignition state.
enum class NitroState {
    Inactive, ///< Nitro boost off.
    Active ///< Nitro boost firing, applying forward thrust impulse.
};

/// @struct ArcadeCarInput
/// @brief Semantic driver control inputs for the arcade vehicle coordinator.
struct ArcadeCarInput {
    double throttle { 0.0 }; ///< Forward / reverse input in [-1.0, 1.0].
    double steer { 0.0 }; ///< Steering input in [-1.0, 1.0] (positive = left).
    double brake { 0.0 }; ///< Service brake input in [0.0, 1.0].
    HandbrakeState handbrake { HandbrakeState::Released }; ///< Handbrake state.
    NitroState nitro { NitroState::Inactive }; ///< Nitro boost activation.
};

/// @struct ArcadeCarConfig
/// @brief Physical and mechanical configuration parameters for arcade car dynamics.
struct ArcadeCarConfig {
    double wheelbase { 2.5 }; ///< Distance between front and rear axles in meters.
    double trackWidth { 1.5 }; ///< Distance between left and right wheels in meters.
    DriveType driveType { DriveType::AllWheelDrive }; ///< Drivetrain wheel distribution.

    double maxDriveForce { 5000.0 }; ///< Maximum forward traction force in Newtons.
    double maxBrakeForce { 8000.0 }; ///< Maximum braking deceleration force in Newtons.
    double maxReverseForce { 3000.0 }; ///< Maximum reverse drive force in Newtons.

    double maxSteerAngle { 0.55 }; ///< Maximum steering angle in radians (~31.5 deg).
    double steerSpeed { 6.0 }; ///< Steering angle rate of change in rad/s.

    double frontLateralGrip { 1600.0 }; ///< Front wheel lateral grip damping in N/(m/s).
    double rearLateralGrip { 1700.0 }; ///< Rear wheel lateral grip damping in N/(m/s).
    double handbrakeGripScale { 0.25 }; ///< Rear lateral grip multiplier during drift.
    double handbrakeBrakeForce { 4500.0 }; ///< Braking force applied by handbrake in N.

    double rollingResistance { 25.0 }; ///< Rolling resistance damping in N/(m/s).
    double linearDragCoefficient { 0.35 }; ///< Aerodynamic drag factor in kg/m.

    double nitroForce { 6000.0 }; ///< Forward boost thrust force in Newtons.
    double maxNitroDuration { 3.0 }; ///< Maximum nitro fuel capacity in seconds.
    double nitroRechargeRate { 0.5 }; ///< Nitro fuel replenishment rate in seconds/second.

    double jumpVerticalSpeed { 5.0 }; ///< Initial vertical jump velocity in m/s.
    double gravityZ { 9.81 }; ///< Vertical gravity acceleration in m/s^2.
};

/// @struct WheelTelemetry
/// @brief Diagnostic telemetry snapshot for an individual wheel.
struct WheelTelemetry {
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Offset in vehicle local chassis coordinates.
    double steerAngle { 0.0 }; ///< Steer angle relative to body forward in radians.
    math::Vec2 worldPosition { 0.0, 0.0 }; ///< Wheel contact position in world coordinates.
    math::Vec2 velocity { 0.0, 0.0 }; ///< Contact point velocity in world coordinates.
    double longitudinalVelocity { 0.0 }; ///< Velocity along wheel forward axis in m/s.
    double lateralVelocity { 0.0 }; ///< Velocity along wheel sideways axis in m/s.
    math::Vec2 force { 0.0, 0.0 }; ///< Ground contact force vector in world coordinates.
    double surfaceGrip { 1.0 }; ///< Local surface friction scale factor.
    bool isDriven { false }; ///< True if powertrain delivers torque to this wheel.
    bool isSteering { false }; ///< True if wheel articulates with steering input.
};

/// @class ArcadeCarDynamics
/// @brief Top-down arcade and off-road vehicle dynamics coordinator.
/// @details Implements Pat Kerr's velocity-decomposed tire model with 4 independent
/// contact points, drift scaling, surface friction modifiers, nitro, and jumps.
class ArcadeCarDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kWheelCount { 4 };
    static constexpr std::size_t kFrontLeft { 0 };
    static constexpr std::size_t kFrontRight { 1 };
    static constexpr std::size_t kRearLeft { 2 };
    static constexpr std::size_t kRearRight { 3 };

    /// @brief Default constructor with baseline parameters.
    ArcadeCarDynamics() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Vehicle configuration parameters.
    explicit ArcadeCarDynamics(const ArcadeCarConfig& config) noexcept;

    /// @brief Destructor.
    ~ArcadeCarDynamics() noexcept override = default;

    /// @brief Advances arcade car simulation by timestep dt.
    /// @param[in,out] body Target rigid body to receive ground and boost forces.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states.
    /// @details Resets nitro fuel, jump height, steer angle, and wheel telemetry.
    void reset() noexcept override;

    /// @brief Queries whether vehicle is currently airborne.
    /// @return True if vehicle vertical height > 0.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets semantic driver control inputs.
    /// @param[in] input Control input commands.
    void setInput(const ArcadeCarInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Active ArcadeCarInput.
    [[nodiscard]] const ArcadeCarInput& getInput() const noexcept;

    /// @brief Gets vehicle configuration.
    /// @return Active ArcadeCarConfig.
    [[nodiscard]] const ArcadeCarConfig& getConfig() const noexcept;

    /// @brief Sets vehicle configuration and recalculates wheel geometry.
    /// @param[in] config New configuration parameters.
    void setConfig(const ArcadeCarConfig& config) noexcept;

    /// @brief Sets global terrain friction and drag modifiers.
    /// @param[in] gripMultiplier Ground grip factor (e.g. 1.0 tarmac, 0.7 dirt, 0.2 ice).
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

    /// @brief Gets remaining nitro boost fuel duration in seconds.
    /// @return Nitro fuel level in seconds.
    [[nodiscard]] double getNitroFuel() const noexcept;

    /// @brief Gets current front wheel steering angle in radians.
    /// @return Steering angle in radians.
    [[nodiscard]] double getSteerAngle() const noexcept;

    /// @brief Gets current forward speed in m/s.
    /// @return Longitudinal forward speed scalar in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Gets diagnostic telemetry for a specific wheel.
    /// @param[in] index Wheel index in [0, 3].
    /// @return WheelTelemetry reference.
    [[nodiscard]] const WheelTelemetry& getWheelTelemetry(std::size_t index) const noexcept;

    /// @brief Queries whether nitro booster is actively firing.
    /// @return True if nitro is firing and fuel remains.
    [[nodiscard]] bool isNitroActive() const noexcept;

    /// @brief Queries whether vehicle is experiencing significant rear lateral drift.
    /// @return True if rear lateral slip speed exceeds drift threshold.
    [[nodiscard]] bool isDrifting() const noexcept;

private:
    void updateWheelLayout() noexcept;
    void updateSteering(double dt) noexcept;
    void updateFlight(double dt) noexcept;
    void updateNitro(double dt) noexcept;

    ArcadeCarConfig mConfig { };
    ArcadeCarInput mInput { };

    std::array<WheelTelemetry, kWheelCount> mWheels { };
    std::array<double, kWheelCount> mWheelGripModifiers { 1.0, 1.0, 1.0, 1.0 };

    double mGlobalGripModifier { 1.0 };
    double mGlobalDragModifier { 1.0 };

    double mSteerAngle { 0.0 };
    double mNitroFuel { 3.0 };
    double mFlightZ { 0.0 };
    double mFlightVz { 0.0 };
    double mForwardSpeed { 0.0 };
    bool mNitroFiring { false };
};

} // namespace vehicle_sim::dynamics
