#pragma once

#include <array>
#include <cstddef>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct MotionLabCarInput
/// @brief Semantic driver control inputs for the Motion Lab car coordinator.
struct MotionLabCarInput {
    double throttle { 0.0 };  ///< Forward/reverse throttle in [-1.0, 1.0].
    double steer { 0.0 };     ///< Steering input in [-1.0, 1.0] (positive = left).
    bool handbrake { false }; ///< Handbrake engagement flag.
};

/// @struct MotionLabCarConfig
/// @brief Physical and mechanical parameters for Pat Kerr's velocity-decomposed car dynamics.
struct MotionLabCarConfig {
    double mass { 1000.0 };              ///< Chassis mass in kg.
    double length { 4.0 };               ///< Chassis length in meters.
    double width { 2.0 };                ///< Chassis width in meters.
    double engineForce { 9750.0 };       ///< Base engine force (6500 * 1.5) in N.
    double reverseScale { 0.75 };        ///< Reverse throttle attenuation factor.
    double maxSteerAngle { 0.55 };       ///< Max front steer angle in radians (~31.5 deg).
    double rollingResistance { 32.5 };   ///< Base rolling resistance damping in N/(m/s).
    double lateralResistance { 1100.0 }; ///< Base lateral cornering damping in N/(m/s).
    double handbrakeGrip { 0.15 };       ///< Rear lateral grip scale when handbrake held.
    double handbrakeStrength { 0.75 };   ///< Handbrake effectiveness in [0.0, 1.0].
    double handbrakeRollingMult { 8.0 }; ///< Multiplier on rear rolling resistance.
    double gripRecoveryTime { 0.15 };    ///< Rear grip recovery duration in seconds.
};

/// @struct MotionLabTireTelemetry
/// @brief Snapshot diagnostic telemetry for an individual tire.
struct MotionLabTireTelemetry {
    math::Vec2 position { 0.0, 0.0 };     ///< Local chassis coordinate offset.
    math::Vec2 worldPosition { 0.0, 0.0 }; ///< World coordinates of tire contact point.
    double steerAngle { 0.0 };            ///< Current wheel steer angle in radians.
    bool steered { false };               ///< True if wheel steers (front axle).
    bool skidding { false };              ///< True if tire is actively sliding sideways.
    double lateralSpeed { 0.0 };          ///< Absolute sideways velocity in m/s.
    double rollingSpeed { 0.0 };          ///< Velocity along wheel heading in m/s.
    math::Vec2 force { 0.0, 0.0 };        ///< Total tire force in world coordinates.
};

/// @class MotionLabCarDynamics
/// @brief Faithful implementation of Pat Kerr's 1996 GTA vehicle physics prototype.
/// @details Evaluates velocity-decomposed tire rolling and lateral resistance at 4 contact
/// points without force caps, supporting handbrake power slides and transient grip recovery.
class MotionLabCarDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kWheelCount { 4 };
    static constexpr std::size_t kFrontLeft { 0 };
    static constexpr std::size_t kFrontRight { 1 };
    static constexpr std::size_t kRearLeft { 2 };
    static constexpr std::size_t kRearRight { 3 };

    /// @brief Default constructor with nominal Motion Lab configuration.
    MotionLabCarDynamics() noexcept;

    /// @brief Parameterized constructor using custom car parameters.
    /// @param[in] config Vehicle configuration settings.
    explicit MotionLabCarDynamics(const MotionLabCarConfig& config) noexcept;

    /// @brief Virtual destructor.
    ~MotionLabCarDynamics() noexcept override = default;

    /// @brief Advances car simulation step, applying tire and drive forces.
    /// @param[in,out] body Target chassis rigid body.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient dynamics states (rear grip loss, skidding).
    void reset() noexcept override;

    /// @brief Queries whether the vehicle is airborne.
    /// @return Always false for 2D Motion Lab ground car.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Sets driver control inputs.
    /// @param[in] input Semantic control inputs.
    void setInput(const MotionLabCarInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Current input struct.
    [[nodiscard]] const MotionLabCarInput& getInput() const noexcept;

    /// @brief Sets car configuration parameters.
    /// @param[in] config New configuration settings.
    void setConfig(const MotionLabCarConfig& config) noexcept;

    /// @brief Gets car configuration parameters.
    /// @return Current configuration struct.
    [[nodiscard]] const MotionLabCarConfig& getConfig() const noexcept;

    /// @brief Gets telemetry for all four wheels.
    /// @return Array of wheel telemetry structures.
    [[nodiscard]] const std::array<MotionLabTireTelemetry, kWheelCount>& getTires() const noexcept;

    /// @brief Queries whether any tire is currently skidding.
    /// @return True if at least one rear tire is sliding sideways.
    [[nodiscard]] bool isSkidding() const noexcept;

    /// @brief Gets rear grip loss fraction.
    /// @return Rear grip loss in [0.0, 1.0].
    [[nodiscard]] double getRearGripLoss() const noexcept;

    /// @brief Sets engine drive force in Newtons.
    /// @param[in] force Peak engine thrust force.
    void setEngineForce(double force) noexcept;

    /// @brief Sets handbrake strength multiplier.
    /// @param[in] strength Handbrake effectiveness in [0.0, 1.0].
    void setHandbrakeStrength(double strength) noexcept;

    /// @brief Computes the 4 vehicle boundary points in world space.
    /// @param[in] body Vehicle rigid body chassis.
    /// @return Array of 4 world-space corner points.
    [[nodiscard]] std::array<math::Vec2, 4> computeWorldPoints(const physics::RigidBody2D& body) const noexcept;

    /// @brief Finds the surface contact point facing inward boundary normal.
    /// @details If a flat face contacts the barrier, returns the midpoint of the vertices.
    /// @param[in] body Vehicle rigid body chassis.
    /// @param[in] normal Inward-facing boundary plane normal.
    /// @return Contact point in world coordinates.
    [[nodiscard]] math::Vec2 computeSupportPoint(const physics::RigidBody2D& body, const math::Vec2& normal) const noexcept;

private:
    void updateWheelLayout() noexcept;

    MotionLabCarConfig mConfig { };
    MotionLabCarInput mInput { };
    std::array<MotionLabTireTelemetry, kWheelCount> mTires { };
    double mRearGripLoss { 0.0 };
    bool mIsSkidding { false };
};

} // namespace vehicle_sim::dynamics
