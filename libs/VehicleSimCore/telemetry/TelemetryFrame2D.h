#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::telemetry {

/// @struct WheelSample2D
/// @brief Diagnostic telemetry snapshot for an individual vehicle wheel or ground contact.
struct WheelSample2D {
    math::Vec2 position {}; ///< World contact position in meters.
    math::Vec2 contactForce {}; ///< Ground force vector in Newtons.
    double slipAngle { 0.0 }; ///< Lateral tire slip angle alpha in radians.
    double slipRatio { 0.0 }; ///< Longitudinal tire slip ratio kappa.
    double normalForce { 0.0 }; ///< Vertical reaction load Fz in Newtons.
    double suspensionTravel { 0.0 }; ///< Suspension deflection in meters.
    double surfaceFriction { 1.0 }; ///< Local terrain friction coefficient mu.
    bool isGrounded { true }; ///< True if contact surface is active.
};

/// @struct TelemetryFrame2D
/// @brief High-rate physical state snapshot sampled during a simulation step.
struct TelemetryFrame2D {
    static constexpr std::size_t kMaxWheels { 4 };
    static constexpr std::size_t kMaxAuxChannels { 8 };

    std::uint64_t tickNumber { 0 }; ///< Monotonic simulation step counter.
    double timestamp { 0.0 }; ///< Simulation time in seconds.

    // Kinematics & Spatial Motion
    math::Vec2 position {}; ///< Center of mass world position in meters.
    math::Vec2 velocity {}; ///< World linear velocity in m/s.
    double forwardSpeed { 0.0 }; ///< Longitudinal chassis speed in m/s.
    double heading { 0.0 }; ///< Chassis body orientation in radians.
    double angularVelocity { 0.0 }; ///< Chassis yaw rate in rad/s.

    // Accelerations & G-Forces
    math::Vec2 acceleration {}; ///< World linear acceleration in m/s^2.
    double longitudinalG { 0.0 }; ///< Forward/rearward load factor in Gs.
    double lateralG { 0.0 }; ///< Left/right cornering load factor in Gs.

    // Driver Actuation
    double throttleInput { 0.0 }; ///< Throttle demand in [0, 1].
    double steeringInput { 0.0 }; ///< Steer demand in [-1, 1].
    double brakeInput { 0.0 }; ///< Brake demand in [0, 1].
    double handbrakeInput { 0.0 }; ///< Handbrake demand in [0, 1].

    // Powertrain States
    double engineRpm { 0.0 }; ///< Powertrain shaft speed in RPM.
    std::int32_t activeGear { 0 }; ///< Engaged transmission gear index.
    double driveTorque { 0.0 }; ///< Total drivetrain output torque in Nm.

    // Contacts & Wheels
    std::size_t wheelCount { 0 }; ///< Active wheel count (<= kMaxWheels).
    std::array<WheelSample2D, kMaxWheels> wheels {}; ///< Per-wheel metrics.

    // Aerial / Fluid Diagnostics
    double altitude { 0.0 }; ///< Vertical elevation or flight height in meters.
    double verticalSpeed { 0.0 }; ///< Vertical ascent/descent rate in m/s.
    bool isAirborne { false }; ///< True if vehicle has broken ground contact.

    // Auxiliary Generic Telemetry
    std::array<double, kMaxAuxChannels> auxChannels {}; ///< Custom channels.

    /// @brief Resets all telemetry fields to default initial values.
    void reset() noexcept;
};

} // namespace vehicle_sim::telemetry
