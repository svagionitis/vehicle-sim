#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetryFrame2D.h"

namespace vehicle_sim::dynamics {
class IVehicleDynamics;
class ArcadeCarDynamics;
} // namespace vehicle_sim::dynamics

namespace vehicle_sim::telemetry {

/// @class TelemetrySampler2D
/// @brief Coordinates physical state extraction, differentiation, and filtering.
/// @details Translates RigidBody2D motion and IVehicleDynamics state into a normalized
/// TelemetryFrame2D sample with filtered accelerations and decomposed G-forces.
class TelemetrySampler2D {
public:
    /// @brief Default constructor with baseline filter smoothing.
    TelemetrySampler2D() noexcept = default;

    /// @brief Parameterized constructor setting filter smoothing factor.
    /// @param[in] filterAlpha Exponential moving average weight in (0.0, 1.0].
    explicit TelemetrySampler2D(double filterAlpha) noexcept;

    /// @brief Resets transient state estimators and previous velocity registers.
    void reset() noexcept;

    /// @brief Sets the exponential moving average alpha factor for acceleration.
    /// @param[in] alpha Smoothing factor clamped to (0.0, 1.0].
    void setFilterAlpha(double alpha) noexcept;

    /// @brief Gets the active exponential moving average alpha factor.
    /// @return Active alpha factor.
    [[nodiscard]] double getFilterAlpha() const noexcept;

    /// @brief Samples current vehicle dynamics and builds a telemetry frame.
    /// @param[in] body Simulated rigid body providing kinematics.
    /// @param[in] dynamics Optional vehicle dynamics coordinator providing airborne status.
    /// @param[in] timestamp Current simulation time in seconds.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Fully populated TelemetryFrame2D.
    [[nodiscard]] TelemetryFrame2D sample(const physics::RigidBody2D& body, const dynamics::IVehicleDynamics* dynamics,
        double timestamp, double dt) noexcept;

    /// @brief Specialized sampler for ArcadeCarDynamics with wheel contacts and inputs.
    /// @param[in] body Simulated rigid body providing kinematics.
    /// @param[in] car Arcade car dynamics coordinator.
    /// @param[in] timestamp Current simulation time in seconds.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Fully populated TelemetryFrame2D including tire and pedal metrics.
    [[nodiscard]] TelemetryFrame2D sampleArcadeCar(
        const physics::RigidBody2D& body, const dynamics::ArcadeCarDynamics& car, double timestamp, double dt) noexcept;

private:
    std::uint64_t m_tick { 0 };
    math::Vec2 m_prevVelocity {};
    math::Vec2 m_filteredAcceleration {};
    double m_filterAlpha { 0.25 };
    bool m_initialized { false };
};

} // namespace vehicle_sim::telemetry
