#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @enum SkidpadRadiusPreset
/// @brief Configurable skidpad circular track dimensions.
enum class SkidpadRadiusPreset {
    Radius15m,  ///< 15.0m radius autocross / Formula SAE circle.
    Radius30m   ///< 30.0m radius standard automotive skidpad.
};

/// @enum SkidpadDirection
/// @brief Orbital rotation direction around skidpad center.
enum class SkidpadDirection {
    Unknown,            ///< Insufficient angular displacement to determine.
    CounterClockwise,   ///< Positive angular velocity (+theta, left turn).
    Clockwise           ///< Negative angular velocity (-theta, right turn).
};

/// @enum SkidpadState
/// @brief Operational state of skidpad proving ground execution.
enum class SkidpadState {
    Staging,       ///< Vehicle in tangent staging chute preparing for entry.
    Entering,      ///< Vehicle transitioning from chute into circular lane.
    TrackingOrbit, ///< Vehicle actively circling within boundary corridor.
    OffCourse      ///< Vehicle deviated outside inner or outer radius corridor.
};

/// @enum HandlingClassification
/// @brief Vehicle handling balance category based on understeer gradient Kus.
enum class HandlingClassification {
    Understeer,  ///< Kus > +0.5 deg/G: Safe push balance.
    Neutral,     ///< -0.5 <= Kus <= +0.5 deg/G: Optimal balance.
    Oversteer    ///< Kus < -0.5 deg/G: Loose / spin-prone balance.
};

/// @struct SkidpadPylon
/// @brief Physical and visual state of a skidpad boundary cone.
struct SkidpadPylon {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };  ///< World coordinates in meters.
    double radius { 0.4 };                          ///< Collision radius in meters.
    vehicle_sim::math::Vec2 toppleDir { 1.0, 0.0 }; ///< Impact displacement direction.
    double toppleAngle { 0.0 };                     ///< Tilt angle in radians (0 to pi/2).
    std::uint32_t coneIndex { 0U };                 ///< 0-based cone index.
    bool isOuterRing { false };                     ///< True for outer ring, false for inner.
    bool wasHit { false };                          ///< Latching contact status flag.
};

/// @struct SkidpadMetrics
/// @brief Dynamic measurements, grip ratings, and handling gradient data.
struct SkidpadMetrics {
    double currentRadius { 30.0 };                 ///< Active track radius in meters.
    double instantaneousSpeed { 0.0 };             ///< Current chassis speed in km/h.
    double instantaneousLatAcc { 0.0 };            ///< Current lateral acceleration in G.
    double peakLatAcc { 0.0 };                     ///< Highest lateral acceleration in G.
    double lapAverageLatAcc { 0.0 };               ///< Official SAE lap-averaged lateral G.
    double currentLapTime { 0.0 };                 ///< Active lap duration in seconds.
    double bestCwLapTime { 0.0 };                  ///< Fastest completed Clockwise lap in seconds.
    double bestCcwLapTime { 0.0 };                 ///< Fastest completed CCW lap in seconds.
    double radiusError { 0.0 };                    ///< Deviation from ideal centerline in meters.
    double steerAngleDeg { 0.0 };                  ///< Front road wheel steer angle in degrees.
    double understeerGradient { 0.0 };             ///< Kus in deg/G.
    std::uint32_t completedLaps { 0U };            ///< Total valid laps completed.
    std::uint32_t hitCount { 0U };                 ///< Total cones struck.
    SkidpadDirection direction { SkidpadDirection::Unknown }; ///< Active rotation direction.
    HandlingClassification balance { HandlingClassification::Understeer }; ///< Handling status.
    bool isSteadyState { false };                  ///< True when vehicle meets steady-state criteria.
    bool hasValidLap { false };                    ///< True if at least one clean lap recorded.
};

/// @class SkidpadCoordinator
/// @brief Coordinates skidpad geometry, steady-state telemetry, and understeer gradient.
class SkidpadCoordinator {
public:
    static constexpr std::size_t kMaxPylons { 80U };
    static constexpr double kCorridorWidth { 3.5 };
    static constexpr double kStagingLength { 40.0 };
    static constexpr double kRadius15m { 15.0 };
    static constexpr double kRadius30m { 30.0 };

    /// @brief Default constructor initializing standard 30m skidpad.
    SkidpadCoordinator() noexcept;

    /// @brief Sets active target skidpad radius preset.
    /// @param[in] preset Target radius (15m or 30m).
    void setRadiusPreset(SkidpadRadiusPreset preset) noexcept;

    /// @brief Gets active target skidpad radius preset.
    /// @return Active SkidpadRadiusPreset.
    [[nodiscard]] SkidpadRadiusPreset getRadiusPreset() const noexcept;

    /// @brief Gets target centerline radius in meters.
    /// @return Centerline radius in meters.
    [[nodiscard]] double getTargetRadius() const noexcept;

    /// @brief Advances simulation, updates lap timing, evaluates Kus, and tests collisions.
    /// @param[in,out] body Target vehicle rigid body.
    /// @param[in] steerAngle Steered road wheel angle in radians.
    /// @param[in] wheelbase Vehicle wheelbase in meters.
    /// @param[in] dt Physics timestep duration in seconds.
    void stepPhysics(vehicle_sim::physics::RigidBody2D& body,
                     double steerAngle,
                     double wheelbase,
                     double dt) noexcept;

    /// @brief Resets vehicle to tangent staging chute and clears run metrics.
    /// @param[in,out] body Target vehicle rigid body.
    void resetToStaging(vehicle_sim::physics::RigidBody2D& body) noexcept;

    /// @brief Toggles skidpad proving ground mode on or off.
    void toggleEnabled() noexcept;

    /// @brief Sets skidpad proving ground enabled state.
    /// @param[in] enabled Desired enabled state.
    void setEnabled(bool enabled) noexcept;

    /// @brief Checks whether skidpad mode is actively enabled.
    /// @return True if active.
    [[nodiscard]] bool isEnabled() const noexcept;

    /// @brief Gets active operational state.
    /// @return Active SkidpadState.
    [[nodiscard]] SkidpadState getState() const noexcept;

    /// @brief Gets active run metrics and handling telemetry.
    /// @return Const reference to SkidpadMetrics.
    [[nodiscard]] const SkidpadMetrics& getMetrics() const noexcept;

    /// @brief Gets array of active boundary pylon cones.
    /// @return Const reference to pylon array.
    [[nodiscard]] const std::array<SkidpadPylon, kMaxPylons>& getPylons() const noexcept;

    /// @brief Gets active pylon count for current radius preset.
    /// @return Number of active cones.
    [[nodiscard]] std::size_t getActivePylonCount() const noexcept;

private:
    void rebuildCourseGeometry() noexcept;
    void evaluateOrbitGate(double prevTheta, double currTheta, double dt) noexcept;
    void updateSteadyState(double steerAngle, double wheelbase, double dt) noexcept;
    void checkPylonCollisions(vehicle_sim::physics::RigidBody2D& body) noexcept;

    SkidpadRadiusPreset mPreset { SkidpadRadiusPreset::Radius30m };
    SkidpadState mState { SkidpadState::Staging };
    SkidpadMetrics mMetrics { };
    std::array<SkidpadPylon, kMaxPylons> mPylons { };
    std::size_t mActivePylonCount { 68U };

    vehicle_sim::math::Vec2 mLastPos { 0.0, 0.0 };
    double mLastTheta { 0.0 };
    double mAccumulatedAngle { 0.0 };
    double mSteadyStateTimer { 0.0 };
    double mLastSpeed { 0.0 };
    double mLastYawRate { 0.0 };
    bool mEnabled { false };
    bool mGateArmed { false };
};

} // namespace motion_lab
