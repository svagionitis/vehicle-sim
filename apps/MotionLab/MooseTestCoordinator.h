#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @enum MooseTestState
/// @brief Operational state of the ISO 3888-2 test execution.
enum class MooseTestState {
    IdleAtStaging,   ///< Vehicle is staged behind start gate at x = -30m.
    Approaching,     ///< Vehicle accelerating towards entry gate x = 0m.
    InManeuver,      ///< Actively navigating within sections 1 through 5.
    Completed,       ///< Vehicle crossed finish line x = 61m; evaluation locked.
    FailedOffCourse  ///< Vehicle spun out or deviated completely outside corridor.
};

/// @enum MooseRunResult
/// @brief Official ISO 3888-2 outcome classification.
enum class MooseRunResult {
    Pending,         ///< Run is in progress or waiting at start.
    CleanPass,       ///< Successful run: zero cone hits, no throttle/brake faults.
    FailedConeHit,   ///< Struck one or more pylon cones.
    Disqualified,    ///< Applied throttle or brake during evasion corridor.
    FailedSpinOut    ///< Vehicle exceeded yaw stability or spun past 90 degrees.
};

/// @struct MooseTestCone
/// @brief State of a single ISO 3888-2 boundary pylon obstacle.
struct MooseTestCone {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };  ///< World position in meters.
    double radius { 0.4 };                          ///< Collision radius in meters.
    vehicle_sim::math::Vec2 toppleDir { 1.0, 0.0 }; ///< Impact displacement direction.
    double toppleAngle { 0.0 };                     ///< Tilt angle in radians (0 to pi/2).
    std::uint32_t coneIndex { 0U };                 ///< 0-based cone index (0 to 23).
    std::uint32_t sectionIndex { 1U };              ///< Associated section (1 to 5).
    bool wasHit { false };                          ///< Single-hit contact latch flag.
};

/// @struct MooseTestSection
/// @brief Geometric boundaries of one of the 5 ISO 3888-2 corridor segments.
struct MooseTestSection {
    double xStart { 0.0 };      ///< Starting longitudinal coordinate in meters.
    double xEnd { 0.0 };        ///< Ending longitudinal coordinate in meters.
    double width { 0.0 };       ///< Transversal corridor width in meters.
    double yOffset { 0.0 };     ///< Transversal centerline offset in meters.
    std::uint32_t index { 1U }; ///< 1-based section identifier (1 to 5).
};

/// @struct MooseTestMetrics
/// @brief Dynamic measurements, speed drops, and outcome data for the active run.
struct MooseTestMetrics {
    double entrySpeed { 0.0 };       ///< Speed at x = 0.0m in km/h.
    double evasionSpeed { 0.0 };     ///< Speed at x = 31.0m (mid Section 3) in km/h.
    double exitSpeed { 0.0 };        ///< Speed at x = 61.0m in km/h.
    double speedDrop { 0.0 };        ///< Total speed reduction (entry - exit) in km/h.
    double peakLatAcc { 0.0 };       ///< Maximum lateral acceleration during run in G.
    double peakYawRate { 0.0 };      ///< Maximum yaw rate in deg/s.
    double elapsedTime { 0.0 };      ///< Time between start and finish gates in seconds.
    double bestPassSpeed { 0.0 };    ///< Highest historical clean pass speed in km/h.
    std::uint32_t hitCount { 0U };   ///< Total number of struck pylons.
    MooseRunResult result { MooseRunResult::Pending }; ///< Outcome classification.
    bool throttleFault { false };    ///< True if throttle was applied in evasion corridor.
    bool brakeFault { false };       ///< True if service brake was applied in corridor.
    bool hasValidBest { false };     ///< True if at least one clean pass was recorded.
};

/// @class MooseTestCoordinator
/// @brief Coordinates ISO 3888-2 corridor geometry, pylon collisions, and test protocol.
class MooseTestCoordinator {
public:
    static constexpr std::size_t kConeCount { 24U };
    static constexpr std::size_t kSectionCount { 5U };
    static constexpr double kStartX { 0.0 };
    static constexpr double kFinishX { 61.0 };
    static constexpr double kStagingX { -30.0 };
    static constexpr double kEvasionMidX { 31.0 };
    static constexpr double kMaxLateralShift { 3.5 };

    /// @brief Default constructor initializing default corridor for classic car width (2.0m).
    MooseTestCoordinator() noexcept;

    /// @brief Updates corridor geometry and cone positions based on active vehicle dimensions.
    /// @param[in] vehicleWidth Width of active vehicle chassis in meters.
    /// @param[in] vehicleLength Length of active vehicle chassis in meters.
    void updateDimensions(double vehicleWidth, double vehicleLength) noexcept;

    /// @brief Advances simulation, checks collisions, updates metrics, and monitors protocol.
    /// @param[in,out] body Active vehicle chassis rigid body.
    /// @param[in] throttleInput Normalized throttle command in range [0, 1].
    /// @param[in] brakeInput Normalized brake command in range [0, 1].
    /// @param[in] dt Timestep duration in seconds.
    void stepPhysics(vehicle_sim::physics::RigidBody2D& body,
                     double throttleInput,
                     double brakeInput,
                     double dt) noexcept;

    /// @brief Teleports vehicle to staging line and resets course metrics for a new attempt.
    /// @param[in,out] body Active vehicle chassis rigid body.
    void resetCourse(vehicle_sim::physics::RigidBody2D& body) noexcept;

    /// @brief Toggles Moose Test mode on or off.
    void toggleEnabled() noexcept;

    /// @brief Enables or disables Moose Test mode.
    /// @param[in] enabled Desired enabled state.
    void setEnabled(bool enabled) noexcept;

    /// @brief Checks whether Moose Test mode is actively enabled.
    /// @return True if active.
    [[nodiscard]] bool isEnabled() const noexcept;

    /// @brief Gets active course operational state.
    /// @return Active MooseTestState.
    [[nodiscard]] MooseTestState getState() const noexcept;

    /// @brief Gets active run metrics and speed telemetry.
    /// @return Const reference to MooseTestMetrics.
    [[nodiscard]] const MooseTestMetrics& getMetrics() const noexcept;

    /// @brief Gets array of all 24 course pylon cones.
    /// @return Const reference to cone array.
    [[nodiscard]] const std::array<MooseTestCone, kConeCount>& getCones() const noexcept;

    /// @brief Gets array of all 5 course sections.
    /// @return Const reference to section array.
    [[nodiscard]] const std::array<MooseTestSection, kSectionCount>& getSections() const noexcept;

    /// @brief Gets active vehicle width used for corridor calculations.
    /// @return Scaled vehicle width in meters.
    [[nodiscard]] double getVehicleWidth() const noexcept;

    /// @brief Gets target entry speed for iterative speed hunting in km/h.
    /// @return Target speed in km/h.
    [[nodiscard]] double getTargetSpeed() const noexcept;

    /// @brief Sets target entry speed for upcoming run.
    /// @param[in] speedKmH Desired target speed in km/h.
    void setTargetSpeed(double speedKmH) noexcept;

private:
    void updateIdle(const vehicle_sim::physics::RigidBody2D& body) noexcept;
    void updateRunning(vehicle_sim::physics::RigidBody2D& body,
                       double throttleInput,
                       double brakeInput,
                       double dt) noexcept;
    void checkCollisions(const vehicle_sim::physics::RigidBody2D& body) noexcept;
    void checkGates(const vehicle_sim::math::Vec2& prevPos,
                    const vehicle_sim::math::Vec2& currPos,
                    const vehicle_sim::physics::RigidBody2D& body) noexcept;
    void recomputeBounds() noexcept;

    std::array<MooseTestCone, kConeCount> mCones {{ }};
    std::array<MooseTestSection, kSectionCount> mSections {{ }};
    MooseTestMetrics mMetrics { };
    MooseTestState mState { MooseTestState::IdleAtStaging };
    vehicle_sim::math::Vec2 mLastPos { -30.0, 0.0 };
    double mVehicleWidth { 2.0 };
    double mVehicleLength { 4.0 };
    double mTargetSpeedKmH { 65.0 };
    bool mEnabled { false };
    bool mPassedEntry { false };
    bool mPassedMid { false };
    bool mPassedFinish { false };
};

} // namespace motion_lab
