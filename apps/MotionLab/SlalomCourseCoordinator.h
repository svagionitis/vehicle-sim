#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @enum SlalomState
/// @brief Operational state of the 100-meter slalom course execution.
enum class SlalomState {
    IdleAtStart,    ///< Vehicle staged behind start gate.
    InProgress,     ///< Timed run active between start and finish.
    Finished        ///< Vehicle crossed finish line.
};

/// @struct SlalomPylon
/// @brief Physical and visual state of a single slalom pylon obstacle.
struct SlalomPylon {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 };  ///< Center coordinates in meters.
    double radius { 0.4 };                          ///< Collision radius in meters.
    vehicle_sim::math::Vec2 toppleDir { 1.0, 0.0 }; ///< Impact displacement direction.
    double toppleAngle { 0.0 };                     ///< Tilt angle in radians (0 to pi/2).
    std::uint32_t coneIndex { 0U };                 ///< 1-based index (1 to 10).
    bool wasHit { false };                          ///< Latching contact status flag.
    bool passed { false };                          ///< Passed longitudinal threshold flag.
    bool correctSide { true };                      ///< Directional weave compliance flag.
};

/// @struct SlalomMetrics
/// @brief Timing splits, penalty tallies, and run metrics for the slalom course.
struct SlalomMetrics {
    double elapsedTime { 0.0 };      ///< Raw elapsed run duration in seconds.
    double penaltyTime { 0.0 };      ///< Accumulated penalty seconds (+2.0s per hit).
    double finalTime { 0.0 };        ///< Total adjusted duration (elapsed + penalty).
    double bestTime { 0.0 };         ///< All-time fastest adjusted run duration.
    double sector1Time { 0.0 };      ///< Split duration for cones 1-3 in seconds.
    double sector2Time { 0.0 };      ///< Split duration for cones 4-7 in seconds.
    double sector3Time { 0.0 };      ///< Split duration for cones 8-10 in seconds.
    std::uint32_t hitCount { 0U };   ///< Number of touched pylons.
    std::uint32_t missedGates { 0U }; ///< Number of skipped or wrong-side gates.
    bool hasValidBest { false };     ///< True if a completed run exists.
};

/// @class SlalomCourseCoordinator
/// @brief Coordinates 100-meter slalom course layout, pylon collision, and timing gates.
class SlalomCourseCoordinator {
public:
    static constexpr std::size_t kPylonCount { 10U };
    static constexpr double kDefaultSpacing { 10.0 };
    static constexpr double kStartX { 0.0 };
    static constexpr double kPenaltyPerHit { 2.0 };
    static constexpr double kArmingSpeed { 0.5 };

    /// @brief Default constructor initializing 10-cone layout.
    SlalomCourseCoordinator() noexcept;

    /// @brief Updates slalom course geometry and pylon positions based on vehicle dimensions.
    /// @param[in] vehicleWidth Width of active vehicle chassis in meters.
    /// @param[in] vehicleLength Length of active vehicle chassis in meters.
    void updateDimensions(double vehicleWidth, double vehicleLength) noexcept;

    /// @brief Advances simulation, checks collisions, updates splits, and manages gates.
    /// @param[in,out] body Active vehicle chassis rigid body.
    /// @param[in] dt Timestep duration in seconds.
    void stepPhysics(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept;

    /// @brief Resets vehicle to staging point and clears course state.
    /// @param[in,out] body Active vehicle chassis rigid body.
    void resetCourse(vehicle_sim::physics::RigidBody2D& body) noexcept;

    /// @brief Toggles slalom course mode on or off.
    void toggleEnabled() noexcept;

    /// @brief Enables or disables slalom course mode.
    /// @param[in] enabled Desired enabled state.
    void setEnabled(bool enabled) noexcept;

    /// @brief Checks whether slalom mode is actively enabled.
    /// @return True if active.
    [[nodiscard]] bool isEnabled() const noexcept;

    /// @brief Gets active course operational state.
    /// @return Active SlalomState.
    [[nodiscard]] SlalomState getState() const noexcept;

    /// @brief Gets active run metrics and splits.
    /// @return Const reference to SlalomMetrics.
    [[nodiscard]] const SlalomMetrics& getMetrics() const noexcept;

    /// @brief Gets array of all 10 slalom pylons.
    /// @return Const reference to pylon array.
    [[nodiscard]] const std::array<SlalomPylon, kPylonCount>& getPylons() const noexcept;

    /// @brief Gets expected weaving sign (+1 for left, -1 for right) for next upcoming pylon.
    /// @return Expected sign of lateral offset.
    [[nodiscard]] double getNextWeaveSign() const noexcept;

    /// @brief Gets index of next upcoming pylon (1 to 10, or 10 if completed).
    /// @return 1-based cone index.
    [[nodiscard]] std::uint32_t getNextConeIndex() const noexcept;

    /// @brief Gets active longitudinal pylon separation in meters.
    /// @return Pylon spacing in meters.
    [[nodiscard]] double getPylonSpacing() const noexcept;

    /// @brief Gets width of active vehicle chassis in meters.
    /// @return Vehicle width in meters.
    [[nodiscard]] double getVehicleWidth() const noexcept;

    /// @brief Gets length of active vehicle chassis in meters.
    /// @return Vehicle length in meters.
    [[nodiscard]] double getVehicleLength() const noexcept;

    /// @brief Gets longitudinal coordinate of finish timing gate in meters.
    /// @return Finish X coordinate in meters.
    [[nodiscard]] double getFinishX() const noexcept;

    /// @brief Gets longitudinal coordinate of sector 1 split in meters.
    /// @return Sector 1 X coordinate in meters.
    [[nodiscard]] double getSector1X() const noexcept;

    /// @brief Gets longitudinal coordinate of sector 2 split in meters.
    /// @return Sector 2 X coordinate in meters.
    [[nodiscard]] double getSector2X() const noexcept;

    /// @brief Gets longitudinal coordinate of staging starting position in meters.
    /// @return Staging start X coordinate in meters.
    [[nodiscard]] double getStagingX() const noexcept;

    /// @brief Gets transversal gate width in meters.
    /// @return Gate width in meters.
    [[nodiscard]] double getGateWidth() const noexcept;

    /// @brief Gets lateral clipping apex offset in meters.
    /// @return Apex Y offset in meters.
    [[nodiscard]] double getApexOffset() const noexcept;

private:
    void recomputeLayout() noexcept;
    void updateIdle(const vehicle_sim::physics::RigidBody2D& body) noexcept;
    void updateRunning(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept;
    void checkCollisions(const vehicle_sim::physics::RigidBody2D& body) noexcept;
    void checkGates(const vehicle_sim::math::Vec2& prevPos,
                    const vehicle_sim::math::Vec2& currPos) noexcept;

    std::array<SlalomPylon, kPylonCount> mPylons {{ }};
    SlalomMetrics mMetrics { };
    SlalomState mState { SlalomState::IdleAtStart };
    double mVehicleWidth { 2.0 };
    double mVehicleLength { 4.0 };
    double mPylonSpacing { 10.0 };
    vehicle_sim::math::Vec2 mLastPos { -15.0, 0.0 };
    std::size_t mNextConeIdx { 0U };
    bool mEnabled { false };
    bool mSec1Recorded { false };
    bool mSec2Recorded { false };
};

} // namespace motion_lab
