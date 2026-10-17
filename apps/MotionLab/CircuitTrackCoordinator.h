#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "apps/MotionLab/MotionLabCamera.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"
#include "libs/VehicleSimCore/terrain/TrackGenerator.h"

namespace motion_lab {

/// @struct CircuitLapMetrics
/// @brief Lap timing metrics and sector splits for the procedural circuit track.
struct CircuitLapMetrics {
    double currentLapTime { 0.0 };      ///< Elapsed time in current lap in seconds.
    double lastLapTime { 0.0 };         ///< Duration of previous lap in seconds.
    double bestLapTime { 0.0 };         ///< Fastest recorded lap duration in seconds.
    double sector1Time { 0.0 };         ///< Split duration for sector 1 in seconds.
    double sector2Time { 0.0 };         ///< Split duration for sector 2 in seconds.
    double sector3Time { 0.0 };         ///< Split duration for sector 3 in seconds.
    std::uint32_t completedLaps { 0U }; ///< Total completed lap count.
    std::uint32_t currentSector { 1U }; ///< Active sector index (1, 2, or 3).
    bool hasValidBest { false };        ///< True if at least one valid lap was completed.
};

/// @class CircuitTrackCoordinator
/// @brief Coordinates procedural closed-circuit track simulation, physics queries, and lap timing.
class CircuitTrackCoordinator {
public:
    /// @brief Default constructor initializing default circuit layout.
    CircuitTrackCoordinator() noexcept;

    /// @brief Regenerates circuit track geometry using a procedural random seed.
    /// @param[in] seed Random seed for track layout.
    void regenerate(std::uint32_t seed) noexcept;

    /// @brief Steps track physics, updates material friction, applies banking, and checks gates.
    /// @param[in,out] body Target vehicle rigid body.
    /// @param[in] dt Physics timestep duration in seconds.
    void stepPhysics(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept;

    /// @brief Teleports vehicle back to the starting grid and resets lap timers.
    /// @param[in,out] body Target vehicle rigid body.
    void resetToGrid(vehicle_sim::physics::RigidBody2D& body) noexcept;

    /// @brief Queries whether circuit mode is actively enabled.
    /// @return True if circuit track mode is active.
    [[nodiscard]] bool isEnabled() const noexcept;

    /// @brief Toggles circuit track mode on or off.
    void toggleEnabled() noexcept;

    /// @brief Enables or disables circuit track mode.
    /// @param[in] enabled Desired enabled state.
    void setEnabled(bool enabled) noexcept;

    /// @brief Gets constant reference to active CircuitTrack2D geometry.
    /// @return Active circuit track geometry.
    [[nodiscard]] const vehicle_sim::terrain::CircuitTrack2D& getTrack() const noexcept;

    /// @brief Gets active lap timing metrics.
    /// @return Const reference to CircuitLapMetrics.
    [[nodiscard]] const CircuitLapMetrics& getMetrics() const noexcept;

    /// @brief Gets current surface material under vehicle.
    /// @return Active SurfaceMaterial under chassis.
    [[nodiscard]] vehicle_sim::terrain::SurfaceMaterial getActiveMaterial() const noexcept;

    /// @brief Gets normalized track progress along centerline in range [0, 1].
    /// @return Fractional progress along centerline.
    [[nodiscard]] double getTrackProgress() const noexcept;

    /// @brief Updates vehicle dimensions and dynamically scales circuit track geometry.
    /// @param[in] vehicleWidth Width of active vehicle in meters.
    /// @param[in] vehicleLength Overall length of active vehicle in meters.
    void updateDimensions(double vehicleWidth, double vehicleLength) noexcept;

    /// @brief Gets dynamic drivable track width in meters.
    /// @return Active track width in meters.
    [[nodiscard]] double getTrackWidth() const noexcept;

    /// @brief Gets active vehicle width in meters.
    /// @return Vehicle width in meters.
    [[nodiscard]] double getVehicleWidth() const noexcept;

    /// @brief Gets active vehicle length in meters.
    /// @return Vehicle length in meters.
    [[nodiscard]] double getVehicleLength() const noexcept;

private:
    void evaluateSurface(vehicle_sim::physics::RigidBody2D& body) noexcept;
    void evaluateBanking(vehicle_sim::physics::RigidBody2D& body) noexcept;
    void checkTimingGates(const vehicle_sim::math::Vec2& prevPos,
                          const vehicle_sim::math::Vec2& currPos) noexcept;

    [[nodiscard]] static bool checkSegmentCross(const vehicle_sim::math::Vec2& a1,
                                                const vehicle_sim::math::Vec2& a2,
                                                const vehicle_sim::math::Vec2& b1,
                                                const vehicle_sim::math::Vec2& b2) noexcept;

    vehicle_sim::terrain::CircuitTrack2D mTrack { };
    CircuitLapMetrics mMetrics { };
    vehicle_sim::terrain::SurfaceMaterial mActiveMaterial { };
    vehicle_sim::math::Vec2 mLastVehiclePos { 0.0, 0.0 };
    double mCurrentProgress { 0.0 };
    double mVehicleWidth { 2.0 };
    double mVehicleLength { 4.0 };
    std::uint32_t mSeed { 2024U };
    bool mEnabled { false };
    bool mHasStartedLap { false };
};

} // namespace motion_lab
