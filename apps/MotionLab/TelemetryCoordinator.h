#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

#include "apps/MotionLab/TelemetryHud.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/CsvTelemetryExporter.h"
#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

namespace motion_lab {

/// @class TelemetryCoordinator
/// @brief Orchestrates live telemetry sampling, history buffers, and disk logging.
/// @details Pre-allocates all memory buffers during initialization to ensure zero
/// dynamic heap allocations during the interactive simulation loop. Coordinates
/// body-frame acceleration extraction, 4-corner dynamic wheel reactions, rolling
/// multi-channel oscilloscope strip charts, and on-demand MoTeC-compatible CSV logging.
class TelemetryCoordinator {
public:
    /// @brief Default constructor allocating 1200-frame ring buffer.
    TelemetryCoordinator();

    /// @brief Destructor safely flushing any active recording session.
    ~TelemetryCoordinator() noexcept;

    TelemetryCoordinator(const TelemetryCoordinator&) = delete;
    TelemetryCoordinator& operator=(const TelemetryCoordinator&) = delete;
    TelemetryCoordinator(TelemetryCoordinator&&) noexcept = default;
    TelemetryCoordinator& operator=(TelemetryCoordinator&&) noexcept = default;

    /// @brief Resets all telemetry buffers, peak statistics, and recording state.
    void reset() noexcept;

    /// @brief Steps telemetry pipeline with latest vehicle physics state.
    /// @param[in] body Active rigid body kinematics.
    /// @param[in] mass Vehicle curb mass in kilograms.
    /// @param[in] wheelbase Axle spacing in meters.
    /// @param[in] trackWidth Average lateral track width in meters.
    /// @param[in] cgHeight Center of gravity height above ground in meters.
    /// @param[in] surfaceMu Road surface friction coefficient.
    /// @param[in] frontSlip Instantaneous front tire slip angle in radians.
    /// @param[in] rearSlip Instantaneous rear tire slip angle in radians.
    /// @param[in] simTime Current simulation timestamp in seconds.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(const vehicle_sim::physics::RigidBody2D& body, double mass, double wheelbase, double trackWidth,
        double cgHeight, double surfaceMu, double frontSlip, double rearSlip, double simTime, double dt) noexcept;

    /// @brief Toggles on-demand disk session recording.
    /// @return True if recording was started, false if stopped.
    [[nodiscard]] bool toggleRecording() noexcept;

    /// @brief Queries whether disk logging is currently active.
    /// @return True if actively streaming frames to disk.
    [[nodiscard]] bool isRecording() const noexcept;

    /// @brief Gets active recording elapsed duration in seconds.
    /// @return Elapsed recording seconds.
    [[nodiscard]] double getRecordDuration() const noexcept;

    /// @brief Gets total frames recorded in current session.
    /// @return Recorded frame count.
    [[nodiscard]] std::uint64_t getRecordFrames() const noexcept;

    /// @brief Gets active recording output file path.
    /// @return Filename string.
    [[nodiscard]] const std::string& getRecordFile() const noexcept;

    /// @brief Populates extended telemetry snapshot for HUD rendering.
    /// @param[out] outSnapshot Destination snapshot structure.
    void populateSnapshot(ExtendedTelemetrySnapshot& outSnapshot) const noexcept;

    /// @brief Accesses underlying ring buffer for verification and testing.
    /// @return Const reference to internal ring buffer.
    [[nodiscard]] const vehicle_sim::telemetry::TelemetryRingBuffer2D& getBuffer() const noexcept;

private:
    /// @brief Computes 4-corner dynamic wheel load transfers.
    void computeLoads(
        double mass, double wheelbase, double trackWidth, double cgHeight, double longG, double latG) noexcept;

    /// @brief Generates ISO 8601 timestamped filename for recording.
    [[nodiscard]] static std::string makeFilename() noexcept;

    vehicle_sim::telemetry::TelemetryRingBuffer2D m_buffer;
    vehicle_sim::telemetry::TelemetrySampler2D m_sampler;
    vehicle_sim::telemetry::CsvTelemetryExporter m_exporter;

    // Rolling Strip Chart Circular Buffers
    OscilloscopeWindow m_oscilloscope { };

    // G-G Trail Buffer
    static constexpr std::size_t kGgTrailSize { ExtendedTelemetrySnapshot::kGgTrailCapacity };
    std::array<vehicle_sim::math::Vec2, kGgTrailSize> m_ggTrail { };
    std::size_t m_ggHead { 0U };
    std::size_t m_ggCount { 0U };

    // Dynamic Normal Reactions
    DynamicWheelLoads m_currentLoads { };

    // Peak G-Force Registers
    double m_peakLongG { 0.0 };
    double m_peakLatG { 0.0 };
    double m_lastSurfaceMu { 1.0 };
    double m_lastSpeedMps { 0.0 };
    double m_lastYawRate { 0.0 };

    // Session Disk Recording
    bool m_isRecording { false };
    double m_recordStartTime { 0.0 };
    double m_recordDuration { 0.0 };
    std::uint64_t m_recordFrames { 0U };
    std::ofstream m_recordStream { };
    std::string m_recordFilename { };
};

} // namespace motion_lab
