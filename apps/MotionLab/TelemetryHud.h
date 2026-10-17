#pragma once

#include <SDL2/SDL.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "apps/MotionLab/MotionLabCamera.h"
#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @struct DynamicWheelLoads
/// @brief 4-corner vertical reaction loads in Newtons.
struct DynamicWheelLoads {
    double frontLeftLoad { 0.0 }; ///< Front-Left normal reaction in N.
    double frontRightLoad { 0.0 }; ///< Front-Right normal reaction in N.
    double rearLeftLoad { 0.0 }; ///< Rear-Left normal reaction in N.
    double rearRightLoad { 0.0 }; ///< Rear-Right normal reaction in N.
    double staticFrontWheel { 0.0 }; ///< Nominal unaccelerated front wheel load.
    double staticRearWheel { 0.0 }; ///< Nominal unaccelerated rear wheel load.
};

/// @struct OscilloscopeWindow
/// @brief Fixed-size historical buffers for rolling strip charts.
struct OscilloscopeWindow {
    static constexpr std::size_t kHistoryCapacity { 200U };
    std::array<float, kHistoryCapacity> speedHistory { };
    std::array<float, kHistoryCapacity> frontSlipHistory { };
    std::array<float, kHistoryCapacity> rearSlipHistory { };
    std::array<float, kHistoryCapacity> yawRateHistory { };
    std::size_t sampleCount { 0U };
    std::size_t headIndex { 0U };
};

/// @struct ExtendedTelemetrySnapshot
/// @brief High-rate analytical state payload for advanced HUD widgets.
struct ExtendedTelemetrySnapshot {
    // Kinematics & Accelerations
    double speedMps { 0.0 };
    double speedMph { 0.0 };
    double angularVelocity { 0.0 };
    double longitudinalG { 0.0 };
    double lateralG { 0.0 };
    double peakLongG { 0.0 };
    double peakLatG { 0.0 };
    double surfaceFriction { 1.0 };

    // Dynamic Normal Reactions
    DynamicWheelLoads wheelLoads { };

    // G-G Trail History (60 samples = 0.5s at 120Hz)
    static constexpr std::size_t kGgTrailCapacity { 60U };
    std::array<vehicle_sim::math::Vec2, kGgTrailCapacity> ggTrail { };
    std::size_t ggTrailCount { 0U };
    std::size_t ggTrailHead { 0U };

    // Strip Chart History
    OscilloscopeWindow stripCharts { };

    // Session Recording State
    bool isRecording { false };
    double recordingDuration { 0.0 };
    std::uint64_t recordedFrames { 0U };

    // Advanced HUD Visibility
    bool advancedHudVisible { true };
};

/// @struct TelemetrySnapshot
/// @brief Live performance and physics metrics for HUD display.
struct TelemetrySnapshot {
    double speedMps { 0.0 }; ///< Speed in meters per second.
    double speedMph { 0.0 }; ///< Speed in miles per hour.
    double angularVelocity { 0.0 }; ///< Yaw angular velocity in rad/s.
    double stepDurationMs { 8.333 }; ///< Physics step duration in milliseconds.
    double droppedSeconds { 0.0 }; ///< Dropped accumulator time in seconds.
    double fps { 60.0 }; ///< Render frame rate in Hertz.
    bool barrierOn { false }; ///< Barrier arena status.
    bool gravityOn { false }; ///< Gravity status.
    bool deadZoneOn { true }; ///< Camera dead zone status.
    bool speedZoomOn { true }; ///< Speed zoom status.
    bool inputHudVisible { true }; ///< Input indicators visibility.
    bool steerLeft { false }; ///< Active left steer input.
    bool steerRight { false }; ///< Active right steer input.
    bool throttleForward { false }; ///< Active acceleration input.
    bool throttleReverse { false }; ///< Active brake/reverse input.
    bool handbrake { false }; ///< Active handbrake input.
    bool isPaused { false }; ///< Active paused status.
    std::string terrainName { "TARMAC" }; ///< Name of active surface terrain.
    double terrainFriction { 1.0 }; ///< Active surface friction coefficient.
    double terrainRollingDrag { 1.0 }; ///< Active surface rolling drag multiplier.
    std::string vehicleName { "CLASSIC CAR" }; ///< Name of active vehicle simulation model.
    std::string vehicleDetails { "" }; ///< Dynamic vehicle telemetry metrics.
    bool circuitMode { false }; ///< Procedural circuit track active status.
    bool slalomMode { false }; ///< 100-meter slalom course active status.
    bool mooseMode { false }; ///< ISO 3888-2 Moose Test active status.
    bool skidpadMode { false }; ///< Skidpad steady-state circles active status.
    double currentLapTime { 0.0 }; ///< Elapsed time in current lap in seconds.
    double bestLapTime { 0.0 }; ///< Fastest recorded lap duration in seconds.
    double lastLapTime { 0.0 }; ///< Previous lap duration in seconds.
    std::uint32_t currentSector { 1U }; ///< Active sector index (1, 2, or 3).
    std::uint32_t completedLaps { 0U }; ///< Total completed laps.
    bool allHudVisible { true }; ///< Clean view status (if false, all HUD overlays hidden).
};

/// @class TelemetryHud
/// @brief Rasterizes Motion Lab telemetry stats and visual input HUD overlay.
/// @details Renders monospace status text, responsive control badges, dynamic G-G
/// friction circle, 4-corner weight transfer load bars, and live rolling oscilloscopes.
class TelemetryHud {
public:
    /// @brief Renders the HUD overlay into the target SDL renderer with extended analytics.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] snapshot Live telemetry metrics and toggle states.
    /// @param[in] extended Advanced analytical loads, G-G circle, and oscilloscope data.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    static void render(SDL_Renderer* renderer, const TelemetrySnapshot& snapshot,
        const ExtendedTelemetrySnapshot& extended, int screenWidth, int screenHeight) noexcept;

    /// @brief Overload for basic HUD rendering with default extended analytics.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] snapshot Live telemetry metrics and toggle states.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    static void render(
        SDL_Renderer* renderer, const TelemetrySnapshot& snapshot, int screenWidth, int screenHeight) noexcept;

    /// @brief Renders the polar G-G friction circle radar.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] extended Extended telemetry snapshot containing Gs and trail.
    /// @param[in] centerX Center X pixel coordinate.
    /// @param[in] centerY Center Y pixel coordinate.
    /// @param[in] radius Radius of the 1.0G outer circle.
    static void renderFrictionCircle(SDL_Renderer* renderer, const ExtendedTelemetrySnapshot& extended, int centerX,
        int centerY, int radius) noexcept;

    /// @brief Renders the 4-corner dynamic wheel load vertical bar graphs.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] loads Dynamic wheel loads and nominal references.
    /// @param[in] x Top-left X coordinate.
    /// @param[in] y Top-left Y coordinate.
    static void renderWeightTransfer(SDL_Renderer* renderer, const DynamicWheelLoads& loads, int x, int y) noexcept;

    /// @brief Renders a rolling oscilloscope strip chart widget.
    /// @param[in] renderer Active hardware SDL renderer.
    /// @param[in] values Rolling data values.
    /// @param[in] count Number of valid samples in buffer.
    /// @param[in] headIndex Circular buffer head offset.
    /// @param[in] x Top-left pixel X coordinate.
    /// @param[in] y Top-left pixel Y coordinate.
    /// @param[in] width Strip chart pixel width.
    /// @param[in] height Strip chart pixel height.
    /// @param[in] minY Minimum engineering scale value.
    /// @param[in] maxY Maximum engineering scale value.
    /// @param[in] title Alphanumeric channel identifier.
    /// @param[in] traceColor Primary line color.
    static void renderStripChart(SDL_Renderer* renderer, const float* values, std::size_t count, std::size_t headIndex,
        int x, int y, int width, int height, float minY, float maxY, const char* title, SDL_Color traceColor) noexcept;

    /// @brief Draws procedural alphanumeric text at pixel coordinates.
    /// @param[in] renderer Active SDL renderer.
    /// @param[in] text ASCII string to rasterize.
    /// @param[in] x Top-left pixel X coordinate.
    /// @param[in] y Top-left pixel Y coordinate.
    /// @param[in] scale Pixel scaling multiplier (default 2).
    /// @param[in] color Text RGBA tint.
    static void drawText(
        SDL_Renderer* renderer, const std::string& text, int x, int y, int scale, SDL_Color color) noexcept;

private:
    [[nodiscard]] static std::uint16_t getGlyph(char c) noexcept;
    static void drawCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color color) noexcept;
};

} // namespace motion_lab
