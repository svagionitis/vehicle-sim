#pragma once

#include <string>
#include <SDL2/SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @struct TelemetrySnapshot
/// @brief Live performance and physics metrics for HUD display.
struct TelemetrySnapshot {
    double speedMps { 0.0 };         ///< Speed in meters per second.
    double speedMph { 0.0 };         ///< Speed in miles per hour.
    double angularVelocity { 0.0 };   ///< Yaw angular velocity in rad/s.
    double stepDurationMs { 8.333 }; ///< Physics step duration in milliseconds.
    double droppedSeconds { 0.0 };   ///< Dropped accumulator time in seconds.
    double fps { 60.0 };             ///< Render frame rate in Hertz.
    bool barrierOn { false };        ///< Barrier arena status.
    bool gravityOn { false };        ///< Gravity status.
    bool deadZoneOn { true };        ///< Camera dead zone status.
    bool speedZoomOn { true };       ///< Speed zoom status.
    bool inputHudVisible { true };   ///< Input indicators visibility.
    bool steerLeft { false };        ///< Active left steer input.
    bool steerRight { false };       ///< Active right steer input.
    bool throttleForward { false };  ///< Active acceleration input.
    bool throttleReverse { false };  ///< Active brake/reverse input.
    bool handbrake { false };        ///< Active handbrake input.
    bool isPaused { false };         ///< Active paused status.
};

/// @class TelemetryHud
/// @brief Rasterizes Motion Lab telemetry stats and visual input HUD overlay.
/// @details Renders monospace status text and responsive control badges using
/// built-in bitmap font rasterization without external font files.
class TelemetryHud {
public:
    /// @brief Renders the HUD overlay into the target SDL renderer.
    /// @param[in] renderer Active SDL hardware renderer.
    /// @param[in] snapshot Live telemetry metrics and toggle states.
    /// @param[in] screenWidth Viewport width in pixels.
    /// @param[in] screenHeight Viewport height in pixels.
    static void render(SDL_Renderer* renderer,
                       const TelemetrySnapshot& snapshot,
                       int screenWidth,
                       int screenHeight) noexcept;

    /// @brief Draws procedural alphanumeric text at pixel coordinates.
    /// @param[in] renderer Active SDL renderer.
    /// @param[in] text ASCII string to rasterize.
    /// @param[in] x Top-left pixel X coordinate.
    /// @param[in] y Top-left pixel Y coordinate.
    /// @param[in] scale Pixel scaling multiplier (default 2).
    /// @param[in] color Text RGBA tint.
    static void drawText(SDL_Renderer* renderer,
                        const std::string& text,
                        int x,
                        int y,
                        int scale,
                        SDL_Color color) noexcept;

private:
    [[nodiscard]] static std::uint16_t getGlyph(char c) noexcept;
};

} // namespace motion_lab
