#pragma once

#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/BarrierSolver2D.h"

namespace motion_lab {

/// @struct CameraConfig
/// @brief Configuration settings for Motion Lab viewport tracking and zoom.
struct CameraConfig {
    double baseZoom { 28.0 };           ///< Nominal pixels per meter.
    double cameraDeadZone { 0.18 };     ///< Viewport fraction for deadzone box.
    double cameraFollowRate { 16.0 };   ///< Exponential camera follow rate in 1/s.
    double zoomRate { 4.0 };            ///< Exponential zoom interpolation rate in 1/s.
    double minVelocityZoomSpeed { 3.0 };///< Speed in m/s at which zoom-out begins.
    double maxVelocityZoomSpeed { 20.0 };///< Speed in m/s at which zoom-out reaches peak.
    double maxVelocityZoomOut { 0.5 };  ///< Maximum zoom reduction factor (50%).
    double speedHistoryWindow { 1.0 };  ///< Trailing window in seconds for speed averaging.
    double barrierInsetPx { 32.0 };     ///< Barrier boundary margin inset in pixels.
    int viewportWidth { 1280 };         ///< Viewport width in pixels.
    int viewportHeight { 720 };         ///< Viewport height in pixels.
};

/// @class MotionLabCamera
/// @brief 2D camera tracker implementing Pat Kerr's dead zone box and dynamic speed zoom.
/// @details Converts between world meters and screen pixels, tracks vehicle displacement with
/// dead zone hysteresis, and time-weights velocity samples to adjust zoom smoothly.
class MotionLabCamera {
public:
    /// @brief Default constructor.
    MotionLabCamera() noexcept;

    /// @brief Parameterized constructor using custom settings.
    /// @param[in] config Initial camera configuration parameters.
    explicit MotionLabCamera(const CameraConfig& config) noexcept;

    /// @brief Updates camera tracking, deadzone, and speed zoom.
    /// @param[in] position Vehicle world position in meters.
    /// @param[in] velocity Vehicle world linear velocity in m/s.
    /// @param[in] dt Frame delta time in seconds.
    void update(const vehicle_sim::math::Vec2& position,
                const vehicle_sim::math::Vec2& velocity,
                double dt) noexcept;

    /// @brief Resets camera tracking position and zoom state.
    /// @param[in] position Vehicle world coordinates.
    void reset(const vehicle_sim::math::Vec2& position) noexcept;

    /// @brief Converts world coordinates in meters to screen pixel coordinates.
    /// @param[in] worldPos World coordinates in meters.
    /// @return Screen coordinates in pixels.
    [[nodiscard]] vehicle_sim::math::Vec2 worldToScreen(const vehicle_sim::math::Vec2& worldPos) const noexcept;

    /// @brief Converts screen pixel coordinates to world coordinates in meters.
    /// @param[in] screenPos Screen coordinates in pixels.
    /// @return World coordinates in meters.
    [[nodiscard]] vehicle_sim::math::Vec2 screenToWorld(const vehicle_sim::math::Vec2& screenPos) const noexcept;

    /// @brief Calculates world arena bounds based on current camera and viewport.
    /// @return BarrierBounds2D struct containing world boundaries.
    [[nodiscard]] vehicle_sim::physics::BarrierBounds2D getBounds() const noexcept;

    /// @brief Updates viewport dimensions and recalculates base zoom.
    /// @param[in] width Screen width in pixels.
    /// @param[in] height Screen height in pixels.
    void resize(int width, int height) noexcept;

    /// @brief Gets active camera center position in world meters.
    [[nodiscard]] const vehicle_sim::math::Vec2& getPosition() const noexcept;

    /// @brief Gets active zoom scale in pixels per meter.
    [[nodiscard]] double getZoom() const noexcept;

    /// @brief Gets base zoom scale in pixels per meter.
    [[nodiscard]] double getBaseZoom() const noexcept;

    /// @brief Sets active camera position explicitly.
    void setPosition(const vehicle_sim::math::Vec2& pos) noexcept;

    /// @brief Queries whether dead zone tracking is enabled.
    [[nodiscard]] bool isDeadZoneEnabled() const noexcept;

    /// @brief Toggles dead zone tracking.
    void setDeadZoneEnabled(bool enabled) noexcept;

    /// @brief Queries whether speed zoom is enabled.
    [[nodiscard]] bool isSpeedZoomEnabled() const noexcept;

    /// @brief Toggles speed zoom.
    void setSpeedZoomEnabled(bool enabled) noexcept;

    /// @brief Queries whether fixed camera mode is active (arena barrier mode).
    [[nodiscard]] bool isFixedCamera() const noexcept;

    /// @brief Toggles fixed camera mode.
    void setFixedCamera(bool fixed) noexcept;

    /// @brief Gets active configuration parameters.
    [[nodiscard]] const CameraConfig& getConfig() const noexcept;

private:
    struct SpeedSample {
        double time { 0.0 };
        double speed { 0.0 };
    };

    void updateCamera(const vehicle_sim::math::Vec2& position, double dt) noexcept;
    void updateZoom(const vehicle_sim::math::Vec2& position, const vehicle_sim::math::Vec2& velocity, double dt) noexcept;
    [[nodiscard]] double computeAverageSpeed(double currentSpeed, double dt) noexcept;

    CameraConfig mConfig { };
    vehicle_sim::math::Vec2 mCamera { 0.0, 0.0 };
    double mZoom { 28.0 };
    double mBaseZoom { 28.0 };
    double mVelocityZoomFactor { 1.0 };
    bool mDeadZoneEnabled { true };
    bool mSpeedZoomEnabled { true };
    bool mFixedCamera { false };
    double mSimulationTime { 0.0 };
    std::vector<SpeedSample> mSpeedHistory { };
};

} // namespace motion_lab
