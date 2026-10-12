#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"

namespace earth_patrol {

/// @struct CameraConfig
/// @brief Configuration settings for 2D viewport tracking and world projection.
struct CameraConfig {
    double pixelsPerMeter { 40.0 }; ///< Spatial scaling multiplier from meters to pixels.
    double screenFractionX { 0.28 }; ///< Horizontal viewport anchor fraction for rover CoM.
    double screenFractionY { 0.70 }; ///< Vertical viewport anchor fraction for ground baseline.
    double lookaheadLead { 3.5 }; ///< Dynamic horizontal lookahead lead in meters.
    double horizontalFollowRate { 4.5 }; ///< Exponential tracking rate along X axis.
    double verticalFollowRate { 3.0 }; ///< Exponential tracking rate along Y axis.
    int viewportWidth { 1280 }; ///< Viewport pixel width.
    int viewportHeight { 720 }; ///< Viewport pixel height.
};

/// @class Camera2D
/// @brief 2D side-scrolling camera tracker with dynamic lookahead lead and smooth damping.
/// @details Converts between world meters and screen pixel coordinates, tracking horizontal
/// rover displacement with speed-proportional lookahead and vertical ground conformity.
class Camera2D {
public:
    /// @brief Default constructor with nominal camera configuration.
    Camera2D() noexcept;

    /// @brief Parameterized constructor using custom camera configuration.
    /// @param[in] config Viewport and tracking settings.
    explicit Camera2D(const CameraConfig& config) noexcept;

    /// @brief Updates camera tracking position toward target with damping.
    /// @param[in] targetPos Target world coordinates to track in meters.
    /// @param[in] forwardSpeed Longitudinal vehicle forward speed in m/s.
    /// @param[in] dt Timestep duration in seconds.
    void update(const vehicle_sim::math::Vec2& targetPos, double forwardSpeed, double dt) noexcept;

    /// @brief Resets camera position immediately to target without damping.
    /// @param[in] targetPos Target world coordinates in meters.
    void reset(const vehicle_sim::math::Vec2& targetPos) noexcept;

    /// @brief Transforms world coordinates to screen pixel coordinates.
    /// @param[in] worldPos World coordinates in meters.
    /// @return Screen coordinates in pixels.
    [[nodiscard]] vehicle_sim::math::Vec2 worldToScreen(const vehicle_sim::math::Vec2& worldPos) const noexcept;

    /// @brief Transforms screen pixel coordinates to world coordinates.
    /// @param[in] screenPos Screen coordinates in pixels.
    /// @return World coordinates in meters.
    [[nodiscard]] vehicle_sim::math::Vec2 screenToWorld(const vehicle_sim::math::Vec2& screenPos) const noexcept;

    /// @brief Gets minimum visible world X coordinate.
    /// @return Left viewport boundary in meters.
    [[nodiscard]] double getVisibleMinX() const noexcept;

    /// @brief Gets maximum visible world X coordinate.
    /// @return Right viewport boundary in meters.
    [[nodiscard]] double getVisibleMaxX() const noexcept;

    /// @brief Gets active camera tracking position.
    /// @return World coordinates of camera center in meters.
    [[nodiscard]] const vehicle_sim::math::Vec2& getPosition() const noexcept;

    /// @brief Sets active camera tracking position explicitly.
    /// @param[in] pos New camera world position.
    void setPosition(const vehicle_sim::math::Vec2& pos) noexcept;

    /// @brief Gets camera configuration parameters.
    /// @return Constant reference to CameraConfig.
    [[nodiscard]] const CameraConfig& getConfig() const noexcept;

    /// @brief Sets camera configuration parameters.
    /// @param[in] config New configuration settings.
    void setConfig(const CameraConfig& config) noexcept;

    /// @brief Updates viewport dimensions.
    /// @param[in] width Viewport width in pixels.
    /// @param[in] height Viewport height in pixels.
    void setViewportSize(int width, int height) noexcept;

    /// @brief Triggers dynamic camera trauma / screen shake on impacts.
    /// @param[in] intensity Peak displacement in meters.
    /// @param[in] duration Shake duration in seconds.
    void addScreenShake(double intensity, double duration) noexcept;

    /// @brief Gets active shake displacement offset in meters.
    /// @return Current shake offset.
    [[nodiscard]] const vehicle_sim::math::Vec2& getShakeOffset() const noexcept;

private:
    CameraConfig config_ { };
    vehicle_sim::math::Vec2 position_ { 0.0, 0.0 };
    double shakeIntensity_ { 0.0 };
    double shakeDuration_ { 0.0 };
    double shakeTimer_ { 0.0 };
    double shakeAngle_ { 0.0 };
    vehicle_sim::math::Vec2 shakeOffset_ { 0.0, 0.0 };
};

} // namespace earth_patrol
