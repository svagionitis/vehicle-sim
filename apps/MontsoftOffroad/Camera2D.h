#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"

namespace montsoft_offroad {

/// @struct CameraConfig
/// @brief Configuration settings for 2D viewport tracking and world projection.
struct CameraConfig {
    double pixelsPerMeter { 20.0 };      ///< Scale factor from world meters to pixels.
    double screenFractionX { 0.28 };     ///< Viewport anchor fraction along X (0.28 = 89.6 px).
    double screenFractionY { 0.72 };     ///< Viewport anchor fraction along Y (0.72 = 144.0 px).
    double lookaheadLead { 0.25 };       ///< Velocity lead time coefficient in seconds.
    double maxLookahead { 3.5 };         ///< Maximum horizontal lookahead distance in meters.
    double followRateX { 5.0 };          ///< Exponential follow rate along X in 1/s.
    double followRateY { 2.5 };          ///< Exponential follow rate along Y in 1/s.
    int viewportWidth { 320 };           ///< Native virtual viewport width in pixels.
    int viewportHeight { 200 };          ///< Native virtual viewport height in pixels.
    double maxShakeDisplacement { 0.5 }; ///< Peak trauma shake displacement in meters.
    double traumaDecayRate { 1.5 };      ///< Linear trauma decay rate in 1/s.
};

/// @class Camera2D
/// @brief 2D side-scrolling camera coordinator with dynamic velocity lead and screen shake.
/// @details Transforms between world coordinates (meters) and virtual 320x200 pixel coordinates.
class Camera2D final {
public:
    /// @brief Default constructor using standard 320x200 configuration.
    Camera2D() noexcept;

    /// @brief Parameterized constructor using custom camera configuration.
    /// @param[in] config Viewport and tracking parameters.
    explicit Camera2D(const CameraConfig& config) noexcept;

    /// @brief Destructor.
    ~Camera2D() noexcept = default;

    Camera2D(const Camera2D&) = default;
    Camera2D& operator=(const Camera2D&) = default;
    Camera2D(Camera2D&&) noexcept = default;
    Camera2D& operator=(Camera2D&&) noexcept = default;

    /// @brief Updates camera tracking position toward vehicle with damping and shake decay.
    /// @param[in] targetPos World position of target vehicle in meters.
    /// @param[in] forwardSpeed Longitudinal vehicle velocity in m/s.
    /// @param[in] dt Timestep duration in seconds.
    void update(const vehicle_sim::math::Vec2& targetPos, double forwardSpeed, double dt) noexcept;

    /// @brief Immediately snaps camera position to target coordinates without smoothing.
    /// @param[in] targetPos World position in meters.
    void reset(const vehicle_sim::math::Vec2& targetPos) noexcept;

    /// @brief Transforms world coordinates to virtual screen pixel coordinates.
    /// @param[in] worldPos World coordinates in meters.
    /// @return Virtual screen coordinates in pixels.
    [[nodiscard]] vehicle_sim::math::Vec2 worldToScreen(
        const vehicle_sim::math::Vec2& worldPos) const noexcept;

    /// @brief Transforms virtual screen pixel coordinates to world coordinates.
    /// @param[in] screenPos Virtual screen coordinates in pixels.
    /// @return World coordinates in meters.
    [[nodiscard]] vehicle_sim::math::Vec2 screenToWorld(
        const vehicle_sim::math::Vec2& screenPos) const noexcept;

    /// @brief Injects dynamic trauma inducing camera screen shake.
    /// @param[in] trauma Trauma increment clamped to [0.0, 1.0].
    void addTrauma(double trauma) noexcept;

    /// @brief Gets minimum visible world X coordinate.
    /// @return Left viewport boundary in meters.
    [[nodiscard]] double getVisibleMinX() const noexcept;

    /// @brief Gets maximum visible world X coordinate.
    /// @return Right viewport boundary in meters.
    [[nodiscard]] double getVisibleMaxX() const noexcept;

    /// @brief Gets minimum visible world Y coordinate.
    /// @return Bottom viewport boundary in meters.
    [[nodiscard]] double getVisibleMinY() const noexcept;

    /// @brief Gets maximum visible world Y coordinate.
    /// @return Top viewport boundary in meters.
    [[nodiscard]] double getVisibleMaxY() const noexcept;

    /// @brief Gets active camera tracking position.
    /// @return World position of camera center in meters.
    [[nodiscard]] const vehicle_sim::math::Vec2& getPosition() const noexcept;

    /// @brief Gets active camera shake offset in meters.
    /// @return World offset caused by trauma shake.
    [[nodiscard]] const vehicle_sim::math::Vec2& getShakeOffset() const noexcept;

    /// @brief Gets current trauma intensity in [0.0, 1.0].
    /// @return Trauma scalar value.
    [[nodiscard]] double getTrauma() const noexcept;

    /// @brief Gets camera configuration settings.
    /// @return Constant reference to active CameraConfig.
    [[nodiscard]] const CameraConfig& getConfig() const noexcept;

    /// @brief Sets camera configuration settings.
    /// @param[in] config New configuration parameters.
    void setConfig(const CameraConfig& config) noexcept;

private:
    CameraConfig config_ {};
    vehicle_sim::math::Vec2 position_ { 0.0, 0.0 };
    vehicle_sim::math::Vec2 shakeOffset_ { 0.0, 0.0 };
    double trauma_ { 0.0 };
    double shakeTime_ { 0.0 };
};

} // namespace montsoft_offroad
