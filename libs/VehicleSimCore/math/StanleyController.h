#pragma once

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/Spline2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @struct StanleyConfig
/// @brief Configuration tuning parameters for the Stanley steering controller.
struct StanleyConfig {
    double crossTrackGain { 1.2 }; ///< Cross-track error gain k_e in 1/s.
    double headingGain { 1.0 }; ///< Heading alignment proportional gain.
    double dampingGain { 0.1 }; ///< Yaw rate damping gain k_d to suppress oscillations.
    double frontAxleOffset { 1.4 }; ///< Distance L_f from COG to front axle in meters.
    double minVelocity { 1.0 }; ///< Regularization velocity floor in m/s to prevent low-speed singularities.
    double maxSteerAngle { 0.65 }; ///< Physical maximum steering lock in radians (approx 37.2 degrees).
};

/// @struct StanleyState
/// @brief Telemetry snapshot produced by a Stanley controller step.
struct StanleyState {
    double steerAngle { 0.0 }; ///< Commanded front steering angle in radians.
    double crossTrackError { 0.0 }; ///< Signed lateral distance e_y to reference path in meters.
    double headingError { 0.0 }; ///< Heading discrepancy theta_e in radians [-pi, pi).
    double pathDistance { 0.0 }; ///< Arc-length distance s* of projected point in meters.
    Vec2 frontAxlePos { 0.0, 0.0 }; ///< Computed world coordinates of front axle in meters.
    Vec2 closestPoint { 0.0, 0.0 }; ///< Projected closest point on spline in meters.
};

/// @class StanleyController
/// @brief Non-linear geometric front-axle trajectory tracking steering controller.
/// @details Implements the Stanley lateral control formulation proven on Stanford's autonomous vehicles.
/// Guarantees global exponential cross-track error convergence under kinematic vehicle models.
class StanleyController {
public:
    /// @brief Default constructor initializing with default tuning.
    constexpr StanleyController() noexcept = default;

    /// @brief Constructs a controller with explicit configuration parameters.
    /// @param[in] config Tuning parameters and geometric vehicle offsets.
    explicit StanleyController(const StanleyConfig& config) noexcept;

    /// @brief Sets the reference path spline to follow.
    /// @param[in] path Pointer to target Spline2D (must remain valid during evaluation).
    void setPath(const Spline2D* path) noexcept;

    /// @brief Sets active tuning configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const StanleyConfig& config) noexcept;

    /// @brief Gets active tuning configuration.
    /// @return Reference to active config.
    [[nodiscard]] const StanleyConfig& getConfig() const noexcept;

    /// @brief Gets assigned reference path pointer.
    /// @return Pointer to target spline, or nullptr if none set.
    [[nodiscard]] const Spline2D* getPath() const noexcept;

    /// @brief Computes front axle world position from COG position and heading.
    /// @param[in] cogPosition Center of gravity coordinates in world meters.
    /// @param[in] headingRad Vehicle body orientation angle in radians.
    /// @return World coordinates of the front axle center point.
    [[nodiscard]] Vec2 getFrontAxle(const Vec2& cogPosition, double headingRad) const noexcept;

    /// @brief Computes commanded steering angle for the current vehicle state.
    /// @param[in] cogPosition Center of gravity coordinates in world meters.
    /// @param[in] headingRad Vehicle body orientation angle in radians.
    /// @param[in] forwardSpeed Forward longitudinal speed v_x in m/s.
    /// @param[in] yawRate Current angular yaw rate in rad/s.
    /// @return Telemetry and computed steering angle clamped to maxSteerAngle.
    [[nodiscard]] StanleyState computeSteer(
        const Vec2& cogPosition, double headingRad, double forwardSpeed, double yawRate) const noexcept;

private:
    StanleyConfig mConfig {};
    const Spline2D* mPath { nullptr };
};

} // namespace vehicle_sim::math
