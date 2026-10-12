#pragma once

#include <cmath>
#include <limits>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @struct RayHit2D
/// @brief Represents the intersection result of a 2D raycast query.
/// @details Contains the hit position in world space, surface contact normal,
/// parametric distance along the ray direction, and normalized fraction of maximum range.
struct RayHit2D {
    Vec2 point { 0.0, 0.0 };
    Vec2 normal { 0.0, 0.0 };
    double distance { 0.0 };
    double fraction { 0.0 };
};

/// @class Ray2D
/// @brief Directed semi-infinite or bounded 2D ray for spatial collision queries.
/// @details Defined by an origin point, unit direction vector, and maximum query range.
/// Adheres to IEEE 754 floating-point determinism and pure C++17 semantics.
class Ray2D {
public:
    Vec2 origin { 0.0, 0.0 };
    Vec2 direction { 1.0, 0.0 };
    double maxDistance { std::numeric_limits<double>::infinity() };

    /// @brief Default constructor initializing to unit ray from origin along X axis.
    constexpr Ray2D() noexcept = default;

    /// @brief Constructs a ray with given origin, direction, and maximum range.
    /// @details Automatically normalizes the direction vector if non-zero.
    /// @param[in] inOrigin Starting point of the ray.
    /// @param[in] inDirection Direction vector of the ray.
    /// @param[in] inMaxDist Maximum distance along the ray (defaults to infinity).
    Ray2D(const Vec2& inOrigin, const Vec2& inDirection,
        double inMaxDist = std::numeric_limits<double>::infinity()) noexcept;

    /// @brief Evaluates point along the ray at parametric distance t.
    /// @details Computes origin + (direction * t).
    /// @param[in] t Distance along ray direction.
    /// @return Evaluated 2D point.
    [[nodiscard]] constexpr Vec2 pointAt(double t) const noexcept
    {
        return origin + (direction * t);
    }

    /// @brief Returns a copy of this ray with updated maximum distance.
    /// @param[in] inMaxDist New maximum distance constraint.
    /// @return Modified ray copy.
    [[nodiscard]] constexpr Ray2D withMaxDistance(double inMaxDist) const noexcept
    {
        Ray2D copy { *this };
        copy.maxDistance = inMaxDist;
        return copy;
    }

    /// @brief Transforms the ray by translation and rotation.
    /// @details Rotates direction and transforms origin.
    /// @param[in] translation Translation offset.
    /// @param[in] rotationRad Rotation angle in radians.
    /// @return Transformed ray.
    [[nodiscard]] Ray2D transformed(const Vec2& translation, double rotationRad) const noexcept;
};

} // namespace vehicle_sim::math
