#pragma once

#include <optional>

#include "AABB2D.h"
#include "Ray2D.h"
#include "Vec2.h"

namespace vehicle_sim::math {

class OBB2D;

/// @class Circle2D
/// @brief 2D geometric circle primitive defined by center and radius.
/// @details Represents a circular bounding volume. Provides point containment,
/// closest-point projection, AABB generation, collision intersections, and analytical raycasting.
class Circle2D {
public:
    Vec2 center { 0.0, 0.0 };
    double radius { 0.5 };

    /// @brief Default constructor creating a circle at origin with radius 0.5.
    constexpr Circle2D() noexcept = default;

    /// @brief Constructs a circle with explicit center and radius.
    /// @param[in] inCenter Geometric center in world coordinates.
    /// @param[in] inRadius Radius in meters.
    constexpr Circle2D(const Vec2& inCenter, double inRadius) noexcept
        : center { inCenter }
        , radius { inRadius }
    {
    }

    /// @brief Checks if a world space point lies within or on the circle boundary.
    /// @param[in] point World space query point.
    /// @return True if point is inside or on the boundary.
    [[nodiscard]] bool contains(const Vec2& point) const noexcept;

    /// @brief Clamps a point to the nearest location inside or on the circle boundary.
    /// @param[in] point World space query point.
    /// @return Closest point on or within the circle.
    [[nodiscard]] Vec2 closestPoint(const Vec2& point) const noexcept;

    /// @brief Computes tight axis-aligned bounding box enclosing this circle.
    /// @return Enclosing AABB.
    [[nodiscard]] AABB2D computeAABB() const noexcept;

    /// @brief Evaluates intersection with another circle.
    /// @param[in] other Other circle primitive.
    /// @return True if circles overlap or touch.
    [[nodiscard]] bool intersects(const Circle2D& other) const noexcept;

    /// @brief Evaluates intersection with an axis-aligned bounding box.
    /// @param[in] aabb Axis-aligned bounding box.
    /// @return True if circle and AABB overlap.
    [[nodiscard]] bool intersects(const AABB2D& aabb) const noexcept;

    /// @brief Evaluates intersection with an oriented bounding box.
    /// @param[in] obb Oriented bounding box.
    /// @return True if circle and OBB overlap.
    [[nodiscard]] bool intersects(const OBB2D& obb) const noexcept;

    /// @brief Performs a raycast against this circle.
    /// @param[in] ray Ray2D query.
    /// @return RayHit2D if intersection occurs within max distance, or std::nullopt.
    [[nodiscard]] std::optional<RayHit2D> raycast(const Ray2D& ray) const noexcept;
};

} // namespace vehicle_sim::math
