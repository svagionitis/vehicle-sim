#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

#include "Ray2D.h"
#include "Vec2.h"

namespace vehicle_sim::math {

class OBB2D;

/// @class AABB2D
/// @brief 2D Axis-Aligned Bounding Box defined by minimum and maximum extents.
/// @details Represents a 2D bounding volume aligned with the Cartesian axes.
/// Provides efficient spatial testing, expansion, point containment, and slab-based raycasting.
class AABB2D {
public:
    Vec2 min { 0.0, 0.0 };
    Vec2 max { 0.0, 0.0 };

    /// @brief Default constructor creating a zero-sized box at the origin.
    constexpr AABB2D() noexcept = default;

    /// @brief Constructs an AABB with explicit min and max corner points.
    /// @param[in] inMin Minimum corner coordinates.
    /// @param[in] inMax Maximum corner coordinates.
    constexpr AABB2D(const Vec2& inMin, const Vec2& inMax) noexcept
        : min { inMin }
        , max { inMax }
    {
    }

    /// @brief Creates an AABB from a center point and half-extents.
    /// @param[in] center Geometric center of the bounding box.
    /// @param[in] halfExtents Half-width and half-height along axes.
    /// @return Axis-aligned bounding box.
    [[nodiscard]] static constexpr AABB2D fromCenterExtents(const Vec2& center, const Vec2& halfExtents) noexcept
    {
        return AABB2D { center - halfExtents, center + halfExtents };
    }

    /// @brief Computes a tight bounding box enclosing an array of points.
    /// @param[in] points Pointer to point array.
    /// @param[in] count Number of points in the array.
    /// @return Enclosing AABB, or zero box if count is zero.
    [[nodiscard]] static AABB2D fromPoints(const Vec2* points, std::size_t count) noexcept;

    /// @brief Calculates the center coordinate of the box.
    /// @return Center point.
    [[nodiscard]] constexpr Vec2 center() const noexcept
    {
        return (min + max) * 0.5;
    }

    /// @brief Calculates the total extents (width and height) of the box.
    /// @return Vector containing (width, height).
    [[nodiscard]] constexpr Vec2 extents() const noexcept
    {
        return max - min;
    }

    /// @brief Calculates the half-extents of the box.
    /// @return Vector containing half-width and half-height.
    [[nodiscard]] constexpr Vec2 halfExtents() const noexcept
    {
        return (max - min) * 0.5;
    }

    /// @brief Returns the width along the X axis.
    /// @return Box width.
    [[nodiscard]] constexpr double width() const noexcept
    {
        return max.x - min.x;
    }

    /// @brief Returns the height along the Y axis.
    /// @return Box height.
    [[nodiscard]] constexpr double height() const noexcept
    {
        return max.y - min.y;
    }

    /// @brief Returns the surface area of the 2D bounding box.
    /// @return Area of the box.
    [[nodiscard]] constexpr double area() const noexcept
    {
        const double w { width() };
        const double h { height() };
        return (w > 0.0 && h > 0.0) ? (w * h) : 0.0;
    }

    /// @brief Checks if the box has valid orientation (min <= max).
    /// @return True if min coordinates are less than or equal to max coordinates.
    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return (min.x <= max.x) && (min.y <= max.y);
    }

    /// @brief Checks if a point lies within or on the boundaries of the box.
    /// @param[in] point 2D point to query.
    /// @return True if point is inside or on boundary.
    [[nodiscard]] constexpr bool contains(const Vec2& point) const noexcept
    {
        return (point.x >= min.x) && (point.x <= max.x) && (point.y >= min.y) && (point.y <= max.y);
    }

    /// @brief Checks if another AABB is fully contained within this box.
    /// @param[in] other Other bounding box.
    /// @return True if other box is completely inside.
    [[nodiscard]] constexpr bool contains(const AABB2D& other) const noexcept
    {
        return (other.min.x >= min.x) && (other.max.x <= max.x) && (other.min.y >= min.y) && (other.max.y <= max.y);
    }

    /// @brief Checks if this box overlaps with another AABB.
    /// @param[in] other Other bounding box.
    /// @return True if boxes overlap or touch.
    [[nodiscard]] constexpr bool intersects(const AABB2D& other) const noexcept
    {
        return (min.x <= other.max.x) && (max.x >= other.min.x) && (min.y <= other.max.y) && (max.y >= other.min.y);
    }

    /// @brief Expands the bounding box uniformly by a scalar margin on all sides.
    /// @param[in] amount Scalar margin to expand by.
    /// @return Expanded bounding box.
    [[nodiscard]] constexpr AABB2D expanded(double amount) const noexcept
    {
        const Vec2 delta { amount, amount };
        return AABB2D { min - delta, max + delta };
    }

    /// @brief Expands the bounding box along each axis independently.
    /// @param[in] amount Vector margins (x and y) to expand by.
    /// @return Expanded bounding box.
    [[nodiscard]] constexpr AABB2D expanded(const Vec2& amount) const noexcept
    {
        return AABB2D { min - amount, max + amount };
    }

    /// @brief Enlarges this bounding box to include a given point.
    /// @param[in] point Point to encapsulate.
    void encapsulate(const Vec2& point) noexcept;

    /// @brief Enlarges this bounding box to include another AABB.
    /// @param[in] other Bounding box to encapsulate.
    void encapsulate(const AABB2D& other) noexcept;

    /// @brief Clamps a point to the nearest location inside or on the box boundary.
    /// @param[in] point World space query point.
    /// @return Closest point on or within the AABB.
    [[nodiscard]] constexpr Vec2 closestPoint(const Vec2& point) const noexcept
    {
        return Vec2 { std::clamp(point.x, min.x, max.x), std::clamp(point.y, min.y, max.y) };
    }

    /// @brief Performs a ray-slab intersection test against this bounding box.
    /// @details Employs Kay-Kajiya slab tests with IEEE 754 division handling.
    /// @param[in] ray Ray2D query.
    /// @return RayHit2D if intersection occurs within ray range, or std::nullopt.
    [[nodiscard]] std::optional<RayHit2D> raycast(const Ray2D& ray) const noexcept;

    /// @brief Converts this AABB into an equivalent unrotated OBB2D.
    /// @return Oriented bounding box with zero rotation.
    [[nodiscard]] OBB2D toOBB() const noexcept;
};

} // namespace vehicle_sim::math
