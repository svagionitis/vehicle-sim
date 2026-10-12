#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "AABB2D.h"
#include "Ray2D.h"
#include "Vec2.h"

namespace vehicle_sim::math {

/// @class OBB2D
/// @brief 2D Oriented Bounding Box defined by center, half-extents, and rotation angle.
/// @details Represents a rotatable rectangular bounding volume. Supports Separating Axis
/// Theorem (SAT) collision queries, closest point projection, tight AABB generation,
/// and local-frame slab raycasting.
class OBB2D {
public:
    Vec2 center { 0.0, 0.0 };
    Vec2 halfExtents { 0.0, 0.0 };
    double rotation { 0.0 };

    /// @brief Default constructor creating a zero-sized unrotated box at the origin.
    constexpr OBB2D() noexcept = default;

    /// @brief Constructs an OBB with explicit center, half-extents, and rotation.
    /// @param[in] inCenter Geometric center in world coordinates.
    /// @param[in] inHalfExtents Half-width and half-height along local axes.
    /// @param[in] inRotationRad Orientation angle in radians (defaults to 0.0).
    constexpr OBB2D(const Vec2& inCenter, const Vec2& inHalfExtents, double inRotationRad = 0.0) noexcept
        : center { inCenter }
        , halfExtents { inHalfExtents }
        , rotation { inRotationRad }
    {
    }

    /// @brief Factory creating an OBB from center, half-extents, and rotation.
    /// @param[in] inCenter Geometric center in world coordinates.
    /// @param[in] inHalfExtents Half-width and half-height along local axes.
    /// @param[in] inRotationRad Orientation angle in radians.
    /// @return Oriented bounding box.
    [[nodiscard]] static constexpr OBB2D fromCenterExtents(
        const Vec2& inCenter, const Vec2& inHalfExtents, double inRotationRad = 0.0) noexcept
    {
        return OBB2D { inCenter, inHalfExtents, inRotationRad };
    }

    /// @brief Returns the unit vector along the local X axis in world coordinates.
    /// @return Local X axis unit vector.
    [[nodiscard]] Vec2 axisX() const noexcept
    {
        return Vec2 { std::cos(rotation), std::sin(rotation) };
    }

    /// @brief Returns the unit vector along the local Y axis in world coordinates.
    /// @return Local Y axis unit vector.
    [[nodiscard]] Vec2 axisY() const noexcept
    {
        return Vec2 { -std::sin(rotation), std::cos(rotation) };
    }

    /// @brief Computes the 4 corners of the oriented box in world coordinates.
    /// @return Array containing corners in counter-clockwise order.
    [[nodiscard]] std::array<Vec2, 4> vertices() const noexcept;

    /// @brief Transforms a point from world space to box local space.
    /// @param[in] worldPoint Point in world coordinates.
    /// @return Point in local box frame relative to center.
    [[nodiscard]] Vec2 toLocal(const Vec2& worldPoint) const noexcept;

    /// @brief Transforms a point from box local space to world space.
    /// @param[in] localPoint Point in local box frame.
    /// @return Point in world coordinates.
    [[nodiscard]] Vec2 toWorld(const Vec2& localPoint) const noexcept;

    /// @brief Checks if a world space point lies within the oriented box.
    /// @param[in] point World space query point.
    /// @return True if point is inside or on boundary.
    [[nodiscard]] bool contains(const Vec2& point) const noexcept;

    /// @brief Clamps a point to the nearest location inside or on the box boundary.
    /// @param[in] point World space query point.
    /// @return Closest point on or within the OBB in world coordinates.
    [[nodiscard]] Vec2 closestPoint(const Vec2& point) const noexcept;

    /// @brief Computes a tight axis-aligned bounding box enclosing this OBB.
    /// @return Enclosing AABB.
    [[nodiscard]] AABB2D computeAABB() const noexcept;

    /// @brief Evaluates intersection with another OBB via Separating Axis Theorem (SAT).
    /// @param[in] other Other oriented bounding box.
    /// @return True if boxes overlap or touch.
    [[nodiscard]] bool intersects(const OBB2D& other) const noexcept;

    /// @brief Evaluates intersection with an axis-aligned bounding box.
    /// @param[in] aabb Axis-aligned bounding box.
    /// @return True if box and AABB overlap.
    [[nodiscard]] bool intersects(const AABB2D& aabb) const noexcept;

    /// @brief Performs a raycast against this oriented box.
    /// @details Maps the ray into local coordinates and performs a slab test.
    /// @param[in] ray Ray2D query.
    /// @return RayHit2D if intersection occurs, or std::nullopt.
    [[nodiscard]] std::optional<RayHit2D> raycast(const Ray2D& ray) const noexcept;
};

/// @brief Type alias matching Section 3.2 specification in ProposedFeatures.md.
using OrientedBox2D = OBB2D;

} // namespace vehicle_sim::math
