#pragma once

#include <optional>

#include "AABB2D.h"
#include "Ray2D.h"
#include "Vec2.h"

namespace vehicle_sim::math {

class Circle2D;
class OBB2D;

/// @class Capsule2D
/// @brief 2D capsule primitive formed by sweeping a disc along a line segment.
/// @details Defined by two backbone segment endpoints and a radius. Provides
/// distance queries, point containment, AABB generation, intersection tests, and raycasting.
class Capsule2D {
public:
    Vec2 p1 { 0.0, -0.5 };
    Vec2 p2 { 0.0, 0.5 };
    double radius { 0.25 };

    /// @brief Default constructor creating a capsule aligned along Y axis.
    constexpr Capsule2D() noexcept = default;

    /// @brief Constructs a capsule with explicit endpoints and radius.
    /// @param[in] inP1 Starting backbone endpoint.
    /// @param[in] inP2 Ending backbone endpoint.
    /// @param[in] inRadius Cap and sleeve radius in meters.
    constexpr Capsule2D(const Vec2& inP1, const Vec2& inP2, double inRadius) noexcept
        : p1 { inP1 }
        , p2 { inP2 }
        , radius { inRadius }
    {
    }

    /// @brief Returns the vector pointing from p1 to p2.
    /// @return Direction vector (p2 - p1).
    [[nodiscard]] constexpr Vec2 segmentVector() const noexcept
    {
        return p2 - p1;
    }

    /// @brief Computes length of the backbone line segment.
    /// @return Segment length in meters.
    [[nodiscard]] double segmentLength() const noexcept;

    /// @brief Clamps a point to the nearest location on the backbone line segment.
    /// @param[in] point World space query point.
    /// @return Closest point along backbone segment [p1, p2].
    [[nodiscard]] Vec2 closestSegmentPoint(const Vec2& point) const noexcept;

    /// @brief Clamps a point to the nearest location inside or on the capsule volume.
    /// @param[in] point World space query point.
    /// @return Closest point on or inside the capsule.
    [[nodiscard]] Vec2 closestPoint(const Vec2& point) const noexcept;

    /// @brief Checks if a world space point lies inside or on the capsule boundary.
    /// @param[in] point World space query point.
    /// @return True if point is inside or on the boundary.
    [[nodiscard]] bool contains(const Vec2& point) const noexcept;

    /// @brief Computes tight axis-aligned bounding box enclosing this capsule.
    /// @return Enclosing AABB.
    [[nodiscard]] AABB2D computeAABB() const noexcept;

    /// @brief Evaluates intersection with another capsule.
    /// @param[in] other Other capsule primitive.
    /// @return True if capsules overlap or touch.
    [[nodiscard]] bool intersects(const Capsule2D& other) const noexcept;

    /// @brief Evaluates intersection with a circle.
    /// @param[in] circle Circle primitive.
    /// @return True if capsule and circle overlap.
    [[nodiscard]] bool intersects(const Circle2D& circle) const noexcept;

    /// @brief Evaluates intersection with an axis-aligned bounding box.
    /// @param[in] aabb Axis-aligned bounding box.
    /// @return True if capsule and AABB overlap.
    [[nodiscard]] bool intersects(const AABB2D& aabb) const noexcept;

    /// @brief Evaluates intersection with an oriented bounding box.
    /// @param[in] obb Oriented bounding box.
    /// @return True if capsule and OBB overlap.
    [[nodiscard]] bool intersects(const OBB2D& obb) const noexcept;

    /// @brief Performs a raycast against this capsule.
    /// @param[in] ray Ray2D query.
    /// @return RayHit2D if intersection occurs within max distance, or std::nullopt.
    [[nodiscard]] std::optional<RayHit2D> raycast(const Ray2D& ray) const noexcept;
};

} // namespace vehicle_sim::math
