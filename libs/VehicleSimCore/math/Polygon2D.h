#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "AABB2D.h"
#include "Ray2D.h"
#include "Vec2.h"

namespace vehicle_sim::math {

/// @class Polygon2D
/// @brief 2D convex polygon primitive defined by ordered counter-clockwise vertices.
/// @details Designed with fixed capacity (up to 8 vertices) to eliminate dynamic allocations
/// and adhere strictly to MISRA C++:2023. Supports SAT collision queries, point containment,
/// closest-point projection, and analytical factories including fromEllipse() and fromOBB().
class Polygon2D {
public:
    static constexpr std::size_t kMaxVertices { 8 };

    /// @brief Default constructor creating an empty polygon.
    Polygon2D() noexcept = default;

    /// @brief Constructs a convex polygon from an array of CCW vertices.
    /// @param[in] inVertices Pointer to array of vertex coordinates.
    /// @param[in] count Number of vertices (clamped to kMaxVertices).
    Polygon2D(const Vec2* inVertices, std::size_t count) noexcept;

    /// @brief Factory creating an n-gon approximating an ellipse.
    /// @param[in] center Center coordinate.
    /// @param[in] radiusA Semi-major axis radius in meters.
    /// @param[in] radiusB Semi-minor axis radius in meters.
    /// @param[in] rotation Orientation angle in radians.
    /// @param[in] segments Number of vertices (clamped to [3, 8], defaults to 8).
    /// @return Convex polygon approximating the ellipse.
    [[nodiscard]] static Polygon2D fromEllipse(
        const Vec2& center, double radiusA, double radiusB, double rotation = 0.0, std::size_t segments = 8) noexcept;

    /// @brief Factory converting an oriented box to an equivalent 4-vertex polygon.
    /// @param[in] center Center of the oriented box.
    /// @param[in] halfExtents Half-width and half-height along local axes.
    /// @param[in] rotation Orientation angle in radians.
    /// @return 4-vertex convex polygon.
    [[nodiscard]] static Polygon2D fromOBB(const Vec2& center, const Vec2& halfExtents, double rotation = 0.0) noexcept;

    /// @brief Returns the number of valid vertices in the polygon.
    /// @return Vertex count in range [0, kMaxVertices].
    [[nodiscard]] std::size_t vertexCount() const noexcept;

    /// @brief Returns vertex coordinate by index.
    /// @param[in] index Vertex index (wrapped modulo vertexCount).
    /// @return Vertex coordinate in world space.
    [[nodiscard]] Vec2 getVertex(std::size_t index) const noexcept;

    /// @brief Computes outward unit normal for the edge starting at index.
    /// @param[in] index Edge index.
    /// @return Outward unit normal vector.
    [[nodiscard]] Vec2 getEdgeNormal(std::size_t index) const noexcept;

    /// @brief Computes the geometric centroid of the polygon.
    /// @return Centroid coordinate.
    [[nodiscard]] Vec2 computeCentroid() const noexcept;

    /// @brief Computes polygon signed area using the Shoelace formula.
    /// @return Positive area in square meters.
    [[nodiscard]] double computeArea() const noexcept;

    /// @brief Computes tight axis-aligned bounding box enclosing this polygon.
    /// @return Enclosing AABB.
    [[nodiscard]] AABB2D computeAABB() const noexcept;

    /// @brief Checks if a world space point lies inside or on the polygon boundary.
    /// @param[in] point World space query point.
    /// @return True if point is inside or on the boundary.
    [[nodiscard]] bool contains(const Vec2& point) const noexcept;

    /// @brief Clamps a point to the nearest location inside or on the polygon boundary.
    /// @param[in] point World space query point.
    /// @return Closest point on or inside the polygon.
    [[nodiscard]] Vec2 closestPoint(const Vec2& point) const noexcept;

    /// @brief Evaluates intersection with another convex polygon via SAT.
    /// @param[in] other Other convex polygon.
    /// @return True if polygons overlap or touch.
    [[nodiscard]] bool intersects(const Polygon2D& other) const noexcept;

    /// @brief Performs a raycast against this convex polygon.
    /// @param[in] ray Ray2D query.
    /// @return RayHit2D if intersection occurs within max distance, or std::nullopt.
    [[nodiscard]] std::optional<RayHit2D> raycast(const Ray2D& ray) const noexcept;

private:
    std::array<Vec2, kMaxVertices> mVertices {};
    std::size_t mCount { 0 };
};

} // namespace vehicle_sim::math
