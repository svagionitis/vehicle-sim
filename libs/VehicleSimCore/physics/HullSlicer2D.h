#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct Waterline2D
/// @brief Geometric definition of a 2D planar water surface boundary.
/// @details Represents the surface as an oriented line passing through point referencePoint
/// with an outward unit normal pointing into the air domain.
struct Waterline2D {
    math::Vec2 referencePoint { 0.0, 0.0 }; ///< Point on waterline in world coordinates.
    math::Vec2 outwardNormal { 0.0, 1.0 };  ///< Unit normal pointing out of water into air.

    /// @brief Creates a calm horizontal waterline at constant elevation y.
    /// @param[in] elevationY Vertical elevation in meters.
    /// @return Horizontal Waterline2D instance.
    [[nodiscard]] static constexpr Waterline2D horizontal(double elevationY) noexcept {
        return Waterline2D { math::Vec2 { 0.0, elevationY }, math::Vec2 { 0.0, 1.0 } };
    }

    /// @brief Computes signed perpendicular distance from point to waterline.
    /// @param[in] point World space query point.
    /// @return Negative if submerged, positive if airborne, zero on surface.
    [[nodiscard]] double getSignedDistance(const math::Vec2& point) const noexcept {
        return (point - referencePoint).dot(outwardNormal);
    }

    /// @brief Checks if a point lies in or on the liquid medium.
    /// @param[in] point World space query point.
    /// @return True if point is submerged or on waterline.
    [[nodiscard]] bool isSubmerged(const math::Vec2& point) const noexcept {
        return getSignedDistance(point) <= 0.0;
    }
};

/// @enum SubmergenceState
/// @brief Classification of hull immersion relative to waterline.
enum class SubmergenceState : std::uint8_t {
    Airborne,           ///< Hull is completely above water (zero submergence).
    PartiallySubmerged, ///< Hull intersects water surface (active waterline cut).
    FullySubmerged      ///< Hull is completely underwater.
};

/// @struct SubmergedSlice2D
/// @brief Geometric result of slicing a 2D hull against a waterline plane.
struct SubmergedSlice2D {
    static constexpr std::size_t kMaxSliceVertices { 10U };

    std::array<math::Vec2, kMaxSliceVertices> vertices {}; ///< Submerged polygon vertices in CCW order.
    std::size_t vertexCount { 0U };                       ///< Number of valid submerged vertices.
    double submergedArea { 0.0 };                         ///< Displaced fluid area in square meters.
    math::Vec2 centerOfBuoyancy { 0.0, 0.0 };             ///< Centroid of displaced fluid in world meters.

    math::Vec2 cutPointA { 0.0, 0.0 };                    ///< First intersection point with waterline.
    math::Vec2 cutPointB { 0.0, 0.0 };                    ///< Second intersection point with waterline.
    double waterplaneWidth { 0.0 };                       ///< Beam length of waterplane cut in meters.
    double waterplaneInertia { 0.0 };                     ///< Second moment of waterplane cut (L^3 / 12).

    SubmergenceState state { SubmergenceState::Airborne }; ///< Immersion state.

    /// @brief Queries whether any portion of the hull is in contact with water.
    /// @return True if partially or fully submerged.
    [[nodiscard]] bool isSubmerged() const noexcept {
        return state != SubmergenceState::Airborne;
    }
};

/// @class HullSlicer2D
/// @brief Geometric processor for slicing rigid vehicle hulls against waterlines.
/// @details Implements Sutherland-Hodgman polygon clipping, Shoelace area integration,
/// centroid/center-of-buoyancy computation, and waterplane moment evaluations.
/// Operates with zero dynamic allocations using fixed-capacity stack storage.
class HullSlicer2D {
public:
    HullSlicer2D() = delete;

    /// @brief Slices a convex polygon against an oriented waterline plane.
    /// @param[in] polygon Convex polygon hull in world coordinates.
    /// @param[in] waterline Oriented water boundary line.
    /// @return SubmergedSlice2D containing displaced area and centroid.
    [[nodiscard]] static SubmergedSlice2D slicePolygon(
        const math::Polygon2D& polygon,
        const Waterline2D& waterline) noexcept;

    /// @brief Slices an oriented bounding box against an oriented waterline plane.
    /// @param[in] obb Oriented bounding box hull in world coordinates.
    /// @param[in] waterline Oriented water boundary line.
    /// @return SubmergedSlice2D containing displaced area and centroid.
    [[nodiscard]] static SubmergedSlice2D sliceOBB(
        const math::OBB2D& obb,
        const Waterline2D& waterline) noexcept;

    /// @brief Slices a circular hull against an oriented waterline plane.
    /// @param[in] circle Circular hull in world coordinates.
    /// @param[in] waterline Oriented water boundary line.
    /// @param[in] segments Approximation fidelity for circular segment (defaults to 8).
    /// @return SubmergedSlice2D containing displaced area and centroid.
    [[nodiscard]] static SubmergedSlice2D sliceCircle(
        const math::Circle2D& circle,
        const Waterline2D& waterline,
        std::size_t segments = 8U) noexcept;

    /// @brief Computes area and geometric centroid of an ordered polygon.
    /// @param[in] inVertices Pointer to array of vertex coordinates.
    /// @param[in] count Number of vertices.
    /// @param[out] outArea Computed area in square meters.
    /// @param[out] outCentroid Computed centroid coordinates.
    static void computeMoments(
        const math::Vec2* inVertices,
        std::size_t count,
        double& outArea,
        math::Vec2& outCentroid) noexcept;
};

} // namespace vehicle_sim::physics
