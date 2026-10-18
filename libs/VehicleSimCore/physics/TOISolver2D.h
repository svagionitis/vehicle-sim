#pragma once

#include "Collider2D.h"
#include "TOIPrimitives2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Transform2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::physics {

/// @class TOISolver2D
/// @brief Exact and iterative time-of-impact root finder for 2D continuous collision detection.
/// @details Evaluates analytical quadratic equations for circular primitives and executes
/// Conservative Advancement for oriented bounding boxes, convex polygons, and heightfield terrain.
class TOISolver2D {
public:
    /// @brief Solves earliest time of impact between two general colliders.
    /// @param[in] colA First moving collider.
    /// @param[in] colB Second moving or static collider.
    /// @param[in] input Time interval and convergence parameters.
    /// @return TOIOutput containing impact state, normalized time alpha*, point, and normal.
    [[nodiscard]] static TOIOutput solveColliderTOI(
        const Collider2D& colA, const Collider2D& colB, const TOIInput& input) noexcept;

    /// @brief Solves analytical time of impact between two moving circles.
    /// @param[in] a Initial geometric circle A.
    /// @param[in] velA Linear velocity vector of circle A.
    /// @param[in] b Initial geometric circle B.
    /// @param[in] velB Linear velocity vector of circle B.
    /// @param[in] input Time step and tolerance settings.
    /// @return Analytical quadratic TOI solution.
    [[nodiscard]] static TOIOutput solveCircleCircleTOI(const math::Circle2D& a, const math::Vec2& velA,
        const math::Circle2D& b, const math::Vec2& velB, const TOIInput& input) noexcept;

    /// @brief Solves analytical time of impact between a moving circle and an oriented box.
    /// @param[in] circle Moving circle geometry.
    /// @param[in] circleVel Linear velocity of circle.
    /// @param[in] obb Oriented bounding box.
    /// @param[in] obbVel Linear velocity of box.
    /// @param[in] input Time step and tolerance settings.
    /// @return Analytical swept TOI solution.
    [[nodiscard]] static TOIOutput solveCircleOBBTOI(const math::Circle2D& circle, const math::Vec2& circleVel,
        const math::OBB2D& obb, const math::Vec2& obbVel, const TOIInput& input) noexcept;

    /// @brief Solves time of impact between two oriented boxes using Conservative Advancement.
    /// @param[in] a First oriented box.
    /// @param[in] velA Linear velocity of box A.
    /// @param[in] omegaA Angular velocity of box A.
    /// @param[in] b Second oriented box.
    /// @param[in] velB Linear velocity of box B.
    /// @param[in] omegaB Angular velocity of box B.
    /// @param[in] input Convergence parameters.
    /// @return Conservative advancement TOI solution.
    [[nodiscard]] static TOIOutput solveOBBOBBTOI(const math::OBB2D& a, const math::Vec2& velA, double omegaA,
        const math::OBB2D& b, const math::Vec2& velB, double omegaB, const TOIInput& input) noexcept;

    /// @brief Solves time of impact between two convex polygons via Conservative Advancement.
    /// @param[in] a First convex polygon.
    /// @param[in] velA Linear velocity of polygon A.
    /// @param[in] omegaA Angular velocity of polygon A.
    /// @param[in] b Second convex polygon.
    /// @param[in] velB Linear velocity of polygon B.
    /// @param[in] omegaB Angular velocity of polygon B.
    /// @param[in] input Convergence parameters.
    /// @return Conservative advancement TOI solution.
    [[nodiscard]] static TOIOutput solvePolygonTOI(const math::Polygon2D& a, const math::Vec2& velA, double omegaA,
        const math::Polygon2D& b, const math::Vec2& velB, double omegaB, const TOIInput& input) noexcept;

    /// @brief Solves time of impact between a moving collider and continuous heightfield terrain.
    /// @param[in] collider Moving collider.
    /// @param[in] terrain Continuous 1D heightfield terrain.
    /// @param[in] input Time step and convergence settings.
    /// @return Terrain TOI solution.
    [[nodiscard]] static TOIOutput solveTerrainTOI(
        const Collider2D& collider, const terrain::Heightfield1D& terrain, const TOIInput& input) noexcept;

    /// @brief Computes maximum bounding radius of a collider around its local center.
    /// @param[in] collider Target collider.
    /// @return Maximum radius enclosing all local vertices.
    [[nodiscard]] static double computeBoundRadius(const Collider2D& collider) noexcept;
};

} // namespace vehicle_sim::physics
