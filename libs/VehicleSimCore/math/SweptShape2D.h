#pragma once

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @class SweptShape2D
/// @brief Analytical swept intersection tests and quadratic root solvers for 2D primitives.
/// @details Evaluates continuous time-of-impact between moving circular and bounding primitives
/// with IEEE 754 floating-point determinism and zero dynamic allocations.
class SweptShape2D {
public:
    /// @brief Solves quadratic equation a*t^2 + b*t + c = 0 for earliest non-negative root.
    /// @param[in] a Quadratic coefficient.
    /// @param[in] b Linear coefficient.
    /// @param[in] c Constant coefficient.
    /// @param[out] outRoot Earliest non-negative root if found.
    /// @return True if a real non-negative root exists.
    [[nodiscard]] static bool solveQuadratic(
        double a, double b, double c, double& outRoot) noexcept;

    /// @brief Solves analytical time-of-impact between two moving circles.
    /// @param[in] a First circle geometry at start of step.
    /// @param[in] velA Velocity vector of circle A.
    /// @param[in] b Second circle geometry at start of step.
    /// @param[in] velB Velocity vector of circle B.
    /// @param[in] maxTime Maximum search time (time step dt).
    /// @param[out] outT Impact time in range [0, maxTime].
    /// @param[out] outNormal Contact normal pointing from A towards B.
    /// @param[out] outPoint World position of contact point.
    /// @return True if collision occurs within [0, maxTime].
    [[nodiscard]] static bool solveSweptCircleCircle(const Circle2D& a, const Vec2& velA,
        const Circle2D& b, const Vec2& velB, double maxTime, double& outT, Vec2& outNormal,
        Vec2& outPoint) noexcept;

    /// @brief Solves analytical time-of-impact between a moving circle and stationary segment.
    /// @param[in] circle Moving circle geometry.
    /// @param[in] vel Velocity vector of circle.
    /// @param[in] p1 Segment start point.
    /// @param[in] p2 Segment end point.
    /// @param[in] maxTime Maximum search time.
    /// @param[out] outT Impact time in range [0, maxTime].
    /// @param[out] outNormal Contact normal pointing from circle towards segment.
    /// @param[out] outPoint World position of contact point.
    /// @return True if collision occurs within [0, maxTime].
    [[nodiscard]] static bool solveSweptCircleSegment(const Circle2D& circle, const Vec2& vel,
        const Vec2& p1, const Vec2& p2, double maxTime, double& outT, Vec2& outNormal,
        Vec2& outPoint) noexcept;

    /// @brief Solves time-of-impact between a moving circle and an oriented box.
    /// @param[in] circle Moving circle geometry.
    /// @param[in] velCircle Velocity vector of circle.
    /// @param[in] obb Oriented bounding box.
    /// @param[in] velOBB Velocity vector of box.
    /// @param[in] maxTime Maximum search time.
    /// @param[out] outT Impact time in range [0, maxTime].
    /// @param[out] outNormal Contact normal pointing from circle towards OBB.
    /// @param[out] outPoint World position of contact point.
    /// @return True if collision occurs within [0, maxTime].
    [[nodiscard]] static bool solveSweptCircleOBB(const Circle2D& circle, const Vec2& velCircle,
        const OBB2D& obb, const Vec2& velOBB, double maxTime, double& outT, Vec2& outNormal,
        Vec2& outPoint) noexcept;

    /// @brief Solves swept axis-aligned bounding box intersection.
    /// @param[in] a First axis-aligned box.
    /// @param[in] velA Velocity vector of box A.
    /// @param[in] b Second axis-aligned box.
    /// @param[in] velB Velocity vector of box B.
    /// @param[in] maxTime Maximum search time.
    /// @param[out] outT Impact time in range [0, maxTime].
    /// @param[out] outNormal Contact normal pointing from A towards B.
    /// @return True if collision occurs within [0, maxTime].
    [[nodiscard]] static bool solveSweptAABB(const AABB2D& a, const Vec2& velA,
        const AABB2D& b, const Vec2& velB, double maxTime, double& outT, Vec2& outNormal) noexcept;
};

} // namespace vehicle_sim::math
