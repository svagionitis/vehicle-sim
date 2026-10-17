#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::physics {

/// @struct BarrierBounds2D
/// @brief Axis-aligned rectangular arena boundary in world meters.
struct BarrierBounds2D {
    double left { -50.0 };   ///< Minimum X coordinate (left wall).
    double right { 50.0 };   ///< Maximum X coordinate (right wall).
    double bottom { -50.0 }; ///< Minimum Y coordinate (bottom wall).
    double top { 50.0 };      ///< Maximum Y coordinate (top wall).
};

/// @class BarrierSolver2D
/// @brief 4-pass support-based frictionless boundary impulse solver.
/// @details Implements Pat Kerr's 1996 boundary solver resolving multi-corner
/// penetrations against infinite-mass planar barriers. Resolves position penetration
/// and applies restitution impulses with low-velocity bounce suppression for resting contact.
class BarrierSolver2D {
public:
    /// @struct BarrierEdge
    /// @brief Internal representation of an inward planar barrier edge.
    struct BarrierEdge {
        math::Vec2 normal { 0.0, 0.0 }; ///< Inward-facing boundary normal.
        double plane { 0.0 };            ///< Boundary plane offset threshold.
    };

    /// @brief Solves boundary penetrations and contact impulses for a rigid body.
    /// @tparam SupportFn Callable with signature: math::Vec2(const math::Vec2& normal).
    /// @param[in,out] body Target rigid body to correct and receive contact impulses.
    /// @param[in] bounds World arena bounding coordinates.
    /// @param[in] supportFn Callback returning the body's furthest surface point facing normal.
    /// @return Maximum contact impulse magnitude in N*s resolved during this step.
    template <typename SupportFn>
    static double solveInside(RigidBody2D& body,
                             const BarrierBounds2D& bounds,
                             SupportFn&& supportFn,
                             double restitution = 0.55) noexcept
    {
        double maxImpulse { 0.0 };
        const std::array<BarrierEdge, 4> barrierEdges {{
            { math::Vec2 { 1.0, 0.0 }, bounds.left },
            { math::Vec2 { -1.0, 0.0 }, -bounds.right },
            { math::Vec2 { 0.0, 1.0 }, bounds.bottom },
            { math::Vec2 { 0.0, -1.0 }, -bounds.top }
        }};

        // Execute 4 sequential passes to resolve simultaneous multi-corner penetrations
        for (std::size_t pass { 0 }; pass < 4; ++pass) {
            for (const auto& edge : barrierEdges) {
                math::Vec2 point { supportFn(edge.normal) };
                const double penetration { edge.plane - point.dot(edge.normal) };
                if (penetration < -1e-8) {
                    continue;
                }

                // Positional correction bringing surface point back inside arena
                const math::Vec2 correction { edge.normal * std::max(0.0, penetration) };
                body.setPosition(body.getPosition() + correction);
                point += correction;

                // Only resolve impulse when contact point is moving into barrier
                const double normalSpeed { body.getVelocityAtPoint(point).dot(edge.normal) };
                if (normalSpeed >= 0.0) {
                    continue;
                }

                // Effective rotational lever arm for contact impulse
                const math::Vec2 r { point - body.getPosition() };
                const double arm { r.cross(edge.normal) };

                // Suppress tiny bounces so body can settle smoothly under gravity
                const double bounce { normalSpeed < -0.5 ? restitution : 0.0 };

                // Contact point effective inverse mass
                const double effectiveInvMass { body.getInverseMass() + (arm * arm) * body.getInverseInertia() };
                if (effectiveInvMass <= 1e-12) {
                    continue;
                }

                const double velocityChange { -(1.0 + bounce) * normalSpeed };
                const double impulseMagnitude { velocityChange / effectiveInvMass };
                maxImpulse = std::max(maxImpulse, impulseMagnitude);
                const math::Vec2 impulse { edge.normal * impulseMagnitude };

                body.applyImpulseAtPoint(impulse, point);
            }
        }
        return maxImpulse;
    }
};

} // namespace vehicle_sim::physics
