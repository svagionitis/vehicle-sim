#pragma once

#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct Contact2D
/// @brief Geometric and physical properties of a 2D contact point.
struct Contact2D {
    math::Vec2 point { 0.0, 0.0 }; ///< World space contact position.
    math::Vec2 normal { 0.0,
        1.0 }; ///< Unit normal pointing from body A towards body B (or towards barrier if body B is nullptr).
    double penetration { 0.0 }; ///< Interpenetration depth in meters.
    double restitution { 0.0 }; ///< Coefficient of restitution [0.0 = inelastic, 1.0 = elastic].
    double friction { 0.0 }; ///< Coulomb friction coefficient.
};

/// @class ContactSolver2D
/// @brief Resolves collisions and contact dynamics via impulse equations.
/// @details Calculates normal restoration impulses and Coulomb tangential friction impulses
/// to resolve rigid body and static obstacle collisions.
class ContactSolver2D {
public:
    /// @brief Resolves collision impulse between two bodies or a body and static obstacle.
    /// @details If bodyB is nullptr, body A is treated as colliding with a static boundary.
    /// @param[in,out] bodyA First colliding rigid body.
    /// @param[in,out] bodyB Second colliding rigid body (or nullptr for static boundary).
    /// @param[in] contact Geometric contact parameters.
    /// @return True if a non-zero impulse was resolved and applied.
    [[nodiscard]] static bool resolve(RigidBody2D& bodyA, RigidBody2D* bodyB, const Contact2D& contact) noexcept;
};

} // namespace vehicle_sim::physics
