#pragma once

#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @class Integrator2D
/// @brief Symplectic Euler numerical integrator for 2D rigid body dynamics.
/// @details Implements semi-implicit Euler integration, updating velocities prior to positions
/// to preserve phase space symplectic invariants and ensure physical energy stability.
class Integrator2D {
public:
    /// @brief Integrates a rigid body forward by time delta dt.
    /// @details Updates velocities first using forces, torques, damping, and gravity,
    /// then updates position and orientation using new velocities, and clears accumulators.
    /// @param[in,out] body Target rigid body to integrate.
    /// @param[in] dt Time step in seconds (must be positive).
    /// @param[in] gravity Global gravity vector in m/s^2 (default 0).
    static void integrate(RigidBody2D& body, double dt, const math::Vec2& gravity = math::Vec2::zero()) noexcept;

    /// @brief Normalizes an angle into the [-pi, pi] interval.
    /// @param[in] angleRadians Input angle in radians.
    /// @return Normalized angle in radians.
    [[nodiscard]] static double normalizeAngle(double angleRadians) noexcept;
};

} // namespace vehicle_sim::physics
