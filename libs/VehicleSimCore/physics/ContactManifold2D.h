#pragma once

#include <array>
#include <cstddef>

#include "ContactSolver2D.h"
#include "RigidBody2D.h"

namespace vehicle_sim::physics {

/// @struct ContactManifold2D
/// @brief Collision manifold containing up to 2 contact points between two bodies.
/// @details Designed with fixed-capacity std::array to eliminate dynamic heap allocations.
/// Represents physical interaction between bodyA and bodyB (or static barrier if bodyB is nullptr).
struct ContactManifold2D {
    RigidBody2D* bodyA { nullptr }; ///< Pointer to first colliding rigid body.
    RigidBody2D* bodyB { nullptr }; ///< Pointer to second rigid body (or nullptr for static barrier).
    std::array<Contact2D, 2> contacts {}; ///< Fixed contact storage for up to 2 points.
    std::size_t contactCount { 0 }; ///< Number of active contact points [0, 2].

    /// @brief Clears all contacts and resets body references.
    void clear() noexcept
    {
        bodyA = nullptr;
        bodyB = nullptr;
        contactCount = 0;
    }

    /// @brief Adds a contact point if capacity permits.
    /// @param[in] contact Contact to insert.
    /// @return True if inserted, false if capacity exceeded.
    bool addContact(const Contact2D& contact) noexcept
    {
        if (contactCount < contacts.size()) {
            contacts[contactCount] = contact;
            ++contactCount;
            return true;
        }
        return false;
    }
};

} // namespace vehicle_sim::physics
