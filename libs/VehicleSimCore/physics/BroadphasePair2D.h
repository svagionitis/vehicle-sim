#pragma once

#include <cstdint>

namespace vehicle_sim::physics {

class Collider2D;

/// @struct BroadphasePair2D
/// @brief Candidate collision pair identified during broadphase spatial hierarchy queries.
/// @details Represents two colliders whose fat AABBs overlap in world space.
struct BroadphasePair2D {
    Collider2D* colliderA { nullptr }; ///< Pointer to first collider.
    Collider2D* colliderB { nullptr }; ///< Pointer to second collider.
    std::int32_t proxyA { -1 };        ///< Broadphase proxy identifier of first collider.
    std::int32_t proxyB { -1 };        ///< Broadphase proxy identifier of second collider.

    /// @brief Compares two pairs for equivalence regardless of collider ordering.
    /// @param[in] other Other pair to compare.
    /// @return True if pairs refer to the same two colliders.
    [[nodiscard]] bool matches(const BroadphasePair2D& other) const noexcept;
};

/// @brief Equality operator for broadphase pairs (order-independent).
/// @param[in] lhs Left-hand broadphase pair.
/// @param[in] rhs Right-hand broadphase pair.
/// @return True if pairs refer to the same colliders.
[[nodiscard]] bool operator==(const BroadphasePair2D& lhs, const BroadphasePair2D& rhs) noexcept;

/// @brief Inequality operator for broadphase pairs.
/// @param[in] lhs Left-hand broadphase pair.
/// @param[in] rhs Right-hand broadphase pair.
/// @return True if pairs differ.
[[nodiscard]] bool operator!=(const BroadphasePair2D& lhs, const BroadphasePair2D& rhs) noexcept;

} // namespace vehicle_sim::physics
