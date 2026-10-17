#include "BroadphasePair2D.h"

namespace vehicle_sim::physics {

bool BroadphasePair2D::matches(const BroadphasePair2D& other) const noexcept
{
    return (colliderA == other.colliderA && colliderB == other.colliderB)
        || (colliderA == other.colliderB && colliderB == other.colliderA);
}

bool operator==(const BroadphasePair2D& lhs, const BroadphasePair2D& rhs) noexcept
{
    return lhs.matches(rhs);
}

bool operator!=(const BroadphasePair2D& lhs, const BroadphasePair2D& rhs) noexcept
{
    return !lhs.matches(rhs);
}

} // namespace vehicle_sim::physics
