#pragma once

#include <cstddef>

#include "Collider2D.h"
#include "ContactManifold2D.h"

namespace vehicle_sim::physics {

/// @struct SpeculativeSettings
/// @brief Configuration parameters for speculative contact generation.
struct SpeculativeSettings {
    double maxDistance { 0.5 }; ///< Maximum separation distance to generate speculative contacts (m).
    double angleThreshold { 0.2588 }; ///< Cosine of maximum allowable approach angle (~75 deg).
    bool enableVoronoiClip { true }; ///< Enables Voronoi feature clipping to eliminate ghost collisions.
};

/// @class SpeculativeContact2D
/// @brief Predictive contact generator for fast non-penetrating bodies.
/// @details Generates contact points with positive separation distance and target velocity bias
/// to prevent tunneling within the sequential impulse velocity solver.
class SpeculativeContact2D {
public:
    /// @brief Detects and emits speculative contacts between two colliders if approaching.
    /// @param[in] colA First moving collider.
    /// @param[in] colB Second moving or static collider.
    /// @param[in] dt Simulation time step duration in seconds.
    /// @param[in] settings Speculative distance limits and angle thresholds.
    /// @param[out] outManifold Manifold receiving generated speculative contacts.
    /// @return True if one or more speculative contacts were generated.
    [[nodiscard]] static bool generateContacts(const Collider2D& colA, const Collider2D& colB,
        double dt, const SpeculativeSettings& settings, ContactManifold2D& outManifold) noexcept;
};

} // namespace vehicle_sim::physics
