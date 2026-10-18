#pragma once

#include <cstddef>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @enum TOIState
/// @brief Result status of a continuous collision time-of-impact query.
enum class TOIState : std::uint8_t {
    Unknown, ///< Uninitialized query state.
    Touching, ///< Shapes achieve contact at calculated time t* in [0, dt].
    Separated, ///< Shapes remain separated throughout entire interval [0, dt].
    Overlapped ///< Shapes are already overlapping at start time t = 0.
};

/// @struct TOIInput
/// @brief Input parameters for continuous collision detection between two colliders.
struct TOIInput {
    double dt { 0.016666666666666666 }; ///< Simulation time step in seconds.
    double tolerance { 1e-4 }; ///< Target distance tolerance for touch condition in meters.
    std::size_t maxIterations { 16 }; ///< Maximum iterations for root-finding.
};

/// @struct TOIOutput
/// @brief Detailed result of a continuous collision time-of-impact calculation.
struct TOIOutput {
    TOIState state { TOIState::Unknown }; ///< Final root finding state.
    double time { 1.0 }; ///< Normalized impact parameter in range [0.0, 1.0].
    math::Vec2 point { 0.0, 0.0 }; ///< World space contact point at time of impact.
    math::Vec2 normal { 0.0, 1.0 }; ///< Unit normal pointing from body A towards body B.
    std::size_t iterations { 0 }; ///< Number of solver iterations consumed.
};

} // namespace vehicle_sim::physics
