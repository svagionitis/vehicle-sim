#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/HullSlicer2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::physics {

class Collider2D;

/// @struct BuoyancyConfig2D
/// @brief Configuration settings for 2D hydrostatic and hydrodynamic force evaluation.
struct BuoyancyConfig2D {
    double fluidDensity { 1000.0 };         ///< Liquid mass density in kg/m^3 (fresh: 1000, sea: 1025).
    math::Vec2 gravity { 0.0, -9.80665 };   ///< Ambient gravitational acceleration vector in m/s^2.
    double dragCoefficientLong { 0.35 };    ///< Longitudinal streamlined form drag coefficient.
    double dragCoefficientLat { 1.85 };     ///< Transverse broadside / keel resistance drag coefficient.
    double rotationalDamping { 0.25 };      ///< Viscous rotational damping coefficient in N*s/(rad*m).
    double linearDamping { 15.0 };          ///< Linear skin friction resistance in N*s/m.
    double suctionCoefficient { 0.0 };      ///< Bernoulli hull bottom suction coefficient.
};

/// @struct BuoyancyResult2D
/// @brief Snapshot of evaluated hydrostatic and hydrodynamic forces.
struct BuoyancyResult2D {
    math::Vec2 totalForce { 0.0, 0.0 };     ///< Net world force applied to rigid body in Newtons.
    double totalTorque { 0.0 };             ///< Net torque applied to rigid body in N*m.
    math::Vec2 buoyantForce { 0.0, 0.0 };   ///< Pure Archimedes hydrostatic buoyant force in Newtons.
    double rightingTorque { 0.0 };          ///< Hydrostatic righting moment about center of mass in N*m.
    math::Vec2 dragForce { 0.0, 0.0 };      ///< Total hydrodynamic skin and form drag force in Newtons.
    double dampingTorque { 0.0 };           ///< Hydrodynamic viscous angular damping moment in N*m.

    double submergedArea { 0.0 };           ///< Submerged displacement area in m^2.
    math::Vec2 centerOfBuoyancy { 0.0, 0.0 }; ///< World space center of buoyancy.
    double metacentricHeight { 0.0 };       ///< Metacentric height GM in meters (positive = stable).
    SubmergenceState state { SubmergenceState::Airborne }; ///< Immersion state.
};

/// @class Buoyancy2D
/// @brief Evaluates hydrostatic buoyancy, righting stability, and hydrodynamic drag forces.
/// @details Couples geometric submerged hull slices from HullSlicer2D with RigidBody2D kinetics.
class Buoyancy2D {
public:
    Buoyancy2D() = delete;

    /// @brief Evaluates buoyancy and hydrodynamic drag from a precomputed submerged slice.
    /// @param[in] slice Submerged hull geometry produced by HullSlicer2D.
    /// @param[in] cog World space center of gravity of the vehicle.
    /// @param[in] linearVelocity World linear velocity of the vehicle in m/s.
    /// @param[in] angularVelocity Scalar angular velocity of the vehicle in rad/s.
    /// @param[in] headingAngle Vehicle orientation angle in radians.
    /// @param[in] fluidVelocity Ambient water current velocity vector in m/s.
    /// @param[in] config Hydrostatic and hydrodynamic configuration parameters.
    /// @return BuoyancyResult2D containing accumulated forces and torques.
    [[nodiscard]] static BuoyancyResult2D computeForces(
        const SubmergedSlice2D& slice,
        const math::Vec2& cog,
        const math::Vec2& linearVelocity,
        double angularVelocity,
        double headingAngle,
        const math::Vec2& fluidVelocity,
        const BuoyancyConfig2D& config) noexcept;

    /// @brief Slices a convex polygon hull and directly applies hydrostatic forces to a rigid body.
    /// @param[in,out] body Target rigid body receiving forces.
    /// @param[in] polygon Convex hull in world coordinates.
    /// @param[in] waterline Oriented water surface boundary.
    /// @param[in] config Buoyancy simulation configuration.
    /// @param[in] fluidVelocity Ambient fluid flow vector in m/s.
    /// @return BuoyancyResult2D detailing applied forces.
    static BuoyancyResult2D applyBuoyancy(
        RigidBody2D& body,
        const math::Polygon2D& polygon,
        const Waterline2D& waterline,
        const BuoyancyConfig2D& config,
        const math::Vec2& fluidVelocity = { 0.0, 0.0 }) noexcept;

    /// @brief Slices an attached collider shape and directly applies hydrostatic forces to parent body.
    /// @param[in,out] body Target rigid body receiving forces.
    /// @param[in] collider Collider geometry attached to body.
    /// @param[in] waterline Oriented water surface boundary.
    /// @param[in] config Buoyancy simulation configuration.
    /// @param[in] fluidVelocity Ambient fluid flow vector in m/s.
    /// @return BuoyancyResult2D detailing applied forces.
    static BuoyancyResult2D applyToCollider(
        RigidBody2D& body,
        const Collider2D& collider,
        const Waterline2D& waterline,
        const BuoyancyConfig2D& config,
        const math::Vec2& fluidVelocity = { 0.0, 0.0 }) noexcept;
};

} // namespace vehicle_sim::physics
