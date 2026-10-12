#pragma once

#include "FluidField2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct FluidDragProperties
/// @brief Aerodynamic and hydrodynamic vehicle geometry and drag coefficients.
/// @details Defines frontal and lateral reference areas, drag coefficients, and center-of-pressure offset.
struct FluidDragProperties {
    double frontalArea { 2.5 };       ///< Frontal projected cross-sectional area in m^2.
    double frontalDragCoeff { 0.35 }; ///< Dimensionless frontal form drag coefficient (Cd).
    double sideArea { 12.0 };         ///< Lateral broadside projected area in m^2.
    double sideDragCoeff { 1.1 };     ///< Dimensionless lateral crosswind drag coefficient (Cd,side).
    double cpOffset { 0.2 };          ///< Longitudinal center-of-pressure offset from center-of-mass in meters.
};

/// @struct CrosswindForces
/// @brief Result of aerodynamic crosswind force and moment evaluation.
/// @details Contains world-space combined force vector, local lateral side force, frontal drag, and yaw torque.
struct CrosswindForces {
    math::Vec2 totalForce { 0.0, 0.0 }; ///< Total aerodynamic force vector in world coordinates in Newtons.
    double sideForce { 0.0 };           ///< Lateral crosswind force magnitude in local frame in Newtons.
    double frontalDrag { 0.0 };         ///< Longitudinal drag force magnitude in local frame in Newtons.
    double yawMoment { 0.0 };           ///< Aerodynamic yaw or overturning moment around CM in N*m.
};

/// @class FluidDrag2D
/// @brief Deterministic aerodynamic and hydrodynamic force evaluation utilities.
/// @details Evaluates relative fluid velocities, quadratic form drag, crosswind lateral loads, and overturning moments.
/// Strictly conforms to IEEE 754 floating-point determinism and pure C++17 standards with zero external dependencies.
class FluidDrag2D {
public:
    FluidDrag2D() = delete;

    /// @brief Computes relative fluid velocity vector.
    /// @details Calculates V_rel = V_body - V_fluid.
    /// @param[in] bodyVelocity World linear velocity of vehicle in m/s.
    /// @param[in] fluidVelocity World fluid velocity at vehicle position in m/s.
    /// @return Relative velocity vector V_rel in m/s.
    [[nodiscard]] static math::Vec2 computeRelativeVel(
        const math::Vec2& bodyVelocity, const math::Vec2& fluidVelocity) noexcept;

    /// @brief Computes quadratic fluid form drag force vector in world frame.
    /// @details Computes F_drag = -0.5 * rho * Cd * A * ||V_rel|| * V_rel.
    /// @param[in] relativeVelocity Relative fluid velocity in m/s.
    /// @param[in] fluidDensity Fluid density in kg/m^3.
    /// @param[in] dragCoeff Dimensionless drag coefficient.
    /// @param[in] referenceArea Projected cross-sectional area in m^2.
    /// @return Opposing drag force vector in Newtons.
    [[nodiscard]] static math::Vec2 computeDragForce(
        const math::Vec2& relativeVelocity, double fluidDensity,
        double dragCoeff, double referenceArea) noexcept;

    /// @brief Computes combined frontal drag, crosswind side force, and yaw moment.
    /// @details Projects relative airflow into vehicle heading and lateral axes to evaluate
    /// longitudinal drag, crosswind side force, and overturning torque around the center of mass.
    /// @param[in] bodyVelocity World velocity of vehicle in m/s.
    /// @param[in] bodyOrientation Vehicle orientation angle in radians.
    /// @param[in] fluidSample Environmental fluid sample at vehicle position.
    /// @param[in] properties Vehicle aerodynamic geometric properties.
    /// @return CrosswindForces struct with total force and yaw moment.
    [[nodiscard]] static CrosswindForces computeCrosswind(
        const math::Vec2& bodyVelocity, double bodyOrientation,
        const FluidSample2D& fluidSample, const FluidDragProperties& properties) noexcept;
};

} // namespace vehicle_sim::physics
