#pragma once

#include <cmath>
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::terrain {

/// @struct HydroplaningConfig
/// @brief Physical and geometric parameters governing tire hydroplaning.
struct HydroplaningConfig {
    double tirePressurePsi { 32.0 }; ///< Tire inflation pressure in psi (typically 24.0 - 45.0).
    double treadDepth { 0.008 }; ///< Tread groove depth in meters (0.008 = 8mm new, 0.0 = slick).
    double voidRatio { 0.32 }; ///< Tread groove channel void area ratio in [0.0, 0.50].
    double liftCoefficient { 0.08 }; ///< Hydrodynamic lift coefficient C_L of the water wedge.
    double sprayDragCoeff { 0.30 }; ///< Water displacement and splash drag coefficient C_D.
    double waterDensity { 1000.0 }; ///< Water density rho in kg/m^3.
    double minFrictionScale { 0.05 }; ///< Minimum residual viscous friction factor eta_min.
    double hydroSharpness { 12.0 }; ///< Logistic decay slope parameter k_hydro.
    double horneMultiplier { 6.36 }; ///< NASA Horne empirical velocity coefficient C_horne.
};

/// @struct HydroplaningState
/// @brief Contact patch kinematics and environmental water inputs.
struct HydroplaningState {
    double forwardVelocity { 0.0 }; ///< Forward rolling velocity v_x in m/s.
    double normalLoad { 4000.0 }; ///< Vertical normal load F_z in Newtons.
    double patchWidth { 0.225 }; ///< Contact patch width b in meters.
    double patchLength { 0.150 }; ///< Contact patch length L in meters.
    double waterDepth { 0.0 }; ///< Standing surface water film depth d_w in meters.
    math::Vec2 velocityVector { 0.0, 0.0 }; ///< World contact velocity vector in m/s.
};

/// @struct HydroplaningResult
/// @brief Hydrodynamic output state, forces, and grip scaling factor.
struct HydroplaningResult {
    double criticalSpeed { 0.0 }; ///< NASA Horne hydroplaning velocity v_hydro in m/s.
    double unDrainedDepth { 0.0 }; ///< Excess un-drained water thickness d_excess in meters.
    double hydroLiftForce { 0.0 }; ///< Upward hydrodynamic fluid lift force F_lift in Newtons.
    double effectiveNormalLoad { 0.0 }; ///< Net normal load F_z,eff = max(0, F_z - F_lift) in N.
    double sprayDragForce { 0.0 }; ///< Retarding fluid displacement drag magnitude in Newtons.
    math::Vec2 sprayDragVector { 0.0, 0.0 }; ///< Planar fluid drag force vector opposing velocity.
    double frictionScale { 1.0 }; ///< Tractive friction multiplier eta_wet in [eta_min, 1.0].
    bool isHydroplaning { false }; ///< True if forward velocity exceeds critical speed with excess water.
};

/// @class HydroplaningSolver
/// @brief Physics solver for wet-weather dynamic tire hydroplaning and water drag.
class HydroplaningSolver {
public:
    HydroplaningSolver() = delete;

    /// @brief Computes NASA Horne critical dynamic hydroplaning speed.
    /// @param[in] pressurePsi Tire inflation pressure in psi.
    /// @param[in] horneMultiplier Empirical coefficient (default 6.36).
    /// @return Critical velocity in m/s.
    [[nodiscard]] static double computeCriticalSpeed(
        double pressurePsi, double horneMultiplier = 6.36) noexcept;

    /// @brief Computes excess un-evacuated water film thickness.
    /// @param[in] waterDepth Surface water film thickness d_w in meters.
    /// @param[in] treadDepth Tire tread groove depth d_tread in meters.
    /// @param[in] voidRatio Tread groove void ratio in [0.0, 1.0].
    /// @return Un-drained water thickness d_excess in meters.
    [[nodiscard]] static double computeUnDrainedDepth(
        double waterDepth, double treadDepth, double voidRatio) noexcept;

    /// @brief Computes upward hydrodynamic fluid lift force on the tire patch.
    /// @param[in] forwardSpeed Forward rolling velocity v_x in m/s.
    /// @param[in] unDrainedDepth Excess water film thickness in meters.
    /// @param[in] patchArea Contact patch area in m^2.
    /// @param[in] config Hydroplaning configuration parameters.
    /// @return Hydrodynamic lift force F_lift in Newtons.
    [[nodiscard]] static double computeHydroLift(
        double forwardSpeed,
        double unDrainedDepth,
        double patchArea,
        const HydroplaningConfig& config) noexcept;

    /// @brief Computes friction attenuation multiplier eta_wet.
    /// @param[in] forwardSpeed Forward rolling velocity v_x in m/s.
    /// @param[in] criticalSpeed NASA Horne critical speed v_hydro in m/s.
    /// @param[in] unDrainedDepth Excess water film thickness in meters.
    /// @param[in] config Hydroplaning configuration parameters.
    /// @return Friction scale factor in [eta_min, 1.0].
    [[nodiscard]] static double computeFrictionScale(
        double forwardSpeed,
        double criticalSpeed,
        double unDrainedDepth,
        const HydroplaningConfig& config) noexcept;

    /// @brief Computes water displacement and splashing drag force.
    /// @param[in] forwardSpeed Forward rolling velocity v_x in m/s.
    /// @param[in] waterDepth Total standing water depth in meters.
    /// @param[in] patchWidth Tire contact width in meters.
    /// @param[in] config Hydroplaning configuration parameters.
    /// @return Drag force magnitude in Newtons.
    [[nodiscard]] static double computeSprayDrag(
        double forwardSpeed,
        double waterDepth,
        double patchWidth,
        const HydroplaningConfig& config) noexcept;

    /// @brief Evaluates complete hydroplaning state, lift, spray drag, and grip scale.
    /// @param[in] state Contact patch kinematics and environmental water depth.
    /// @param[in] config Hydroplaning configuration parameters.
    /// @return Complete HydroplaningResult snapshot.
    [[nodiscard]] static HydroplaningResult evaluateHydroplaning(
        const HydroplaningState& state, const HydroplaningConfig& config) noexcept;
};

} // namespace vehicle_sim::terrain
