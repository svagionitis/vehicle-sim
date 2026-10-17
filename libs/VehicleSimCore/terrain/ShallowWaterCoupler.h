#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/WaveGrid2D.h"

namespace vehicle_sim::terrain {

/// @struct HydrodynamicCouplingForces
/// @brief Result of two-way wave-hull interaction force evaluation.
struct HydrodynamicCouplingForces {
    math::Vec2 totalForce { 0.0, 0.0 }; ///< Total world force applied to hull in Newtons.
    double buoyancyForce { 0.0 };       ///< Dynamic vertical Archimedes buoyancy in Newtons.
    double waveSurfingForce { 0.0 };    ///< Downslope wave thrust force in Newtons.
    double pitchTorque { 0.0 };         ///< Hydrostatic righting / tilting torque in N*m.
    double submergedDepth { 0.0 };      ///< Instantaneous draft penetration depth in meters.
    bool inWater { false };             ///< True if hull is in contact with water surface.
};

/// @class ShallowWaterCoupler
/// @brief Coordinator for two-way fluid-structure interaction between vehicles and wave grids.
/// @details Evaluates dynamic Archimedes buoyancy, wave tilt moments, downslope surfing forces,
/// and deposits wake momentum impulses into WaveGrid2D or WaveGrid1D.
class ShallowWaterCoupler {
public:
    ShallowWaterCoupler() = delete;

    /// @brief Evaluates hydrodynamic interaction forces and updates wave basin.
    /// @param[in,out] body Target vehicle rigid body.
    /// @param[in,out] waves Active 2D wave basin.
    /// @param[in] stamp Vehicle hull footprint and parameters.
    /// @param[in] dt Simulation timestep in seconds.
    /// @return HydrodynamicCouplingForces containing computed reactions.
    [[nodiscard]] static HydrodynamicCouplingForces updateFsi(
        physics::RigidBody2D& body,
        WaveGrid2D& waves,
        const WaterWakeStamp2D& stamp,
        double dt) noexcept;

    /// @brief Computes dynamic Archimedes buoyancy force given local wave elevation.
    /// @param[in] hullPosY Vertical world position of hull keel in meters.
    /// @param[in] waveElevation Local water surface elevation in meters.
    /// @param[in] draft Nominal hull draft in meters.
    /// @param[in] displacement Mass displacement in kg.
    /// @return Vertical buoyant force in Newtons.
    [[nodiscard]] static double computeBuoyancy(
        double hullPosY, double waveElevation, double draft, double displacement) noexcept;

    /// @brief Computes hydrostatic pitch restoring torque from wave slope.
    /// @param[in] hullOrientation Heading angle of hull in radians.
    /// @param[in] waveSlopeAngle Surface inclination angle of wave in radians.
    /// @param[in] stiffness Restoring stiffness in N*m/rad.
    /// @param[in] damping Angular pitch damping in N*m*s.
    /// @param[in] angularVelocity Hull pitch rate in rad/s.
    /// @return Restoring pitch moment in N*m.
    [[nodiscard]] static double computeWaveTorque(
        double hullOrientation,
        double waveSlopeAngle,
        double stiffness,
        double damping,
        double angularVelocity) noexcept;
};

} // namespace vehicle_sim::terrain
