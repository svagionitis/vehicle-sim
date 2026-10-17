#include "ShallowWaterCoupler.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

HydrodynamicCouplingForces ShallowWaterCoupler::updateFsi(
    physics::RigidBody2D& body,
    WaveGrid2D& waves,
    const WaterWakeStamp2D& stamp,
    double dt) noexcept
{
    HydrodynamicCouplingForces forces {};
    if (dt <= 0.0) {
        return forces;
    }

    const math::Vec2 pos { body.getPosition() };
    const double orientation { body.getOrientation() };
    const double angularVel { body.getAngularVelocity() };
    const double mass { body.getMass() };

    const WaveSample2D wave { waves.getSample(pos) };
    const double submergedDepth { std::max(0.0, wave.elevation - pos.y) };
    forces.submergedDepth = submergedDepth;
    forces.inWater = (submergedDepth > 0.01);

    if (forces.inWater && mass > 0.0) {
        // 1. Dynamic Archimedes buoyancy force
        const double buoyancy { computeBuoyancy(pos.y, wave.elevation, stamp.draft, mass) };
        forces.buoyancyForce = buoyancy;

        // 2. Wave downslope surfing thrust force
        const double surfingThrust { mass * 9.80665 * std::sin(wave.slopeAngle) };
        forces.waveSurfingForce = surfingThrust;

        // 3. Hydrostatic righting / wave tilting moment
        constexpr double kRestoringStiffness { 14000.0 };
        constexpr double kPitchDamping { 2500.0 };
        const double torque { computeWaveTorque(
            orientation, wave.slopeAngle, kRestoringStiffness, kPitchDamping, angularVel) };
        forces.pitchTorque = torque;

        // Apply forces to RigidBody2D
        const math::Vec2 verticalBuoyancyForce { 0.0, buoyancy };
        const math::Vec2 waveSurfingVec { surfingThrust * std::cos(orientation), 0.0 };
        forces.totalForce = verticalBuoyancyForce + waveSurfingVec;

        body.applyForce(forces.totalForce);
        body.applyTorque(forces.pitchTorque);
    }

    // 4. Deposit hull wake into wave grid
    waves.applyWakeStamp(stamp);

    return forces;
}

double ShallowWaterCoupler::computeBuoyancy(
    double hullPosY, double waveElevation, double draft, double displacement) noexcept
{
    const double submerged { std::max(0.0, waveElevation - hullPosY) };
    const double refDraft { std::max(0.05, draft) };
    const double submergenceRatio { std::clamp(submerged / refDraft, 0.0, 1.5) };
    constexpr double kGravity { 9.80665 };
    return submergenceRatio * displacement * kGravity;
}

double ShallowWaterCoupler::computeWaveTorque(
    double hullOrientation,
    double waveSlopeAngle,
    double stiffness,
    double damping,
    double angularVelocity) noexcept
{
    const double deltaAngle { hullOrientation - waveSlopeAngle };
    return (-std::sin(deltaAngle) * stiffness) - (damping * angularVelocity);
}

} // namespace vehicle_sim::terrain
