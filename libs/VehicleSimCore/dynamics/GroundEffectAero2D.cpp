#include "GroundEffectAero2D.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

GroundEffectAero2D::GroundEffectAero2D() noexcept
    : GroundEffectAero2D(GroundEffectAeroConfig {})
{
}

GroundEffectAero2D::GroundEffectAero2D(const GroundEffectAeroConfig& config) noexcept
    : mConfig { config }
    , mFlapLimiter { config.flapOpenSpeedDegPerSec, config.flapCloseSpeedDegPerSec }
{
    mFlapLimiter.reset(0.0);
}

void GroundEffectAero2D::reset() noexcept
{
    mFlapLimiter.reset(0.0);
    mLastOutput = GroundEffectAeroOutput {};
    mIsStalled = false;
    mIsDrsInhibited = false;
}

const GroundEffectAeroConfig& GroundEffectAero2D::getConfig() const noexcept
{
    return mConfig;
}

void GroundEffectAero2D::setConfig(const GroundEffectAeroConfig& config) noexcept
{
    mConfig = config;
    mFlapLimiter.setRateLimits(config.flapOpenSpeedDegPerSec, config.flapCloseSpeedDegPerSec);
}

double GroundEffectAero2D::getFlapAngle() const noexcept
{
    return mFlapLimiter.value();
}

bool GroundEffectAero2D::isStalled() const noexcept
{
    return mIsStalled;
}

bool GroundEffectAero2D::isDrsActive() const noexcept
{
    return mFlapLimiter.value() > 1.0;
}

double GroundEffectAero2D::computeStall(double rideHeight) const noexcept
{
    if (rideHeight >= mConfig.stallRideHeight) {
        return 1.0;
    }
    if (rideHeight <= mConfig.minChokeRideHeight) {
        return mConfig.stallResidualRatio;
    }

    const double span { mConfig.stallRideHeight - mConfig.minChokeRideHeight };
    const double u { (rideHeight - mConfig.minChokeRideHeight) / std::max(span, 1e-6) };
    const double clampedU { std::clamp(u, 0.0, 1.0) };
    const double hermite { (3.0 * clampedU * clampedU) - (2.0 * clampedU * clampedU * clampedU) };

    return mConfig.stallResidualRatio + ((1.0 - mConfig.stallResidualRatio) * hermite);
}

bool GroundEffectAero2D::checkInterlock(const GroundEffectAeroInput& input) const noexcept
{
    if (input.isAirborne) {
        return false;
    }
    if (input.brakeDemand > mConfig.brakeDisengageThreshold) {
        return false;
    }
    if (std::abs(input.steerDemand) > mConfig.steerDisengageThreshold) {
        return false;
    }
    if (std::abs(input.lateralAccelerationG) > mConfig.lateralGDisengageThreshold) {
        return false;
    }
    return true;
}

GroundEffectAeroOutput GroundEffectAero2D::update(const GroundEffectAeroInput& input, const math::Vec2& bodyLinearVel,
    double bodyHeading, const physics::FluidSample2D& fluidSample, double dt) noexcept
{
    GroundEffectAeroOutput out {};

    // 1. Evaluate Relative Fluid Airspeed
    const math::Vec2 relativeVel { physics::FluidDrag2D::computeRelativeVel(bodyLinearVel, fluidSample.velocity) };
    const double airspeed { relativeVel.length() };

    // 2. Evaluate DRS Actuation & Interlock
    const bool interlockPermitted { checkInterlock(input) };
    mIsDrsInhibited = input.requestDrs && (!interlockPermitted);

    const double targetFlapDeg { (input.requestDrs && interlockPermitted) ? mConfig.maxFlapAngleDeg : 0.0 };
    out.flapAngleDeg = mFlapLimiter.update(targetFlapDeg, dt);
    out.isDrsActive = out.flapAngleDeg > 1.0;
    out.isDrsInhibited = mIsDrsInhibited;

    const double flapRatio { std::clamp(out.flapAngleDeg / std::max(mConfig.maxFlapAngleDeg, 1e-6), 0.0, 1.0) };

    // 3. Ground Clearance & Venturi Lift Evaluation
    const double meanRideHeight { std::max(0.0, 0.5 * (input.frontRideHeight + input.rearRideHeight)) };
    const double stallFactor { computeStall(meanRideHeight) };
    mIsStalled = meanRideHeight < mConfig.stallRideHeight;
    out.isStalled = mIsStalled;

    // Rake angle sensitivity
    const double rakeDelta { input.rearRideHeight - input.frontRideHeight };
    const double rakeModulation { 1.0 + (mConfig.rakeSensitivity * std::tanh(rakeDelta / 0.05)) };

    // Underbody Venturi Lift
    const double effectiveRideHeight { std::max(meanRideHeight, mConfig.stallRideHeight) };
    const double venturiTerm { mConfig.venturiFactor / (effectiveRideHeight + 1e-5) };
    const double underbodyCl { mConfig.baseGroundLiftCoeff * (1.0 + venturiTerm) * rakeModulation * stallFactor };

    // Wing Downforce with DRS Shedding
    const double wingCl { mConfig.baseWingLiftCoeff * (1.0 - (mConfig.maxDownforceShedding * flapRatio)) };
    out.effectiveCl = underbodyCl + wingCl;

    // 4. Drag Coefficient with DRS Reduction & Induced Drag
    const double baseCd { mConfig.baseDragCoeff * (1.0 - (mConfig.maxDragReduction * flapRatio)) };
    const double inducedCd { mConfig.inducedDragFactor * (out.effectiveCl * out.effectiveCl) };
    out.effectiveCd = baseCd + inducedCd;

    // 5. Dynamic Pressure & Force Magnitude Evaluation
    const double q { 0.5 * fluidSample.density * airspeed * airspeed };
    out.downforce = q * mConfig.referenceArea * out.effectiveCl;
    out.totalDrag = q * mConfig.referenceArea * out.effectiveCd;

    // 6. Center of Pressure & Pitch Moment
    out.centerOfPressure = mConfig.staticCpOffset + (mConfig.pitchCpMigrationRate * input.pitchAngle)
        + (mConfig.drsCpMigrationRate * flapRatio);
    out.pitchMoment = out.downforce * out.centerOfPressure;

    // 7. Coordinate Transformation into World Space
    const math::Vec2 forwardDir { std::cos(bodyHeading), std::sin(bodyHeading) };
    const math::Vec2 normalDir { -std::sin(bodyHeading), std::cos(bodyHeading) };

    const math::Vec2 dragForce { forwardDir * (-out.totalDrag) };
    const math::Vec2 downforceVector { normalDir * (-out.downforce) };

    out.totalForceWorld = dragForce + downforceVector;
    mLastOutput = out;
    return out;
}

GroundEffectAeroOutput GroundEffectAero2D::applyForces(physics::RigidBody2D& body, const GroundEffectAeroInput& input,
    const physics::FluidSample2D& fluidSample, double dt) noexcept
{
    const GroundEffectAeroOutput out { update(
        input, body.getLinearVelocity(), body.getOrientation(), fluidSample, dt) };

    body.applyForce(out.totalForceWorld);
    body.applyTorque(out.pitchMoment);

    return out;
}

} // namespace vehicle_sim::dynamics
