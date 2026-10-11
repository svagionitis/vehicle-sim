#include "RocketDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

RocketDynamics::RocketDynamics() noexcept
    : RocketDynamics(RocketConfig { })
{
}

RocketDynamics::RocketDynamics(const RocketConfig& config) noexcept
    : mConfig { config }
    , mFuelMass { std::max(0.0, config.initialFuelMass) }
{
    mTelemetry.fuelMass = mFuelMass;
    mTelemetry.totalMass = mConfig.dryMass + mFuelMass;
    mTelemetry.inertia = computeInertia(mTelemetry.totalMass);
}

void RocketDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Gimbal steering servo slewing
    const double targetGimbal { std::clamp(mInput.gimbal, -1.0, 1.0) * mConfig.maxGimbalAngle };
    if (mConfig.gimbalSpeed <= 0.0) {
        mGimbalAngle = targetGimbal;
    } else {
        const double deltaGimbal { targetGimbal - mGimbalAngle };
        const double maxDelta { mConfig.gimbalSpeed * dt };
        mGimbalAngle += std::clamp(deltaGimbal, -maxDelta, maxDelta);
    }

    // 2. Propellant consumption and engine flameout
    const double clampedThrottle { std::clamp(mInput.throttle, 0.0, 1.0) };
    const double desiredThrust { clampedThrottle * mConfig.maxThrust };

    const double denomIsp { mConfig.specificImpulse * mConfig.standardGravity };
    const double mainBurnRate { denomIsp > 1e-4 ? (desiredThrust / denomIsp) : 0.0 };

    const double rcsAttitudeCmd { std::clamp(mInput.rcs, -1.0, 1.0) };
    const math::Vec2 rcsTransCmd { std::clamp(mInput.rcsTranslate.x, -1.0, 1.0),
        std::clamp(mInput.rcsTranslate.y, -1.0, 1.0) };
    const double rcsDuty { std::abs(rcsAttitudeCmd) + std::abs(rcsTransCmd.x) + std::abs(rcsTransCmd.y) };
    const double rcsBurnRate { mConfig.rcsFuelRate * std::min(3.0, rcsDuty) };

    const double totalBurnRate { mainBurnRate + rcsBurnRate };

    bool flameout { false };
    double thrustForce { 0.0 };
    const double fuelBurned { totalBurnRate * dt };

    if (mFuelMass <= 0.0) {
        mFuelMass = 0.0;
        flameout = true;
        thrustForce = 0.0;
    } else if (fuelBurned > mFuelMass) {
        const double fraction { fuelBurned > 1e-9 ? (mFuelMass / fuelBurned) : 0.0 };
        thrustForce = desiredThrust * fraction;
        mFuelMass = 0.0;
        flameout = true;
    } else {
        mFuelMass -= fuelBurned;
        thrustForce = desiredThrust;
        flameout = (mFuelMass <= 0.0);
    }

    // 3. Update rigid body inertial properties
    const double totalMass { mConfig.dryMass + mFuelMass };
    const double currentInertia { computeInertia(totalMass) };
    body.setMass(totalMass);
    body.setInertia(currentInertia);

    // 4. Main engine vectoring thrust
    if (thrustForce > 0.0) {
        const math::Vec2 localThrustDir { std::cos(mGimbalAngle), std::sin(mGimbalAngle) };
        const math::Vec2 thrustLocal { localThrustDir * thrustForce };
        const math::Vec2 thrustWorld { body.toWorldVec(thrustLocal) };
        body.applyForceAtLocalPoint(thrustWorld, mConfig.thrusterOffset);
    }

    // 5. RCS attitude and SAS rate damping
    double appliedRcsTorque { 0.0 };
    if (!flameout) {
        appliedRcsTorque = rcsAttitudeCmd * mConfig.rcsTorque;
    }

    double sasTorque { 0.0 };
    if (mInput.sasEnabled && std::abs(rcsAttitudeCmd) < 1e-4) {
        sasTorque = -mConfig.sasDamping * body.getAngularVelocity();
        sasTorque = std::clamp(sasTorque, -mConfig.rcsTorque, mConfig.rcsTorque);
    }

    const double netTorque { appliedRcsTorque + sasTorque };
    if (std::abs(netTorque) > 1e-6) {
        body.applyTorque(netTorque);
    }

    // 6. Translational RCS thrusters
    math::Vec2 rcsForceWorld { math::Vec2::zero() };
    if (!flameout && (std::abs(rcsTransCmd.x) > 1e-4 || std::abs(rcsTransCmd.y) > 1e-4)) {
        const math::Vec2 rcsForceLocal { rcsTransCmd * mConfig.rcsThrust };
        rcsForceWorld = body.toWorldVec(rcsForceLocal);
        body.applyForce(rcsForceWorld);
    }

    // 7. Atmospheric aerodynamic drag
    math::Vec2 aeroDragWorld { math::Vec2::zero() };
    if (mConfig.atmosphericDensity > 0.0 && mConfig.dragCoefficient > 0.0) {
        const math::Vec2 vel { body.getLinearVelocity() };
        const double speed { vel.length() };
        if (speed > 1e-4) {
            const double q { 0.5 * mConfig.atmosphericDensity * speed * speed };
            const double dragMag { q * mConfig.dragCoefficient * mConfig.crossSectionArea };
            aeroDragWorld = -vel.normalized() * dragMag;
            body.applyForce(aeroDragWorld);
        }
    }

    // 8. Environmental gravity
    if (mConfig.gravity.length() > 1e-6) {
        const math::Vec2 gravForce { body.getMass() * mConfig.gravity };
        body.applyForce(gravForce);
    }

    // 9. Update diagnostic telemetry
    mTelemetry.fuelMass = mFuelMass;
    mTelemetry.totalMass = totalMass;
    mTelemetry.inertia = currentInertia;
    mTelemetry.currentThrust = thrustForce;
    mTelemetry.gimbalAngle = mGimbalAngle;
    mTelemetry.massFlowRate = flameout ? 0.0 : totalBurnRate;
    mTelemetry.rcsTorque = netTorque;
    mTelemetry.rcsForce = rcsForceWorld;
    mTelemetry.aeroDrag = aeroDragWorld;
    mTelemetry.isFlameout = flameout;
}

void RocketDynamics::reset() noexcept
{
    mInput = RocketInput { };
    mGimbalAngle = 0.0;
    mTelemetry = RocketTelemetry { };
    mTelemetry.fuelMass = mFuelMass;
    mTelemetry.totalMass = mConfig.dryMass + mFuelMass;
    mTelemetry.inertia = computeInertia(mTelemetry.totalMass);
}

bool RocketDynamics::isAirborne() const noexcept
{
    return true;
}

const RocketConfig& RocketDynamics::getConfig() const noexcept
{
    return mConfig;
}

void RocketDynamics::setConfig(const RocketConfig& config) noexcept
{
    mConfig = config;
    mFuelMass = std::clamp(mFuelMass, 0.0, std::max(0.0, config.initialFuelMass));
}

const RocketInput& RocketDynamics::getInput() const noexcept
{
    return mInput;
}

void RocketDynamics::setInput(const RocketInput& input) noexcept
{
    mInput = input;
}

const RocketTelemetry& RocketDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

double RocketDynamics::getFuelMass() const noexcept
{
    return mFuelMass;
}

void RocketDynamics::setFuelMass(double massKg) noexcept
{
    mFuelMass = std::max(0.0, massKg);
}

double RocketDynamics::getDryMass() const noexcept
{
    return mConfig.dryMass;
}

double RocketDynamics::getTotalMass() const noexcept
{
    return mConfig.dryMass + mFuelMass;
}

double RocketDynamics::getInertia() const noexcept
{
    return computeInertia(getTotalMass());
}

double RocketDynamics::getFuelRatio() const noexcept
{
    if (mConfig.initialFuelMass <= 1e-9) {
        return 0.0;
    }
    return std::clamp(mFuelMass / mConfig.initialFuelMass, 0.0, 1.0);
}

bool RocketDynamics::isFuelEmpty() const noexcept
{
    return mFuelMass <= 0.0;
}

double RocketDynamics::getGimbalAngle() const noexcept
{
    return mGimbalAngle;
}

void RocketDynamics::setGimbalAngle(double angleRad) noexcept
{
    mGimbalAngle = std::clamp(angleRad, -mConfig.maxGimbalAngle, mConfig.maxGimbalAngle);
}

double RocketDynamics::getThrustForce() const noexcept
{
    return mTelemetry.currentThrust;
}

bool RocketDynamics::isSasEnabled() const noexcept
{
    return mInput.sasEnabled;
}

void RocketDynamics::setSasEnabled(bool enabled) noexcept
{
    mInput.sasEnabled = enabled;
}

void RocketDynamics::refuel() noexcept
{
    mFuelMass = std::max(0.0, mConfig.initialFuelMass);
}

double RocketDynamics::computeInertia(double currentMass) const noexcept
{
    const double lengthSq { mConfig.length * mConfig.length };
    const double radiusSq { mConfig.radius * mConfig.radius };
    const double inertiaFactor { (1.0 / 12.0) * lengthSq + (1.0 / 4.0) * radiusSq };
    return std::max(1e-3, currentMass * inertiaFactor);
}

} // namespace vehicle_sim::dynamics
