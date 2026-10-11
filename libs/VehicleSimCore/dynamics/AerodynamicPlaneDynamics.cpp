#include "AerodynamicPlaneDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

AerodynamicPlaneDynamics::AerodynamicPlaneDynamics() noexcept
    : AerodynamicPlaneDynamics(AeroPlaneConfig { })
{
}

AerodynamicPlaneDynamics::AerodynamicPlaneDynamics(const AeroPlaneConfig& config) noexcept
    : mConfig { config }
{
}

void AerodynamicPlaneDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Airspeed and flight frame kinematics
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const double airspeed { worldVel.length() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double omega { body.getAngularVelocity() };

    mTelemetry.airspeed = airspeed;
    mTelemetry.forwardVelocity = localVel.x;
    mTelemetry.climbRate = worldVel.y;

    // 2. Dynamic pressure q = 0.5 * rho * V^2
    const double dynamicPressure { 0.5 * mConfig.airDensity * airspeed * airspeed };
    mTelemetry.dynamicPressure = dynamicPressure;

    // 3. Angle of attack (alpha)
    double alpha { 0.0 };
    if (airspeed > 0.05) {
        alpha = -std::atan2(localVel.y, localVel.x);
    }
    mTelemetry.angleOfAttack = alpha;

    // 4. Stall detection and lift coefficient CL
    const double stallAngle { mConfig.stallAngle };
    const bool isStalledFlight { std::abs(alpha) > stallAngle };
    mTelemetry.isStalled = isStalledFlight;

    const double flaps { std::clamp(mInput.flaps, 0.0, 1.0) };
    double liftCoeff { 0.0 };

    if (!isStalledFlight) {
        // Linear attached-flow regime
        liftCoeff = (mConfig.liftSlope * alpha) + (mConfig.flapLiftCoeff * flaps);
    } else {
        // Post-stall flow separation regime with dramatic lift drop-off
        const double deltaAlpha { std::abs(alpha) - stallAngle };
        const double peakLift { (mConfig.liftSlope * stallAngle) + (mConfig.flapLiftCoeff * flaps) };
        const double decayedPeak { peakLift * std::exp(-2.5 * deltaAlpha) };
        const double flatPlateLift { 0.6 * std::sin(2.0 * alpha) };
        liftCoeff = std::copysign(decayedPeak + flatPlateLift, alpha);
    }
    mTelemetry.liftCoeff = liftCoeff;

    // 5. Drag coefficient CD (parasitic + induced + control surfaces + post-stall form drag)
    const double airbrake { std::clamp(mInput.airbrake, 0.0, 1.0) };
    const double inducedDrag { mConfig.inducedDragFactor * liftCoeff * liftCoeff };
    const double surfaceDrag { (mConfig.flapDragCoeff * flaps) + (mConfig.brakeDragCoeff * airbrake) };
    double dragCoeff { mConfig.zeroLiftDrag + inducedDrag + surfaceDrag };

    if (isStalledFlight) {
        dragCoeff += 1.2 * std::sin(alpha) * std::sin(alpha);
    }
    mTelemetry.dragCoeff = dragCoeff;

    // 6. Aerodynamic forces (Lift perpendicular to velocity, Drag opposing velocity)
    double liftMagnitude { 0.0 };
    double dragMagnitude { 0.0 };

    if (airspeed > 0.05) {
        liftMagnitude = dynamicPressure * mConfig.wingArea * liftCoeff;
        dragMagnitude = dynamicPressure * mConfig.wingArea * dragCoeff;

        const math::Vec2 velUnit { localVel.x / airspeed, localVel.y / airspeed };
        const math::Vec2 liftUnit { -velUnit.y, velUnit.x };

        const math::Vec2 localLift { liftUnit * liftMagnitude };
        const math::Vec2 localDrag { velUnit * (-dragMagnitude) };
        const math::Vec2 totalAeroLocal { localLift + localDrag };

        body.applyForce(body.toWorldVec(totalAeroLocal));
    }
    mTelemetry.liftForce = liftMagnitude;
    mTelemetry.dragForce = dragMagnitude;

    // 7. Propulsion thrust along fuselage axis
    const double throttle { std::clamp(mInput.throttle, 0.0, 1.0) };
    const double thrustForce { throttle * mConfig.maxThrust };
    mTelemetry.thrustForce = thrustForce;

    if (thrustForce > 0.0) {
        const math::Vec2 thrustLocal { thrustForce, 0.0 };
        body.applyForce(body.toWorldVec(thrustLocal));
    }

    // 8. Pitch control authority, stability restoring moment, and damping
    const double qRef { 0.5 * mConfig.airDensity * 25.0 * 25.0 }; // Reference q at 25 m/s (~90 km/h)
    const double qRatio { std::clamp(dynamicPressure / qRef, 0.0, 2.5) };

    const double elevatorCmd { std::clamp(mInput.elevator, -1.0, 1.0) };
    const double elevatorTorque { elevatorCmd * mConfig.pitchAuthority * qRatio };

    const double restoreTorque { -mConfig.weathercockStability * alpha * qRatio };
    const double pitchDampingTorque { -mConfig.pitchDamping * omega * std::max(0.1, std::sqrt(qRatio)) };

    const double netPitchTorque { elevatorTorque + restoreTorque + pitchDampingTorque };
    body.applyTorque(netPitchTorque);

    // 9. Environmental gravity acceleration
    if (body.getMass() > 0.0) {
        body.applyForce(mConfig.gravity * body.getMass());
    }
}

void AerodynamicPlaneDynamics::reset() noexcept
{
    mInput = AeroPlaneInput { };
    mTelemetry = AeroPlaneTelemetry { };
}

bool AerodynamicPlaneDynamics::isAirborne() const noexcept
{
    return true;
}

const AeroPlaneConfig& AerodynamicPlaneDynamics::getConfig() const noexcept
{
    return mConfig;
}

void AerodynamicPlaneDynamics::setConfig(const AeroPlaneConfig& config) noexcept
{
    mConfig = config;
}

const AeroPlaneInput& AerodynamicPlaneDynamics::getInput() const noexcept
{
    return mInput;
}

void AerodynamicPlaneDynamics::setInput(const AeroPlaneInput& input) noexcept
{
    mInput = input;
}

const AeroPlaneTelemetry& AerodynamicPlaneDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

double AerodynamicPlaneDynamics::getAirspeed() const noexcept
{
    return mTelemetry.airspeed;
}

double AerodynamicPlaneDynamics::getAngleOfAttack() const noexcept
{
    return mTelemetry.angleOfAttack;
}

bool AerodynamicPlaneDynamics::isStalled() const noexcept
{
    return mTelemetry.isStalled;
}

double AerodynamicPlaneDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardVelocity;
}

} // namespace vehicle_sim::dynamics
