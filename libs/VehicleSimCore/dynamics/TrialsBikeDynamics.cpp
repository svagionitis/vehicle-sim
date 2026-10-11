#include "TrialsBikeDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

TrialsBikeDynamics::TrialsBikeDynamics() noexcept
    : TrialsBikeDynamics(TrialsBikeConfig { })
{
}

TrialsBikeDynamics::TrialsBikeDynamics(const TrialsBikeConfig& config) noexcept
    : mConfig { config }
{
}

void TrialsBikeDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Compute chassis kinematics
    const double totalMass { mConfig.mass + mConfig.riderMass };
    body.setMass(totalMass);

    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double forwardSpeed { localVel.x };
    const double omega { body.getAngularVelocity() };
    mTelemetry.forwardSpeed = forwardSpeed;

    // 2. Rider active center-of-mass lean moment
    const double leanCmd { std::clamp(mInput.riderLean, -1.0, 1.0) };
    const double cgShiftX { leanCmd * mConfig.riderMaxLeanX };
    mTelemetry.riderCgShift = cgShiftX;

    const double leanTorque { -mConfig.riderMass * 9.80665 * cgShiftX };
    body.applyTorque(leanTorque);

    // 3. Front and rear wheel suspension ground normal forces
    const double frontComp { std::max(0.0, mConfig.wheelRadius - mInput.groundDistanceFront) };
    const bool frontContact { frontComp > 1e-4 };

    const double rearComp { std::max(0.0, mConfig.wheelRadius - mInput.groundDistanceRear) };
    const bool rearContact { rearComp > 1e-4 };

    const double frontVy { localVel.y + (omega * mConfig.frontWheelOffset.x) };
    const double rearVy { localVel.y + (omega * mConfig.rearWheelOffset.x) };

    double frontNormal { 0.0 };
    if (frontContact) {
        frontNormal = std::max(0.0, (mConfig.suspensionStiffness * frontComp) - (mConfig.suspensionDamping * frontVy));
    }

    double rearNormal { 0.0 };
    if (rearContact) {
        rearNormal = std::max(0.0, (mConfig.suspensionStiffness * rearComp) - (mConfig.suspensionDamping * rearVy));
    }

    mTelemetry.frontNormalForce = frontNormal;
    mTelemetry.rearNormalForce = rearNormal;
    mTelemetry.frontContact = frontContact;
    mTelemetry.rearContact = rearContact;

    if (frontNormal > 0.0) {
        body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { 0.0, frontNormal }), mConfig.frontWheelOffset);
    }
    if (rearNormal > 0.0) {
        body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { 0.0, rearNormal }), mConfig.rearWheelOffset);
    }

    // 4. Stunt balance states (wheelies, stoppies, and flight)
    const bool isAirborneFlight { !frontContact && !rearContact };
    mTelemetry.isAirborne = isAirborneFlight;
    mTelemetry.isWheelie = (rearContact && !frontContact);
    mTelemetry.isStoppie = (frontContact && !rearContact);

    // 5. Drive and braking tractive forces on ground contact
    const double rWheel { std::max(0.05, mConfig.wheelRadius) };

    if (rearContact) {
        const double throttle { std::clamp(mInput.throttle, 0.0, 1.0) };
        const double rearBrake { std::clamp(mInput.rearBrake, 0.0, 1.0) };
        const double rawDriveForce { (throttle * mConfig.maxDriveTorque) / rWheel };
        const double rawBrakeForce { (rearBrake * mConfig.rearBrakeTorque) / rWheel };

        const double netLongRear { rawDriveForce - (forwardSpeed > 0.0 ? rawBrakeForce : -rawBrakeForce * 0.5) };
        const double maxRearTraction { rearNormal * mConfig.surfaceFriction };
        const double appliedRearForce { std::clamp(netLongRear, -maxRearTraction, maxRearTraction) };

        body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { appliedRearForce, 0.0 }), mConfig.rearWheelOffset);
    }

    if (frontContact) {
        const double frontBrake { std::clamp(mInput.frontBrake, 0.0, 1.0) };
        const double rawFrontBrake { (frontBrake * mConfig.frontBrakeTorque) / rWheel };
        const double frontBrakeForce { forwardSpeed > 0.0 ? -rawFrontBrake : 0.0 };

        const double maxFrontTraction { frontNormal * mConfig.surfaceFriction };
        const double appliedFrontForce { std::clamp(frontBrakeForce, -maxFrontTraction, maxFrontTraction) };

        body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { appliedFrontForce, 0.0 }), mConfig.frontWheelOffset);
    }

    // 6. In-air wheel angular momentum reaction (gyroscopic pitch control)
    if (isAirborneFlight) {
        const double throttle { std::clamp(mInput.throttle, 0.0, 1.0) };
        const double rearBrake { std::clamp(mInput.rearBrake, 0.0, 1.0) };
        const double airPitchReaction { (throttle * mConfig.airPitchTorque) - (rearBrake * mConfig.airPitchTorque) };
        body.applyTorque(airPitchReaction);
    }

    // 7. Ambient gravity acceleration
    body.applyForce(mConfig.gravity * totalMass);
}

void TrialsBikeDynamics::reset() noexcept
{
    mInput = TrialsBikeInput { };
    mTelemetry = TrialsBikeTelemetry { };
}

bool TrialsBikeDynamics::isAirborne() const noexcept
{
    return mTelemetry.isAirborne;
}

const TrialsBikeConfig& TrialsBikeDynamics::getConfig() const noexcept
{
    return mConfig;
}

void TrialsBikeDynamics::setConfig(const TrialsBikeConfig& config) noexcept
{
    mConfig = config;
}

const TrialsBikeInput& TrialsBikeDynamics::getInput() const noexcept
{
    return mInput;
}

void TrialsBikeDynamics::setInput(const TrialsBikeInput& input) noexcept
{
    mInput = input;
}

const TrialsBikeTelemetry& TrialsBikeDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool TrialsBikeDynamics::isWheelie() const noexcept
{
    return mTelemetry.isWheelie;
}

bool TrialsBikeDynamics::isStoppie() const noexcept
{
    return mTelemetry.isStoppie;
}

double TrialsBikeDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardSpeed;
}

} // namespace vehicle_sim::dynamics
