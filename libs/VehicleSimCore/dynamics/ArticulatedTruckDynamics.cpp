#include "ArticulatedTruckDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

namespace {
    inline double normalizeAngle(double angle) noexcept
    {
        return math::Angle2D::normalizeRad(angle);
    }
} // namespace

ArticulatedTruckDynamics::ArticulatedTruckDynamics() noexcept
    : ArticulatedTruckDynamics(ArticulatedTruckConfig {})
{
}

ArticulatedTruckDynamics::ArticulatedTruckDynamics(const ArticulatedTruckConfig& config) noexcept
    : mConfig { config }
{
    mTelemetry.totalMass = mConfig.tractorMass + mConfig.trailerMass;
}

void ArticulatedTruckDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Steering servo actuation
    const double targetSteer { std::clamp(mInput.steer, -1.0, 1.0) * mConfig.maxSteerAngle };
    if (mConfig.steerSlewRate <= 0.0) {
        mSteerAngle = targetSteer;
    } else {
        const double deltaSteer { targetSteer - mSteerAngle };
        const double maxSteerStep { mConfig.steerSlewRate * dt };
        mSteerAngle += std::clamp(deltaSteer, -maxSteerStep, maxSteerStep);
    }

    // 2. Tractor kinematic state
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double speed { worldVel.length() };
    const double forwardSpeed { localVel.x };
    const double lateralSpeed { localVel.y };
    const double omega { body.getAngularVelocity() };
    const double tractorHeading { body.getOrientation() };

    // 3. Fifth-wheel kingpin velocity
    const math::Vec2 kingpinLocalPos { mConfig.kingpinOffset, 0.0 };
    const math::Vec2 kingpinLocalVel { localVel.x, localVel.y + (omega * mConfig.kingpinOffset) };
    const math::Vec2 kingpinWorldVel { body.toWorldVec(kingpinLocalVel) };

    // 4. Trailer orientation and articulation hitch angle integration
    const math::Vec2 trailerNormal { -std::sin(mTrailerHeading), std::cos(mTrailerHeading) };
    const double kingpinTransverseVel { kingpinWorldVel.dot(trailerNormal) };
    const double trailerYawRate { (mConfig.trailerLength > 0.0) ? (kingpinTransverseVel / mConfig.trailerLength)
                                                                : 0.0 };

    mTrailerHeading = normalizeAngle(mTrailerHeading + (trailerYawRate * dt));
    mHitchAngle = normalizeAngle(mTrailerHeading - tractorHeading);

    // Limit physical mechanical hitch articulation
    if (std::abs(mHitchAngle) > mConfig.maxHitchAngle) {
        mHitchAngle = std::clamp(mHitchAngle, -mConfig.maxHitchAngle, mConfig.maxHitchAngle);
        mTrailerHeading = normalizeAngle(tractorHeading + mHitchAngle);
    }

    const double absHitch { std::abs(mHitchAngle) };
    const bool jackknifed { absHitch >= mConfig.jackknifeThresholdAngle };
    const double jackknifeSeverity { std::clamp(absHitch / mConfig.jackknifeThresholdAngle, 0.0, 1.0) };

    // 5. Driver propulsion and braking distribution
    const double throttle { std::clamp(mInput.throttle, -1.0, 1.0) };
    const double serviceBrake { std::clamp(mInput.serviceBrake, 0.0, 1.0) };
    const double trolleyBrake { std::clamp(mInput.trailerBrake, 0.0, 1.0) };

    const double driveForce { throttle * mConfig.maxDriveForce };
    const double tractorBrakeForce { serviceBrake * mConfig.maxTractorBrakeForce };
    const double trailerBrakeForce { std::min(
        mConfig.maxTrailerBrakeForce, (serviceBrake * 0.5 + trolleyBrake) * mConfig.maxTrailerBrakeForce) };

    const double speedSign { (forwardSpeed > 0.1) ? 1.0 : ((forwardSpeed < -0.1) ? -1.0 : 0.0) };
    const double rollingResistance { mConfig.rollingResistanceCoeff * speed };
    const double totalBraking { (tractorBrakeForce + (trailerBrakeForce * std::cos(mHitchAngle))) * speedSign };
    const double netLongitudinalForce { driveForce - totalBraking - (rollingResistance * speedSign) };

    body.applyForce(body.toWorldVec(math::Vec2 { netLongitudinalForce, 0.0 }));

    // 6. Tractor tire cornering forces and steering moment
    const double halfWheelbase { mConfig.tractorWheelbase * 0.5 };
    const double refSpeed { std::max(1.0, std::abs(forwardSpeed)) };
    const double frontSlip { ((lateralSpeed + (omega * halfWheelbase)) / refSpeed) - mSteerAngle };
    const double rearSlip { (lateralSpeed - (omega * halfWheelbase)) / refSpeed };

    const double frontCorneringForce { std::clamp(-frontSlip * mConfig.tractorCorneringStiffness, -35000.0, 35000.0) };
    const double rearCorneringForce { std::clamp(-rearSlip * mConfig.tractorCorneringStiffness, -40000.0, 40000.0) };

    const double steeringYawTorque { (frontCorneringForce * halfWheelbase) - (rearCorneringForce * halfWheelbase) };
    body.applyTorque(steeringYawTorque);

    // 7. Fifth-wheel jackknifing / tension restoring lateral forces
    // Compressive force from tractor braking and reverse drive pushes kingpin; trailer hand-brake creates tension
    // pulling kingpin straight
    const double reverseDrivePush { (forwardSpeed < -0.1)
            ? (std::abs(driveForce) * (mConfig.trailerMass / (mConfig.tractorMass + mConfig.trailerMass)))
            : 0.0 };
    const double compressivePush { (tractorBrakeForce + reverseDrivePush)
        * (mConfig.trailerMass / (mConfig.tractorMass + mConfig.trailerMass)) };
    const double kingpinPushLat { compressivePush * std::sin(mHitchAngle) };
    const double kingpinTensionRestoring { -trailerBrakeForce * std::sin(mHitchAngle) };
    const double trailerLateralTireGrip { -mConfig.trailerCorneringStiffness * std::sin(mHitchAngle) };

    const double totalKingpinLateralForce { std::clamp(
        kingpinPushLat + kingpinTensionRestoring + trailerLateralTireGrip, -60000.0, 60000.0) };
    body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { 0.0, totalKingpinLateralForce }), kingpinLocalPos);

    // 8. Fifth-wheel plate rotational friction
    const double hitchFrictionTorque { -mConfig.hitchFrictionDamping * (trailerYawRate - omega) };
    body.applyTorque(hitchFrictionTorque);

    // 9. Aerodynamic drag
    if (speed > 1e-3) {
        const double aeroDrag { mConfig.aerodynamicDragCoeff * speed * speed };
        body.applyForce(-worldVel.normalized() * aeroDrag);
    }

    // 10. Telemetry snapshot update
    mTelemetry.speed = speed;
    mTelemetry.forwardSpeed = forwardSpeed;
    mTelemetry.steerAngle = mSteerAngle;
    mTelemetry.hitchAngle = mHitchAngle;
    mTelemetry.trailerHeading = mTrailerHeading;
    mTelemetry.jackknifeSeverity = jackknifeSeverity;
    mTelemetry.tractorBrakeForce = tractorBrakeForce;
    mTelemetry.trailerBrakeForce = trailerBrakeForce;
    mTelemetry.totalMass = mConfig.tractorMass + mConfig.trailerMass;
    mTelemetry.offTrackingDistance = std::abs(mConfig.trailerLength * std::sin(mHitchAngle));
    mTelemetry.isJackknifed = jackknifed;
}

void ArticulatedTruckDynamics::reset() noexcept
{
    mInput = ArticulatedTruckInput {};
    mSteerAngle = 0.0;
    mHitchAngle = 0.0;
    mTrailerHeading = 0.0;
    mTelemetry = ArticulatedTruckTelemetry {};
    mTelemetry.totalMass = mConfig.tractorMass + mConfig.trailerMass;
}

bool ArticulatedTruckDynamics::isAirborne() const noexcept
{
    return false;
}

const ArticulatedTruckConfig& ArticulatedTruckDynamics::getConfig() const noexcept
{
    return mConfig;
}

void ArticulatedTruckDynamics::setConfig(const ArticulatedTruckConfig& config) noexcept
{
    mConfig = config;
}

const ArticulatedTruckInput& ArticulatedTruckDynamics::getInput() const noexcept
{
    return mInput;
}

void ArticulatedTruckDynamics::setInput(const ArticulatedTruckInput& input) noexcept
{
    mInput = input;
}

const ArticulatedTruckTelemetry& ArticulatedTruckDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

double ArticulatedTruckDynamics::getHitchAngle() const noexcept
{
    return mTelemetry.hitchAngle;
}

void ArticulatedTruckDynamics::setHitchAngle(double angle) noexcept
{
    mHitchAngle = std::clamp(angle, -mConfig.maxHitchAngle, mConfig.maxHitchAngle);
    mTrailerHeading = normalizeAngle(mHitchAngle);
    mTelemetry.hitchAngle = mHitchAngle;
    mTelemetry.trailerHeading = mTrailerHeading;
}

double ArticulatedTruckDynamics::getTrailerHeading() const noexcept
{
    return mTelemetry.trailerHeading;
}

bool ArticulatedTruckDynamics::isJackknifed() const noexcept
{
    return mTelemetry.isJackknifed;
}

double ArticulatedTruckDynamics::getSteerAngle() const noexcept
{
    return mTelemetry.steerAngle;
}

double ArticulatedTruckDynamics::getSpeed() const noexcept
{
    return mTelemetry.speed;
}

} // namespace vehicle_sim::dynamics
