#include "TrackedTankDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

TrackedTankDynamics::TrackedTankDynamics() noexcept
    : TrackedTankDynamics(TrackedTankConfig { })
{
}

TrackedTankDynamics::TrackedTankDynamics(const TrackedTankConfig& config) noexcept
    : mConfig { config }
{
}

void TrackedTankDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Compute local velocities and yaw rate
    const math::Vec2 localVel { body.toLocalVec(body.getLinearVelocity()) };
    const double omega { body.getAngularVelocity() };
    const double halfTrackWidth { mConfig.trackWidth * 0.5 };

    mTelemetry.forwardSpeed = localVel.x;
    mTelemetry.yawRate = omega;

    // 2. Kinematic track velocities
    const double leftKinSpeed { localVel.x - (halfTrackWidth * omega) };
    const double rightKinSpeed { localVel.x + (halfTrackWidth * omega) };
    mTelemetry.leftTrackSpeed = leftKinSpeed;
    mTelemetry.rightTrackSpeed = rightKinSpeed;

    // 3. Commanded target track speeds
    const double leftCmdInput { std::clamp(mInput.leftTrack, -1.0, 1.0) };
    const double rightCmdInput { std::clamp(mInput.rightTrack, -1.0, 1.0) };
    const double leftCmdSpeed { leftCmdInput * mConfig.maxTrackSpeed };
    const double rightCmdSpeed { rightCmdInput * mConfig.maxTrackSpeed };

    // 4. Track slip differences and tractive forces
    const double leftDelta { leftCmdSpeed - leftKinSpeed };
    const double rightDelta { rightCmdSpeed - rightKinSpeed };

    const double surfFriction { std::max(0.0, mConfig.surfaceFriction) };
    const double maxForce { mConfig.maxTrackForce * surfFriction };

    const double leftForce { std::clamp(leftDelta * mConfig.driveStiffness, -maxForce, maxForce) };
    const double rightForce { std::clamp(rightDelta * mConfig.driveStiffness, -maxForce, maxForce) };

    mTelemetry.leftTrackForce = leftForce;
    mTelemetry.rightTrackForce = rightForce;

    // Apply track tractive forces at track contact centers in local chassis coordinates
    const math::Vec2 leftForceLocal { leftForce, 0.0 };
    const math::Vec2 rightForceLocal { rightForce, 0.0 };
    const math::Vec2 leftOffset { 0.0, halfTrackWidth };
    const math::Vec2 rightOffset { 0.0, -halfTrackWidth };

    body.applyForceAtLocalPoint(body.toWorldVec(leftForceLocal), leftOffset);
    body.applyForceAtLocalPoint(body.toWorldVec(rightForceLocal), rightOffset);

    // 5. Skid scrubbing resistive torque and damping
    const double skidFrictionTorque { -std::tanh(omega * 10.0) * mConfig.skidTorque * surfFriction };
    const double rotDampingTorque { -mConfig.angularDamping * omega };
    const double netSkidTorque { skidFrictionTorque + rotDampingTorque };
    mTelemetry.skidTorque = netSkidTorque;
    body.applyTorque(netSkidTorque);

    // 6. Lateral tread shear friction
    const double maxLatForce { mConfig.maxLateralForce * surfFriction };
    const double latForce { std::clamp(-localVel.y * mConfig.lateralGrip, -maxLatForce, maxLatForce) };
    mTelemetry.lateralForce = latForce;
    body.applyForce(body.toWorldVec(math::Vec2 { 0.0, latForce }));

    // 7. Longitudinal motion drag
    const double rollDrag { -mConfig.linearDrag * localVel.x * std::abs(localVel.x) };
    body.applyForce(body.toWorldVec(math::Vec2 { rollDrag, 0.0 }));

    // 8. Turret cannon recoil impulse handling
    if (mInput.fireRecoil) {
        mRecoilTimer = mConfig.recoilDuration;
        mRecoilAngle = mInput.turretAngle;
        mInput.fireRecoil = false;
    }

    if (mRecoilTimer > 0.0) {
        mTelemetry.isRecoiling = true;
        mTelemetry.recoilTimer = mRecoilTimer;

        // Recoil force pushes strictly opposite to turret barrel pointing direction
        const math::Vec2 barrelDir { std::cos(mRecoilAngle), std::sin(mRecoilAngle) };
        const math::Vec2 recoilLocal { barrelDir * (-mConfig.recoilForce) };
        const math::Vec2 recoilWorld { body.toWorldVec(recoilLocal) };

        body.applyForceAtLocalPoint(recoilWorld, mConfig.turretOffset);
        mTelemetry.recoilForce = recoilWorld;

        mRecoilTimer = std::max(0.0, mRecoilTimer - dt);
    } else {
        mTelemetry.isRecoiling = false;
        mTelemetry.recoilTimer = 0.0;
        mTelemetry.recoilForce = math::Vec2::zero();
    }
}

void TrackedTankDynamics::reset() noexcept
{
    mInput = TrackedTankInput { };
    mTelemetry = TrackedTankTelemetry { };
    mRecoilTimer = 0.0;
    mRecoilAngle = 0.0;
}

bool TrackedTankDynamics::isAirborne() const noexcept
{
    return false;
}

const TrackedTankConfig& TrackedTankDynamics::getConfig() const noexcept
{
    return mConfig;
}

void TrackedTankDynamics::setConfig(const TrackedTankConfig& config) noexcept
{
    mConfig = config;
}

const TrackedTankInput& TrackedTankDynamics::getInput() const noexcept
{
    return mInput;
}

void TrackedTankDynamics::setInput(const TrackedTankInput& input) noexcept
{
    mInput = input;
}

void TrackedTankDynamics::fireRecoil() noexcept
{
    mRecoilTimer = mConfig.recoilDuration;
    mRecoilAngle = mInput.turretAngle;
}

void TrackedTankDynamics::setSurfaceFriction(double friction) noexcept
{
    mConfig.surfaceFriction = std::max(0.0, friction);
}

const TrackedTankTelemetry& TrackedTankDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

double TrackedTankDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardSpeed;
}

double TrackedTankDynamics::getYawRate() const noexcept
{
    return mTelemetry.yawRate;
}

} // namespace vehicle_sim::dynamics
