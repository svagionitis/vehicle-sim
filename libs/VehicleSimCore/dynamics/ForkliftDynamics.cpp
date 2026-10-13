#include "libs/VehicleSimCore/dynamics/ForkliftDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

ForkliftDynamics::ForkliftDynamics() noexcept
    : ForkliftDynamics(ForkliftConfig {})
{
}

ForkliftDynamics::ForkliftDynamics(const ForkliftConfig& config) noexcept
    : mConfig { config }
{
    updateCenterOfGravity();
}

void ForkliftDynamics::updateCenterOfGravity() noexcept
{
    const double totalMass { mConfig.chassisMass + mPayloadMass };
    if (totalMass <= 0.0) {
        return;
    }

    // Distance behind front axle (positive value = behind front axle)
    const double xChassis { -mConfig.chassisCogDistFront };
    const double xPayload { mConfig.forkDistFront };
    const double xComposite { (mConfig.chassisMass * xChassis + mPayloadMass * xPayload) / totalMass };
    mCompositeCogDistFront = -xComposite;

    // Vertical height above ground
    const double hPayload { mConfig.baseForkHeight + mMastHeight };
    mCompositeCogHeight = (mConfig.chassisMass * mConfig.chassisCogHeight + mPayloadMass * hPayload) / totalMass;

    evaluateStability(0.0, 0.0);
}

void ForkliftDynamics::evaluateStability(double ax, double ay) noexcept
{
    constexpr double gravityConst { 9.81 };
    const double totalMass { mConfig.chassisMass + mPayloadMass };
    const double L { mConfig.wheelbase };

    // Normal force on rear steering pivot (<= 0 implies forward tipping)
    const double fzRear { (totalMass * gravityConst * mCompositeCogDistFront - totalMass * ax * mCompositeCogHeight)
        / L };
    mRearPivotLoad = fzRear;

    // Effective half-track width of 3-point stability triangle at composite COG
    const double relCogDist { std::clamp(mCompositeCogDistFront / L, 0.0, 1.0) };
    const double effectiveHalfWidth { (mConfig.frontTrackWidth * 0.5) * std::max(0.05, 1.0 - relCogDist) };

    // Dynamic lateral rollover index
    mRolloverIndex = (std::abs(ay) * mCompositeCogHeight) / (gravityConst * effectiveHalfWidth);

    if (mRearPivotLoad <= 0.0 || mRolloverIndex >= 1.0) {
        mIsTippedOver = true;
    }
}

void ForkliftDynamics::reset() noexcept
{
    mThrottle = 0.0;
    mBrake = 0.0;
    mSteer = 0.0;
    mMastHeight = 0.0;
    mPayloadMass = 0.0;
    mRolloverIndex = 0.0;
    mRearPivotLoad = 0.0;
    mPrevLongVel = 0.0;
    mReverse = false;
    mIsTippedOver = false;
    mIsAirborne = false;
    updateCenterOfGravity();
}

bool ForkliftDynamics::isAirborne() const noexcept
{
    return mIsAirborne || mIsTippedOver;
}

void ForkliftDynamics::setThrottle(double throttle) noexcept
{
    mThrottle = std::clamp(throttle, 0.0, 1.0);
}

void ForkliftDynamics::setBrake(double brake) noexcept
{
    mBrake = std::clamp(brake, 0.0, 1.0);
}

void ForkliftDynamics::setSteer(double steer) noexcept
{
    mSteer = std::clamp(steer, -1.0, 1.0);
}

void ForkliftDynamics::setMastHeight(double height) noexcept
{
    mMastHeight = std::clamp(height, 0.0, mConfig.maxMastHeight);
    updateCenterOfGravity();
}

void ForkliftDynamics::setPayloadMass(double mass) noexcept
{
    mPayloadMass = std::max(0.0, mass);
    updateCenterOfGravity();
}

void ForkliftDynamics::setReverse(bool reverse) noexcept
{
    mReverse = reverse;
}

double ForkliftDynamics::getRolloverIndex() const noexcept
{
    return mRolloverIndex;
}

double ForkliftDynamics::getRearPivotLoad() const noexcept
{
    return mRearPivotLoad;
}

double ForkliftDynamics::getCompositeCogHeight() const noexcept
{
    return mCompositeCogHeight;
}

bool ForkliftDynamics::isTippedOver() const noexcept
{
    return mIsTippedOver;
}

const ForkliftConfig& ForkliftDynamics::getConfig() const noexcept
{
    return mConfig;
}

void ForkliftDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const math::Vec2 bodyVelWorld { body.getLinearVelocity() };
    const math::Vec2 bodyVelLocal { body.toLocalVec(bodyVelWorld) };
    const double forwardSpeed { bodyVelLocal.x };
    const double yawRate { body.getAngularVelocity() };

    const double ax { std::clamp((forwardSpeed - mPrevLongVel) / dt, -15.0, 15.0) };
    const double ay { std::clamp(forwardSpeed * yawRate, -20.0, 20.0) };
    mPrevLongVel = forwardSpeed;

    evaluateStability(ax, ay);

    if (mIsTippedOver) {
        // Degrade traction when tipped over
        const math::Vec2 dragForceWorld { body.toWorldVec(
            math::Vec2 { -forwardSpeed * 2000.0, -bodyVelLocal.y * 2000.0 }) };
        body.applyForce(dragForceWorld);
        body.applyTorque(-yawRate * 1500.0);
        return;
    }

    // Front drive traction with speed governor
    const double gearSign { mReverse ? -1.0 : 1.0 };
    const double speedRatio { std::abs(forwardSpeed) / mConfig.maxSpeed };
    const double governorFactor { std::clamp(1.0 - speedRatio, 0.0, 1.0) };
    const double driveForce { mThrottle * mConfig.maxDriveForce * governorFactor * gearSign };

    // Service brake force opposing motion
    double brakeForce { 0.0 };
    if (mBrake > 0.0 && std::abs(forwardSpeed) > 1e-2) {
        const double brakeSign { (forwardSpeed > 0.0) ? -1.0 : 1.0 };
        brakeForce = brakeSign * mBrake * mConfig.maxBrakeForce;
    }

    // Front axle forces applied at (0, 0) in front-axle relative coordinates
    const double frontLatVel { bodyVelLocal.y };
    const double frontLatForce { -mConfig.lateralGripCoeff * frontLatVel };
    const math::Vec2 frontForceLocal { driveForce + brakeForce, frontLatForce };
    body.applyForceAtLocalPoint(body.toWorldVec(frontForceLocal), math::Vec2 { 0.0, 0.0 });

    // Rear steer pivot forces applied at (-L, 0)
    // To turn left (positive yaw), rear wheels steer right (negative angle)
    const double rearSteerAngle { -mSteer * mConfig.maxSteerAngle };
    const math::Vec2 rearPivotOffset { -mConfig.wheelbase, 0.0 };
    const math::Vec2 rearVelWorld { body.getVelocityAtLocalPoint(rearPivotOffset) };
    const math::Vec2 rearVelLocal { body.toLocalVec(rearVelWorld) };

    const double cosS { std::cos(rearSteerAngle) };
    const double sinS { std::sin(rearSteerAngle) };
    const math::Vec2 steerHeadingLocal { cosS, sinS };
    const math::Vec2 steerLateralLocal { -sinS, cosS };

    // Lateral scrub velocity perpendicular to rear steer angle
    const double rearLatVel { rearVelLocal.dot(steerLateralLocal) };
    const double rearLatForce { -mConfig.lateralGripCoeff * rearLatVel };
    const math::Vec2 rearForceLocal { steerLateralLocal * rearLatForce };

    body.applyForceAtLocalPoint(body.toWorldVec(rearForceLocal), rearPivotOffset);
}

} // namespace vehicle_sim::dynamics
