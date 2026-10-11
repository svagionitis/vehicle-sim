#include "SubmarineDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

SubmarineDynamics::SubmarineDynamics() noexcept
    : SubmarineDynamics(SubmarineConfig { })
{
}

SubmarineDynamics::SubmarineDynamics(const SubmarineConfig& config) noexcept
    : mConfig { config }
    , mBallastMass { config.initialBallastMass }
{
    mTelemetry.ballastMass = mBallastMass;
    mTelemetry.totalMass = mConfig.dryMass + mBallastMass;
    mTelemetry.ballastRatio = (mConfig.maxBallastMass > 0.0)
        ? (mBallastMass / mConfig.maxBallastMass) : 0.0;
}

void SubmarineDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Kinematics retrieval
    const math::Vec2 worldPos { body.getPosition() };
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double forwardSpeed { localVel.x };
    const double omega { body.getAngularVelocity() };
    const double orientation { body.getOrientation() };

    // 2. Ballast tanks flood and blow mass management
    const double flood { std::clamp(mInput.floodRate, 0.0, 1.0) };
    const double blow { std::clamp(mInput.blowRate, 0.0, 1.0) };
    const double deltaBallast { (flood - blow) * mConfig.pumpRate * dt };

    mBallastMass = std::clamp(mBallastMass + deltaBallast, 0.0, mConfig.maxBallastMass);
    const double totalMass { mConfig.dryMass + mBallastMass };
    body.setMass(totalMass);

    // 3. Submerged depth and submergence ratio
    const double submergedDepth { std::max(0.0, mConfig.waterlineY - worldPos.y) };
    const bool inWater { worldPos.y <= mConfig.waterlineY };
    const double hullHeightRef { 4.0 };
    const double submergenceRatio { std::clamp(submergedDepth / hullHeightRef, 0.0, 1.0) };
    const bool crushed { submergedDepth >= mConfig.crushDepth };

    // 4. Gravity and Archimedes hydrostatic buoyancy
    const double displacementVolume { mConfig.hullVolume * submergenceRatio };
    const double buoyantForce { displacementVolume * mConfig.waterDensity * 9.80665 };
    const double gravityMagnitude { totalMass * 9.80665 };
    const double netVerticalBuoyancy { buoyantForce - gravityMagnitude };

    body.applyForce(mConfig.gravity * totalMass);
    if (inWater) {
        body.applyForce(math::Vec2 { 0.0, buoyantForce });
    }

    // 5. Hydrodynamic drag (streamlined longitudinal vs bluff transverse/vertical)
    if (inWater) {
        const double dragX { -mConfig.longitudinalDragCoeff * std::abs(localVel.x) * localVel.x * submergenceRatio };
        const double dragY { -mConfig.lateralDragCoeff * std::abs(localVel.y) * localVel.y * submergenceRatio };
        body.applyForce(body.toWorldVec(math::Vec2 { dragX, dragY }));
    }

    // 6. Hydrofoil dive plane actuation and dynamic lift moments
    const double targetDivePlane { std::clamp(mInput.divePlane, -1.0, 1.0) * mConfig.maxDivePlaneAngle };
    if (mConfig.divePlaneSlewRate <= 0.0) {
        mDivePlaneAngle = targetDivePlane;
    } else {
        const double deltaPlane { targetDivePlane - mDivePlaneAngle };
        const double maxPlaneStep { mConfig.divePlaneSlewRate * dt };
        mDivePlaneAngle += std::clamp(deltaPlane, -maxPlaneStep, maxPlaneStep);
    }

    if (inWater && std::abs(forwardSpeed) > 1e-3) {
        const double diveLift { mConfig.divePlaneLiftCoeff * mDivePlaneAngle * forwardSpeed
            * std::abs(forwardSpeed) * submergenceRatio };
        const math::Vec2 bowLocalPos { mConfig.divePlaneDistance, 0.0 };
        body.applyForceAtLocalPoint(body.toWorldVec(math::Vec2 { 0.0, diveLift }), bowLocalPos);
    }

    // 7. Water rudder servo actuation and propulsion thrust vectoring
    const double targetRudder { std::clamp(mInput.rudder, -1.0, 1.0) * mConfig.maxRudderAngle };
    if (mConfig.rudderSlewRate <= 0.0) {
        mRudderAngle = targetRudder;
    } else {
        const double deltaRudder { targetRudder - mRudderAngle };
        const double maxRudderStep { mConfig.rudderSlewRate * dt };
        mRudderAngle += std::clamp(deltaRudder, -maxRudderStep, maxRudderStep);
    }

    const double throttle { std::clamp(mInput.throttle, -1.0, 1.0) };
    const double waterCoupling { inWater ? 1.0 : 0.05 };
    const double thrustMagnitude { throttle * mConfig.maxThrust * waterCoupling };

    if (std::abs(thrustMagnitude) > 1e-4) {
        const math::Vec2 thrustDirLocal { std::cos(mRudderAngle), -std::sin(mRudderAngle) };
        const math::Vec2 thrustLocal { thrustDirLocal * thrustMagnitude };
        const math::Vec2 sternLocalPos { -mConfig.rudderDistance, 0.0 };
        body.applyForceAtLocalPoint(body.toWorldVec(thrustLocal), sternLocalPos);
    }

    // 8. Metacentric pitch stability and rotational damping
    if (inWater) {
        const double restoringTorque { -std::sin(orientation) * mConfig.metacentricStiffness * submergenceRatio };
        const double dampingTorque { -mConfig.rotationalDamping * omega * submergenceRatio };
        body.applyTorque(restoringTorque + dampingTorque);
    }

    // 9. Telemetry update
    mTelemetry.depth = submergedDepth;
    mTelemetry.ballastMass = mBallastMass;
    mTelemetry.ballastRatio = (mConfig.maxBallastMass > 0.0)
        ? (mBallastMass / mConfig.maxBallastMass) : 0.0;
    mTelemetry.totalMass = totalMass;
    mTelemetry.netBuoyancy = netVerticalBuoyancy;
    mTelemetry.thrustForce = thrustMagnitude;
    mTelemetry.divePlaneAngle = mDivePlaneAngle;
    mTelemetry.rudderAngle = mRudderAngle;
    mTelemetry.forwardSpeed = forwardSpeed;
    mTelemetry.verticalSpeed = worldVel.y;
    mTelemetry.isSubmerged = inWater;
    mTelemetry.isCrushed = crushed;
}

void SubmarineDynamics::reset() noexcept
{
    mInput = SubmarineInput { };
    mBallastMass = mConfig.initialBallastMass;
    mRudderAngle = 0.0;
    mDivePlaneAngle = 0.0;
    mTelemetry = SubmarineTelemetry { };
    mTelemetry.ballastMass = mBallastMass;
    mTelemetry.totalMass = mConfig.dryMass + mBallastMass;
    mTelemetry.ballastRatio = (mConfig.maxBallastMass > 0.0)
        ? (mBallastMass / mConfig.maxBallastMass) : 0.0;
}

bool SubmarineDynamics::isAirborne() const noexcept
{
    return !mTelemetry.isSubmerged;
}

const SubmarineConfig& SubmarineDynamics::getConfig() const noexcept
{
    return mConfig;
}

void SubmarineDynamics::setConfig(const SubmarineConfig& config) noexcept
{
    mConfig = config;
}

const SubmarineInput& SubmarineDynamics::getInput() const noexcept
{
    return mInput;
}

void SubmarineDynamics::setInput(const SubmarineInput& input) noexcept
{
    mInput = input;
}

const SubmarineTelemetry& SubmarineDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool SubmarineDynamics::isSubmerged() const noexcept
{
    return mTelemetry.isSubmerged;
}

bool SubmarineDynamics::isCrushed() const noexcept
{
    return mTelemetry.isCrushed;
}

double SubmarineDynamics::getDepth() const noexcept
{
    return mTelemetry.depth;
}

double SubmarineDynamics::getNetBuoyancy() const noexcept
{
    return mTelemetry.netBuoyancy;
}

double SubmarineDynamics::getBallastMass() const noexcept
{
    return mBallastMass;
}

} // namespace vehicle_sim::dynamics
