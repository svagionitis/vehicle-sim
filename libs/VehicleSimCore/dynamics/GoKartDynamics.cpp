#include "libs/VehicleSimCore/dynamics/GoKartDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

GoKartDynamics::GoKartDynamics() noexcept
    : GoKartDynamics(GoKartConfig {})
{
}

GoKartDynamics::GoKartDynamics(const GoKartConfig& config) noexcept
    : mConfig { config }
{
    setupWheelOffsets();

    PacejkaConfig frontCfg { mConfig.frontTireConfig };
    if (frontCfg.nominalLoadFz <= 0.0) {
        frontCfg = PacejkaTireModel::getPreset(TirePresetType::RacingSlick);
        frontCfg.wheelRadius = 0.13; // 5-inch kart rim (~10x4.5-5 tire)
        frontCfg.nominalLoadFz = 400.0;
        frontCfg.lateral.D = 1.30;
    }
    PacejkaConfig rearCfg { mConfig.rearTireConfig };
    if (rearCfg.nominalLoadFz <= 0.0) {
        rearCfg = PacejkaTireModel::getPreset(TirePresetType::RacingSlick);
        rearCfg.wheelRadius = 0.14; // ~11x7.10-5 wide rear slick
        rearCfg.nominalLoadFz = 600.0;
        rearCfg.lateral.D = 1.35;
    }

    mTires[0].setConfig(frontCfg);
    mTires[1].setConfig(frontCfg);
    mTires[2].setConfig(rearCfg);
    mTires[3].setConfig(rearCfg);
}

void GoKartDynamics::setupWheelOffsets() noexcept
{
    const double a { mConfig.cogDistFront };
    const double b { mConfig.wheelbase - a };
    const double halfTrackFront { mConfig.trackWidthFront * 0.5 };
    const double halfTrackRear { mConfig.trackWidthRear * 0.5 };

    // 0: FL, 1: FR, 2: RL, 3: RR
    mWheels[0].localOffset = math::Vec2 { a, halfTrackFront };
    mWheels[1].localOffset = math::Vec2 { a, -halfTrackFront };
    mWheels[2].localOffset = math::Vec2 { -b, halfTrackRear };
    mWheels[3].localOffset = math::Vec2 { -b, -halfTrackRear };
}

void GoKartDynamics::reset() noexcept
{
    for (auto& tire : mTires) {
        tire.reset();
    }
    for (auto& wheel : mWheels) {
        wheel.normalLoad = 0.0;
        wheel.slipRatio = 0.0;
        wheel.slipAngle = 0.0;
        wheel.tractiveForce = 0.0;
        wheel.corneringForce = 0.0;
        wheel.steerAngle = 0.0;
        wheel.isLifted = false;
    }
    mThrottle = 0.0;
    mBrake = 0.0;
    mSteer = 0.0;
    mAxleScrubTorque = 0.0;
    mCurrentJackingForce = 0.0;
    mIsInnerWheelLifted = false;
    mIsAirborne = false;
}

bool GoKartDynamics::isAirborne() const noexcept
{
    return mIsAirborne;
}

void GoKartDynamics::setThrottle(double throttle) noexcept
{
    mThrottle = std::clamp(throttle, 0.0, 1.0);
}

void GoKartDynamics::setBrake(double brake) noexcept
{
    mBrake = std::clamp(brake, 0.0, 1.0);
}

void GoKartDynamics::setSteer(double steer) noexcept
{
    mSteer = std::clamp(steer, -1.0, 1.0);
}

void GoKartDynamics::setAirborne(bool airborne) noexcept
{
    mIsAirborne = airborne;
}

bool GoKartDynamics::isInnerWheelLifted() const noexcept
{
    return mIsInnerWheelLifted;
}

double GoKartDynamics::getAxleScrubTorque() const noexcept
{
    return mAxleScrubTorque;
}

double GoKartDynamics::getJackingForce() const noexcept
{
    return mCurrentJackingForce;
}

const std::array<GoKartWheelState, 4>& GoKartDynamics::getWheelStates() const noexcept
{
    return mWheels;
}

const GoKartConfig& GoKartDynamics::getConfig() const noexcept
{
    return mConfig;
}

void GoKartDynamics::computeJackingLoads(double ay) noexcept
{
    constexpr double gravityConst { 9.81 };
    const double totalWeight { mConfig.mass * gravityConst };
    const double L { mConfig.wheelbase };
    const double a { mConfig.cogDistFront };
    const double b { L - a };
    const double h { mConfig.cogHeight };
    const double Tr { mConfig.trackWidthRear };
    const double Tf { mConfig.trackWidthFront };

    const double staticFront { totalWeight * (b / L) };
    const double staticRear { totalWeight * (a / L) };

    // Front lateral load transfer
    const double deltaLatFront { (staticFront / totalWeight) * mConfig.mass * ay * (h / Tf) };
    mWheels[0].normalLoad = std::max(0.0, 0.5 * staticFront - deltaLatFront);
    mWheels[1].normalLoad = std::max(0.0, 0.5 * staticFront + deltaLatFront);

    // Caster jacking force: combines front spindle height difference with chassis roll moment
    const double steerAngleAbs { std::abs(mSteer * mConfig.maxSteerAngle) };
    const double casterGeoJacking { staticFront * mConfig.jackingFactor * steerAngleAbs
        * std::sin(mConfig.casterAngleRad) };
    const double latAccJacking { mConfig.mass * std::abs(ay) * (h / Tr) };
    const double totalJacking { casterGeoJacking + latAccJacking };
    mCurrentJackingForce = totalJacking;

    const double staticRearPerWheel { 0.5 * staticRear };

    mWheels[2].isLifted = false;
    mWheels[3].isLifted = false;
    mIsInnerWheelLifted = false;

    if (mSteer > 1e-3) {
        // Turning Left: RL (wheel 2) is the inside wheel
        const double insideLoad { staticRearPerWheel - totalJacking };
        if (insideLoad <= 0.0) {
            mWheels[2].normalLoad = 0.0;
            mWheels[2].isLifted = true;
            mWheels[3].normalLoad = staticRear; // Outside wheel takes full axle load
            mIsInnerWheelLifted = true;
        } else {
            mWheels[2].normalLoad = insideLoad;
            mWheels[3].normalLoad = staticRear - insideLoad;
        }
    } else if (mSteer < -1e-3) {
        // Turning Right: RR (wheel 3) is the inside wheel
        const double insideLoad { staticRearPerWheel - totalJacking };
        if (insideLoad <= 0.0) {
            mWheels[3].normalLoad = 0.0;
            mWheels[3].isLifted = true;
            mWheels[2].normalLoad = staticRear; // Outside wheel takes full axle load
            mIsInnerWheelLifted = true;
        } else {
            mWheels[3].normalLoad = insideLoad;
            mWheels[2].normalLoad = staticRear - insideLoad;
        }
    } else {
        // Straight line
        mWheels[2].normalLoad = staticRearPerWheel;
        mWheels[3].normalLoad = staticRearPerWheel;
    }
}

void GoKartDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const math::Vec2 bodyVelWorld { body.getLinearVelocity() };
    const math::Vec2 bodyVelLocal { body.toLocalVec(bodyVelWorld) };
    const double yawRate { body.getAngularVelocity() };
    const double forwardSpeed { bodyVelLocal.x };
    const double lateralSpeed { bodyVelLocal.y };

    if (mIsAirborne) {
        for (auto& wheel : mWheels) {
            wheel.normalLoad = 0.0;
            wheel.tractiveForce = 0.0;
            wheel.corneringForce = 0.0;
            wheel.slipRatio = 0.0;
            wheel.slipAngle = 0.0;
            wheel.isLifted = true;
        }
        mAxleScrubTorque = 0.0;
        return;
    }

    const double ay { forwardSpeed * yawRate };
    computeJackingLoads(ay);

    // Direct 1:1 steering to front spindles
    const double steerAngle { mSteer * mConfig.maxSteerAngle };
    mWheels[0].steerAngle = steerAngle;
    mWheels[1].steerAngle = steerAngle;
    mWheels[2].steerAngle = 0.0;
    mWheels[3].steerAngle = 0.0;

    // Total drive and braking demands
    const double totalDrive { mThrottle * mConfig.maxDriveForce };
    const double totalBrake { mBrake * mConfig.maxBrakeForce };

    // Front wheels (unpowered, free-rolling)
    for (std::size_t i { 0 }; i < 2; ++i) {
        GoKartWheelState& wheel { mWheels[i] };
        if (wheel.normalLoad <= 0.0) {
            wheel.tractiveForce = 0.0;
            wheel.corneringForce = 0.0;
            wheel.slipRatio = 0.0;
            wheel.slipAngle = 0.0;
            continue;
        }

        const math::Vec2 hubVelWorld { body.getVelocityAtLocalPoint(wheel.localOffset) };
        const math::Vec2 hubVelLocal { body.toLocalVec(hubVelWorld) };

        const double cosS { std::cos(wheel.steerAngle) };
        const double sinS { std::sin(wheel.steerAngle) };

        const double wheelLongVel { hubVelLocal.x * cosS + hubVelLocal.y * sinS };
        const double wheelLatVel { -hubVelLocal.x * sinS + hubVelLocal.y * cosS };

        // Front wheels have slight rolling resistance
        constexpr double rollingCoeff { 0.015 };
        const double fLong { -rollingCoeff * wheel.normalLoad * std::tanh(wheelLongVel * 3.0) };

        const double safeVx { std::max(0.5, std::abs(wheelLongVel)) };
        const double slipAngle { -std::atan2(wheelLatVel, safeVx) };

        TireState tireState {};
        tireState.forwardSpeed = wheelLongVel;
        tireState.lateralSpeed = wheelLatVel;
        tireState.normalLoad = wheel.normalLoad;
        tireState.surfaceFriction = 1.0;
        tireState.angularVelocity = wheelLongVel / 0.13;

        const TireForces forces { mTires[i].evaluateForces(tireState, dt) };

        wheel.slipRatio = 0.0;
        wheel.slipAngle = slipAngle;
        wheel.tractiveForce = fLong;
        wheel.corneringForce = forces.fLat;

        const double fLocalX { fLong * cosS - forces.fLat * sinS };
        const double fLocalY { fLong * sinS + forces.fLat * cosS };
        const math::Vec2 forceLocal { fLocalX, fLocalY };
        const math::Vec2 forceWorld { body.toWorldVec(forceLocal) };

        body.applyForceAtLocalPoint(forceWorld, wheel.localOffset);
    }

    // Rear solid spool axle simulation (indices 2 & 3: RL & RR)
    const double halfTr { mConfig.trackWidthRear * 0.5 };
    const double speedRL { forwardSpeed - halfTr * yawRate };
    const double speedRR { forwardSpeed + halfTr * yawRate };
    const double avgRearSpeed { forwardSpeed };

    // Solid shaft common drive speed
    const double driveNetPerWheel { (totalDrive - totalBrake) * 0.5 };

    if (!mIsInnerWheelLifted) {
        // Both rear wheels on track -> solid axle forces scrub slip differences
        const double safeSpdRL { std::max(0.5, std::abs(speedRL)) };
        const double safeSpdRR { std::max(0.5, std::abs(speedRR)) };

        const double scrubRatioRL { (avgRearSpeed - speedRL) / safeSpdRL };
        const double scrubRatioRR { (avgRearSpeed - speedRR) / safeSpdRR };

        for (std::size_t i { 2 }; i < 4; ++i) {
            GoKartWheelState& wheel { mWheels[i] };
            const bool isRL { i == 2 };
            const double wheelLongVel { isRL ? speedRL : speedRR };
            const double scrubRatio { isRL ? scrubRatioRL : scrubRatioRR };
            const double netSlipRatio { std::clamp(
                (driveNetPerWheel / (wheel.normalLoad + 1e-4)) + scrubRatio * 0.5, -1.0, 1.0) };

            const double safeVx { std::max(0.5, std::abs(wheelLongVel)) };
            const double slipAngle { -std::atan2(lateralSpeed, safeVx) };

            TireState tireState {};
            tireState.forwardSpeed = wheelLongVel;
            tireState.lateralSpeed = lateralSpeed;
            tireState.normalLoad = wheel.normalLoad;
            tireState.surfaceFriction = 1.0;
            tireState.angularVelocity = (wheelLongVel + netSlipRatio * safeVx) / 0.14;

            const TireForces forces { mTires[i].evaluateForces(tireState, dt) };

            wheel.slipRatio = netSlipRatio;
            wheel.slipAngle = slipAngle;
            wheel.tractiveForce = forces.fLong;
            wheel.corneringForce = forces.fLat;

            const math::Vec2 forceLocal { forces.fLong, forces.fLat };
            const math::Vec2 forceWorld { body.toWorldVec(forceLocal) };
            body.applyForceAtLocalPoint(forceWorld, wheel.localOffset);
        }

        // Calculate opposing scrub torque opposing turn-in
        mAxleScrubTorque = (mWheels[3].tractiveForce - mWheels[2].tractiveForce) * halfTr;
    } else {
        // Tripod cornering: inside wheel lifted! Zero axle binding scrub!
        mAxleScrubTorque = 0.0;
        const std::size_t loadedIdx { mWheels[2].isLifted ? 3ULL : 2ULL };
        const std::size_t liftedIdx { mWheels[2].isLifted ? 2ULL : 3ULL };

        // Lifted wheel
        mWheels[liftedIdx].tractiveForce = 0.0;
        mWheels[liftedIdx].corneringForce = 0.0;
        mWheels[liftedIdx].slipRatio = 0.0;
        mWheels[liftedIdx].slipAngle = 0.0;

        // Loaded wheel gets full rear drive & braking
        GoKartWheelState& loadedWheel { mWheels[loadedIdx] };
        const bool isRL { loadedIdx == 2 };
        const double wheelLongVel { isRL ? speedRL : speedRR };
        const double safeVx { std::max(0.5, std::abs(wheelLongVel)) };
        const double slipAngle { -std::atan2(lateralSpeed, safeVx) };
        const double slipRatio { std::clamp(
            (totalDrive - totalBrake) / (loadedWheel.normalLoad * 1.5 + 1e-4), -1.0, 1.0) };

        TireState tireState {};
        tireState.forwardSpeed = wheelLongVel;
        tireState.lateralSpeed = lateralSpeed;
        tireState.normalLoad = loadedWheel.normalLoad;
        tireState.surfaceFriction = 1.0;
        tireState.angularVelocity = (wheelLongVel + slipRatio * safeVx) / 0.14;

        const TireForces forces { mTires[loadedIdx].evaluateForces(tireState, dt) };

        loadedWheel.slipRatio = slipRatio;
        loadedWheel.slipAngle = slipAngle;
        loadedWheel.tractiveForce = forces.fLong;
        loadedWheel.corneringForce = forces.fLat;

        const math::Vec2 forceLocal { forces.fLong, forces.fLat };
        const math::Vec2 forceWorld { body.toWorldVec(forceLocal) };
        body.applyForceAtLocalPoint(forceWorld, loadedWheel.localOffset);
    }
}

} // namespace vehicle_sim::dynamics
