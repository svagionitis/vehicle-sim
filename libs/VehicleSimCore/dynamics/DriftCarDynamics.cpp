#include "libs/VehicleSimCore/dynamics/DriftCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

DriftCarDynamics::DriftCarDynamics() noexcept
    : DriftCarDynamics(DriftCarConfig {})
{
}

DriftCarDynamics::DriftCarDynamics(const DriftCarConfig& config) noexcept
    : mConfig { config }
{
    setupWheelOffsets();

    // Default configuration for tires if uninitialized
    PacejkaConfig frontCfg { mConfig.frontTireConfig };
    if (frontCfg.nominalLoadFz <= 0.0) {
        frontCfg = PacejkaTireModel::getPreset(TirePresetType::RacingSlick);
        frontCfg.lateral.D = 1.15; // Higher grip front tires to steer out of drift
    }
    PacejkaConfig rearCfg { mConfig.rearTireConfig };
    if (rearCfg.nominalLoadFz <= 0.0) {
        rearCfg = PacejkaTireModel::getPreset(TirePresetType::StandardRoad);
        rearCfg.lateral.D = 0.88; // Lower grip drift compound rear tires
        rearCfg.lateral.E = -0.7; // Progressive breakaway curve
    }

    mTires[0].setConfig(frontCfg);
    mTires[1].setConfig(frontCfg);
    mTires[2].setConfig(rearCfg);
    mTires[3].setConfig(rearCfg);

    for (auto& wheel : mWheels) {
        wheel.temperature = mConfig.ambientTemperature;
        wheel.effectiveMu = 1.0;
    }
}

void DriftCarDynamics::setupWheelOffsets() noexcept
{
    const double a { mConfig.cogDistFront };
    const double b { mConfig.wheelbase - a };
    const double halfTrack { mConfig.trackWidth * 0.5 };

    // 0: FL, 1: FR, 2: RL, 3: RR
    mWheels[0].localOffset = math::Vec2 { a, halfTrack };
    mWheels[1].localOffset = math::Vec2 { a, -halfTrack };
    mWheels[2].localOffset = math::Vec2 { -b, halfTrack };
    mWheels[3].localOffset = math::Vec2 { -b, -halfTrack };
}

void DriftCarDynamics::reset() noexcept
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
        wheel.temperature = mConfig.ambientTemperature;
        wheel.effectiveMu = 1.0;
    }
    mThrottle = 0.0;
    mBrake = 0.0;
    mSteer = 0.0;
    mHandbrake = 0.0;
    mClutchKickTimer = 0.0;
    mCurrentDriftAngle = 0.0;
    mCurrentAlignTorque = 0.0;
    mIsAirborne = false;
}

bool DriftCarDynamics::isAirborne() const noexcept
{
    return mIsAirborne;
}

void DriftCarDynamics::setThrottle(double throttle) noexcept
{
    mThrottle = std::clamp(throttle, 0.0, 1.0);
}

void DriftCarDynamics::setBrake(double brake) noexcept
{
    mBrake = std::clamp(brake, 0.0, 1.0);
}

void DriftCarDynamics::setSteer(double steer) noexcept
{
    mSteer = std::clamp(steer, -1.0, 1.0);
}

void DriftCarDynamics::setHandbrake(double handbrake) noexcept
{
    mHandbrake = std::clamp(handbrake, 0.0, 1.0);
}

void DriftCarDynamics::triggerClutchKick() noexcept
{
    mClutchKickTimer = mConfig.clutchKickDuration;
}

void DriftCarDynamics::setAirborne(bool airborne) noexcept
{
    mIsAirborne = airborne;
}

double DriftCarDynamics::getDriftAngle() const noexcept
{
    return mCurrentDriftAngle;
}

double DriftCarDynamics::getAligningTorque() const noexcept
{
    return mCurrentAlignTorque;
}

bool DriftCarDynamics::isDrifting() const noexcept
{
    constexpr double minDriftAngle { 0.20 }; // ~11.5 degrees
    constexpr double minSlideSlip { 0.15 };
    const bool rearSlipping { (std::abs(mWheels[2].slipAngle) > minSlideSlip)
        || (std::abs(mWheels[3].slipAngle) > minSlideSlip) };
    return (std::abs(mCurrentDriftAngle) >= minDriftAngle) && rearSlipping;
}

std::array<double, 4> DriftCarDynamics::getTireTemperatures() const noexcept
{
    return { mWheels[0].temperature, mWheels[1].temperature, mWheels[2].temperature, mWheels[3].temperature };
}

const std::array<DriftCarWheelState, 4>& DriftCarDynamics::getWheelStates() const noexcept
{
    return mWheels;
}

const DriftCarConfig& DriftCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

void DriftCarDynamics::computeWheelLoads(double ax, double ay) noexcept
{
    constexpr double gravityConst { 9.81 };
    const double totalWeight { mConfig.mass * gravityConst };
    const double L { mConfig.wheelbase };
    const double a { mConfig.cogDistFront };
    const double b { L - a };
    const double h { mConfig.cogHeight };
    const double T { mConfig.trackWidth };

    const double staticFront { totalWeight * (b / L) };
    const double staticRear { totalWeight * (a / L) };

    const double deltaLong { mConfig.mass * ax * (h / L) };
    constexpr double rollStiffFront { 0.52 };
    const double deltaLatFront { rollStiffFront * mConfig.mass * ay * (h / T) };
    const double deltaLatRear { (1.0 - rollStiffFront) * mConfig.mass * ay * (h / T) };

    // 0: FL, 1: FR, 2: RL, 3: RR
    mWheels[0].normalLoad = std::max(0.0, 0.5 * staticFront - 0.5 * deltaLong - deltaLatFront);
    mWheels[1].normalLoad = std::max(0.0, 0.5 * staticFront - 0.5 * deltaLong + deltaLatFront);
    mWheels[2].normalLoad = std::max(0.0, 0.5 * staticRear + 0.5 * deltaLong - deltaLatRear);
    mWheels[3].normalLoad = std::max(0.0, 0.5 * staticRear + 0.5 * deltaLong + deltaLatRear);
}

void DriftCarDynamics::updateThermalModel(double dt) noexcept
{
    for (auto& wheel : mWheels) {
        // Friction power dissipation: P = |Fx * vx_slip| + |Fy * vy_slip|
        constexpr double wheelRadius { 0.32 };
        const double vxSlip { std::abs(wheel.slipRatio) * wheelRadius * 10.0 };
        const double vySlip { std::abs(std::tan(wheel.slipAngle)) * 10.0 };
        const double frictionPower { std::abs(wheel.tractiveForce) * vxSlip + std::abs(wheel.corneringForce) * vySlip };

        const double heatInput { (frictionPower / mConfig.thermalCapacity) * dt };
        const double heatLoss { mConfig.coolingRate * (wheel.temperature - mConfig.ambientTemperature) * dt };
        wheel.temperature = std::clamp(wheel.temperature + heatInput - heatLoss, mConfig.ambientTemperature, 350.0);

        // Effective friction degradation
        const double overTemp { std::max(0.0, wheel.temperature - mConfig.optimalTireTemp) };
        const double dropFactor { mConfig.thermalFrictionDecay * std::tanh(overTemp / mConfig.tempScaleFactor) };
        wheel.effectiveMu = std::clamp(1.0 - dropFactor, 0.20, 1.0);
    }
}

void DriftCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const math::Vec2 bodyVelWorld { body.getLinearVelocity() };
    const math::Vec2 bodyVelLocal { body.toLocalVec(bodyVelWorld) };
    const double yawRate { body.getAngularVelocity() };
    const double forwardSpeed { bodyVelLocal.x };
    const double lateralSpeed { bodyVelLocal.y };

    // Body slip angle beta = atan2(v, u)
    if (std::abs(forwardSpeed) > 0.5 || std::abs(lateralSpeed) > 0.5) {
        mCurrentDriftAngle = std::atan2(lateralSpeed, std::max(0.1, std::abs(forwardSpeed)));
    } else {
        mCurrentDriftAngle = 0.0;
    }

    if (mIsAirborne) {
        for (auto& wheel : mWheels) {
            wheel.normalLoad = 0.0;
            wheel.tractiveForce = 0.0;
            wheel.corneringForce = 0.0;
            wheel.slipRatio = 0.0;
            wheel.slipAngle = 0.0;
        }
        mCurrentAlignTorque = 0.0;
        return;
    }

    const double ax { std::clamp(forwardSpeed * 0.5, -25.0, 25.0) };
    const double ay { std::clamp(forwardSpeed * yawRate, -30.0, 30.0) };
    computeWheelLoads(ax, ay);

    // Clutch kick update
    double clutchKickAddTorque { 0.0 };
    if (mClutchKickTimer > 0.0) {
        clutchKickAddTorque = mConfig.clutchKickTorque;
        mClutchKickTimer = std::max(0.0, mClutchKickTimer - dt);
    }

    // Front high-angle Ackermann steering
    const double baseSteer { mSteer * mConfig.maxSteerAngle };
    double steerFL { baseSteer };
    double steerFR { baseSteer };
    if (std::abs(baseSteer) > 1e-4) {
        const double L { mConfig.wheelbase };
        const double halfT { mConfig.trackWidth * 0.5 };
        const double turnRadius { L / std::tan(baseSteer) };
        steerFL = std::atan(L / (turnRadius - halfT));
        steerFR = std::atan(L / (turnRadius + halfT));
    }
    mWheels[0].steerAngle = steerFL;
    mWheels[1].steerAngle = steerFR;
    mWheels[2].steerAngle = 0.0;
    mWheels[3].steerAngle = 0.0;

    // Longitudinal forces per wheel
    const double totalDrive { mThrottle * mConfig.maxDriveForce + clutchKickAddTorque };
    const double rearDrivePerWheel { totalDrive * 0.5 }; // Rear-wheel drive

    constexpr double footBrakeBiasFront { 0.65 };
    const double totalFootBrake { mBrake * mConfig.maxFootBrakeForce };
    const double frontFootBrake { totalFootBrake * footBrakeBiasFront * 0.5 };
    const double rearFootBrake { totalFootBrake * (1.0 - footBrakeBiasFront) * 0.5 };

    const double handbrakeRearPerWheel { mHandbrake * mConfig.maxHandbrakeForce * 0.5 };

    // Process each wheel
    for (std::size_t i { 0 }; i < 4; ++i) {
        DriftCarWheelState& wheel { mWheels[i] };
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

        const bool isFront { i < 2 };
        double driveDemand { isFront ? 0.0 : rearDrivePerWheel };
        double brakeDemand { isFront ? frontFootBrake : (rearFootBrake + handbrakeRearPerWheel) };

        double tractiveForce { driveDemand };
        if (brakeDemand > 0.0 && std::abs(wheelLongVel) > 1e-2) {
            const double brakeSign { (wheelLongVel > 0.0) ? -1.0 : 1.0 };
            tractiveForce += brakeSign * brakeDemand;
        }

        // Handbrake lockup enforcement: forces high negative slip ratio
        double slipRatio { 0.0 };
        if (!isFront && mHandbrake > 0.1) {
            slipRatio = -0.95 * mHandbrake;
            tractiveForce = -wheel.normalLoad * 0.85 * mHandbrake * ((wheelLongVel >= 0.0) ? 1.0 : -1.0);
        } else if (!isFront && clutchKickAddTorque > 0.0) {
            slipRatio = 0.85; // High spin breaking static adhesion
        } else {
            slipRatio = std::clamp(tractiveForce / (wheel.normalLoad * 1.5 + 1e-4), -1.0, 1.0);
        }

        const double safeVx { std::max(1.0, std::abs(wheelLongVel)) };
        const double slipAngle { -std::atan2(wheelLatVel, safeVx) };

        TireState tireState {};
        tireState.forwardSpeed = wheelLongVel;
        tireState.lateralSpeed = wheelLatVel;
        tireState.normalLoad = wheel.normalLoad;
        tireState.surfaceFriction = wheel.effectiveMu;
        constexpr double wheelRadius { 0.32 };
        tireState.angularVelocity = (wheelLongVel + slipRatio * safeVx) / wheelRadius;

        const TireForces forces { mTires[i].evaluateForces(tireState, dt) };

        wheel.slipRatio = slipRatio;
        wheel.slipAngle = slipAngle;
        wheel.tractiveForce = forces.fLong;
        wheel.corneringForce = forces.fLat;

        // Transform wheel frame force back to chassis local frame
        const double fLocalX { forces.fLong * cosS - forces.fLat * sinS };
        const double fLocalY { forces.fLong * sinS + forces.fLat * cosS };
        const math::Vec2 forceLocal { fLocalX, fLocalY };
        const math::Vec2 forceWorld { body.toWorldVec(forceLocal) };

        body.applyForceAtLocalPoint(forceWorld, wheel.localOffset);
    }

    // Compute front aligning torque from caster & pneumatic trail
    const double frontCorneringSum { mWheels[0].corneringForce + mWheels[1].corneringForce };
    constexpr double pneumaticTrail { 0.020 };
    const double totalTrail { pneumaticTrail + mConfig.mechanicalTrail };
    mCurrentAlignTorque = -totalTrail * frontCorneringSum;

    // Thermal simulation tick
    updateThermalModel(dt);
}

} // namespace vehicle_sim::dynamics
