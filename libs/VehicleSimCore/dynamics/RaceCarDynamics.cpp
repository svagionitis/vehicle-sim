#include "libs/VehicleSimCore/dynamics/RaceCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

RaceCarDynamics::RaceCarDynamics() noexcept
    : RaceCarDynamics(RaceCarConfig {})
{
}

RaceCarDynamics::RaceCarDynamics(const RaceCarConfig& config) noexcept
    : mConfig { config }
    , mAero { config.aeroConfig }
{
    setupWheelOffsets();
    for (auto& tire : mTires) {
        tire.setConfig(mConfig.tireConfig);
    }
}

void RaceCarDynamics::setupWheelOffsets() noexcept
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

void RaceCarDynamics::reset() noexcept
{
    mAero.reset();
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
    }
    mThrottle = 0.0;
    mBrake = 0.0;
    mSteer = 0.0;
    mPrevLongVel = 0.0;
    mPrevLatVel = 0.0;
    mCurrentDownforce = 0.0;
    mCurrentDrsAngle = 0.0;
    mDrsRequested = false;
    mIsAirborne = false;
}

bool RaceCarDynamics::isAirborne() const noexcept
{
    return mIsAirborne;
}

void RaceCarDynamics::setThrottle(double throttle) noexcept
{
    mThrottle = std::clamp(throttle, 0.0, 1.0);
}

void RaceCarDynamics::setBrake(double brake) noexcept
{
    mBrake = std::clamp(brake, 0.0, 1.0);
}

void RaceCarDynamics::setSteer(double steer) noexcept
{
    mSteer = std::clamp(steer, -1.0, 1.0);
}

void RaceCarDynamics::setDrsActive(bool active) noexcept
{
    mDrsRequested = active;
}

void RaceCarDynamics::setRideHeight(double frontHeight, double rearHeight) noexcept
{
    mFrontRideHeight = std::max(0.001, frontHeight);
    mRearRideHeight = std::max(0.001, rearHeight);
}

void RaceCarDynamics::setAirborne(bool airborne) noexcept
{
    mIsAirborne = airborne;
}

const std::array<RaceCarWheelState, 4>& RaceCarDynamics::getWheelStates() const noexcept
{
    return mWheels;
}

double RaceCarDynamics::getDownforce() const noexcept
{
    return mCurrentDownforce;
}

double RaceCarDynamics::getDrsAngleDeg() const noexcept
{
    return mCurrentDrsAngle;
}

const RaceCarConfig& RaceCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

void RaceCarDynamics::computeWheelLoads(double ax, double ay, double downforce) noexcept
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
    const double deltaLatFront { mConfig.rollStiffnessFront * mConfig.mass * ay * (h / T) };
    const double deltaLatRear { (1.0 - mConfig.rollStiffnessFront) * mConfig.mass * ay * (h / T) };

    const double downforceFront { downforce * (b / L) };
    const double downforceRear { downforce * (a / L) };

    // 0: FL, 1: FR, 2: RL, 3: RR
    // Positive ay is left turn -> lateral weight transfers to outside (Right) wheels
    mWheels[0].normalLoad = std::max(0.0, 0.5 * staticFront - 0.5 * deltaLong - deltaLatFront + 0.5 * downforceFront);
    mWheels[1].normalLoad = std::max(0.0, 0.5 * staticFront - 0.5 * deltaLong + deltaLatFront + 0.5 * downforceFront);
    mWheels[2].normalLoad = std::max(0.0, 0.5 * staticRear + 0.5 * deltaLong - deltaLatRear + 0.5 * downforceRear);
    mWheels[3].normalLoad = std::max(0.0, 0.5 * staticRear + 0.5 * deltaLong + deltaLatRear + 0.5 * downforceRear);
}

void RaceCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const math::Vec2 bodyVelWorld { body.getLinearVelocity() };
    const math::Vec2 bodyVelLocal { body.toLocalVec(bodyVelWorld) };
    const double yawRate { body.getAngularVelocity() };
    const double forwardSpeed { bodyVelLocal.x };
    const double lateralSpeed { bodyVelLocal.y };

    const double ax { std::clamp((forwardSpeed - mPrevLongVel) / dt, -30.0, 30.0) };
    const double ay { std::clamp(forwardSpeed * yawRate, -35.0, 35.0) };
    mPrevLongVel = forwardSpeed;
    mPrevLatVel = lateralSpeed;

    // Aerodynamics update
    GroundEffectAeroInput aeroInput {};
    aeroInput.frontRideHeight = mFrontRideHeight;
    aeroInput.rearRideHeight = mRearRideHeight;
    aeroInput.pitchAngle = body.getOrientation();
    aeroInput.steerDemand = mSteer;
    aeroInput.brakeDemand = mBrake;
    aeroInput.lateralAccelerationG = std::abs(ay) / 9.81;
    aeroInput.isAirborne = mIsAirborne;
    aeroInput.requestDrs = mDrsRequested;

    physics::FluidSample2D fluidSample {};
    fluidSample.density = 1.225;
    fluidSample.velocity = math::Vec2 { 0.0, 0.0 };

    const GroundEffectAeroOutput aeroOutput { mAero.applyForces(body, aeroInput, fluidSample, dt) };
    mCurrentDownforce = aeroOutput.downforce;
    mCurrentDrsAngle = aeroOutput.flapAngleDeg;

    if (mIsAirborne) {
        for (auto& wheel : mWheels) {
            wheel.normalLoad = 0.0;
            wheel.tractiveForce = 0.0;
            wheel.corneringForce = 0.0;
            wheel.slipRatio = 0.0;
            wheel.slipAngle = 0.0;
        }
        return;
    }

    computeWheelLoads(ax, ay, aeroOutput.downforce);

    // Compute Ackermann steer angles for front axle
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

    // Distribute drive and brake forces
    const double totalDrive { mThrottle * mConfig.maxDriveForce };
    const double rearDrivePerWheel { totalDrive * 0.5 }; // Rear-wheel drive racecar

    const double totalBrake { mBrake * mConfig.maxBrakeForce };
    const double frontBrakePerWheel { totalBrake * mConfig.brakeBiasFront * 0.5 };
    const double rearBrakePerWheel { totalBrake * (1.0 - mConfig.brakeBiasFront) * 0.5 };

    // Process each wheel
    for (std::size_t i { 0 }; i < 4; ++i) {
        RaceCarWheelState& wheel { mWheels[i] };
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

        // Velocities in wheel frame
        const double wheelLongVel { hubVelLocal.x * cosS + hubVelLocal.y * sinS };
        const double wheelLatVel { -hubVelLocal.x * sinS + hubVelLocal.y * cosS };

        // Wheel drive / brake demand
        const bool isFront { i < 2 };
        double driveCmd { isFront ? 0.0 : rearDrivePerWheel };
        const double brakeCmd { isFront ? frontBrakePerWheel : rearBrakePerWheel };

        double tractiveForce { driveCmd };
        if (brakeCmd > 0.0 && std::abs(wheelLongVel) > 1e-2) {
            const double brakeSign { (wheelLongVel > 0.0) ? -1.0 : 1.0 };
            tractiveForce += brakeSign * brakeCmd;
        }

        // Rolling resistance retarding motion
        const double rollingRes { -mConfig.rollingResistance * wheel.normalLoad * std::tanh(wheelLongVel * 2.0) };
        tractiveForce += rollingRes;

        // Kinematic slip calculation
        const double safeVx { std::max(1.0, std::abs(wheelLongVel)) };
        const double slipAngle { -std::atan2(wheelLatVel, safeVx) };
        const double slipRatio { std::clamp(tractiveForce / (wheel.normalLoad * 1.5 + 1e-4), -1.0, 1.0) };

        TireState tireState {};
        tireState.forwardSpeed = wheelLongVel;
        tireState.lateralSpeed = wheelLatVel;
        tireState.normalLoad = wheel.normalLoad;
        tireState.surfaceFriction = 1.0;
        tireState.angularVelocity = (wheelLongVel + slipRatio * safeVx) / mConfig.tireConfig.wheelRadius;

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
}

} // namespace vehicle_sim::dynamics
