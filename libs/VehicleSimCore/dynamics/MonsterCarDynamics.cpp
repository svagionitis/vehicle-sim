#include "MonsterCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

MonsterCarDynamics::MonsterCarDynamics() noexcept
    : MonsterCarDynamics(MonsterCarConfig {})
{
}

MonsterCarDynamics::MonsterCarDynamics(const MonsterCarConfig& config) noexcept
    : mConfig { config }
    , mEngineRPM { config.engine.idleRPM }
{
    const double totalWeight { mConfig.mass * mConfig.gravity };
    const double totalWheelbase { mConfig.distCgToFront + mConfig.distCgToRear };
    const double wheelbase { (totalWheelbase > 1e-4) ? totalWheelbase : 1.0 };

    mFrontAxleWeight = (mConfig.distCgToRear / wheelbase) * totalWeight;
    mRearAxleWeight = (mConfig.distCgToFront / wheelbase) * totalWeight;

    mWheels[kFrontLeft].normalLoad = 0.5 * mFrontAxleWeight;
    mWheels[kFrontRight].normalLoad = 0.5 * mFrontAxleWeight;
    mWheels[kRearLeft].normalLoad = 0.5 * mRearAxleWeight;
    mWheels[kRearRight].normalLoad = 0.5 * mRearAxleWeight;

    for (auto& tire : mPacejkaTires) {
        tire.setConfig(mConfig.pacejkaConfig);
    }

    mGroundEffectAero.setConfig(mConfig.groundEffectConfig);
    updateWheelLayout();
}

void MonsterCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    updateFlight(dt);
    updateSteering(dt);

    const math::Vec2 bodyLocalVel { body.toLocalVec(body.getLinearVelocity()) };
    mSpeed = bodyLocalVel.x;

    double rawAccel { 0.0 };
    if (!mHasSpeedSample) {
        mPrevSpeed = mSpeed;
        mHasSpeedSample = true;
    } else {
        rawAccel = (mSpeed - mPrevSpeed) / dt;
        mPrevSpeed = mSpeed;
    }
    mAcceleration = (0.25 * rawAccel) + (0.75 * mAcceleration);

    if (isAirborne()) {
        for (auto& wheel : mWheels) {
            wheel.totalForce = math::Vec2::zero();
            wheel.velocity = math::Vec2::zero();
            wheel.longitudinalVelocity = 0.0;
            wheel.lateralVelocity = 0.0;
            wheel.slipAngle = 0.0;
            wheel.normalLoad = 0.0;
            wheel.lateralForce = 0.0;
            wheel.longitudinalForce = 0.0;
            wheel.isSaturated = false;
        }

        const math::Vec2 v { body.getLinearVelocity() };
        const double speed { v.length() };
        if (speed > 1e-4) {
            const math::Vec2 airDrag { -mConfig.airDragCoefficient * speed * v };
            body.applyForce(airDrag);
        }
        return;
    }

    const double totalWeight { mConfig.mass * mConfig.gravity };
    const double totalWheelbase { mConfig.distCgToFront + mConfig.distCgToRear };
    const double wheelbase { (totalWheelbase > 1e-4) ? totalWheelbase : 1.0 };

    const double weightTransfer { (mConfig.cgHeight / wheelbase) * mConfig.mass * mAcceleration };
    double wFront { ((mConfig.distCgToRear / wheelbase) * totalWeight) - weightTransfer };
    double wRear { ((mConfig.distCgToFront / wheelbase) * totalWeight) + weightTransfer };

    if (wFront < 0.0) {
        wFront = 0.0;
        wRear = totalWeight;
    } else if (wRear < 0.0) {
        wRear = 0.0;
        wFront = totalWeight;
    }

    mFrontAxleWeight = wFront;
    mRearAxleWeight = wRear;

    mWheels[kFrontLeft].normalLoad = 0.5 * wFront;
    mWheels[kFrontRight].normalLoad = 0.5 * wFront;
    mWheels[kRearLeft].normalLoad = 0.5 * wRear;
    mWheels[kRearRight].normalLoad = 0.5 * wRear;

    double currentRatio { 0.0 };
    if (mCurrentGear == -1) {
        currentRatio = mConfig.transmission.reverseRatio;
    } else if (mCurrentGear >= 1) {
        const std::size_t gearIndex { static_cast<std::size_t>(mCurrentGear - 1) };
        if (gearIndex < mConfig.transmission.gearRatios.size()) {
            currentRatio = mConfig.transmission.gearRatios[gearIndex];
        }
    }

    const double safeWheelRadius { (mConfig.wheelRadius > 1e-4) ? mConfig.wheelRadius : 0.34 };
    const double wheelAngSpeed { std::abs(mSpeed) / safeWheelRadius };
    const double engineAngSpeed { wheelAngSpeed * std::abs(currentRatio) * mConfig.transmission.differentialRatio };
    const double rawRPM { engineAngSpeed * (60.0 / (2.0 * 3.14159265358979323846)) };

    mEngineRPM = std::clamp(rawRPM, mConfig.engine.idleRPM, mConfig.engine.redlineRPM);
    mEngineTorque = calculateTorque(mEngineRPM);

    updateTransmission(dt);

    double driveForceTotal { 0.0 };
    const double clampedThrottle { std::clamp(mInput.throttle, 0.0, 1.0) };

    if (mCurrentGear > 0 && currentRatio > 0.0) {
        const double driveTorque { mEngineTorque * currentRatio * mConfig.transmission.differentialRatio
            * mConfig.transmission.transmissionEfficiency };
        driveForceTotal = clampedThrottle * (driveTorque / safeWheelRadius);
    } else if (mCurrentGear == -1 && currentRatio < 0.0) {
        const double driveTorque { mEngineTorque * std::abs(currentRatio) * mConfig.transmission.differentialRatio
            * mConfig.transmission.transmissionEfficiency };
        driveForceTotal = -clampedThrottle * (driveTorque / safeWheelRadius);
    }
    mDriveForce = driveForceTotal;

    std::size_t drivenCount { 0 };
    for (const auto& wheel : mWheels) {
        if (wheel.isDriven) {
            ++drivenCount;
        }
    }
    const double drivenFactor { drivenCount > 0 ? (1.0 / static_cast<double>(drivenCount)) : 0.0 };
    const double drivePerWheel { driveForceTotal * drivenFactor };

    const double clampedBrake { std::clamp(mInput.brake, 0.0, 1.0) };
    const double brakePerWheel { clampedBrake * mConfig.maxBrakeForce * 0.25 };

    for (std::size_t i { 0 }; i < kWheelCount; ++i) {
        MonsterWheelTelemetry& wheel { mWheels[i] };
        wheel.worldPosition = body.toWorldPoint(wheel.localOffset);
        wheel.velocity = body.getVelocityAtPoint(wheel.worldPosition);

        const double steerRad { wheel.steerAngle };
        const math::Vec2 wheelFwdLocal { std::cos(steerRad), std::sin(steerRad) };
        const math::Vec2 wheelLatLocal { -std::sin(steerRad), std::cos(steerRad) };

        const math::Vec2 uLong { body.toWorldVec(wheelFwdLocal) };
        const math::Vec2 uLat { body.toWorldVec(wheelLatLocal) };

        const double vLong { wheel.velocity.dot(uLong) };
        const double vLat { wheel.velocity.dot(uLat) };
        wheel.longitudinalVelocity = vLong;
        wheel.lateralVelocity = vLat;

        double slipAngle { 0.0 };
        const double absVLong { std::abs(vLong) };
        if (absVLong >= mConfig.lowSpeedCutoff) {
            slipAngle = std::atan2(vLat, absVLong);
        } else if (absVLong > 1e-3 || std::abs(vLat) > 1e-3) {
            const double blend { absVLong / mConfig.lowSpeedCutoff };
            slipAngle = std::atan2(vLat, mConfig.lowSpeedCutoff) * blend;
        }
        wheel.slipAngle = slipAngle;

        const double surfaceGrip { mGlobalGripModifier * mWheelGripModifiers[i] };
        wheel.surfaceGrip = surfaceGrip;

        double fLong { 0.0 };
        double fLat { 0.0 };

        if (mConfig.usePacejkaTire) {
            if (wheel.isDriven) {
                fLong += drivePerWheel * surfaceGrip;
            }
            const double rollingRes { -mConfig.rollingResistanceCoeff * vLong * mGlobalDragModifier };
            fLong += rollingRes;

            if (brakePerWheel > 0.0 && std::abs(vLong) > 1e-3) {
                const double brakeDir { (vLong > 0.0) ? -1.0 : 1.0 };
                fLong += brakeDir * brakePerWheel * surfaceGrip;
            }
            if (mInput.handbrake && !wheel.isSteering && std::abs(vLong) > 1e-3) {
                const double hbDir { (vLong > 0.0) ? -1.0 : 1.0 };
                fLong += hbDir * (mConfig.handbrakeBrakeForce * 0.5) * surfaceGrip;
            }

            TireState tState {};
            tState.forwardSpeed = vLong;
            tState.lateralSpeed = vLat;
            tState.angularVelocity = vLong / mConfig.wheelRadius;
            tState.normalLoad = wheel.normalLoad;
            tState.surfaceFriction = surfaceGrip;
            tState.surfaceDrag = mGlobalDragModifier;

            const TireForces tForces { mPacejkaTires[i].evaluateForces(tState, dt) };
            const double fLat0 { tForces.fLat };

            const double fMax { mConfig.tireFrictionCoefficient * surfaceGrip * wheel.normalLoad };
            const double Dx { (fMax > 1e-3) ? fMax : 1.0 };
            const double Dy { Dx };
            const double rho { std::hypot(fLong / Dx, fLat0 / Dy) };

            if (rho > 1.0) {
                fLong /= rho;
                fLat = fLat0 / rho;
                wheel.isSaturated = true;
            } else {
                fLat = fLat0;
                wheel.isSaturated = (rho > 0.95);
            }

            body.applyTorque(tForces.alignTorque);
        } else {
            const double corneringStiff { wheel.isSteering ? mConfig.frontCorneringStiffness
                                                           : mConfig.rearCorneringStiffness };
            fLat = -corneringStiff * slipAngle * surfaceGrip;

            if (wheel.isDriven) {
                fLong += drivePerWheel * surfaceGrip;
            }

            const double rollingRes { -mConfig.rollingResistanceCoeff * vLong * mGlobalDragModifier };
            fLong += rollingRes;

            if (brakePerWheel > 0.0 && std::abs(vLong) > 1e-3) {
                const double brakeDir { (vLong > 0.0) ? -1.0 : 1.0 };
                fLong += brakeDir * brakePerWheel * surfaceGrip;
            }

            if (mInput.handbrake && !wheel.isSteering && std::abs(vLong) > 1e-3) {
                const double hbDir { (vLong > 0.0) ? -1.0 : 1.0 };
                fLong += hbDir * (mConfig.handbrakeBrakeForce * 0.5) * surfaceGrip;
            }

            const double fMax { mConfig.tireFrictionCoefficient * surfaceGrip * wheel.normalLoad };
            const double fMag { std::hypot(fLong, fLat) };

            if (fMag > fMax && fMag > 1e-6) {
                const double scale { fMax / fMag };
                fLong *= scale;
                fLat *= scale;
                wheel.isSaturated = true;
            } else {
                wheel.isSaturated = false;
            }
        }

        wheel.longitudinalForce = fLong;
        wheel.lateralForce = fLat;

        const math::Vec2 wheelForceWorld { (uLong * fLong) + (uLat * fLat) };
        wheel.totalForce = wheelForceWorld;

        body.applyForceAtLocalPoint(wheelForceWorld, wheel.localOffset);
    }

    const math::Vec2 linVel { body.getLinearVelocity() };
    const double currentSpeed { linVel.length() };
    if (mConfig.useGroundEffect) {
        GroundEffectAeroInput aeroIn {};
        const double baseRideHeight { 0.05 + mFlightZ };
        aeroIn.frontRideHeight = std::max(0.005, baseRideHeight);
        aeroIn.rearRideHeight = std::max(0.005, baseRideHeight);
        aeroIn.brakeDemand = mInput.brake;
        aeroIn.steerDemand = mInput.steer;
        aeroIn.isAirborne = isAirborne();
        aeroIn.requestDrs = mInput.requestDrs;

        physics::FluidSample2D fluidSample {};
        fluidSample.density = 1.225;
        fluidSample.velocity = math::Vec2::zero();
        fluidSample.medium = physics::FluidMediumType::Air;

        static_cast<void>(mGroundEffectAero.applyForces(body, aeroIn, fluidSample, dt));
    } else if (currentSpeed > 1e-4) {
        const math::Vec2 dragForce { -mConfig.airDragCoefficient * currentSpeed * linVel };
        body.applyForce(dragForce);
    }
}

void MonsterCarDynamics::reset() noexcept
{
    mInput = MonsterCarInput {};
    mCurrentGear = 1;
    mSteerAngle = 0.0;
    mEngineRPM = mConfig.engine.idleRPM;
    mEngineTorque = 0.0;
    mDriveForce = 0.0;
    mSpeed = 0.0;
    mPrevSpeed = 0.0;
    mAcceleration = 0.0;
    mFlightZ = 0.0;
    mFlightVz = 0.0;
    mGlobalGripModifier = 1.0;
    mGlobalDragModifier = 1.0;
    mWheelGripModifiers = { 1.0, 1.0, 1.0, 1.0 };

    const double totalWeight { mConfig.mass * mConfig.gravity };
    const double totalWheelbase { mConfig.distCgToFront + mConfig.distCgToRear };
    const double wheelbase { (totalWheelbase > 1e-4) ? totalWheelbase : 1.0 };

    mFrontAxleWeight = (mConfig.distCgToRear / wheelbase) * totalWeight;
    mRearAxleWeight = (mConfig.distCgToFront / wheelbase) * totalWeight;
    mHasSpeedSample = false;

    mWheels[kFrontLeft].normalLoad = 0.5 * mFrontAxleWeight;
    mWheels[kFrontRight].normalLoad = 0.5 * mFrontAxleWeight;
    mWheels[kRearLeft].normalLoad = 0.5 * mRearAxleWeight;
    mWheels[kRearRight].normalLoad = 0.5 * mRearAxleWeight;

    for (auto& wheel : mWheels) {
        wheel.steerAngle = 0.0;
        wheel.totalForce = math::Vec2::zero();
        wheel.velocity = math::Vec2::zero();
        wheel.longitudinalVelocity = 0.0;
        wheel.lateralVelocity = 0.0;
        wheel.slipAngle = 0.0;
        wheel.lateralForce = 0.0;
        wheel.longitudinalForce = 0.0;
        wheel.surfaceGrip = 1.0;
        wheel.isSaturated = false;
    }

    for (auto& tire : mPacejkaTires) {
        tire.reset();
    }
    mGroundEffectAero.reset();
}

[[nodiscard]] bool MonsterCarDynamics::isAirborne() const noexcept
{
    return mFlightZ > 0.0;
}

void MonsterCarDynamics::setInput(const MonsterCarInput& input) noexcept
{
    mInput = input;
}

[[nodiscard]] const MonsterCarInput& MonsterCarDynamics::getInput() const noexcept
{
    return mInput;
}

[[nodiscard]] const MonsterCarConfig& MonsterCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

void MonsterCarDynamics::setConfig(const MonsterCarConfig& config) noexcept
{
    mConfig = config;
    for (auto& tire : mPacejkaTires) {
        tire.setConfig(mConfig.pacejkaConfig);
    }
    mGroundEffectAero.setConfig(mConfig.groundEffectConfig);
    updateWheelLayout();
}

[[nodiscard]] const GroundEffectAero2D& MonsterCarDynamics::getGroundEffect() const noexcept
{
    return mGroundEffectAero;
}

void MonsterCarDynamics::setGear(std::int32_t gear) noexcept
{
    const std::int32_t maxGear { static_cast<std::int32_t>(mConfig.transmission.gearRatios.size()) };
    if (gear >= -1 && gear <= maxGear) {
        mCurrentGear = gear;
    }
}

[[nodiscard]] std::int32_t MonsterCarDynamics::getGear() const noexcept
{
    return mCurrentGear;
}

void MonsterCarDynamics::shiftUp() noexcept
{
    const std::int32_t maxGear { static_cast<std::int32_t>(mConfig.transmission.gearRatios.size()) };
    if (mCurrentGear < maxGear) {
        if (mCurrentGear == -1) {
            mCurrentGear = 1;
        } else {
            ++mCurrentGear;
        }
    }
}

void MonsterCarDynamics::shiftDown() noexcept
{
    if (mCurrentGear > 1) {
        --mCurrentGear;
    } else if (mCurrentGear == 1) {
        mCurrentGear = -1;
    }
}

void MonsterCarDynamics::setAutomatic(bool automatic) noexcept
{
    mConfig.transmission.isAutomatic = automatic;
}

[[nodiscard]] bool MonsterCarDynamics::isAutomatic() const noexcept
{
    return mConfig.transmission.isAutomatic;
}

[[nodiscard]] double MonsterCarDynamics::getEngineRPM() const noexcept
{
    return mEngineRPM;
}

[[nodiscard]] double MonsterCarDynamics::getEngineTorque() const noexcept
{
    return mEngineTorque;
}

[[nodiscard]] double MonsterCarDynamics::getDriveForce() const noexcept
{
    return mDriveForce;
}

[[nodiscard]] double MonsterCarDynamics::getFrontAxleWeight() const noexcept
{
    return mFrontAxleWeight;
}

[[nodiscard]] double MonsterCarDynamics::getRearAxleWeight() const noexcept
{
    return mRearAxleWeight;
}

[[nodiscard]] double MonsterCarDynamics::getSteerAngle() const noexcept
{
    return mSteerAngle;
}

[[nodiscard]] double MonsterCarDynamics::getSpeed() const noexcept
{
    return mSpeed;
}

[[nodiscard]] double MonsterCarDynamics::getAcceleration() const noexcept
{
    return mAcceleration;
}

[[nodiscard]] bool MonsterCarDynamics::isDrifting() const noexcept
{
    const double rearLateralSlip { 0.5
        * (std::abs(mWheels[kRearLeft].lateralVelocity) + std::abs(mWheels[kRearRight].lateralVelocity)) };
    const bool rearSaturated { mWheels[kRearLeft].isSaturated || mWheels[kRearRight].isSaturated };
    return (rearSaturated || rearLateralSlip > 0.8) && (std::abs(mSpeed) > 1.0);
}

[[nodiscard]] const MonsterWheelTelemetry& MonsterCarDynamics::getWheelTelemetry(std::size_t index) const noexcept
{
    if (index >= kWheelCount) {
        return mWheels[0];
    }
    return mWheels[index];
}

void MonsterCarDynamics::setSurfaceModifier(double gripMultiplier, double dragMultiplier) noexcept
{
    mGlobalGripModifier = std::max(0.01, gripMultiplier);
    mGlobalDragModifier = std::max(0.01, dragMultiplier);
}

void MonsterCarDynamics::setWheelSurfaceModifier(std::size_t wheelIndex, double gripMultiplier) noexcept
{
    if (wheelIndex < kWheelCount) {
        mWheelGripModifiers[wheelIndex] = std::max(0.01, gripMultiplier);
    }
}

void MonsterCarDynamics::triggerJump(double verticalSpeed) noexcept
{
    if (!isAirborne()) {
        mFlightVz = (verticalSpeed > 0.0) ? verticalSpeed : mConfig.jumpVerticalSpeed;
        mFlightZ = 0.01;
    }
}

[[nodiscard]] double MonsterCarDynamics::getFlightHeight() const noexcept
{
    return mFlightZ;
}

void MonsterCarDynamics::updateWheelLayout() noexcept
{
    const double b { mConfig.distCgToFront };
    const double c { mConfig.distCgToRear };
    const double halfTrack { 0.5 * mConfig.trackWidth };

    mWheels[kFrontLeft].localOffset = math::Vec2 { b, halfTrack };
    mWheels[kFrontRight].localOffset = math::Vec2 { b, -halfTrack };
    mWheels[kRearLeft].localOffset = math::Vec2 { -c, halfTrack };
    mWheels[kRearRight].localOffset = math::Vec2 { -c, -halfTrack };

    mWheels[kFrontLeft].isSteering = true;
    mWheels[kFrontRight].isSteering = true;
    mWheels[kRearLeft].isSteering = false;
    mWheels[kRearRight].isSteering = false;

    switch (mConfig.driveType) {
    case MonsterDriveType::FrontWheelDrive:
        mWheels[kFrontLeft].isDriven = true;
        mWheels[kFrontRight].isDriven = true;
        mWheels[kRearLeft].isDriven = false;
        mWheels[kRearRight].isDriven = false;
        break;
    case MonsterDriveType::RearWheelDrive:
        mWheels[kFrontLeft].isDriven = false;
        mWheels[kFrontRight].isDriven = false;
        mWheels[kRearLeft].isDriven = true;
        mWheels[kRearRight].isDriven = true;
        break;
    case MonsterDriveType::AllWheelDrive:
        mWheels[kFrontLeft].isDriven = true;
        mWheels[kFrontRight].isDriven = true;
        mWheels[kRearLeft].isDriven = true;
        mWheels[kRearRight].isDriven = true;
        break;
    }
}

void MonsterCarDynamics::updateSteering(double dt) noexcept
{
    const double clampedInput { std::clamp(mInput.steer, -1.0, 1.0) };
    const double targetAngle { clampedInput * mConfig.maxSteerAngle };
    const double maxDelta { mConfig.steerSpeed * dt };

    const double delta { targetAngle - mSteerAngle };
    mSteerAngle += std::clamp(delta, -maxDelta, maxDelta);

    mWheels[kFrontLeft].steerAngle = mSteerAngle;
    mWheels[kFrontRight].steerAngle = mSteerAngle;
    mWheels[kRearLeft].steerAngle = 0.0;
    mWheels[kRearRight].steerAngle = 0.0;
}

void MonsterCarDynamics::updateFlight(double dt) noexcept
{
    if (mFlightZ <= 0.0 && mFlightVz <= 0.0) {
        mFlightZ = 0.0;
        mFlightVz = 0.0;
        return;
    }

    mFlightVz -= mConfig.gravityZ * dt;
    mFlightZ += mFlightVz * dt;

    if (mFlightZ <= 0.0) {
        mFlightZ = 0.0;
        mFlightVz = 0.0;
    }
}

void MonsterCarDynamics::updateTransmission(double dt) noexcept
{
    (void)dt;
    if (!mConfig.transmission.isAutomatic) {
        return;
    }

    const std::int32_t maxGear { static_cast<std::int32_t>(mConfig.transmission.gearRatios.size()) };

    if (mCurrentGear >= 1) {
        if (mEngineRPM >= mConfig.engine.upshiftRPM && mCurrentGear < maxGear) {
            ++mCurrentGear;
        } else if (mEngineRPM <= mConfig.engine.downshiftRPM && mCurrentGear > 1) {
            --mCurrentGear;
        } else if (mSpeed < 0.25 && mInput.throttle < -0.1) {
            mCurrentGear = -1;
        }
    } else if (mCurrentGear == -1) {
        if (mSpeed > -0.25 && mInput.throttle > 0.1) {
            mCurrentGear = 1;
        }
    }
}

[[nodiscard]] double MonsterCarDynamics::calculateTorque(double rpm) const noexcept
{
    const double range { mConfig.engine.redlineRPM - mConfig.engine.idleRPM };
    const double safeRange { (range > 100.0) ? range : 5000.0 };
    const double normalizedDev { (rpm - mConfig.engine.peakTorqueRPM) / safeRange };

    const double torqueFactor { 1.0 - (0.75 * normalizedDev * normalizedDev) };
    const double clampedFactor { std::clamp(torqueFactor, 0.4, 1.0) };

    return mConfig.engine.peakTorque * clampedFactor;
}

} // namespace vehicle_sim::dynamics
