#include "ArcadeCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

ArcadeCarDynamics::ArcadeCarDynamics() noexcept
    : ArcadeCarDynamics(ArcadeCarConfig { })
{
}

ArcadeCarDynamics::ArcadeCarDynamics(const ArcadeCarConfig& config) noexcept
    : mConfig { config }
    , mNitroFuel { config.maxNitroDuration }
{
    updateWheelLayout();
}

void ArcadeCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    updateFlight(dt);
    updateSteering(dt);
    updateNitro(dt);

    const math::Vec2 bodyLocalVel { body.toLocalVec(body.getLinearVelocity()) };
    mForwardSpeed = bodyLocalVel.x;

    if (isAirborne()) {
        for (auto& wheel : mWheels) {
            wheel.force = math::Vec2::zero();
            wheel.velocity = math::Vec2::zero();
            wheel.longitudinalVelocity = 0.0;
            wheel.lateralVelocity = 0.0;
        }

        const math::Vec2 v { body.getLinearVelocity() };
        const double speed { v.length() };
        if (speed > 1e-4) {
            const math::Vec2 airDrag { -mConfig.linearDragCoefficient * speed * v };
            body.applyForce(airDrag);
        }
        return;
    }

    std::size_t drivenCount { 0 };
    for (const auto& wheel : mWheels) {
        if (wheel.isDriven) {
            ++drivenCount;
        }
    }
    const double drivenFactor { drivenCount > 0 ? (1.0 / static_cast<double>(drivenCount)) : 0.0 };

    double driveForcePerWheel { 0.0 };
    if (mInput.throttle > 0.0) {
        const double clampedThrottle { std::clamp(mInput.throttle, 0.0, 1.0) };
        driveForcePerWheel = clampedThrottle * mConfig.maxDriveForce * drivenFactor;
    } else if (mInput.throttle < 0.0) {
        const double clampedReverse { std::clamp(-mInput.throttle, 0.0, 1.0) };
        driveForcePerWheel = -clampedReverse * mConfig.maxReverseForce * drivenFactor;
    }

    const double clampedBrake { std::clamp(mInput.brake, 0.0, 1.0) };
    const double brakeForcePerWheel { clampedBrake * mConfig.maxBrakeForce * 0.25 };
    const bool isHandbrakeEngaged { mInput.handbrake == HandbrakeState::Engaged };

    for (std::size_t i { 0 }; i < kWheelCount; ++i) {
        WheelTelemetry& wheel { mWheels[i] };
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

        const double surfaceGrip { mGlobalGripModifier * mWheelGripModifiers[i] };
        wheel.surfaceGrip = surfaceGrip;

        double latGripCoeff { wheel.isSteering ? mConfig.frontLateralGrip : mConfig.rearLateralGrip };
        if (isHandbrakeEngaged && !wheel.isSteering) {
            latGripCoeff *= mConfig.handbrakeGripScale;
        }
        const double fLat { -latGripCoeff * vLat * surfaceGrip };

        double fLong { 0.0 };
        if (wheel.isDriven) {
            fLong += driveForcePerWheel * surfaceGrip;
        }

        const double fRollingResistance { -mConfig.rollingResistance * vLong * mGlobalDragModifier };
        fLong += fRollingResistance;

        if (brakeForcePerWheel > 0.0 && std::abs(vLong) > 1e-3) {
            const double brakeDirection { (vLong > 0.0) ? -1.0 : 1.0 };
            fLong += brakeDirection * brakeForcePerWheel * surfaceGrip;
        }

        if (isHandbrakeEngaged && !wheel.isSteering && std::abs(vLong) > 1e-3) {
            const double handbrakeDirection { (vLong > 0.0) ? -1.0 : 1.0 };
            fLong += handbrakeDirection * (mConfig.handbrakeBrakeForce * 0.5) * surfaceGrip;
        }

        const math::Vec2 wheelForceWorld { (uLong * fLong) + (uLat * fLat) };
        wheel.force = wheelForceWorld;

        body.applyForceAtLocalPoint(wheelForceWorld, wheel.localOffset);
    }

    const math::Vec2 linVel { body.getLinearVelocity() };
    const double currentSpeed { linVel.length() };
    if (currentSpeed > 1e-4) {
        const math::Vec2 dragForce { -mConfig.linearDragCoefficient * currentSpeed * linVel };
        body.applyForce(dragForce);
    }

    if (mNitroFiring) {
        const math::Vec2 headingWorld { body.toWorldVec(math::Vec2 { 1.0, 0.0 }) };
        const math::Vec2 nitroThrust { headingWorld * mConfig.nitroForce };
        body.applyForce(nitroThrust);
    }
}

void ArcadeCarDynamics::reset() noexcept
{
    mInput = ArcadeCarInput { };
    mSteerAngle = 0.0;
    mNitroFuel = mConfig.maxNitroDuration;
    mFlightZ = 0.0;
    mFlightVz = 0.0;
    mForwardSpeed = 0.0;
    mNitroFiring = false;
    mGlobalGripModifier = 1.0;
    mGlobalDragModifier = 1.0;
    mWheelGripModifiers = { 1.0, 1.0, 1.0, 1.0 };

    for (auto& wheel : mWheels) {
        wheel.steerAngle = 0.0;
        wheel.force = math::Vec2::zero();
        wheel.velocity = math::Vec2::zero();
        wheel.longitudinalVelocity = 0.0;
        wheel.lateralVelocity = 0.0;
        wheel.surfaceGrip = 1.0;
    }
}

[[nodiscard]] bool ArcadeCarDynamics::isAirborne() const noexcept
{
    return mFlightZ > 0.0;
}

void ArcadeCarDynamics::setInput(const ArcadeCarInput& input) noexcept
{
    mInput = input;
}

[[nodiscard]] const ArcadeCarInput& ArcadeCarDynamics::getInput() const noexcept
{
    return mInput;
}

[[nodiscard]] const ArcadeCarConfig& ArcadeCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

void ArcadeCarDynamics::setConfig(const ArcadeCarConfig& config) noexcept
{
    mConfig = config;
    updateWheelLayout();
}

void ArcadeCarDynamics::setSurfaceModifier(double gripMultiplier, double dragMultiplier) noexcept
{
    mGlobalGripModifier = std::max(0.0, gripMultiplier);
    mGlobalDragModifier = std::max(0.0, dragMultiplier);
}

void ArcadeCarDynamics::setWheelSurfaceModifier(std::size_t wheelIndex, double gripMultiplier) noexcept
{
    if (wheelIndex < kWheelCount) {
        mWheelGripModifiers[wheelIndex] = std::max(0.0, gripMultiplier);
    }
}

void ArcadeCarDynamics::triggerJump(double verticalSpeed) noexcept
{
    const double initialVz { verticalSpeed > 0.0 ? verticalSpeed : mConfig.jumpVerticalSpeed };
    mFlightVz = initialVz;
    mFlightZ = 0.01;
}

[[nodiscard]] double ArcadeCarDynamics::getFlightHeight() const noexcept
{
    return mFlightZ;
}

[[nodiscard]] double ArcadeCarDynamics::getNitroFuel() const noexcept
{
    return mNitroFuel;
}

[[nodiscard]] double ArcadeCarDynamics::getSteerAngle() const noexcept
{
    return mSteerAngle;
}

[[nodiscard]] double ArcadeCarDynamics::getSpeed() const noexcept
{
    return mForwardSpeed;
}

[[nodiscard]] const WheelTelemetry& ArcadeCarDynamics::getWheelTelemetry(std::size_t index) const noexcept
{
    return mWheels[index % kWheelCount];
}

[[nodiscard]] bool ArcadeCarDynamics::isNitroActive() const noexcept
{
    return mNitroFiring;
}

[[nodiscard]] bool ArcadeCarDynamics::isDrifting() const noexcept
{
    const double rearLateralSlip { 0.5
        * (std::abs(mWheels[kRearLeft].lateralVelocity) + std::abs(mWheels[kRearRight].lateralVelocity)) };
    return (rearLateralSlip > 0.75) && (std::abs(mForwardSpeed) > 1.0);
}

void ArcadeCarDynamics::updateWheelLayout() noexcept
{
    const double halfWheelbase { 0.5 * mConfig.wheelbase };
    const double halfTrackWidth { 0.5 * mConfig.trackWidth };

    mWheels[kFrontLeft].localOffset = math::Vec2 { halfWheelbase, halfTrackWidth };
    mWheels[kFrontRight].localOffset = math::Vec2 { halfWheelbase, -halfTrackWidth };
    mWheels[kRearLeft].localOffset = math::Vec2 { -halfWheelbase, halfTrackWidth };
    mWheels[kRearRight].localOffset = math::Vec2 { -halfWheelbase, -halfTrackWidth };

    mWheels[kFrontLeft].isSteering = true;
    mWheels[kFrontRight].isSteering = true;
    mWheels[kRearLeft].isSteering = false;
    mWheels[kRearRight].isSteering = false;

    switch (mConfig.driveType) {
    case DriveType::FrontWheelDrive:
        mWheels[kFrontLeft].isDriven = true;
        mWheels[kFrontRight].isDriven = true;
        mWheels[kRearLeft].isDriven = false;
        mWheels[kRearRight].isDriven = false;
        break;
    case DriveType::RearWheelDrive:
        mWheels[kFrontLeft].isDriven = false;
        mWheels[kFrontRight].isDriven = false;
        mWheels[kRearLeft].isDriven = true;
        mWheels[kRearRight].isDriven = true;
        break;
    case DriveType::AllWheelDrive:
        mWheels[kFrontLeft].isDriven = true;
        mWheels[kFrontRight].isDriven = true;
        mWheels[kRearLeft].isDriven = true;
        mWheels[kRearRight].isDriven = true;
        break;
    }
}

void ArcadeCarDynamics::updateSteering(double dt) noexcept
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

void ArcadeCarDynamics::updateFlight(double dt) noexcept
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

void ArcadeCarDynamics::updateNitro(double dt) noexcept
{
    if (mInput.nitro == NitroState::Active && mNitroFuel > 0.0 && !isAirborne()) {
        mNitroFiring = true;
        mNitroFuel = std::max(0.0, mNitroFuel - dt);
    } else {
        mNitroFiring = false;
        if (mInput.nitro == NitroState::Inactive && mNitroFuel < mConfig.maxNitroDuration) {
            mNitroFuel = std::min(mConfig.maxNitroDuration, mNitroFuel + (mConfig.nitroRechargeRate * dt));
        }
    }
}

} // namespace vehicle_sim::dynamics
