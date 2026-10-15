#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

MotionLabCarDynamics::MotionLabCarDynamics() noexcept
    : MotionLabCarDynamics(MotionLabCarConfig { })
{
}

MotionLabCarDynamics::MotionLabCarDynamics(const MotionLabCarConfig& config) noexcept
    : mConfig { config }
{
    updateWheelLayout();
}

void MotionLabCarDynamics::updateWheelLayout() noexcept
{
    // Tires inset 0.3m from shell boundaries (Length: 4.0m -> x = 1.35m, Width: 2.0m -> y = 0.70m)
    mTires[kFrontLeft].position = math::Vec2 { 1.35, 0.70 };
    mTires[kFrontLeft].steered = true;

    mTires[kFrontRight].position = math::Vec2 { 1.35, -0.70 };
    mTires[kFrontRight].steered = true;

    mTires[kRearLeft].position = math::Vec2 { -1.35, 0.70 };
    mTires[kRearLeft].steered = false;

    mTires[kRearRight].position = math::Vec2 { -1.35, -0.70 };
    mTires[kRearRight].steered = false;

    reset();
}

void MotionLabCarDynamics::reset() noexcept
{
    mRearGripLoss = 0.0;
    mIsSkidding = false;
    for (auto& tire : mTires) {
        tire.skidding = false;
        tire.lateralSpeed = 0.0;
        tire.rollingSpeed = 0.0;
        tire.steerAngle = 0.0;
        tire.force = math::Vec2::zero();
        tire.worldPosition = math::Vec2::zero();
    }
}

bool MotionLabCarDynamics::isAirborne() const noexcept
{
    return false;
}

void MotionLabCarDynamics::setInput(const MotionLabCarInput& input) noexcept
{
    mInput = input;
}

const MotionLabCarInput& MotionLabCarDynamics::getInput() const noexcept
{
    return mInput;
}

void MotionLabCarDynamics::setConfig(const MotionLabCarConfig& config) noexcept
{
    mConfig = config;
    updateWheelLayout();
}

const MotionLabCarConfig& MotionLabCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

const std::array<MotionLabTireTelemetry, MotionLabCarDynamics::kWheelCount>&
MotionLabCarDynamics::getTires() const noexcept
{
    return mTires;
}

bool MotionLabCarDynamics::isSkidding() const noexcept
{
    return mIsSkidding;
}

double MotionLabCarDynamics::getRearGripLoss() const noexcept
{
    return mRearGripLoss;
}

void MotionLabCarDynamics::setEngineForce(double force) noexcept
{
    mConfig.engineForce = std::max(0.0, force);
}

void MotionLabCarDynamics::setHandbrakeStrength(double strength) noexcept
{
    mConfig.handbrakeStrength = std::clamp(strength, 0.0, 1.0);
}

void MotionLabCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // Handbrake: engages immediately; recovers linearly upon release
    if (mInput.handbrake) {
        mRearGripLoss = 1.0;
    } else {
        const double decayRate { mConfig.gripRecoveryTime > 0.0 ? (dt / mConfig.gripRecoveryTime) : 1.0 };
        mRearGripLoss = std::max(0.0, mRearGripLoss - decayRate);
    }

    const double angle { body.getOrientation() };
    const math::Vec2 forward { std::cos(angle), std::sin(angle) };

    // Reverse throttle scale: brakes at full strength if moving forward, attenuated when stopped/reversing
    const double forwardSpeed { body.getLinearVelocity().dot(forward) };
    const double reverseScale { (mInput.throttle < 0.0 && forwardSpeed <= 0.0) ? mConfig.reverseScale : 1.0 };

    const math::Vec2 driveForce { forward * (mInput.throttle * mConfig.engineForce * reverseScale) };
    body.applyForce(driveForce);

    mIsSkidding = false;

    for (auto& tire : mTires) {
        tire.steerAngle = tire.steered ? (mInput.steer * mConfig.maxSteerAngle) : 0.0;
        tire.worldPosition = body.toWorldPoint(tire.position);

        const math::Vec2 velocity { body.getVelocityAtPoint(tire.worldPosition) };
        const double wheelHeading { angle + tire.steerAngle };

        const math::Vec2 rolling { std::cos(wheelHeading), std::sin(wheelHeading) };
        const math::Vec2 lateral { -rolling.y, rolling.x };

        const double rollingSpeed { velocity.dot(rolling) };
        const double lateralSpeed { velocity.dot(lateral) };

        tire.rollingSpeed = rollingSpeed;
        tire.lateralSpeed = std::abs(lateralSpeed);

        const bool isRear { !tire.steered };

        // Dual-threshold hysteresis for rear skidmark generation
        const double skidThreshold { tire.skidding ? 0.8 : 1.5 };
        tire.skidding = isRear && (tire.lateralSpeed * 0.7 > skidThreshold);
        if (tire.skidding) {
            mIsSkidding = true;
        }

        double rollingRes { mConfig.rollingResistance };
        if (isRear && mInput.handbrake) {
            rollingRes *= (1.0 + mConfig.handbrakeStrength * (mConfig.handbrakeRollingMult - 1.0));
        }

        double lateralRes { mConfig.lateralResistance };
        if (isRear) {
            lateralRes *= (1.0 - mRearGripLoss * mConfig.handbrakeStrength * (1.0 - mConfig.handbrakeGrip));
        }

        const math::Vec2 rollingForce { rolling * (-rollingRes * rollingSpeed) };
        const math::Vec2 lateralForce { lateral * (-lateralRes * lateralSpeed) };
        const math::Vec2 totalTireForce { rollingForce + lateralForce };

        tire.force = totalTireForce;
        body.applyForceAtPoint(totalTireForce, tire.worldPosition);
    }
}

std::array<math::Vec2, 4> MotionLabCarDynamics::computeWorldPoints(const physics::RigidBody2D& body) const noexcept
{
    const double halfLength { mConfig.length * 0.5 };
    const double halfWidth { mConfig.width * 0.5 };

    return std::array<math::Vec2, 4> {{
        body.toWorldPoint(math::Vec2 { -halfLength, -halfWidth }),
        body.toWorldPoint(math::Vec2 { halfLength, -halfWidth }),
        body.toWorldPoint(math::Vec2 { halfLength, halfWidth }),
        body.toWorldPoint(math::Vec2 { -halfLength, halfWidth })
    }};
}

math::Vec2 MotionLabCarDynamics::computeSupportPoint(const physics::RigidBody2D& body,
                                                    const math::Vec2& normal) const noexcept
{
    const std::array<math::Vec2, 4> points { computeWorldPoints(body) };

    double minProj { points[0].dot(normal) };
    for (std::size_t i { 1 }; i < points.size(); ++i) {
        minProj = std::min(minProj, points[i].dot(normal));
    }

    math::Vec2 sum { 0.0, 0.0 };
    double count { 0.0 };

    for (const auto& pt : points) {
        if (std::abs(pt.dot(normal) - minProj) < 1e-8) {
            sum += pt;
            count += 1.0;
        }
    }

    return (count > 0.0) ? (sum / count) : points[0];
}

} // namespace vehicle_sim::dynamics
