#include "HovercraftDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

HovercraftDynamics::HovercraftDynamics() noexcept
    : HovercraftDynamics(HovercraftConfig { })
{
}

HovercraftDynamics::HovercraftDynamics(const HovercraftConfig& config) noexcept
    : mConfig { config }
{
    mTelemetry.altitude = config.hoverHeight;
}

void HovercraftDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Compute linear kinematics and drift metrics
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double speed { worldVel.length() };
    const double omega { body.getAngularVelocity() };

    mTelemetry.forwardSpeed = localVel.x;
    mTelemetry.driftSpeed = localVel.y;

    if (speed > 0.05) {
        mTelemetry.driftAngle = std::atan2(localVel.y, localVel.x);
    } else {
        mTelemetry.driftAngle = 0.0;
    }

    // 2. Rudder servo steering slewing
    const double targetRudder { std::clamp(mInput.steering, -1.0, 1.0) * mConfig.maxRudderAngle };
    if (mConfig.rudderSlewRate <= 0.0) {
        mRudderAngle = targetRudder;
    } else {
        const double deltaRudder { targetRudder - mRudderAngle };
        const double maxDelta { mConfig.rudderSlewRate * dt };
        mRudderAngle += std::clamp(deltaRudder, -maxDelta, maxDelta);
    }
    mTelemetry.rudderAngle = mRudderAngle;

    // 3. Air cushion vertical suspension simulation
    const double altitude { mInput.groundDistance };
    mTelemetry.altitude = altitude;

    double cushionForce { 0.0 };
    bool isHoveringState { false };
    bool isSkirtDragging { false };

    if (mInput.liftFanActive) {
        if (altitude < mConfig.hoverHeight) {
            const double compression { mConfig.hoverHeight - altitude };
            cushionForce = (mConfig.cushionStiffness * compression) - (mConfig.cushionDamping * worldVel.y);
            cushionForce = std::max(0.0, cushionForce);
        }
        isHoveringState = (altitude > 0.05);
        isSkirtDragging = (altitude <= 0.05);
    } else {
        isHoveringState = false;
        isSkirtDragging = true;
    }

    mTelemetry.cushionForce = cushionForce;
    mTelemetry.isHovering = isHoveringState;
    mTelemetry.isSkirtScraping = isSkirtDragging;

    if (cushionForce > 0.0) {
        body.applyForce(math::Vec2 { 0.0, cushionForce });
    }

    // Apply ambient gravity
    if (body.getMass() > 0.0) {
        body.applyForce(mConfig.gravity * body.getMass());
    }

    // 4. Ground/surface friction (drifting vs skirt dragging)
    double surfCoeff { isSkirtDragging ? mConfig.skirtFriction : mConfig.driftFriction };
    surfCoeff *= std::max(0.0, mConfig.surfaceFriction);

    if (speed > 0.01 && body.getMass() > 0.0) {
        const double normalForce { body.getMass() * 9.80665 };
        const double maxFriction { surfCoeff * normalForce };
        const double stoppingFriction { speed * body.getMass() / dt };
        const double appliedFriction { std::min(maxFriction, stoppingFriction) };
        const math::Vec2 frictionForce { (worldVel / speed) * (-appliedFriction) };
        body.applyForce(frictionForce);
    }

    // Lateral skirt shear resistance
    const double latResistance { -localVel.y * mConfig.lateralGrip * (isSkirtDragging ? 3.0 : 1.0) };
    body.applyForce(body.toWorldVec(math::Vec2 { 0.0, latResistance }));

    // 5. Vectored rear propulsion thrust
    const double throttle { std::clamp(mInput.throttle, -1.0, 1.0) };
    double thrustMagnitude { 0.0 };

    if (throttle >= 0.0) {
        thrustMagnitude = throttle * mConfig.maxThrust;
    } else {
        thrustMagnitude = throttle * mConfig.maxThrust * mConfig.reverseThrustRatio;
    }
    mTelemetry.thrustForce = thrustMagnitude;

    if (std::abs(thrustMagnitude) > 1e-4) {
        const math::Vec2 thrustDirLocal { std::cos(mRudderAngle), -std::sin(mRudderAngle) };
        const math::Vec2 thrustLocal { thrustDirLocal * thrustMagnitude };
        const math::Vec2 thrustWorld { body.toWorldVec(thrustLocal) };

        body.applyForceAtLocalPoint(thrustWorld, mConfig.thrusterOffset);
    }

    // 6. Quadratic aerodynamic air drag and rotational damping
    if (speed > 1e-4) {
        const math::Vec2 aeroDrag { worldVel * (-mConfig.airDrag * speed) };
        body.applyForce(aeroDrag);
    }

    const double rotDamping { -mConfig.angularDamping * omega };
    body.applyTorque(rotDamping);
}

void HovercraftDynamics::reset() noexcept
{
    mInput = HovercraftInput { };
    mTelemetry = HovercraftTelemetry { };
    mRudderAngle = 0.0;
    mTelemetry.altitude = mConfig.hoverHeight;
}

bool HovercraftDynamics::isAirborne() const noexcept
{
    return (mInput.groundDistance > mConfig.hoverHeight * 1.5);
}

const HovercraftConfig& HovercraftDynamics::getConfig() const noexcept
{
    return mConfig;
}

void HovercraftDynamics::setConfig(const HovercraftConfig& config) noexcept
{
    mConfig = config;
}

const HovercraftInput& HovercraftDynamics::getInput() const noexcept
{
    return mInput;
}

void HovercraftDynamics::setInput(const HovercraftInput& input) noexcept
{
    mInput = input;
}

void HovercraftDynamics::setSurfaceFriction(double friction) noexcept
{
    mConfig.surfaceFriction = std::max(0.0, friction);
}

const HovercraftTelemetry& HovercraftDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool HovercraftDynamics::isHovering() const noexcept
{
    return mTelemetry.isHovering;
}

double HovercraftDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardSpeed;
}

double HovercraftDynamics::getDriftSpeed() const noexcept
{
    return mTelemetry.driftSpeed;
}

double HovercraftDynamics::getRudderAngle() const noexcept
{
    return mTelemetry.rudderAngle;
}

} // namespace vehicle_sim::dynamics
