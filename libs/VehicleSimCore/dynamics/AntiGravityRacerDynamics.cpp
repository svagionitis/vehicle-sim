#include "AntiGravityRacerDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

AntiGravityRacerDynamics::AntiGravityRacerDynamics() noexcept
    : AntiGravityRacerDynamics(AntiGravityConfig { })
{
}

AntiGravityRacerDynamics::AntiGravityRacerDynamics(const AntiGravityConfig& config) noexcept
    : mConfig { config }
    , mBoostEnergy { config.maxBoostEnergy }
{
    mTelemetry.boostEnergy = mBoostEnergy;
    mTelemetry.rideHeight = mConfig.targetRideHeight;
    mTelemetry.isLevitating = true;
}

void AntiGravityRacerDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Kinematics analysis
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double speed { worldVel.length() };
    const double forwardSpeed { localVel.x };
    const double lateralSpeed { localVel.y };
    const double omega { body.getAngularVelocity() };

    // 2. Gravity application
    body.applyForce(mConfig.gravity * mConfig.mass);

    // 3. Magnetic levitation repulsion and damping
    const double trackDist { std::max(0.0, mInput.trackDistance) };
    const bool levitating { trackDist > 0.0 && trackDist < mConfig.maxLevitationRange };
    double suspensionForce { 0.0 };

    if (levitating) {
        const double compression { mConfig.targetRideHeight - trackDist };
        const double springForce { mConfig.suspensionStiffness * compression };
        const double damperForce { -mConfig.suspensionDamping * worldVel.y };
        suspensionForce = std::max(0.0, springForce + damperForce);
        body.applyForce(math::Vec2 { 0.0, suspensionForce });
    }

    // 4. Boost capacitor energy depletion and recharge
    bool boostActive { false };
    if (mInput.boost && mBoostEnergy > 0.0) {
        boostActive = true;
        mBoostEnergy = std::max(0.0, mBoostEnergy - dt);
    } else {
        mBoostEnergy = std::min(mConfig.maxBoostEnergy, mBoostEnergy + (mConfig.boostRechargeRate * dt));
    }

    // 5. Ion engine propulsion and reverse thrusters
    const double throttle { std::clamp(mInput.throttle, -1.0, 1.0) };
    double thrustMagnitude { (throttle >= 0.0)
        ? (throttle * mConfig.maxForwardThrust)
        : (throttle * mConfig.maxReverseThrust) };

    if (boostActive && throttle > 0.0) {
        thrustMagnitude *= mConfig.boostMultiplier;
    }

    body.applyForce(body.toWorldVec(math::Vec2 { thrustMagnitude, 0.0 }));

    // 6. Independent differential airbrakes
    const double leftBrake { std::clamp(mInput.leftAirbrake, 0.0, 1.0) };
    const double rightBrake { std::clamp(mInput.rightAirbrake, 0.0, 1.0) };
    const double airbrakeDeploy { leftBrake + rightBrake };
    double airbrakeDragForce { 0.0 };

    if (airbrakeDeploy > 0.0 && speed > 1e-3) {
        airbrakeDragForce = airbrakeDeploy * mConfig.airbrakeDragCoeff * speed * speed * 0.5;
        const math::Vec2 airbrakeRetard { -worldVel.normalized() * airbrakeDragForce };
        body.applyForce(airbrakeRetard);
    }

    // Differential airbrake yaw torque (Left airbrake pulls nose left, Right pulls nose right)
    const double airbrakeSpeedFactor { std::clamp(speed / 15.0, 0.0, 1.0) };
    const double airbrakeTorque { (leftBrake - rightBrake) * mConfig.airbrakeYawTorque * airbrakeSpeedFactor };

    // 7. Steering torque and angular damping
    const double steerCommand { std::clamp(mInput.steer, -1.0, 1.0) };
    const double levitationAuthority { levitating ? 1.0 : 0.25 };
    const double steerTorque { steerCommand * mConfig.steerTorque * levitationAuthority };
    const double dampingTorque { -mConfig.angularDamping * omega };
    const double totalYawTorque { steerTorque + airbrakeTorque + dampingTorque };

    body.applyTorque(totalYawTorque);

    // 8. Lateral guide rail traction and high-speed drift
    if (levitating) {
        const double lateralShear { std::clamp(-lateralSpeed * mConfig.lateralGripStiffness, -50000.0, 50000.0) };
        body.applyForce(body.toWorldVec(math::Vec2 { 0.0, lateralShear }));
    }

    // 9. Aerodynamic drag and supersonic shockwave drag
    if (speed > 1e-3) {
        double dragCoeff { mConfig.linearDragCoeff };
        if (speed > mConfig.supersonicThreshold) {
            dragCoeff *= mConfig.supersonicDragMult;
        }
        const double aeroDragMag { dragCoeff * speed * speed };
        body.applyForce(-worldVel.normalized() * aeroDragMag);
    }

    // 10. Dynamic roll banking angle
    const double bankAngle { -omega * mConfig.bankingCoupling * speed };

    // 11. Telemetry snapshot update
    mTelemetry.speed = speed;
    mTelemetry.forwardSpeed = forwardSpeed;
    mTelemetry.lateralSpeed = lateralSpeed;
    mTelemetry.rideHeight = trackDist;
    mTelemetry.suspensionForce = suspensionForce;
    mTelemetry.thrustForce = thrustMagnitude;
    mTelemetry.airbrakeDrag = airbrakeDragForce;
    mTelemetry.netYawTorque = totalYawTorque;
    mTelemetry.bankAngle = bankAngle;
    mTelemetry.boostEnergy = mBoostEnergy;
    mTelemetry.isBoostActive = boostActive;
    mTelemetry.isLevitating = levitating;
    mTelemetry.isDrifting = (std::abs(lateralSpeed) > mConfig.driftThreshold);
}

void AntiGravityRacerDynamics::reset() noexcept
{
    mInput = AntiGravityInput { };
    mBoostEnergy = mConfig.maxBoostEnergy;
    mTelemetry = AntiGravityTelemetry { };
    mTelemetry.boostEnergy = mBoostEnergy;
    mTelemetry.rideHeight = mConfig.targetRideHeight;
    mTelemetry.isLevitating = true;
}

bool AntiGravityRacerDynamics::isAirborne() const noexcept
{
    return !mTelemetry.isLevitating;
}

const AntiGravityConfig& AntiGravityRacerDynamics::getConfig() const noexcept
{
    return mConfig;
}

void AntiGravityRacerDynamics::setConfig(const AntiGravityConfig& config) noexcept
{
    mConfig = config;
}

const AntiGravityInput& AntiGravityRacerDynamics::getInput() const noexcept
{
    return mInput;
}

void AntiGravityRacerDynamics::setInput(const AntiGravityInput& input) noexcept
{
    mInput = input;
}

const AntiGravityTelemetry& AntiGravityRacerDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool AntiGravityRacerDynamics::isLevitating() const noexcept
{
    return mTelemetry.isLevitating;
}

bool AntiGravityRacerDynamics::isBoostActive() const noexcept
{
    return mTelemetry.isBoostActive;
}

bool AntiGravityRacerDynamics::isDrifting() const noexcept
{
    return mTelemetry.isDrifting;
}

double AntiGravityRacerDynamics::getRideHeight() const noexcept
{
    return mTelemetry.rideHeight;
}

double AntiGravityRacerDynamics::getBankAngle() const noexcept
{
    return mTelemetry.bankAngle;
}

double AntiGravityRacerDynamics::getSpeed() const noexcept
{
    return mTelemetry.speed;
}

double AntiGravityRacerDynamics::getBoostEnergy() const noexcept
{
    return mBoostEnergy;
}

} // namespace vehicle_sim::dynamics
