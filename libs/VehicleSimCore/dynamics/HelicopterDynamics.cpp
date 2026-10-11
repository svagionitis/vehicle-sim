#include "HelicopterDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

HelicopterDynamics::HelicopterDynamics() noexcept
    : HelicopterDynamics(HelicopterConfig { })
{
}

HelicopterDynamics::HelicopterDynamics(const HelicopterConfig& config) noexcept
    : mConfig { config }
    , mRotorRpm { config.maxRotorRpm }
{
    mTelemetry.rotorSpeed = mRotorRpm;
    mTelemetry.altitude = mInput.groundDistance;
}

void HelicopterDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Flight kinematics and altitude tracking
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double speed { worldVel.length() };
    const double forwardSpeed { localVel.x };
    const double climbRate { worldVel.y };
    const double omega { body.getAngularVelocity() };
    const double altitude { std::max(0.0, mInput.groundDistance) };

    mTelemetry.forwardSpeed = forwardSpeed;
    mTelemetry.climbRate = climbRate;
    mTelemetry.altitude = altitude;

    // 2. Turboshaft engine spool and autorotation
    if (mInput.enginePower) {
        const double rpmDelta { mConfig.maxRotorRpm - mRotorRpm };
        const double maxDelta { mConfig.rotorSpoolRate * dt };
        mRotorRpm += std::clamp(rpmDelta, -maxDelta, maxDelta);
        mTelemetry.isAutorotating = false;
    } else {
        // Engine disengaged: autorotation windmilling driven by upward airflow
        const double autoTargetRpm { std::clamp(-climbRate * 45.0, 80.0, mConfig.maxRotorRpm * 0.8) };
        const double autoDelta { autoTargetRpm - mRotorRpm };
        const double maxAutoDelta { mConfig.rotorSpoolRate * 0.5 * dt };
        mRotorRpm += std::clamp(autoDelta, -maxAutoDelta, maxAutoDelta);
        mTelemetry.isAutorotating = true;
    }
    mTelemetry.rotorSpeed = mRotorRpm;

    const double rpmRatio { std::clamp(mRotorRpm / std::max(1.0, mConfig.maxRotorRpm), 0.0, 1.2) };
    const double rpmFactor { rpmRatio * rpmRatio };

    // 3. Main rotor collective lift
    const double collective { std::clamp(mInput.collective, 0.0, 1.0) };
    const double baseThrust { collective * mConfig.maxCollectiveThrust * rpmFactor };

    // 4. In-Ground Effect (IGE) cushion
    bool isIge { false };
    double igeFactor { 1.0 };
    if (altitude < mConfig.inGroundEffectHeight && altitude > 0.0) {
        const double normAlt { altitude / mConfig.inGroundEffectHeight };
        igeFactor = 1.0 + (mConfig.igeMaxBoostRatio * (1.0 - normAlt) * (1.0 - normAlt));
        isIge = true;
    }
    mTelemetry.isGroundEffect = isIge;

    // 5. Effective Translational Lift (ETL)
    bool isEtl { false };
    double etlFactor { 1.0 };
    if (std::abs(forwardSpeed) > mConfig.etlSpeedThreshold) {
        const double excessSpeed { std::abs(forwardSpeed) - mConfig.etlSpeedThreshold };
        etlFactor = 1.0 + (0.15 * std::min(1.0, excessSpeed / 12.0));
        isEtl = true;
    }
    mTelemetry.isTranslationalLift = isEtl;

    const double totalThrust { baseThrust * igeFactor * etlFactor };
    mTelemetry.rotorThrust = totalThrust;

    // Apply primary rotor lift along local vertical mast axis
    body.applyForce(body.toWorldVec(math::Vec2 { 0.0, totalThrust }));

    // 6. Cyclic pitch stick tilt moments and horizontal vectoring
    const double cyclicCmd { std::clamp(mInput.cyclicPitch, -1.0, 1.0) };
    const double cyclicTorque { -cyclicCmd * mConfig.cyclicPitchAuthority * rpmRatio };
    const double horizontalThrust { cyclicCmd * totalThrust * 0.15 };

    body.applyForce(body.toWorldVec(math::Vec2 { horizontalThrust, 0.0 }));
    body.applyTorque(cyclicTorque);

    // 7. Reactive main rotor torque and tail rotor anti-torque balance
    const double mainReactionTorque { mConfig.mainReactionTorque * rpmRatio };
    const double rudderCmd { std::clamp(mInput.rudder, -1.0, 1.0) };
    const double tailRotorTorque { (rudderCmd * mConfig.tailRotorMaxTorque * rpmRatio) - mainReactionTorque };
    mTelemetry.tailTorque = tailRotorTorque;

    const double netYawTorque { mainReactionTorque + tailRotorTorque };
    body.applyTorque(netYawTorque);

    // 8. Fuselage parasite drag and aerodynamic angular damping
    if (speed > 1e-3) {
        const math::Vec2 parasiteDrag { (worldVel / speed) * (-mConfig.fuselageDrag * speed * speed) };
        body.applyForce(parasiteDrag);
    }
    body.applyTorque(-mConfig.angularDamping * omega);

    // 9. Ambient gravity acceleration
    if (body.getMass() > 0.0) {
        body.applyForce(mConfig.gravity * body.getMass());
    }
}

void HelicopterDynamics::reset() noexcept
{
    mInput = HelicopterInput { };
    mTelemetry = HelicopterTelemetry { };
    mRotorRpm = mConfig.maxRotorRpm;
    mTelemetry.rotorSpeed = mRotorRpm;
    mTelemetry.altitude = mInput.groundDistance;
}

bool HelicopterDynamics::isAirborne() const noexcept
{
    return (mTelemetry.altitude > 0.5);
}

const HelicopterConfig& HelicopterDynamics::getConfig() const noexcept
{
    return mConfig;
}

void HelicopterDynamics::setConfig(const HelicopterConfig& config) noexcept
{
    mConfig = config;
}

const HelicopterInput& HelicopterDynamics::getInput() const noexcept
{
    return mInput;
}

void HelicopterDynamics::setInput(const HelicopterInput& input) noexcept
{
    mInput = input;
}

const HelicopterTelemetry& HelicopterDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool HelicopterDynamics::isAutorotating() const noexcept
{
    return mTelemetry.isAutorotating;
}

double HelicopterDynamics::getRotorSpeed() const noexcept
{
    return mTelemetry.rotorSpeed;
}

double HelicopterDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardSpeed;
}

} // namespace vehicle_sim::dynamics
