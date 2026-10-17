#include "WatercraftDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/WaveGrid2D.h"

namespace vehicle_sim::dynamics {

WatercraftDynamics::WatercraftDynamics() noexcept
    : WatercraftDynamics(WatercraftConfig { })
{
}

WatercraftDynamics::WatercraftDynamics(const WatercraftConfig& config) noexcept
    : mConfig { config }
{
}

void WatercraftDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Compute hull world and local kinematics
    const math::Vec2 worldPos { body.getPosition() };
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double speed { worldVel.length() };
    const double forwardSpeed { localVel.x };
    const double omega { body.getAngularVelocity() };
    const double orientation { body.getOrientation() };

    mTelemetry.forwardVelocity = forwardSpeed;
    mTelemetry.lateralVelocity = localVel.y;
    mTelemetry.speed = speed;

    double activeWaterline { mConfig.waterlineY };
    double waveSlope { 0.0 };
    if (mWaveGrid != nullptr) {
        const terrain::WaveSample2D wave { mWaveGrid->getSample(worldPos) };
        activeWaterline = wave.elevation;
        waveSlope = wave.slopeAngle;

        terrain::WaterWakeStamp2D stamp {};
        stamp.position = worldPos;
        stamp.heading = orientation;
        stamp.length = mConfig.length;
        stamp.beam = mConfig.beam;
        stamp.draft = mConfig.draft;
        stamp.velocity = worldVel;
        stamp.verticalVelocity = worldVel.y;
        stamp.displacement = (body.getMass() > 0.0) ? body.getMass() : mConfig.mass;
        mWaveGrid->applyWakeStamp(stamp);
    }

    // 2. Submerged draft depth and immersion ratio
    const double submergedDepth { std::max(0.0, activeWaterline - worldPos.y) };
    mTelemetry.draftDepth = submergedDepth;

    const double draftRef { std::max(0.1, mConfig.draft) };
    const double submergenceRatio { std::clamp(submergedDepth / draftRef, 0.0, 1.5) };
    const bool inWater { submergedDepth > 0.01 };
    mTelemetry.isSubmerged = inWater;

    // 3. Archimedes hydrostatic buoyancy and vertical heave damping
    double buoyancy { 0.0 };
    if (inWater && body.getMass() > 0.0) {
        const double fullBuoyancy { body.getMass() * 9.80665 };
        buoyancy = submergenceRatio * fullBuoyancy;
        const double heaveDamper { -mConfig.heaveDamping * worldVel.y * submergenceRatio };
        body.applyForce(math::Vec2 { 0.0, buoyancy + heaveDamper });
    }
    mTelemetry.buoyancyForce = buoyancy;

    if (body.getMass() > 0.0) {
        body.applyForce(mConfig.gravity * body.getMass());
    }

    // 4. Hydrodynamic Planing Lift and Drag reduction
    double planingLift { 0.0 };
    bool isPlaningMode { false };
    double dragCoeff { mConfig.displacementDragCoeff };

    if (inWater && forwardSpeed > mConfig.planingThresholdSpeed) {
        const double excessSpeed { forwardSpeed - mConfig.planingThresholdSpeed };
        const double planRatio { std::clamp(excessSpeed / 5.0, 0.0, 1.0) };
        planingLift = planRatio * mConfig.maxPlaningLift * submergenceRatio;
        dragCoeff = ((1.0 - planRatio) * mConfig.displacementDragCoeff) + (planRatio * mConfig.planingDragCoeff);
        isPlaningMode = (planRatio > 0.5);
        body.applyForce(math::Vec2 { 0.0, planingLift });
    }
    mTelemetry.planingLift = planingLift;
    mTelemetry.isPlaning = isPlaningMode;

    // 5. Hydrodynamic form drag and keel chine lateral resistance
    double totalDrag { 0.0 };
    if (inWater && speed > 1e-3) {
        totalDrag = dragCoeff * speed * speed * submergenceRatio;
        const math::Vec2 waterDrag { (worldVel / speed) * (-totalDrag) };
        body.applyForce(waterDrag);

        const double latShear { std::clamp(-localVel.y * mConfig.lateralResistance, -25000.0, 25000.0)
            * submergenceRatio };
        body.applyForce(body.toWorldVec(math::Vec2 { 0.0, latShear }));
    }
    mTelemetry.dragForce = totalDrag;

    // 6. Water rudder steering servo slewing
    const double targetRudder { std::clamp(mInput.rudder, -1.0, 1.0) * mConfig.maxRudderAngle };
    if (mConfig.rudderSlewRate <= 0.0) {
        mRudderAngle = targetRudder;
    } else {
        const double deltaRudder { targetRudder - mRudderAngle };
        const double maxDelta { mConfig.rudderSlewRate * dt };
        mRudderAngle += std::clamp(deltaRudder, -maxDelta, maxDelta);
    }
    mTelemetry.rudderAngle = mRudderAngle;

    // 7. Propeller propulsion thrust vectoring
    const double throttle { std::clamp(mInput.throttle, -1.0, 1.0) };
    const double waterCoupling { inWater ? 1.0 : 0.05 }; // Propeller ventilates/spins out if airborne
    const double thrustMag { throttle * mConfig.maxPropellerThrust * waterCoupling };
    mTelemetry.thrustForce = thrustMag;

    if (std::abs(thrustMag) > 1e-4) {
        const math::Vec2 thrustDirLocal { std::cos(mRudderAngle), -std::sin(mRudderAngle) };
        const math::Vec2 thrustLocal { thrustDirLocal * thrustMag };
        body.applyForceAtLocalPoint(body.toWorldVec(thrustLocal), mConfig.rudderOffset);
    }

    // 8. Metacentric pitch stability in water
    if (inWater) {
        const double deltaAngle { orientation - waveSlope };
        const double restoreTorque { -std::sin(deltaAngle) * mConfig.metacentricStiffness * submergenceRatio };
        const double pitchDampTorque { -mConfig.pitchDamping * omega * submergenceRatio };
        body.applyTorque(restoreTorque + pitchDampTorque);

        if (std::abs(waveSlope) > 1e-4 && body.getMass() > 0.0) {
            const double surfingThrust { body.getMass() * 9.80665 * std::sin(waveSlope) * submergenceRatio };
            body.applyForce(body.toWorldVec(math::Vec2 { surfingThrust, 0.0 }));
        }
    }
}

void WatercraftDynamics::reset() noexcept
{
    mInput = WatercraftInput { };
    mTelemetry = WatercraftTelemetry { };
    mRudderAngle = 0.0;
}

bool WatercraftDynamics::isAirborne() const noexcept
{
    return !mTelemetry.isSubmerged;
}

const WatercraftConfig& WatercraftDynamics::getConfig() const noexcept
{
    return mConfig;
}

void WatercraftDynamics::setConfig(const WatercraftConfig& config) noexcept
{
    mConfig = config;
}

const WatercraftInput& WatercraftDynamics::getInput() const noexcept
{
    return mInput;
}

void WatercraftDynamics::setInput(const WatercraftInput& input) noexcept
{
    mInput = input;
}

const WatercraftTelemetry& WatercraftDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

bool WatercraftDynamics::isPlaning() const noexcept
{
    return mTelemetry.isPlaning;
}

bool WatercraftDynamics::isSubmerged() const noexcept
{
    return mTelemetry.isSubmerged;
}

double WatercraftDynamics::getSpeed() const noexcept
{
    return mTelemetry.forwardVelocity;
}

double WatercraftDynamics::getRudderAngle() const noexcept
{
    return mTelemetry.rudderAngle;
}

void WatercraftDynamics::attachWaveGrid(terrain::WaveGrid2D* waveGrid) noexcept
{
    mWaveGrid = waveGrid;
}

terrain::WaveGrid2D* WatercraftDynamics::getWaveGrid() const noexcept
{
    return mWaveGrid;
}

} // namespace vehicle_sim::dynamics
