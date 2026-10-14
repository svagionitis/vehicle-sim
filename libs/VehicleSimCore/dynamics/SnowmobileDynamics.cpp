#include "libs/VehicleSimCore/dynamics/SnowmobileDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

SnowmobileDynamics::SnowmobileDynamics() noexcept
    : SnowmobileDynamics(SnowmobileConfig{}) {}

SnowmobileDynamics::SnowmobileDynamics(const SnowmobileConfig& config) noexcept
    : m_config{ config } {}

void SnowmobileDynamics::step(physics::RigidBody2D& body, double dt) noexcept {
    if (dt <= 0.0) {
        return;
    }

    const double totalMass { m_config.chassisMass + m_config.riderMass };
    const double gravity { 9.80665 };
    const double wheelbase { m_config.distFrontSkis + m_config.distRearTrack };
    const double halfStance { m_config.skiStanceWidth * 0.5 };

    const math::Vec2 bodyVel { body.getLinearVelocity() };
    const double bodyAngle { body.getOrientation() };
    const double yawRate { body.getAngularVelocity() };

    const math::Vec2 forwardHeading { std::cos(bodyAngle), std::sin(bodyAngle) };
    const math::Vec2 lateralHeading { -std::sin(bodyAngle), std::cos(bodyAngle) };

    const double forwardSpeed { bodyVel.dot(forwardHeading) };
    const double lateralSpeed { bodyVel.dot(lateralHeading) };

    const double driveThrust { m_input.throttle * m_config.maxDriveForce };
    const double trackBrake { m_input.brake * m_config.maxBrakeForce };
    const double accelX { std::clamp((driveThrust - trackBrake) / totalMass, -20.0, 20.0) };
    const double accelY { forwardSpeed * yawRate };

    // Static weight distribution
    const double staticFrontLoad { totalMass * gravity * (m_config.distRearTrack / wheelbase) };
    const double staticTrackLoad { totalMass * gravity * (m_config.distFrontSkis / wheelbase) };

    // Longitudinal load transfer
    const double longLoadShift { totalMass * accelX * (m_config.cogHeight / wheelbase) };
    const double frontNormalLoad { std::max(0.0, staticFrontLoad - longLoadShift) };
    const double trackNormalLoad { std::max(0.0, staticTrackLoad + longLoadShift) };

    // Active rider lateral weight shift
    const double riderShiftY { m_input.riderLean * m_config.riderMaxShiftY };
    const double latLoadShift { (frontNormalLoad * 0.5) * accelY * (m_config.cogHeight / m_config.skiStanceWidth) -
                                m_config.riderMass * gravity * (riderShiftY / m_config.skiStanceWidth) };

    const double leftSkiLoad { std::max(0.0, frontNormalLoad * 0.5 - latLoadShift) };
    const double rightSkiLoad { std::max(0.0, frontNormalLoad * 0.5 + latLoadShift) };

    // Steering geometry
    const double steerAngle { -m_input.steer * m_config.maxSteerAngle };
    const double cosSteer { std::cos(steerAngle) };
    const double sinSteer { std::sin(steerAngle) };

    // Ski velocities and slip angles
    const double leftSkiVelX { forwardSpeed - yawRate * halfStance };
    const double leftSkiVelY { lateralSpeed + yawRate * m_config.distFrontSkis };
    const double leftSkiLocalX { leftSkiVelX * cosSteer + leftSkiVelY * sinSteer };
    const double leftSkiLocalY { -leftSkiVelX * sinSteer + leftSkiVelY * cosSteer };
    const double leftSlipAngle { -std::atan2(leftSkiLocalY, std::max(0.1, std::abs(leftSkiLocalX))) };

    const double rightSkiVelX { forwardSpeed + yawRate * halfStance };
    const double rightSkiVelY { lateralSpeed + yawRate * m_config.distFrontSkis };
    const double rightSkiLocalX { rightSkiVelX * cosSteer + rightSkiVelY * sinSteer };
    const double rightSkiLocalY { -rightSkiVelX * sinSteer + rightSkiVelY * cosSteer };
    const double rightSlipAngle { -std::atan2(rightSkiLocalY, std::max(0.1, std::abs(rightSkiLocalX))) };

    // Ski forces
    const double maxShearForce { m_config.snowShearStrength * (m_config.skiLength * m_config.skiWidth) };

    auto computeSkiForces = [&](double load, double slipAngle, double localVx) {
        if (load <= 0.0) {
            return std::pair<double, double>{ 0.0, 0.0 };
        }
        const double baseFriction { load * m_config.skiBaseFriction * std::sin(slipAngle) };
        const double edgeCarve { load * m_config.skiCarveStiffness * std::tan(m_rollAngleRad) * std::cos(slipAngle) };
        double latForce { baseFriction + edgeCarve };
        const double limit { maxShearForce + load * m_config.skiBaseFriction };
        latForce = std::clamp(latForce, -limit, limit);

        const double dir { (localVx >= 0.0) ? 1.0 : -1.0 };
        const double brakeForce { m_input.brake * m_config.maxBrakeForce * 0.15 };
        const double longForce { -load * m_config.skiBaseFriction * dir - brakeForce };

        const double bodyFx { longForce * cosSteer - latForce * sinSteer };
        const double bodyFy { longForce * sinSteer + latForce * cosSteer };
        return std::pair<double, double>{ bodyFx, bodyFy };
    };

    const auto [leftSkiFx, leftSkiFy] { computeSkiForces(leftSkiLoad, leftSlipAngle, leftSkiLocalX) };
    const auto [rightSkiFx, rightSkiFy] { computeSkiForces(rightSkiLoad, rightSlipAngle, rightSkiLocalX) };

    // Rear continuous track propulsion & snow mechanics
    const double trackDriveForce { driveThrust };
    const double trackBrakeForce { trackBrake * 0.70 };

    const double slipRatio { (trackNormalLoad > 1.0) ? (trackDriveForce / (trackNormalLoad * m_config.trackFrictionPeak + 1e-3)) : 0.0 };
    const double contactArea { m_config.trackWidth * m_config.trackLength };
    const double sinkageBase { trackNormalLoad / (m_config.snowCompactionModulus * contactArea + 1e-3) };
    const double snowSinkage { std::pow(std::max(0.0, sinkageBase), 1.0 / m_config.snowSinkageExponent) *
                               (1.0 + m_config.slipTrenchingGain * std::min(1.0, std::abs(slipRatio))) };

    const double dirX { (forwardSpeed >= 0.0) ? 1.0 : -1.0 };
    const double trenchDrag { 0.5 * m_config.snowDensity * gravity * m_config.trackWidth * snowSinkage * snowSinkage * dirX };

    const double trackFx { trackDriveForce - trenchDrag - trackBrakeForce - trackNormalLoad * 0.02 * dirX };
    const double trackVelY { lateralSpeed - yawRate * m_config.distRearTrack };
    const double trackSlipAngle { -std::atan2(trackVelY, std::max(0.1, std::abs(forwardSpeed))) };
    const double trackFy { trackNormalLoad * m_config.trackFrictionPeak * std::sin(trackSlipAngle) };

    // Roll dynamics integration
    const double centripetalTorque { -totalMass * accelY * m_config.cogHeight };
    const double riderTorque { m_config.riderMass * gravity * riderShiftY };
    const double suspTorque { -m_config.rollStiffness * m_rollAngleRad - m_config.rollDamping * m_rollRateRadPerSec };
    const double rollAcc { (centripetalTorque + riderTorque + suspTorque) / m_config.rollInertia };

    m_rollRateRadPerSec += rollAcc * dt;
    m_rollAngleRad += m_rollRateRadPerSec * dt;
    m_rollAngleRad = std::clamp(m_rollAngleRad, -0.60, 0.60);

    // Accumulate net chassis forces & yaw torque
    const double totalBodyFx { leftSkiFx + rightSkiFx + trackFx };
    const double totalBodyFy { leftSkiFy + rightSkiFy + trackFy };

    const math::Vec2 netWorldForce { forwardHeading * totalBodyFx + lateralHeading * totalBodyFy };
    const double netYawTorque { m_config.distFrontSkis * (leftSkiFy + rightSkiFy) -
                                m_config.distRearTrack * trackFy +
                                halfStance * (rightSkiFx - leftSkiFx) };

    body.applyForce(netWorldForce);
    body.applyTorque(netYawTorque);

    // Diagnostics & telemetry export
    m_isAirborne = (frontNormalLoad + trackNormalLoad <= 1.0);

    m_telemetry.forwardSpeed = forwardSpeed;
    m_telemetry.lateralSpeed = lateralSpeed;
    m_telemetry.yawRate = yawRate;
    m_telemetry.rollAngle = m_rollAngleRad;
    m_telemetry.rollRate = m_rollRateRadPerSec;

    m_telemetry.leftSkiLoad = leftSkiLoad;
    m_telemetry.rightSkiLoad = rightSkiLoad;
    m_telemetry.trackLoad = trackNormalLoad;

    m_telemetry.leftSkiCarveForce = leftSkiFy;
    m_telemetry.rightSkiCarveForce = rightSkiFy;
    m_telemetry.trackTractiveForce = trackFx;
    m_telemetry.trackTrenchingDrag = trenchDrag;
    m_telemetry.trackSlipRatio = slipRatio;
    m_telemetry.snowSinkageDepth = snowSinkage;

    m_telemetry.isInsideSkiLifted = (leftSkiLoad <= 1.0 || rightSkiLoad <= 1.0) && (frontNormalLoad > 10.0);
    m_telemetry.isPowerWheelie = (frontNormalLoad <= 1.0) && (trackNormalLoad > 10.0);
    m_telemetry.isAirborne = m_isAirborne;
}

void SnowmobileDynamics::reset() noexcept {
    m_input = SnowmobileInput{};
    m_telemetry = SnowmobileTelemetry{};
    m_rollAngleRad = 0.0;
    m_rollRateRadPerSec = 0.0;
    m_lastForwardSpeed = 0.0;
    m_isAirborne = false;
}

bool SnowmobileDynamics::isAirborne() const noexcept {
    return m_isAirborne;
}

const SnowmobileConfig& SnowmobileDynamics::getConfig() const noexcept {
    return m_config;
}

void SnowmobileDynamics::setConfig(const SnowmobileConfig& config) noexcept {
    m_config = config;
}

const SnowmobileInput& SnowmobileDynamics::getInput() const noexcept {
    return m_input;
}

void SnowmobileDynamics::setInput(const SnowmobileInput& input) noexcept {
    m_input = input;
}

const SnowmobileTelemetry& SnowmobileDynamics::getTelemetry() const noexcept {
    return m_telemetry;
}

} // namespace vehicle_sim::dynamics
