#include "libs/VehicleSimCore/dynamics/BicycleDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

BicycleDynamics::BicycleDynamics() noexcept
    : BicycleDynamics(BicycleConfig{}) {}

BicycleDynamics::BicycleDynamics(const BicycleConfig& config) noexcept
    : m_config{ config } {}

void BicycleDynamics::step(physics::RigidBody2D& body, double dt) noexcept {
    if (dt <= 0.0) {
        return;
    }

    const double gravity { 9.80665 };
    const double L_wb { m_config.wheelbase };
    const double b_f { m_config.distFrontAxle };
    const double b_r { L_wb - b_f };
    const double lambda { m_config.headTubeAngleRad };
    const double cosLambda { std::cos(lambda) };
    const double sinLambda { std::sin(lambda) };

    const math::Vec2 bodyVel { body.getLinearVelocity() };
    const double bodyAngle { body.getOrientation() };
    const double yawRate { body.getAngularVelocity() };

    const math::Vec2 forwardHeading { std::cos(bodyAngle), std::sin(bodyAngle) };
    const math::Vec2 lateralHeading { -std::sin(bodyAngle), std::cos(bodyAngle) };

    const double forwardSpeed { bodyVel.dot(forwardHeading) };
    const double lateralSpeed { bodyVel.dot(lateralHeading) };

    const double driveForce { m_input.pedalingThrottle * m_config.maxDriveForce };
    const double frontBrake { m_input.frontBrake * m_config.maxFrontBrakeForce };
    const double rearBrake { m_input.rearBrake * m_config.maxRearBrakeForce };
    const double accelX { std::clamp((driveForce - frontBrake - rearBrake) / m_config.totalMass, -20.0, 20.0) };

    // Mechanical trail calculation
    const double mechanicalTrail { (m_config.frontWheelRadius * cosLambda - m_config.forkRakeOffset) /
                                   std::max(0.1, sinLambda) };

    // Dynamic vertical load transfer
    const double staticFrontLoad { m_config.totalMass * gravity * (b_r / L_wb) };
    const double staticRearLoad { m_config.totalMass * gravity * (b_f / L_wb) };
    const double pitchShift { m_config.totalMass * accelX * (m_config.cogHeight / L_wb) };

    const double frontNormalLoad { std::max(0.0, staticFrontLoad - pitchShift) };
    const double rearNormalLoad { std::max(0.0, staticRearLoad + pitchShift) };

    // Angular momentum of wheels
    const double absSpeed { std::max(0.0, forwardSpeed) };
    const double L_wheel_f { m_config.frontWheelSpinInertia * (absSpeed / m_config.frontWheelRadius) };

    // Gyroscopic precession and trail moments
    const double gyroPrecessionTorque { L_wheel_f * m_rollRateRadPerSec * sinLambda };
    const double gyroRollTorque { -L_wheel_f * m_steerRateRadPerSec * sinLambda };

    // Slip angles & tire forces
    const double frontSlipAngle { (lateralSpeed + b_f * yawRate) / std::max(0.5, std::abs(forwardSpeed)) -
                                  m_steerAngleRad * cosLambda };
    const double rearSlipAngle { (lateralSpeed - b_r * yawRate) / std::max(0.5, std::abs(forwardSpeed)) };

    const double frontCamberThrust { m_config.frontCamberStiffness * m_rollAngleRad };
    const double rearCamberThrust { m_config.rearCamberStiffness * m_rollAngleRad };

    double frontLatForce { -m_config.frontCorneringStiffness * frontSlipAngle + frontCamberThrust };
    double rearLatForce { -m_config.rearCorneringStiffness * rearSlipAngle + rearCamberThrust };

    const double maxFrontLat { m_config.tireFrictionCoeff * frontNormalLoad };
    const double maxRearLat { m_config.tireFrictionCoeff * rearNormalLoad };
    frontLatForce = std::clamp(frontLatForce, -maxFrontLat, maxFrontLat);
    rearLatForce = std::clamp(rearLatForce, -maxRearLat, maxRearLat);

    const double casterTrailTorque { mechanicalTrail * frontLatForce * cosLambda };

    // Whipple 2-DOF Generalized Torques Q_phi and Q_delta
    const double gravTorque { m_config.totalMass * gravity * m_config.cogHeight * m_rollAngleRad };
    const double centripetalRoll { -m_config.totalMass * m_config.cogHeight *
                                   (absSpeed * absSpeed * m_steerAngleRad * cosLambda / L_wb) };
    const double riderRollTorque { m_input.riderLeanRoll * (m_config.riderMass * gravity * 0.15) };
    const double rollDamping { -16.0 * m_rollRateRadPerSec };

    const double Q_phi { gravTorque + centripetalRoll + gyroRollTorque + riderRollTorque + rollDamping };

    const double forkDropTorque { -m_config.forkMass * gravity * mechanicalTrail * m_steerAngleRad * sinLambda };
    const double steerDampingTorque { -m_config.steerDamping * m_steerRateRadPerSec };

    const double Q_delta { gyroPrecessionTorque + casterTrailTorque + m_input.steerTorque +
                           forkDropTorque + steerDampingTorque };

    // Solve 2x2 mass matrix: M * [phi_ddot, delta_ddot]^T = [Q_phi, Q_delta]^T
    const double M11 { m_config.massRollInertia };
    const double M12 { m_config.massCrossInertia };
    const double M22 { m_config.massSteerInertia };
    const double detM { M11 * M22 - M12 * M12 };

    if (detM > 1e-4) {
        const double rollAcc { (M22 * Q_phi - M12 * Q_delta) / detM };
        const double steerAcc { (M11 * Q_delta - M12 * Q_phi) / detM };

        m_rollRateRadPerSec += rollAcc * dt;
        m_rollAngleRad += m_rollRateRadPerSec * dt;
        m_rollAngleRad = std::clamp(m_rollAngleRad, -1.0, 1.0);

        m_steerRateRadPerSec += steerAcc * dt;
        m_steerAngleRad += m_steerRateRadPerSec * dt;
        m_steerAngleRad = std::clamp(m_steerAngleRad, -m_config.maxSteerAngle, m_config.maxSteerAngle);
    }

    // Longitudinal forces
    const double dirX { (forwardSpeed >= 0.0) ? 1.0 : -1.0 };
    const double rollingResistance { (frontNormalLoad + rearNormalLoad) * 0.005 * dirX };

    const double netBodyFx { driveForce - frontBrake - rearBrake - rollingResistance };
    const double netBodyFy { frontLatForce + rearLatForce };

    const math::Vec2 netWorldForce { forwardHeading * netBodyFx + lateralHeading * netBodyFy };
    const double netYawTorque { b_f * frontLatForce - b_r * rearLatForce };

    body.applyForce(netWorldForce);
    body.applyTorque(netYawTorque);

    // Diagnostics & telemetry export
    m_isAirborne = (frontNormalLoad + rearNormalLoad <= 1.0);

    m_telemetry.forwardSpeed = forwardSpeed;
    m_telemetry.rollAngle = m_rollAngleRad;
    m_telemetry.rollRate = m_rollRateRadPerSec;
    m_telemetry.steerAngle = m_steerAngleRad;
    m_telemetry.steerRate = m_steerRateRadPerSec;

    m_telemetry.mechanicalTrail = mechanicalTrail;
    m_telemetry.frontWheelGyroscopicTorque = gyroPrecessionTorque;
    m_telemetry.casterRestoringTorque = casterTrailTorque;

    m_telemetry.frontNormalLoad = frontNormalLoad;
    m_telemetry.rearNormalLoad = rearNormalLoad;
    m_telemetry.frontLateralForce = frontLatForce;
    m_telemetry.rearLateralForce = rearLatForce;
    m_telemetry.frontCamberThrust = frontCamberThrust;
    m_telemetry.rearCamberThrust = rearCamberThrust;

    m_telemetry.isCounterSteering = (m_rollAngleRad * m_steerAngleRad < -1e-6) ||
                                   (m_steerAngleRad * m_rollRateRadPerSec < -1e-4) ||
                                   (m_input.steerTorque * m_rollRateRadPerSec < -1e-4);
    m_telemetry.isSelfStable = (absSpeed >= 4.2 && absSpeed <= 8.5);
    m_telemetry.isCapsizeMode = (absSpeed > 8.5);
    m_telemetry.isAirborne = m_isAirborne;
}

void BicycleDynamics::reset() noexcept {
    m_input = BicycleInput{};
    m_telemetry = BicycleTelemetry{};
    m_rollAngleRad = 0.0;
    m_rollRateRadPerSec = 0.0;
    m_steerAngleRad = 0.0;
    m_steerRateRadPerSec = 0.0;
    m_lastForwardSpeed = 0.0;
    m_isAirborne = false;
}

bool BicycleDynamics::isAirborne() const noexcept {
    return m_isAirborne;
}

const BicycleConfig& BicycleDynamics::getConfig() const noexcept {
    return m_config;
}

void BicycleDynamics::setConfig(const BicycleConfig& config) noexcept {
    m_config = config;
}

const BicycleInput& BicycleDynamics::getInput() const noexcept {
    return m_input;
}

void BicycleDynamics::setInput(const BicycleInput& input) noexcept {
    m_input = input;
}

const BicycleTelemetry& BicycleDynamics::getTelemetry() const noexcept {
    return m_telemetry;
}

} // namespace vehicle_sim::dynamics
