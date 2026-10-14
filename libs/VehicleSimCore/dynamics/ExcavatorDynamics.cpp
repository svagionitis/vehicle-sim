#include "libs/VehicleSimCore/dynamics/ExcavatorDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

namespace {
    constexpr double kGravity = 9.81;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kEpsilon = 1.0e-5;
} // namespace

ExcavatorDynamics::ExcavatorDynamics() noexcept
    : ExcavatorDynamics(ExcavatorConfig {})
{
}

ExcavatorDynamics::ExcavatorDynamics(const ExcavatorConfig& config) noexcept
    : m_config { config }
    , m_boomAngle { 0.35 }
    , m_stickAngle { 1.57 }
    , m_bucketAngle { 0.78 }
{
    setArmJointAngles(m_boomAngle, m_stickAngle, m_bucketAngle);
}

void ExcavatorDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Advance crawler undercarriage skid-steer kinematics & ground contact forces
    updateUndercarriage(body, dt);

    // 2. Advance 360-degree upper superstructure slew ring bearing
    updateSlewRing(dt);

    // 3. Advance planar 3-DOF articulated arm kinematics & digging reaction
    updateArmKinematics(dt);

    // 4. Evaluate machine static and dynamic tipping margin about crawler fulcrums
    evaluateTippingMargin();

    // 5. Export synchronized diagnostic telemetry
    m_telemetry.slewAngle = m_slewAngle;
    m_telemetry.slewSpeed = m_slewVelocity;
    m_telemetry.boomAngle = m_boomAngle;
    m_telemetry.stickAngle = m_stickAngle;
    m_telemetry.bucketAngle = m_bucketAngle;
    m_telemetry.digForce = m_digForceWorld;
    m_telemetry.isTippedOver = m_isTippedOver;
}

void ExcavatorDynamics::reset() noexcept
{
    m_input = ExcavatorInput {};
    m_telemetry = ExcavatorTelemetry {};
    m_slewAngle = 0.0;
    m_slewVelocity = 0.0;
    m_boomAngle = 0.35;
    m_stickAngle = 1.57;
    m_bucketAngle = 0.78;
    m_bucketPayload = 0.0;
    m_digForceWorld = math::Vec2 { 0.0, 0.0 };
    m_isTippedOver = false;
}

bool ExcavatorDynamics::isAirborne() const noexcept
{
    return m_isTippedOver;
}

const ExcavatorConfig& ExcavatorDynamics::getConfig() const noexcept
{
    return m_config;
}

void ExcavatorDynamics::setConfig(const ExcavatorConfig& config) noexcept
{
    m_config = config;
}

const ExcavatorInput& ExcavatorDynamics::getInput() const noexcept
{
    return m_input;
}

void ExcavatorDynamics::setInput(const ExcavatorInput& input) noexcept
{
    m_input = input;
}

const ExcavatorTelemetry& ExcavatorDynamics::getTelemetry() const noexcept
{
    return m_telemetry;
}

void ExcavatorDynamics::setDigForce(const math::Vec2& force) noexcept
{
    m_digForceWorld = force;
}

void ExcavatorDynamics::setBucketPayload(double payloadMass) noexcept
{
    m_bucketPayload = std::clamp(payloadMass, 0.0, m_config.arm.maxBucketPayload);
}

math::Vec2 ExcavatorDynamics::getTcpPosition() const noexcept
{
    return m_telemetry.tcpPosition;
}

double ExcavatorDynamics::getSlewAngle() const noexcept
{
    return m_slewAngle;
}

bool ExcavatorDynamics::isTippedOver() const noexcept
{
    return m_isTippedOver;
}

void ExcavatorDynamics::setArmJointAngles(double boom, double stick, double bucket) noexcept
{
    m_boomAngle = std::clamp(boom, m_config.arm.boomMinAngle, m_config.arm.boomMaxAngle);
    m_stickAngle = std::clamp(stick, m_config.arm.stickMinAngle, m_config.arm.stickMaxAngle);
    m_bucketAngle = std::clamp(bucket, m_config.arm.bucketMinAngle, m_config.arm.bucketMaxAngle);

    // Update TCP immediately
    const double q1 = m_boomAngle;
    const double q12 = q1 + m_stickAngle;
    const double q123 = q12 + m_bucketAngle;
    const double x = m_config.arm.boomLength * std::cos(q1) + m_config.arm.stickLength * std::cos(q12)
        + m_config.arm.bucketRadius * std::cos(q123);
    const double z = m_config.arm.boomLength * std::sin(q1) + m_config.arm.stickLength * std::sin(q12)
        + m_config.arm.bucketRadius * std::sin(q123);
    m_telemetry.tcpPosition = math::Vec2 { x, z };
}

void ExcavatorDynamics::updateUndercarriage(physics::RigidBody2D& body, double dt) noexcept
{
    // Left and right track linear velocity commands
    const double leftCmd = std::clamp(m_input.leftTrack, -1.0, 1.0);
    const double rightCmd = std::clamp(m_input.rightTrack, -1.0, 1.0);

    const double targetLeftSpeed = leftCmd * m_config.maxTrackSpeed;
    const double targetRightSpeed = rightCmd * m_config.maxTrackSpeed;

    // Kinematic forward and yaw velocity
    const double forwardSpeed = 0.5 * (targetRightSpeed + targetLeftSpeed);
    const double trackGauge = std::max(m_config.trackGauge, 0.5);
    const double yawRate = (targetRightSpeed - targetLeftSpeed) / trackGauge;

    // Ground scrubbing resistance torque opposing turning
    double scrubTorque = 0.0;
    if (std::abs(yawRate) > kEpsilon) {
        const double scrubSign = (yawRate > 0.0) ? 1.0 : -1.0;
        scrubTorque = -scrubSign * m_config.groundScrubTorque * std::tanh(std::abs(yawRate) * 5.0);
    }

    // Undercarriage drive force along vehicle heading
    const double heading = body.getOrientation();
    const math::Vec2 forwardDir { std::cos(heading), std::sin(heading) };

    (void)dt;
    const double driveForceMag = (leftCmd + rightCmd) * 0.5 * m_config.maxTrackForce;
    const math::Vec2 driveForce = forwardDir * driveForceMag;

    // Apply forces and moments to RigidBody2D
    body.applyForce(driveForce);
    body.applyTorque(scrubTorque);

    // Update body velocities directly according to track kinematics
    body.setLinearVelocity(forwardDir * forwardSpeed);
    body.setAngularVelocity(yawRate);

    m_telemetry.forwardSpeed = forwardSpeed;
    m_telemetry.yawRate = yawRate;
}

void ExcavatorDynamics::updateSlewRing(double dt) noexcept
{
    // Dynamic composite moment of inertia adjustment based on arm reach
    const double tcpReach = std::max(std::abs(m_telemetry.tcpPosition.x), 1.0);
    const double armTotalMass
        = m_config.arm.boomMass + m_config.arm.stickMass + m_config.arm.bucketMass + m_bucketPayload;
    const double effectiveSlewInertia = m_config.slewInertia + armTotalMass * (tcpReach * 0.5) * (tcpReach * 0.5);

    if (m_input.slewBrakeActive) {
        // Holding brake engaged: decelerate rapidly to full stop
        constexpr double kHoldingBrakeTorque = 150000.0;
        const double brakeSign = (m_slewVelocity > 0.0) ? 1.0 : -1.0;
        const double maxBrakeTorque = std::abs(m_slewVelocity) * effectiveSlewInertia / dt;
        const double brakeTorque = -brakeSign * std::min(kHoldingBrakeTorque, maxBrakeTorque);
        const double angAcc = brakeTorque / effectiveSlewInertia;
        m_slewVelocity += angAcc * dt;
        if (std::abs(m_slewVelocity) < 1.0e-3) {
            m_slewVelocity = 0.0;
        }
    } else {
        const double slewCmd = std::clamp(m_input.slewCommand, -1.0, 1.0);
        const double motorTorque = slewCmd * m_config.maxSlewTorque;
        const double dampingTorque = -m_config.slewDamping * m_slewVelocity;
        const double netTorque = motorTorque + dampingTorque;

        const double angAcc = netTorque / effectiveSlewInertia;
        m_slewVelocity += angAcc * dt;
        m_slewVelocity = std::clamp(m_slewVelocity, -m_config.maxSlewSpeed, m_config.maxSlewSpeed);
    }

    m_slewAngle += m_slewVelocity * dt;

    // Wrap slew angle to [-pi, pi]
    while (m_slewAngle > 3.14159265358979323846) {
        m_slewAngle -= kTwoPi;
    }
    while (m_slewAngle < -3.14159265358979323846) {
        m_slewAngle += kTwoPi;
    }
}

void ExcavatorDynamics::updateArmKinematics(double dt) noexcept
{
    // Commanded actuator joint velocities
    double boomRate = std::clamp(m_input.boomCommand, -1.0, 1.0) * m_config.arm.maxBoomSpeed;
    double stickRate = std::clamp(m_input.stickCommand, -1.0, 1.0) * m_config.arm.maxStickSpeed;
    double bucketRate = std::clamp(m_input.bucketCommand, -1.0, 1.0) * m_config.arm.maxBucketSpeed;

    // Forward Kinematics angles
    const double q1 = m_boomAngle;
    const double q12 = q1 + m_stickAngle;
    const double q123 = q12 + m_bucketAngle;

    const double s1 = std::sin(q1);
    const double c1 = std::cos(q1);
    const double s12 = std::sin(q12);
    const double c12 = std::cos(q12);
    const double s123 = std::sin(q123);
    const double c123 = std::cos(q123);

    const double L1 = m_config.arm.boomLength;
    const double L2 = m_config.arm.stickLength;
    const double L3 = m_config.arm.bucketRadius;

    // Analytical Jacobian Matrix J(q) in R^(2x3):
    // J11 = -L1*s1 - L2*s12 - L3*s123, J12 = -L2*s12 - L3*s123, J13 = -L3*s123
    // J21 =  L1*c1 + L2*c12 + L3*c123, J22 =  L2*c12 + L3*c123, J23 =  L3*c123
    const double J11 = -L1 * s1 - L2 * s12 - L3 * s123;
    const double J12 = -L2 * s12 - L3 * s123;
    const double J13 = -L3 * s123;

    const double J21 = L1 * c1 + L2 * c12 + L3 * c123;
    const double J22 = L2 * c12 + L3 * c123;
    const double J23 = L3 * c123;

    // Reflected joint torques from digging force: tau = J^T * F_dig
    const double Fx = m_digForceWorld.x;
    const double Fz = m_digForceWorld.y;

    const double tauBoom = J11 * Fx + J21 * Fz;
    const double tauStick = J12 * Fx + J22 * Fz;
    const double tauBucket = J13 * Fx + J23 * Fz;

    const double maxTorque = m_config.arm.maxDiggingForce * L3;
    const bool isStalled = (std::abs(tauBoom) > maxTorque * 2.5) || (std::abs(tauStick) > maxTorque * 1.5)
        || (std::abs(tauBucket) > maxTorque);
    m_telemetry.isArmStalled = isStalled;

    if (isStalled) {
        // Digging resistance halts or severely resists cylinder motion
        boomRate *= 0.1;
        stickRate *= 0.1;
        bucketRate *= 0.1;
    }

    // Integrate joint angles with soft limits
    m_boomAngle += boomRate * dt;
    m_stickAngle += stickRate * dt;
    m_bucketAngle += bucketRate * dt;

    m_boomAngle = std::clamp(m_boomAngle, m_config.arm.boomMinAngle, m_config.arm.boomMaxAngle);
    m_stickAngle = std::clamp(m_stickAngle, m_config.arm.stickMinAngle, m_config.arm.stickMaxAngle);
    m_bucketAngle = std::clamp(m_bucketAngle, m_config.arm.bucketMinAngle, m_config.arm.bucketMaxAngle);

    // Compute updated Tool Center Point (TCP) in cabin coordinates
    const double updatedQ1 = m_boomAngle;
    const double updatedQ12 = updatedQ1 + m_stickAngle;
    const double updatedQ123 = updatedQ12 + m_bucketAngle;

    const double tcpX = L1 * std::cos(updatedQ1) + L2 * std::cos(updatedQ12) + L3 * std::cos(updatedQ123);
    const double tcpZ = L1 * std::sin(updatedQ1) + L2 * std::sin(updatedQ12) + L3 * std::sin(updatedQ123);
    m_telemetry.tcpPosition = math::Vec2 { tcpX, tcpZ };

    // Linear velocity of TCP: v_tcp = J * q_dot
    const double vTcpX = J11 * boomRate + J12 * stickRate + J13 * bucketRate;
    const double vTcpZ = J21 * boomRate + J22 * stickRate + J23 * bucketRate;
    m_telemetry.tcpVelocity = math::Vec2 { vTcpX, vTcpZ };
}

void ExcavatorDynamics::evaluateTippingMargin() noexcept
{
    // Stability fulcrum: crawler track contact edges x = +/- (L_c / 2)
    const double halfContact = m_config.trackContactLength * 0.5;

    // Undercarriage and superstructure base restoring moment about front fulcrum (+halfContact)
    const double baseMass = m_config.undercarriageMass + m_config.superstructureMass;
    const double restoringMoment = baseMass * kGravity * halfContact;

    // Overturning moment from arm extension and payload
    const double armTotalMass
        = m_config.arm.boomMass + m_config.arm.stickMass + m_config.arm.bucketMass + m_bucketPayload;
    // Arm center of gravity reaches forward proportional to TCP reach
    const double armCogX = m_telemetry.tcpPosition.x * 0.6;
    const double armOverturnMoment = armTotalMass * kGravity * std::max(0.0, armCogX - halfContact);

    // External digging force overturning contribution
    const double digOverturnMoment
        = std::max(0.0, -m_digForceWorld.y * std::max(0.0, m_telemetry.tcpPosition.x - halfContact));

    const double totalOverturnMoment = armOverturnMoment + digOverturnMoment;
    const double tippingIndex = totalOverturnMoment / std::max(restoringMoment, 1.0);

    m_telemetry.tippingIndex = tippingIndex;
    m_telemetry.tippingMargin = std::clamp(1.0 - tippingIndex, 0.0, 1.0);
    m_isTippedOver = (tippingIndex >= 1.0);
}

} // namespace vehicle_sim::dynamics
