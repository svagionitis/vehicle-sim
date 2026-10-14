#include "libs/VehicleSimCore/dynamics/QuadBikeDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

QuadBikeDynamics::QuadBikeDynamics() noexcept
    : QuadBikeDynamics(QuadBikeConfig{}) {}

QuadBikeDynamics::QuadBikeDynamics(const QuadBikeConfig& config) noexcept
    : m_config{ config } {}

void QuadBikeDynamics::step(physics::RigidBody2D& body, double dt) noexcept {
    if (dt <= 0.0) {
        return;
    }

    const double gravity { 9.80665 };
    const double L_wb { m_config.wheelbase };
    const double a_static { m_config.distFrontAxle };
    const double b_static { L_wb - a_static };
    const double w_f { m_config.trackWidthFront };
    const double w_r { m_config.trackWidthRear };
    const double w_avg { (w_f + w_r) * 0.5 };
    const double totalMass { m_config.chassisMass + m_config.riderMass };

    const math::Vec2 bodyVel { body.getLinearVelocity() };
    const double bodyAngle { body.getOrientation() };
    const double yawRate { body.getAngularVelocity() };

    const math::Vec2 forwardHeading { std::cos(bodyAngle), std::sin(bodyAngle) };
    const math::Vec2 lateralHeading { -std::sin(bodyAngle), std::cos(bodyAngle) };

    const double forwardSpeed { bodyVel.dot(forwardHeading) };
    const double lateralSpeed { bodyVel.dot(lateralHeading) };

    const double driveThrust { m_input.throttle * m_config.maxDriveForce };
    const double totalBrake { m_input.frontBrake * m_config.maxFrontBrakeForce +
                              m_input.rearBrake * m_config.maxRearBrakeForce };
    const double accelX { std::clamp((driveThrust - totalBrake) / totalMass, -20.0, 20.0) };
    const double accelY { forwardSpeed * yawRate };

    // Dynamic active rider torso relocation
    const double shiftRiderX { m_input.riderLeanPitch * m_config.riderMaxShiftX };
    const double shiftRiderY { m_input.riderLeanRoll * m_config.riderMaxShiftY };

    const double cogOffsetX { (m_config.riderMass * shiftRiderX) / totalMass };
    const double cogOffsetY { (m_config.riderMass * shiftRiderY) / totalMass };
    const double h_cog { m_config.cogHeightStatic };

    const double a_eff { a_static - cogOffsetX };
    const double b_eff { b_static + cogOffsetX };

    // Dynamic normal load transfer across all 4 wheels
    const double staticFrontAxle { totalMass * gravity * (b_eff / L_wb) };
    const double staticRearAxle { totalMass * gravity * (a_eff / L_wb) };
    const double pitchShift { totalMass * accelX * (h_cog / L_wb) };

    const double latShiftFront { 0.5 * totalMass * accelY * (h_cog / w_f) - totalMass * gravity * (cogOffsetY / w_f) };
    const double latShiftRear { 0.5 * totalMass * accelY * (h_cog / w_r) - totalMass * gravity * (cogOffsetY / w_r) };

    double loadFL { std::max(0.0, 0.5 * staticFrontAxle - 0.5 * pitchShift - latShiftFront) };
    double loadFR { std::max(0.0, 0.5 * staticFrontAxle - 0.5 * pitchShift + latShiftFront) };
    double loadRL { std::max(0.0, 0.5 * staticRearAxle + 0.5 * pitchShift - latShiftRear) };
    double loadRR { std::max(0.0, 0.5 * staticRearAxle + 0.5 * pitchShift + latShiftRear) };

    std::array<double, 4> wheelLoads { loadFL, loadFR, loadRL, loadRR };

    // Balloon tire compliance and footprint expansion
    std::array<double, 4> carcassDeflections {};
    std::array<double, 4> patchAreas {};
    std::array<double, 4> gripMultipliers {};

    for (size_t i = 0; i < 4; ++i) {
        carcassDeflections[i] = wheelLoads[i] / m_config.tireCarcassStiffness;
        patchAreas[i] = m_config.nominalPatchArea *
                        (1.0 + m_config.footprintExpansionGain * (carcassDeflections[i] / m_config.tireRadiusFree));
        const double areaExpansionRatio { (patchAreas[i] - m_config.nominalPatchArea) / m_config.nominalPatchArea };
        gripMultipliers[i] = 1.0 + m_config.softSoilGripGain * std::max(0.0, areaExpansionRatio);
    }

    // Steering geometry
    const double steerAngle { -m_input.steer * m_config.maxSteerAngle };
    const double cosSteer { std::cos(steerAngle) };
    const double sinSteer { std::sin(steerAngle) };

    // Wheel kinematic velocities
    const double vxFL { forwardSpeed - yawRate * (w_f * 0.5) };
    const double vyFL { lateralSpeed + yawRate * a_eff };
    const double vxFR { forwardSpeed + yawRate * (w_f * 0.5) };
    const double vyFR { lateralSpeed + yawRate * a_eff };
    const double vxRL { forwardSpeed - yawRate * (w_r * 0.5) };
    const double vyRL { lateralSpeed - yawRate * b_eff };
    const double vxRR { forwardSpeed + yawRate * (w_r * 0.5) };
    const double vyRR { lateralSpeed - yawRate * b_eff };

    // Slip angles
    const double localVxFL { vxFL * cosSteer + vyFL * sinSteer };
    const double localVyFL { -vxFL * sinSteer + vyFL * cosSteer };
    const double slipAngleFL { -std::atan2(localVyFL, std::max(0.2, std::abs(localVxFL))) };

    const double localVxFR { vxFR * cosSteer + vyFR * sinSteer };
    const double localVyFR { -vxFR * sinSteer + vyFR * cosSteer };
    const double slipAngleFR { -std::atan2(localVyFR, std::max(0.2, std::abs(localVxFR))) };

    const double slipAngleRL { -std::atan2(vyRL, std::max(0.2, std::abs(vxRL))) };
    const double slipAngleRR { -std::atan2(vyRR, std::max(0.2, std::abs(vxRR))) };

    // Solid spool rear axle cornering scrub
    double scrubForceRL { 0.0 };
    double scrubForceRR { 0.0 };
    double rearBindingTorque { 0.0 };

    if (m_config.rearDiffMode == QuadDiffMode::LockedSpool) {
        const double avgRearVx { (vxRL + vxRR) * 0.5 };
        const double scrubSlipRL { (avgRearVx - vxRL) / std::max(0.5, std::abs(vxRL)) };
        const double scrubSlipRR { (avgRearVx - vxRR) / std::max(0.5, std::abs(vxRR)) };

        scrubForceRL = wheelLoads[2] * gripMultipliers[2] * 0.75 * std::clamp(scrubSlipRL, -1.0, 1.0);
        scrubForceRR = wheelLoads[3] * gripMultipliers[3] * 0.75 * std::clamp(scrubSlipRR, -1.0, 1.0);
        rearBindingTorque = (scrubForceRR - scrubForceRL) * (w_r * 0.5);
    }

    // Drive and brake distribution
    const double frontBrake { m_input.frontBrake * m_config.maxFrontBrakeForce * 0.5 };
    const double rearBrake { m_input.rearBrake * m_config.maxRearBrakeForce * 0.5 };

    const double drivePerRearWheel { driveThrust * 0.5 };

    // Lateral cornering forces
    auto computeCornering = [](double load, double grip, double slip) {
        if (load <= 0.0) {
            return 0.0;
        }
        return load * grip * std::sin(1.3 * std::atan(7.0 * slip));
    };

    const double latFL { computeCornering(wheelLoads[0], gripMultipliers[0], slipAngleFL) };
    const double latFR { computeCornering(wheelLoads[1], gripMultipliers[1], slipAngleFR) };
    const double latRL { computeCornering(wheelLoads[2], gripMultipliers[2], slipAngleRL) };
    const double latRR { computeCornering(wheelLoads[3], gripMultipliers[3], slipAngleRR) };

    // Wheel local forces to body frame
    const double bodyFxFL { -frontBrake * cosSteer - latFL * sinSteer };
    const double bodyFyFL { -frontBrake * sinSteer + latFL * cosSteer };

    const double bodyFxFR { -frontBrake * cosSteer - latFR * sinSteer };
    const double bodyFyFR { -frontBrake * sinSteer + latFR * cosSteer };

    const double bodyFxRL { drivePerRearWheel - rearBrake + scrubForceRL };
    const double bodyFyRL { latRL };

    const double bodyFxRR { drivePerRearWheel - rearBrake + scrubForceRR };
    const double bodyFyRR { latRR };

    // Accumulate total chassis forces and yaw moment
    const double totalBodyFx { bodyFxFL + bodyFxFR + bodyFxRL + bodyFxRR };
    const double totalBodyFy { bodyFyFL + bodyFyFR + bodyFyRL + bodyFyRR };

    const math::Vec2 netWorldForce { forwardHeading * totalBodyFx + lateralHeading * totalBodyFy };
    const double netYawTorque { a_eff * (bodyFyFL + bodyFyFR) - b_eff * (bodyFyRL + bodyFyRR) +
                                (w_f * 0.5) * (bodyFxFR - bodyFxFL) +
                                (w_r * 0.5) * (bodyFxRR - bodyFxRL) +
                                rearBindingTorque };

    body.applyForce(netWorldForce);
    body.applyTorque(netYawTorque);

    // Rollover & Pitchover indices
    const double rolloverIndex { (std::abs(accelY) * h_cog + gravity * std::abs(cogOffsetY)) /
                                 (gravity * (w_avg * 0.5)) };
    const double pitchoverIndex { (std::abs(accelX) * h_cog + gravity * std::abs(cogOffsetX)) /
                                  (gravity * (L_wb * 0.5)) };

    // Wheel lift flags
    const bool twoWheelSide { (wheelLoads[0] <= 1.0 && wheelLoads[2] <= 1.0) ||
                              (wheelLoads[1] <= 1.0 && wheelLoads[3] <= 1.0) };
    const bool wheelie { (wheelLoads[0] <= 1.0 && wheelLoads[1] <= 1.0) && (wheelLoads[2] + wheelLoads[3] > 10.0) };
    const bool stoppie { (wheelLoads[2] <= 1.0 && wheelLoads[3] <= 1.0) && (wheelLoads[0] + wheelLoads[1] > 10.0) };
    m_isAirborne = (wheelLoads[0] + wheelLoads[1] + wheelLoads[2] + wheelLoads[3] <= 1.0);

    // Diagnostics & telemetry export
    m_telemetry.forwardSpeed = forwardSpeed;
    m_telemetry.lateralSpeed = lateralSpeed;
    m_telemetry.yawRate = yawRate;
    m_telemetry.compositeCogOffset = math::Vec2{ cogOffsetX, cogOffsetY };
    m_telemetry.lateralRolloverIndex = rolloverIndex;
    m_telemetry.longitudinalPitchoverIndex = pitchoverIndex;
    m_telemetry.rearAxleBindingTorque = rearBindingTorque;

    auto fillWheelState = [&](size_t idx, double load, double defl, double area, double slip, double tractive, double cornering, bool lifted) {
        m_telemetry.wheels[idx].normalLoad = load;
        m_telemetry.wheels[idx].carcassDeflection = defl;
        m_telemetry.wheels[idx].contactArea = area;
        m_telemetry.wheels[idx].slipRatio = 0.0;
        m_telemetry.wheels[idx].slipAngle = slip;
        m_telemetry.wheels[idx].tractiveForce = tractive;
        m_telemetry.wheels[idx].corneringForce = cornering;
        m_telemetry.wheels[idx].isWheelLifted = lifted;
    };

    fillWheelState(0, wheelLoads[0], carcassDeflections[0], patchAreas[0], slipAngleFL, bodyFxFL, bodyFyFL, wheelLoads[0] <= 1.0);
    fillWheelState(1, wheelLoads[1], carcassDeflections[1], patchAreas[1], slipAngleFR, bodyFxFR, bodyFyFR, wheelLoads[1] <= 1.0);
    fillWheelState(2, wheelLoads[2], carcassDeflections[2], patchAreas[2], slipAngleRL, bodyFxRL, bodyFyRL, wheelLoads[2] <= 1.0);
    fillWheelState(3, wheelLoads[3], carcassDeflections[3], patchAreas[3], slipAngleRR, bodyFxRR, bodyFyRR, wheelLoads[3] <= 1.0);

    m_telemetry.isTwoWheelSideRiding = twoWheelSide;
    m_telemetry.isWheelie = wheelie;
    m_telemetry.isStoppie = stoppie;
    m_telemetry.isAirborne = m_isAirborne;
}

void QuadBikeDynamics::reset() noexcept {
    m_input = QuadBikeInput{};
    m_telemetry = QuadBikeTelemetry{};
    m_lastForwardSpeed = 0.0;
    m_isAirborne = false;
}

bool QuadBikeDynamics::isAirborne() const noexcept {
    return m_isAirborne;
}

const QuadBikeConfig& QuadBikeDynamics::getConfig() const noexcept {
    return m_config;
}

void QuadBikeDynamics::setConfig(const QuadBikeConfig& config) noexcept {
    m_config = config;
}

const QuadBikeInput& QuadBikeDynamics::getInput() const noexcept {
    return m_input;
}

void QuadBikeDynamics::setInput(const QuadBikeInput& input) noexcept {
    m_input = input;
}

const QuadBikeTelemetry& QuadBikeDynamics::getTelemetry() const noexcept {
    return m_telemetry;
}

} // namespace vehicle_sim::dynamics
