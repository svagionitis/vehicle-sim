#include "libs/VehicleSimCore/dynamics/TrainRailDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

namespace {
    constexpr double kGravity = 9.81;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinVelocityThreshold = 0.5;
    constexpr double kEpsilon = 1.0e-6;
} // namespace

TrainRailDynamics::TrainRailDynamics() noexcept
    : TrainRailDynamics(TrainRailConfig {})
{
}

TrainRailDynamics::TrainRailDynamics(const TrainRailConfig& config) noexcept
    : m_config { config }
    , m_cantAngle { config.cantAngle }
{
}

void TrainRailDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    if (m_isDerailed) {
        m_telemetry.isDerailmentRisk = true;
        return;
    }

    // 1. Evaluate wheel-rail longitudinal traction and braking forces
    const double tractiveForce = evaluateTractionAndCreep();

    // 2. Evaluate multi-car coupler draft gear slack action
    evaluateCouplerForces();

    // 3. Total longitudinal propelling/resisting force along track tangent
    const double netTangentForce = tractiveForce + m_couplerForce;
    const double longitudinalAcc = netTangentForce / std::max(m_config.mass, 1.0);

    // 4. Update track longitudinal velocity and curvilinear distance s
    m_speed += longitudinalAcc * dt;
    if (std::abs(m_speed) < 1.0e-4 && std::abs(m_input.throttle) < 1.0e-4 && m_input.serviceBrake > 0.1) {
        m_speed = 0.0;
    }
    m_arcLength += m_speed * dt;

    // Handle track wrapping/clamping if track spline is bound
    double curvature = 0.0;
    math::Vec2 trackTangent { 1.0, 0.0 };
    math::Vec2 trackNormal { 0.0, 1.0 };
    math::Vec2 trackPos { body.getPosition() };

    if (m_track != nullptr && m_track->getSegmentCount() > 0) {
        const double totalLength = m_track->getTotalLength();
        if (totalLength > 0.0) {
            if (m_track->isClosed()) {
                while (m_arcLength < 0.0) {
                    m_arcLength += totalLength;
                }
                while (m_arcLength >= totalLength) {
                    m_arcLength -= totalLength;
                }
            } else {
                m_arcLength = std::clamp(m_arcLength, 0.0, totalLength);
            }
            const auto sample = m_track->evaluateAtDistance(m_arcLength);
            trackPos = sample.point;
            trackTangent = sample.tangent;
            trackNormal = sample.normal;
            curvature = sample.curvature;
        }
    }

    // 5. Evaluate lateral kinematics, Klingel hunting oscillation & wheel flange reaction
    evaluateFlangeAndNadal(dt);

    // 6. Centripetal balance and cant / superelevation uncompensated lateral acceleration
    const double radius = 1.0 / (std::abs(curvature) + kEpsilon);
    const double centrifugalAcc = (m_speed * m_speed) / radius * (curvature >= 0.0 ? 1.0 : -1.0);
    const double cantBalancingAcc = kGravity * std::sin(m_cantAngle);
    const double uncompLateralAcc = centrifugalAcc - cantBalancingAcc;

    // 7. Update rigid body pose to track Frenet frame + lateral offset
    const math::Vec2 bodyPos = trackPos + trackNormal * m_lateralOffset;
    body.setPosition(bodyPos);

    const double trackAngle = std::atan2(trackTangent.y, trackTangent.x);
    body.setOrientation(trackAngle);
    body.setLinearVelocity(trackTangent * m_speed + trackNormal * m_lateralVelocity);
    body.setAngularVelocity(m_speed * curvature);

    // 8. Apply forces to rigid body for external listeners / physics worlds
    body.applyForce(trackTangent * netTangentForce);
    body.applyForce(trackNormal * (-m_telemetry.flangeForce));

    // 9. Synchronize diagnostic telemetry
    m_telemetry.trackArcLength = m_arcLength;
    m_telemetry.trackSpeed = m_speed;
    m_telemetry.trackCurvature = curvature;
    m_telemetry.cantAngle = m_cantAngle;
    m_telemetry.uncompLateralAcc = uncompLateralAcc;
    m_telemetry.lateralDisplacement = m_lateralOffset;
    m_telemetry.tractiveForce = tractiveForce;
    m_telemetry.couplerForce = m_couplerForce;
    m_telemetry.isSanding = m_input.sanderActive;
    m_telemetry.isDerailmentRisk
        = (m_telemetry.nadalRatio >= m_telemetry.nadalLimit) || (std::abs(uncompLateralAcc) > 3.0);
}

void TrainRailDynamics::reset() noexcept
{
    m_input = TrainRailInput {};
    m_telemetry = TrainRailTelemetry {};
    m_speed = 0.0;
    m_lateralOffset = 0.0;
    m_lateralVelocity = 0.0;
    m_couplerForce = 0.0;
    m_isDerailed = false;
}

bool TrainRailDynamics::isAirborne() const noexcept
{
    return m_isDerailed;
}

const TrainRailConfig& TrainRailDynamics::getConfig() const noexcept
{
    return m_config;
}

void TrainRailDynamics::setConfig(const TrainRailConfig& config) noexcept
{
    m_config = config;
    m_cantAngle = config.cantAngle;
}

const TrainRailInput& TrainRailDynamics::getInput() const noexcept
{
    return m_input;
}

void TrainRailDynamics::setInput(const TrainRailInput& input) noexcept
{
    m_input = input;
}

const TrainRailTelemetry& TrainRailDynamics::getTelemetry() const noexcept
{
    return m_telemetry;
}

void TrainRailDynamics::setTrack(const math::Spline2D* track) noexcept
{
    m_track = track;
}

void TrainRailDynamics::setArcLength(double s) noexcept
{
    m_arcLength = s;
    m_telemetry.trackArcLength = s;
}

void TrainRailDynamics::setTrackSpeed(double speed) noexcept
{
    m_speed = speed;
    m_telemetry.trackSpeed = speed;
}

void TrainRailDynamics::setLateralOffset(double offset) noexcept
{
    m_lateralOffset = offset;
    m_telemetry.lateralDisplacement = offset;
}

void TrainRailDynamics::setLeadCar(const TrainRailDynamics* leadCar) noexcept
{
    m_leadCar = leadCar;
}

void TrainRailDynamics::setCantAngle(double cantRadians) noexcept
{
    m_cantAngle = cantRadians;
}

bool TrainRailDynamics::isDerailed() const noexcept
{
    return m_isDerailed;
}

double TrainRailDynamics::getArcLength() const noexcept
{
    return m_arcLength;
}

double TrainRailDynamics::getTrackSpeed() const noexcept
{
    return m_speed;
}

double TrainRailDynamics::getLateralOffset() const noexcept
{
    return m_lateralOffset;
}

void TrainRailDynamics::evaluateFlangeAndNadal(double dt) noexcept
{
    const double b = m_config.trackGauge * 0.5;
    const double r0 = std::max(m_config.wheelRadius, 0.05);
    const double gamma = std::max(m_config.wheelConicity, 0.001);

    // Klingel hunting natural angular frequency: omega_h = v * sqrt(gamma / (r0 * b))
    const double huntingAngularFreq = std::abs(m_speed) * std::sqrt(gamma / (r0 * b));
    m_telemetry.huntingFrequency = huntingAngularFreq / (2.0 * kPi);

    // Kinematic self-centering restoring acceleration towards centerline
    const double kCentering = huntingAngularFreq * huntingAngularFreq;
    const double centeringAcc = -kCentering * m_lateralOffset;
    const double dampingAcc = -2.0 * 0.1 * huntingAngularFreq * m_lateralVelocity;

    double flangeStrikeForce = 0.0;
    const double absOffset = std::abs(m_lateralOffset);
    if (absOffset > m_config.flangeClearance) {
        const double penetration = absOffset - m_config.flangeClearance;
        const double sign = (m_lateralOffset >= 0.0) ? 1.0 : -1.0;
        flangeStrikeForce
            = (m_config.flangeStiffness * penetration + m_config.flangeDamping * (m_lateralVelocity * sign)) * sign;
    }

    const double flangeAcc = -flangeStrikeForce / std::max(m_config.mass, 1.0);
    const double lateralAccTotal = centeringAcc + dampingAcc + flangeAcc;

    m_lateralVelocity += lateralAccTotal * dt;
    m_lateralOffset += m_lateralVelocity * dt;

    // Hard physical rail boundary limit
    const double maxPhysicalDeviation = m_config.flangeClearance * 3.5;
    if (std::abs(m_lateralOffset) > maxPhysicalDeviation) {
        m_lateralOffset = (m_lateralOffset >= 0.0 ? 1.0 : -1.0) * maxPhysicalDeviation;
        m_lateralVelocity = 0.0;
    }

    // Nadal criterion calculation: (Y / Q)_limit = (tan(delta) - mu) / (1 + mu * tan(delta))
    const double tanDelta = std::tan(m_config.flangeAngle);
    const double muFlange = m_config.flangeFriction;
    const double nadalLimit = (tanDelta - muFlange) / (1.0 + muFlange * tanDelta);
    m_telemetry.nadalLimit = nadalLimit;

    // Normal wheel load per wheel Q (assuming 4 axles / 8 wheels standard bogie setup)
    constexpr double kWheelsPerCar = 8.0;
    const double wheelVerticalLoad = (m_config.mass * kGravity) / kWheelsPerCar;
    const double lateralFlangeForceMagnitude = std::abs(flangeStrikeForce);
    const double nadalRatio = lateralFlangeForceMagnitude / std::max(wheelVerticalLoad, 1.0);

    m_telemetry.flangeForce = flangeStrikeForce;
    m_telemetry.nadalRatio = nadalRatio;

    if (nadalRatio >= nadalLimit) {
        m_telemetry.isDerailmentRisk = true;
    }
}

double TrainRailDynamics::evaluateTractionAndCreep() noexcept
{
    // 1. Determine active rail adhesion coefficient
    double activeAdhesion = m_config.dryAdhesion;
    if (m_input.sanderActive) {
        activeAdhesion = m_config.sandedAdhesion;
    }

    // Normal gravitational force on drive wheels
    const double totalWeight = m_config.mass * kGravity;
    const double maxAdhesionForce = totalWeight * activeAdhesion;

    // 2. Throttle demand
    const double throttleDemand = std::clamp(m_input.throttle, 0.0, 1.0);
    const double reverserSign = static_cast<double>(m_input.reverser);
    const double commandedTraction = throttleDemand * m_config.maxTractiveForce * reverserSign;

    // 3. Creep calculation: xi_x = (v_wheel - v_track) / max(|v_track|, v_thresh)
    const double speedDenom = std::max(std::abs(m_speed), kMinVelocityThreshold);
    // Commanded linear wheel speed relative to tractive force
    const double targetWheelSlip = commandedTraction / std::max(m_config.kalkerCoefficient, 1.0);
    const double creepRatio = targetWheelSlip / speedDenom;
    m_telemetry.creepRatio = creepRatio;

    // 4. Polach non-linear saturation via arctan formulation
    double availableTraction = commandedTraction;
    if (maxAdhesionForce > 0.0) {
        const double normalizedCreep = (kPi * commandedTraction) / (2.0 * maxAdhesionForce);
        availableTraction = maxAdhesionForce * (2.0 / kPi) * std::atan(normalizedCreep);
    }

    // 5. Pneumatic service and dynamic brakes
    const double brakeInput = std::clamp(m_input.serviceBrake + m_input.dynamicBrake, 0.0, 1.0);
    const double brakeForceCapacity = brakeInput * m_config.maxBrakingForce;
    double appliedBrake = 0.0;
    if (std::abs(m_speed) > 1.0e-3) {
        appliedBrake = -brakeForceCapacity * (m_speed > 0.0 ? 1.0 : -1.0);
    }

    // 6. Rolling and aerodynamic drag resistance (Davis equation: A + B*v + C*v^2)
    constexpr double kDavisA = 1200.0;
    constexpr double kDavisB = 35.0;
    constexpr double kDavisC = 4.2;
    const double sign = (m_speed >= 0.0) ? 1.0 : -1.0;
    const double absSpeed = std::abs(m_speed);
    const double runningResistance = sign * (kDavisA + kDavisB * absSpeed + kDavisC * absSpeed * absSpeed);

    return availableTraction + appliedBrake - runningResistance;
}

void TrainRailDynamics::evaluateCouplerForces() noexcept
{
    if (m_leadCar == nullptr) {
        m_couplerForce = 0.0;
        m_telemetry.couplerForce = 0.0;
        return;
    }

    // Relative distance between cars along curvilinear track
    const double nominalSeparation = m_config.carLength;
    const double actualSeparation = m_leadCar->getArcLength() - m_arcLength;
    const double displacement = actualSeparation - nominalSeparation;

    // Draft gear mechanical slack deadband [-draftSlack, +draftSlack]
    double effectiveSpringStretch = 0.0;
    if (displacement > m_config.draftSlack) {
        // Coupler under tension / draft
        effectiveSpringStretch = displacement - m_config.draftSlack;
    } else if (displacement < -m_config.draftSlack) {
        // Coupler under compression / buff
        effectiveSpringStretch = displacement + m_config.draftSlack;
    } else {
        // Inside free slack space
        effectiveSpringStretch = 0.0;
    }

    if (std::abs(effectiveSpringStretch) > 1.0e-9) {
        const double relativeVelocity = m_leadCar->getTrackSpeed() - m_speed;
        m_couplerForce = m_config.draftStiffness * effectiveSpringStretch + m_config.draftDamping * relativeVelocity;
    } else {
        m_couplerForce = 0.0;
    }
    m_telemetry.couplerForce = m_couplerForce;
}

} // namespace vehicle_sim::dynamics
