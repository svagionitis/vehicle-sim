#include "VTOLTiltRotorDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

namespace {
constexpr double kPi { 3.14159265358979323846 };
constexpr double kRadToDeg { 180.0 / kPi };
} // namespace

VTOLTiltRotorDynamics::VTOLTiltRotorDynamics() noexcept
    : VTOLTiltRotorDynamics(VTOLTiltRotorConfig { })
{
}

VTOLTiltRotorDynamics::VTOLTiltRotorDynamics(const VTOLTiltRotorConfig& config) noexcept
    : mConfig { config }
    , mTiltLimiter { config.maxTiltSlewRate }
    , mRotorRpm { config.ratedRotorRpm }
    , mActiveNacelleAngle { config.maxNacelleAngle }
{
    mTiltLimiter.reset(mActiveNacelleAngle);

    mTelemetry.rotorRpm = mRotorRpm;
    mTelemetry.nacelleAngleRad = mActiveNacelleAngle;
    mTelemetry.nacelleAngleDeg = mActiveNacelleAngle * kRadToDeg;
    mTelemetry.flightMode = VTOLFlightMode::Hover;
    mTelemetry.isCorridorSafe = true;
}

void VTOLTiltRotorDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Flight kinematics and coordinate frame transforms
    const math::Vec2 worldVel { body.getLinearVelocity() };
    const double airspeed { worldVel.length() };
    const math::Vec2 localVel { body.toLocalVec(worldVel) };
    const double forwardSpeed { localVel.x };
    const double climbRate { worldVel.y };
    const double omega { body.getAngularVelocity() };
    const double altitude { std::max(0.0, mInput.groundDistance) };

    mTelemetry.airspeed = airspeed;
    mTelemetry.forwardVelocity = forwardSpeed;
    mTelemetry.climbRate = climbRate;

    // 2. Nacelle angle conversion kinematics & slew-rate limiter
    const double commandedAngle { std::clamp(
        mInput.targetNacelleAngle, mConfig.minNacelleAngle, mConfig.maxNacelleAngle) };
    const double prevNacelleAngle { mActiveNacelleAngle };
    mActiveNacelleAngle = mTiltLimiter.update(commandedAngle, dt);
    const double tiltRate { (mActiveNacelleAngle - prevNacelleAngle) / dt };

    mTelemetry.nacelleAngleRad = mActiveNacelleAngle;
    mTelemetry.nacelleAngleDeg = mActiveNacelleAngle * kRadToDeg;
    mTelemetry.tiltRateDegPerSec = tiltRate * kRadToDeg;

    // 3. Operational flight mode categorization
    const double degAngle { mTelemetry.nacelleAngleDeg };
    if (degAngle >= 75.0) {
        mTelemetry.flightMode = VTOLFlightMode::Hover;
    } else if (degAngle <= 15.0) {
        mTelemetry.flightMode = VTOLFlightMode::AirplaneCruise;
    } else if (tiltRate < -1e-4) {
        mTelemetry.flightMode = VTOLFlightMode::ConversionForward;
    } else {
        mTelemetry.flightMode = VTOLFlightMode::ConversionAft;
    }

    // 4. Proprotor spool dynamics & thrust calculation
    if (mInput.enginePower) {
        const double rpmDelta { mConfig.ratedRotorRpm - mRotorRpm };
        const double maxDelta { mConfig.rotorSpoolRate * dt };
        mRotorRpm += std::clamp(rpmDelta, -maxDelta, maxDelta);
    } else {
        mRotorRpm = std::max(0.0, mRotorRpm - (mConfig.rotorSpoolRate * 0.5 * dt));
    }
    mTelemetry.rotorRpm = mRotorRpm;

    const double rpmRatio { std::clamp(mRotorRpm / std::max(1.0, mConfig.ratedRotorRpm), 0.0, 1.2) };
    const double throttle { std::clamp(mInput.throttle, 0.0, 1.0) };
    const double baseThrust { throttle * mConfig.maxTotalThrust * rpmRatio * rpmRatio };

    // In-Ground Effect (IGE) & Effective Translational Lift (ETL)
    const double sinNacelle { std::sin(mActiveNacelleAngle) };
    const double cosNacelle { std::cos(mActiveNacelleAngle) };

    double igeFactor { 1.0 };
    bool isIge { false };
    if (altitude < mConfig.igeHeight && altitude > 0.0 && sinNacelle > 0.5) {
        const double normAlt { altitude / mConfig.igeHeight };
        igeFactor = 1.0 + (mConfig.igeMaxBoost * (1.0 - normAlt) * (1.0 - normAlt) * sinNacelle);
        isIge = true;
    }
    mTelemetry.isGroundEffect = isIge;

    double etlFactor { 1.0 };
    if (airspeed > mConfig.etlSpeedThreshold && sinNacelle > 0.5) {
        const double excessSpeed { airspeed - mConfig.etlSpeedThreshold };
        etlFactor = 1.0 + (0.15 * std::min(1.0, excessSpeed / 12.0) * sinNacelle);
    }

    const double netThrust { baseThrust * igeFactor * etlFactor };
    mTelemetry.rotorThrust = netThrust;

    // 5. Proprotor thrust vectoring in body frame
    const math::Vec2 localRotorThrust { netThrust * cosNacelle, netThrust * sinNacelle };
    body.applyForce(body.toWorldVec(localRotorThrust));

    // 6. Dynamic pressure & wing aerodynamics
    const double dynamicPressure { 0.5 * mConfig.airDensity * airspeed * airspeed };
    mTelemetry.dynamicPressure = dynamicPressure;

    double alpha { 0.0 };
    if (airspeed > 0.05) {
        alpha = -std::atan2(localVel.y, localVel.x);
    }
    mTelemetry.angleOfAttack = alpha;

    const bool isWingStalled { std::abs(alpha) > mConfig.stallAngle };
    mTelemetry.isWingStalled = isWingStalled;

    const double flaps { std::clamp(mInput.flaps, 0.0, 1.0) };
    double liftCoeff { 0.0 };
    if (!isWingStalled) {
        liftCoeff = (mConfig.liftSlope * alpha) + (mConfig.flapLiftCoeff * flaps);
    } else {
        const double deltaAlpha { std::abs(alpha) - mConfig.stallAngle };
        const double peakLift { (mConfig.liftSlope * mConfig.stallAngle) + (mConfig.flapLiftCoeff * flaps) };
        const double decayedPeak { peakLift * std::exp(-2.5 * deltaAlpha) };
        const double flatPlateLift { 0.6 * std::sin(2.0 * alpha) };
        liftCoeff = std::copysign(decayedPeak + flatPlateLift, alpha);
    }

    const double inducedDrag { mConfig.inducedDragFactor * liftCoeff * liftCoeff };
    const double flapDrag { mConfig.flapDragCoeff * flaps };
    double dragCoeff { mConfig.zeroLiftDrag + inducedDrag + flapDrag };
    if (isWingStalled) {
        dragCoeff += 1.2 * std::sin(alpha) * std::sin(alpha);
    }

    double wingLift { 0.0 };
    double wingDrag { 0.0 };
    if (airspeed > 0.05) {
        wingLift = dynamicPressure * mConfig.wingArea * liftCoeff;
        wingDrag = dynamicPressure * mConfig.wingArea * dragCoeff;

        const math::Vec2 velUnit { localVel.x / airspeed, localVel.y / airspeed };
        const math::Vec2 liftUnit { -velUnit.y, velUnit.x };

        const math::Vec2 localLift { liftUnit * wingLift };
        const math::Vec2 localDrag { velUnit * (-wingDrag) };
        body.applyForce(body.toWorldVec(localLift + localDrag));
    }
    mTelemetry.wingLift = wingLift;
    mTelemetry.wingDrag = wingDrag;

    // 7. Wing download interference penalty in hover
    const double downloadForce { netThrust * mConfig.downloadFactor * sinNacelle * sinNacelle
        * (1.0 - (0.45 * flaps)) };
    mTelemetry.wingDownload = downloadForce;
    if (downloadForce > 0.0) {
        body.applyForce(body.toWorldVec(math::Vec2 { 0.0, -downloadForce }));
    }

    // 8. Conversion corridor evaluation
    const double minSpeed { mConfig.stallAirspeed * std::sqrt(std::max(0.0, 1.0 - sinNacelle)) };
    const double maxSpeed { mConfig.maxCruiseAirspeed
        - ((mConfig.maxCruiseAirspeed - mConfig.maxHoverAirspeed) * sinNacelle) };
    mTelemetry.minCorridorSpeed = minSpeed;
    mTelemetry.maxCorridorSpeed = maxSpeed;
    mTelemetry.isCorridorSafe = (airspeed >= minSpeed) && (airspeed <= maxSpeed);

    // 9. Control cross-fading & pitch moments
    const double qRef { 0.5 * mConfig.airDensity * 35.0 * 35.0 }; // q at 35 m/s
    const double qRatio { std::clamp(dynamicPressure / qRef, 0.0, 2.5) };
    const double aeroBlend { cosNacelle * std::min(1.0, qRatio) };

    const double pitchCmd { std::clamp(mInput.pitchStick, -1.0, 1.0) };
    const double cyclicTorque { -pitchCmd * mConfig.cyclicPitchAuth * rpmRatio };
    const double elevatorTorque { pitchCmd * mConfig.elevatorPitchAuth * qRatio };
    const double blendedControlTorque { ((1.0 - aeroBlend) * cyclicTorque) + (aeroBlend * elevatorTorque) };

    const double weathercockTorque { -mConfig.weathercockStab * alpha * aeroBlend };
    const double pitchDampingTorque { -mConfig.pitchDamping * omega * std::max(0.2, std::sqrt(qRatio)) };

    // Nacelle CG shift moment trim
    const double cgOffset { mConfig.nacelleMastLength * cosNacelle };
    const double nacelleCgTorque { mConfig.nacelleMass * 9.80665 * cgOffset };

    const double netPitchTorque { blendedControlTorque + weathercockTorque + pitchDampingTorque + nacelleCgTorque };
    body.applyTorque(netPitchTorque);

    // 10. Environmental gravity acceleration
    if (body.getMass() > 0.0) {
        body.applyForce(mConfig.gravity * body.getMass());
    }
}

void VTOLTiltRotorDynamics::reset() noexcept
{
    mInput = VTOLTiltRotorInput { };
    mTelemetry = VTOLTiltRotorTelemetry { };
    mRotorRpm = mConfig.ratedRotorRpm;
    mActiveNacelleAngle = mConfig.maxNacelleAngle;
    mTiltLimiter.reset(mActiveNacelleAngle);

    mTelemetry.rotorRpm = mRotorRpm;
    mTelemetry.nacelleAngleRad = mActiveNacelleAngle;
    mTelemetry.nacelleAngleDeg = mActiveNacelleAngle * kRadToDeg;
    mTelemetry.flightMode = VTOLFlightMode::Hover;
    mTelemetry.isCorridorSafe = true;
}

bool VTOLTiltRotorDynamics::isAirborne() const noexcept
{
    return (mInput.groundDistance > 0.5);
}

const VTOLTiltRotorConfig& VTOLTiltRotorDynamics::getConfig() const noexcept
{
    return mConfig;
}

void VTOLTiltRotorDynamics::setConfig(const VTOLTiltRotorConfig& config) noexcept
{
    mConfig = config;
    mTiltLimiter.setRateLimit(config.maxTiltSlewRate);
}

const VTOLTiltRotorInput& VTOLTiltRotorDynamics::getInput() const noexcept
{
    return mInput;
}

void VTOLTiltRotorDynamics::setInput(const VTOLTiltRotorInput& input) noexcept
{
    mInput = input;
}

const VTOLTiltRotorTelemetry& VTOLTiltRotorDynamics::getTelemetry() const noexcept
{
    return mTelemetry;
}

double VTOLTiltRotorDynamics::getNacelleAngle() const noexcept
{
    return mActiveNacelleAngle;
}

double VTOLTiltRotorDynamics::getNacelleDeg() const noexcept
{
    return mActiveNacelleAngle * kRadToDeg;
}

double VTOLTiltRotorDynamics::getAirspeed() const noexcept
{
    return mTelemetry.airspeed;
}

double VTOLTiltRotorDynamics::getRotorThrust() const noexcept
{
    return mTelemetry.rotorThrust;
}

double VTOLTiltRotorDynamics::getWingLift() const noexcept
{
    return mTelemetry.wingLift;
}

bool VTOLTiltRotorDynamics::isStalled() const noexcept
{
    return mTelemetry.isWingStalled;
}

bool VTOLTiltRotorDynamics::isCorridorSafe() const noexcept
{
    return mTelemetry.isCorridorSafe;
}

VTOLFlightMode VTOLTiltRotorDynamics::getFlightMode() const noexcept
{
    return mTelemetry.flightMode;
}

} // namespace vehicle_sim::dynamics
