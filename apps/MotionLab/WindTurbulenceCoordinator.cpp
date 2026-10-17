#include "WindTurbulenceCoordinator.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

namespace {

constexpr double kPi { 3.14159265358979323846 };
constexpr double kDegToRad { kPi / 180.0 };
constexpr double kRadToDeg { 180.0 / kPi };
constexpr double kMpsToMph { 2.2369362920544 };

} // namespace

WindTurbulenceCoordinator::WindTurbulenceCoordinator() noexcept
{
    mFluidField.setWaterline(-10000.0);
    applyWindPreset(vehicle_sim::physics::WindEnvironmentPreset::Calm);
}

void WindTurbulenceCoordinator::stepPhysics(
    vehicle_sim::physics::RigidBody2D& body,
    MotionLabVehicleType vehicleType,
    double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    mSimulationTime += dt;

    if (mGustDuration > 0.0) {
        mGustTimer += dt;
        if (mGustTimer >= mGustDuration) {
            mGustTimer = 0.0;
            mGustDuration = 0.0;
        }
    }

    if (mSnapshot.activePreset == vehicle_sim::physics::WindEnvironmentPreset::DynamicStorm) {
        mWindHeadingDeg = std::fmod(mWindHeadingDeg + (dt * 3.5), 360.0);
        updateWindVector();
    }

    const vehicle_sim::math::Vec2 vehPos { body.getPosition() };
    const vehicle_sim::math::Vec2 vehVel { body.getLinearVelocity() };
    const double vehHeading { body.getOrientation() };

    vehicle_sim::physics::FluidSample2D sample { };
    if (mWindActive) {
        sample = mFluidField.sample(vehPos, mSimulationTime);
    } else {
        sample.velocity = vehicle_sim::math::Vec2 { 0.0, 0.0 };
        sample.density = 1.225;
        sample.medium = vehicle_sim::physics::FluidMediumType::Air;
    }

    const auto props = getAeroProps(vehicleType);
    const double downforceCoeff = getDownforceCoeff(vehicleType);

    const auto aero = vehicle_sim::physics::FluidDrag2D::computeAeroForces(
        vehVel, vehHeading, sample, props, downforceCoeff);

    if (mWindActive) {
        body.applyForce(aero.totalForceWorld);
        body.applyTorque(aero.yawMomentAroundCm);
    }

    const vehicle_sim::math::Vec2 ambientWind { mFluidField.getConfig().ambientWind };
    const vehicle_sim::math::Vec2 gustPerturb { sample.velocity - ambientWind };

    const double trueSpeed { sample.velocity.length() };
    double trueHeadingDeg { 0.0 };
    if (trueSpeed > 1e-4) {
        trueHeadingDeg = std::fmod(
            std::atan2(-sample.velocity.x, -sample.velocity.y) * kRadToDeg + 360.0, 360.0);
    }

    const double apparentWindDeg { aero.sideslipAngleRad * kRadToDeg };
    const double mass = body.getMass();
    const double crosswindGripLimit { 0.35 * mass * 9.81 };
    const bool alert { std::abs(aero.latSideForce) > crosswindGripLimit };

    mSnapshot.windVelocity = sample.velocity;
    mSnapshot.gustPerturbation = gustPerturb;
    mSnapshot.trueWindSpeed = trueSpeed;
    mSnapshot.trueWindHeadingDeg = trueHeadingDeg;
    mSnapshot.relativeAirspeed = aero.relativeAirspeed;
    mSnapshot.relativeAirspeedMph = aero.relativeAirspeed * kMpsToMph;
    mSnapshot.apparentWindAngleDeg = apparentWindDeg;
    mSnapshot.lateralAeroForce = aero.latSideForce;
    mSnapshot.yawOverturningMoment = aero.yawMomentAroundCm;
    mSnapshot.downforceLoad = aero.downforceLoad;
    mSnapshot.crosswindAlert = alert;
    mSnapshot.gustPulseActive = (mGustDuration > 0.0);
}

void WindTurbulenceCoordinator::setPreset(
    vehicle_sim::physics::WindEnvironmentPreset preset) noexcept
{
    applyWindPreset(preset);
}

void WindTurbulenceCoordinator::cycleNextPreset() noexcept
{
    using Preset = vehicle_sim::physics::WindEnvironmentPreset;
    Preset next { Preset::Calm };
    switch (mSnapshot.activePreset) {
    case Preset::Calm:
        next = Preset::LightBreeze;
        break;
    case Preset::LightBreeze:
        next = Preset::CrosswindGale;
        break;
    case Preset::CrosswindGale:
        next = Preset::SevereStorm;
        break;
    case Preset::SevereStorm:
        next = Preset::Microburst;
        break;
    case Preset::Microburst:
        next = Preset::DynamicStorm;
        break;
    case Preset::DynamicStorm:
        next = Preset::Calm;
        break;
    }
    applyWindPreset(next);
}

void WindTurbulenceCoordinator::triggerDiscreteGust(
    double peakVelocity, double durationPulse) noexcept
{
    if (peakVelocity <= 0.0 || durationPulse <= 0.0) {
        return;
    }

    const double rad { mWindHeadingDeg * kDegToRad };
    const vehicle_sim::math::Vec2 gustDir { -std::sin(rad), -std::cos(rad) };

    mFluidField.triggerGust(gustDir, peakVelocity, durationPulse, mSimulationTime);
    mGustTimer = 0.0;
    mGustDuration = durationPulse;
}

void WindTurbulenceCoordinator::adjustWindSpeed(double scaleFactor) noexcept
{
    if (scaleFactor <= 0.0) {
        return;
    }
    mWindSpeed = std::clamp(mWindSpeed * scaleFactor, 0.0, 50.0);
    updateWindVector();
}

void WindTurbulenceCoordinator::rotateWindHeading(double deltaDegrees) noexcept
{
    mWindHeadingDeg = std::fmod(mWindHeadingDeg + deltaDegrees + 360.0, 360.0);
    updateWindVector();
}

void WindTurbulenceCoordinator::toggleWindActive() noexcept
{
    mWindActive = !mWindActive;
}

void WindTurbulenceCoordinator::setWindActive(bool active) noexcept
{
    mWindActive = active;
}

bool WindTurbulenceCoordinator::isWindActive() const noexcept
{
    return mWindActive;
}

const WindTelemetrySnapshot& WindTurbulenceCoordinator::getSnapshot() const noexcept
{
    return mSnapshot;
}

vehicle_sim::math::Vec2 WindTurbulenceCoordinator::sampleWindAt(
    const vehicle_sim::math::Vec2& worldPos) const noexcept
{
    if (!mWindActive) {
        return vehicle_sim::math::Vec2 { 0.0, 0.0 };
    }
    return mFluidField.getVelocity(worldPos, mSimulationTime);
}

void WindTurbulenceCoordinator::reset() noexcept
{
    mFluidField.clearGusts();
    mFluidField.setWaterline(-10000.0);
    mGustTimer = 0.0;
    mGustDuration = 0.0;
    mSimulationTime = 0.0;
    applyWindPreset(vehicle_sim::physics::WindEnvironmentPreset::Calm);
}

vehicle_sim::physics::FluidDragProperties WindTurbulenceCoordinator::getAeroProps(
    MotionLabVehicleType type) const noexcept
{
    vehicle_sim::physics::FluidDragProperties props { };
    switch (type) {
    case MotionLabVehicleType::ClassicCar:
        props.frontalArea = 2.20;
        props.frontalDragCoeff = 0.32;
        props.sideArea = 6.80;
        props.sideDragCoeff = 1.05;
        props.cpOffset = -0.15;
        break;
    case MotionLabVehicleType::RaceCar:
        props.frontalArea = 1.65;
        props.frontalDragCoeff = 0.38;
        props.sideArea = 4.50;
        props.sideDragCoeff = 0.95;
        props.cpOffset = -0.45;
        break;
    case MotionLabVehicleType::Rocket:
        props.frontalArea = 1.80;
        props.frontalDragCoeff = 0.22;
        props.sideArea = 14.50;
        props.sideDragCoeff = 1.15;
        props.cpOffset = -1.80;
        break;
    case MotionLabVehicleType::Helicopter:
        props.frontalArea = 3.50;
        props.frontalDragCoeff = 0.45;
        props.sideArea = 11.20;
        props.sideDragCoeff = 1.20;
        props.cpOffset = 0.10;
        break;
    case MotionLabVehicleType::Hovercraft:
        props.frontalArea = 2.80;
        props.frontalDragCoeff = 0.40;
        props.sideArea = 8.20;
        props.sideDragCoeff = 1.10;
        props.cpOffset = 0.05;
        break;
    case MotionLabVehicleType::TrackedTank:
        props.frontalArea = 5.80;
        props.frontalDragCoeff = 0.75;
        props.sideArea = 18.50;
        props.sideDragCoeff = 1.35;
        props.cpOffset = -0.10;
        break;
    case MotionLabVehicleType::AntiGravity:
        props.frontalArea = 1.90;
        props.frontalDragCoeff = 0.25;
        props.sideArea = 5.20;
        props.sideDragCoeff = 0.85;
        props.cpOffset = -0.30;
        break;
    case MotionLabVehicleType::DriftCar:
        props.frontalArea = 2.15;
        props.frontalDragCoeff = 0.34;
        props.sideArea = 6.90;
        props.sideDragCoeff = 1.08;
        props.cpOffset = 0.12;
        break;
    case MotionLabVehicleType::MonsterTruck:
        props.frontalArea = 6.20;
        props.frontalDragCoeff = 0.68;
        props.sideArea = 16.50;
        props.sideDragCoeff = 1.25;
        props.cpOffset = 0.25;
        break;
    case MotionLabVehicleType::SemiTruck:
        props.frontalArea = 8.50;
        props.frontalDragCoeff = 0.55;
        props.sideArea = 38.00;
        props.sideDragCoeff = 1.40;
        props.cpOffset = 0.85;
        break;
    case MotionLabVehicleType::Forklift:
        props.frontalArea = 2.40;
        props.frontalDragCoeff = 0.65;
        props.sideArea = 5.80;
        props.sideDragCoeff = 1.15;
        props.cpOffset = 0.40;
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        props.frontalArea = 0.15;
        props.frontalDragCoeff = 0.50;
        props.sideArea = 0.25;
        props.sideDragCoeff = 0.80;
        props.cpOffset = 0.00;
        break;
    }
    return props;
}

double WindTurbulenceCoordinator::getDownforceCoeff(
    MotionLabVehicleType type) const noexcept
{
    switch (type) {
    case MotionLabVehicleType::RaceCar:
        return 2.85;
    case MotionLabVehicleType::AntiGravity:
        return 1.40;
    case MotionLabVehicleType::ClassicCar:
        return 0.10;
    case MotionLabVehicleType::DriftCar:
        return 0.25;
    default:
        return 0.0;
    }
}

void WindTurbulenceCoordinator::applyWindPreset(
    vehicle_sim::physics::WindEnvironmentPreset preset) noexcept
{
    mSnapshot.activePreset = preset;
    mFluidField.clearGusts();
    mGustTimer = 0.0;
    mGustDuration = 0.0;

    vehicle_sim::physics::DynamicGustConfig turb { };

    switch (preset) {
    case vehicle_sim::physics::WindEnvironmentPreset::Calm:
        mWindSpeed = 0.0;
        mWindHeadingDeg = 0.0;
        turb.enableTurbulence = false;
        break;
    case vehicle_sim::physics::WindEnvironmentPreset::LightBreeze:
        mWindSpeed = 5.0;
        mWindHeadingDeg = 90.0;
        turb.enableTurbulence = true;
        turb.turbulenceIntensity = 0.10;
        turb.integralLengthScale = 20.0;
        turb.temporalFrequency = 0.15;
        turb.randomSeed = 2001U;
        break;
    case vehicle_sim::physics::WindEnvironmentPreset::CrosswindGale:
        mWindSpeed = 15.0;
        mWindHeadingDeg = 315.0;
        turb.enableTurbulence = true;
        turb.turbulenceIntensity = 0.18;
        turb.integralLengthScale = 25.0;
        turb.temporalFrequency = 0.20;
        turb.randomSeed = 1996U;
        break;
    case vehicle_sim::physics::WindEnvironmentPreset::SevereStorm:
        mWindSpeed = 25.0;
        mWindHeadingDeg = 270.0;
        turb.enableTurbulence = true;
        turb.turbulenceIntensity = 0.28;
        turb.integralLengthScale = 30.0;
        turb.temporalFrequency = 0.25;
        turb.randomSeed = 1080U;
        break;
    case vehicle_sim::physics::WindEnvironmentPreset::Microburst:
        mWindSpeed = 38.0;
        mWindHeadingDeg = 180.0;
        turb.enableTurbulence = true;
        turb.turbulenceIntensity = 0.35;
        turb.integralLengthScale = 15.0;
        turb.temporalFrequency = 0.30;
        turb.randomSeed = 4096U;
        break;
    case vehicle_sim::physics::WindEnvironmentPreset::DynamicStorm:
        mWindSpeed = 20.0;
        mWindHeadingDeg = 240.0;
        turb.enableTurbulence = true;
        turb.turbulenceIntensity = 0.25;
        turb.integralLengthScale = 28.0;
        turb.temporalFrequency = 0.22;
        turb.randomSeed = 7777U;
        break;
    }

    mFluidField.setTurbulence(turb);
    updateWindVector();
}

void WindTurbulenceCoordinator::updateWindVector() noexcept
{
    const double rad { mWindHeadingDeg * kDegToRad };
    const vehicle_sim::math::Vec2 ambientFlow {
        -mWindSpeed * std::sin(rad),
        -mWindSpeed * std::cos(rad)
    };
    mFluidField.setAmbientWind(ambientFlow);
    mSnapshot.trueWindSpeed = mWindSpeed;
    mSnapshot.trueWindHeadingDeg = mWindHeadingDeg;
    mSnapshot.windVelocity = ambientFlow;
}

} // namespace motion_lab
