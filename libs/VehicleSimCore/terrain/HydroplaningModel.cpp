#include "HydroplaningModel.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

namespace {
    constexpr double kEpsilon = 1e-7;
    constexpr double kSpeedCutoff = 0.10;
    constexpr double kSaturationDepth = 0.004; // 4mm excess water for full lift saturation
} // namespace

double HydroplaningSolver::computeCriticalSpeed(
    double pressurePsi, double horneMultiplier) noexcept
{
    if (pressurePsi <= 0.0) {
        return 0.0;
    }
    const double mult = (horneMultiplier > 0.0) ? horneMultiplier : 6.36;
    return mult * std::sqrt(pressurePsi);
}

double HydroplaningSolver::computeUnDrainedDepth(
    double waterDepth, double treadDepth, double voidRatio) noexcept
{
    if (waterDepth <= 0.0) {
        return 0.0;
    }
    const double effectiveTread = (treadDepth > 0.0) ? treadDepth : 0.0;
    const double effectiveVoid = std::clamp(voidRatio, 0.0, 1.0);
    const double drainageCapacity = effectiveTread * effectiveVoid;

    const double excess = waterDepth - drainageCapacity;
    return (excess > 0.0) ? excess : 0.0;
}

double HydroplaningSolver::computeHydroLift(
    double forwardSpeed,
    double unDrainedDepth,
    double patchArea,
    const HydroplaningConfig& config) noexcept
{
    if (forwardSpeed <= 0.0 || unDrainedDepth <= kEpsilon || patchArea <= 0.0) {
        return 0.0;
    }

    const double filmRatio = std::clamp(unDrainedDepth / kSaturationDepth, 0.0, 1.0);
    const double lift = 0.5 * config.liftCoefficient * config.waterDensity
        * (forwardSpeed * forwardSpeed) * patchArea * filmRatio;

    return (lift > 0.0) ? lift : 0.0;
}

double HydroplaningSolver::computeFrictionScale(
    double forwardSpeed,
    double criticalSpeed,
    double unDrainedDepth,
    const HydroplaningConfig& config) noexcept
{
    if (forwardSpeed <= kEpsilon || unDrainedDepth <= kEpsilon || criticalSpeed <= kEpsilon) {
        return 1.0;
    }

    const double ratio = forwardSpeed / criticalSpeed;
    if (ratio < 0.20) {
        return 1.0;
    }

    const double minScale = std::clamp(config.minFrictionScale, 0.01, 1.0);
    const double exponent = std::clamp(config.hydroSharpness * (ratio - 1.0), -30.0, 30.0);
    const double logisticDecay = 1.0 / (1.0 + std::exp(exponent));
    const double rawScale = minScale + ((1.0 - minScale) * logisticDecay);

    // Film presence modulation: scale activates progressively as unDrainedDepth reaches 1mm
    const double filmPresence = std::clamp(unDrainedDepth / 0.001, 0.0, 1.0);
    const double finalScale = 1.0 - (filmPresence * (1.0 - rawScale));

    return std::clamp(finalScale, minScale, 1.0);
}

double HydroplaningSolver::computeSprayDrag(
    double forwardSpeed,
    double waterDepth,
    double patchWidth,
    const HydroplaningConfig& config) noexcept
{
    if (forwardSpeed <= kEpsilon || waterDepth <= kEpsilon || patchWidth <= kEpsilon) {
        return 0.0;
    }

    const double drag = 0.5 * config.sprayDragCoeff * config.waterDensity
        * patchWidth * waterDepth * (forwardSpeed * forwardSpeed);

    return (drag > 0.0) ? drag : 0.0;
}

HydroplaningResult HydroplaningSolver::evaluateHydroplaning(
    const HydroplaningState& state, const HydroplaningConfig& config) noexcept
{
    HydroplaningResult result {};

    const double criticalSpeed = computeCriticalSpeed(config.tirePressurePsi, config.horneMultiplier);
    const double unDrainedDepth = computeUnDrainedDepth(state.waterDepth, config.treadDepth, config.voidRatio);

    const double width = (state.patchWidth > 0.01) ? state.patchWidth : 0.01;
    const double length = (state.patchLength > 0.01) ? state.patchLength : 0.01;
    const double patchArea = width * length;

    const double speed = std::abs(state.forwardVelocity);
    const double liftForce = computeHydroLift(speed, unDrainedDepth, patchArea, config);
    const double effNormal = (state.normalLoad > liftForce) ? (state.normalLoad - liftForce) : 0.0;

    const double frictionScale = computeFrictionScale(speed, criticalSpeed, unDrainedDepth, config);
    const double sprayDrag = computeSprayDrag(speed, state.waterDepth, width, config);

    math::Vec2 sprayVector { 0.0, 0.0 };
    const double velLength = state.velocityVector.length();
    if (velLength > kEpsilon && sprayDrag > 0.0) {
        const double factor = -sprayDrag / (velLength + kSpeedCutoff);
        sprayVector = state.velocityVector * factor;
    }

    result.criticalSpeed = criticalSpeed;
    result.unDrainedDepth = unDrainedDepth;
    result.hydroLiftForce = liftForce;
    result.effectiveNormalLoad = effNormal;
    result.sprayDragForce = sprayDrag;
    result.sprayDragVector = sprayVector;
    result.frictionScale = frictionScale;
    result.isHydroplaning = (speed >= criticalSpeed) && (unDrainedDepth > 0.0005);

    return result;
}

} // namespace vehicle_sim::terrain
