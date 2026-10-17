#include "StratifiedSoil.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::terrain {

namespace {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kEpsilon = 1e-7;
    constexpr double kGravity = 9.81;
    constexpr double kSpeedCutoff = 0.10;
} // namespace

double StratifiedSoilSolver::computeModulus(
    double depth, double width, const StratifiedSoilConfig& config) noexcept
{
    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double k1 = BekkerWongSolver::computeModulus(effectiveWidth, config.topLayer.soil);

    if (!config.enableStratification) {
        return k1;
    }

    const double k2 = BekkerWongSolver::computeModulus(effectiveWidth, config.substrate.soil);
    const double dTop = (config.topLayer.layerDepth > kEpsilon) ? config.topLayer.layerDepth : kEpsilon;
    const double delta = (config.transitionZone > kEpsilon) ? config.transitionZone : 0.02;

    const double lowerBound = dTop - delta;
    const double span = 2.0 * delta;
    const double u = std::clamp((depth - lowerBound) / span, 0.0, 1.0);
    const double hermite = (3.0 * u * u) - (2.0 * u * u * u);

    return ((1.0 - hermite) * k1) + (hermite * k2);
}

StratifiedSinkageResult StratifiedSoilSolver::computeSinkage(
    double pressure, double width, const StratifiedSoilConfig& config) noexcept
{
    StratifiedSinkageResult result {};
    if (pressure <= 0.0) {
        return result;
    }

    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double k1 = BekkerWongSolver::computeModulus(effectiveWidth, config.topLayer.soil);
    if (k1 <= kEpsilon || config.topLayer.soil.sinkageExponent <= kEpsilon) {
        return result;
    }

    const double n1 = config.topLayer.soil.sinkageExponent;
    const double z1Raw = std::pow(pressure / k1, 1.0 / n1);
    const double dTop = (config.topLayer.layerDepth > kEpsilon) ? config.topLayer.layerDepth : kEpsilon;

    if (!config.enableStratification) {
        const double clampedZ = std::clamp(z1Raw, 0.0, config.topLayer.soil.maxPlasticDepth);
        result.totalSinkage = clampedZ;
        result.topLayerSinkage = clampedZ;
        result.substrateSinkage = 0.0;
        result.reachedSubstrate = false;
        result.effectiveModulus = k1;
        return result;
    }

    if (z1Raw <= dTop) {
        result.totalSinkage = z1Raw;
        result.topLayerSinkage = z1Raw;
        result.substrateSinkage = 0.0;
        result.reachedSubstrate = false;
        result.effectiveModulus = computeModulus(z1Raw, effectiveWidth, config);
        return result;
    }

    // Punched through top layer into substrate
    result.reachedSubstrate = true;
    result.topLayerSinkage = dTop;

    const double pCapacity1 = k1 * std::pow(dTop, n1);
    const double pExcess = (pressure > pCapacity1) ? (pressure - pCapacity1) : 0.0;

    double deltaZ2 = 0.0;
    const double k2 = BekkerWongSolver::computeModulus(effectiveWidth, config.substrate.soil);
    const double n2 = config.substrate.soil.sinkageExponent;

    if (pExcess > 0.0 && k2 > kEpsilon && n2 > kEpsilon) {
        const double rawDelta2 = std::pow(pExcess / k2, 1.0 / n2);
        deltaZ2 = std::clamp(rawDelta2, 0.0, config.substrate.soil.maxPlasticDepth);
    }

    result.substrateSinkage = deltaZ2;
    result.totalSinkage = dTop + deltaZ2;
    result.effectiveModulus = computeModulus(result.totalSinkage, effectiveWidth, config);

    return result;
}

double StratifiedSoilSolver::computeCompactionDrag(
    double sinkage, double width, const StratifiedSoilConfig& config) noexcept
{
    if (sinkage <= 0.0) {
        return 0.0;
    }

    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double k1 = BekkerWongSolver::computeModulus(effectiveWidth, config.topLayer.soil);
    const double n1 = config.topLayer.soil.sinkageExponent;
    const double dTop = (config.topLayer.layerDepth > kEpsilon) ? config.topLayer.layerDepth : kEpsilon;

    if (!config.enableStratification || sinkage <= dTop) {
        return (effectiveWidth * k1 / (n1 + 1.0)) * std::pow(sinkage, n1 + 1.0);
    }

    // Two-layer compaction work
    const double rc1 = (effectiveWidth * k1 / (n1 + 1.0)) * std::pow(dTop, n1 + 1.0);
    const double k2 = BekkerWongSolver::computeModulus(effectiveWidth, config.substrate.soil);
    const double n2 = config.substrate.soil.sinkageExponent;
    const double z2 = sinkage - dTop;

    double rc2 = 0.0;
    if (k2 > kEpsilon && n2 > kEpsilon && z2 > 0.0) {
        rc2 = (effectiveWidth * k2 / (n2 + 1.0)) * std::pow(z2, n2 + 1.0);
    }

    return rc1 + rc2;
}

double StratifiedSoilSolver::computeBulldozeDrag(
    double sinkage, double width, const StratifiedSoilConfig& config) noexcept
{
    if (sinkage <= 0.0) {
        return 0.0;
    }

    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double dTop = (config.topLayer.layerDepth > kEpsilon) ? config.topLayer.layerDepth : kEpsilon;
    const double zBulldoze = config.enableStratification ? std::min(sinkage, dTop) : sinkage;

    const double gamma = config.topLayer.soil.bulkDensity * kGravity;
    const double phi = std::clamp(config.topLayer.soil.frictionAngle, 0.01, 1.50);

    const double tanAngle = std::tan(0.25 * kPi + 0.5 * phi);
    const double kp = tanAngle * tanAngle;
    const double kpGamma = (2.0 * kp) / (std::tan(phi) + kEpsilon);
    const double kpC = 2.0 * std::sqrt(kp);

    const double rb = (0.5 * effectiveWidth * gamma * (zBulldoze * zBulldoze) * kpGamma)
        + (2.0 * zBulldoze * config.topLayer.soil.cohesion * effectiveWidth * kpC);

    return (rb > 0.0) ? rb : 0.0;
}

StratifiedSinkageResult StratifiedSoilSolver::evaluateStamp(
    const RutStamp2D& stamp, const StratifiedSoilConfig& config) noexcept
{
    StratifiedSinkageResult result {};
    if (stamp.normalForce <= 0.0) {
        return result;
    }

    const double width = (stamp.width > 0.01) ? stamp.width : 0.01;
    const double length = (stamp.length > 0.01) ? stamp.length : 0.01;
    const double area = width * length;
    const double pressure = stamp.normalForce / area;

    result = computeSinkage(pressure, width, config);

    const double rc = computeCompactionDrag(result.totalSinkage, width, config);
    const double rb = computeBulldozeDrag(result.totalSinkage, width, config);
    const double totalDrag = rc + rb;

    math::Vec2 dragVector { 0.0, 0.0 };
    const double speed = stamp.velocity.length();
    if (speed > kEpsilon) {
        const double factor = -totalDrag / (speed + kSpeedCutoff);
        dragVector = stamp.velocity * factor;
    }

    result.compactionDrag = rc;
    result.bulldozeDrag = rb;
    result.dragForce = dragVector;

    return result;
}

StratifiedSoilConfig StratifiedSoilSolver::getMudOnBedrock() noexcept
{
    StratifiedSoilConfig config {};
    config.topLayer.soil = SurfaceMaterial::getPreset(MaterialType::Mud).soil;
    config.topLayer.layerDepth = 0.12;
    config.topLayer.bearingCapacity = 100000.0;
    config.topLayer.name = "ViscousMud";

    config.substrate.soil = SurfaceMaterial::getPreset(MaterialType::Concrete).soil;
    config.substrate.soil.cohesionModulus = 50000.0;
    config.substrate.soil.frictionModulus = 2000000.0;
    config.substrate.soil.sinkageExponent = 1.0;
    config.substrate.soil.maxPlasticDepth = 0.01;
    config.substrate.soil.isDeformable = false;
    config.substrate.layerDepth = 1.0;
    config.substrate.bearingCapacity = 5000000.0;
    config.substrate.name = "Bedrock";

    config.transitionZone = 0.02;
    config.enableStratification = true;
    return config;
}

StratifiedSoilConfig StratifiedSoilSolver::getSnowOnAsphalt() noexcept
{
    StratifiedSoilConfig config {};
    config.topLayer.soil = SurfaceMaterial::getPreset(MaterialType::Snow).soil;
    config.topLayer.layerDepth = 0.15;
    config.topLayer.bearingCapacity = 80000.0;
    config.topLayer.name = "FreshSnow";

    config.substrate.soil = SurfaceMaterial::getPreset(MaterialType::Tarmac).soil;
    config.substrate.soil.cohesionModulus = 30000.0;
    config.substrate.soil.frictionModulus = 1500000.0;
    config.substrate.soil.sinkageExponent = 1.0;
    config.substrate.soil.maxPlasticDepth = 0.005;
    config.substrate.soil.isDeformable = false;
    config.substrate.layerDepth = 0.50;
    config.substrate.bearingCapacity = 4000000.0;
    config.substrate.name = "AsphaltPavement";

    config.transitionZone = 0.02;
    config.enableStratification = true;
    return config;
}

} // namespace vehicle_sim::terrain
