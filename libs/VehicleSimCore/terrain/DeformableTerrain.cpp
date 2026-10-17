#include "DeformableTerrain.h"
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

double BekkerWongSolver::computeModulus(double width, const SoilMechanicsConfig& soil) noexcept
{
    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double keq = (soil.cohesionModulus / effectiveWidth) + soil.frictionModulus;
    return (keq > 0.0) ? keq : 0.0;
}

double BekkerWongSolver::computeSinkage(double pressure, double width, const SoilMechanicsConfig& soil) noexcept
{
    if (!soil.isDeformable || pressure <= 0.0 || soil.sinkageExponent <= kEpsilon) {
        return 0.0;
    }

    const double keq = computeModulus(width, soil);
    if (keq <= kEpsilon) {
        return 0.0;
    }

    const double ratio = pressure / keq;
    const double exponent = 1.0 / soil.sinkageExponent;
    const double z = std::pow(ratio, exponent);

    return std::clamp(z, 0.0, soil.maxPlasticDepth);
}

double BekkerWongSolver::computeCompactionDrag(double sinkage, double width, const SoilMechanicsConfig& soil) noexcept
{
    if (!soil.isDeformable || sinkage <= 0.0) {
        return 0.0;
    }

    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double keq = computeModulus(effectiveWidth, soil);
    const double n = soil.sinkageExponent;
    const double Rc = (effectiveWidth * keq / (n + 1.0)) * std::pow(sinkage, n + 1.0);

    return (Rc > 0.0) ? Rc : 0.0;
}

double BekkerWongSolver::computeBulldozeDrag(double sinkage, double width, const SoilMechanicsConfig& soil) noexcept
{
    if (!soil.isDeformable || sinkage <= 0.0) {
        return 0.0;
    }

    const double effectiveWidth = (width > kEpsilon) ? width : kEpsilon;
    const double gamma = soil.bulkDensity * kGravity;
    const double phi = std::clamp(soil.frictionAngle, 0.01, 1.50);

    const double tanAngle = std::tan(0.25 * kPi + 0.5 * phi);
    const double Kp = tanAngle * tanAngle;
    const double KpGamma = (2.0 * Kp) / (std::tan(phi) + kEpsilon);
    const double KpC = 2.0 * std::sqrt(Kp);

    const double Rb = (0.5 * effectiveWidth * gamma * (sinkage * sinkage) * KpGamma)
        + (2.0 * sinkage * soil.cohesion * effectiveWidth * KpC);

    return (Rb > 0.0) ? Rb : 0.0;
}

SoilSinkageResult BekkerWongSolver::evaluateStamp(const RutStamp2D& stamp, const SoilMechanicsConfig& soil) noexcept
{
    SoilSinkageResult result {};
    if (!soil.isDeformable || stamp.normalForce <= 0.0) {
        return result;
    }

    const double width = (stamp.width > 0.01) ? stamp.width : 0.01;
    const double length = (stamp.length > 0.01) ? stamp.length : 0.01;
    const double area = width * length;
    const double pressure = stamp.normalForce / area;

    const double z = computeSinkage(pressure, width, soil);
    const double zp = z * std::clamp(soil.plasticRatio, 0.0, 1.0);

    const double Rc = computeCompactionDrag(z, width, soil);
    const double Rb = computeBulldozeDrag(z, width, soil);
    const double totalDrag = Rc + Rb;

    math::Vec2 dragVector { 0.0, 0.0 };
    const double speed = stamp.velocity.length();
    if (speed > kEpsilon) {
        const double factor = -totalDrag / (speed + kSpeedCutoff);
        dragVector = stamp.velocity * factor;
    }

    const double bermWidth = width * 0.50;
    const double bermHeight = ((1.0 - std::clamp(soil.compactionRatio, 0.0, 1.0)) * z * width) / (2.0 * bermWidth);

    result.sinkage = z;
    result.plasticSinkage = zp;
    result.compactionDrag = Rc;
    result.bulldozeDrag = Rb;
    result.dragForce = dragVector;
    result.contactPressure = pressure;
    result.bermHeight = bermHeight;

    return result;
}

void DeformableTerrainLayer::initGrid(
    const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height)
{
    mOrigin = origin;
    mCellSize = { (cellSize.x > kEpsilon) ? cellSize.x : 1.0, (cellSize.y > kEpsilon) ? cellSize.y : 1.0 };
    mWidth = width;
    mHeight = height;

    const std::size_t totalCells = width * height;
    mPlasticDelta.assign(totalCells, 0.0);
    mElasticDelta.assign(totalCells, 0.0);
    mHardening.assign(totalCells, 0.0);
}

SoilSinkageResult DeformableTerrainLayer::applyRutStamp(
    const RutStamp2D& stamp, const SoilMechanicsConfig& soil) noexcept
{
    const SoilSinkageResult result = BekkerWongSolver::evaluateStamp(stamp, soil);
    if (!isInitialized() || result.sinkage <= 0.0) {
        return result;
    }

    const double width = (stamp.width > 0.01) ? stamp.width : 0.01;
    const double length = (stamp.length > 0.01) ? stamp.length : 0.01;
    const double bermWidth = width * 0.50;
    const double totalHalfWidth = (width * 0.50) + bermWidth;
    const double totalHalfLength = length * 0.50;

    const double cosAngle = std::cos(stamp.heading);
    const double sinAngle = std::sin(stamp.heading);

    const double maxRadius = std::hypot(totalHalfWidth, totalHalfLength);
    const double minWorldX = stamp.position.x - maxRadius;
    const double maxWorldX = stamp.position.x + maxRadius;
    const double minWorldY = stamp.position.y - maxRadius;
    const double maxWorldY = stamp.position.y + maxRadius;

    const auto minCol = static_cast<std::size_t>(
        std::clamp(std::floor((minWorldX - mOrigin.x) / mCellSize.x), 0.0, static_cast<double>(mWidth - 1U)));
    const auto maxCol = static_cast<std::size_t>(
        std::clamp(std::ceil((maxWorldX - mOrigin.x) / mCellSize.x), 0.0, static_cast<double>(mWidth - 1U)));
    const auto minRow = static_cast<std::size_t>(
        std::clamp(std::floor((minWorldY - mOrigin.y) / mCellSize.y), 0.0, static_cast<double>(mHeight - 1U)));
    const auto maxRow = static_cast<std::size_t>(
        std::clamp(std::ceil((maxWorldY - mOrigin.y) / mCellSize.y), 0.0, static_cast<double>(mHeight - 1U)));

    for (std::size_t j = minRow; j <= maxRow; ++j) {
        for (std::size_t i = minCol; i <= maxCol; ++i) {
            const double vx = mOrigin.x + (static_cast<double>(i) * mCellSize.x);
            const double vy = mOrigin.y + (static_cast<double>(j) * mCellSize.y);

            const double dx = vx - stamp.position.x;
            const double dy = vy - stamp.position.y;

            // Longitudinal along heading:
            const double u = (dx * cosAngle) + (dy * sinAngle);
            // Lateral perpendicular to heading:
            const double w = (-dx * sinAngle) + (dy * cosAngle);

            if (std::abs(u) > totalHalfLength) {
                continue;
            }

            const double absW = std::abs(w);
            const std::size_t idx = (j * mWidth) + i;

            if (absW <= (width * 0.50)) {
                // Rut trough depression
                const double normalizedW = absW / (width * 0.50);
                const double cosTerm = std::cos(0.5 * kPi * normalizedW);
                const double shape = cosTerm * cosTerm;

                const double priorHardening = mHardening[idx];
                const double hardScale
                    = (soil.maxPlasticDepth > kEpsilon) ? (priorHardening / soil.maxPlasticDepth) : 0.0;
                const double stiffnessFactor = 1.0 + (soil.hardeningRate * std::clamp(hardScale, 0.0, 2.0));

                const double effectiveZ = result.sinkage / stiffnessFactor;
                const double effectiveZp = effectiveZ * std::clamp(soil.plasticRatio, 0.0, 1.0);
                const double effectiveZe = effectiveZ * (1.0 - std::clamp(soil.plasticRatio, 0.0, 1.0));

                const double plasticDelta = effectiveZp * shape;
                const double elasticDelta = effectiveZe * shape;

                const double currentPlasticDepth = -mPlasticDelta[idx];
                const double newPlasticDepth = std::min(currentPlasticDepth + plasticDelta, soil.maxPlasticDepth);

                mPlasticDelta[idx] = -newPlasticDepth;
                mElasticDelta[idx] = -std::max(-mElasticDelta[idx], elasticDelta);
                mHardening[idx] = newPlasticDepth;
            } else if (absW <= totalHalfWidth) {
                // Shoulder berm extrusion
                const double bermOffset = absW - (width * 0.50);
                const double normalizedBerm = bermOffset / bermWidth;
                const double bermShape = std::sin(kPi * normalizedBerm);
                const double positiveBerm = result.bermHeight * bermShape * bermShape;

                const double priorBerm = std::max(0.0, mPlasticDelta[idx]);
                mPlasticDelta[idx] = std::max(priorBerm, positiveBerm);
            }
        }
    }

    return result;
}

void DeformableTerrainLayer::updateRelaxation(double dt, const SoilMechanicsConfig& soil) noexcept
{
    if (!isInitialized() || dt <= 0.0 || soil.recoveryTime <= kEpsilon) {
        return;
    }

    const double decay = std::exp(-dt / soil.recoveryTime);
    const std::size_t totalCells = mWidth * mHeight;

    for (std::size_t i = 0U; i < totalCells; ++i) {
        mElasticDelta[i] *= decay;
        if (std::abs(mElasticDelta[i]) < 1e-5) {
            mElasticDelta[i] = 0.0;
        }
    }
}

double DeformableTerrainLayer::sampleBilinear(const std::vector<double>& data, double x, double y) const noexcept
{
    if (data.empty() || mWidth < 2U || mHeight < 2U) {
        return 0.0;
    }

    const double gx = (x - mOrigin.x) / mCellSize.x;
    const double gy = (y - mOrigin.y) / mCellSize.y;

    if (gx < 0.0 || gx > static_cast<double>(mWidth - 1U) || gy < 0.0 || gy > static_cast<double>(mHeight - 1U)) {
        return 0.0;
    }

    const auto i0 = static_cast<std::size_t>(std::clamp(std::floor(gx), 0.0, static_cast<double>(mWidth - 2U)));
    const auto j0 = static_cast<std::size_t>(std::clamp(std::floor(gy), 0.0, static_cast<double>(mHeight - 2U)));
    const std::size_t i1 = i0 + 1U;
    const std::size_t j1 = j0 + 1U;

    const double fx = gx - static_cast<double>(i0);
    const double fy = gy - static_cast<double>(j0);

    const double s00 = data[(j0 * mWidth) + i0];
    const double s10 = data[(j0 * mWidth) + i1];
    const double s01 = data[(j1 * mWidth) + i0];
    const double s11 = data[(j1 * mWidth) + i1];

    const double top = (s00 * (1.0 - fx)) + (s10 * fx);
    const double bottom = (s01 * (1.0 - fx)) + (s11 * fx);

    return (top * (1.0 - fy)) + (bottom * fy);
}

double DeformableTerrainLayer::getRutDelta(double x, double y) const noexcept
{
    if (!isInitialized()) {
        return 0.0;
    }

    const double plastic = sampleBilinear(mPlasticDelta, x, y);
    const double elastic = sampleBilinear(mElasticDelta, x, y);

    return plastic + elastic;
}

void DeformableTerrainLayer::clearRuts() noexcept
{
    if (!mPlasticDelta.empty()) {
        std::fill(mPlasticDelta.begin(), mPlasticDelta.end(), 0.0);
    }
    if (!mElasticDelta.empty()) {
        std::fill(mElasticDelta.begin(), mElasticDelta.end(), 0.0);
    }
    if (!mHardening.empty()) {
        std::fill(mHardening.begin(), mHardening.end(), 0.0);
    }
}

bool DeformableTerrainLayer::isInitialized() const noexcept
{
    return mWidth > 0U && mHeight > 0U && !mPlasticDelta.empty();
}

double DeformableTerrainLayer::getMaxRutDepth() const noexcept
{
    if (mPlasticDelta.empty()) {
        return 0.0;
    }

    double maxDepth = 0.0;
    const std::size_t totalCells = mWidth * mHeight;
    for (std::size_t i = 0U; i < totalCells; ++i) {
        const double depth = -(mPlasticDelta[i] + mElasticDelta[i]);
        if (depth > maxDepth) {
            maxDepth = depth;
        }
    }

    return maxDepth;
}

std::size_t DeformableTerrainLayer::getGridWidth() const noexcept
{
    return mWidth;
}

std::size_t DeformableTerrainLayer::getGridHeight() const noexcept
{
    return mHeight;
}

double DeformableTerrainLayer::getPlasticDelta(std::size_t index) const noexcept
{
    if (index < mPlasticDelta.size()) {
        return mPlasticDelta[index];
    }
    return 0.0;
}

double DeformableTerrainLayer::getElasticDelta(std::size_t index) const noexcept
{
    if (index < mElasticDelta.size()) {
        return mElasticDelta[index];
    }
    return 0.0;
}

void DeformableTerrainLayer::setTopStratumDepth(double depth) noexcept
{
    mBaseStratumDepth = (depth > kEpsilon) ? depth : 0.01;
}

void DeformableTerrainLayer::setTopStratumGrid(const std::vector<double>& depths)
{
    mStratumDepth = depths;
}

double DeformableTerrainLayer::getStratumDepth(double x, double y) const noexcept
{
    if (mStratumDepth.empty() || !isInitialized()) {
        return mBaseStratumDepth;
    }
    return sampleBilinear(mStratumDepth, x, y);
}

StratifiedSinkageResult DeformableTerrainLayer::applyStratifiedRut(
    const RutStamp2D& stamp, const StratifiedSoilConfig& config) noexcept
{
    const StratifiedSinkageResult result = StratifiedSoilSolver::evaluateStamp(stamp, config);
    if (!isInitialized() || result.totalSinkage <= 0.0) {
        return result;
    }

    const double width = (stamp.width > 0.01) ? stamp.width : 0.01;
    const double length = (stamp.length > 0.01) ? stamp.length : 0.01;
    const double bermWidth = width * 0.50;
    const double totalHalfWidth = (width * 0.50) + bermWidth;
    const double totalHalfLength = length * 0.50;

    const double cosAngle = std::cos(stamp.heading);
    const double sinAngle = std::sin(stamp.heading);

    const double maxRadius = std::hypot(totalHalfWidth, totalHalfLength);
    const double minWorldX = stamp.position.x - maxRadius;
    const double maxWorldX = stamp.position.x + maxRadius;
    const double minWorldY = stamp.position.y - maxRadius;
    const double maxWorldY = stamp.position.y + maxRadius;

    const auto minCol = static_cast<std::size_t>(
        std::clamp(std::floor((minWorldX - mOrigin.x) / mCellSize.x), 0.0, static_cast<double>(mWidth - 1U)));
    const auto maxCol = static_cast<std::size_t>(
        std::clamp(std::ceil((maxWorldX - mOrigin.x) / mCellSize.x), 0.0, static_cast<double>(mWidth - 1U)));
    const auto minRow = static_cast<std::size_t>(
        std::clamp(std::floor((minWorldY - mOrigin.y) / mCellSize.y), 0.0, static_cast<double>(mHeight - 1U)));
    const auto maxRow = static_cast<std::size_t>(
        std::clamp(std::ceil((maxWorldY - mOrigin.y) / mCellSize.y), 0.0, static_cast<double>(mHeight - 1U)));

    const double plasticRatio = std::clamp(config.topLayer.soil.plasticRatio, 0.0, 1.0);
    const double bermHeight = ((1.0 - std::clamp(config.topLayer.soil.compactionRatio, 0.0, 1.0))
        * result.totalSinkage * width) / (2.0 * bermWidth);

    for (std::size_t j = minRow; j <= maxRow; ++j) {
        for (std::size_t i = minCol; i <= maxCol; ++i) {
            const double vx = mOrigin.x + (static_cast<double>(i) * mCellSize.x);
            const double vy = mOrigin.y + (static_cast<double>(j) * mCellSize.y);

            const double dx = vx - stamp.position.x;
            const double dy = vy - stamp.position.y;

            const double u = (dx * cosAngle) + (dy * sinAngle);
            const double w = (-dx * sinAngle) + (dy * cosAngle);

            if (std::abs(u) > totalHalfLength) {
                continue;
            }

            const double absW = std::abs(w);
            const std::size_t idx = (j * mWidth) + i;

            if (absW <= (width * 0.50)) {
                const double normalizedW = absW / (width * 0.50);
                const double cosTerm = std::cos(0.5 * kPi * normalizedW);
                const double shape = cosTerm * cosTerm;

                const double effectiveZ = result.totalSinkage;
                const double effectiveZp = effectiveZ * plasticRatio;
                const double effectiveZe = effectiveZ * (1.0 - plasticRatio);

                const double plasticDelta = effectiveZp * shape;
                const double elasticDelta = effectiveZe * shape;

                const double currentPlasticDepth = -mPlasticDelta[idx];
                const double maxDepth = config.topLayer.layerDepth + config.substrate.soil.maxPlasticDepth;
                const double newPlasticDepth = std::min(currentPlasticDepth + plasticDelta, maxDepth);

                mPlasticDelta[idx] = -newPlasticDepth;
                mElasticDelta[idx] = -std::max(-mElasticDelta[idx], elasticDelta);
                mHardening[idx] = newPlasticDepth;
            } else if (absW <= totalHalfWidth) {
                const double bermOffset = absW - (width * 0.50);
                const double normalizedBerm = bermOffset / bermWidth;
                const double bermShape = std::sin(kPi * normalizedBerm);
                const double positiveBerm = bermHeight * bermShape * bermShape;

                const double priorBerm = std::max(0.0, mPlasticDelta[idx]);
                mPlasticDelta[idx] = std::max(priorBerm, positiveBerm);
            }
        }
    }

    return result;
}

} // namespace vehicle_sim::terrain
