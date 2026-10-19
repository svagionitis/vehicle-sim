#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/HydroplaningModel.h"
#include "libs/VehicleSimCore/terrain/StratifiedSoil.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace vehicle_sim::terrain {

namespace {

    [[nodiscard]] double catmullRom(double p0, double p1, double p2, double p3, double t) noexcept
    {
        const double t2 { t * t };
        const double t3 { t2 * t };
        return 0.5
            * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
    }

    [[nodiscard]] double catmullRomDeriv(double p0, double p1, double p2, double p3, double t) noexcept
    {
        const double t2 { t * t };
        return 0.5
            * ((-p0 + p2) + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t
                + 3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t2);
    }

} // namespace

void SurfaceMap2D::setBaseHeight(double baseHeight) noexcept
{
    mBaseHeight = baseHeight;
}

double SurfaceMap2D::getBaseHeight() const noexcept
{
    return mBaseHeight;
}

void SurfaceMap2D::setGrid(const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height,
    const std::vector<double>& elevations)
{
    if (width == 0U || height == 0U || cellSize.x <= 0.0 || cellSize.y <= 0.0
        || elevations.size() != (width * height)) {
        return;
    }

    mGridOrigin = origin;
    mCellSize = cellSize;
    mGridWidth = width;
    mGridHeight = height;
    mGridData = elevations;

    if (mEnableDeformable) {
        mDeformableLayer.initGrid(origin, cellSize, width, height);
    }
}

void SurfaceMap2D::setGridElevation(std::size_t xIndex, std::size_t yIndex, double elevation) noexcept
{
    if (xIndex < mGridWidth && yIndex < mGridHeight) {
        mGridData[yIndex * mGridWidth + xIndex] = elevation;
    }
}

double SurfaceMap2D::getGridElevation(std::size_t xIndex, std::size_t yIndex) const noexcept
{
    if (xIndex < mGridWidth && yIndex < mGridHeight) {
        return mGridData[yIndex * mGridWidth + xIndex];
    }
    return 0.0;
}

void SurfaceMap2D::clearGrid() noexcept
{
    mGridWidth = 0U;
    mGridHeight = 0U;
    mGridData.clear();
}

bool SurfaceMap2D::hasGrid() const noexcept
{
    return (mGridWidth > 0U) && (mGridHeight > 0U) && !mGridData.empty();
}

std::size_t SurfaceMap2D::getGridWidth() const noexcept
{
    return mGridWidth;
}

std::size_t SurfaceMap2D::getGridHeight() const noexcept
{
    return mGridHeight;
}

math::Vec2 SurfaceMap2D::getGridOrigin() const noexcept
{
    return mGridOrigin;
}

math::Vec2 SurfaceMap2D::getCellSize() const noexcept
{
    return mCellSize;
}

void SurfaceMap2D::setInterpolation(ElevationInterp2D interp) noexcept
{
    mInterpolation = interp;
}

ElevationInterp2D SurfaceMap2D::getInterpolation() const noexcept
{
    return mInterpolation;
}

void SurfaceMap2D::setBoundaryMode(ElevationBoundary2D boundary) noexcept
{
    mBoundaryMode = boundary;
}

ElevationBoundary2D SurfaceMap2D::getBoundaryMode() const noexcept
{
    return mBoundaryMode;
}

void SurfaceMap2D::setNoiseTransform(
    double verticalScale, const math::Vec2& horizontalScale, const math::Vec2& offset) noexcept
{
    mVerticalScale = verticalScale;
    mHorizontalScale = horizontalScale;
    mNoiseOffset = offset;
}

void SurfaceMap2D::enableNoise(bool enable) noexcept
{
    mEnableNoise = enable;
}

bool SurfaceMap2D::isNoiseEnabled() const noexcept
{
    return mEnableNoise;
}

void SurfaceMap2D::setNoiseConfig(const FractalConfig& config) noexcept
{
    mNoise.setConfig(config);
}

const FractalNoise& SurfaceMap2D::getNoise() const noexcept
{
    return mNoise;
}

void SurfaceMap2D::setDefaultMaterial(const SurfaceMaterial& material) noexcept
{
    mDefaultMaterial = material;
}

const SurfaceMaterial& SurfaceMap2D::getDefaultMaterial() const noexcept
{
    return mDefaultMaterial;
}

void SurfaceMap2D::addBoxZone(const math::Vec2& min, const math::Vec2& max, const SurfaceMaterial& material)
{
    MaterialZone2D zone {};
    zone.shape = ZoneShape2D::Box;
    zone.min = math::Vec2 { std::min(min.x, max.x), std::min(min.y, max.y) };
    zone.max = math::Vec2 { std::max(min.x, max.x), std::max(min.y, max.y) };
    zone.material = material;
    mMaterialZones.push_back(zone);
}

void SurfaceMap2D::addBoxZone(const math::Vec2& min, const math::Vec2& max, MaterialType materialType)
{
    addBoxZone(min, max, SurfaceMaterial::getPreset(materialType));
}

void SurfaceMap2D::addCircleZone(const math::Vec2& center, double radius, const SurfaceMaterial& material)
{
    MaterialZone2D zone {};
    zone.shape = ZoneShape2D::Circle;
    zone.center = center;
    zone.radius = std::abs(radius);
    zone.material = material;
    mMaterialZones.push_back(zone);
}

void SurfaceMap2D::addCircleZone(const math::Vec2& center, double radius, MaterialType materialType)
{
    addCircleZone(center, radius, SurfaceMaterial::getPreset(materialType));
}

void SurfaceMap2D::stampMaterialDisc(const math::Vec2& center, double radius, const SurfaceMaterial& material)
{
    addCircleZone(center, radius, material);
}

void SurfaceMap2D::clearMaterialZones() noexcept
{
    mMaterialZones.clear();
}

std::size_t SurfaceMap2D::getMaterialZoneCount() const noexcept
{
    return mMaterialZones.size();
}

SurfaceMaterial SurfaceMap2D::getMaterial(double x, double y) const noexcept
{
    return getMaterial(math::Vec2 { x, y });
}

SurfaceMaterial SurfaceMap2D::getMaterial(const math::Vec2& point) const noexcept
{
    // Evaluate in reverse order so latest added zone has precedence
    for (auto it = mMaterialZones.rbegin(); it != mMaterialZones.rend(); ++it) {
        if (it->shape == ZoneShape2D::Box) {
            if (point.x >= it->min.x && point.x <= it->max.x && point.y >= it->min.y && point.y <= it->max.y) {
                return it->material;
            }
        } else if (it->shape == ZoneShape2D::Circle) {
            const double dx { point.x - it->center.x };
            const double dy { point.y - it->center.y };
            if ((dx * dx + dy * dy) <= (it->radius * it->radius)) {
                return it->material;
            }
        }
    }

    return mDefaultMaterial;
}

double SurfaceMap2D::getGridSample(std::int32_t ix, std::int32_t iy) const noexcept
{
    if (!hasGrid()) {
        return 0.0;
    }

    const std::int32_t width { static_cast<std::int32_t>(mGridWidth) };
    const std::int32_t height { static_cast<std::int32_t>(mGridHeight) };

    std::int32_t clampedX { ix };
    std::int32_t clampedY { iy };

    switch (mBoundaryMode) {
    case ElevationBoundary2D::Clamp:
        clampedX = std::clamp(ix, 0, width - 1);
        clampedY = std::clamp(iy, 0, height - 1);
        break;

    case ElevationBoundary2D::Repeat:
        clampedX = ix % width;
        if (clampedX < 0) {
            clampedX += width;
        }
        clampedY = iy % height;
        if (clampedY < 0) {
            clampedY += height;
        }
        break;

    case ElevationBoundary2D::Zero:
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
            return 0.0;
        }
        break;
    }

    const std::size_t index { static_cast<std::size_t>(clampedY) * mGridWidth + static_cast<std::size_t>(clampedX) };
    return mGridData[index];
}

std::pair<double, math::Vec2> SurfaceMap2D::sampleGrid(double x, double y) const noexcept
{
    if (!hasGrid()) {
        return { 0.0, math::Vec2::zero() };
    }

    const double u { (x - mGridOrigin.x) / mCellSize.x };
    const double v { (y - mGridOrigin.y) / mCellSize.y };

    if (mInterpolation == ElevationInterp2D::Nearest) {
        const int ix { static_cast<int>(std::round(u)) };
        const int iy { static_cast<int>(std::round(v)) };
        return { getGridSample(ix, iy), math::Vec2::zero() };
    }

    const int i { static_cast<int>(std::floor(u)) };
    const int j { static_cast<int>(std::floor(v)) };
    const double tx { u - static_cast<double>(i) };
    const double ty { v - static_cast<double>(j) };

    if (mInterpolation == ElevationInterp2D::Bilinear) {
        const double z00 { getGridSample(i, j) };
        const double z10 { getGridSample(i + 1, j) };
        const double z01 { getGridSample(i, j + 1) };
        const double z11 { getGridSample(i + 1, j + 1) };

        const double height { (1.0 - tx) * (1.0 - ty) * z00 + tx * (1.0 - ty) * z10 + (1.0 - tx) * ty * z01
            + tx * ty * z11 };

        const double dhdu { (1.0 - ty) * (z10 - z00) + ty * (z11 - z01) };
        const double dhdv { (1.0 - tx) * (z01 - z00) + tx * (z11 - z10) };

        return { height, math::Vec2 { dhdu / mCellSize.x, dhdv / mCellSize.y } };
    }

    // Bicubic Catmull-Rom Hermite interpolation
    std::array<double, 4> rowVal {};
    std::array<double, 4> rowDerivX {};

    for (int r { 0 }; r < 4; ++r) {
        const int jy { j - 1 + r };
        const double p0 { getGridSample(i - 1, jy) };
        const double p1 { getGridSample(i, jy) };
        const double p2 { getGridSample(i + 1, jy) };
        const double p3 { getGridSample(i + 2, jy) };

        rowVal[static_cast<std::size_t>(r)] = catmullRom(p0, p1, p2, p3, tx);
        rowDerivX[static_cast<std::size_t>(r)] = catmullRomDeriv(p0, p1, p2, p3, tx);
    }

    const double height { catmullRom(rowVal[0], rowVal[1], rowVal[2], rowVal[3], ty) };
    const double dhdv { catmullRomDeriv(rowVal[0], rowVal[1], rowVal[2], rowVal[3], ty) };
    const double dhdu { catmullRom(rowDerivX[0], rowDerivX[1], rowDerivX[2], rowDerivX[3], ty) };

    return { height, math::Vec2 { dhdu / mCellSize.x, dhdv / mCellSize.y } };
}

double SurfaceMap2D::getHeight(double x, double y) const noexcept
{
    return sampleSurface(x, y).elevation;
}

double SurfaceMap2D::getHeight(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).elevation;
}

math::Vec2 SurfaceMap2D::getGradient(double x, double y) const noexcept
{
    return sampleSurface(x, y).gradient;
}

math::Vec2 SurfaceMap2D::getGradient(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).gradient;
}

double SurfaceMap2D::getSlopeAngle(double x, double y) const noexcept
{
    return sampleSurface(x, y).slopeAngle;
}

double SurfaceMap2D::getSlopeAngle(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).slopeAngle;
}

math::Vec2 SurfaceMap2D::getDownhill(double x, double y) const noexcept
{
    return sampleSurface(x, y).downhill;
}

math::Vec2 SurfaceMap2D::getDownhill(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).downhill;
}

SurfaceNormal3D SurfaceMap2D::getNormal3D(double x, double y) const noexcept
{
    return sampleSurface(x, y).normal;
}

SurfaceNormal3D SurfaceMap2D::getNormal3D(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).normal;
}

SurfaceSample2D SurfaceMap2D::sampleSurface(double x, double y) const noexcept
{
    SurfaceSample2D sample {};
    const auto [gridHeight, gridGrad] = sampleGrid(x, y);

    double totalElevation { mBaseHeight + gridHeight };
    math::Vec2 totalGrad { gridGrad };

    if (mEnableNoise) {
        const double nx { (x + mNoiseOffset.x) * mHorizontalScale.x };
        const double ny { (y + mNoiseOffset.y) * mHorizontalScale.y };
        const auto noiseSample = mNoise.sample2DWithDeriv(nx, ny);

        totalElevation += mVerticalScale * noiseSample.value;
        totalGrad.x += mVerticalScale * mHorizontalScale.x * noiseSample.gradient.x;
        totalGrad.y += mVerticalScale * mHorizontalScale.y * noiseSample.gradient.y;
    }

    if (mEnableDeformable && mDeformableLayer.isInitialized()) {
        totalElevation += mDeformableLayer.getRutDelta(x, y);
    }

    sample.elevation = totalElevation;
    sample.gradient = totalGrad;

    const double gradLen { totalGrad.length() };
    sample.slopeAngle = std::atan(gradLen);

    if (gradLen > 1e-9) {
        sample.downhill = -totalGrad * (1.0 / gradLen);
    } else {
        sample.downhill = math::Vec2::zero();
    }

    const double denom { std::sqrt(1.0 + gradLen * gradLen) };
    sample.normal.x = -totalGrad.x / denom;
    sample.normal.y = -totalGrad.y / denom;
    sample.normal.z = 1.0 / denom;

    sample.material = getMaterial(x, y);

    return sample;
}

SurfaceSample2D SurfaceMap2D::sampleSurface(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y);
}

bool SurfaceMap2D::isLiquidAt(const math::Vec2& point) const noexcept
{
    return getMaterial(point).isLiquid;
}

math::Vec2 SurfaceMap2D::computeSlopeForce(const physics::RigidBody2D& body, double gravity) const noexcept
{
    const auto sample = sampleSurface(body.getPosition());
    const double mass { body.getMass() };
    const double gradLen { sample.gradient.length() };
    const double denom { std::sqrt(1.0 + gradLen * gradLen) };

    return math::Vec2 { -mass * gravity * (sample.gradient.x / denom), -mass * gravity * (sample.gradient.y / denom) };
}

void SurfaceMap2D::applySlopeForce(physics::RigidBody2D& body, double gravity) const noexcept
{
    body.applyForce(computeSlopeForce(body, gravity));
}

math::Vec2 SurfaceMap2D::computeLiquidDrag(const physics::RigidBody2D& body, double dragFactor) const noexcept
{
    const auto sample = sampleSurface(body.getPosition());
    if (!sample.material.isLiquid) {
        return math::Vec2::zero();
    }

    const math::Vec2 vel { body.getLinearVelocity() };
    const double speed { vel.length() };
    if (speed < 1e-6) {
        return math::Vec2::zero();
    }

    const double coeff { sample.material.rollingDrag * dragFactor };
    return -vel * (coeff * speed);
}

void SurfaceMap2D::applySurfaceToCar(dynamics::ArcadeCarDynamics& car, const physics::RigidBody2D& body) const noexcept
{
    const auto chassisSample = sampleSurface(body.getPosition());
    car.setSurfaceModifier(1.0, chassisSample.material.rollingDrag);

    for (std::size_t w { 0U }; w < dynamics::ArcadeCarDynamics::kWheelCount; ++w) {
        const auto& tel = car.getWheelTelemetry(w);
        const math::Vec2 worldPos { body.toWorldPoint(tel.localOffset) };
        const auto wheelSample = sampleSurface(worldPos);
        car.setWheelSurfaceModifier(w, wheelSample.material.friction);
    }
}

void SurfaceMap2D::enableDeformableLayer(bool enable) noexcept
{
    mEnableDeformable = enable;
    if (mEnableDeformable && hasGrid() && !mDeformableLayer.isInitialized()) {
        mDeformableLayer.initGrid(mGridOrigin, mCellSize, mGridWidth, mGridHeight);
    }
}

bool SurfaceMap2D::isDeformableEnabled() const noexcept
{
    return mEnableDeformable;
}

void SurfaceMap2D::initDeformableLayer(
    const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height)
{
    mDeformableLayer.initGrid(origin, cellSize, width, height);
}

SoilSinkageResult SurfaceMap2D::applyRutStamp(const RutStamp2D& stamp) noexcept
{
    if (!mEnableDeformable) {
        return SoilSinkageResult {};
    }

    if (!mDeformableLayer.isInitialized() && hasGrid()) {
        mDeformableLayer.initGrid(mGridOrigin, mCellSize, mGridWidth, mGridHeight);
    }

    const SurfaceMaterial mat = getMaterial(stamp.position);
    return mDeformableLayer.applyRutStamp(stamp, mat.soil);
}

void SurfaceMap2D::updateRutRecovery(double dt) noexcept
{
    if (mEnableDeformable && mDeformableLayer.isInitialized()) {
        mDeformableLayer.updateRelaxation(dt, mDefaultMaterial.soil);
    }
}

void SurfaceMap2D::clearRuts() noexcept
{
    mDeformableLayer.clearRuts();
}

double SurfaceMap2D::getDeformedHeight(double x, double y) const noexcept
{
    return sampleSurface(x, y).elevation;
}

double SurfaceMap2D::getDeformedHeight(const math::Vec2& point) const noexcept
{
    return sampleSurface(point.x, point.y).elevation;
}

double SurfaceMap2D::getRutDelta(double x, double y) const noexcept
{
    if (!mEnableDeformable || !mDeformableLayer.isInitialized()) {
        return 0.0;
    }
    return mDeformableLayer.getRutDelta(x, y);
}

double SurfaceMap2D::getMaxRutDepth() const noexcept
{
    if (!mDeformableLayer.isInitialized()) {
        return 0.0;
    }
    return mDeformableLayer.getMaxRutDepth();
}

const DeformableTerrainLayer& SurfaceMap2D::getDeformableLayer() const noexcept
{
    return mDeformableLayer;
}

DeformableTerrainLayer& SurfaceMap2D::getDeformableLayer() noexcept
{
    return mDeformableLayer;
}

void SurfaceMap2D::setBaseWaterDepth(double waterDepth) noexcept
{
    mBaseWaterDepth = (waterDepth > 0.0) ? waterDepth : 0.0;
}

double SurfaceMap2D::getBaseWaterDepth() const noexcept
{
    return mBaseWaterDepth;
}

void SurfaceMap2D::setWaterFilmGrid(const std::vector<double>& waterDepths)
{
    mWaterFilmData = waterDepths;
}

double SurfaceMap2D::getWaterDepth(double x, double y) const noexcept
{
    if (mWaterFilmData.empty() || mGridWidth == 0U || mGridHeight == 0U) {
        const auto mat = getMaterial(x, y);
        if (mat.type == MaterialType::Water) {
            return (mBaseWaterDepth > 0.0) ? mBaseWaterDepth : 0.05;
        }
        return mBaseWaterDepth;
    }

    const double gx = (x - mGridOrigin.x) / mCellSize.x;
    const double gy = (y - mGridOrigin.y) / mCellSize.y;

    const auto x0 = static_cast<std::size_t>(std::clamp(std::floor(gx), 0.0, static_cast<double>(mGridWidth - 1U)));
    const auto x1 = static_cast<std::size_t>(std::clamp(x0 + 1U, std::size_t { 0U }, mGridWidth - 1U));
    const auto y0 = static_cast<std::size_t>(std::clamp(std::floor(gy), 0.0, static_cast<double>(mGridHeight - 1U)));
    const auto y1 = static_cast<std::size_t>(std::clamp(y0 + 1U, std::size_t { 0U }, mGridHeight - 1U));

    const double fx = std::clamp(gx - std::floor(gx), 0.0, 1.0);
    const double fy = std::clamp(gy - std::floor(gy), 0.0, 1.0);

    const double d00 = mWaterFilmData[(y0 * mGridWidth) + x0];
    const double d10 = mWaterFilmData[(y0 * mGridWidth) + x1];
    const double d01 = mWaterFilmData[(y1 * mGridWidth) + x0];
    const double d11 = mWaterFilmData[(y1 * mGridWidth) + x1];

    const double d0 = d00 + (fx * (d10 - d00));
    const double d1 = d01 + (fx * (d11 - d01));

    return std::max(0.0, d0 + (fy * (d1 - d0)));
}

double SurfaceMap2D::getWaterDepth(const math::Vec2& point) const noexcept
{
    return getWaterDepth(point.x, point.y);
}

StratifiedSinkageResult SurfaceMap2D::applyStratifiedRut(
    const RutStamp2D& stamp, const StratifiedSoilConfig& config) noexcept
{
    if (!mEnableDeformable || !mDeformableLayer.isInitialized()) {
        return StratifiedSoilSolver::evaluateStamp(stamp, config);
    }
    return mDeformableLayer.applyStratifiedRut(stamp, config);
}

HydroplaningResult SurfaceMap2D::sampleHydroplaning(
    const HydroplaningState& state, const HydroplaningConfig& config) const noexcept
{
    return HydroplaningSolver::evaluateHydroplaning(state, config);
}

} // namespace vehicle_sim::terrain
