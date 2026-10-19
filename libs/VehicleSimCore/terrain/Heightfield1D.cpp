#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vehicle_sim::terrain {

dynamics::SideViewPlatform HeightfieldPlatform::toSideViewPlatform() const noexcept
{
    dynamics::SideViewPlatform plat {};
    plat.y = y;
    plat.minX = minX;
    plat.maxX = maxX;
    plat.friction = friction;
    plat.oneWay = oneWay;
    return plat;
}

void Heightfield1D::setBaseHeight(double baseHeight) noexcept
{
    mBaseHeight = baseHeight;
}

double Heightfield1D::getBaseHeight() const noexcept
{
    return mBaseHeight;
}

void Heightfield1D::setNoiseTransform(double verticalScale, double horizontalScale, double horizontalOffset) noexcept
{
    mVerticalScale = verticalScale;
    mHorizontalScale = horizontalScale;
    mHorizontalOffset = horizontalOffset;
}

void Heightfield1D::enableNoise(bool enable) noexcept
{
    mEnableNoise = enable;
}

bool Heightfield1D::isNoiseEnabled() const noexcept
{
    return mEnableNoise;
}

void Heightfield1D::setNoiseConfig(const FractalConfig& config) noexcept
{
    mNoise.setConfig(config);
}

const FractalNoise& Heightfield1D::getNoise() const noexcept
{
    return mNoise;
}

void Heightfield1D::setSamples(const std::vector<HeightSample1D>& samples)
{
    mSamples = samples;
    std::sort(mSamples.begin(), mSamples.end(),
        [](const HeightSample1D& a, const HeightSample1D& b) noexcept { return a.x < b.x; });
}

void Heightfield1D::addSample(const HeightSample1D& sample)
{
    mSamples.push_back(sample);
    std::sort(mSamples.begin(), mSamples.end(),
        [](const HeightSample1D& a, const HeightSample1D& b) noexcept { return a.x < b.x; });
}

void Heightfield1D::clearSamples() noexcept
{
    mSamples.clear();
}

std::size_t Heightfield1D::getSampleCount() const noexcept
{
    return mSamples.size();
}

const std::vector<HeightSample1D>& Heightfield1D::getSamples() const noexcept
{
    return mSamples;
}

void Heightfield1D::setInterpolation(HeightfieldInterp interp) noexcept
{
    mInterpolation = interp;
}

HeightfieldInterp Heightfield1D::getInterpolation() const noexcept
{
    return mInterpolation;
}

void Heightfield1D::setBoundaryMode(HeightfieldBoundary boundary) noexcept
{
    mBoundaryMode = boundary;
}

HeightfieldBoundary Heightfield1D::getBoundaryMode() const noexcept
{
    return mBoundaryMode;
}

void Heightfield1D::setDefaultMaterial(const SurfaceMaterial& material) noexcept
{
    mDefaultMaterial = material;
}

const SurfaceMaterial& Heightfield1D::getDefaultMaterial() const noexcept
{
    return mDefaultMaterial;
}

void Heightfield1D::addMaterialZone(double minX, double maxX, const SurfaceMaterial& material)
{
    MaterialZone zone {};
    zone.minX = std::min(minX, maxX);
    zone.maxX = std::max(minX, maxX);
    zone.material = material;
    mMaterialZones.push_back(zone);
}

void Heightfield1D::addMaterialZone(double minX, double maxX, MaterialType materialType)
{
    addMaterialZone(minX, maxX, SurfaceMaterial::getPreset(materialType));
}

void Heightfield1D::clearMaterialZones() noexcept
{
    mMaterialZones.clear();
}

std::size_t Heightfield1D::getMaterialZoneCount() const noexcept
{
    return mMaterialZones.size();
}

std::pair<double, double> Heightfield1D::sampleDiscrete(double x) const noexcept
{
    if (mSamples.empty()) {
        return { 0.0, 0.0 };
    }

    if (mSamples.size() == 1U) {
        return { mSamples.front().y, 0.0 };
    }

    if (x < mSamples.front().x) {
        switch (mBoundaryMode) {
        case HeightfieldBoundary::Clamp:
            return { mSamples.front().y, 0.0 };
        case HeightfieldBoundary::Zero:
            return { 0.0, 0.0 };
        case HeightfieldBoundary::Extrapolate: {
            const double dx { mSamples[1].x - mSamples[0].x };
            const double slope { (dx > 1e-12) ? ((mSamples[1].y - mSamples[0].y) / dx) : 0.0 };
            return { mSamples[0].y + (x - mSamples[0].x) * slope, slope };
        }
        }
    }

    if (x > mSamples.back().x) {
        const std::size_t n { mSamples.size() };
        switch (mBoundaryMode) {
        case HeightfieldBoundary::Clamp:
            return { mSamples.back().y, 0.0 };
        case HeightfieldBoundary::Zero:
            return { 0.0, 0.0 };
        case HeightfieldBoundary::Extrapolate: {
            const double dx { mSamples[n - 1].x - mSamples[n - 2].x };
            const double slope { (dx > 1e-12) ? ((mSamples[n - 1].y - mSamples[n - 2].y) / dx) : 0.0 };
            return { mSamples[n - 1].y + (x - mSamples[n - 1].x) * slope, slope };
        }
        }
    }

    auto it = std::lower_bound(
        mSamples.begin(), mSamples.end(), x, [](const HeightSample1D& s, double val) noexcept { return s.x < val; });

    std::size_t idx { 0U };
    if (it == mSamples.begin()) {
        idx = 0U;
    } else {
        const auto dist = std::distance(mSamples.begin(), it);
        idx = static_cast<std::size_t>(dist) - 1U;
        if (idx >= mSamples.size() - 1U) {
            idx = mSamples.size() - 2U;
        }
    }

    const double x0 { mSamples[idx].x };
    const double y0 { mSamples[idx].y };
    const double x1 { mSamples[idx + 1U].x };
    const double y1 { mSamples[idx + 1U].y };
    const double h { x1 - x0 };

    if (h < 1e-12) {
        return { y0, 0.0 };
    }

    const double t { std::clamp((x - x0) / h, 0.0, 1.0) };

    if (mInterpolation == HeightfieldInterp::Linear) {
        const double elev { (1.0 - t) * y0 + t * y1 };
        const double deriv { (y1 - y0) / h };
        return { elev, deriv };
    }

    // Cubic Hermite (Catmull-Rom) interpolation
    double m0 { 0.0 };
    if (idx > 0U) {
        const double dxPrev { mSamples[idx + 1U].x - mSamples[idx - 1U].x };
        m0 = (dxPrev > 1e-12) ? ((mSamples[idx + 1U].y - mSamples[idx - 1U].y) / dxPrev) : 0.0;
    } else {
        m0 = (y1 - y0) / h;
    }

    double m1 { 0.0 };
    if (idx + 2U < mSamples.size()) {
        const double dxNext { mSamples[idx + 2U].x - mSamples[idx].x };
        m1 = (dxNext > 1e-12) ? ((mSamples[idx + 2U].y - mSamples[idx].y) / dxNext) : 0.0;
    } else {
        m1 = (y1 - y0) / h;
    }

    const double t2 { t * t };
    const double t3 { t2 * t };

    const double h00 { 2.0 * t3 - 3.0 * t2 + 1.0 };
    const double h10 { t3 - 2.0 * t2 + t };
    const double h01 { -2.0 * t3 + 3.0 * t2 };
    const double h11 { t3 - t2 };

    const double elev { h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1 };

    const double dh00 { 6.0 * t2 - 6.0 * t };
    const double dh10 { 3.0 * t2 - 4.0 * t + 1.0 };
    const double dh01 { -6.0 * t2 + 6.0 * t };
    const double dh11 { 3.0 * t2 - 2.0 * t };

    const double dy_dt { dh00 * y0 + dh10 * h * m0 + dh01 * y1 + dh11 * h * m1 };
    const double deriv { dy_dt / h };

    return { elev, deriv };
}

double Heightfield1D::getHeight(double x) const noexcept
{
    return sampleSurface(x).elevation;
}

double Heightfield1D::getSlope(double x) const noexcept
{
    return sampleSurface(x).derivative;
}

double Heightfield1D::getSlopeAngle(double x) const noexcept
{
    return sampleSurface(x).slopeAngle;
}

math::Vec2 Heightfield1D::getNormal(double x) const noexcept
{
    return sampleSurface(x).normal;
}

math::Vec2 Heightfield1D::getTangent(double x) const noexcept
{
    return sampleSurface(x).tangent;
}

SurfaceMaterial Heightfield1D::getMaterial(double x) const noexcept
{
    for (auto it = mMaterialZones.rbegin(); it != mMaterialZones.rend(); ++it) {
        if (x >= it->minX && x <= it->maxX) {
            return it->material;
        }
    }
    return mDefaultMaterial;
}

SurfaceSample1D Heightfield1D::sampleSurface(double x) const noexcept
{
    double elevation { mBaseHeight };
    double derivative { 0.0 };

    if (!mSamples.empty()) {
        const auto [sElev, sDeriv] = sampleDiscrete(x);
        elevation += sElev;
        derivative += sDeriv;
    }

    if (mEnableNoise) {
        const double nx { (x - mHorizontalOffset) * mHorizontalScale };
        const auto nSample = mNoise.sample1DWithDeriv(nx);
        elevation += mVerticalScale * nSample.value;
        derivative += mVerticalScale * mHorizontalScale * nSample.derivative;
    }

    const double hyp { std::hypot(1.0, derivative) };
    const math::Vec2 normal { -derivative / hyp, 1.0 / hyp };
    const math::Vec2 tangent { 1.0 / hyp, derivative / hyp };
    const double slopeAngle { std::atan(derivative) };
    const SurfaceMaterial material { getMaterial(x) };

    return SurfaceSample1D { elevation, derivative, slopeAngle, normal, tangent, material };
}

double Heightfield1D::getPenetration(const math::Vec2& point) const noexcept
{
    return getHeight(point.x) - point.y;
}

bool Heightfield1D::isUnderground(const math::Vec2& point) const noexcept
{
    return point.y < getHeight(point.x);
}

HeightfieldRayHit Heightfield1D::raycast(
    const math::Vec2& origin, const math::Vec2& direction, double maxDistance) const noexcept
{
    if (maxDistance <= 0.0 || direction.isZero()) {
        return HeightfieldRayHit {};
    }

    const double originHeight { getHeight(origin.x) };
    if (origin.y <= originHeight) {
        return HeightfieldRayHit { origin, getNormal(origin.x), 0.0, 0.0, getMaterial(origin.x), true };
    }

    const math::Vec2 dir { direction.normalized() };

    // Optimize vertical downwards raycast (common for wheel sensors)
    if (std::abs(dir.x) <= 1e-9) {
        if (dir.y >= 0.0) {
            return HeightfieldRayHit {}; // Pointing upwards away from terrain
        }
        const double hitDist { (origin.y - originHeight) / (-dir.y) };
        if (hitDist <= maxDistance) {
            const math::Vec2 hitPoint { origin.x, originHeight };
            return HeightfieldRayHit { hitPoint, getNormal(origin.x), hitDist, hitDist / maxDistance,
                getMaterial(origin.x), true };
        }
        return HeightfieldRayHit {};
    }

    // General ray marching with bisection refinement
    const double step { std::max(0.01, std::min(0.25, maxDistance / 32.0)) };
    double prevT { 0.0 };
    double t { step };
    bool found { false };

    while (t <= maxDistance) {
        const math::Vec2 p { origin + dir * t };
        const double h { getHeight(p.x) };
        if (p.y <= h) {
            found = true;
            break;
        }
        prevT = t;
        t += step;
        if (t > maxDistance && prevT < maxDistance) {
            t = maxDistance;
            const math::Vec2 pEnd { origin + dir * t };
            if (pEnd.y <= getHeight(pEnd.x)) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        return HeightfieldRayHit {};
    }

    double low { prevT };
    double high { t };
    for (std::int32_t i { 0 }; i < 16; ++i) {
        const double mid { 0.5 * (low + high) };
        const math::Vec2 pMid { origin + dir * mid };
        const double hMid { getHeight(pMid.x) };
        if (pMid.y <= hMid) {
            high = mid;
        } else {
            low = mid;
        }
    }

    const double hitT { 0.5 * (low + high) };
    const math::Vec2 hitPoint { origin + dir * hitT };
    return HeightfieldRayHit { hitPoint, getNormal(hitPoint.x), hitT, hitT / maxDistance, getMaterial(hitPoint.x),
        true };
}

std::vector<HeightfieldPlatform> Heightfield1D::extractPlatforms(const PlatformExtractionConfig& config) const
{
    std::vector<HeightfieldPlatform> platforms {};

    if (config.maxX <= config.minX || config.sampleStep <= 1e-4) {
        return platforms;
    }

    if (config.method == PlatformExtractionMethod::FlatPlateau) {
        bool inCandidate { false };
        double candStart { 0.0 };
        double candEnd { 0.0 };
        double candElevSum { 0.0 };
        std::size_t candCount { 0U };
        SurfaceMaterial candMat {};

        const auto flushCandidate = [&](std::vector<HeightfieldPlatform>& platList) {
            if (!inCandidate) {
                return;
            }
            const double width { candEnd - candStart };
            if (width >= config.minWidth && candCount > 0U) {
                const double avgElev { candElevSum / static_cast<double>(candCount) };
                if (!platList.empty() && std::abs(platList.back().y - avgElev) <= config.mergeElevationTolerance
                    && std::abs(candStart - platList.back().maxX) <= (config.sampleStep * 1.5)
                    && platList.back().materialType == candMat.type) {
                    platList.back().maxX = candEnd;
                } else {
                    HeightfieldPlatform p {};
                    p.y = avgElev;
                    p.minX = candStart;
                    p.maxX = candEnd;
                    p.friction = candMat.friction;
                    p.oneWay = config.defaultOneWay;
                    p.materialType = candMat.type;
                    platList.push_back(p);
                }
            }
            inCandidate = false;
            candElevSum = 0.0;
            candCount = 0U;
        };

        for (double x { config.minX }; x <= config.maxX; x += config.sampleStep) {
            const auto sample = sampleSurface(x);
            const bool isFlat { std::abs(sample.derivative) <= config.maxSlope };

            if (isFlat) {
                if (!inCandidate) {
                    inCandidate = true;
                    candStart = x;
                    candEnd = x;
                    candElevSum = sample.elevation;
                    candCount = 1U;
                    candMat = sample.material;
                } else {
                    candEnd = x;
                    candElevSum += sample.elevation;
                    candCount += 1U;
                }
            } else {
                flushCandidate(platforms);
            }
        }
        flushCandidate(platforms);

    } else if (config.method == PlatformExtractionMethod::StepQuantization) {
        const double tierH { (config.tierHeight > 1e-4) ? config.tierHeight : 1.0 };
        bool inSpan { false };
        double currentTier { 0.0 };
        double spanStart { 0.0 };
        double spanEnd { 0.0 };
        SurfaceMaterial spanMat {};

        const auto flushSpan = [&](std::vector<HeightfieldPlatform>& platList) {
            if (!inSpan) {
                return;
            }
            const double width { spanEnd - spanStart };
            if (width >= config.minWidth) {
                HeightfieldPlatform p {};
                p.y = currentTier;
                p.minX = spanStart;
                p.maxX = spanEnd;
                p.friction = spanMat.friction;
                p.oneWay = config.defaultOneWay;
                p.materialType = spanMat.type;
                platList.push_back(p);
            }
            inSpan = false;
        };

        for (double x { config.minX }; x <= config.maxX; x += config.sampleStep) {
            const auto sample = sampleSurface(x);
            const double tier { std::round(sample.elevation / tierH) * tierH };

            if (!inSpan) {
                inSpan = true;
                currentTier = tier;
                spanStart = x;
                spanEnd = x;
                spanMat = sample.material;
            } else if (std::abs(tier - currentTier) < (tierH * 0.25)) {
                spanEnd = x;
            } else {
                flushSpan(platforms);
                inSpan = true;
                currentTier = tier;
                spanStart = x;
                spanEnd = x;
                spanMat = sample.material;
            }
        }
        flushSpan(platforms);
    }

    return platforms;
}

std::vector<dynamics::SideViewPlatform> Heightfield1D::extractSideViewPlatforms(
    const PlatformExtractionConfig& config) const
{
    const auto plats = extractPlatforms(config);
    std::vector<dynamics::SideViewPlatform> sidePlats {};
    sidePlats.reserve(plats.size());
    for (const auto& plat : plats) {
        sidePlats.push_back(plat.toSideViewPlatform());
    }
    return sidePlats;
}

void Heightfield1D::populateSideViewCar(
    dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting) const
{
    if (clearExisting) {
        car.clearPlatforms();
    }
    const auto sidePlats = extractSideViewPlatforms(config);
    for (const auto& plat : sidePlats) {
        car.addPlatform(plat);
    }
}

} // namespace vehicle_sim::terrain
