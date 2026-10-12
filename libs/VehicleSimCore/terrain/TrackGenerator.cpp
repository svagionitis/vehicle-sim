#include "TrackGenerator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vehicle_sim::terrain {

namespace {

    constexpr double kPi { 3.14159265358979323846 };
    constexpr double kTwoPi { 2.0 * kPi };

    /// @class DeterministicRng
    /// @brief Cross-platform pseudo-random generator with guaranteed deterministic sequences.
    class DeterministicRng {
    public:
        explicit DeterministicRng(std::uint32_t seed) noexcept
            : mState { seed != 0U ? seed : 1U }
        {
        }

        [[nodiscard]] std::uint32_t next() noexcept
        {
            mState = mState * 1664525U + 1013904223U;
            return mState;
        }

        [[nodiscard]] double nextDouble(double minVal, double maxVal) noexcept
        {
            const double unit { static_cast<double>(next()) / 4294967295.0 };
            return minVal + unit * (maxVal - minVal);
        }

        [[nodiscard]] bool nextBool(double probability) noexcept
        {
            return nextDouble(0.0, 1.0) < probability;
        }

    private:
        std::uint32_t mState { 1U };
    };

    /// @brief Computes shortest Euclidean distance from point P to segment [A, B].
    [[nodiscard]] double distToSegment(
        const math::Vec2& point, const math::Vec2& segA, const math::Vec2& segB, double* outT = nullptr) noexcept
    {
        const math::Vec2 diff { segB - segA };
        const double lenSq { diff.lengthSquared() };
        if (lenSq < 1e-12) {
            if (outT != nullptr) {
                *outT = 0.0;
            }
            return (point - segA).length();
        }

        const double t { std::clamp((point - segA).dot(diff) / lenSq, 0.0, 1.0) };
        if (outT != nullptr) {
            *outT = t;
        }
        const math::Vec2 proj { segA + diff * t };
        return (point - proj).length();
    }

} // namespace

void TrialsCourse::populateSideViewCar(
    dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting) const
{
    terrain.populateSideViewCar(car, config, clearExisting);
}

void PlatformerLevel::populateSideViewCar(dynamics::SideViewCarDynamics& car, bool clearExisting) const
{
    if (clearExisting) {
        car.clearPlatforms();
    }
    for (const auto& plat : platforms) {
        car.addPlatform(plat);
    }
}

void HillClimbCourse::populateSideViewCar(
    dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting) const
{
    terrain.populateSideViewCar(car, config, clearExisting);
}

bool CircuitTrack2D::isPointOnTrack(const math::Vec2& point) const noexcept
{
    return getDistanceToCenter(point) <= (trackWidth * 0.5);
}

double CircuitTrack2D::getDistanceToCenter(const math::Vec2& point) const noexcept
{
    if (centerline.size() < 2U) {
        return 0.0;
    }

    double minDist { std::numeric_limits<double>::infinity() };
    const std::size_t segCount { isClosed ? centerline.size() : (centerline.size() - 1U) };

    for (std::size_t i { 0 }; i < segCount; ++i) {
        const math::Vec2& segA { centerline[i] };
        const math::Vec2& segB { centerline[(i + 1U) % centerline.size()] };
        const double d { distToSegment(point, segA, segB) };
        if (d < minDist) {
            minDist = d;
        }
    }

    return minDist;
}

double CircuitTrack2D::getTrackProgress(const math::Vec2& point) const noexcept
{
    if (centerline.size() < 2U || trackLength < 1e-6) {
        return 0.0;
    }

    double minDist { std::numeric_limits<double>::infinity() };
    std::size_t bestSeg { 0 };
    double bestT { 0.0 };
    const std::size_t segCount { isClosed ? centerline.size() : (centerline.size() - 1U) };

    for (std::size_t i { 0 }; i < segCount; ++i) {
        const math::Vec2& segA { centerline[i] };
        const math::Vec2& segB { centerline[(i + 1U) % centerline.size()] };
        double t { 0.0 };
        const double d { distToSegment(point, segA, segB, &t) };
        if (d < minDist) {
            minDist = d;
            bestSeg = i;
            bestT = t;
        }
    }

    double accumulatedDist { 0.0 };
    for (std::size_t i { 0 }; i < bestSeg; ++i) {
        accumulatedDist += (centerline[(i + 1U) % centerline.size()] - centerline[i]).length();
    }
    accumulatedDist += (centerline[(bestSeg + 1U) % centerline.size()] - centerline[bestSeg]).length() * bestT;

    return std::clamp(accumulatedDist / trackLength, 0.0, 1.0);
}

double CircuitTrack2D::getBankAngleAtProgress(double progress) const noexcept
{
    if (bankAngles.empty()) {
        return 0.0;
    }
    if (bankAngles.size() == 1U) {
        return bankAngles[0];
    }

    const double clampedProg { std::clamp(progress, 0.0, 1.0) };
    if (isClosed) {
        const double scaled { clampedProg * static_cast<double>(bankAngles.size()) };
        const std::size_t idx0 { static_cast<std::size_t>(scaled) % bankAngles.size() };
        const std::size_t idx1 { (idx0 + 1U) % bankAngles.size() };
        const double frac { scaled - std::floor(scaled) };
        return (1.0 - frac) * bankAngles[idx0] + frac * bankAngles[idx1];
    }

    const double scaled { clampedProg * static_cast<double>(bankAngles.size() - 1U) };
    const std::size_t idx0 { static_cast<std::size_t>(scaled) };
    if (idx0 >= bankAngles.size() - 1U) {
        return bankAngles.back();
    }
    const std::size_t idx1 { idx0 + 1U };
    const double frac { scaled - static_cast<double>(idx0) };
    return (1.0 - frac) * bankAngles[idx0] + frac * bankAngles[idx1];
}

double CircuitTrack2D::getBankAngle(const math::Vec2& point) const noexcept
{
    if (bankAngles.empty()) {
        return 0.0;
    }
    const double dist { getDistanceToCenter(point) };
    if (dist > (trackWidth + shoulderWidth) * 0.5) {
        return 0.0;
    }
    return getBankAngleAtProgress(getTrackProgress(point));
}

CamberProfileSample CircuitTrack2D::sampleCamber(const math::Vec2& point) const noexcept
{
    if (centerline.size() < 2U) {
        return camberProfile.evaluateCamber(0.0, 0.0, trackWidth * 0.5, math::Vec2 { 1.0, 0.0 });
    }

    double minDist { std::numeric_limits<double>::infinity() };
    std::size_t bestSeg { 0 };
    double bestT { 0.0 };
    const std::size_t segCount { isClosed ? centerline.size() : (centerline.size() - 1U) };

    for (std::size_t i { 0 }; i < segCount; ++i) {
        const math::Vec2& segA { centerline[i] };
        const math::Vec2& segB { centerline[(i + 1U) % centerline.size()] };
        double t { 0.0 };
        const double d { distToSegment(point, segA, segB, &t) };
        if (d < minDist) {
            minDist = d;
            bestSeg = i;
            bestT = t;
        }
    }

    const math::Vec2& segA { centerline[bestSeg] };
    const math::Vec2& segB { centerline[(bestSeg + 1U) % centerline.size()] };
    const math::Vec2 diff { segB - segA };
    const math::Vec2 tangent { diff.normalized() };
    const math::Vec2 latNorm { -tangent.y, tangent.x };
    const math::Vec2 proj { segA + diff * bestT };

    const double w { (point - proj).dot(latNorm) };

    double bank { 0.0 };
    if (!bankAngles.empty()) {
        const double bankA { bankAngles[bestSeg] };
        const double bankB { bankAngles[(bestSeg + 1U) % bankAngles.size()] };
        bank = (1.0 - bestT) * bankA + bestT * bankB;
    }

    return camberProfile.evaluateCamber(bank, w, trackWidth * 0.5, tangent);
}

TrialsCourse TrackGenerator::generateTrialsCourse(const TrialsCourseConfig& config)
{
    TrialsCourse course { };
    std::vector<HeightSample1D> samples { };
    double currentX { 0.0 };
    double currentY { 0.0 };

    // 1. Flat starting platform (Concrete)
    const double startLen { std::max(5.0, config.startLength) };
    samples.push_back({ currentX, currentY });
    currentX += startLen;
    samples.push_back({ currentX, currentY });
    course.terrain.addMaterialZone(0.0, currentX, MaterialType::Concrete);

    // 2. Undulating camel humps (Dirt)
    const double humpsStart { currentX };
    const std::size_t numHumps { std::max(std::size_t { 1 }, config.humpCount) };
    const double humpSpan { std::max(2.0, config.humpSpacing) };
    const double humpAmp { std::max(0.2, config.humpHeight) };

    for (std::size_t h { 0 }; h < numHumps; ++h) {
        constexpr std::size_t kSubSteps { 8 };
        for (std::size_t s { 1 }; s <= kSubSteps; ++s) {
            const double frac { static_cast<double>(s) / static_cast<double>(kSubSteps) };
            const double x { currentX + frac * humpSpan };
            const double y { currentY + humpAmp * 0.5 * (1.0 - std::cos(kTwoPi * frac)) };
            samples.push_back({ x, y });
        }
        currentX += humpSpan;
    }
    course.terrain.addMaterialZone(humpsStart, currentX, MaterialType::Dirt);

    // 3. Ascending step-up ledges (Tarmac)
    const double ledgesStart { currentX };
    const std::size_t numLedges { std::max(std::size_t { 1 }, config.stepLedgeCount) };
    const double ledgeH { std::max(0.5, config.stepLedgeHeight) };
    const double ledgeW { std::max(2.0, config.stepLedgeWidth) };

    for (std::size_t l { 0 }; l < numLedges; ++l) {
        currentX += 0.25;
        currentY += ledgeH;
        samples.push_back({ currentX, currentY });

        currentX += ledgeW;
        samples.push_back({ currentX, currentY });
    }
    course.terrain.addMaterialZone(ledgesStart, currentX, MaterialType::Tarmac);

    // 4. Steep mountain climb (Gravel)
    const double climbStart { currentX };
    const double climbLen { std::max(5.0, config.climbLength) };
    const double climbRise { climbLen * config.climbSlope };
    currentX += climbLen;
    currentY += climbRise;
    samples.push_back({ currentX, currentY });
    course.terrain.addMaterialZone(climbStart, currentX, MaterialType::Gravel);

    // 5. Rocky rough terrain section (Mud / Gravel)
    const double roughStart { currentX };
    const double roughLen { std::max(5.0, config.roughLength) };
    const double roughAmp { std::max(0.1, config.roughAmplitude) };
    const std::size_t roughSteps { static_cast<std::size_t>(roughLen * 2.0) };

    for (std::size_t r { 1 }; r <= roughSteps; ++r) {
        const double frac { static_cast<double>(r) / static_cast<double>(roughSteps) };
        const double x { roughStart + frac * roughLen };
        const double noiseOffset { roughAmp
            * (0.6 * std::sin(3.5 * frac * roughLen) + 0.4 * std::cos(7.2 * frac * roughLen)) };
        samples.push_back({ x, currentY + noiseOffset });
    }
    currentX += roughLen;
    course.terrain.addMaterialZone(roughStart, currentX, MaterialType::Mud);

    // 6. Flat finish landing platform (Tarmac)
    const double finishStart { currentX };
    const double finishLen { std::max(5.0, config.finishLength) };
    samples.push_back({ currentX, currentY });
    currentX += finishLen;
    samples.push_back({ currentX, currentY });
    course.terrain.addMaterialZone(finishStart, currentX, MaterialType::Tarmac);

    course.terrain.setSamples(samples);
    course.terrain.setInterpolation(HeightfieldInterp::Linear);
    course.terrain.setBoundaryMode(HeightfieldBoundary::Clamp);

    course.totalLength = currentX;
    course.finishHeight = currentY;
    course.startPosition = math::Vec2 { startLen * 0.5, 0.0 };
    course.finishPosition = math::Vec2 { finishStart + finishLen * 0.5, currentY };

    return course;
}

PlatformerLevel TrackGenerator::generatePlatformerLevel(const PlatformerLevelConfig& config)
{
    PlatformerLevel level { };
    DeterministicRng rng { config.seed };

    const double halfWidth { std::max(20.0, config.levelWidth) * 0.5 };
    const double minX { -halfWidth };
    const double maxX { halfWidth };
    const double tierH { std::max(1.5, config.tierHeight) };
    const std::size_t numTiers { std::max(std::size_t { 1 }, config.tierCount) };

    level.tierCount = numTiers;
    level.spawnPoint = math::Vec2 { minX + 5.0, config.groundY };

    // Continuous Ground Floor (Tier 0)
    if (config.hasGroundFloor) {
        dynamics::SideViewPlatform ground { };
        ground.y = config.groundY;
        ground.minX = minX;
        ground.maxX = maxX;
        ground.friction = SurfaceMaterial::getPreset(MaterialType::Tarmac).friction;
        ground.oneWay = false;
        level.platforms.push_back(ground);
    }

    // Floating Platform Tiers (Tier 1 .. numTiers)
    for (std::size_t t { 1 }; t <= numTiers; ++t) {
        const double tierY { config.groundY + static_cast<double>(t) * tierH };
        double cursorX { minX + rng.nextDouble(1.0, 4.0) };

        while (cursorX < maxX - 4.0) {
            const double platLen { rng.nextDouble(config.platformMinLength, config.platformMaxLength) };
            const double platEnd { std::min(cursorX + platLen, maxX) };

            if (platEnd - cursorX >= config.platformMinLength * 0.6) {
                dynamics::SideViewPlatform plat { };
                plat.y = tierY;
                plat.minX = cursorX;
                plat.maxX = platEnd;
                plat.oneWay = config.oneWayPlatforms;

                // Determine hazard property
                if (rng.nextBool(config.hazardProbability)) {
                    if (rng.nextBool(0.5)) {
                        plat.friction = SurfaceMaterial::getPreset(MaterialType::Ice).friction;
                    } else {
                        plat.friction = SurfaceMaterial::getPreset(MaterialType::OilSlick).friction;
                    }
                } else {
                    plat.friction = SurfaceMaterial::getPreset(MaterialType::Tarmac).friction;
                }

                level.platforms.push_back(plat);
            }

            cursorX = platEnd + rng.nextDouble(config.gapMinLength, config.gapMaxLength);
        }
    }

    // Set goal point near the right edge of top tier
    const double topTierY { config.groundY + static_cast<double>(numTiers) * tierH };
    level.goalPoint = math::Vec2 { maxX - 8.0, topTierY };

    return level;
}

HillClimbCourse TrackGenerator::generateHillClimb(const HillClimbConfig& config)
{
    HillClimbCourse course { };
    DeterministicRng rng { config.seed };

    const double totalLen { std::max(50.0, config.totalLength) };
    const double baseSlope { std::max(0.05, config.baseSlope) };
    std::vector<HeightSample1D> samples { };

    // Base uniform samples with elevated steep pitches
    const std::size_t numSamples { static_cast<std::size_t>(totalLen / 10.0) + 1U };
    const double stepDx { totalLen / static_cast<double>(numSamples - 1U) };

    double currentX { 0.0 };
    double currentY { 0.0 };
    samples.push_back({ currentX, currentY });

    // Procedural pitch distribution
    const std::size_t steepInterval { numSamples / (config.steepPitchCount + 1U) };

    for (std::size_t i { 1 }; i < numSamples; ++i) {
        currentX += stepDx;
        const bool isSteep { (i % steepInterval == 0U) && (i < numSamples - 1U) };
        const double slope { isSteep ? config.steepSlope : baseSlope };
        currentY += slope * stepDx;
        samples.push_back({ currentX, currentY });
    }

    course.terrain.setSamples(samples);
    course.terrain.setInterpolation(HeightfieldInterp::CubicHermite);
    course.terrain.setBoundaryMode(HeightfieldBoundary::Clamp);
    course.terrain.setDefaultMaterial(SurfaceMaterial::getPreset(MaterialType::Dirt));

    // Fractal noise overlay
    FractalConfig fractalConfig { };
    fractalConfig.octaves = 4;
    fractalConfig.persistence = 0.5;
    fractalConfig.lacunarity = 2.0;
    fractalConfig.seed = config.seed;
    course.terrain.setNoiseConfig(fractalConfig);
    course.terrain.setNoiseTransform(config.noiseAmplitude, config.noiseFrequency, 0.0);
    course.terrain.enableNoise(true);

    // Add mud zones at intermediate elevations
    const double mudSpan { totalLen / 6.0 };
    for (std::size_t m { 0 }; m < config.mudZoneCount; ++m) {
        const double startM { totalLen * 0.25 + static_cast<double>(m) * mudSpan };
        course.terrain.addMaterialZone(startM, startM + 20.0, MaterialType::Mud);
    }

    // Add ice zones near the peak
    const double iceSpan { totalLen / 8.0 };
    for (std::size_t ic { 0 }; ic < config.iceZoneCount; ++ic) {
        const double startIce { totalLen * 0.70 + static_cast<double>(ic) * iceSpan };
        course.terrain.addMaterialZone(startIce, startIce + 18.0, MaterialType::Ice);
    }

    course.totalElevationGain = course.terrain.getHeight(totalLen) - course.terrain.getHeight(0.0);
    course.startPosition = math::Vec2 { 0.0, course.terrain.getHeight(0.0) };
    course.peakPosition = math::Vec2 { totalLen, course.terrain.getHeight(totalLen) };

    // Find maximum slope angle across course
    double maxSlope { 0.0 };
    for (double x { 0.0 }; x <= totalLen; x += 1.0) {
        const double angle { std::abs(course.terrain.getSlopeAngle(x)) };
        if (angle > maxSlope) {
            maxSlope = angle;
        }
    }
    course.maxSlopeAngle = maxSlope;

    return course;
}

CircuitTrack2D TrackGenerator::generateCircuitTrack(const CircuitTrackConfig& config)
{
    CircuitTrack2D track { };
    track.trackWidth = std::max(4.0, config.trackWidth);
    track.shoulderWidth = std::max(1.0, config.shoulderWidth);
    track.isClosed = config.isClosed;

    const std::size_t numWaypoints { std::max(std::size_t { 8 }, config.waypointCount) };
    const double radX { std::max(20.0, config.radiusX) };
    const double radY { std::max(20.0, config.radiusY) };
    const double perturb { std::clamp(config.perturbation, 0.0, 0.40) };
    const double seedAngle { static_cast<double>(config.seed % 360U) * (kPi / 180.0) };

    // Dense polyline interpolation for smooth ribbon
    const std::size_t interpolatedSteps { numWaypoints * 8U };
    track.centerline.reserve(interpolatedSteps);
    track.leftBoundary.reserve(interpolatedSteps);
    track.rightBoundary.reserve(interpolatedSteps);

    double minBoundX { std::numeric_limits<double>::infinity() };
    double maxBoundX { -std::numeric_limits<double>::infinity() };
    double minBoundY { std::numeric_limits<double>::infinity() };
    double maxBoundY { -std::numeric_limits<double>::infinity() };

    for (std::size_t i { 0 }; i < interpolatedSteps; ++i) {
        const double theta { kTwoPi * (static_cast<double>(i) / static_cast<double>(interpolatedSteps)) };
        const double deltaR { perturb
            * (0.6 * std::sin(3.0 * theta + seedAngle) + 0.4 * std::cos(5.0 * theta + seedAngle)) };
        const double rX { radX * (1.0 + deltaR) };
        const double rY { radY * (1.0 + deltaR) };

        const math::Vec2 pos { rX * std::cos(theta), rY * std::sin(theta) };
        track.centerline.push_back(pos);

        minBoundX = std::min(minBoundX, pos.x);
        maxBoundX = std::max(maxBoundX, pos.x);
        minBoundY = std::min(minBoundY, pos.y);
        maxBoundY = std::max(maxBoundY, pos.y);
    }

    // Compute boundaries and total perimeter length
    double accumLength { 0.0 };
    const double halfW { track.trackWidth * 0.5 };

    for (std::size_t i { 0 }; i < interpolatedSteps; ++i) {
        const std::size_t prevIdx { (i + interpolatedSteps - 1U) % interpolatedSteps };
        const std::size_t nextIdx { (i + 1U) % interpolatedSteps };

        const math::Vec2 tangent { (track.centerline[nextIdx] - track.centerline[prevIdx]).normalized() };
        const math::Vec2 normal { -tangent.y, tangent.x }; // Left normal

        track.leftBoundary.push_back(track.centerline[i] + normal * halfW);
        track.rightBoundary.push_back(track.centerline[i] - normal * halfW);

        accumLength += (track.centerline[nextIdx] - track.centerline[i]).length();
    }
    track.trackLength = accumLength;

    // Configure camber and banking evaluator
    BankedTrackConfig bankConfig { };
    bankConfig.enabled = config.enableBanking;
    bankConfig.camberType = config.camberType;
    bankConfig.designSpeed = config.designSpeed;
    bankConfig.maxBankAngle = config.maxBankAngle;
    bankConfig.crownSlope = config.crownSlope;
    bankConfig.transitionLength = config.transitionLength;
    bankConfig.bowlCurvature = config.bowlCurvature;
    track.camberProfile.setConfig(bankConfig);

    // Compute raw curvature and equilibrium banking along centerline
    std::vector<double> rawBankAngles(interpolatedSteps, 0.0);
    track.bankAngles.resize(interpolatedSteps, 0.0);

    for (std::size_t i { 0 }; i < interpolatedSteps; ++i) {
        const std::size_t prevIdx { (i + interpolatedSteps - 1U) % interpolatedSteps };
        const std::size_t nextIdx { (i + 1U) % interpolatedSteps };

        const math::Vec2 v1 { track.centerline[i] - track.centerline[prevIdx] };
        const math::Vec2 v2 { track.centerline[nextIdx] - track.centerline[i] };
        const double d1 { v1.length() };
        const double d2 { v2.length() };
        const double ds { 0.5 * (d1 + d2) };

        if (ds > 1e-6 && d1 > 1e-6 && d2 > 1e-6) {
            const math::Vec2 u1 { v1 * (1.0 / d1) };
            const math::Vec2 u2 { v2 * (1.0 / d2) };
            const double cross { u1.x * u2.y - u1.y * u2.x };
            const double kappa { cross / ds };
            rawBankAngles[i] = track.camberProfile.computeBankAngle(kappa);
        }
    }

    // Smooth bank angles over transition runoff length
    const double avgStep { track.trackLength / static_cast<double>(interpolatedSteps) };
    const std::size_t winRadius { static_cast<std::size_t>(
        std::max(1.0, std::round(0.5 * std::max(5.0, config.transitionLength) / std::max(0.1, avgStep)))) };

    for (std::size_t i { 0 }; i < interpolatedSteps; ++i) {
        double weightedSum { 0.0 };
        double totalWeight { 0.0 };
        for (int offset { -static_cast<int>(winRadius) }; offset <= static_cast<int>(winRadius); ++offset) {
            const double distNorm { static_cast<double>(std::abs(offset)) / static_cast<double>(winRadius) };
            const double wgt { 1.0 - distNorm * distNorm }; // Epanechnikov parabolic filter
            const std::size_t sampleIdx { (i + static_cast<std::size_t>(offset + static_cast<int>(interpolatedSteps)))
                % interpolatedSteps };
            weightedSum += rawBankAngles[sampleIdx] * wgt;
            totalWeight += wgt;
        }
        track.bankAngles[i] = (totalWeight > 1e-9) ? (weightedSum / totalWeight) : rawBankAngles[i];
    }

    // Checkpoints distributed along perimeter
    const std::size_t checkpointCount { std::max(std::size_t { 4 }, numWaypoints / 4U) };
    for (std::size_t c { 0 }; c < checkpointCount; ++c) {
        const std::size_t idx { (c * interpolatedSteps) / checkpointCount };
        track.checkpoints.push_back(track.centerline[idx]);
    }

    // Starting grid setup
    track.startGridPosition = track.centerline[0];
    const math::Vec2 initialTangent { (track.centerline[1] - track.centerline[0]).normalized() };
    track.startHeading = std::atan2(initialTangent.y, initialTangent.x);

    // Initialize 2D surface map elevation grid & material
    track.surfaceMap.setDefaultMaterial(config.offtrackMaterial);

    constexpr std::size_t kGridDim { 64 };
    const double margin { track.trackWidth + track.shoulderWidth + 10.0 };
    const double gridMinX { minBoundX - margin };
    const double gridMinY { minBoundY - margin };
    const double gridMaxX { maxBoundX + margin };
    const double gridMaxY { maxBoundY + margin };
    const double stepX { (gridMaxX - gridMinX) / static_cast<double>(kGridDim - 1U) };
    const double stepY { (gridMaxY - gridMinY) / static_cast<double>(kGridDim - 1U) };

    std::vector<double> gridHeights(kGridDim * kGridDim, 0.0);
    const double elevAmp { config.elevationVariation * 0.5 };
    const double totalHalfWidth { (track.trackWidth + track.shoulderWidth) * 0.5 };

    for (std::size_t row { 0 }; row < kGridDim; ++row) {
        const double y { gridMinY + static_cast<double>(row) * stepY };
        for (std::size_t col { 0 }; col < kGridDim; ++col) {
            const double x { gridMinX + static_cast<double>(col) * stepX };
            const double baseElev { elevAmp * (std::sin(x / 40.0) + std::cos(y / 40.0)) };
            const math::Vec2 pt { x, y };

            // Find closest centerline segment
            double minDist { std::numeric_limits<double>::infinity() };
            std::size_t bestSeg { 0 };
            double bestT { 0.0 };

            for (std::size_t seg { 0 }; seg < interpolatedSteps; ++seg) {
                const math::Vec2& segA { track.centerline[seg] };
                const math::Vec2& segB { track.centerline[(seg + 1U) % interpolatedSteps] };
                double t { 0.0 };
                const double d { distToSegment(pt, segA, segB, &t) };
                if (d < minDist) {
                    minDist = d;
                    bestSeg = seg;
                    bestT = t;
                }
            }

            if (minDist <= totalHalfWidth) {
                const math::Vec2& segA { track.centerline[bestSeg] };
                const math::Vec2& segB { track.centerline[(bestSeg + 1U) % interpolatedSteps] };
                const math::Vec2 diff { segB - segA };
                const math::Vec2 tangent { diff.normalized() };
                const math::Vec2 latNorm { -tangent.y, tangent.x };
                const math::Vec2 proj { segA + diff * bestT };
                const double w { (pt - proj).dot(latNorm) };

                const double bankA { track.bankAngles[bestSeg] };
                const double bankB { track.bankAngles[(bestSeg + 1U) % interpolatedSteps] };
                const double bank { (1.0 - bestT) * bankA + bestT * bankB };

                const auto camber = track.camberProfile.evaluateCamber(bank, w, halfW, tangent);

                double weight { 1.0 };
                const double absW { std::abs(w) };
                if (absW > halfW) {
                    weight = std::clamp(1.0 - ((absW - halfW) / track.shoulderWidth), 0.0, 1.0);
                }

                gridHeights[row * kGridDim + col] = baseElev + weight * camber.elevationOffset;
            } else {
                gridHeights[row * kGridDim + col] = baseElev;
            }
        }
    }

    track.surfaceMap.setGrid(
        math::Vec2 { gridMinX, gridMinY }, math::Vec2 { stepX, stepY }, kGridDim, kGridDim, gridHeights);
    track.surfaceMap.setInterpolation(ElevationInterp2D::Bicubic);
    track.surfaceMap.setBoundaryMode(ElevationBoundary2D::Clamp);

    // Add material zones along track checkpoints for roadway and shoulders
    for (const auto& cp : track.checkpoints) {
        track.surfaceMap.addCircleZone(cp, track.trackWidth * 0.5, config.roadMaterial);
    }

    return track;
}

} // namespace vehicle_sim::terrain
