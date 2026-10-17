#include "HazardScattering.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

#include "SurfaceMap2D.h"
#include "TrackGenerator.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::terrain {

namespace {

constexpr double kPi { 3.14159265358979323846 };

/// @class LocalHazardRng
/// @brief Deterministic 64-bit LCG for hazard archetype classification and rotation noise.
class LocalHazardRng {
public:
    explicit LocalHazardRng(std::uint32_t seed) noexcept
        : mState { (static_cast<std::uint64_t>(seed) != 0ULL) ? static_cast<std::uint64_t>(seed) : 1ULL }
    {
    }

    [[nodiscard]] std::uint64_t next() noexcept
    {
        mState = mState * 6364136223846793005ULL + 1442695040888963407ULL;
        return mState;
    }

    [[nodiscard]] double nextDouble(double minVal, double maxVal) noexcept
    {
        const double unit { static_cast<double>(next() >> 11U) / 9007199254740992.0 };
        return minVal + unit * (maxVal - minVal);
    }

private:
    std::uint64_t mState { 1ULL };
};

} // namespace

std::vector<ScatteredHazard> HazardScattering::scatterOnTrack(
    const CircuitTrack2D& track, const HazardScatteringConfig& config)
{
    std::vector<ScatteredHazard> results { };

    if (track.centerline.size() < 2U || track.trackLength <= 0.0) {
        return results;
    }

    // 1. Calculate corridor bounding box AABB2D
    double minX { track.centerline[0].x };
    double maxX { track.centerline[0].x };
    double minY { track.centerline[0].y };
    double maxY { track.centerline[0].y };

    for (const math::Vec2& pt : track.centerline) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
    }
    for (const math::Vec2& pt : track.leftBoundary) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
    }
    for (const math::Vec2& pt : track.rightBoundary) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
        minY = std::min(minY, pt.y);
        maxY = std::max(maxY, pt.y);
    }

    const double margin { track.shoulderWidth + 10.0 };
    const math::AABB2D trackBounds {
        math::Vec2 { minX - margin, minY - margin },
        math::Vec2 { maxX + margin, maxY + margin }
    };

    // 2. Zone filtering and exclusion predicate
    const double halfRoad { 0.5 * track.trackWidth };
    const double halfShoulder { halfRoad + track.shoulderWidth };

    auto predicate = [&](const math::Vec2& pt) -> bool {
        const double progress { track.getTrackProgress(pt) };
        const double s { progress * track.trackLength };

        if (s < config.startClearanceDistance) {
            return false;
        }
        if ((track.trackLength - s) < config.finishClearanceDistance) {
            return false;
        }

        const double dist { track.getDistanceToCenter(pt) };

        switch (config.zonePolicy) {
        case HazardZonePolicy::TrackRibbonOnly:
            return dist <= halfRoad;
        case HazardZonePolicy::ShoulderOnly:
            return (dist > halfRoad) && (dist <= halfShoulder);
        case HazardZonePolicy::TrackAndShoulder:
            return dist <= halfShoulder;
        case HazardZonePolicy::TrackBorders:
            return std::abs(dist - halfRoad) <= 1.5;
        }
        return false;
    };

    // 3. Poisson-disk blue-noise sampling
    PoissonSamplerConfig samplerCfg { };
    samplerCfg.minDistance = config.minDistance;
    samplerCfg.bounds = trackBounds;
    samplerCfg.seed = config.seed;
    samplerCfg.maxAttempts = 30U;

    PoissonDiskSampler sampler { samplerCfg };
    const std::vector<math::Vec2> rawPoints { sampler.generate(predicate) };

    if (rawPoints.empty()) {
        return results;
    }

    // 4. Classification and archetype decorator
    LocalHazardRng rng { config.seed + 777U };

    const double totalWeight {
        config.coneProbability + config.oilSlickProbability +
        config.puddleProbability + config.rockProbability + config.boostPadProbability
    };
    const double invTotalWeight { totalWeight > 1e-6 ? 1.0 / totalWeight : 1.0 };

    const double pCone { config.coneProbability * invTotalWeight };
    const double pOil { pCone + config.oilSlickProbability * invTotalWeight };
    const double pPuddle { pOil + config.puddleProbability * invTotalWeight };
    const double pRock { pPuddle + config.rockProbability * invTotalWeight };

    const std::size_t countToEmit { std::min(rawPoints.size(), config.maxHazardCount) };
    results.reserve(countToEmit);

    for (std::size_t i { 0U }; i < countToEmit; ++i) {
        const math::Vec2& pt { rawPoints[i] };
        ScatteredHazard h { };
        h.position = pt;
        h.trackProgress = track.getTrackProgress(pt);

        // Find closest centerline segment to evaluate tangent & lateral offset
        double bestDistSq { 1e12 };
        math::Vec2 bestTangent { 1.0, 0.0 };
        double bestOffset { 0.0 };

        const std::size_t segCount { track.centerline.size() - 1U };
        for (std::size_t seg { 0U }; seg < segCount; ++seg) {
            const math::Vec2& a { track.centerline[seg] };
            const math::Vec2& b { track.centerline[seg + 1U] };
            const math::Vec2 ab { b - a };
            const double lenSq { ab.lengthSquared() };
            if (lenSq < 1e-8) {
                continue;
            }
            const double t { std::clamp((pt - a).dot(ab) / lenSq, 0.0, 1.0) };
            const math::Vec2 proj { a + ab * t };
            const double distSq { (pt - proj).lengthSquared() };
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                const double segLen { std::sqrt(lenSq) };
                bestTangent = ab * (1.0 / segLen);
                // Normal vector N = (-Ty, Tx)
                const math::Vec2 normal { -bestTangent.y, bestTangent.x };
                bestOffset = (pt - proj).dot(normal);
            }
        }

        h.lateralOffset = bestOffset;
        const double trackHeading { std::atan2(bestTangent.y, bestTangent.x) };

        // Pick hazard type based on cumulative probability mass
        const double dice { rng.nextDouble(0.0, 1.0) };
        if (dice < pCone) {
            h.type = HazardType::TrafficCone;
            h.boundingRadius = 0.25;
            h.orientation = trackHeading + rng.nextDouble(-kPi / 4.0, kPi / 4.0);
            h.severity = 1.0;
        } else if (dice < pOil) {
            h.type = HazardType::OilSlick;
            h.boundingRadius = 1.6;
            h.orientation = rng.nextDouble(-kPi, kPi);
            h.severity = 0.08; // Friction multiplier
        } else if (dice < pPuddle) {
            h.type = HazardType::StandingPuddle;
            h.boundingRadius = 1.4;
            h.orientation = rng.nextDouble(-kPi, kPi);
            h.severity = 0.04; // Water depth in meters (40 mm)
        } else if (dice < pRock) {
            h.type = HazardType::Rock;
            h.boundingRadius = 0.55;
            h.orientation = rng.nextDouble(-kPi, kPi);
            h.severity = 1.0;
        } else {
            h.type = HazardType::BoostPad;
            h.boundingRadius = 1.5;
            h.orientation = trackHeading; // Boost pads strictly align with track tangent
            h.severity = 10.0; // Boost velocity increment in m/s
        }

        results.push_back(h);
    }

    return results;
}

HazardPhysicalProps::HazardPhysicalProps() = default;
HazardPhysicalProps::~HazardPhysicalProps() = default;
HazardPhysicalProps::HazardPhysicalProps(HazardPhysicalProps&&) noexcept = default;
HazardPhysicalProps& HazardPhysicalProps::operator=(HazardPhysicalProps&&) noexcept = default;

void HazardScattering::stampSurfaceMaterials(
    SurfaceMap2D& surfaceMap, const std::vector<ScatteredHazard>& hazards)
{
    for (const ScatteredHazard& h : hazards) {
        if (h.type == HazardType::OilSlick) {
            surfaceMap.stampMaterialDisc(
                h.position, h.boundingRadius, SurfaceMaterial::getPreset(MaterialType::OilSlick));
        } else if (h.type == HazardType::StandingPuddle) {
            SurfaceMaterial waterMat { SurfaceMaterial::getPreset(MaterialType::Water) };
            waterMat.friction = 0.35;
            waterMat.rollingDrag = 2.0;
            surfaceMap.stampMaterialDisc(h.position, h.boundingRadius, waterMat);
        }
    }
}

void HazardScattering::spawnPhysicsColliders(
    physics::PhysicsWorld2D& world,
    const std::vector<ScatteredHazard>& hazards,
    HazardPhysicalProps& outProps)
{
    for (const ScatteredHazard& h : hazards) {
        if (h.type == HazardType::Rock) {
            physics::RigidBodyConfig bodyCfg { };
            bodyCfg.bodyType = physics::BodyType::Static;
            bodyCfg.position = h.position;
            bodyCfg.orientation = h.orientation;

            auto rockBody = std::make_unique<physics::RigidBody2D>(bodyCfg);
            math::Circle2D circleShape { math::Vec2 { 0.0, 0.0 }, h.boundingRadius };
            auto collider = std::make_unique<physics::Collider2D>(circleShape, rockBody.get());

            world.addBody(*rockBody);
            outProps.bodies.push_back(std::move(rockBody));
            outProps.colliders.push_back(std::move(collider));
        } else if (h.type == HazardType::TrafficCone) {
            physics::RigidBodyConfig bodyCfg { };
            bodyCfg.bodyType = physics::BodyType::Dynamic;
            bodyCfg.position = h.position;
            bodyCfg.orientation = h.orientation;
            bodyCfg.mass = 3.5;
            bodyCfg.inertia = 0.5 * 3.5 * h.boundingRadius * h.boundingRadius;
            bodyCfg.linearDamping = 3.0;
            bodyCfg.angularDamping = 4.0;

            auto coneBody = std::make_unique<physics::RigidBody2D>(bodyCfg);
            math::Circle2D circleShape { math::Vec2 { 0.0, 0.0 }, h.boundingRadius };
            auto collider = std::make_unique<physics::Collider2D>(circleShape, coneBody.get());

            world.addBody(*coneBody);
            outProps.bodies.push_back(std::move(coneBody));
            outProps.colliders.push_back(std::move(collider));
        }
    }
}

} // namespace vehicle_sim::terrain
