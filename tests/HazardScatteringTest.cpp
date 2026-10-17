#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/Collider2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/HazardScattering.h"
#include "libs/VehicleSimCore/terrain/PoissonDiskSampler.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/TrackGenerator.h"

using namespace vehicle_sim;
using namespace vehicle_sim::terrain;
using namespace vehicle_sim::math;

namespace {

CircuitTrack2D createTestCircuit(std::uint32_t seed = 2024U, bool enableHazards = false)
{
    CircuitTrackConfig cfg { };
    cfg.radiusX = 80.0;
    cfg.radiusY = 60.0;
    cfg.waypointCount = 20U;
    cfg.trackWidth = 10.0;
    cfg.shoulderWidth = 4.0;
    cfg.elevationVariation = 0.0;
    cfg.seed = seed;
    cfg.enableHazards = enableHazards;
    cfg.hazardConfig.minDistance = 6.0;
    cfg.hazardConfig.seed = seed + 100U;
    return TrackGenerator::generateCircuitTrack(cfg);
}

} // namespace

// =============================================================================
// PoissonDiskSampler Unit Tests
// =============================================================================

TEST(PoissonSamplerTest, EnforcesMinimumDistance)
{
    PoissonSamplerConfig cfg { };
    cfg.minDistance = 5.0;
    cfg.bounds = AABB2D { Vec2 { -50.0, -50.0 }, Vec2 { 50.0, 50.0 } };
    cfg.seed = 42U;
    cfg.maxAttempts = 30U;

    PoissonDiskSampler sampler { cfg };
    const std::vector<Vec2> samples { sampler.generate() };

    ASSERT_GT(samples.size(), 30U);

    const double minDistSq { (cfg.minDistance - 1e-6) * (cfg.minDistance - 1e-6) };
    for (std::size_t i { 0U }; i < samples.size(); ++i) {
        for (std::size_t j { i + 1U }; j < samples.size(); ++j) {
            const double distSq { (samples[i] - samples[j]).lengthSquared() };
            EXPECT_GE(distSq, minDistSq)
                << "Samples " << i << " and " << j << " violate minDistance!";
        }
    }
}

TEST(PoissonSamplerTest, DeterministicIdenticalSeeds)
{
    PoissonSamplerConfig cfg { };
    cfg.minDistance = 4.0;
    cfg.bounds = AABB2D { Vec2 { -40.0, -40.0 }, Vec2 { 40.0, 40.0 } };
    cfg.seed = 9999U;

    PoissonDiskSampler samplerA { cfg };
    PoissonDiskSampler samplerB { cfg };

    const std::vector<Vec2> samplesA { samplerA.generate() };
    const std::vector<Vec2> samplesB { samplerB.generate() };

    ASSERT_EQ(samplesA.size(), samplesB.size());
    for (std::size_t i { 0U }; i < samplesA.size(); ++i) {
        EXPECT_DOUBLE_EQ(samplesA[i].x, samplesB[i].x);
        EXPECT_DOUBLE_EQ(samplesA[i].y, samplesB[i].y);
    }
}

TEST(PoissonSamplerTest, DivergesOnDifferentSeeds)
{
    PoissonSamplerConfig cfgA { };
    cfgA.minDistance = 6.0;
    cfgA.bounds = AABB2D { Vec2 { -40.0, -40.0 }, Vec2 { 40.0, 40.0 } };
    cfgA.seed = 1111U;

    PoissonSamplerConfig cfgB { cfgA };
    cfgB.seed = 2222U;

    PoissonDiskSampler samplerA { cfgA };
    PoissonDiskSampler samplerB { cfgB };

    const std::vector<Vec2> samplesA { samplerA.generate() };
    const std::vector<Vec2> samplesB { samplerB.generate() };

    bool anyDifference { samplesA.size() != samplesB.size() };
    if (!anyDifference) {
        for (std::size_t i { 0U }; i < samplesA.size(); ++i) {
            if (std::abs(samplesA[i].x - samplesB[i].x) > 1e-5 ||
                std::abs(samplesA[i].y - samplesB[i].y) > 1e-5) {
                anyDifference = true;
                break;
            }
        }
    }
    EXPECT_TRUE(anyDifference);
}

TEST(PoissonSamplerTest, StrictBoundaryContainment)
{
    PoissonSamplerConfig cfg { };
    cfg.minDistance = 5.0;
    cfg.bounds = AABB2D { Vec2 { -30.0, -20.0 }, Vec2 { 30.0, 20.0 } };
    cfg.seed = 777U;

    PoissonDiskSampler sampler { cfg };
    const std::vector<Vec2> samples { sampler.generate() };

    ASSERT_FALSE(samples.empty());
    for (const Vec2& pt : samples) {
        EXPECT_TRUE(cfg.bounds.contains(pt))
            << "Point (" << pt.x << ", " << pt.y << ") lies outside domain bounds!";
    }
}

TEST(PoissonSamplerTest, HonorsAcceptancePredicate)
{
    PoissonSamplerConfig cfg { };
    cfg.minDistance = 4.0;
    cfg.bounds = AABB2D { Vec2 { -50.0, -50.0 }, Vec2 { 50.0, 50.0 } };
    cfg.seed = 1234U;

    // Exclude points inside circle of radius 20 at center
    const double excludedRadiusSq { 20.0 * 20.0 };
    auto predicate = [&](const Vec2& pt) -> bool {
        return pt.lengthSquared() >= excludedRadiusSq;
    };

    PoissonDiskSampler sampler { cfg };
    const std::vector<Vec2> samples { sampler.generate(predicate) };

    ASSERT_FALSE(samples.empty());
    for (const Vec2& pt : samples) {
        EXPECT_GE(pt.lengthSquared(), excludedRadiusSq - 1e-6)
            << "Point fell inside excluded circle region!";
    }
}

TEST(PoissonSamplerTest, SeedPointsPreserved)
{
    PoissonSamplerConfig cfg { };
    cfg.minDistance = 5.0;
    cfg.bounds = AABB2D { Vec2 { -40.0, -40.0 }, Vec2 { 40.0, 40.0 } };
    cfg.seed = 42U;

    const std::vector<Vec2> seedPoints {
        Vec2 { 0.0, 0.0 },
        Vec2 { 10.0, 10.0 }
    };

    PoissonDiskSampler sampler { cfg };
    const std::vector<Vec2> samples { sampler.generateWithSeedPoints(seedPoints) };

    ASSERT_GE(samples.size(), 2U);
    EXPECT_DOUBLE_EQ(samples[0].x, 0.0);
    EXPECT_DOUBLE_EQ(samples[0].y, 0.0);
    EXPECT_DOUBLE_EQ(samples[1].x, 10.0);
    EXPECT_DOUBLE_EQ(samples[1].y, 10.0);
}

// =============================================================================
// HazardScattering Domain Tests
// =============================================================================

TEST(HazardScatteringTest, EmptyTrackReturnsEmpty)
{
    CircuitTrack2D emptyTrack { };
    HazardScatteringConfig cfg { };

    const std::vector<ScatteredHazard> hazards {
        HazardScattering::scatterOnTrack(emptyTrack, cfg)
    };
    EXPECT_TRUE(hazards.empty());
}

TEST(HazardScatteringTest, ZoneFilteringRibbonOnly)
{
    const CircuitTrack2D track { createTestCircuit(2024U, false) };
    HazardScatteringConfig cfg { };
    cfg.minDistance = 6.0;
    cfg.zonePolicy = HazardZonePolicy::TrackRibbonOnly;
    cfg.startClearanceDistance = 10.0;
    cfg.finishClearanceDistance = 10.0;
    cfg.seed = 5555U;

    const std::vector<ScatteredHazard> hazards {
        HazardScattering::scatterOnTrack(track, cfg)
    };

    ASSERT_GT(hazards.size(), 5U);
    const double halfRoad { 0.5 * track.trackWidth + 0.1 };

    for (const ScatteredHazard& h : hazards) {
        const double dist { track.getDistanceToCenter(h.position) };
        EXPECT_LE(dist, halfRoad)
            << "Hazard placed outside roadway ribbon under TrackRibbonOnly policy!";
    }
}

TEST(HazardScatteringTest, ZoneFilteringShoulderOnly)
{
    const CircuitTrack2D track { createTestCircuit(2024U, false) };
    HazardScatteringConfig cfg { };
    cfg.minDistance = 6.0;
    cfg.zonePolicy = HazardZonePolicy::ShoulderOnly;
    cfg.startClearanceDistance = 10.0;
    cfg.finishClearanceDistance = 10.0;
    cfg.seed = 8888U;

    const std::vector<ScatteredHazard> hazards {
        HazardScattering::scatterOnTrack(track, cfg)
    };

    ASSERT_GT(hazards.size(), 5U);
    const double halfRoad { 0.5 * track.trackWidth - 0.1 };
    const double halfShoulder { (0.5 * track.trackWidth) + track.shoulderWidth + 0.1 };

    for (const ScatteredHazard& h : hazards) {
        const double dist { track.getDistanceToCenter(h.position) };
        EXPECT_GT(dist, halfRoad)
            << "Hazard placed on roadway under ShoulderOnly policy!";
        EXPECT_LE(dist, halfShoulder)
            << "Hazard placed beyond shoulder under ShoulderOnly policy!";
    }
}

TEST(HazardScatteringTest, StartLineSafetyBuffer)
{
    const CircuitTrack2D track { createTestCircuit(2024U, false) };
    HazardScatteringConfig cfg { };
    cfg.startClearanceDistance = 40.0;
    cfg.finishClearanceDistance = 25.0;
    cfg.seed = 4321U;

    const std::vector<ScatteredHazard> hazards {
        HazardScattering::scatterOnTrack(track, cfg)
    };

    ASSERT_GT(hazards.size(), 5U);
    for (const ScatteredHazard& h : hazards) {
        const double s { h.trackProgress * track.trackLength };
        EXPECT_GE(s, cfg.startClearanceDistance - 1e-4)
            << "Hazard violated starting line clearance buffer!";
        EXPECT_LE(s, (track.trackLength - cfg.finishClearanceDistance) + 1e-4)
            << "Hazard violated finish line clearance buffer!";
    }
}

TEST(HazardScatteringTest, MaterialStampingMutatesGrid)
{
    const CircuitTrack2D track { createTestCircuit(2024U, false) };
    SurfaceMap2D surfaceMap { };
    surfaceMap.setDefaultMaterial(SurfaceMaterial::getPreset(MaterialType::Tarmac));

    ScatteredHazard oilHazard { };
    oilHazard.position = Vec2 { 20.0, 30.0 };
    oilHazard.boundingRadius = 2.0;
    oilHazard.type = HazardType::OilSlick;

    ScatteredHazard puddleHazard { };
    puddleHazard.position = Vec2 { -40.0, 10.0 };
    puddleHazard.boundingRadius = 1.8;
    puddleHazard.type = HazardType::StandingPuddle;

    const std::vector<ScatteredHazard> testHazards { oilHazard, puddleHazard };
    HazardScattering::stampSurfaceMaterials(surfaceMap, testHazards);

    const SurfaceMaterial oilMat { surfaceMap.getMaterial(oilHazard.position.x, oilHazard.position.y) };
    EXPECT_LT(oilMat.friction, 0.15); // Oil slick friction ~0.08

    const SurfaceMaterial puddleMat { surfaceMap.getMaterial(puddleHazard.position.x, puddleHazard.position.y) };
    EXPECT_LT(puddleMat.friction, 0.50); // Water/mud grip ~0.35
}

TEST(HazardScatteringTest, BoostPadForwardTangent)
{
    const CircuitTrack2D track { createTestCircuit(2024U, false) };
    HazardScatteringConfig cfg { };
    cfg.coneProbability = 0.0;
    cfg.oilSlickProbability = 0.0;
    cfg.puddleProbability = 0.0;
    cfg.rockProbability = 0.0;
    cfg.boostPadProbability = 1.0;
    cfg.seed = 7777U;

    const std::vector<ScatteredHazard> hazards {
        HazardScattering::scatterOnTrack(track, cfg)
    };

    ASSERT_FALSE(hazards.empty());
    for (const ScatteredHazard& h : hazards) {
        EXPECT_EQ(h.type, HazardType::BoostPad);
        EXPECT_DOUBLE_EQ(h.severity, 10.0);
    }
}

TEST(HazardScatteringTest, PhysicsCollidersSpawned)
{
    physics::PhysicsWorld2D world { Vec2 { 0.0, 0.0 } };
    HazardPhysicalProps props { };

    ScatteredHazard rockHazard { };
    rockHazard.position = Vec2 { 10.0, 20.0 };
    rockHazard.boundingRadius = 0.6;
    rockHazard.type = HazardType::Rock;

    ScatteredHazard coneHazard { };
    coneHazard.position = Vec2 { -15.0, 5.0 };
    coneHazard.boundingRadius = 0.25;
    coneHazard.type = HazardType::TrafficCone;

    ScatteredHazard oilHazard { }; // Non-physical body
    oilHazard.position = Vec2 { 0.0, 0.0 };
    oilHazard.type = HazardType::OilSlick;

    const std::vector<ScatteredHazard> testHazards { rockHazard, coneHazard, oilHazard };
    HazardScattering::spawnPhysicsColliders(world, testHazards, props);

    EXPECT_EQ(props.bodies.size(), 2U);
    EXPECT_EQ(props.colliders.size(), 2U);
    EXPECT_EQ(world.getBodyCount(), 2U);

    // Verify rock is static
    EXPECT_TRUE(props.bodies[0]->isStatic());
    EXPECT_DOUBLE_EQ(props.bodies[0]->getPosition().x, 10.0);
    EXPECT_DOUBLE_EQ(props.bodies[0]->getPosition().y, 20.0);

    // Verify cone is dynamic with mass 3.5 kg
    EXPECT_TRUE(props.bodies[1]->isDynamic());
    EXPECT_DOUBLE_EQ(props.bodies[1]->getMass(), 3.5);
}

TEST(HazardScatteringTest, CircuitTrackIntegratedGeneration)
{
    const CircuitTrack2D track { createTestCircuit(2024U, true) };

    EXPECT_GT(track.getHazardCount(), 0U);
    EXPECT_EQ(track.hazards.size(), track.getHazardCount());

    CircuitTrack2D mutableTrack { track };
    mutableTrack.clearHazards();
    EXPECT_EQ(mutableTrack.getHazardCount(), 0U);
    EXPECT_TRUE(mutableTrack.hazards.empty());
}
