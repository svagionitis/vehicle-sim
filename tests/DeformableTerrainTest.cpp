#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/DeformableTerrain.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

using namespace vehicle_sim::math;
using namespace vehicle_sim::terrain;

TEST(BekkerWongSolverTest, ZeroLoadProducesZeroSinkage)
{
    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    RutStamp2D stamp {};
    stamp.normalForce = 0.0;
    stamp.velocity = { 5.0, 0.0 };
    stamp.width = 0.30;
    stamp.length = 0.40;

    const SoilSinkageResult result = BekkerWongSolver::evaluateStamp(stamp, mud.soil);
    EXPECT_DOUBLE_EQ(result.sinkage, 0.0);
    EXPECT_DOUBLE_EQ(result.plasticSinkage, 0.0);
    EXPECT_DOUBLE_EQ(result.compactionDrag, 0.0);
    EXPECT_DOUBLE_EQ(result.bulldozeDrag, 0.0);
    EXPECT_DOUBLE_EQ(result.dragForce.x, 0.0);
    EXPECT_DOUBLE_EQ(result.dragForce.y, 0.0);
}

TEST(BekkerWongSolverTest, MudVsTarmacSinkageComparison)
{
    const auto tarmac = SurfaceMaterial::getPreset(MaterialType::Tarmac);
    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    RutStamp2D stamp {};
    stamp.normalForce = 4000.0; // 400 kg wheel load
    stamp.velocity = { 5.0, 0.0 };
    stamp.width = 0.25;
    stamp.length = 0.35;

    const SoilSinkageResult tarmacResult = BekkerWongSolver::evaluateStamp(stamp, tarmac.soil);
    EXPECT_DOUBLE_EQ(tarmacResult.sinkage, 0.0);
    EXPECT_DOUBLE_EQ(tarmacResult.compactionDrag, 0.0);

    const SoilSinkageResult mudResult = BekkerWongSolver::evaluateStamp(stamp, mud.soil);
    EXPECT_GT(mudResult.sinkage, 0.05);
    EXPECT_LT(mudResult.sinkage, 0.35);
    EXPECT_GT(mudResult.compactionDrag, 100.0);
    EXPECT_LT(mudResult.dragForce.x, 0.0); // Opposing positive forward velocity
}

TEST(BekkerWongSolverTest, CompactionDragIncreasesWithLoad)
{
    const auto dirt = SurfaceMaterial::getPreset(MaterialType::Dirt);

    RutStamp2D stampLight {};
    stampLight.normalForce = 2000.0;
    stampLight.velocity = { 8.0, 0.0 };
    stampLight.width = 0.25;
    stampLight.length = 0.30;

    RutStamp2D stampHeavy = stampLight;
    stampHeavy.normalForce = 6000.0;

    const SoilSinkageResult lightResult = BekkerWongSolver::evaluateStamp(stampLight, dirt.soil);
    const SoilSinkageResult heavyResult = BekkerWongSolver::evaluateStamp(stampHeavy, dirt.soil);

    EXPECT_GT(heavyResult.sinkage, lightResult.sinkage);
    EXPECT_GT(heavyResult.compactionDrag, lightResult.compactionDrag);
}

TEST(BekkerWongSolverTest, HardpanDepthClampingPreventsAbyss)
{
    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    RutStamp2D stampExtreme {};
    stampExtreme.normalForce = 500000.0; // 50 tons extreme force
    stampExtreme.velocity = { 2.0, 0.0 };
    stampExtreme.width = 0.20;
    stampExtreme.length = 0.20;

    const SoilSinkageResult result = BekkerWongSolver::evaluateStamp(stampExtreme, mud.soil);
    EXPECT_DOUBLE_EQ(result.sinkage, mud.soil.maxPlasticDepth);
}

TEST(BekkerWongSolverTest, BulldozingDragActiveAtDeepSinkage)
{
    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    const double shallowDrag = BekkerWongSolver::computeBulldozeDrag(0.02, 0.30, mud.soil);
    const double deepDrag = BekkerWongSolver::computeBulldozeDrag(0.20, 0.30, mud.soil);

    EXPECT_GT(shallowDrag, 0.0);
    EXPECT_GT(deepDrag, shallowDrag * 10.0); // Quadratic depth scaling
}

TEST(BekkerWongSolverTest, ZeroVelocityYieldsZeroNaNVector)
{
    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    RutStamp2D stampStop {};
    stampStop.normalForce = 4000.0;
    stampStop.velocity = { 0.0, 0.0 }; // Stopped vehicle
    stampStop.width = 0.25;
    stampStop.length = 0.30;

    const SoilSinkageResult result = BekkerWongSolver::evaluateStamp(stampStop, mud.soil);
    EXPECT_GT(result.sinkage, 0.0);
    EXPECT_DOUBLE_EQ(result.dragForce.x, 0.0);
    EXPECT_DOUBLE_EQ(result.dragForce.y, 0.0);
    EXPECT_FALSE(std::isnan(result.dragForce.x));
    EXPECT_FALSE(std::isnan(result.dragForce.y));
}

TEST(BekkerWongSolverTest, SoilModulusCalculations)
{
    SoilMechanicsConfig soil {};
    soil.cohesionModulus = 1000.0;
    soil.frictionModulus = 50000.0;

    const double keq = BekkerWongSolver::computeModulus(0.20, soil);
    // keq = (1000 / 0.2) + 50000 = 5000 + 50000 = 55000
    EXPECT_NEAR(keq, 55000.0, 1.0);
}

TEST(DeformableLayerTest, StampDepressesCenterElevation)
{
    DeformableTerrainLayer layer {};
    layer.initGrid({ -10.0, -10.0 }, { 0.20, 0.20 }, 100, 100);
    ASSERT_TRUE(layer.isInitialized());

    const auto dirt = SurfaceMaterial::getPreset(MaterialType::Dirt);

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.heading = 0.0;
    stamp.width = 0.40;
    stamp.length = 0.60;
    stamp.normalForce = 5000.0;
    stamp.velocity = { 4.0, 0.0 };

    const SoilSinkageResult result = layer.applyRutStamp(stamp, dirt.soil);
    EXPECT_GT(result.sinkage, 0.0);

    // Center point must be depressed (negative elevation delta)
    const double centerDelta = layer.getRutDelta(0.0, 0.0);
    EXPECT_LT(centerDelta, -0.01);
    EXPECT_NEAR(centerDelta, -result.sinkage, 0.05);

    // Far point outside stamp must be untouched
    const double farDelta = layer.getRutDelta(5.0, 5.0);
    EXPECT_DOUBLE_EQ(farDelta, 0.0);
}

TEST(DeformableLayerTest, BermMoundingExtrusion)
{
    DeformableTerrainLayer layer {};
    layer.initGrid({ -10.0, -10.0 }, { 0.05, 0.05 }, 400, 400);

    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.heading = 0.0;
    stamp.width = 0.40; // b/2 = 0.20
    stamp.length = 0.80;
    stamp.normalForce = 6000.0;
    stamp.velocity = { 5.0, 0.0 };

    layer.applyRutStamp(stamp, mud.soil);

    // Center must be depressed
    EXPECT_LT(layer.getRutDelta(0.0, 0.0), 0.0);

    // Shoulder point (y = 0.28, within berm width [0.20, 0.40]) must be elevated
    const double bermDelta = layer.getRutDelta(0.0, 0.28);
    EXPECT_GT(bermDelta, 0.0);
}

TEST(DeformableLayerTest, ViscoelasticRelaxationRefillsMudOverTime)
{
    DeformableTerrainLayer layer {};
    layer.initGrid({ -5.0, -5.0 }, { 0.10, 0.10 }, 100, 100);

    const auto mud = SurfaceMaterial::getPreset(MaterialType::Mud);
    ASSERT_GT(mud.soil.recoveryTime, 0.0);

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.width = 0.30;
    stamp.length = 0.50;
    stamp.normalForce = 4000.0;
    stamp.velocity = { 3.0, 0.0 };

    layer.applyRutStamp(stamp, mud.soil);

    const double initialDelta = layer.getRutDelta(0.0, 0.0);
    EXPECT_LT(initialDelta, 0.0);

    // Simulate 3 x recovery time
    layer.updateRelaxation(mud.soil.recoveryTime * 3.0, mud.soil);

    const double relaxedDelta = layer.getRutDelta(0.0, 0.0);
    // Elastic portion should have rebounded, leaving mainly plastic depression
    EXPECT_GT(relaxedDelta, initialDelta);
    EXPECT_LT(relaxedDelta, 0.0);
}

TEST(DeformableLayerTest, MultiPassCompactionHardensSoil)
{
    DeformableTerrainLayer layer {};
    layer.initGrid({ -5.0, -5.0 }, { 0.10, 0.10 }, 100, 100);

    const auto dirt = SurfaceMaterial::getPreset(MaterialType::Dirt);

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.width = 0.30;
    stamp.length = 0.40;
    stamp.normalForce = 4000.0;
    stamp.velocity = { 2.0, 0.0 };

    // Pass 1
    layer.applyRutStamp(stamp, dirt.soil);
    const double pass1Delta = layer.getRutDelta(0.0, 0.0);

    // Pass 2 over same rut
    layer.applyRutStamp(stamp, dirt.soil);
    const double pass2Delta = layer.getRutDelta(0.0, 0.0);

    const double pass1Increment = std::abs(pass1Delta);
    const double pass2Increment = std::abs(pass2Delta - pass1Delta);

    // Incremental sinkage of pass 2 must be smaller due to hardening
    EXPECT_LT(pass2Increment, pass1Increment);
}

TEST(DeformableLayerTest, ClearRutsResetsGrid)
{
    DeformableTerrainLayer layer {};
    layer.initGrid({ -5.0, -5.0 }, { 0.10, 0.10 }, 100, 100);

    const auto dirt = SurfaceMaterial::getPreset(MaterialType::Dirt);
    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.normalForce = 4000.0;
    stamp.velocity = { 2.0, 0.0 };

    layer.applyRutStamp(stamp, dirt.soil);
    EXPECT_GT(layer.getMaxRutDepth(), 0.0);

    layer.clearRuts();
    EXPECT_DOUBLE_EQ(layer.getMaxRutDepth(), 0.0);
    EXPECT_DOUBLE_EQ(layer.getRutDelta(0.0, 0.0), 0.0);
}

TEST(SurfaceMap2DTest, DeformableLayerDisabledByDefault)
{
    SurfaceMap2D map {};
    EXPECT_FALSE(map.isDeformableEnabled());
}

TEST(SurfaceMap2DTest, RutLayerModifiesSurfaceElevationSample)
{
    SurfaceMap2D map {};
    map.setBaseHeight(10.0);

    std::vector<double> elevations(100 * 100, 0.0);
    map.setGrid({ -10.0, -10.0 }, { 0.20, 0.20 }, 100, 100, elevations);
    map.setDefaultMaterial(SurfaceMaterial::getPreset(MaterialType::Mud));
    map.enableDeformableLayer(true);
    EXPECT_TRUE(map.isDeformableEnabled());

    // Baseline elevation without ruts
    EXPECT_DOUBLE_EQ(map.getHeight(0.0, 0.0), 10.0);

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.width = 0.30;
    stamp.length = 0.40;
    stamp.normalForce = 4000.0;
    stamp.velocity = { 3.0, 0.0 };

    const SoilSinkageResult result = map.applyRutStamp(stamp);
    EXPECT_GT(result.sinkage, 0.0);

    // Deformed elevation at rut center should be lower than baseline 10.0
    const double deformedH = map.getHeight(0.0, 0.0);
    EXPECT_LT(deformedH, 10.0);
    EXPECT_NEAR(deformedH, 10.0 - result.sinkage, 0.05);

    EXPECT_GT(map.getMaxRutDepth(), 0.0);

    map.clearRuts();
    EXPECT_DOUBLE_EQ(map.getHeight(0.0, 0.0), 10.0);
}

TEST(SurfaceMap2DTest, DeformedHeightMatchesRutDelta)
{
    SurfaceMap2D map {};
    map.setBaseHeight(5.0);
    std::vector<double> elevations(50 * 50, 0.0);
    map.setGrid({ -5.0, -5.0 }, { 0.20, 0.20 }, 50, 50, elevations);
    map.setDefaultMaterial(SurfaceMaterial::getPreset(MaterialType::Dirt));
    map.enableDeformableLayer(true);

    RutStamp2D stamp {};
    stamp.position = { 1.0, 2.0 };
    stamp.normalForce = 3500.0;
    stamp.velocity = { 2.0, 0.0 };

    map.applyRutStamp(stamp);

    const double delta = map.getRutDelta(1.0, 2.0);
    const double totalH = map.getHeight(1.0, 2.0);
    EXPECT_DOUBLE_EQ(totalH, 5.0 + delta);
}
