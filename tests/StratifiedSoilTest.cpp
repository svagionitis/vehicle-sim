#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/DeformableTerrain.h"
#include "libs/VehicleSimCore/terrain/StratifiedSoil.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

using namespace vehicle_sim::math;
using namespace vehicle_sim::terrain;

TEST(StratifiedSoilTest, PureUpperLayerSinkage)
{
    const auto config = StratifiedSoilSolver::getMudOnBedrock();

    // Moderate wheel load: 2000 N over 0.25m x 0.35m => ~22.8 kPa
    RutStamp2D stamp {};
    stamp.normalForce = 2000.0;
    stamp.velocity = { 5.0, 0.0 };
    stamp.width = 0.25;
    stamp.length = 0.35;

    const StratifiedSinkageResult result = StratifiedSoilSolver::evaluateStamp(stamp, config);

    EXPECT_GT(result.totalSinkage, 0.0);
    EXPECT_LT(result.totalSinkage, config.topLayer.layerDepth);
    EXPECT_FALSE(result.reachedSubstrate);
    EXPECT_DOUBLE_EQ(result.substrateSinkage, 0.0);
    EXPECT_DOUBLE_EQ(result.topLayerSinkage, result.totalSinkage);
    EXPECT_GT(result.compactionDrag, 0.0);
    EXPECT_GT(result.bulldozeDrag, 0.0);
    EXPECT_LT(result.dragForce.x, 0.0); // Opposing positive forward velocity
}

TEST(StratifiedSoilTest, BedrockClampingAtThreshold)
{
    const auto config = StratifiedSoilSolver::getMudOnBedrock();

    // Heavy normal load: 25000 N (2.5 tonnes on a single wheel)
    RutStamp2D stamp {};
    stamp.normalForce = 25000.0;
    stamp.velocity = { 4.0, 0.0 };
    stamp.width = 0.25;
    stamp.length = 0.30;

    const StratifiedSinkageResult result = StratifiedSoilSolver::evaluateStamp(stamp, config);

    EXPECT_TRUE(result.reachedSubstrate);
    EXPECT_DOUBLE_EQ(result.topLayerSinkage, config.topLayer.layerDepth);
    // Substrate has extremely high stiffness, so penetration into bedrock should be tiny (< 1 cm)
    EXPECT_LT(result.substrateSinkage, 0.015);
    EXPECT_NEAR(result.totalSinkage, config.topLayer.layerDepth, 0.015);
}

TEST(StratifiedSoilTest, ModulusTransitionSmoothness)
{
    const auto config = StratifiedSoilSolver::getMudOnBedrock();
    const double width = 0.25;
    const double dTop = config.topLayer.layerDepth;

    const double k1 = BekkerWongSolver::computeModulus(width, config.topLayer.soil);
    const double k2 = BekkerWongSolver::computeModulus(width, config.substrate.soil);

    EXPECT_LT(k1, k2);

    const double kWellBelow = StratifiedSoilSolver::computeModulus(dTop - 0.05, width, config);
    const double kAtInterface = StratifiedSoilSolver::computeModulus(dTop, width, config);
    const double kWellAbove = StratifiedSoilSolver::computeModulus(dTop + 0.05, width, config);

    EXPECT_NEAR(kWellBelow, k1, 1e-4);
    EXPECT_GT(kAtInterface, k1);
    EXPECT_LT(kAtInterface, k2);
    EXPECT_NEAR(kWellAbove, k2, 1e-4);

    // Monotonic increase across sweep
    double priorK = k1;
    for (int i = -20; i <= 20; ++i) {
        const double depth = dTop + (static_cast<double>(i) * 0.001);
        const double k = StratifiedSoilSolver::computeModulus(depth, width, config);
        EXPECT_GE(k, priorK - 1e-6);
        priorK = k;
    }
}

TEST(StratifiedSoilTest, BulldozeBoundedByTopStratum)
{
    const auto config = StratifiedSoilSolver::getMudOnBedrock();
    const double width = 0.25;
    const double dTop = config.topLayer.layerDepth;

    const double rbAtBedrock = StratifiedSoilSolver::computeBulldozeDrag(dTop, width, config);
    const double rbBeyondBedrock = StratifiedSoilSolver::computeBulldozeDrag(dTop + 0.20, width, config);

    // Bulldozing drag must not increase when sinkage attempts to penetrate bedrock
    EXPECT_DOUBLE_EQ(rbAtBedrock, rbBeyondBedrock);
    EXPECT_GT(rbAtBedrock, 0.0);
}

TEST(StratifiedSoilTest, CompactionWorkContinuity)
{
    const auto config = StratifiedSoilSolver::getMudOnBedrock();
    const double width = 0.25;
    const double dTop = config.topLayer.layerDepth;

    const double rcJustBelow = StratifiedSoilSolver::computeCompactionDrag(dTop - 1e-6, width, config);
    const double rcAtInterface = StratifiedSoilSolver::computeCompactionDrag(dTop, width, config);
    const double rcJustAbove = StratifiedSoilSolver::computeCompactionDrag(dTop + 1e-6, width, config);

    EXPECT_NEAR(rcJustBelow, rcAtInterface, 0.02);
    EXPECT_NEAR(rcJustAbove, rcAtInterface, 0.02);
}

TEST(StratifiedSoilTest, PresetsIntegrity)
{
    const auto mudBedrock = StratifiedSoilSolver::getMudOnBedrock();
    EXPECT_TRUE(mudBedrock.enableStratification);
    EXPECT_EQ(mudBedrock.topLayer.name, "ViscousMud");
    EXPECT_EQ(mudBedrock.substrate.name, "Bedrock");
    EXPECT_GT(mudBedrock.substrate.bearingCapacity, mudBedrock.topLayer.bearingCapacity);

    const auto snowAsphalt = StratifiedSoilSolver::getSnowOnAsphalt();
    EXPECT_TRUE(snowAsphalt.enableStratification);
    EXPECT_EQ(snowAsphalt.topLayer.name, "FreshSnow");
    EXPECT_EQ(snowAsphalt.substrate.name, "AsphaltPavement");
    EXPECT_GT(snowAsphalt.substrate.bearingCapacity, snowAsphalt.topLayer.bearingCapacity);
}

TEST(StratifiedSoilTest, DeformableLayerStratifiedRutStamp)
{
    DeformableTerrainLayer layer {};
    layer.initGrid(Vec2 { -10.0, -10.0 }, Vec2 { 0.5, 0.5 }, 40U, 40U);
    layer.setTopStratumDepth(0.10);

    const auto config = StratifiedSoilSolver::getMudOnBedrock();

    RutStamp2D stamp {};
    stamp.position = { 0.0, 0.0 };
    stamp.heading = 0.0;
    stamp.width = 0.40;
    stamp.length = 0.50;
    stamp.normalForce = 30000.0; // Heavy load
    stamp.velocity = { 2.0, 0.0 };

    const StratifiedSinkageResult result = layer.applyStratifiedRut(stamp, config);

    EXPECT_TRUE(result.reachedSubstrate);
    const double rutDepth = -layer.getRutDelta(0.0, 0.0);
    EXPECT_GT(rutDepth, 0.05);
    EXPECT_LT(rutDepth, 0.15); // Capped near top stratum depth
}
