#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/HydroplaningModel.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

using namespace vehicle_sim::math;
using namespace vehicle_sim::terrain;
using namespace vehicle_sim::dynamics;

TEST(HydroplaningModelTest, NASAHorneStandardPressure)
{
    // Standard passenger car tire pressure: 32 psi
    const double vHydro32 = HydroplaningSolver::computeCriticalSpeed(32.0);
    // 6.36 * sqrt(32) ~= 35.9815 m/s (~129.5 km/h)
    EXPECT_NEAR(vHydro32, 35.9815, 0.01);

    // High performance tire pressure: 25 psi
    const double vHydro25 = HydroplaningSolver::computeCriticalSpeed(25.0);
    // 6.36 * sqrt(25) = 6.36 * 5 = 31.8 m/s (~114.5 km/h)
    EXPECT_DOUBLE_EQ(vHydro25, 31.8);

    // Zero or negative pressure guard
    EXPECT_DOUBLE_EQ(HydroplaningSolver::computeCriticalSpeed(0.0), 0.0);
    EXPECT_DOUBLE_EQ(HydroplaningSolver::computeCriticalSpeed(-10.0), 0.0);
}

TEST(HydroplaningModelTest, TreadEvacuationPreventsPlaning)
{
    // 2.0 mm water depth with new 8.0 mm tread depth and 0.35 void ratio:
    // Capacity = 8.0 * 0.35 = 2.8 mm > 2.0 mm => excess = 0.0
    const double unDrained = HydroplaningSolver::computeUnDrainedDepth(0.002, 0.008, 0.35);
    EXPECT_DOUBLE_EQ(unDrained, 0.0);

    HydroplaningConfig config {};
    config.tirePressurePsi = 32.0;
    config.treadDepth = 0.008;
    config.voidRatio = 0.35;

    const double frictionScale = HydroplaningSolver::computeFrictionScale(40.0, 36.0, unDrained, config);
    EXPECT_DOUBLE_EQ(frictionScale, 1.0);
}

TEST(HydroplaningModelTest, SlickTireCatastrophicPlaning)
{
    // Racing slick has 0.0 tread depth => all water is un-drained
    const double unDrained = HydroplaningSolver::computeUnDrainedDepth(0.003, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(unDrained, 0.003);

    HydroplaningConfig config {};
    config.tirePressurePsi = 28.0;
    config.treadDepth = 0.0;
    config.voidRatio = 0.0;

    const double criticalSpeed = HydroplaningSolver::computeCriticalSpeed(config.tirePressurePsi);

    // Well above critical speed in un-drained water
    const double forwardSpeed = criticalSpeed * 1.30;
    const double frictionScale = HydroplaningSolver::computeFrictionScale(
        forwardSpeed, criticalSpeed, unDrained, config);

    EXPECT_LT(frictionScale, 0.15);
    EXPECT_GE(frictionScale, config.minFrictionScale);
}

TEST(HydroplaningModelTest, HydroLiftReducesNormalLoad)
{
    HydroplaningConfig config {};
    config.tirePressurePsi = 30.0;
    config.treadDepth = 0.002; // Worn tire
    config.voidRatio = 0.25;

    HydroplaningState state {};
    state.forwardVelocity = 38.0; // Above critical speed
    state.normalLoad = 4000.0;
    state.patchWidth = 0.225;
    state.patchLength = 0.150;
    state.waterDepth = 0.005; // 5mm puddle
    state.velocityVector = { 38.0, 0.0 };

    const HydroplaningResult result = HydroplaningSolver::evaluateHydroplaning(state, config);

    EXPECT_TRUE(result.isHydroplaning);
    EXPECT_GT(result.hydroLiftForce, 100.0);
    EXPECT_LT(result.effectiveNormalLoad, state.normalLoad);
    EXPECT_DOUBLE_EQ(result.effectiveNormalLoad, state.normalLoad - result.hydroLiftForce);
    EXPECT_LT(result.frictionScale, 0.50);
    EXPECT_GT(result.sprayDragForce, 0.0);
    EXPECT_LT(result.sprayDragVector.x, 0.0); // Opposing forward velocity
}

TEST(HydroplaningModelTest, SprayDragIncreasesQuadratically)
{
    HydroplaningConfig config {};
    const double waterDepth = 0.004;
    const double patchWidth = 0.20;

    const double drag10 = HydroplaningSolver::computeSprayDrag(10.0, waterDepth, patchWidth, config);
    const double drag20 = HydroplaningSolver::computeSprayDrag(20.0, waterDepth, patchWidth, config);

    EXPECT_GT(drag10, 0.0);
    // Doubling velocity quadruples aerodynamic / fluid drag
    EXPECT_NEAR(drag20, drag10 * 4.0, 1e-4);
}

TEST(HydroplaningModelTest, ZeroSpeedYieldsZeroDrag)
{
    HydroplaningConfig config {};
    HydroplaningState state {};
    state.forwardVelocity = 0.0;
    state.normalLoad = 3500.0;
    state.waterDepth = 0.010;
    state.velocityVector = { 0.0, 0.0 };

    const HydroplaningResult result = HydroplaningSolver::evaluateHydroplaning(state, config);

    EXPECT_DOUBLE_EQ(result.hydroLiftForce, 0.0);
    EXPECT_DOUBLE_EQ(result.effectiveNormalLoad, state.normalLoad);
    EXPECT_DOUBLE_EQ(result.sprayDragForce, 0.0);
    EXPECT_DOUBLE_EQ(result.sprayDragVector.x, 0.0);
    EXPECT_DOUBLE_EQ(result.sprayDragVector.y, 0.0);
    EXPECT_DOUBLE_EQ(result.frictionScale, 1.0);
    EXPECT_FALSE(result.isHydroplaning);
}

TEST(HydroplaningModelTest, SurfaceMap2DWaterFilmSampling)
{
    SurfaceMap2D map {};
    map.setBaseWaterDepth(0.002);
    EXPECT_DOUBLE_EQ(map.getBaseWaterDepth(), 0.002);
    EXPECT_DOUBLE_EQ(map.getWaterDepth(10.0, 20.0), 0.002);

    // Configure 2x2 grid with varying water puddle
    map.setGrid(Vec2 { 0.0, 0.0 }, Vec2 { 10.0, 10.0 }, 2U, 2U, { 0.0, 0.0, 0.0, 0.0 });
    map.setWaterFilmGrid({ 0.0, 0.008, 0.0, 0.008 });

    // Midpoint along X at Y = 0: average between 0.0 and 0.008 = 0.004
    const double midWater = map.getWaterDepth(5.0, 0.0);
    EXPECT_NEAR(midWater, 0.004, 1e-4);
}

TEST(HydroplaningModelTest, PacejkaTireHydroplaningCoupling)
{
    PacejkaTireModel tire {};
    tire.loadPreset(TirePresetType::StandardRoad);

    TireState dryState {};
    dryState.forwardSpeed = 38.0; // High speed
    dryState.lateralSpeed = 2.0; // Small cornering slip
    dryState.angularVelocity = 38.0 / 0.32;
    dryState.normalLoad = 4000.0;
    dryState.surfaceFriction = 1.0;
    dryState.waterFilmDepth = 0.0; // Dry

    const TireForces dryForces = tire.evaluateForces(dryState, 0.016);
    EXPECT_FALSE(dryForces.isHydroplaning);
    EXPECT_DOUBLE_EQ(dryForces.effectiveLoadFz, 4000.0);
    EXPECT_DOUBLE_EQ(dryForces.wetFrictionScale, 1.0);
    EXPECT_GT(std::abs(dryForces.fLat), 500.0); // Good cornering force

    tire.reset();

    // Now test with 5 mm standing water film (deep puddle)
    TireState wetState = dryState;
    wetState.waterFilmDepth = 0.005;
    wetState.tirePressurePsi = 32.0;
    wetState.treadDepth = 0.003; // Worn tire
    wetState.treadVoidRatio = 0.25;

    const TireForces wetForces = tire.evaluateForces(wetState, 0.016);
    EXPECT_TRUE(wetForces.isHydroplaning);
    EXPECT_GT(wetForces.hydroLiftForce, 0.0);
    EXPECT_LT(wetForces.effectiveLoadFz, dryForces.effectiveLoadFz);
    EXPECT_LT(wetForces.wetFrictionScale, 0.60);
    EXPECT_LT(std::abs(wetForces.fLat), std::abs(dryForces.fLat) * 0.60); // Cornering severely compromised
}
