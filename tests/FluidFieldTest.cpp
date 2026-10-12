#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/FluidDrag2D.h"
#include "libs/VehicleSimCore/physics/FluidField2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::CrosswindForces;
using vehicle_sim::physics::DiscreteGust2D;
using vehicle_sim::physics::FluidDrag2D;
using vehicle_sim::physics::FluidDragProperties;
using vehicle_sim::physics::FluidField2D;
using vehicle_sim::physics::FluidFieldConfig;
using vehicle_sim::physics::FluidMediumType;
using vehicle_sim::physics::FluidSample2D;
using vehicle_sim::physics::HarmonicGust2D;
using vehicle_sim::physics::PhysicsWorld2D;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kTolerance { 1e-6 };

// -----------------------------------------------------------------------------
// FluidField2D Construction & Baseline State
// -----------------------------------------------------------------------------

TEST(FluidFieldTest, DefaultConstructionAndDefaults)
{
    const FluidField2D field { };
    const FluidFieldConfig& config { field.getConfig() };

    EXPECT_DOUBLE_EQ(config.ambientWind.x, 0.0);
    EXPECT_DOUBLE_EQ(config.ambientWind.y, 0.0);
    EXPECT_DOUBLE_EQ(config.ambientCurrent.x, 0.0);
    EXPECT_DOUBLE_EQ(config.ambientCurrent.y, 0.0);
    EXPECT_DOUBLE_EQ(config.waterlineY, 0.0);
    EXPECT_DOUBLE_EQ(config.airDensity, 1.225);
    EXPECT_DOUBLE_EQ(config.waterDensity, 1025.0);
    EXPECT_FALSE(config.enableWindShear);
    EXPECT_FALSE(config.enableCurrentDecay);
    EXPECT_EQ(field.getDiscreteGustCount(), 0U);
    EXPECT_EQ(field.getHarmonicGustCount(), 0U);
}

TEST(FluidFieldTest, ParameterizedConfigConstruction)
{
    FluidFieldConfig config { };
    config.ambientWind = Vec2 { 8.5, 1.2 };
    config.ambientCurrent = Vec2 { 1.5, -0.2 };
    config.waterlineY = 10.0;
    config.airDensity = 1.20;
    config.waterDensity = 1000.0;
    config.enableWindShear = true;
    config.windShearRefHeight = 15.0;
    config.windShearExponent = 0.20;
    config.enableCurrentDecay = true;
    config.currentDecayDepth = 40.0;

    const FluidField2D field { config };
    const FluidFieldConfig& readConfig { field.getConfig() };

    EXPECT_DOUBLE_EQ(readConfig.ambientWind.x, 8.5);
    EXPECT_DOUBLE_EQ(readConfig.ambientWind.y, 1.2);
    EXPECT_DOUBLE_EQ(readConfig.ambientCurrent.x, 1.5);
    EXPECT_DOUBLE_EQ(readConfig.ambientCurrent.y, -0.2);
    EXPECT_DOUBLE_EQ(readConfig.waterlineY, 10.0);
    EXPECT_DOUBLE_EQ(readConfig.airDensity, 1.20);
    EXPECT_DOUBLE_EQ(readConfig.waterDensity, 1000.0);
    EXPECT_TRUE(readConfig.enableWindShear);
    EXPECT_DOUBLE_EQ(readConfig.windShearRefHeight, 15.0);
    EXPECT_DOUBLE_EQ(readConfig.windShearExponent, 0.20);
    EXPECT_TRUE(readConfig.enableCurrentDecay);
    EXPECT_DOUBLE_EQ(readConfig.currentDecayDepth, 40.0);
}

// -----------------------------------------------------------------------------
// Domain Partitioning & Density
// -----------------------------------------------------------------------------

TEST(FluidFieldTest, DomainPartitioningAirVsWater)
{
    FluidFieldConfig config { };
    config.waterlineY = 5.0;
    config.airDensity = 1.225;
    config.waterDensity = 1025.0;
    config.ambientWind = Vec2 { 12.0, 0.0 };
    config.ambientCurrent = Vec2 { 2.0, 0.0 };

    const FluidField2D field { config };

    // Point above waterline (Air domain)
    const Vec2 airPoint { 0.0, 10.0 };
    EXPECT_FALSE(field.isSubmerged(airPoint));
    EXPECT_DOUBLE_EQ(field.getDensity(airPoint), 1.225);
    const FluidSample2D airSample { field.sample(airPoint, 0.0) };
    EXPECT_EQ(airSample.medium, FluidMediumType::Air);
    EXPECT_DOUBLE_EQ(airSample.density, 1.225);
    EXPECT_DOUBLE_EQ(airSample.velocity.x, 12.0);
    EXPECT_DOUBLE_EQ(airSample.velocity.y, 0.0);

    // Point below waterline (Water domain)
    const Vec2 waterPoint { 0.0, 2.0 };
    EXPECT_TRUE(field.isSubmerged(waterPoint));
    EXPECT_DOUBLE_EQ(field.getDensity(waterPoint), 1025.0);
    const FluidSample2D waterSample { field.sample(waterPoint, 0.0) };
    EXPECT_EQ(waterSample.medium, FluidMediumType::Water);
    EXPECT_DOUBLE_EQ(waterSample.density, 1025.0);
    EXPECT_DOUBLE_EQ(waterSample.velocity.x, 2.0);
    EXPECT_DOUBLE_EQ(waterSample.velocity.y, 0.0);

    // Exact waterline boundary (treated as water / liquid interface)
    const Vec2 boundaryPoint { 0.0, 5.0 };
    EXPECT_TRUE(field.isSubmerged(boundaryPoint));
    EXPECT_DOUBLE_EQ(field.getDensity(boundaryPoint), 1025.0);
}

// -----------------------------------------------------------------------------
// Wind Shear & Current Depth Profiles
// -----------------------------------------------------------------------------

TEST(FluidFieldTest, HellmannWindShearProfile)
{
    FluidFieldConfig config { };
    config.waterlineY = 0.0;
    config.ambientWind = Vec2 { 10.0, 0.0 };
    config.enableWindShear = true;
    config.windShearRefHeight = 10.0;
    config.windShearExponent = 0.143;

    const FluidField2D field { config };

    // At reference height (y = 10.0): (10 / 10)^0.143 == 1.0
    const Vec2 refPos { 50.0, 10.0 };
    const Vec2 refVel { field.getVelocity(refPos, 0.0) };
    EXPECT_NEAR(refVel.x, 10.0, kTolerance);
    EXPECT_NEAR(refVel.y, 0.0, kTolerance);

    // At double reference height (y = 20.0): factor == 2.0^0.143
    const Vec2 highPos { 50.0, 20.0 };
    const Vec2 highVel { field.getVelocity(highPos, 0.0) };
    const double expectedHigh { 10.0 * std::pow(2.0, 0.143) };
    EXPECT_NEAR(highVel.x, expectedHigh, kTolerance);

    // Near surface (y = 0.1): factor == (0.1 / 10.0)^0.143
    const Vec2 lowPos { 50.0, 0.1 };
    const Vec2 lowVel { field.getVelocity(lowPos, 0.0) };
    const double expectedLow { 10.0 * std::pow(0.01, 0.143) };
    EXPECT_NEAR(lowVel.x, expectedLow, kTolerance);
}

TEST(FluidFieldTest, OceanCurrentDepthDecayProfile)
{
    FluidFieldConfig config { };
    config.waterlineY = 0.0;
    config.ambientCurrent = Vec2 { 3.0, 0.0 };
    config.enableCurrentDecay = true;
    config.currentDecayDepth = 20.0;

    const FluidField2D field { config };

    // At water surface (y = 0.0): exp(-0 / 20) == 1.0
    const Vec2 surfPos { 0.0, 0.0 };
    const Vec2 surfVel { field.getVelocity(surfPos, 0.0) };
    EXPECT_NEAR(surfVel.x, 3.0, kTolerance);

    // At depth y = -20.0: exp(-20 / 20) == exp(-1)
    const Vec2 deepPos { 0.0, -20.0 };
    const Vec2 deepVel { field.getVelocity(deepPos, 0.0) };
    EXPECT_NEAR(deepVel.x, 3.0 * std::exp(-1.0), kTolerance);

    // At depth y = -40.0: exp(-40 / 20) == exp(-2)
    const Vec2 deeperPos { 0.0, -40.0 };
    const Vec2 deeperVel { field.getVelocity(deeperPos, 0.0) };
    EXPECT_NEAR(deeperVel.x, 3.0 * std::exp(-2.0), kTolerance);
}

// -----------------------------------------------------------------------------
// Gust & Turbulence Models
// -----------------------------------------------------------------------------

TEST(FluidFieldTest, DiscreteOneCosineGustPulse)
{
    FluidFieldConfig config { };
    config.ambientWind = Vec2 { 5.0, 0.0 };
    FluidField2D field { config };

    DiscreteGust2D gust { };
    gust.direction = Vec2 { 1.0, 0.0 };
    gust.amplitude = 10.0;
    gust.startTime = 2.0;
    gust.duration = 4.0;
    field.addDiscreteGust(gust);
    EXPECT_EQ(field.getDiscreteGustCount(), 1U);

    const Vec2 testPos { 0.0, 10.0 };

    // Before gust initiation (t = 1.0): base wind only
    const Vec2 velPre { field.getVelocity(testPos, 1.0) };
    EXPECT_DOUBLE_EQ(velPre.x, 5.0);

    // At gust peak (t = 4.0, midpoint: 2.0 + 4.0/2): 1 - cos(pi) == 2, peak is 10.0
    const Vec2 velPeak { field.getVelocity(testPos, 4.0) };
    EXPECT_NEAR(velPeak.x, 5.0 + 10.0, kTolerance);

    // At quarter pulse (t = 3.0): 0.5 * (1 - cos(pi/2)) == 0.5 -> perturbation is 5.0
    const Vec2 velQuarter { field.getVelocity(testPos, 3.0) };
    EXPECT_NEAR(velQuarter.x, 5.0 + 5.0, kTolerance);

    // At pulse completion (t = 6.0): 0.5 * (1 - cos(2*pi)) == 0.0
    const Vec2 velEnd { field.getVelocity(testPos, 6.0) };
    EXPECT_NEAR(velEnd.x, 5.0, kTolerance);

    // After pulse completion (t = 7.0): back to ambient
    const Vec2 velPost { field.getVelocity(testPos, 7.0) };
    EXPECT_DOUBLE_EQ(velPost.x, 5.0);
}

TEST(FluidFieldTest, HarmonicTurbulenceWave)
{
    FluidFieldConfig config { };
    config.ambientWind = Vec2 { 0.0, 0.0 };
    FluidField2D field { config };

    HarmonicGust2D harmonic { };
    harmonic.waveVector = Vec2 { 0.1, 0.0 };
    harmonic.amplitude = Vec2 { 4.0, 0.0 };
    harmonic.frequency = 1.0;
    harmonic.phase = 0.0;
    field.addHarmonicGust(harmonic);
    EXPECT_EQ(field.getHarmonicGustCount(), 1U);

    // At origin at t = 0: sin(0) == 0
    EXPECT_NEAR(field.getVelocity(Vec2 { 0.0, 10.0 }, 0.0).x, 0.0, kTolerance);

    // At origin at t = pi/2: sin(-pi/2) == -1 -> -4.0
    const double tQuarter { kPi / 2.0 };
    EXPECT_NEAR(field.getVelocity(Vec2 { 0.0, 10.0 }, tQuarter).x, -4.0, kTolerance);

    // Repeatable evaluation (IEEE 754 determinism)
    const Vec2 v1 { field.getVelocity(Vec2 { 15.0, 10.0 }, 2.5) };
    const Vec2 v2 { field.getVelocity(Vec2 { 15.0, 10.0 }, 2.5) };
    EXPECT_DOUBLE_EQ(v1.x, v2.x);
    EXPECT_DOUBLE_EQ(v1.y, v2.y);
}

TEST(FluidFieldTest, ClearGustsRemovesAllTurbulence)
{
    FluidFieldConfig config { };
    config.ambientWind = Vec2 { 5.0, 0.0 };
    FluidField2D field { config };

    DiscreteGust2D discrete { };
    discrete.amplitude = 10.0;
    field.addDiscreteGust(discrete);

    HarmonicGust2D harmonic { };
    harmonic.amplitude = Vec2 { 2.0, 0.0 };
    field.addHarmonicGust(harmonic);

    EXPECT_EQ(field.getDiscreteGustCount(), 1U);
    EXPECT_EQ(field.getHarmonicGustCount(), 1U);

    field.clearGusts();
    EXPECT_EQ(field.getDiscreteGustCount(), 0U);
    EXPECT_EQ(field.getHarmonicGustCount(), 0U);

    const Vec2 vel { field.getVelocity(Vec2 { 0.0, 10.0 }, 1.5) };
    EXPECT_DOUBLE_EQ(vel.x, 5.0);
    EXPECT_DOUBLE_EQ(vel.y, 0.0);
}

TEST(FluidFieldTest, MutatorsUpdateFieldState)
{
    FluidField2D field { };
    field.setAmbientWind(Vec2 { 15.0, -2.0 });
    field.setAmbientCurrent(Vec2 { 1.2, 0.5 });
    field.setWaterline(12.5);

    EXPECT_DOUBLE_EQ(field.getConfig().ambientWind.x, 15.0);
    EXPECT_DOUBLE_EQ(field.getConfig().ambientWind.y, -2.0);
    EXPECT_DOUBLE_EQ(field.getConfig().ambientCurrent.x, 1.2);
    EXPECT_DOUBLE_EQ(field.getConfig().ambientCurrent.y, 0.5);
    EXPECT_DOUBLE_EQ(field.getConfig().waterlineY, 12.5);
}

// -----------------------------------------------------------------------------
// FluidDrag2D Relative Kinematics & Force Solvers
// -----------------------------------------------------------------------------

TEST(FluidDragTest, RelativeVelocityCalculation)
{
    const Vec2 bodyVel { 25.0, 0.0 };
    const Vec2 headwind { -10.0, 0.0 };
    const Vec2 tailwind { 10.0, 0.0 };
    const Vec2 crosswind { 0.0, 8.0 };

    // Headwind increases relative velocity
    const Vec2 vRelHead { FluidDrag2D::computeRelativeVel(bodyVel, headwind) };
    EXPECT_DOUBLE_EQ(vRelHead.x, 35.0);
    EXPECT_DOUBLE_EQ(vRelHead.y, 0.0);

    // Tailwind decreases relative velocity
    const Vec2 vRelTail { FluidDrag2D::computeRelativeVel(bodyVel, tailwind) };
    EXPECT_DOUBLE_EQ(vRelTail.x, 15.0);
    EXPECT_DOUBLE_EQ(vRelTail.y, 0.0);

    // Crosswind introduces lateral component
    const Vec2 vRelCross { FluidDrag2D::computeRelativeVel(bodyVel, crosswind) };
    EXPECT_DOUBLE_EQ(vRelCross.x, 25.0);
    EXPECT_DOUBLE_EQ(vRelCross.y, -8.0);
}

TEST(FluidDragTest, QuadraticDragForceEvaluation)
{
    const Vec2 vRel { 20.0, 0.0 };
    const double rho { 1.225 };
    const double cd { 0.30 };
    const double area { 2.0 };

    // F_drag = -0.5 * 1.225 * 0.30 * 2.0 * 20^2 = -0.5 * 0.735 * 400 = -147.0 N
    const Vec2 drag { FluidDrag2D::computeDragForce(vRel, rho, cd, area) };
    EXPECT_NEAR(drag.x, -147.0, kTolerance);
    EXPECT_NEAR(drag.y, 0.0, kTolerance);

    // Zero relative speed yields zero drag force
    const Vec2 zeroDrag { FluidDrag2D::computeDragForce(Vec2 { 0.0, 0.0 }, rho, cd, area) };
    EXPECT_DOUBLE_EQ(zeroDrag.x, 0.0);
    EXPECT_DOUBLE_EQ(zeroDrag.y, 0.0);

    // Invalid/non-positive parameters yield zero drag force
    EXPECT_DOUBLE_EQ(FluidDrag2D::computeDragForce(vRel, 0.0, cd, area).x, 0.0);
    EXPECT_DOUBLE_EQ(FluidDrag2D::computeDragForce(vRel, rho, 0.0, area).x, 0.0);
    EXPECT_DOUBLE_EQ(FluidDrag2D::computeDragForce(vRel, rho, cd, 0.0).x, 0.0);
}

TEST(FluidDragTest, CrosswindLateralForceAndOverturningTorque)
{
    FluidDragProperties props { };
    props.frontalArea = 3.0;
    props.frontalDragCoeff = 0.40;
    props.sideArea = 20.0;
    props.sideDragCoeff = 1.20;
    props.cpOffset = 0.50; // CP is 0.5m ahead of CM

    FluidSample2D sample { };
    sample.density = 1.225;
    // Pure crosswind blowing in +Y direction at 15 m/s
    sample.velocity = Vec2 { 0.0, 15.0 };

    // Vehicle driving forward along +X at 20 m/s with heading = 0
    const Vec2 bodyVel { 20.0, 0.0 };
    const double orientation { 0.0 };

    const CrosswindForces result {
        FluidDrag2D::computeCrosswind(bodyVel, orientation, sample, props)
    };

    // Forward relative velocity is +20 m/s -> opposing frontal drag localFx < 0
    // F_front = -0.5 * 1.225 * 0.40 * 3.0 * 20^2 = -294.0 N
    EXPECT_NEAR(result.frontalDrag, -294.0, kTolerance);

    // Lateral relative velocity is 0 - 15 = -15 m/s (airflow from positive Y towards -Y)
    // Lateral drag opposes relative velocity: pushes vehicle toward +Y (positive lateral force)
    // F_side = -0.5 * 1.225 * 1.20 * 20.0 * (-15) * 15 = +3307.5 N
    EXPECT_NEAR(result.sideForce, 3307.5, kTolerance);

    // Overturning / yaw moment: F_side * cpOffset = 3307.5 * 0.50 = 1653.75 N*m
    EXPECT_NEAR(result.yawMoment, 1653.75, kTolerance);

    // Total world force
    EXPECT_NEAR(result.totalForce.x, -294.0, kTolerance);
    EXPECT_NEAR(result.totalForce.y, 3307.5, kTolerance);
}

// -----------------------------------------------------------------------------
// PhysicsWorld2D Registration Integration
// -----------------------------------------------------------------------------

TEST(FluidFieldTest, PhysicsWorldRegistration)
{
    PhysicsWorld2D world { };
    EXPECT_EQ(world.getFluidField(), nullptr);

    FluidField2D field { };
    world.setFluidField(&field);
    EXPECT_EQ(world.getFluidField(), &field);

    world.setFluidField(nullptr);
    EXPECT_EQ(world.getFluidField(), nullptr);
}

} // namespace vehicle_sim::testing
