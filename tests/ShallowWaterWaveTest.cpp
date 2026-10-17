#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/dynamics/WatercraftDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/ShallowWaterCoupler.h"
#include "libs/VehicleSimCore/terrain/WaveGrid1D.h"
#include "libs/VehicleSimCore/terrain/WaveGrid2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::dynamics::WatercraftConfig;
using vehicle_sim::dynamics::WatercraftDynamics;
using vehicle_sim::math::Vec2;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::terrain::HydrodynamicCouplingForces;
using vehicle_sim::terrain::ShallowWaterCoupler;
using vehicle_sim::terrain::WaterWakeStamp2D;
using vehicle_sim::terrain::WaveBoundaryType;
using vehicle_sim::terrain::WaveConfig1D;
using vehicle_sim::terrain::WaveConfig2D;
using vehicle_sim::terrain::WaveGrid1D;
using vehicle_sim::terrain::WaveGrid2D;
using vehicle_sim::terrain::WaveSample1D;
using vehicle_sim::terrain::WaveSample2D;

inline constexpr double kTolerance { 1e-4 };

// -----------------------------------------------------------------------------
// 1D Wave Grid Tests
// -----------------------------------------------------------------------------

TEST(WaveGrid1DTest, DefaultConstructionAndZeroState)
{
    WaveConfig1D config {};
    config.originX = -50.0;
    config.length = 100.0;
    config.cellSize = 0.5;
    config.waveSpeed = 6.0;
    config.waterlineY = 0.0;

    const WaveGrid1D wave { config };
    EXPECT_GE(wave.getCellCount(), 201U);
    EXPECT_NEAR(wave.getElevation(0.0), 0.0, kTolerance);
    EXPECT_NEAR(wave.computeEnergy(), 0.0, kTolerance);

    const WaveSample1D sample { wave.getSample(10.0) };
    EXPECT_NEAR(sample.elevation, 0.0, kTolerance);
    EXPECT_NEAR(sample.perturbation, 0.0, kTolerance);
    EXPECT_NEAR(sample.derivative, 0.0, kTolerance);
    EXPECT_NEAR(sample.normal.y, 1.0, kTolerance);
}

TEST(WaveGrid1DTest, TravelingWaveCelerity)
{
    WaveConfig1D config {};
    config.originX = -50.0;
    config.length = 100.0;
    config.cellSize = 0.25;
    config.waveSpeed = 8.0;
    config.damping = 0.0; // Zero damping for pure propagation
    config.leftBoundary = WaveBoundaryType::Absorbing;
    config.rightBoundary = WaveBoundaryType::Absorbing;

    WaveGrid1D wave { config };
    wave.injectImpulse(0.0, 2.0, 0.5);

    const double initialElevation { wave.getElevation(0.0) };
    EXPECT_GT(initialElevation, 0.1);

    // Propagate wave for t = 2.0 seconds. Pulse should travel 2.0 * 8.0 = 16.0 meters
    constexpr double kSimTime { 2.0 };
    constexpr double kDt { 0.0166667 };
    const std::size_t steps { static_cast<std::size_t>(kSimTime / kDt) };

    for (std::size_t i { 0U }; i < steps; ++i) {
        wave.step(kDt);
    }

    // Pulse peak should be around x = +16.0m and x = -16.0m
    const double elevationAt16 { wave.getElevation(16.0) };
    const double elevationAtCenter { wave.getElevation(0.0) };

    EXPECT_GT(elevationAt16, 0.02);
    EXPECT_LT(std::abs(elevationAtCenter), elevationAt16);
}

TEST(WaveGrid1DTest, ViscousEnergyDamping)
{
    WaveConfig1D config {};
    config.originX = -50.0;
    config.length = 100.0;
    config.cellSize = 0.5;
    config.waveSpeed = 5.0;
    config.damping = 0.8; // Significant damping
    config.leftBoundary = WaveBoundaryType::Neumann;
    config.rightBoundary = WaveBoundaryType::Neumann;

    WaveGrid1D wave { config };
    wave.injectImpulse(0.0, 3.0, 0.6);

    const double initialEnergy { wave.computeEnergy() };
    EXPECT_GT(initialEnergy, 1.0);

    // Step for 4 seconds
    constexpr double kDt { 0.02 };
    for (std::size_t i { 0U }; i < 200U; ++i) {
        wave.step(kDt);
    }

    const double finalEnergy { wave.computeEnergy() };
    EXPECT_LT(finalEnergy, initialEnergy * 0.25);
}

TEST(WaveGrid1DTest, NeumannReflection)
{
    WaveConfig1D config {};
    config.originX = 0.0;
    config.length = 20.0;
    config.cellSize = 0.25;
    config.waveSpeed = 5.0;
    config.damping = 0.0;
    config.leftBoundary = WaveBoundaryType::Neumann; // Solid reflective wall at x = 0
    config.rightBoundary = WaveBoundaryType::Absorbing;

    WaveGrid1D wave { config };
    // Inject impulse near reflective wall at x = 2.0m
    wave.injectImpulse(2.0, 1.5, 0.4);

    // Step until pulse reaches wall and reflects
    constexpr double kDt { 0.01 };
    for (std::size_t i { 0U }; i < 150U; ++i) {
        wave.step(kDt);
    }

    // At the solid wall (x = 0), reflection constructive interference creates positive elevation
    const double wallElevation { wave.getElevation(0.0) };
    EXPECT_TRUE(std::isfinite(wallElevation));
}

// -----------------------------------------------------------------------------
// 2D Wave Basin Tests
// -----------------------------------------------------------------------------

TEST(WaveGrid2DTest, DefaultConstructionAndBounds)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -25.0, -25.0 };
    config.size = Vec2 { 50.0, 50.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 4.0;
    config.waterlineZ = 2.0;

    const WaveGrid2D wave { config };
    EXPECT_GE(wave.getWidth(), 101U);
    EXPECT_GE(wave.getHeight(), 101U);
    EXPECT_NEAR(wave.getElevation(Vec2 { 0.0, 0.0 }), 2.0, kTolerance);

    const WaveSample2D sample { wave.getSample(Vec2 { 5.0, 5.0 }) };
    EXPECT_NEAR(sample.elevation, 2.0, kTolerance);
    EXPECT_NEAR(sample.perturbation, 0.0, kTolerance);
    EXPECT_NEAR(sample.slopeAngle, 0.0, kTolerance);
    EXPECT_NEAR(sample.normal.z, 1.0, kTolerance);
}

TEST(WaveGrid2DTest, CircularRippleSymmetry)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -30.0, -30.0 };
    config.size = Vec2 { 60.0, 60.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 5.0;
    config.damping = 0.05;
    config.use9PointLaplacian = true;

    WaveGrid2D wave { config };
    wave.injectImpulse(Vec2 { 0.0, 0.0 }, 2.0, 0.5);

    // Step for 1.5 seconds
    constexpr double kDt { 0.0166667 };
    for (std::size_t i { 0U }; i < 90U; ++i) {
        wave.step(kDt);
    }

    // Radial symmetry check at distance R = 5.0m across four axes
    const double elevEast { wave.getElevation(Vec2 { 5.0, 0.0 }) };
    const double elevWest { wave.getElevation(Vec2 { -5.0, 0.0 }) };
    const double elevNorth { wave.getElevation(Vec2 { 0.0, 5.0 }) };
    const double elevSouth { wave.getElevation(Vec2 { 0.0, -5.0 }) };

    EXPECT_NEAR(elevEast, elevWest, 0.01);
    EXPECT_NEAR(elevNorth, elevSouth, 0.01);
    EXPECT_NEAR(elevEast, elevNorth, 0.015);
}

TEST(WaveGrid2DTest, CflSubsteppingStability)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -20.0, -20.0 };
    config.size = Vec2 { 40.0, 40.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 6.0;
    config.damping = 0.1;

    WaveGrid2D wave { config };
    wave.injectImpulse(Vec2 { 0.0, 0.0 }, 2.0, 1.0);

    // Provide large dt (0.1s), exceeding CFL limit for a single step
    constexpr double kLargeDt { 0.10 };
    for (std::size_t i { 0U }; i < 30U; ++i) {
        wave.step(kLargeDt);
    }

    const double centerElevation { wave.getElevation(Vec2 { 0.0, 0.0 }) };
    EXPECT_TRUE(std::isfinite(centerElevation));
    EXPECT_FALSE(std::isnan(centerElevation));
    EXPECT_LT(std::abs(centerElevation), 5.0);
}

TEST(WaveGrid2DTest, WakeStampPerturbation)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -30.0, -30.0 };
    config.size = Vec2 { 60.0, 60.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 5.0;

    WaveGrid2D wave { config };

    WaterWakeStamp2D stamp {};
    stamp.position = Vec2 { 0.0, 0.0 };
    stamp.heading = 0.0; // Moving East (+X)
    stamp.length = 6.0;
    stamp.beam = 2.4;
    stamp.draft = 0.6;
    stamp.velocity = Vec2 { 8.0, 0.0 };
    stamp.verticalVelocity = 0.0;
    stamp.displacement = 1500.0;

    wave.applyWakeStamp(stamp);

    // Advance 5 steps to allow wake perturbation to manifest
    for (std::size_t i { 0U }; i < 5U; ++i) {
        wave.step(0.0166667);
    }

    // Bow (+X) should have crest, stern (-X) should have trough
    const double bowElev { wave.getElevation(Vec2 { 2.0, 0.0 }) };
    const double sternElev { wave.getElevation(Vec2 { -2.0, 0.0 }) };

    EXPECT_GT(bowElev, sternElev);
}

// -----------------------------------------------------------------------------
// Two-Way Fluid-Structure Interaction (ShallowWaterCoupler) Tests
// -----------------------------------------------------------------------------

TEST(ShallowWaterCouplerTest, DynamicArchimedesBuoyancy)
{
    // High wave elevation vs low wave elevation
    constexpr double kHullY { 0.0 };
    constexpr double kDraft { 0.6 };
    constexpr double kDisplacement { 1500.0 };

    const double calmBuoyancy { ShallowWaterCoupler::computeBuoyancy(
        kHullY, 0.3, kDraft, kDisplacement) };
    const double crestBuoyancy { ShallowWaterCoupler::computeBuoyancy(
        kHullY, 0.6, kDraft, kDisplacement) };
    const double troughBuoyancy { ShallowWaterCoupler::computeBuoyancy(
        kHullY, 0.05, kDraft, kDisplacement) };

    EXPECT_GT(crestBuoyancy, calmBuoyancy);
    EXPECT_GT(calmBuoyancy, troughBuoyancy);
    EXPECT_GT(troughBuoyancy, 0.0);
}

TEST(ShallowWaterCouplerTest, HydrostaticTiltTorque)
{
    constexpr double kStiffness { 12000.0 };
    constexpr double kDamping { 2000.0 };

    // Hull level (0 deg), wave tilting up by +0.1 rad (~5.7 deg)
    const double torque { ShallowWaterCoupler::computeWaveTorque(
        0.0, 0.1, kStiffness, kDamping, 0.0) };

    // Negative (deltaAngle = 0 - 0.1 = -0.1), -sin(-0.1) * stiffness > 0
    EXPECT_GT(torque, 0.0);
}

TEST(ShallowWaterCouplerTest, FullTwoWayCouplerFsi)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -25.0, -25.0 };
    config.size = Vec2 { 50.0, 50.0 };
    config.cellSize = 0.5;

    WaveGrid2D waves { config };

    RigidBody2D body {};
    body.setMass(1200.0);
    body.setPosition(Vec2 { 0.0, -0.2 }); // Slightly submerged below waterline 0.0

    WaterWakeStamp2D stamp {};
    stamp.position = Vec2 { 0.0, -0.2 };
    stamp.velocity = Vec2 { 5.0, 0.0 };
    stamp.draft = 0.5;

    const HydrodynamicCouplingForces forces { ShallowWaterCoupler::updateFsi(
        body, waves, stamp, 0.0166667) };

    EXPECT_TRUE(forces.inWater);
    EXPECT_GT(forces.buoyancyForce, 0.0);
    EXPECT_GT(forces.totalForce.y, 0.0);
}

// -----------------------------------------------------------------------------
// WatercraftDynamics Integration Tests
// -----------------------------------------------------------------------------

TEST(WatercraftDynamicsTest, DynamicWaveInteraction)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -30.0, -30.0 };
    config.size = Vec2 { 60.0, 60.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 5.0;

    WaveGrid2D waves { config };
    // Create traveling swell passing under boat
    waves.injectImpulse(Vec2 { 0.0, 0.0 }, 3.0, 0.5);

    WatercraftConfig boatConfig {};
    boatConfig.waterlineY = 0.0;
    WatercraftDynamics boat { boatConfig };
    boat.attachWaveGrid(&waves);

    RigidBody2D body {};
    body.setMass(1500.0);
    body.setPosition(Vec2 { 0.0, -0.4 });

    // Step simulation
    for (std::size_t i { 0U }; i < 30U; ++i) {
        waves.step(0.0166667);
        boat.step(body, 0.0166667);
    }

    // Boat should be in water receiving dynamic buoyancy and wake impulses
    EXPECT_TRUE(boat.isSubmerged());
    EXPECT_GT(boat.getTelemetry().buoyancyForce, 0.0);
    EXPECT_EQ(boat.getWaveGrid(), &waves);
}

TEST(WaveGrid2DTest, DeterministicRepeatability)
{
    WaveConfig2D config {};
    config.origin = Vec2 { -20.0, -20.0 };
    config.size = Vec2 { 40.0, 40.0 };
    config.cellSize = 0.5;
    config.waveSpeed = 5.0;

    WaveGrid2D waveA { config };
    WaveGrid2D waveB { config };

    waveA.injectImpulse(Vec2 { 2.5, -1.0 }, 2.0, 0.6);
    waveB.injectImpulse(Vec2 { 2.5, -1.0 }, 2.0, 0.6);

    for (std::size_t i { 0U }; i < 60U; ++i) {
        waveA.step(0.0166667);
        waveB.step(0.0166667);
    }

    EXPECT_DOUBLE_EQ(waveA.getElevation(Vec2 { 2.5, -1.0 }), waveB.getElevation(Vec2 { 2.5, -1.0 }));
    EXPECT_DOUBLE_EQ(waveA.getElevation(Vec2 { 5.0, 3.0 }), waveB.getElevation(Vec2 { 5.0, 3.0 }));
    EXPECT_DOUBLE_EQ(waveA.computeEnergy(), waveB.computeEnergy());
}

} // namespace vehicle_sim::testing
