#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/HovercraftDynamics.h"
#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/TrialsBikeDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/ElevationMap2D.h"
#include "libs/VehicleSimCore/terrain/FractalNoise.h"
#include "libs/VehicleSimCore/terrain/GradientNoise.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"
#include "libs/VehicleSimCore/terrain/TrackGenerator.h"

namespace vehicle_sim::testing {

TEST(SurfaceMaterialTest, DefaultConstructionIsTarmac)
{
    const terrain::SurfaceMaterial mat {};
    EXPECT_EQ(mat.type, terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(mat.friction, 1.0);
    EXPECT_DOUBLE_EQ(mat.rollingDrag, 1.0);
    EXPECT_DOUBLE_EQ(mat.restitution, 0.10);
    EXPECT_FALSE(mat.isLiquid);
}

TEST(SurfaceMaterialTest, PresetRetrievalAndValidation)
{
    const auto tarmac = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Tarmac);
    EXPECT_EQ(tarmac.type, terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(tarmac.friction, 1.0);
    EXPECT_DOUBLE_EQ(tarmac.rollingDrag, 1.0);
    EXPECT_FALSE(tarmac.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Tarmac), "Tarmac");

    const auto ice = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Ice);
    EXPECT_EQ(ice.type, terrain::MaterialType::Ice);
    EXPECT_LT(ice.friction, tarmac.friction);
    EXPECT_DOUBLE_EQ(ice.friction, 0.10);
    EXPECT_FALSE(ice.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Ice), "Ice");

    const auto mud = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Mud);
    EXPECT_EQ(mud.type, terrain::MaterialType::Mud);
    EXPECT_GT(mud.rollingDrag, tarmac.rollingDrag);
    EXPECT_TRUE(mud.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Mud), "Mud");

    const auto oilSlick = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::OilSlick);
    EXPECT_DOUBLE_EQ(oilSlick.friction, 0.05);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::OilSlick), "OilSlick");

    const auto water = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Water);
    EXPECT_TRUE(water.isLiquid);
    EXPECT_GT(water.rollingDrag, 3.0);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Water), "Water");
}

TEST(SurfaceMaterialTest, AllPresetsCovered)
{
    const std::array<terrain::MaterialType, 10> types { terrain::MaterialType::Tarmac, terrain::MaterialType::Concrete,
        terrain::MaterialType::Dirt, terrain::MaterialType::Gravel, terrain::MaterialType::Mud,
        terrain::MaterialType::Ice, terrain::MaterialType::Snow, terrain::MaterialType::OilSlick,
        terrain::MaterialType::Water, terrain::MaterialType::Grass };

    for (const auto type : types) {
        const auto mat = terrain::SurfaceMaterial::getPreset(type);
        EXPECT_EQ(mat.type, type);
        EXPECT_GT(mat.friction, 0.0);
        EXPECT_GT(mat.rollingDrag, 0.0);
        EXPECT_GE(mat.restitution, 0.0);
        EXPECT_NE(terrain::SurfaceMaterial::getName(type), "Unknown");
    }
}

TEST(GradientNoiseTest, DeterministicRepeatability)
{
    const terrain::GradientNoise noiseA { 1337U };
    const terrain::GradientNoise noiseB { 1337U };
    const terrain::GradientNoise noiseC { 9999U };

    for (double x { -5.0 }; x <= 5.0; x += 0.5) {
        const auto sampleA = noiseA.sample1DWithDeriv(x);
        const auto sampleB = noiseB.sample1DWithDeriv(x);
        EXPECT_DOUBLE_EQ(sampleA.value, sampleB.value);
        EXPECT_DOUBLE_EQ(sampleA.derivative, sampleB.derivative);

        const auto sample2DA = noiseA.sample2DWithDeriv(x, x * 0.5);
        const auto sample2DB = noiseB.sample2DWithDeriv(x, x * 0.5);
        EXPECT_DOUBLE_EQ(sample2DA.value, sample2DB.value);
        EXPECT_DOUBLE_EQ(sample2DA.gradient.x, sample2DB.gradient.x);
        EXPECT_DOUBLE_EQ(sample2DA.gradient.y, sample2DB.gradient.y);
    }

    // Different seeds produce different fields
    const auto valA = noiseA.sample1D(2.34);
    const auto valC = noiseC.sample1D(2.34);
    EXPECT_NE(valA, valC);
}

TEST(GradientNoiseTest, BoundsAndIntegerGridValues)
{
    const terrain::GradientNoise noise { 42U };

    // At exact integer points, Perlin gradient noise value is 0.0
    for (int i { -10 }; i <= 10; ++i) {
        const double x { static_cast<double>(i) };
        EXPECT_NEAR(noise.sample1D(x), 0.0, 1e-12);
        EXPECT_NEAR(noise.sample2D(x, x), 0.0, 1e-12);
    }

    // Between grid points, values remain bounded in [-1.0, 1.0]
    for (double x { -10.0 }; x <= 10.0; x += 0.1) {
        const double val1D { noise.sample1D(x) };
        EXPECT_GE(val1D, -1.05);
        EXPECT_LE(val1D, 1.05);

        const double val2D { noise.sample2D(x, -x * 0.7) };
        EXPECT_GE(val2D, -1.05);
        EXPECT_LE(val2D, 1.05);
    }
}

TEST(GradientNoiseTest, AnalyticalDerivativeAccuracy1D)
{
    const terrain::GradientNoise noise { 12345U };
    constexpr double eps { 1e-6 };

    for (double x { -4.75 }; x <= 4.75; x += 0.25) {
        const auto sample = noise.sample1DWithDeriv(x);
        const double valPlus = noise.sample1D(x + eps);
        const double valMinus = noise.sample1D(x - eps);
        const double finiteDiff = (valPlus - valMinus) / (2.0 * eps);

        EXPECT_NEAR(sample.derivative, finiteDiff, 1e-5);
    }
}

TEST(GradientNoiseTest, AnalyticalGradientAccuracy2D)
{
    const terrain::GradientNoise noise { 54321U };
    constexpr double eps { 1e-6 };

    for (double x { -3.2 }; x <= 3.2; x += 0.8) {
        for (double y { -3.2 }; y <= 3.2; y += 0.8) {
            const auto sample = noise.sample2DWithDeriv(x, y);

            const double finiteDiffX = (noise.sample2D(x + eps, y) - noise.sample2D(x - eps, y)) / (2.0 * eps);
            const double finiteDiffY = (noise.sample2D(x, y + eps) - noise.sample2D(x, y - eps)) / (2.0 * eps);

            EXPECT_NEAR(sample.gradient.x, finiteDiffX, 1e-5);
            EXPECT_NEAR(sample.gradient.y, finiteDiffY, 1e-5);
        }
    }
}

TEST(FractalNoiseTest, DefaultConfigurationAndGetters)
{
    const terrain::FractalNoise fractal {};
    const auto& cfg = fractal.getConfig();

    EXPECT_EQ(cfg.octaves, 4U);
    EXPECT_DOUBLE_EQ(cfg.frequency, 1.0);
    EXPECT_DOUBLE_EQ(cfg.amplitude, 1.0);
    EXPECT_DOUBLE_EQ(cfg.lacunarity, 2.0);
    EXPECT_DOUBLE_EQ(cfg.persistence, 0.5);
    EXPECT_EQ(cfg.seed, 0U);
    EXPECT_EQ(cfg.type, terrain::FractalType::Fbm);

    EXPECT_EQ(fractal.getOctaves(), 4U);
    EXPECT_DOUBLE_EQ(fractal.getFrequency(), 1.0);
    EXPECT_DOUBLE_EQ(fractal.getAmplitude(), 1.0);
    EXPECT_DOUBLE_EQ(fractal.getLacunarity(), 2.0);
    EXPECT_DOUBLE_EQ(fractal.getPersistence(), 0.5);
    EXPECT_EQ(fractal.getSeed(), 0U);
    EXPECT_EQ(fractal.getType(), terrain::FractalType::Fbm);

    // Theoretical maximum amplitude: 1.0 + 0.5 + 0.25 + 0.125 = 1.875
    EXPECT_DOUBLE_EQ(fractal.getMaxAmplitude(), 1.875);
}

TEST(FractalNoiseTest, DeterministicRepeatability)
{
    terrain::FractalConfig config {};
    config.seed = 777U;
    config.octaves = 5U;
    config.frequency = 0.5;
    config.amplitude = 2.0;

    const terrain::FractalNoise fractalA { config };
    const terrain::FractalNoise fractalB { config };

    for (double x { -10.0 }; x <= 10.0; x += 1.0) {
        const auto sampleA = fractalA.sample1DWithDeriv(x);
        const auto sampleB = fractalB.sample1DWithDeriv(x);
        EXPECT_DOUBLE_EQ(sampleA.value, sampleB.value);
        EXPECT_DOUBLE_EQ(sampleA.derivative, sampleB.derivative);

        const auto sample2DA = fractalA.sample2DWithDeriv(x, x * 0.2);
        const auto sample2DB = fractalB.sample2DWithDeriv(x, x * 0.2);
        EXPECT_DOUBLE_EQ(sample2DA.value, sample2DB.value);
        EXPECT_DOUBLE_EQ(sample2DA.gradient.x, sample2DB.gradient.x);
        EXPECT_DOUBLE_EQ(sample2DA.gradient.y, sample2DB.gradient.y);
    }
}

TEST(FractalNoiseTest, OctaveOctaveAccumulation)
{
    terrain::FractalConfig config {};
    config.seed = 2026U;
    config.frequency = 1.0;
    config.amplitude = 1.0;
    config.persistence = 0.5;

    config.octaves = 1U;
    const terrain::FractalNoise fractal1 { config };

    config.octaves = 4U;
    const terrain::FractalNoise fractal4 { config };

    // 1 octave max amplitude is 1.0, 4 octaves is 1.875
    EXPECT_DOUBLE_EQ(fractal1.getMaxAmplitude(), 1.0);
    EXPECT_DOUBLE_EQ(fractal4.getMaxAmplitude(), 1.875);

    // Multi-octave terrain differs from single-octave base noise
    bool differencesObserved { false };
    for (double x { 0.1 }; x < 2.0; x += 0.2) {
        if (std::abs(fractal1.sample1D(x) - fractal4.sample1D(x)) > 1e-4) {
            differencesObserved = true;
            break;
        }
    }
    EXPECT_TRUE(differencesObserved);
}

TEST(FractalNoiseTest, SynthesisVariantsFbmBillowRidged)
{
    terrain::FractalConfig config {};
    config.seed = 456U;
    config.octaves = 3U;

    config.type = terrain::FractalType::Fbm;
    const terrain::FractalNoise fbm { config };

    config.type = terrain::FractalType::Billow;
    const terrain::FractalNoise billow { config };

    config.type = terrain::FractalType::Ridged;
    const terrain::FractalNoise ridged { config };

    const double testX { 1.45 };
    const double valFbm { fbm.sample1D(testX) };
    const double valBillow { billow.sample1D(testX) };
    const double valRidged { ridged.sample1D(testX) };

    // Distinct topologies generated
    EXPECT_NE(valFbm, valBillow);
    EXPECT_NE(valFbm, valRidged);
    EXPECT_NE(valBillow, valRidged);
}

TEST(FractalNoiseTest, AnalyticalDerivativeAndSurfaceNormal)
{
    terrain::FractalConfig config {};
    config.seed = 987U;
    config.octaves = 4U;
    config.frequency = 0.25;
    config.amplitude = 3.0;

    const terrain::FractalNoise fractal { config };
    constexpr double eps { 1e-6 };

    for (double x { -5.0 }; x <= 5.0; x += 0.5) {
        const auto sample = fractal.sample1DWithDeriv(x);
        const double finiteDiff = (fractal.sample1D(x + eps) - fractal.sample1D(x - eps)) / (2.0 * eps);

        EXPECT_NEAR(sample.derivative, finiteDiff, 1e-5);

        // Surface normal and tangent geometry
        const math::Vec2 normal = fractal.computeNormal1D(x);
        const math::Vec2 tangent = fractal.computeTangent1D(x);
        const double slopeAngle = fractal.computeSlopeAngle1D(x);

        // Normal points upward
        EXPECT_GT(normal.y, 0.0);

        // Normal and tangent are unit vectors
        EXPECT_NEAR(normal.length(), 1.0, 1e-9);
        EXPECT_NEAR(tangent.length(), 1.0, 1e-9);

        // Normal and tangent are orthogonal: normal . tangent == 0
        EXPECT_NEAR(normal.dot(tangent), 0.0, 1e-9);

        // Slope angle corresponds to arctan(derivative)
        EXPECT_NEAR(slopeAngle, std::atan(sample.derivative), 1e-12);
    }
}

TEST(Heightfield1DTest, DefaultConfigurationAndBaseHeight)
{
    terrain::Heightfield1D hf {};

    EXPECT_DOUBLE_EQ(hf.getBaseHeight(), 0.0);
    EXPECT_FALSE(hf.isNoiseEnabled());
    EXPECT_EQ(hf.getSampleCount(), 0U);
    EXPECT_EQ(hf.getInterpolation(), terrain::HeightfieldInterp::Linear);
    EXPECT_EQ(hf.getBoundaryMode(), terrain::HeightfieldBoundary::Clamp);
    EXPECT_EQ(hf.getDefaultMaterial().type, terrain::MaterialType::Tarmac);
    EXPECT_EQ(hf.getMaterialZoneCount(), 0U);

    // Uniform base height
    hf.setBaseHeight(12.5);
    EXPECT_DOUBLE_EQ(hf.getBaseHeight(), 12.5);

    for (double x { -50.0 }; x <= 50.0; x += 10.0) {
        EXPECT_DOUBLE_EQ(hf.getHeight(x), 12.5);
        EXPECT_DOUBLE_EQ(hf.getSlope(x), 0.0);
        EXPECT_DOUBLE_EQ(hf.getSlopeAngle(x), 0.0);
        const math::Vec2 n = hf.getNormal(x);
        EXPECT_DOUBLE_EQ(n.x, 0.0);
        EXPECT_DOUBLE_EQ(n.y, 1.0);
        const math::Vec2 t = hf.getTangent(x);
        EXPECT_DOUBLE_EQ(t.x, 1.0);
        EXPECT_DOUBLE_EQ(t.y, 0.0);
    }
}

TEST(Heightfield1DTest, DiscreteSamplesLinearInterpolation)
{
    terrain::Heightfield1D hf {};

    const std::vector<terrain::HeightSample1D> samples { { 0.0, 0.0 }, { 10.0, 5.0 }, { 20.0, 5.0 }, { 30.0, 0.0 } };
    hf.setSamples(samples);
    EXPECT_EQ(hf.getSampleCount(), 4U);

    // Exact control nodes
    EXPECT_NEAR(hf.getHeight(0.0), 0.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(10.0), 5.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(20.0), 5.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(30.0), 0.0, 1e-12);

    // Segment 1: [0, 10], slope = 0.5
    EXPECT_NEAR(hf.getHeight(5.0), 2.5, 1e-12);
    EXPECT_NEAR(hf.getSlope(5.0), 0.5, 1e-12);

    // Segment 2: [10, 20], flat plateau, slope = 0.0
    EXPECT_NEAR(hf.getHeight(15.0), 5.0, 1e-12);
    EXPECT_NEAR(hf.getSlope(15.0), 0.0, 1e-12);

    // Segment 3: [20, 30], slope = -0.5
    EXPECT_NEAR(hf.getHeight(25.0), 2.5, 1e-12);
    EXPECT_NEAR(hf.getSlope(25.0), -0.5, 1e-12);
}

TEST(Heightfield1DTest, DiscreteSamplesCubicHermiteSmoothness)
{
    terrain::Heightfield1D hf {};
    hf.setInterpolation(terrain::HeightfieldInterp::CubicHermite);

    const std::vector<terrain::HeightSample1D> samples { { -20.0, 0.0 }, { -10.0, 4.0 }, { 0.0, 8.0 }, { 10.0, 4.0 },
        { 20.0, 0.0 } };
    hf.setSamples(samples);

    // Control nodes match exactly
    EXPECT_NEAR(hf.getHeight(-10.0), 4.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(0.0), 8.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(10.0), 4.0, 1e-12);

    // Analytical derivative matches finite differences
    constexpr double eps { 1e-6 };
    for (double x { -18.0 }; x <= 18.0; x += 1.5) {
        const double analyticSlope { hf.getSlope(x) };
        const double finiteDiff { (hf.getHeight(x + eps) - hf.getHeight(x - eps)) / (2.0 * eps) };
        EXPECT_NEAR(analyticSlope, finiteDiff, 1e-4);
    }
}

TEST(Heightfield1DTest, BoundaryHandlingModes)
{
    terrain::Heightfield1D hf {};
    hf.setSamples({ { 0.0, 2.0 }, { 10.0, 6.0 } }); // slope = 0.4

    // Mode 1: Clamp
    hf.setBoundaryMode(terrain::HeightfieldBoundary::Clamp);
    EXPECT_NEAR(hf.getHeight(-5.0), 2.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(15.0), 6.0, 1e-12);
    EXPECT_NEAR(hf.getSlope(-5.0), 0.0, 1e-12);
    EXPECT_NEAR(hf.getSlope(15.0), 0.0, 1e-12);

    // Mode 2: Zero
    hf.setBoundaryMode(terrain::HeightfieldBoundary::Zero);
    EXPECT_NEAR(hf.getHeight(-5.0), 0.0, 1e-12);
    EXPECT_NEAR(hf.getHeight(15.0), 0.0, 1e-12);

    // Mode 3: Extrapolate
    hf.setBoundaryMode(terrain::HeightfieldBoundary::Extrapolate);
    EXPECT_NEAR(hf.getHeight(-5.0), 0.0, 1e-12); // 2.0 + (-5 - 0)*0.4 = 0.0
    EXPECT_NEAR(hf.getHeight(15.0), 8.0, 1e-12); // 6.0 + (15 - 10)*0.4 = 8.0
    EXPECT_NEAR(hf.getSlope(-5.0), 0.4, 1e-12);
    EXPECT_NEAR(hf.getSlope(15.0), 0.4, 1e-12);
}

TEST(Heightfield1DTest, ProceduralNoiseLayerAndAnalyticalGradients)
{
    terrain::Heightfield1D hf {};
    hf.setBaseHeight(5.0);

    terrain::FractalConfig noiseConfig {};
    noiseConfig.seed = 333U;
    noiseConfig.octaves = 4U;
    noiseConfig.frequency = 0.1;
    noiseConfig.amplitude = 1.0;
    hf.setNoiseConfig(noiseConfig);
    hf.setNoiseTransform(2.5, 0.5, 10.0);
    hf.enableNoise(true);
    EXPECT_TRUE(hf.isNoiseEnabled());

    constexpr double eps { 1e-6 };
    for (double x { -20.0 }; x <= 20.0; x += 2.0) {
        const auto sample = hf.sampleSurface(x);
        const double finiteDiff { (hf.getHeight(x + eps) - hf.getHeight(x - eps)) / (2.0 * eps) };

        // Analytical derivative matches finite differences
        EXPECT_NEAR(sample.derivative, finiteDiff, 1e-4);

        // Normal and tangent unit length and orthogonality
        EXPECT_NEAR(sample.normal.length(), 1.0, 1e-9);
        EXPECT_NEAR(sample.tangent.length(), 1.0, 1e-9);
        EXPECT_NEAR(sample.normal.dot(sample.tangent), 0.0, 1e-9);
        EXPECT_GT(sample.normal.y, 0.0);
        EXPECT_GT(sample.tangent.x, 0.0);

        // Slope angle
        EXPECT_NEAR(sample.slopeAngle, std::atan(sample.derivative), 1e-12);
    }
}

TEST(Heightfield1DTest, MaterialZonesAndLookup)
{
    terrain::Heightfield1D hf {};
    hf.setDefaultMaterial(terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Tarmac));

    // Add material zones
    hf.addMaterialZone(10.0, 30.0, terrain::MaterialType::Dirt);
    hf.addMaterialZone(50.0, 70.0, terrain::MaterialType::Ice);
    hf.addMaterialZone(90.0, 110.0, terrain::MaterialType::Mud);
    EXPECT_EQ(hf.getMaterialZoneCount(), 3U);

    // Query points
    EXPECT_EQ(hf.getMaterial(0.0).type, terrain::MaterialType::Tarmac);
    EXPECT_EQ(hf.getMaterial(20.0).type, terrain::MaterialType::Dirt);
    EXPECT_EQ(hf.getMaterial(40.0).type, terrain::MaterialType::Tarmac);
    EXPECT_EQ(hf.getMaterial(60.0).type, terrain::MaterialType::Ice);
    EXPECT_EQ(hf.getMaterial(80.0).type, terrain::MaterialType::Tarmac);
    EXPECT_EQ(hf.getMaterial(100.0).type, terrain::MaterialType::Mud);
    EXPECT_EQ(hf.getMaterial(120.0).type, terrain::MaterialType::Tarmac);

    // Clear zones
    hf.clearMaterialZones();
    EXPECT_EQ(hf.getMaterialZoneCount(), 0U);
    EXPECT_EQ(hf.getMaterial(60.0).type, terrain::MaterialType::Tarmac);
}

TEST(Heightfield1DTest, PenetrationAndUndergroundQueries)
{
    terrain::Heightfield1D hf {};
    hf.setBaseHeight(10.0);

    // Above ground
    const math::Vec2 abovePoint { 5.0, 15.0 };
    EXPECT_DOUBLE_EQ(hf.getPenetration(abovePoint), -5.0);
    EXPECT_FALSE(hf.isUnderground(abovePoint));

    // Exactly on surface
    const math::Vec2 onPoint { 5.0, 10.0 };
    EXPECT_DOUBLE_EQ(hf.getPenetration(onPoint), 0.0);
    EXPECT_FALSE(hf.isUnderground(onPoint));

    // Underground
    const math::Vec2 belowPoint { 5.0, 7.5 };
    EXPECT_DOUBLE_EQ(hf.getPenetration(belowPoint), 2.5);
    EXPECT_TRUE(hf.isUnderground(belowPoint));
}

TEST(Heightfield1DTest, PlatformExtractionFlatPlateau)
{
    terrain::Heightfield1D hf {};

    // Build terrain:
    // Plateau A: x in [0, 20], y = 0
    // Ramp: x in [20, 30], y rises from 0 to 8
    // Plateau B: x in [30, 50], y = 8
    const std::vector<terrain::HeightSample1D> samples { { 0.0, 0.0 }, { 20.0, 0.0 }, { 30.0, 8.0 }, { 50.0, 8.0 } };
    hf.setSamples(samples);
    hf.addMaterialZone(0.0, 25.0, terrain::MaterialType::Tarmac);
    hf.addMaterialZone(25.0, 50.0, terrain::MaterialType::Dirt);

    terrain::PlatformExtractionConfig config {};
    config.minX = 0.0;
    config.maxX = 50.0;
    config.sampleStep = 0.5;
    config.maxSlope = 0.05;
    config.minWidth = 5.0;
    config.method = terrain::PlatformExtractionMethod::FlatPlateau;

    const auto platforms = hf.extractPlatforms(config);
    ASSERT_EQ(platforms.size(), 2U);

    // First plateau
    EXPECT_NEAR(platforms[0].y, 0.0, 1e-2);
    EXPECT_NEAR(platforms[0].minX, 0.0, 1e-1);
    EXPECT_NEAR(platforms[0].maxX, 20.0, 1e-1);
    EXPECT_DOUBLE_EQ(platforms[0].friction, 1.0); // Tarmac

    // Second plateau
    EXPECT_NEAR(platforms[1].y, 8.0, 1e-2);
    EXPECT_NEAR(platforms[1].minX, 30.0, config.sampleStep);
    EXPECT_NEAR(platforms[1].maxX, 50.0, 1e-1);
    EXPECT_DOUBLE_EQ(platforms[1].friction, 0.75); // Dirt
}

TEST(Heightfield1DTest, PlatformExtractionStepQuantization)
{
    terrain::Heightfield1D hf {};

    // Continuous linear slope: y = 0.1 * x from x = 0 to 60 (y from 0 to 6)
    hf.setSamples({ { 0.0, 0.0 }, { 60.0, 6.0 } });

    terrain::PlatformExtractionConfig config {};
    config.minX = 0.0;
    config.maxX = 60.0;
    config.sampleStep = 0.5;
    config.tierHeight = 2.0; // Tiers at 0.0, 2.0, 4.0, 6.0
    config.minWidth = 4.0;
    config.method = terrain::PlatformExtractionMethod::StepQuantization;

    const auto platforms = hf.extractPlatforms(config);
    EXPECT_GE(platforms.size(), 3U);

    for (const auto& plat : platforms) {
        EXPECT_GE(plat.maxX - plat.minX, config.minWidth);
        // Elevation must be an exact multiple of tierHeight
        const double remainder { std::fmod(std::abs(plat.y), config.tierHeight) };
        EXPECT_NEAR(remainder, 0.0, 1e-9);
    }
}

TEST(Heightfield1DTest, PopulateSideViewCarDynamicsIntegration)
{
    terrain::Heightfield1D hf {};

    // Ground platform at y = 0, x in [-50, 50]
    hf.setSamples({ { -50.0, 0.0 }, { 50.0, 0.0 } });
    hf.setDefaultMaterial(terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Concrete));

    terrain::PlatformExtractionConfig config {};
    config.minX = -50.0;
    config.maxX = 50.0;
    config.sampleStep = 1.0;
    config.minWidth = 10.0;

    dynamics::SideViewCarDynamics car {};
    hf.populateSideViewCar(car, config, true);

    ASSERT_EQ(car.getPlatformCount(), 1U);
    const auto& plat = car.getPlatforms().front();
    EXPECT_DOUBLE_EQ(plat.y, 0.0);
    EXPECT_NEAR(plat.minX, -50.0, 1e-1);
    EXPECT_NEAR(plat.maxX, 50.0, 1e-1);
    EXPECT_DOUBLE_EQ(plat.friction, 1.05); // Concrete

    // Simulate vehicle falling onto extracted platform
    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = math::Vec2 { 0.0, 2.0 }; // Start 2 meters in the air
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Vehicle lands and establishes contact
    EXPECT_TRUE(car.isGrounded());
    EXPECT_FALSE(car.isAirborne());
    EXPECT_NEAR(body.getPosition().y, car.getConfig().wheelRadius, 0.1);
}

TEST(SurfaceMap2DTest, DefaultConfigurationAndBaseHeight)
{
    const terrain::SurfaceMap2D map {};
    EXPECT_DOUBLE_EQ(map.getBaseHeight(), 0.0);
    EXPECT_FALSE(map.hasGrid());
    EXPECT_EQ(map.getMaterialZoneCount(), 0U);
    EXPECT_FALSE(map.isNoiseEnabled());
    EXPECT_EQ(map.getDefaultMaterial().type, terrain::MaterialType::Tarmac);

    terrain::SurfaceMap2D elevatedMap {};
    elevatedMap.setBaseHeight(15.0);
    EXPECT_DOUBLE_EQ(elevatedMap.getBaseHeight(), 15.0);

    const auto sample = elevatedMap.sampleSurface(3.5, -4.2);
    EXPECT_DOUBLE_EQ(sample.elevation, 15.0);
    EXPECT_DOUBLE_EQ(sample.gradient.x, 0.0);
    EXPECT_DOUBLE_EQ(sample.gradient.y, 0.0);
    EXPECT_DOUBLE_EQ(sample.slopeAngle, 0.0);
    EXPECT_DOUBLE_EQ(sample.downhill.x, 0.0);
    EXPECT_DOUBLE_EQ(sample.downhill.y, 0.0);
    EXPECT_DOUBLE_EQ(sample.normal.x, 0.0);
    EXPECT_DOUBLE_EQ(sample.normal.y, 0.0);
    EXPECT_DOUBLE_EQ(sample.normal.z, 1.0);
    EXPECT_EQ(sample.material.type, terrain::MaterialType::Tarmac);
}

TEST(SurfaceMap2DTest, DiscreteGridBilinearInterpolationAndGradients)
{
    terrain::SurfaceMap2D map {};

    // 3x3 regular grid spanning [0, 4] x [0, 4] with cell size (2, 2)
    // Linear inclined plane: h(x, y) = 0.5 * x + 0.25 * y
    // Row 0 (y = 0): x=0: 0.0,  x=2: 1.0,  x=4: 2.0
    // Row 1 (y = 2): x=0: 0.5,  x=2: 1.5,  x=4: 2.5
    // Row 2 (y = 4): x=0: 1.0,  x=2: 2.0,  x=4: 3.0
    const std::vector<double> gridData { 0.0, 1.0, 2.0, 0.5, 1.5, 2.5, 1.0, 2.0, 3.0 };

    map.setGrid(math::Vec2 { 0.0, 0.0 }, math::Vec2 { 2.0, 2.0 }, 3U, 3U, gridData);
    map.setInterpolation(terrain::ElevationInterp2D::Bilinear);

    EXPECT_TRUE(map.hasGrid());
    EXPECT_EQ(map.getGridWidth(), 3U);
    EXPECT_EQ(map.getGridHeight(), 3U);
    EXPECT_DOUBLE_EQ(map.getGridElevation(1U, 1U), 1.5);

    // Exact grid point queries
    EXPECT_DOUBLE_EQ(map.getHeight(0.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(map.getHeight(2.0, 2.0), 1.5);
    EXPECT_DOUBLE_EQ(map.getHeight(4.0, 4.0), 3.0);

    // Mid-cell bilinear query at (1.0, 1.0): h = 0.5 * 1 + 0.25 * 1 = 0.75
    const auto midSample = map.sampleSurface(1.0, 1.0);
    EXPECT_DOUBLE_EQ(midSample.elevation, 0.75);

    // Gradient dh/dx = 0.5, dh/dy = 0.25
    EXPECT_NEAR(midSample.gradient.x, 0.5, 1e-9);
    EXPECT_NEAR(midSample.gradient.y, 0.25, 1e-9);

    // Downhill vector must be anti-parallel to gradient
    const math::Vec2 expectedGrad { 0.5, 0.25 };
    const math::Vec2 expectedDownhill { -expectedGrad * (1.0 / expectedGrad.length()) };
    EXPECT_NEAR(midSample.downhill.x, expectedDownhill.x, 1e-9);
    EXPECT_NEAR(midSample.downhill.y, expectedDownhill.y, 1e-9);

    // Unit normal vector validation (nx^2 + ny^2 + nz^2 == 1, nz > 0)
    const double normLenSq { midSample.normal.x * midSample.normal.x + midSample.normal.y * midSample.normal.y
        + midSample.normal.z * midSample.normal.z };
    EXPECT_NEAR(normLenSq, 1.0, 1e-9);
    EXPECT_GT(midSample.normal.z, 0.0);
}

TEST(SurfaceMap2DTest, DiscreteGridBicubicSmoothnessAndBoundaryModes)
{
    terrain::SurfaceMap2D map {};

    // 4x4 grid
    const std::vector<double> elevations { 0.0, 1.0, 2.0, 1.0, 1.0, 3.0, 4.0, 2.0, 2.0, 4.0, 5.0, 3.0, 1.0, 2.0, 3.0,
        1.0 };

    map.setGrid(math::Vec2 { 0.0, 0.0 }, math::Vec2 { 1.0, 1.0 }, 4U, 4U, elevations);
    map.setInterpolation(terrain::ElevationInterp2D::Bicubic);

    // Grid nodes must match exact elevation
    EXPECT_NEAR(map.getHeight(1.0, 1.0), 3.0, 1e-6);
    EXPECT_NEAR(map.getHeight(2.0, 2.0), 5.0, 1e-6);

    // Bicubic C1 continuity: gradient evaluated on either side of boundary
    constexpr double eps { 1e-4 };
    const auto gradLeft = map.getGradient(1.5 - eps, 1.5);
    const auto gradRight = map.getGradient(1.5 + eps, 1.5);
    EXPECT_NEAR(gradLeft.x, gradRight.x, 1e-3);
    EXPECT_NEAR(gradLeft.y, gradRight.y, 1e-3);

    // Boundary modes
    map.setBoundaryMode(terrain::ElevationBoundary2D::Clamp);
    EXPECT_NEAR(map.getHeight(10.0, 10.0), map.getHeight(3.0, 3.0), 1e-9);

    map.setBoundaryMode(terrain::ElevationBoundary2D::Zero);
    EXPECT_DOUBLE_EQ(map.getHeight(10.0, 10.0), 0.0);

    map.setBoundaryMode(terrain::ElevationBoundary2D::Repeat);
    // (4.0, 4.0) wraps to (0.0, 0.0)
    EXPECT_NEAR(map.getHeight(4.0, 4.0), map.getHeight(0.0, 0.0), 1e-9);
}

TEST(SurfaceMap2DTest, ProceduralNoiseLayerAndCompositeElevation)
{
    terrain::SurfaceMap2D map {};
    map.setBaseHeight(5.0);

    terrain::FractalConfig noiseCfg {};
    noiseCfg.octaves = 3U;
    noiseCfg.frequency = 0.1;
    noiseCfg.amplitude = 1.0;
    noiseCfg.seed = 2026U;

    map.setNoiseConfig(noiseCfg);
    map.setNoiseTransform(2.5, math::Vec2 { 1.0, 1.0 }, math::Vec2 { 0.0, 0.0 });
    map.enableNoise(true);

    EXPECT_TRUE(map.isNoiseEnabled());

    const auto sample = map.sampleSurface(10.37, 15.62);
    // Base height + noise (amplitude bounded by maxAmplitude * verticalScale)
    EXPECT_NE(sample.elevation, 5.0);
    EXPECT_NEAR(sample.elevation, 5.0, 2.5 * map.getNoise().getMaxAmplitude() + 1e-5);

    // Normal vector must be normalized
    const double lenSq { sample.normal.x * sample.normal.x + sample.normal.y * sample.normal.y
        + sample.normal.z * sample.normal.z };
    EXPECT_NEAR(lenSq, 1.0, 1e-9);

    // Analytical gradient consistency against finite differences
    constexpr double h { 1e-5 };
    const double hxPlus { map.getHeight(10.37 + h, 15.62) };
    const double hxMinus { map.getHeight(10.37 - h, 15.62) };
    const double numGradX { (hxPlus - hxMinus) / (2.0 * h) };

    const double hyPlus { map.getHeight(10.37, 15.62 + h) };
    const double hyMinus { map.getHeight(10.37, 15.62 - h) };
    const double numGradY { (hyPlus - hyMinus) / (2.0 * h) };

    EXPECT_NEAR(sample.gradient.x, numGradX, 1e-3);
    EXPECT_NEAR(sample.gradient.y, numGradY, 1e-3);
}

TEST(SurfaceMap2DTest, MaterialZonesBoxAndCircleWithLiquid)
{
    terrain::SurfaceMap2D map {};

    // Default: Tarmac
    EXPECT_EQ(map.getMaterial(0.0, 0.0).type, terrain::MaterialType::Tarmac);
    EXPECT_FALSE(map.isLiquidAt(math::Vec2 { 0.0, 0.0 }));

    // Box zone: Dirt in [-10, 10] x [10, 30]
    map.addBoxZone(math::Vec2 { -10.0, 10.0 }, math::Vec2 { 10.0, 30.0 }, terrain::MaterialType::Dirt);

    // Circle zone: Mud pit at center (0, 0) with radius 5.0
    map.addCircleZone(math::Vec2 { 0.0, 0.0 }, 5.0, terrain::MaterialType::Mud);

    EXPECT_EQ(map.getMaterialZoneCount(), 2U);

    // Inside mud pit circle
    EXPECT_EQ(map.getMaterial(0.0, 0.0).type, terrain::MaterialType::Mud);
    EXPECT_TRUE(map.isLiquidAt(math::Vec2 { 0.0, 0.0 }));
    EXPECT_DOUBLE_EQ(map.getMaterial(3.0, 3.0).friction, 0.40);

    // Inside dirt box
    EXPECT_EQ(map.getMaterial(0.0, 20.0).type, terrain::MaterialType::Dirt);
    EXPECT_FALSE(map.isLiquidAt(math::Vec2 { 0.0, 20.0 }));
    EXPECT_DOUBLE_EQ(map.getMaterial(0.0, 20.0).friction, 0.75);

    // Outside all zones -> Tarmac
    EXPECT_EQ(map.getMaterial(50.0, 50.0).type, terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(map.getMaterial(50.0, 50.0).friction, 1.0);

    // Priority test: adding ice patch over center overrides mud
    map.addCircleZone(math::Vec2 { 0.0, 0.0 }, 2.0, terrain::MaterialType::Ice);
    EXPECT_EQ(map.getMaterial(0.0, 0.0).type, terrain::MaterialType::Ice);
    EXPECT_DOUBLE_EQ(map.getMaterial(0.0, 0.0).friction, 0.10);

    map.clearMaterialZones();
    EXPECT_EQ(map.getMaterialZoneCount(), 0U);
    EXPECT_EQ(map.getMaterial(0.0, 0.0).type, terrain::MaterialType::Tarmac);
}

TEST(SurfaceMap2DTest, DownhillSlopeForceAndLiquidDrag)
{
    terrain::SurfaceMap2D map {};

    // 2x2 grid with constant incline: dh/dx = 0.2, dh/dy = 0.0
    const std::vector<double> elev { 0.0, 0.2, 0.0, 0.2 };
    map.setGrid(math::Vec2 { 0.0, 0.0 }, math::Vec2 { 1.0, 1.0 }, 2U, 2U, elev);
    map.setInterpolation(terrain::ElevationInterp2D::Bilinear);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1000.0;
    bodyConfig.position = math::Vec2 { 0.5, 0.5 };
    physics::RigidBody2D body { bodyConfig };

    // Gravity force pushes downhill along -X
    const math::Vec2 slopeForce = map.computeSlopeForce(body, 9.81);
    EXPECT_LT(slopeForce.x, 0.0);
    EXPECT_NEAR(slopeForce.y, 0.0, 1e-9);

    map.applySlopeForce(body, 9.81);
    physics::Integrator2D::integrate(body, 0.1);
    EXPECT_LT(body.getLinearVelocity().x, 0.0);

    // Liquid drag test
    map.addCircleZone(math::Vec2 { 0.0, 0.0 }, 10.0, terrain::MaterialType::Water);
    body.setLinearVelocity(math::Vec2 { 10.0, 0.0 });

    const math::Vec2 dragForce = map.computeLiquidDrag(body, 1.0);
    EXPECT_LT(dragForce.x, 0.0); // Retarding
    EXPECT_NEAR(dragForce.y, 0.0, 1e-9);
}

TEST(SurfaceMap2DTest, ArcadeCarDynamicsIntegration)
{
    terrain::SurfaceMap2D map {};

    // Ice patch at front wheels (x in [1.5, 4.0]), dry tarmac elsewhere
    map.addBoxZone(math::Vec2 { 1.5, -2.0 }, math::Vec2 { 4.0, 2.0 }, terrain::MaterialType::Ice);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.position = math::Vec2 { 1.0, 0.0 }; // Car center at x = 1.0
    bodyConfig.orientation = 0.0; // Facing east (+X)
    physics::RigidBody2D body { bodyConfig };

    dynamics::ArcadeCarDynamics car {};

    map.applySurfaceToCar(car, body);
    car.step(body, 1.0 / 60.0);

    // Front wheels (index 0, 1 at x = 1.0 + 1.25 = 2.25) are inside ice patch
    EXPECT_NEAR(car.getWheelTelemetry(dynamics::ArcadeCarDynamics::kFrontLeft).surfaceGrip, 0.10, 1e-5);
    EXPECT_NEAR(car.getWheelTelemetry(dynamics::ArcadeCarDynamics::kFrontRight).surfaceGrip, 0.10, 1e-5);

    // Rear wheels (index 2, 3 at x = 1.0 - 1.25 = -0.25) are on default tarmac
    EXPECT_NEAR(car.getWheelTelemetry(dynamics::ArcadeCarDynamics::kRearLeft).surfaceGrip, 1.00, 1e-5);
    EXPECT_NEAR(car.getWheelTelemetry(dynamics::ArcadeCarDynamics::kRearRight).surfaceGrip, 1.00, 1e-5);
}

TEST(SurfaceMap2DTest, ElevationMap2DAliasEquivalence)
{
    terrain::ElevationMap2D elevationMap {};
    elevationMap.setBaseHeight(42.0);
    EXPECT_DOUBLE_EQ(elevationMap.getBaseHeight(), 42.0);
    EXPECT_DOUBLE_EQ(elevationMap.getHeight(0.0, 0.0), 42.0);
}

TEST(TrackGeneratorTest, TrialsCourseProfileAndFeatures)
{
    terrain::TrialsCourseConfig config {};
    config.startLength = 20.0;
    config.humpCount = 3;
    config.humpSpacing = 5.0;
    config.humpHeight = 1.0;
    config.stepLedgeCount = 2;
    config.stepLedgeHeight = 1.2;
    config.stepLedgeWidth = 4.0;
    config.climbLength = 15.0;
    config.climbSlope = 0.40;
    config.roughLength = 20.0;
    config.roughAmplitude = 0.4;
    config.finishLength = 15.0;
    config.seed = 4242U;

    const terrain::TrialsCourse course { terrain::TrackGenerator::generateTrialsCourse(config) };

    EXPECT_GT(course.totalLength, 80.0);
    EXPECT_GT(course.finishHeight, 5.0);
    EXPECT_EQ(course.startPosition.x, 10.0);
    EXPECT_EQ(course.startPosition.y, 0.0);
    EXPECT_DOUBLE_EQ(course.finishPosition.y, course.finishHeight);

    // Heightfield surface queries
    EXPECT_DOUBLE_EQ(course.terrain.getHeight(0.0), 0.0);
    EXPECT_DOUBLE_EQ(course.terrain.getHeight(10.0), 0.0);
    EXPECT_GT(course.terrain.getHeight(course.totalLength), 5.0);

    // Verify material presets across zones
    const auto startSample { course.terrain.sampleSurface(5.0) };
    EXPECT_EQ(startSample.material.type, terrain::MaterialType::Concrete);

    const auto humpSample { course.terrain.sampleSurface(25.0) };
    EXPECT_EQ(humpSample.material.type, terrain::MaterialType::Dirt);

    // Test SideViewCarDynamics integration
    dynamics::SideViewCarDynamics car {};
    terrain::PlatformExtractionConfig platConfig {};
    platConfig.minX = 0.0;
    platConfig.maxX = course.totalLength;
    platConfig.minWidth = 3.0;

    course.populateSideViewCar(car, platConfig, true);
    EXPECT_GE(car.getPlatformCount(), 3U);

    // Deterministic repeatability
    const terrain::TrialsCourse course2 { terrain::TrackGenerator::generateTrialsCourse(config) };
    EXPECT_DOUBLE_EQ(course.totalLength, course2.totalLength);
    EXPECT_DOUBLE_EQ(course.finishHeight, course2.finishHeight);
    EXPECT_DOUBLE_EQ(course.terrain.getHeight(40.0), course2.terrain.getHeight(40.0));
}

TEST(TrackGeneratorTest, PlatformerLevelTiersAndHazards)
{
    terrain::PlatformerLevelConfig config {};
    config.levelWidth = 100.0;
    config.tierHeight = 4.0;
    config.tierCount = 3;
    config.platformMinLength = 8.0;
    config.platformMaxLength = 14.0;
    config.hasGroundFloor = true;
    config.hazardProbability = 0.50;
    config.seed = 777U;

    const terrain::PlatformerLevel level { terrain::TrackGenerator::generatePlatformerLevel(config) };

    EXPECT_EQ(level.tierCount, 3U);
    EXPECT_FALSE(level.platforms.empty());
    EXPECT_EQ(level.spawnPoint.x, -45.0);
    EXPECT_EQ(level.spawnPoint.y, 0.0);
    EXPECT_EQ(level.goalPoint.y, 12.0);

    // Check ground platform existence
    const auto& groundPlat = level.platforms.front();
    EXPECT_DOUBLE_EQ(groundPlat.y, 0.0);
    EXPECT_DOUBLE_EQ(groundPlat.minX, -50.0);
    EXPECT_DOUBLE_EQ(groundPlat.maxX, 50.0);
    EXPECT_FALSE(groundPlat.oneWay);

    // Check floating platforms
    bool foundOneWay { false };
    bool foundHazard { false };
    for (std::size_t i { 1 }; i < level.platforms.size(); ++i) {
        const auto& plat = level.platforms[i];
        EXPECT_GT(plat.y, 0.0);
        EXPECT_LE(plat.maxX - plat.minX, config.platformMaxLength + 1e-6);
        if (plat.oneWay) {
            foundOneWay = true;
        }
        if (plat.friction < 0.5) {
            foundHazard = true;
        }
    }
    EXPECT_TRUE(foundOneWay);
    EXPECT_TRUE(foundHazard);

    // Test SideViewCarDynamics integration
    dynamics::SideViewCarDynamics car {};
    level.populateSideViewCar(car, true);
    EXPECT_EQ(car.getPlatformCount(), level.platforms.size());

    // Deterministic repeatability
    const terrain::PlatformerLevel level2 { terrain::TrackGenerator::generatePlatformerLevel(config) };
    EXPECT_EQ(level.platforms.size(), level2.platforms.size());
    EXPECT_DOUBLE_EQ(level.platforms[1].minX, level2.platforms[1].minX);
}

TEST(TrackGeneratorTest, HillClimbElevationGainAndSteepPitches)
{
    terrain::HillClimbConfig config {};
    config.totalLength = 200.0;
    config.baseSlope = 0.15;
    config.steepPitchCount = 2;
    config.steepSlope = 0.50;
    config.noiseAmplitude = 3.0;
    config.seed = 5555U;

    const terrain::HillClimbCourse hill { terrain::TrackGenerator::generateHillClimb(config) };

    EXPECT_GT(hill.totalElevationGain, 25.0);
    EXPECT_EQ(hill.startPosition.x, 0.0);
    EXPECT_EQ(hill.peakPosition.x, 200.0);
    EXPECT_GT(hill.peakPosition.y, hill.startPosition.y);
    EXPECT_GT(hill.maxSlopeAngle, std::atan(config.baseSlope));

    // Verify material zones
    const auto baseSample { hill.terrain.sampleSurface(10.0) };
    EXPECT_EQ(baseSample.material.type, terrain::MaterialType::Dirt);

    // Populate car platforms
    dynamics::SideViewCarDynamics car {};
    terrain::PlatformExtractionConfig platConfig {};
    platConfig.minX = 0.0;
    platConfig.maxX = config.totalLength;
    platConfig.sampleStep = 0.5;
    platConfig.maxSlope = 0.10;
    platConfig.minWidth = 2.0;

    hill.populateSideViewCar(car, platConfig, true);
    EXPECT_GT(car.getPlatformCount(), 0U);

    // Deterministic repeatability
    const terrain::HillClimbCourse hill2 { terrain::TrackGenerator::generateHillClimb(config) };
    EXPECT_DOUBLE_EQ(hill.totalElevationGain, hill2.totalElevationGain);
    EXPECT_DOUBLE_EQ(hill.maxSlopeAngle, hill2.maxSlopeAngle);
    EXPECT_DOUBLE_EQ(hill.terrain.getHeight(100.0), hill2.terrain.getHeight(100.0));
}

TEST(TrackGeneratorTest, CircuitTrackCenterlineAndRibbonQueries)
{
    terrain::CircuitTrackConfig config {};
    config.radiusX = 80.0;
    config.radiusY = 50.0;
    config.waypointCount = 16;
    config.trackWidth = 8.0;
    config.shoulderWidth = 3.0;
    config.perturbation = 0.20;
    config.elevationVariation = 4.0;
    config.seed = 8888U;

    const terrain::CircuitTrack2D track { terrain::TrackGenerator::generateCircuitTrack(config) };

    EXPECT_TRUE(track.isClosed);
    EXPECT_DOUBLE_EQ(track.trackWidth, 8.0);
    EXPECT_DOUBLE_EQ(track.shoulderWidth, 3.0);
    EXPECT_GT(track.trackLength, 300.0);
    EXPECT_GE(track.centerline.size(), 64U);
    EXPECT_EQ(track.leftBoundary.size(), track.centerline.size());
    EXPECT_EQ(track.rightBoundary.size(), track.centerline.size());
    EXPECT_FALSE(track.checkpoints.empty());

    // Boundary width invariant: left and right boundaries must be ~trackWidth apart
    for (std::size_t i { 0 }; i < track.centerline.size(); ++i) {
        const double boundarySpan { (track.leftBoundary[i] - track.rightBoundary[i]).length() };
        EXPECT_NEAR(boundarySpan, track.trackWidth, 1e-4);
    }

    // Starting grid and heading
    EXPECT_DOUBLE_EQ(track.startGridPosition.x, track.centerline[0].x);
    EXPECT_DOUBLE_EQ(track.startGridPosition.y, track.centerline[0].y);

    // Ribbon distance and track membership queries
    EXPECT_TRUE(track.isPointOnTrack(track.centerline[0]));
    EXPECT_NEAR(track.getDistanceToCenter(track.centerline[0]), 0.0, 1e-5);
    EXPECT_NEAR(track.getTrackProgress(track.centerline[0]), 0.0, 1e-3);

    // Point far away at world origin (0, 0) is well inside the inner loop, outside the ribbon
    EXPECT_FALSE(track.isPointOnTrack(math::Vec2 { 0.0, 0.0 }));
    EXPECT_GT(track.getDistanceToCenter(math::Vec2 { 0.0, 0.0 }), 30.0);

    // Surface map elevation and bicubic interpolation
    EXPECT_TRUE(track.surfaceMap.hasGrid());
    const double elevAtStart { track.surfaceMap.getHeight(track.startGridPosition.x, track.startGridPosition.y) };
    EXPECT_TRUE(std::isfinite(elevAtStart));

    // Deterministic repeatability
    const terrain::CircuitTrack2D track2 { terrain::TrackGenerator::generateCircuitTrack(config) };
    EXPECT_DOUBLE_EQ(track.trackLength, track2.trackLength);
    EXPECT_DOUBLE_EQ(track.startGridPosition.x, track2.startGridPosition.x);
    EXPECT_DOUBLE_EQ(track.startHeading, track2.startHeading);
    EXPECT_DOUBLE_EQ(track.centerline[5].x, track2.centerline[5].x);
}

TEST(Heightfield1DTest, RaycastHitsAndMisses)
{
    terrain::Heightfield1D hf {};
    hf.setBaseHeight(0.0);
    hf.setDefaultMaterial(terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Concrete));
    hf.addMaterialZone(15.0, 30.0, terrain::MaterialType::Mud);

    // Ramp from x in [10, 20], rising from 0 to 5
    hf.setSamples({ { 10.0, 0.0 }, { 20.0, 5.0 } });

    // 1. Vertical downward hit onto base height at x = 0 (elevation = 0.0)
    const math::Vec2 originA { 0.0, 10.0 };
    const math::Vec2 dirDown { 0.0, -1.0 };
    const auto hitA = hf.raycast(originA, dirDown, 20.0);
    EXPECT_TRUE(hitA.hit);
    EXPECT_DOUBLE_EQ(hitA.distance, 10.0);
    EXPECT_DOUBLE_EQ(hitA.fraction, 10.0 / 20.0);
    EXPECT_DOUBLE_EQ(hitA.point.x, 0.0);
    EXPECT_DOUBLE_EQ(hitA.point.y, 0.0);
    EXPECT_NEAR(hitA.normal.x, 0.0, 1e-9);
    EXPECT_NEAR(hitA.normal.y, 1.0, 1e-9);
    EXPECT_EQ(hitA.material.type, terrain::MaterialType::Concrete);

    // 2. Vertical raycast pointing up away from terrain
    const math::Vec2 dirUp { 0.0, 1.0 };
    const auto hitUp = hf.raycast(originA, dirUp, 20.0);
    EXPECT_FALSE(hitUp.hit);

    // 3. Short max distance misses ground
    const auto hitShort = hf.raycast(originA, dirDown, 5.0);
    EXPECT_FALSE(hitShort.hit);

    // 4. Origin already underground
    const math::Vec2 originUnder { 0.0, -2.0 };
    const auto hitUnder = hf.raycast(originUnder, dirDown, 10.0);
    EXPECT_TRUE(hitUnder.hit);
    EXPECT_DOUBLE_EQ(hitUnder.distance, 0.0);
    EXPECT_DOUBLE_EQ(hitUnder.fraction, 0.0);

    // 5. Angled raycast hitting mud ramp at x in [15, 20]
    const math::Vec2 originRamp { 17.0, 10.0 };
    const math::Vec2 dirAngled { 0.0, -1.0 };
    const auto hitRamp = hf.raycast(originRamp, dirAngled, 30.0);
    EXPECT_TRUE(hitRamp.hit);
    EXPECT_GT(hitRamp.distance, 0.0);
    EXPECT_NEAR(hitRamp.point.y, hf.getHeight(hitRamp.point.x), 1e-4);
    EXPECT_EQ(hitRamp.material.type, terrain::MaterialType::Mud);

    // 6. Zero distance or zero direction vector
    EXPECT_FALSE(hf.raycast(originA, dirDown, 0.0).hit);
    EXPECT_FALSE(hf.raycast(originA, math::Vec2 { 0.0, 0.0 }, 10.0).hit);
}

TEST(TerrainGeometricInvariantsTest, NormalAndTangentUnitLengthInvariants)
{
    // 1D Heightfield normal & tangent unit length and orthogonality
    terrain::Heightfield1D hf {};
    hf.enableNoise(true);
    terrain::FractalConfig config {};
    config.octaves = 4U;
    config.amplitude = 12.0;
    config.frequency = 0.05;
    hf.setNoiseConfig(config);

    for (double x { -40.0 }; x <= 40.0; x += 1.0) {
        const auto sample = hf.sampleSurface(x);
        const double normalLen = sample.normal.length();
        const double tangentLen = sample.tangent.length();
        const double dotProd = sample.normal.dot(sample.tangent);

        EXPECT_NEAR(normalLen, 1.0, 1e-9);
        EXPECT_NEAR(tangentLen, 1.0, 1e-9);
        EXPECT_NEAR(dotProd, 0.0, 1e-9);
        EXPECT_GT(sample.normal.y, 0.0); // Upward normal invariant
    }

    // 2D SurfaceMap normal unit length invariant
    terrain::SurfaceMap2D map {};
    map.enableNoise(true);
    map.setNoiseConfig(config);

    for (double x { -20.0 }; x <= 20.0; x += 2.0) {
        for (double y { -20.0 }; y <= 20.0; y += 2.0) {
            const auto sample2D = map.sampleSurface(x, y);
            const double normLen3D = std::sqrt(sample2D.normal.x * sample2D.normal.x
                + sample2D.normal.y * sample2D.normal.y + sample2D.normal.z * sample2D.normal.z);
            EXPECT_NEAR(normLen3D, 1.0, 1e-9);
            EXPECT_GT(sample2D.normal.z, 0.0); // Upward normal invariant
        }
    }
}

TEST(TerrainIntegrationTest, TrialsBikeTerrainClearanceAndWheelSuspensions)
{
    terrain::Heightfield1D hf {};
    hf.setBaseHeight(0.0);

    dynamics::TrialsBikeConfig bikeConfig {};
    dynamics::TrialsBikeDynamics bike { bikeConfig };

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = bikeConfig.mass + bikeConfig.riderMass;
    bodyConfig.inertia = 65.0;
    bodyConfig.position = math::Vec2 { 0.0, 0.65 }; // Axles at 0.65 - 0.35 = 0.30 (sag 0.05)
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    for (std::int32_t step { 0 }; step < 30; ++step) {
        const math::Vec2 pos = body.getPosition();
        const math::Transform2D xf { pos, body.getOrientation() };

        const math::Vec2 frontAxle = xf.transformPoint(bikeConfig.frontWheelOffset);
        const math::Vec2 rearAxle = xf.transformPoint(bikeConfig.rearWheelOffset);

        const double frontClearance = frontAxle.y - hf.getHeight(frontAxle.x);
        const double rearClearance = rearAxle.y - hf.getHeight(rearAxle.x);

        dynamics::TrialsBikeInput input {};
        input.throttle = 1.0;
        input.groundDistanceFront = frontClearance;
        input.groundDistanceRear = rearClearance;

        bike.setInput(input);
        bike.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    const auto& telemetry = bike.getTelemetry();
    EXPECT_GT(telemetry.forwardSpeed, 0.0);
    EXPECT_GT(body.getPosition().x, 0.0);
    EXPECT_FALSE(telemetry.isAirborne);
}

TEST(TerrainIntegrationTest, SideViewCarDrivingAlongGeneratedTerrain)
{
    terrain::PlatformerLevelConfig config {};
    config.levelWidth = 100.0;
    config.hasGroundFloor = true;
    const terrain::PlatformerLevel level { terrain::TrackGenerator::generatePlatformerLevel(config) };

    dynamics::SideViewCarConfig carConfig {};
    dynamics::SideViewCarDynamics car { carConfig };
    level.populateSideViewCar(car, true);
    EXPECT_GT(car.getPlatformCount(), 0U);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1500.0;
    bodyConfig.position = math::Vec2 { level.spawnPoint.x, 0.35 };
    physics::RigidBody2D body { bodyConfig };

    dynamics::SideViewCarInput input {};
    input.throttle = 1.0;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (std::int32_t step { 0 }; step < 60; ++step) {
        car.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(body.getPosition().x, level.spawnPoint.x);
    EXPECT_GT(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getPosition().y, 0.35, 0.05);
    EXPECT_TRUE(car.isGrounded());
    EXPECT_GT(car.getSpeed(), 0.0);
}

TEST(TerrainIntegrationTest, HovercraftSurfaceMapInteraction)
{
    terrain::SurfaceMap2D map {};
    map.addBoxZone(math::Vec2 { 0.0, -10.0 }, math::Vec2 { 50.0, 10.0 }, terrain::MaterialType::Ice);

    dynamics::HovercraftConfig hoverConfig {};
    dynamics::HovercraftDynamics hovercraft { hoverConfig };

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1200.0;
    bodyConfig.inertia = 2800.0;
    bodyConfig.position = math::Vec2 { -10.0, 0.0 };
    physics::RigidBody2D body { bodyConfig };

    // Query surface at initial position (tarmac)
    const auto tarmacSample = map.sampleSurface(body.getPosition().x, body.getPosition().y);
    hovercraft.setSurfaceFriction(tarmacSample.material.friction);
    EXPECT_DOUBLE_EQ(hovercraft.getConfig().surfaceFriction, 1.0);

    // Query surface at ice zone
    const auto iceSample = map.sampleSurface(15.0, 0.0);
    hovercraft.setSurfaceFriction(iceSample.material.friction);
    EXPECT_DOUBLE_EQ(hovercraft.getConfig().surfaceFriction, 0.10);

    // Advance hovercraft with throttle
    dynamics::HovercraftInput input {};
    input.throttle = 1.0;
    input.liftFanActive = true;
    input.groundDistance = hoverConfig.hoverHeight;
    hovercraft.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (std::int32_t step { 0 }; step < 30; ++step) {
        hovercraft.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(hovercraft.getTelemetry().forwardSpeed, 0.0);
}

} // namespace vehicle_sim::testing
