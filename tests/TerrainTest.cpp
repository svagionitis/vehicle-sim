#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/FractalNoise.h"
#include "libs/VehicleSimCore/terrain/GradientNoise.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

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

} // namespace vehicle_sim::testing
