#include <gtest/gtest.h>

#include "libs/VehicleSimCore/terrain/FractalNoise.h"
#include "libs/VehicleSimCore/terrain/GradientNoise.h"
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

} // namespace vehicle_sim::testing
