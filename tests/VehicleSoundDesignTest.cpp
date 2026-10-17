#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "libs/VehicleSimCore/audio/AudioChannels.h"
#include "libs/VehicleSimCore/audio/CylinderHarmonicSynth.h"
#include "libs/VehicleSimCore/audio/DrivetrainAcousticSynth.h"
#include "libs/VehicleSimCore/audio/ExhaustPopSynth.h"
#include "libs/VehicleSimCore/audio/ForcedInductionSynth.h"
#include "libs/VehicleSimCore/audio/VehicleAcousticProfile.h"

namespace vehicle_sim::audio {

TEST(VehicleSoundDesignTest, FleetProfilesAreValid)
{
    for (int i { 0 }; i < 12; ++i) {
        const auto& profile = AcousticProfileRegistry::getProfile(i);
        EXPECT_GT(profile.idleFrequency, 0.0);
        EXPECT_GT(profile.redlineFrequency, profile.idleFrequency);
        EXPECT_GE(profile.cylinderHarmonicWeight, 0.0);
        EXPECT_LE(profile.cylinderHarmonicWeight, 1.0);
        EXPECT_GE(profile.subHarmonicWeight, 0.0);
        EXPECT_LE(profile.subHarmonicWeight, 1.0);
        EXPECT_GE(profile.dieselKnockIntensity, 0.0);
        EXPECT_LE(profile.dieselKnockIntensity, 1.0);
        EXPECT_GT(profile.exhaustResonanceFreq1, 0.0);
        EXPECT_GT(profile.exhaustResonanceFreq2, profile.exhaustResonanceFreq1);
        EXPECT_GT(profile.superchargerGearRatio, 0.0);
    }
}

TEST(VehicleSoundDesignTest, CylinderHarmonicSynthConfigs)
{
    constexpr double kSampleRate { 44100.0 };
    CylinderHarmonicSynth synth { };

    const std::vector<EngineConfiguration> configs {
        EngineConfiguration::Inline4,
        EngineConfiguration::V6,
        EngineConfiguration::CrossplaneV8,
        EngineConfiguration::FlatplaneV8,
        EngineConfiguration::V10,
        EngineConfiguration::HeavyDieselV12,
        EngineConfiguration::HeavyDieselI6,
        EngineConfiguration::ElectricPWM,
        EngineConfiguration::SciFiWarp
    };

    for (const auto config : configs) {
        synth.reset();
        synth.configure(config, (config == EngineConfiguration::HeavyDieselV12) ? 0.45 : 0.0);

        bool hasNonZero { false };
        for (int i { 0 }; i < 512; ++i) {
            const double s = synth.process(4000.0, 0.8, kSampleRate);
            EXPECT_FALSE(std::isnan(s));
            EXPECT_FALSE(std::isinf(s));
            EXPECT_GE(s, -1.0);
            EXPECT_LE(s, 1.0);
            if (std::abs(s) > 0.01) {
                hasNonZero = true;
            }
        }
        EXPECT_TRUE(hasNonZero);
    }
}

TEST(VehicleSoundDesignTest, TurboSpoolAndBovTrigger)
{
    constexpr double kSampleRate { 44100.0 };
    ForcedInductionSynth turbo { };
    turbo.configure(ForcedInductionType::Turbocharger, 2.4, 5500.0);

    // Accelerate: spool up
    for (int i { 0 }; i < 22050; ++i) { // 0.5s at full throttle
        const double s = turbo.process(6000.0, 1.0, kSampleRate);
        EXPECT_FALSE(std::isnan(s));
        EXPECT_FALSE(std::isinf(s));
    }

    // Sudden throttle lift-off triggers BOV
    bool bovActive { false };
    for (int i { 0 }; i < 4410; ++i) { // 0.1s
        const double s = turbo.process(5500.0, 0.0, kSampleRate);
        if (std::abs(s) > 0.1) {
            bovActive = true;
        }
    }
    EXPECT_TRUE(bovActive);
}

TEST(VehicleSoundDesignTest, SuperchargerWhineProducesSound)
{
    constexpr double kSampleRate { 44100.0 };
    ForcedInductionSynth blower { };
    blower.configure(ForcedInductionType::Supercharger, 2.45, 5000.0);

    bool hasWhine { false };
    for (int i { 0 }; i < 512; ++i) {
        const double s = blower.process(5000.0, 0.85, kSampleRate);
        EXPECT_FALSE(std::isnan(s));
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        if (std::abs(s) > 0.01) {
            hasWhine = true;
        }
    }
    EXPECT_TRUE(hasWhine);
}

TEST(VehicleSoundDesignTest, ExhaustPopStochasticGeneration)
{
    constexpr double kSampleRate { 44100.0 };
    ExhaustPopSynth popSynth { };

    bool popped { false };
    // Simulate high RPM throttle lift over 1 second
    for (int i { 0 }; i < 44100; ++i) {
        const double s = popSynth.process(6500.0, 0.0, 0.9, kSampleRate);
        EXPECT_FALSE(std::isnan(s));
        if (std::abs(s) > 0.1) {
            popped = true;
        }
    }
    EXPECT_TRUE(popped);

    // Process with zero intensity -> should be silent
    popSynth.reset();
    for (int i { 0 }; i < 2000; ++i) {
        const double s = popSynth.process(6500.0, 0.0, 0.0, kSampleRate);
        EXPECT_DOUBLE_EQ(s, 0.0);
    }
}

TEST(VehicleSoundDesignTest, DrivetrainAcousticSynthesis)
{
    constexpr double kSampleRate { 44100.0 };
    DrivetrainAcousticSynth drivetrain { };

    // Gear whine
    const double whine = drivetrain.processGearWhine(35.0, kSampleRate);
    EXPECT_FALSE(std::isnan(whine));
    EXPECT_GE(whine, -1.0);
    EXPECT_LE(whine, 1.0);

    // Air brakes
    drivetrain.triggerAirBrakes(0.8);
    const double air = drivetrain.processAirBrakes(kSampleRate);
    EXPECT_FALSE(std::isnan(air));
    EXPECT_GT(std::abs(air), 0.001);

    // Track clatter
    const double clatter = drivetrain.processTrackClatter(10.0, kSampleRate);
    EXPECT_FALSE(std::isnan(clatter));

    // Electric PWM
    const double pwm = drivetrain.processElectricPwm(0.8, 0.7, kSampleRate);
    EXPECT_FALSE(std::isnan(pwm));
}

TEST(VehicleSoundDesignTest, EngineSynthesizerWithFleetProfiles)
{
    constexpr double kSampleRate { 44100.0 };
    EngineSynthesizer engine { };

    for (int i { 0 }; i < 12; ++i) {
        const auto& profile = AcousticProfileRegistry::getProfile(i);
        engine.reset();
        engine.configure(profile);

        for (int sample { 0 }; sample < 512; ++sample) {
            const double s = engine.process(4500.0, 0.75, kSampleRate);
            EXPECT_FALSE(std::isnan(s));
            EXPECT_FALSE(std::isinf(s));
            EXPECT_GE(s, -1.0);
            EXPECT_LE(s, 1.0);
        }
    }
}

TEST(VehicleSoundDesignTest, PerformanceBenchmark)
{
    constexpr double kSampleRate { 44100.0 };
    EngineSynthesizer engine { };
    engine.configure(AcousticProfileRegistry::getProfile(7)); // Drift car: Twin-Turbo V8 + Pops

    const auto start = std::chrono::high_resolution_clock::now();
    for (int i { 0 }; i < 512; ++i) {
        const double s = engine.process(5500.0, 0.8, kSampleRate);
        (void)s;
    }
    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::milli> elapsed = end - start;

    // Buffer duration is 512 / 44100 = 11.6 ms.
    // 512 samples should synthesize in well under 0.5 ms.
    EXPECT_LT(elapsed.count(), 1.0);
}

} // namespace vehicle_sim::audio
