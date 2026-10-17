#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "apps/MotionLab/AudioSystem.h"
#include "libs/VehicleSimCore/audio/AudioChannels.h"
#include "libs/VehicleSimCore/audio/AudioDspMath.h"

using namespace motion_lab;
using namespace vehicle_sim::audio;

TEST(AudioDspMathTest, PolyBlepResidualContinuity)
{
    constexpr double phaseInc { 440.0 / 44100.0 };

    // Outside boundary regions, residual must be 0
    EXPECT_DOUBLE_EQ(AudioDspMath::polyBlep(0.5, phaseInc), 0.0);
    EXPECT_DOUBLE_EQ(AudioDspMath::polyBlep(0.2, phaseInc), 0.0);

    // Near 0 phase boundary
    const double nearZero { AudioDspMath::polyBlep(phaseInc * 0.5, phaseInc) };
    EXPECT_LT(nearZero, 0.0);

    // Near 1 phase boundary
    const double nearOne { AudioDspMath::polyBlep(1.0 - (phaseInc * 0.5), phaseInc) };
    EXPECT_GT(nearOne, 0.0);
}

TEST(AudioDspMathTest, PolyBlepSawSampleBounds)
{
    constexpr double sampleRate { 44100.0 };
    constexpr double freq { 440.0 };
    constexpr double phaseInc { freq / sampleRate };

    double phase { 0.0 };
    for (int i { 0 }; i < 1000; ++i) {
        const double sample { AudioDspMath::polyBlepSaw(phase, phaseInc) };
        EXPECT_GE(sample, -1.05);
        EXPECT_LE(sample, 1.05);

        phase += phaseInc;
        if (phase >= 1.0) {
            phase -= 1.0;
        }
    }
}

TEST(AudioDspMathTest, Xorshift32UniformDistribution)
{
    std::uint32_t state { 0x13579BDFU };
    constexpr int kSampleCount { 10000 };

    double sum { 0.0 };
    double sumSq { 0.0 };

    for (int i { 0 }; i < kSampleCount; ++i) {
        const double sample { AudioDspMath::xorshift32(state) };
        EXPECT_GE(sample, -1.0);
        EXPECT_LE(sample, 1.0);
        sum += sample;
        sumSq += (sample * sample);
    }

    const double mean { sum / kSampleCount };
    const double variance { (sumSq / kSampleCount) - (mean * mean) };

    // Uniform [-1, 1] theoretical mean = 0.0, variance = 1/3 ~ 0.333
    EXPECT_NEAR(mean, 0.0, 0.05);
    EXPECT_NEAR(variance, 0.333, 0.05);
}

TEST(AudioDspMathTest, SoftLimitSaturationBounds)
{
    // Linear region
    EXPECT_NEAR(AudioDspMath::softLimit(0.0), 0.0, 1e-6);
    EXPECT_NEAR(AudioDspMath::softLimit(0.5), (1.5 * 0.5) - (0.5 * 0.125), 1e-6);

    // Extreme overload saturation
    EXPECT_DOUBLE_EQ(AudioDspMath::softLimit(1.0), 1.0);
    EXPECT_DOUBLE_EQ(AudioDspMath::softLimit(5.0), 1.0);
    EXPECT_DOUBLE_EQ(AudioDspMath::softLimit(-1.0), -1.0);
    EXPECT_DOUBLE_EQ(AudioDspMath::softLimit(-10.0), -1.0);
}

TEST(AudioDspMathTest, BiquadFilterLowPassAndBandPass)
{
    BiquadFilter filter { };
    filter.setLowPass(44100.0, 1000.0, 1.414);

    // DC input test (1.0 -> should settle close to 1.0)
    double out { 0.0 };
    for (int i { 0 }; i < 200; ++i) {
        out = filter.process(1.0);
        EXPECT_FALSE(std::isnan(out));
        EXPECT_FALSE(std::isinf(out));
    }
    EXPECT_NEAR(out, 1.0, 0.05);

    // Reset clears state
    filter.reset();
    out = filter.process(0.0);
    EXPECT_DOUBLE_EQ(out, 0.0);

    // Bandpass test
    filter.setBandPass(44100.0, 1800.0, 3.0);
    for (int i { 0 }; i < 50; ++i) {
        out = filter.process(0.5);
        EXPECT_FALSE(std::isnan(out));
    }
}

TEST(AudioChannelsTest, EngineSynthesizerExecution)
{
    EngineSynthesizer engine { };

    // At idle 800 RPM
    double maxSample { 0.0 };
    for (int i { 0 }; i < 500; ++i) {
        const double s { engine.process(800.0, 0.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        maxSample = std::max(maxSample, std::abs(s));
    }
    EXPECT_GT(maxSample, 0.1);

    // Under full throttle 6500 RPM
    maxSample = 0.0;
    for (int i { 0 }; i < 500; ++i) {
        const double s { engine.process(6500.0, 1.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        maxSample = std::max(maxSample, std::abs(s));
    }
    EXPECT_GT(maxSample, 0.1);
}

TEST(AudioChannelsTest, TireSlipSynthesizerExecution)
{
    TireSlipSynthesizer tire { };

    // At zero slip, should produce 0
    for (int i { 0 }; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(tire.process(0.0, 1.0, 44100.0), 0.0);
    }

    // At high slip, should produce noise
    double energy { 0.0 };
    for (int i { 0 }; i < 500; ++i) {
        const double s { tire.process(5.0, 1.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        energy += (s * s);
    }
    EXPECT_GT(energy, 1.0);
}

TEST(AudioChannelsTest, RotorSynthesizerExecution)
{
    RotorSynthesizer rotor { };

    // At 0 RPM, silent
    EXPECT_DOUBLE_EQ(rotor.process(0.0, 2, 0.5, 44100.0), 0.0);

    // At 450 RPM (helicopter), generates rotor chop
    double energy { 0.0 };
    for (int i { 0 }; i < 1000; ++i) {
        const double s { rotor.process(450.0, 2, 0.8, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        energy += (s * s);
    }
    EXPECT_GT(energy, 1.0);
}

TEST(AudioChannelsTest, ThrusterSynthesizerExecution)
{
    ThrusterSynthesizer thruster { };

    // Zero throttle, silent
    EXPECT_DOUBLE_EQ(thruster.process(0.0, 44100.0), 0.0);

    // Full throttle, generates pink noise roar
    double energy { 0.0 };
    for (int i { 0 }; i < 500; ++i) {
        const double s { thruster.process(1.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        energy += (s * s);
    }
    EXPECT_GT(energy, 1.0);
}

TEST(AudioChannelsTest, ImpactSynthesizerDecay)
{
    ImpactSynthesizer impact { };

    // Silent before trigger
    EXPECT_DOUBLE_EQ(impact.process(44100.0), 0.0);

    // Trigger impulse
    impact.trigger(1500.0);

    double initialPeak { 0.0 };
    for (int i { 0 }; i < 200; ++i) {
        const double s { impact.process(44100.0) };
        initialPeak = std::max(initialPeak, std::abs(s));
    }
    EXPECT_GT(initialPeak, 0.1);

    // After decay time (e.g., 0.5s = 22050 samples), should be silent
    for (int i { 0 }; i < 25000; ++i) {
        static_cast<void>(impact.process(44100.0));
    }
    EXPECT_DOUBLE_EQ(impact.process(44100.0), 0.0);
}

TEST(AudioSystemTest, HeadlessInitializationAndControls)
{
    AudioSystem audio { };
    EXPECT_FALSE(audio.isInitialized());

    EXPECT_TRUE(audio.initialize(true)); // headless = true
    EXPECT_TRUE(audio.isInitialized());
    EXPECT_TRUE(audio.isMuted());
    EXPECT_NEAR(audio.getVolume(), 0.80f, 1e-4f);

    // Mute toggle
    audio.toggleMute();
    EXPECT_FALSE(audio.isMuted());
    audio.toggleMute();
    EXPECT_TRUE(audio.isMuted());

    // Volume adjustment
    audio.adjustVolume(0.10f);
    EXPECT_NEAR(audio.getVolume(), 0.90f, 1e-4f);
    audio.adjustVolume(-0.50f);
    EXPECT_NEAR(audio.getVolume(), 0.40f, 1e-4f);

    audio.shutdown();
    EXPECT_FALSE(audio.isInitialized());
}

TEST(AudioSystemTest, HeadlessSynthesisAndSampleBounds)
{
    AudioSystem audio { };
    EXPECT_TRUE(audio.initialize(true));

    // Telemetry: Classic Car idling
    audio.updateTelemetry(MotionLabVehicleType::ClassicCar, 850.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    audio.setMuted(false);

    std::vector<std::int16_t> buffer(512, 0);
    audio.synthesizeHeadless(buffer.data(), buffer.size());

    // Check that samples are non-zero and within signed 16-bit bounds
    bool hasNonZero { false };
    for (const auto s : buffer) {
        EXPECT_GE(s, -32767);
        EXPECT_LE(s, 32767);
        if (s != 0) {
            hasNonZero = true;
        }
    }
    EXPECT_TRUE(hasNonZero);

    // Check RMS levels
    const auto levels = audio.getLevels();
    EXPECT_GT(levels.engineLevel, 0.01f);
    EXPECT_GT(levels.masterLevel, 0.01f);
    EXPECT_NEAR(levels.tireLevel, 0.0f, 0.001f);
    EXPECT_NEAR(levels.rotorLevel, 0.0f, 0.001f);
    EXPECT_NEAR(levels.thrusterLevel, 0.0f, 0.001f);

    // Muted synthesis produces pure silence
    audio.setMuted(true);
    audio.synthesizeHeadless(buffer.data(), buffer.size());
    for (const auto s : buffer) {
        EXPECT_EQ(s, 0);
    }
}

TEST(AudioSystemTest, FleetAcousticChannelRouting)
{
    AudioSystem audio { };
    EXPECT_TRUE(audio.initialize(true));
    audio.setMuted(false);
    std::vector<std::int16_t> buffer(512, 0);

    // Rocket test -> thruster active, engine inactive
    audio.updateTelemetry(MotionLabVehicleType::Rocket, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
    audio.synthesizeHeadless(buffer.data(), buffer.size());
    auto levels = audio.getLevels();
    EXPECT_GT(levels.thrusterLevel, 0.05f);
    EXPECT_NEAR(levels.engineLevel, 0.0f, 0.001f);

    // Drone test -> rotor active, engine inactive
    audio.updateTelemetry(MotionLabVehicleType::QuadcopterDrone, 0.0, 0.8, 0.0, 1.0, 4500.0, 0.0);
    audio.synthesizeHeadless(buffer.data(), buffer.size());
    levels = audio.getLevels();
    EXPECT_GT(levels.rotorLevel, 0.05f);
    EXPECT_NEAR(levels.engineLevel, 0.0f, 0.001f);

    // Drift Car scrubbing tires -> engine and tire slip both active
    audio.updateTelemetry(MotionLabVehicleType::DriftCar, 5500.0, 0.9, 6.2, 1.0, 0.0, 0.0);
    audio.synthesizeHeadless(buffer.data(), buffer.size());
    levels = audio.getLevels();
    EXPECT_GT(levels.engineLevel, 0.05f);
    EXPECT_GT(levels.tireLevel, 0.05f);
}
