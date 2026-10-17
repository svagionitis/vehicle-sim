#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/audio/AudioChannels.h"
#include "libs/VehicleSimCore/audio/AudioDspMath.h"

using namespace vehicle_sim::audio;

TEST(VehicleSimCoreAudioTest, PolyBlepSawAntiAliasing)
{
    constexpr double freq { 220.0 };
    constexpr double sampleRate { 44100.0 };
    constexpr double phaseInc { freq / sampleRate };

    double phase { 0.0 };
    for (int i { 0 }; i < 500; ++i) {
        const double saw { AudioDspMath::polyBlepSaw(phase, phaseInc) };
        EXPECT_GE(saw, -1.05);
        EXPECT_LE(saw, 1.05);

        phase += phaseInc;
        if (phase >= 1.0) {
            phase -= 1.0;
        }
    }
}

TEST(VehicleSimCoreAudioTest, BiquadFilterStability)
{
    BiquadFilter lp { };
    lp.setLowPass(44100.0, 1200.0, 1.414);

    double val { 0.0 };
    for (int i { 0 }; i < 100; ++i) {
        val = lp.process(1.0);
        EXPECT_FALSE(std::isnan(val));
        EXPECT_FALSE(std::isinf(val));
    }
    EXPECT_NEAR(val, 1.0, 0.05);
}

TEST(VehicleSimCoreAudioTest, EngineSynthesizerIdleAndRedline)
{
    EngineSynthesizer engine { };

    double idleEnergy { 0.0 };
    for (int i { 0 }; i < 200; ++i) {
        const double s { engine.process(800.0, 0.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        idleEnergy += (s * s);
    }
    EXPECT_GT(idleEnergy, 0.01);

    double redlineEnergy { 0.0 };
    for (int i { 0 }; i < 200; ++i) {
        const double s { engine.process(7200.0, 1.0, 44100.0) };
        EXPECT_GE(s, -1.0);
        EXPECT_LE(s, 1.0);
        redlineEnergy += (s * s);
    }
    EXPECT_GT(redlineEnergy, 0.01);
}

TEST(VehicleSimCoreAudioTest, TireSlipAndImpactSynthesizers)
{
    TireSlipSynthesizer tire { };
    EXPECT_DOUBLE_EQ(tire.process(0.0, 1.0, 44100.0), 0.0);

    double slipEnergy { 0.0 };
    for (int i { 0 }; i < 200; ++i) {
        const double s { tire.process(4.0, 1.0, 44100.0) };
        slipEnergy += (s * s);
    }
    EXPECT_GT(slipEnergy, 0.1);

    ImpactSynthesizer impact { };
    EXPECT_DOUBLE_EQ(impact.process(44100.0), 0.0);
    impact.trigger(500.0);
    const double out { impact.process(44100.0) };
    EXPECT_NE(out, 0.0);
}
