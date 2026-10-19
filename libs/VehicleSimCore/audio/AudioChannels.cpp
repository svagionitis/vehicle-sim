#include "libs/VehicleSimCore/audio/AudioChannels.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vehicle_sim::audio {

// --- EngineSynthesizer ---

void EngineSynthesizer::configure(const VehicleAcousticProfile& profile) noexcept
{
    profile_ = profile;
    configured_ = true;
    cylinderSynth_.configure(profile.engineConfig, profile.dieselKnockIntensity);
    inductionSynth_.configure(profile.inductionType, profile.superchargerGearRatio, profile.turboWhistleMaxFreq);
}

double EngineSynthesizer::process(double rpm, double throttle, double sampleRate) noexcept
{
    if (sampleRate <= 0.0) {
        return 0.0;
    }

    if (configured_) {
        const double combustion { cylinderSynth_.process(rpm, throttle, sampleRate) };
        const double induction { inductionSynth_.process(rpm, throttle, sampleRate) };
        lastInduction_ = induction;
        const double pops {
            profile_.hasOverrunPops ? popSynth_.process(rpm, throttle, profile_.overrunPopIntensity, sampleRate) : 0.0
        };

        const double raw { (0.68 * combustion) + (0.45 * induction) + (0.50 * pops) };
        return std::clamp(raw, -1.0, 1.0);
    }

    const double baseFreq { std::clamp(45.0 + (rpm * 0.055), 45.0, 700.0) };
    const double phaseInc { baseFreq / sampleRate };

    phase_ += phaseInc;
    if (phase_ >= 1.0) {
        phase_ -= 1.0;
    }

    subPhase_ += (phaseInc * 0.5);
    if (subPhase_ >= 1.0) {
        subPhase_ -= 1.0;
    }

    const double saw { AudioDspMath::polyBlepSaw(phase_, phaseInc) };
    const double subSaw { AudioDspMath::polyBlepSaw(subPhase_, phaseInc * 0.5) };
    const double raw { (0.75 * saw) + (0.25 * subSaw) };

    const double clampedThrottle { std::clamp(throttle, 0.0, 1.0) };
    const double cutoff { 250.0 + (3800.0 * clampedThrottle * clampedThrottle) };
    if (std::abs(cutoff - lastCutoff_) > 10.0) {
        filter_.setLowPass(sampleRate, cutoff, 1.414);
        lastCutoff_ = cutoff;
    }

    double out { filter_.process(raw) };

    // Rev-limiter spark cut emulation at high RPM
    if (rpm >= 7000.0) {
        const double stut { std::sin(AudioDspMath::kTwoPi * 22.0 * phase_) };
        if (stut < 0.0) {
            out *= 0.15;
        }
    }

    return std::clamp(out, -1.0, 1.0);
}

void EngineSynthesizer::reset() noexcept
{
    phase_ = 0.0;
    subPhase_ = 0.0;
    lastCutoff_ = 0.0;
    lastInduction_ = 0.0;
    filter_.reset();
    cylinderSynth_.reset();
    inductionSynth_.reset();
    popSynth_.reset();
}

// --- TireSlipSynthesizer ---

TireSlipSynthesizer::TireSlipSynthesizer() noexcept
{
    bandpass_.setBandPass(44100.0, 1800.0, 3.2);
    gravelFilter_.setBandPass(44100.0, 480.0, 1.5);
    lastCenterFreq_ = 1800.0;
}

double TireSlipSynthesizer::process(double slipVelocity, double surfaceMu, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || slipVelocity < 0.05 || surfaceMu < 0.02) {
        return 0.0;
    }

    const double centerFreq { std::clamp(1500.0 + (surfaceMu * 400.0) + (slipVelocity * 40.0), 1200.0, 3000.0) };
    if (std::abs(centerFreq - lastCenterFreq_) > 15.0) {
        bandpass_.setBandPass(sampleRate, centerFreq, 3.2);
        lastCenterFreq_ = centerFreq;
    }

    const double noise { AudioDspMath::xorshift32(rngState_) };
    const double filtered { bandpass_.process(noise) };

    const double normalizedSlip { std::clamp(slipVelocity / 5.5, 0.0, 1.0) };
    const double amp { std::pow(normalizedSlip, 1.25) * std::clamp(surfaceMu, 0.0, 1.0) };

    double out { filtered * amp * 1.8 };
    if (surfaceMu < 0.70) {
        // Coarse gravel/dirt crunch texture
        const double gravelNoise { AudioDspMath::xorshift32(rngState_) };
        const double gravelFiltered { gravelFilter_.process(gravelNoise) };
        out += (gravelFiltered * normalizedSlip * (1.0 - surfaceMu) * 1.2);
    }

    return std::clamp(out, -1.0, 1.0);
}

void TireSlipSynthesizer::reset() noexcept
{
    rngState_ = 0x12345678U;
    lastCenterFreq_ = 0.0;
    bandpass_.reset();
    gravelFilter_.reset();
}

// --- RotorSynthesizer ---

double RotorSynthesizer::process(
    double rotorRpm, std::int32_t bladeCount, double thrustLoad, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || rotorRpm < 10.0) {
        return 0.0;
    }

    const double bpf { std::clamp((static_cast<double>(bladeCount) * rotorRpm) / 60.0, 5.0, 400.0) };
    const double phaseInc { bpf / sampleRate };

    phase_ += phaseInc;
    if (phase_ >= 1.0) {
        phase_ -= 1.0;
    }

    // Asymmetric impulsive raised-cosine modulation envelope
    const double s { 0.5 * (1.0 + std::sin(AudioDspMath::kTwoPi * phase_)) };
    const double env { s * s * s };

    const double tri { (phase_ < 0.5) ? ((4.0 * phase_) - 1.0) : (3.0 - (4.0 * phase_)) };
    const double noise { AudioDspMath::xorshift32(rngState_) };
    const double carrier { (0.60 * tri) + (0.40 * noise) };

    const double amp { std::clamp(0.20 + (0.80 * thrustLoad), 0.0, 1.0) };
    return std::clamp(carrier * env * amp * 1.5, -1.0, 1.0);
}

void RotorSynthesizer::reset() noexcept
{
    phase_ = 0.0;
    rngState_ = 0x87654321U;
}

// --- ThrusterSynthesizer ---

ThrusterSynthesizer::ThrusterSynthesizer() noexcept
{
    rumbleFilter_.setLowPass(44100.0, 48.0, 2.2);
}

double ThrusterSynthesizer::process(double throttle, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || throttle <= 0.001) {
        return 0.0;
    }

    const double white { AudioDspMath::xorshift32(rngState_) };

    // 3-pole Paul Kellet pink noise approximation
    b0_ = (0.99765 * b0_) + (white * 0.0990460);
    b1_ = (0.96300 * b1_) + (white * 0.2965164);
    b2_ = (0.57000 * b2_) + (white * 1.0526913);
    const double pink { (b0_ + b1_ + b2_ + (white * 0.1848)) * 0.20 };

    const double rumble { rumbleFilter_.process(pink) };
    const double composite { (0.65 * pink) + (0.35 * rumble) };
    const double driven { std::tanh(1.8 * composite) };

    return std::clamp(driven * std::clamp(throttle, 0.0, 1.0), -1.0, 1.0);
}

void ThrusterSynthesizer::reset() noexcept
{
    b0_ = 0.0;
    b1_ = 0.0;
    b2_ = 0.0;
    rngState_ = 0xABCDEF01U;
    rumbleFilter_.reset();
}

// --- ImpactSynthesizer ---

void ImpactSynthesizer::trigger(double impulse) noexcept
{
    const double amp { std::clamp(impulse / 1200.0, 0.0, 1.0) };
    amplitude_ = std::max(amplitude_, amp);
    time_ = 0.0;
}

double ImpactSynthesizer::process(double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || amplitude_ <= 0.001) {
        return 0.0;
    }

    time_ += (1.0 / sampleRate);
    if (time_ > 0.45) {
        amplitude_ = 0.0;
        return 0.0;
    }

    constexpr double f1 { 440.0 };
    const double m1 { std::sin(AudioDspMath::kTwoPi * f1 * time_) * std::exp(-time_ / 0.25) };
    const double m2 { 0.65 * std::sin(AudioDspMath::kTwoPi * (1.62 * f1) * time_) * std::exp(-time_ / 0.16) };
    const double m3 { 0.40 * std::sin(AudioDspMath::kTwoPi * (2.38 * f1) * time_) * std::exp(-time_ / 0.10) };
    const double m4 { 0.22 * std::sin(AudioDspMath::kTwoPi * (3.45 * f1) * time_) * std::exp(-time_ / 0.05) };

    const double out { amplitude_ * (m1 + m2 + m3 + m4) * 0.55 };
    return std::clamp(out, -1.0, 1.0);
}

void ImpactSynthesizer::reset() noexcept
{
    amplitude_ = 0.0;
    time_ = 0.0;
}

} // namespace vehicle_sim::audio
