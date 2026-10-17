#include "libs/VehicleSimCore/audio/CylinderHarmonicSynth.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::audio {

void CylinderHarmonicSynth::configure(EngineConfiguration config, double knockIntensity) noexcept
{
    config_ = config;
    knockIntensity_ = std::clamp(knockIntensity, 0.0, 1.0);
}

double CylinderHarmonicSynth::process(double rpm, double throttle, double sampleRate) noexcept
{
    if (sampleRate <= 0.0) {
        return 0.0;
    }

    const double clampedRpm { std::max(rpm, 100.0) };
    const double crankFreq { clampedRpm / 60.0 };

    // Determine cylinder firing multiplier
    double firingMultiplier { 2.0 };
    switch (config_) {
    case EngineConfiguration::Inline4:
        firingMultiplier = 2.0; // 4 cyl / 2 revs
        break;
    case EngineConfiguration::V6:
    case EngineConfiguration::HeavyDieselI6:
        firingMultiplier = 3.0; // 6 cyl / 2 revs
        break;
    case EngineConfiguration::CrossplaneV8:
    case EngineConfiguration::FlatplaneV8:
        firingMultiplier = 4.0; // 8 cyl / 2 revs
        break;
    case EngineConfiguration::V10:
        firingMultiplier = 5.0; // 10 cyl / 2 revs
        break;
    case EngineConfiguration::HeavyDieselV12:
        firingMultiplier = 6.0; // 12 cyl / 2 revs
        break;
    case EngineConfiguration::ElectricPWM:
    case EngineConfiguration::SciFiWarp:
        firingMultiplier = 1.0;
        break;
    }

    const double fireFreq { std::clamp(crankFreq * firingMultiplier, 15.0, 3200.0) };
    const double phaseInc { fireFreq / sampleRate };
    const double crankInc { crankFreq / sampleRate };

    crankPhase_ += crankInc;
    if (crankPhase_ >= 1.0) {
        crankPhase_ -= 1.0;
        // Trigger TDC knock on crank revolution for diesel
        if (knockIntensity_ > 0.01) {
            knockTime_ = 0.0;
            knockAmp_ = knockIntensity_ * (0.4 + (0.6 * std::clamp(throttle, 0.0, 1.0)));
        }
    }

    subPhase_ += (phaseInc * 0.5);
    if (subPhase_ >= 1.0) {
        subPhase_ -= 1.0;
    }

    double rawSignal { 0.0 };

    if (config_ == EngineConfiguration::ElectricPWM) {
        // High-frequency PWM inverter whine + motor fundamental
        const double pwmCarrier { 2400.0 + (clampedRpm * 0.25) };
        const double pwmSample { std::sin(AudioDspMath::kTwoPi * pwmCarrier * (crankPhase_ * (sampleRate / crankFreq)) / sampleRate) };
        const double motorHum { AudioDspMath::polyBlepSaw(crankPhase_, crankInc) };
        rawSignal = (0.65 * pwmSample * (0.3 + (0.7 * throttle))) + (0.35 * motorHum);
    } else if (config_ == EngineConfiguration::SciFiWarp) {
        // Dual-carrier FM synthesis
        const double carrierPhase { crankPhase_ };
        const double modPhase { std::fmod(crankPhase_ * 2.414, 1.0) };
        const double modulator { std::sin(AudioDspMath::kTwoPi * modPhase) };
        rawSignal = std::sin((AudioDspMath::kTwoPi * carrierPhase) + (2.5 * modulator));
    } else if (config_ == EngineConfiguration::CrossplaneV8) {
        // Asymmetric syncopated V8 burble
        const double pulse1 { AudioDspMath::polyBlepSaw(std::fmod(crankPhase_ * 4.0, 1.0), crankInc * 4.0) };
        const double pulse2 { AudioDspMath::polyBlepSaw(std::fmod((crankPhase_ * 4.0) + 0.125, 1.0), crankInc * 4.0) };
        const double subSaw { AudioDspMath::polyBlepSaw(subPhase_, phaseInc * 0.5) };
        rawSignal = (0.50 * pulse1) + (0.30 * pulse2) + (0.20 * subSaw);
    } else {
        // Regular even-pulse PolyBLEP saw with sub-harmonic
        const double saw { AudioDspMath::polyBlepSaw(std::fmod(crankPhase_ * firingMultiplier, 1.0), phaseInc) };
        const double subSaw { AudioDspMath::polyBlepSaw(subPhase_, phaseInc * 0.5) };
        rawSignal = (0.72 * saw) + (0.28 * subSaw);
    }

    // Add diesel combustion knock transient
    if (knockTime_ < 0.025) {
        knockTime_ += (1.0 / sampleRate);
        constexpr double kKnockFreq { 2200.0 };
        const double knockEnv { knockAmp_ * std::exp(-knockTime_ / 0.0045) };
        const double knockSample { knockEnv * std::sin(AudioDspMath::kTwoPi * kKnockFreq * knockTime_) };
        rawSignal += (0.45 * knockSample);
    }

    // Formant exhaust chamber filtering
    const double clampedThrottle { std::clamp(throttle, 0.0, 1.0) };
    const double cutoff1 { 140.0 + (70.0 * clampedThrottle) };
    const double cutoff2 { 480.0 + (3200.0 * clampedThrottle * clampedThrottle) };

    if (std::abs(cutoff1 - lastCutoff1_) > 8.0) {
        formantFilter1_.setBandPass(sampleRate, cutoff1, 2.2);
        lastCutoff1_ = cutoff1;
    }
    if (std::abs(cutoff2 - lastCutoff2_) > 20.0) {
        formantFilter2_.setLowPass(sampleRate, cutoff2, 1.414);
        lastCutoff2_ = cutoff2;
    }

    const double filteredLow { formantFilter2_.process(rawSignal) };
    const double cavityResonance { formantFilter1_.process(rawSignal) };
    double out { (0.75 * filteredLow) + (0.25 * cavityResonance) };

    // Rev limiter stutter
    if (rpm >= 7200.0 && config_ != EngineConfiguration::ElectricPWM) {
        const double stut { std::sin(AudioDspMath::kTwoPi * 24.0 * crankPhase_) };
        if (stut < 0.0) {
            out *= 0.12;
        }
    }

    return std::clamp(out, -1.0, 1.0);
}

void CylinderHarmonicSynth::reset() noexcept
{
    crankPhase_ = 0.0;
    subPhase_ = 0.0;
    knockTime_ = 1.0;
    knockAmp_ = 0.0;
    lastCutoff1_ = 0.0;
    lastCutoff2_ = 0.0;
    formantFilter1_.reset();
    formantFilter2_.reset();
}

} // namespace vehicle_sim::audio
