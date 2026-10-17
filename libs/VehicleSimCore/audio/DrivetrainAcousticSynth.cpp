#include "libs/VehicleSimCore/audio/DrivetrainAcousticSynth.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::audio {

double DrivetrainAcousticSynth::processGearWhine(double wheelSpeed, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || wheelSpeed < 0.5) {
        return 0.0;
    }

    // Straight-cut gear teeth mesh frequency
    constexpr double kEffectiveTireRadius { 0.33 };
    constexpr double kGearTeeth { 34.0 };
    const double meshFreq { std::clamp(kGearTeeth * (wheelSpeed / (AudioDspMath::kTwoPi * kEffectiveTireRadius)), 40.0, 4800.0) };

    gearPhase_ += (meshFreq / sampleRate);
    if (gearPhase_ >= 1.0) {
        gearPhase_ -= 1.0;
    }

    const double fund { std::sin(AudioDspMath::kTwoPi * gearPhase_) };
    const double harmonic { 0.35 * std::sin(AudioDspMath::kTwoPi * 2.0 * gearPhase_) };
    const double rawWhine { fund + harmonic };

    // Amplitude increases with vehicle speed
    const double normSpeed { std::clamp(wheelSpeed / 50.0, 0.0, 1.0) };
    const double amp { std::pow(normSpeed, 1.1) * 0.45 };

    return std::clamp(rawWhine * amp, -1.0, 1.0);
}

void DrivetrainAcousticSynth::triggerAirBrakes(double intensity) noexcept
{
    airTime_ = 0.0;
    airAmp_ = std::clamp(intensity, 0.1, 1.0);
    airFilter_.setBandPass(44100.0, 1800.0, 1.5);
}

double DrivetrainAcousticSynth::processAirBrakes(double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || airTime_ >= 0.40) {
        return 0.0;
    }

    airTime_ += (1.0 / sampleRate);
    const double noise { AudioDspMath::xorshift32(rngState_) };
    const double filtered { airFilter_.process(noise) };

    // Air hiss with sharp mechanical pressure relief purge
    const double hissEnv { airAmp_ * std::exp(-airTime_ / 0.12) };
    const double purgeClick { (airTime_ < 0.02) ? (airAmp_ * 0.8) : 0.0 };

    return std::clamp((filtered * hissEnv * 1.6) + purgeClick, -1.0, 1.0);
}

double DrivetrainAcousticSynth::processTrackClatter(double trackSpeed, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || trackSpeed < 0.3) {
        return 0.0;
    }

    // Sprocket teeth tooth strike frequency
    const double strikeFreq { std::clamp(trackSpeed * 18.0, 10.0, 600.0) };
    trackPhase_ += (strikeFreq / sampleRate);
    if (trackPhase_ >= 1.0) {
        trackPhase_ -= 1.0;
    }

    const double strike { (trackPhase_ < 0.15) ? (1.0 - (trackPhase_ / 0.15)) : 0.0 };
    const double noise { AudioDspMath::xorshift32(rngState_) * 0.25 };
    const double normSpeed { std::clamp(trackSpeed / 15.0, 0.0, 1.0) };

    return std::clamp((strike + noise) * normSpeed * 0.45, -1.0, 1.0);
}

double DrivetrainAcousticSynth::processElectricPwm(double motorTorque, double motorSpeed, double sampleRate) noexcept
{
    if (sampleRate <= 0.0) {
        return 0.0;
    }

    const double clampedSpeed { std::clamp(motorSpeed, 0.0, 1.0) };
    const double clampedTorque { std::clamp(motorTorque, 0.0, 1.0) };

    const double pwmFreq { 2400.0 + (clampedSpeed * 800.0) };
    pwmPhase_ += (pwmFreq / sampleRate);
    if (pwmPhase_ >= 1.0) {
        pwmPhase_ -= 1.0;
    }

    const double pwmSine { std::sin(AudioDspMath::kTwoPi * pwmPhase_) };
    const double amp { (0.15 + (0.85 * clampedTorque)) * (0.20 + (0.80 * clampedSpeed)) * 0.35 };

    return std::clamp(pwmSine * amp, -1.0, 1.0);
}

void DrivetrainAcousticSynth::reset() noexcept
{
    gearPhase_ = 0.0;
    trackPhase_ = 0.0;
    pwmPhase_ = 0.0;
    airTime_ = 1.0;
    airAmp_ = 0.0;
    airFilter_.reset();
    gearFilter_.reset();
}

} // namespace vehicle_sim::audio
