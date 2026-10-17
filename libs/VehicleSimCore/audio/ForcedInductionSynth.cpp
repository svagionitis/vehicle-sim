#include "libs/VehicleSimCore/audio/ForcedInductionSynth.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::audio {

void ForcedInductionSynth::configure(ForcedInductionType type, double gearRatio, double maxWhistleFreq) noexcept
{
    type_ = type;
    gearRatio_ = std::max(gearRatio, 1.0);
    maxWhistleFreq_ = std::clamp(maxWhistleFreq, 2000.0, 12000.0);
    bovFilter_.setHighPass(44100.0, 2400.0, 1.2);
}

double ForcedInductionSynth::process(double rpm, double throttle, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || type_ == ForcedInductionType::None) {
        return 0.0;
    }

    const double dt { 1.0 / sampleRate };
    const double clampedThrottle { std::clamp(throttle, 0.0, 1.0) };
    const double clampedRpm { std::max(rpm, 100.0) };

    if (type_ == ForcedInductionType::Turbocharger) {
        // Target spool level based on engine load & RPM
        const double normRpm { std::clamp((clampedRpm - 1000.0) / 6000.0, 0.0, 1.0) };
        const double targetSpool { normRpm * clampedThrottle };

        // First order lag dynamics
        const double spoolRate { (targetSpool >= turboSpool_) ? (dt / 0.45) : (dt / 1.10) };
        turboSpool_ += spoolRate * (targetSpool - turboSpool_);
        turboSpool_ = std::clamp(turboSpool_, 0.0, 1.0);

        // Detect throttle lift-off rate for Blow-Off Valve trigger
        const double throttleRate { (clampedThrottle - lastThrottle_) / dt };
        lastThrottle_ = clampedThrottle;

        if (throttleRate < -2.2 && turboSpool_ > 0.30 && bovTime_ >= 0.50) {
            bovTime_ = 0.0;
            bovAmplitude_ = std::clamp(turboSpool_ * 1.2, 0.2, 1.0);
            flutterPhase_ = 0.0;
        }

        // 1. Continuous turbo spool whistle
        const double whistleFreq { 1200.0 + (turboSpool_ * (maxWhistleFreq_ - 1200.0)) };
        whistlePhase_ += (whistleFreq / sampleRate);
        if (whistlePhase_ >= 1.0) {
            whistlePhase_ -= 1.0;
        }
        const double whistle { std::sin(AudioDspMath::kTwoPi * whistlePhase_) };
        const double whistleAmp { std::pow(turboSpool_, 2.2) * (0.25 + (0.75 * clampedThrottle)) * 0.45 };
        double out { whistle * whistleAmp };

        // 2. Blow-Off Valve sneeze & compressor surge flutter
        if (bovTime_ < 0.55) {
            bovTime_ += dt;
            flutterPhase_ += (18.0 / sampleRate); // 18 Hz cyclic flutter
            if (flutterPhase_ >= 1.0) {
                flutterPhase_ -= 1.0;
            }

            const double noise { AudioDspMath::xorshift32(rngState_) };
            const double filteredNoise { bovFilter_.process(noise) };

            // Initial pneumatic sneeze
            const double sneezeEnv { bovAmplitude_ * std::exp(-bovTime_ / 0.10) };
            const double sneeze { filteredNoise * sneezeEnv * 0.65 };

            // Decaying cyclic surge flutter
            const double flutterMod { 0.5 * (1.0 + std::sin(AudioDspMath::kTwoPi * flutterPhase_)) };
            const double flutterEnv { bovAmplitude_ * std::exp(-bovTime_ / 0.35) * flutterMod * flutterMod };
            const double flutter { (0.6 * filteredNoise + 0.4 * whistle) * flutterEnv * 0.50 };

            out += (sneeze + flutter);
        }

        return std::clamp(out, -1.0, 1.0);
    }

    if (type_ == ForcedInductionType::Supercharger) {
        // Crank-driven Roots / Screw blower gear mesh whine
        const double blowerFreq { std::clamp(gearRatio_ * 3.0 * (clampedRpm / 60.0), 30.0, 4500.0) };
        blowerPhase_ += (blowerFreq / sampleRate);
        if (blowerPhase_ >= 1.0) {
            blowerPhase_ -= 1.0;
        }

        const double fund { std::sin(AudioDspMath::kTwoPi * blowerPhase_) };
        const double secondHarmonic { std::sin(AudioDspMath::kTwoPi * 2.0 * blowerPhase_) };
        const double blowerSample { (0.70 * fund) + (0.30 * secondHarmonic) };

        const double normRpm { std::clamp(clampedRpm / 6500.0, 0.0, 1.0) };
        const double blowerAmp { (0.20 + (0.80 * normRpm)) * std::pow(clampedThrottle, 1.3) * 0.50 };

        return std::clamp(blowerSample * blowerAmp, -1.0, 1.0);
    }

    return 0.0;
}

void ForcedInductionSynth::reset() noexcept
{
    turboSpool_ = 0.0;
    lastThrottle_ = 0.0;
    whistlePhase_ = 0.0;
    blowerPhase_ = 0.0;
    bovTime_ = 1.0;
    bovAmplitude_ = 0.0;
    flutterPhase_ = 0.0;
    bovFilter_.reset();
}

} // namespace vehicle_sim::audio
