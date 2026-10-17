#include "libs/VehicleSimCore/audio/ExhaustPopSynth.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::audio {

double ExhaustPopSynth::process(double rpm, double throttle, double intensity, double sampleRate) noexcept
{
    if (sampleRate <= 0.0 || intensity <= 0.01) {
        return 0.0;
    }

    const double dt { 1.0 / sampleRate };
    const double clampedThrottle { std::clamp(throttle, 0.0, 1.0) };

    // Trigger condition: throttle lifted, elevated RPM
    if (clampedThrottle < 0.08 && rpm > 3200.0) {
        const double normRpm { std::clamp((rpm - 3200.0) / 4500.0, 0.0, 1.0) };
        // Poisson rate lambda in pops per second (up to 14 pops/sec under high RPM)
        const double lambda { intensity * normRpm * normRpm * 14.0 };
        const double probPerSample { lambda * dt };

        const double rnd { (AudioDspMath::xorshift32(rngState_) + 1.0) * 0.5 };
        if (rnd < probPerSample && popTime_ >= 0.04) {
            popTime_ = 0.0;
            popDecay_ = 0.020 + (0.035 * ((AudioDspMath::xorshift32(rngState_) + 1.0) * 0.5));
            popAmp_ = std::clamp(intensity * (0.6 + (0.4 * normRpm)), 0.1, 1.0);

            // Randomize filter frequency between 650 Hz and 1500 Hz
            const double centerFreq { 650.0 + (850.0 * ((AudioDspMath::xorshift32(rngState_) + 1.0) * 0.5)) };
            popBandpass_.setBandPass(sampleRate, centerFreq, 2.5);
        }
    }

    lastThrottle_ = clampedThrottle;

    if (popTime_ < 0.08) {
        popTime_ += dt;
        const double noise { AudioDspMath::xorshift32(rngState_) };
        const double filtered { popBandpass_.process(noise) };
        const double env { popAmp_ * std::exp(-popTime_ / popDecay_) };
        const double raw { filtered * env * 2.2 };
        return std::clamp(std::tanh(raw), -1.0, 1.0);
    }

    return 0.0;
}

void ExhaustPopSynth::reset() noexcept
{
    popTime_ = 1.0;
    popDecay_ = 0.03;
    popAmp_ = 0.0;
    lastThrottle_ = 0.0;
    popBandpass_.reset();
}

} // namespace vehicle_sim::audio
