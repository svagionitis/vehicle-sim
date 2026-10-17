#pragma once

#include <cstdint>

#include "libs/VehicleSimCore/audio/AudioDspMath.h"

namespace vehicle_sim::audio {

/// @class ExhaustPopSynth
/// @brief Stochastic deceleration overrun pop and backfire generator.
class ExhaustPopSynth {
public:
    constexpr ExhaustPopSynth() noexcept = default;

    /// @brief Advances synthesis by one sample.
    /// @param[in] rpm Engine rotational speed in RPM.
    /// @param[in] throttle Throttle opening in [0.0, 1.0].
    /// @param[in] intensity Tuning intensity of backfires [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized exhaust pop sample in [-1.0, 1.0].
    [[nodiscard]] double process(double rpm, double throttle, double intensity, double sampleRate) noexcept;

    /// @brief Resets pop state and filter registers.
    void reset() noexcept;

private:
    double popTime_ { 1.0 };
    double popDecay_ { 0.03 };
    double popAmp_ { 0.0 };
    double lastThrottle_ { 0.0 };

    std::uint32_t rngState_ { 0x99887766U };
    BiquadFilter popBandpass_ { };
};

} // namespace vehicle_sim::audio
