#pragma once

#include <cstdint>

#include "libs/VehicleSimCore/audio/AudioDspMath.h"
#include "libs/VehicleSimCore/audio/VehicleAcousticProfile.h"

namespace vehicle_sim::audio {

/// @class CylinderHarmonicSynth
/// @brief Multi-cylinder internal combustion acoustic engine generator.
/// @details Synthesizes individual cylinder firing pulses, crossplane syncopated phase offsets,
/// and heavy diesel combustion knock transients with zero dynamic allocations.
class CylinderHarmonicSynth {
public:
    constexpr CylinderHarmonicSynth() noexcept = default;

    /// @brief Configures engine synthesis parameters.
    /// @param[in] config Cylinder arrangement and firing sequence.
    /// @param[in] knockIntensity Transient diesel knock amplitude [0.0, 1.0].
    void configure(EngineConfiguration config, double knockIntensity) noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] rpm Engine rotational speed in RPM.
    /// @param[in] throttle Throttle opening [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized combustion sample in [-1.0, 1.0].
    [[nodiscard]] double process(double rpm, double throttle, double sampleRate) noexcept;

    /// @brief Resets all oscillator phases and filter states.
    void reset() noexcept;

private:
    EngineConfiguration config_ { EngineConfiguration::Inline4 };
    double knockIntensity_ { 0.0 };
    double crankPhase_ { 0.0 };
    double subPhase_ { 0.0 };

    double knockTime_ { 1.0 };
    double knockAmp_ { 0.0 };

    double lastCutoff1_ { 0.0 };
    double lastCutoff2_ { 0.0 };
    BiquadFilter formantFilter1_ { };
    BiquadFilter formantFilter2_ { };
};

} // namespace vehicle_sim::audio
