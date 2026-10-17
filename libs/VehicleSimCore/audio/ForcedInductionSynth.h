#pragma once

#include <cstdint>

#include "libs/VehicleSimCore/audio/AudioDspMath.h"
#include "libs/VehicleSimCore/audio/VehicleAcousticProfile.h"

namespace vehicle_sim::audio {

/// @class ForcedInductionSynth
/// @brief Synthesizes turbocharger spool, blow-off valve sneeze, and supercharger whine.
class ForcedInductionSynth {
public:
    constexpr ForcedInductionSynth() noexcept = default;

    /// @brief Configures forced induction model.
    /// @param[in] type Turbocharger or Supercharger.
    /// @param[in] gearRatio Pulley drive ratio for supercharger blower.
    /// @param[in] maxWhistleFreq Peak turbo whistle frequency in Hz.
    void configure(ForcedInductionType type, double gearRatio, double maxWhistleFreq) noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] rpm Engine rotational speed in RPM.
    /// @param[in] throttle Throttle opening [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized forced induction sample in [-1.0, 1.0].
    [[nodiscard]] double process(double rpm, double throttle, double sampleRate) noexcept;

    /// @brief Resets internal states and timers.
    void reset() noexcept;

private:
    ForcedInductionType type_ { ForcedInductionType::None };
    double gearRatio_ { 2.4 };
    double maxWhistleFreq_ { 5200.0 };

    double turboSpool_ { 0.0 };
    double lastThrottle_ { 0.0 };
    double whistlePhase_ { 0.0 };
    double blowerPhase_ { 0.0 };

    // BOV and compressor surge flutter states
    double bovTime_ { 1.0 };
    double bovAmplitude_ { 0.0 };
    double flutterPhase_ { 0.0 };

    std::uint32_t rngState_ { 0x55AA33CCU };
    BiquadFilter bovFilter_ { };
};

} // namespace vehicle_sim::audio
