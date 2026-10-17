#pragma once

#include <cstdint>

#include "libs/VehicleSimCore/audio/AudioDspMath.h"
#include "libs/VehicleSimCore/audio/CylinderHarmonicSynth.h"
#include "libs/VehicleSimCore/audio/DrivetrainAcousticSynth.h"
#include "libs/VehicleSimCore/audio/ExhaustPopSynth.h"
#include "libs/VehicleSimCore/audio/ForcedInductionSynth.h"
#include "libs/VehicleSimCore/audio/VehicleAcousticProfile.h"

namespace vehicle_sim::audio {

/// @class EngineSynthesizer
/// @brief Synthesizes internal combustion engine acoustic signatures.
/// @details Combines multi-cylinder harmonic modeling, turbo/supercharger induction,
/// and throttle overrun crackles with zero heap allocation.
class EngineSynthesizer {
public:
    EngineSynthesizer() noexcept = default;

    /// @brief Configures engine synthesis with vehicle acoustic profile.
    /// @param[in] profile Vehicle acoustic calibration profile.
    void configure(const VehicleAcousticProfile& profile) noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] rpm Engine revolutions per minute.
    /// @param[in] throttle Throttle opening [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized engine sample in [-1.0, 1.0].
    [[nodiscard]] double process(double rpm, double throttle, double sampleRate) noexcept;

    /// @brief Resets oscillator phase and filter registers.
    void reset() noexcept;

    /// @brief Queries current turbocharger / induction sound component.
    /// @return Most recent forced induction component sample.
    [[nodiscard]] double getLastInductionSample() const noexcept { return lastInduction_; }

private:
    VehicleAcousticProfile profile_ { };
    CylinderHarmonicSynth cylinderSynth_ { };
    ForcedInductionSynth inductionSynth_ { };
    ExhaustPopSynth popSynth_ { };
    bool configured_ { false };
    double lastInduction_ { 0.0 };

    double phase_ { 0.0 };
    double subPhase_ { 0.0 };
    double lastCutoff_ { 0.0 };
    BiquadFilter filter_ { };
};

/// @class TireSlipSynthesizer
/// @brief Synthesizes tire scrub and skid squeal noise across road surfaces.
/// @details Generates resonant bandpass-filtered white noise scaling with slip velocity
/// and surface friction texture.
class TireSlipSynthesizer {
public:
    TireSlipSynthesizer() noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] slipVelocity Composite tire scrub velocity in m/s.
    /// @param[in] surfaceMu Road surface friction coefficient.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized squeal sample in [-1.0, 1.0].
    [[nodiscard]] double process(double slipVelocity, double surfaceMu, double sampleRate) noexcept;

    /// @brief Resets noise seed and filter state.
    void reset() noexcept;

private:
    std::uint32_t rngState_ { 0x12345678U };
    double lastCenterFreq_ { 0.0 };
    BiquadFilter bandpass_ { };
    BiquadFilter gravelFilter_ { };
};

/// @class RotorSynthesizer
/// @brief Synthesizes helicopter and drone aerodynamic rotor chopping.
class RotorSynthesizer {
public:
    constexpr RotorSynthesizer() noexcept = default;

    /// @brief Advances synthesis by one sample.
    /// @param[in] rotorRpm Rotor rotational speed in RPM.
    /// @param[in] bladeCount Number of rotor blades (2 for heli, 3 for drone).
    /// @param[in] thrustLoad Aerodynamic thrust factor [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized rotor chop sample in [-1.0, 1.0].
    [[nodiscard]] double process(double rotorRpm, int bladeCount, double thrustLoad, double sampleRate) noexcept;

    /// @brief Resets phase accumulator.
    void reset() noexcept;

private:
    double phase_ { 0.0 };
    std::uint32_t rngState_ { 0x87654321U };
};

/// @class ThrusterSynthesizer
/// @brief Synthesizes rocket exhaust roar and hovercraft fan acoustics.
class ThrusterSynthesizer {
public:
    ThrusterSynthesizer() noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] throttle Thruster throttle level [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized thruster roar sample in [-1.0, 1.0].
    [[nodiscard]] double process(double throttle, double sampleRate) noexcept;

    /// @brief Resets filter states.
    void reset() noexcept;

private:
    double b0_ { 0.0 };
    double b1_ { 0.0 };
    double b2_ { 0.0 };
    std::uint32_t rngState_ { 0xABCDEF01U };
    BiquadFilter rumbleFilter_ { };
};

/// @class ImpactSynthesizer
/// @brief Multi-modal metallic resonance ping triggered on collision impulses.
class ImpactSynthesizer {
public:
    constexpr ImpactSynthesizer() noexcept = default;

    /// @brief Triggers a new metallic collision ping.
    /// @param[in] impulse Contact impulse magnitude in N*s.
    void trigger(double impulse) noexcept;

    /// @brief Advances synthesis by one sample.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized impact sample in [-1.0, 1.0].
    [[nodiscard]] double process(double sampleRate) noexcept;

    /// @brief Resets modal states and silences active ping.
    void reset() noexcept;

private:
    double amplitude_ { 0.0 };
    double time_ { 0.0 };
};

} // namespace vehicle_sim::audio
