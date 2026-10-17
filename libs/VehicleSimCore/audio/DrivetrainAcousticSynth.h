#pragma once

#include <cstdint>

#include "libs/VehicleSimCore/audio/AudioDspMath.h"

namespace vehicle_sim::audio {

/// @class DrivetrainAcousticSynth
/// @brief Synthesizes straight-cut transmission whine, air-brake purge, and electric PWM.
class DrivetrainAcousticSynth {
public:
    constexpr DrivetrainAcousticSynth() noexcept = default;

    /// @brief Advances transmission gear whine synthesis.
    /// @param[in] wheelSpeed Linear wheel rotational speed in m/s.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized straight-cut gear sample in [-1.0, 1.0].
    [[nodiscard]] double processGearWhine(double wheelSpeed, double sampleRate) noexcept;

    /// @brief Advances pneumatic air-brake purge synthesis.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized air-brake hiss sample in [-1.0, 1.0].
    [[nodiscard]] double processAirBrakes(double sampleRate) noexcept;

    /// @brief Triggers commercial pneumatic air-brake release hiss.
    /// @param[in] intensity Purge intensity [0.0, 1.0].
    void triggerAirBrakes(double intensity) noexcept;

    /// @brief Advances tank track mechanical link clatter.
    /// @param[in] trackSpeed Linear track speed in m/s.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized track link clatter sample in [-1.0, 1.0].
    [[nodiscard]] double processTrackClatter(double trackSpeed, double sampleRate) noexcept;

    /// @brief Advances electric motor inverter PWM whine synthesis.
    /// @param[in] motorTorque Motor load factor [0.0, 1.0].
    /// @param[in] motorSpeed Motor speed factor [0.0, 1.0].
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @return Synthesized electric hum sample in [-1.0, 1.0].
    [[nodiscard]] double processElectricPwm(double motorTorque, double motorSpeed, double sampleRate) noexcept;

    /// @brief Resets internal states and filters.
    void reset() noexcept;

private:
    double gearPhase_ { 0.0 };
    double trackPhase_ { 0.0 };
    double pwmPhase_ { 0.0 };
    double airTime_ { 1.0 };
    double airAmp_ { 0.0 };

    std::uint32_t rngState_ { 0x11223344U };
    BiquadFilter airFilter_ { };
    BiquadFilter gearFilter_ { };
};

} // namespace vehicle_sim::audio
