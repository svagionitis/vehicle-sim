#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <SDL2/SDL.h>

#include "apps/MotionLab/MotionLabVehicleType.h"
#include "libs/VehicleSimCore/audio/AudioChannels.h"

namespace motion_lab {

namespace audio = vehicle_sim::audio;

/// @struct AudioChannelLevels
/// @brief Real-time RMS output levels for HUD telemetry visualization.
struct AudioChannelLevels {
    float engineLevel { 0.0f };   ///< Engine combustion channel RMS in [0.0, 1.0].
    float tireLevel { 0.0f };     ///< Tire scrub noise channel RMS in [0.0, 1.0].
    float rotorLevel { 0.0f };    ///< Rotor aerodynamic chop channel RMS in [0.0, 1.0].
    float thrusterLevel { 0.0f }; ///< Thruster roar channel RMS in [0.0, 1.0].
    float impactLevel { 0.0f };   ///< Impact collision clang channel RMS in [0.0, 1.0].
    float masterLevel { 0.0f };   ///< Composite master output level in [0.0, 1.0].
};

/// @class AudioSystem
/// @brief Master procedural audio coordinator for Motion Lab 2D.
/// @details Opens SDL2 audio hardware playback device, dispatches audio callbacks,
/// and synchronizes physics parameters across threads via lock-free atomic registers.
class AudioSystem {
public:
    /// @brief Default constructor initializing pre-allocated buffers.
    AudioSystem() noexcept;

    /// @brief Destructor safely closing active SDL audio hardware device.
    ~AudioSystem() noexcept;

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) noexcept = delete;
    AudioSystem& operator=(AudioSystem&&) noexcept = delete;

    /// @brief Initializes SDL audio subsystem and opens default playback device.
    /// @param[in] headless True to skip physical audio hardware (for CI tests).
    /// @return True if initialized successfully, false otherwise.
    [[nodiscard]] bool initialize(bool headless = false) noexcept;

    /// @brief Shuts down audio device and releases hardware resources.
    void shutdown() noexcept;

    /// @brief Updates vehicle telemetry parameters from main simulation loop.
    /// @param[in] type Active vehicle chassis type.
    /// @param[in] rpm Engine rotational speed in RPM.
    /// @param[in] throttle Throttle opening [0.0, 1.0].
    /// @param[in] slipVelocity Combined tire scrub velocity in m/s.
    /// @param[in] surfaceMu Ground friction coefficient.
    /// @param[in] rotorSpeed Rotor rotational speed in RPM.
    /// @param[in] thrusterPower Thruster power level [0.0, 1.0].
    void updateTelemetry(MotionLabVehicleType type, double rpm, double throttle, double slipVelocity,
        double surfaceMu, double rotorSpeed, double thrusterPower) noexcept;

    /// @brief Triggers structural collision impact clang.
    /// @param[in] impulse Contact impulse magnitude in N*s.
    void triggerImpact(double impulse) noexcept;

    /// @brief Toggles global audio mute state.
    void toggleMute() noexcept;

    /// @brief Sets global audio mute state.
    /// @param[in] muted True to silence all sound output.
    void setMuted(bool muted) noexcept;

    /// @brief Queries global audio mute state.
    /// @return True if audio output is suppressed.
    [[nodiscard]] bool isMuted() const noexcept;

    /// @brief Adjusts master volume level.
    /// @param[in] delta Signed volume change [-1.0, +1.0].
    void adjustVolume(float delta) noexcept;

    /// @brief Queries current master volume level in [0.0, 1.0].
    /// @return Master volume ratio.
    [[nodiscard]] float getVolume() const noexcept;

    /// @brief Queries whether audio device opened successfully.
    /// @return True if initialized and device active.
    [[nodiscard]] bool isInitialized() const noexcept;

    /// @brief Queries real-time channel RMS levels for HUD display.
    /// @return Snapshot of channel RMS levels.
    [[nodiscard]] AudioChannelLevels getLevels() const noexcept;

    /// @brief Generates synthetic PCM buffer for headless testing.
    /// @param[out] outSamples Destination buffer receiving 16-bit PCM samples.
    /// @param[in] sampleCount Number of samples to synthesize.
    void synthesizeHeadless(std::int16_t* outSamples, std::size_t sampleCount) noexcept;

    /// @brief Resets all synthesizer internal states.
    void resetSynths() noexcept;

private:
    static void audioCallback(void* userdata, std::uint8_t* stream, int len) noexcept;

    struct LockFreeParams {
        std::atomic<float> rpm { 800.0f };
        std::atomic<float> throttle { 0.0f };
        std::atomic<float> slipVelocity { 0.0f };
        std::atomic<float> surfaceMu { 1.0f };
        std::atomic<float> rotorSpeed { 0.0f };
        std::atomic<float> thrusterPower { 0.0f };
        std::atomic<float> impactImpulse { 0.0f };
        std::atomic<std::uint8_t> vehicleType { 0U };
        std::atomic<bool> muted { true };
        std::atomic<float> masterVolume { 0.80f };
    };

    void processBuffer(std::int16_t* outStream, std::size_t sampleCount) noexcept;

    SDL_AudioDeviceID deviceId_ { 0 };
    bool initialized_ { false };
    bool headless_ { false };

    LockFreeParams params_ { };

    audio::EngineSynthesizer engineSynth_ { };
    audio::TireSlipSynthesizer tireSynth_ { };
    audio::RotorSynthesizer rotorSynth_ { };
    audio::ThrusterSynthesizer thrusterSynth_ { };
    audio::ImpactSynthesizer impactSynth_ { };

    std::atomic<float> rmsEngine_ { 0.0f };
    std::atomic<float> rmsTire_ { 0.0f };
    std::atomic<float> rmsRotor_ { 0.0f };
    std::atomic<float> rmsThruster_ { 0.0f };
    std::atomic<float> rmsImpact_ { 0.0f };
    std::atomic<float> rmsMaster_ { 0.0f };
};

} // namespace motion_lab
