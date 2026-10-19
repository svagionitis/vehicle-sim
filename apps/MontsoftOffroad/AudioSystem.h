#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include <SDL2/SDL.h>

namespace montsoft_offroad {

/// @enum SoundEffect
/// @brief Catalog of discrete synthesized procedural retro sound events.
enum class SoundEffect : std::uint8_t {
    JumpThruster = 0,
    RocketLaunch,
    MortarLaunch,
    ExplosionBlast,
    ArmorClink,
    CheckpointChime,
    BossWarning,
    BossRoar,
    StageClear,
    GameOver
};

/// @struct SoundChannelState
/// @brief Telemetry and oscillator parameters for a synthetic tone or noise generator.
struct SoundChannelState {
    bool active { false }; ///< Active tone generation flag.
    double frequency { 440.0 }; ///< Current instantaneous frequency in Hz.
    double targetFrequency { 440.0 }; ///< Target frequency for continuous sweeps.
    double freqSlideRate { 0.0 }; ///< Frequency change in Hz per second.
    double phase { 0.0 }; ///< Continuous phase accumulator in [0.0, 1.0).
    double dutyCycle { 0.5 }; ///< Pulse width duty cycle in [0.0, 1.0].
    double amplitude { 1.0 }; ///< Instantaneous output amplitude in [0.0, 1.0].
    double decayRate { 0.0 }; ///< Linear amplitude decay in units per second.
    double duration { 0.0 }; ///< Total allotted duration in seconds.
    double elapsed { 0.0 }; ///< Elapsed tone duration in seconds.
    bool isNoise { false }; ///< True if generator utilizes LFSR noise.
};

/// @class AudioSystem
/// @brief PC-Speaker square wave and noise synthesizer for Montsoft Offroad.
/// @details Synthesizes real-time audio via SDL audio callback stream, emulating
/// the 8253 PIT square-wave oscillator and 16-bit Galois LFSR noise generator.
class AudioSystem final {
public:
    static constexpr int kSampleRate { 44100 };
    static constexpr std::size_t kChannelCount { 4 };

    /// @brief Default constructor.
    AudioSystem() noexcept;

    /// @brief Destructor closing audio device cleanly.
    ~AudioSystem() noexcept;

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) = delete;
    AudioSystem& operator=(AudioSystem&&) = delete;

    /// @brief Initializes SDL audio subsystem and opens playback device stream.
    /// @param[in] headless True to run without attempting hardware audio init.
    /// @return True on successful initialization or graceful headless bypass.
    [[nodiscard]] bool initialize(bool headless = false) noexcept;

    /// @brief Shuts down audio device and releases memory.
    void shutdown() noexcept;

    /// @brief Updates continuous audio parameters and steps envelopes.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Triggers a discrete synthesized sound effect.
    /// @param[in] sfx SoundEffect enum identifier.
    void playSfx(SoundEffect sfx) noexcept;

    /// @brief Sets engine oscillator pitch from vehicle forward speed.
    /// @param[in] speed Longitudinal speed in m/s.
    /// @param[in] throttle Throttle demand in [0.0, 1.0].
    /// @param[in] isAirborne True if both axles have lost terrain contact.
    void setEngineSpeed(double speed, double throttle, bool isAirborne) noexcept;

    /// @brief Silences the continuous engine sound channel.
    void stopEngine() noexcept;

    /// @brief Sets master playback volume.
    /// @param[in] volume Normalized volume in [0.0, 1.0].
    void setMasterVolume(double volume) noexcept;

    /// @brief Gets master playback volume.
    /// @return Current normalized volume in [0.0, 1.0].
    [[nodiscard]] double getMasterVolume() const noexcept;

    /// @brief Toggles audio mute state.
    void toggleMute() noexcept;

    /// @brief Sets audio mute state.
    /// @details Silences all procedural sound generators and music playback when enabled.
    /// @param[in] muted True to silence audio output, false to enable.
    void setMuted(bool muted) noexcept;

    /// @brief Queries whether audio is currently muted.
    /// @return True if muted.
    [[nodiscard]] bool isMuted() const noexcept;

    /// @brief Checks if hardware audio playback device is initialized and active.
    /// @return True if device is open.
    [[nodiscard]] bool isAudioActive() const noexcept;

    /// @brief SDL audio callback processing function.
    /// @param[in] userdata Pointer to AudioSystem instance.
    /// @param[out] stream Raw audio byte stream to populate.
    /// @param[in] len Length of audio buffer in bytes.
    static void audioCallback(void* userdata, Uint8* stream, int len) noexcept;

private:
    void processAudioBuffer(std::int16_t* buffer, int sampleCount) noexcept;
    void stepLfsrNoise() noexcept;

    SDL_AudioDeviceID deviceId_ { 0 };
    bool audioActive_ { false };
    std::atomic<bool> muted_ { true };
    std::atomic<double> masterVolume_ { 0.75 };

    mutable std::mutex audioMutex_ {};
    std::array<SoundChannelState, kChannelCount> channels_ {};

    // Continuous engine telemetry
    double engineSpeed_ { 0.0 };
    double engineThrottle_ { 0.0 };
    bool engineAirborne_ { false };
    double enginePhaseSub_ { 0.0 };

    // LFSR noise generator state
    std::uint16_t lfsrState_ { 0xACE1U };
};

} // namespace montsoft_offroad
