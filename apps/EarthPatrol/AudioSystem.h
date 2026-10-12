#pragma once

#include <cstdint>

#include <SDL2/SDL.h>

namespace earth_patrol {

/// @enum SoundEffect
/// @brief Classification of procedural retro arcade sound effects.
enum class SoundEffect : std::uint8_t {
    JumpThruster, ///< Frequency-sweep thruster whoosh.
    WheelLanding, ///< Low-frequency suspension touchdown thump.
    CrashExplosion, ///< Decaying white noise burst and sub-bass rumble.
    MilestoneChime, ///< Dual-tone high melodic checkpoint bell.
    StageClearFanfare ///< Multi-note triumphant arpeggio sequence.
};

/// @class AudioSystem
/// @brief Lightweight procedural audio synthesizer for Earth Patrol arcade SFX.
/// @details Generates retro arcade waveforms on-the-fly and enqueues them via SDL audio
/// with zero external media files, asset load failures, or heap fragmentation.
class AudioSystem {
public:
    /// @brief Default constructor.
    AudioSystem() noexcept;

    /// @brief Destructor closing active audio device.
    ~AudioSystem() noexcept;

    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;
    AudioSystem(AudioSystem&&) noexcept;
    AudioSystem& operator=(AudioSystem&&) noexcept;

    /// @brief Initializes SDL audio subsystem and opens default playback device.
    /// @param[in] headless True to skip opening physical audio hardware.
    /// @return True if initialized successfully, false otherwise.
    [[nodiscard]] bool initialize(bool headless = false) noexcept;

    /// @brief Triggers the jump thruster ascent sound effect.
    /// @details Synthesizes a frequency-modulated upward sine sweep and pushes it to SDL audio queue.
    void playJump() noexcept;

    /// @brief Triggers the wheel touchdown suspension thump sound effect.
    /// @details Synthesizes a decaying low-frequency square waveform simulation of tire compression.
    void playLanding() noexcept;

    /// @brief Triggers the vehicle crash destruction explosion sound effect.
    /// @details Synthesizes a decaying white noise burst mixed with low-frequency rumble.
    void playCrash() noexcept;

    /// @brief Triggers the checkpoint milestone chime sound effect.
    /// @details Synthesizes a two-tone high pitch harmonic bell sound.
    void playMilestone() noexcept;

    /// @brief Triggers the stage clear celebratory fanfare sound effect.
    /// @details Synthesizes an arpeggiated melodic retro sequence for milestone completion.
    void playStageClear() noexcept;

    /// @brief Sets global audio mute state.
    /// @details When muted, sound effect synthesis requests are discarded immediately.
    /// @param[in] muted True to silence all sound effects.
    void setMuted(bool muted) noexcept;

    /// @brief Queries global audio mute state.
    /// @details Checks whether audio output is currently suppressed.
    /// @return True if muted.
    [[nodiscard]] bool isMuted() const noexcept;

    /// @brief Queries whether the audio device was opened successfully.
    /// @details Checks hardware playback device status.
    /// @return True if initialized.
    [[nodiscard]] bool isInitialized() const noexcept;

private:
    /// @brief Generates and enqueues PCM samples for the specified effect.
    /// @param[in] effect SoundEffect identifier to synthesize.
    void playEffect(SoundEffect effect) noexcept;

    SDL_AudioDeviceID deviceId_ { 0 };
    bool initialized_ { false };
    bool muted_ { false };
    bool headless_ { false };
};

} // namespace earth_patrol
