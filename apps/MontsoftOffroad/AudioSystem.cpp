#include "apps/MontsoftOffroad/AudioSystem.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace montsoft_offroad {

AudioSystem::AudioSystem() noexcept = default;

AudioSystem::~AudioSystem() noexcept
{
    shutdown();
}

bool AudioSystem::initialize(bool headless) noexcept
{
    if (headless) {
        audioActive_ = false;
        return true;
    }

    if (SDL_WasInit(SDL_INIT_AUDIO) == 0) {
        if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
            audioActive_ = false;
            return true; // Graceful fallback
        }
    }

    SDL_AudioSpec desired {};
    desired.freq = kSampleRate;
    desired.format = AUDIO_S16SYS;
    desired.channels = 1; // Mono PC Speaker
    desired.samples = 1024;
    desired.callback = &AudioSystem::audioCallback;
    desired.userdata = this;

    SDL_AudioSpec obtained {};
    deviceId_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
    if (deviceId_ > 0) {
        audioActive_ = true;
        SDL_PauseAudioDevice(deviceId_, 0); // Start playback
    } else {
        audioActive_ = false;
    }

    return true;
}

void AudioSystem::shutdown() noexcept
{
    if (deviceId_ > 0) {
        SDL_PauseAudioDevice(deviceId_, 1);
        SDL_CloseAudioDevice(deviceId_);
        deviceId_ = 0;
    }
    audioActive_ = false;
}

void AudioSystem::update(double dt) noexcept
{
    std::lock_guard<std::mutex> lock { audioMutex_ };

    // Update Channel 0 (Engine)
    auto& ch0 { channels_[0] };
    if (ch0.active) {
        const double targetFreq {
            engineAirborne_ ? 290.0 : (65.0 + std::clamp(std::abs(engineSpeed_) / 35.0, 0.0, 1.0) * (290.0 - 65.0))
        };
        // Smoothly interpolate engine frequency
        ch0.frequency += (targetFreq - ch0.frequency) * std::min(1.0, dt * 15.0);
    }

    // Step channels 1 through 3
    for (std::size_t i { 1 }; i < kChannelCount; ++i) {
        auto& ch { channels_[i] };
        if (!ch.active) {
            continue;
        }

        ch.elapsed += dt;
        if (ch.duration > 0.0 && ch.elapsed >= ch.duration) {
            ch.active = false;
            continue;
        }

        // Apply frequency slide
        if (std::abs(ch.freqSlideRate) > 1e-6) {
            ch.frequency += ch.freqSlideRate * dt;
            if (ch.freqSlideRate < 0.0 && ch.frequency < ch.targetFrequency) {
                ch.frequency = ch.targetFrequency;
            } else if (ch.freqSlideRate > 0.0 && ch.frequency > ch.targetFrequency) {
                ch.frequency = ch.targetFrequency;
            }
        }

        // Apply amplitude decay
        if (ch.decayRate > 0.0) {
            ch.amplitude = std::max(0.0, ch.amplitude - ch.decayRate * dt);
            if (ch.amplitude <= 0.0) {
                ch.active = false;
            }
        }
    }
}

void AudioSystem::playSfx(SoundEffect sfx) noexcept
{
    std::lock_guard<std::mutex> lock { audioMutex_ };

    switch (sfx) {
    case SoundEffect::JumpThruster: {
        auto& ch { channels_[1] };
        ch.active = true;
        ch.frequency = 180.0;
        ch.targetFrequency = 880.0;
        ch.freqSlideRate = (880.0 - 180.0) / 0.22;
        ch.dutyCycle = 0.50;
        ch.amplitude = 0.65;
        ch.decayRate = 0.65 / 0.22;
        ch.duration = 0.22;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::RocketLaunch: {
        auto& ch { channels_[1] };
        ch.active = true;
        ch.frequency = 950.0;
        ch.targetFrequency = 320.0;
        ch.freqSlideRate = (320.0 - 950.0) / 0.15;
        ch.dutyCycle = 0.25;
        ch.amplitude = 0.70;
        ch.decayRate = 0.70 / 0.15;
        ch.duration = 0.15;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::MortarLaunch: {
        auto& ch { channels_[1] };
        ch.active = true;
        ch.frequency = 120.0;
        ch.targetFrequency = 440.0;
        ch.freqSlideRate = (440.0 - 120.0) / 0.18;
        ch.dutyCycle = 0.50;
        ch.amplitude = 0.80;
        ch.decayRate = 0.80 / 0.18;
        ch.duration = 0.18;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::ExplosionBlast: {
        auto& ch { channels_[2] };
        ch.active = true;
        ch.frequency = 120.0;
        ch.amplitude = 0.85;
        ch.decayRate = 0.85 / 0.65;
        ch.duration = 0.65;
        ch.elapsed = 0.0;
        ch.isNoise = true;
        break;
    }
    case SoundEffect::ArmorClink: {
        auto& ch { channels_[1] };
        ch.active = true;
        ch.frequency = 1600.0;
        ch.targetFrequency = 1800.0;
        ch.freqSlideRate = (1800.0 - 1600.0) / 0.06;
        ch.dutyCycle = 0.125;
        ch.amplitude = 0.50;
        ch.decayRate = 0.50 / 0.06;
        ch.duration = 0.06;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::CheckpointChime: {
        auto& ch { channels_[3] };
        ch.active = true;
        ch.frequency = 523.0; // C5
        ch.targetFrequency = 784.0; // G5
        ch.freqSlideRate = (784.0 - 523.0) / 0.35;
        ch.dutyCycle = 0.50;
        ch.amplitude = 0.75;
        ch.decayRate = 0.75 / 0.35;
        ch.duration = 0.35;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::BossWarning: {
        auto& ch { channels_[1] };
        ch.active = true;
        ch.frequency = 440.0;
        ch.targetFrequency = 660.0;
        ch.freqSlideRate = (660.0 - 440.0) / 0.40;
        ch.dutyCycle = 0.25;
        ch.amplitude = 0.70;
        ch.decayRate = 0.0;
        ch.duration = 0.40;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::BossRoar: {
        auto& ch { channels_[2] };
        ch.active = true;
        ch.frequency = 85.0;
        ch.targetFrequency = 35.0;
        ch.freqSlideRate = (35.0 - 85.0) / 0.50;
        ch.amplitude = 0.90;
        ch.decayRate = 0.90 / 0.50;
        ch.duration = 0.50;
        ch.elapsed = 0.0;
        ch.isNoise = true;
        break;
    }
    case SoundEffect::StageClear: {
        auto& ch { channels_[3] };
        ch.active = true;
        ch.frequency = 523.0; // C5
        ch.targetFrequency = 1046.0; // C6
        ch.freqSlideRate = (1046.0 - 523.0) / 1.20;
        ch.dutyCycle = 0.50;
        ch.amplitude = 0.80;
        ch.decayRate = 0.80 / 1.20;
        ch.duration = 1.20;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    case SoundEffect::GameOver: {
        auto& ch { channels_[3] };
        ch.active = true;
        ch.frequency = 440.0; // A4
        ch.targetFrequency = 220.0; // A3
        ch.freqSlideRate = (220.0 - 440.0) / 1.50;
        ch.dutyCycle = 0.50;
        ch.amplitude = 0.75;
        ch.decayRate = 0.75 / 1.50;
        ch.duration = 1.50;
        ch.elapsed = 0.0;
        ch.isNoise = false;
        break;
    }
    default:
        break;
    }
}

void AudioSystem::setEngineSpeed(double speed, double throttle, bool isAirborne) noexcept
{
    std::lock_guard<std::mutex> lock { audioMutex_ };
    engineSpeed_ = speed;
    engineThrottle_ = throttle;
    engineAirborne_ = isAirborne;

    auto& ch0 { channels_[0] };
    ch0.active = true;
    ch0.duration = -1.0; // Infinite continuous channel
    ch0.dutyCycle = 0.50;
    ch0.amplitude = 0.35 + 0.15 * std::clamp(throttle, 0.0, 1.0);
    ch0.decayRate = 0.0;
    ch0.isNoise = false;
}

void AudioSystem::stopEngine() noexcept
{
    std::lock_guard<std::mutex> lock { audioMutex_ };
    channels_[0].active = false;
}

void AudioSystem::setMasterVolume(double volume) noexcept
{
    masterVolume_.store(std::clamp(volume, 0.0, 1.0));
}

double AudioSystem::getMasterVolume() const noexcept
{
    return masterVolume_.load();
}

void AudioSystem::toggleMute() noexcept
{
    muted_.store(!muted_.load());
}

bool AudioSystem::isMuted() const noexcept
{
    return muted_.load();
}

bool AudioSystem::isAudioActive() const noexcept
{
    return audioActive_;
}

void AudioSystem::stepLfsrNoise() noexcept
{
    const std::uint16_t lsb { static_cast<std::uint16_t>(lfsrState_ & 1U) };
    lfsrState_ >>= 1U;
    if (lsb != 0U) {
        lfsrState_ ^= 0xB400U; // Galois polynomial feedback mask
    }
}

void AudioSystem::processAudioBuffer(std::int16_t* buffer, int sampleCount) noexcept
{
    if (muted_.load()) {
        std::memset(buffer, 0, static_cast<std::size_t>(sampleCount) * sizeof(std::int16_t));
        return;
    }

    const double masterVol { masterVolume_.load() };
    const double dtSample { 1.0 / static_cast<double>(kSampleRate) };

    std::lock_guard<std::mutex> lock { audioMutex_ };

    for (int i { 0 }; i < sampleCount; ++i) {
        double mixSample { 0.0 };

        // 1. Channel 0: Engine Square + Sub-harmonic growl
        auto& ch0 { channels_[0] };
        if (ch0.active && ch0.amplitude > 0.0) {
            ch0.phase += ch0.frequency * dtSample;
            if (ch0.phase >= 1.0) {
                ch0.phase -= std::floor(ch0.phase);
            }
            const double mainSquare { (ch0.phase < ch0.dutyCycle) ? 1.0 : -1.0 };

            // Sub-harmonic oscillator at half frequency
            enginePhaseSub_ += (0.5 * ch0.frequency) * dtSample;
            if (enginePhaseSub_ >= 1.0) {
                enginePhaseSub_ -= std::floor(enginePhaseSub_);
            }
            const double subSquare { (enginePhaseSub_ < 0.5) ? 1.0 : -1.0 };
            const double engineVal { 0.70 * mainSquare + 0.30 * subSquare };

            mixSample += engineVal * ch0.amplitude;
        }

        // 2. Channels 1 to 3: SFX, Noise, Melodies
        for (std::size_t c { 1 }; c < kChannelCount; ++c) {
            auto& ch { channels_[c] };
            if (!ch.active || ch.amplitude <= 0.0) {
                continue;
            }

            if (ch.isNoise) {
                stepLfsrNoise();
                // Map LFSR state [0, 65535] to [-1.0, 1.0]
                const double noiseSample { (static_cast<double>(lfsrState_) / 32767.5) - 1.0 };
                mixSample += noiseSample * ch.amplitude;
            } else {
                ch.phase += ch.frequency * dtSample;
                if (ch.phase >= 1.0) {
                    ch.phase -= std::floor(ch.phase);
                }
                const double squareSample { (ch.phase < ch.dutyCycle) ? 1.0 : -1.0 };
                mixSample += squareSample * ch.amplitude;
            }
        }

        // Apply master volume, soft saturation limiter, and clamp to int16 range
        const double clampedSample { std::clamp(mixSample * masterVol, -1.0, 1.0) };
        buffer[i] = static_cast<std::int16_t>(clampedSample * 32767.0);
    }
}

void AudioSystem::audioCallback(void* userdata, Uint8* stream, int len) noexcept
{
    if (userdata == nullptr || stream == nullptr || len <= 0) {
        return;
    }

    auto* self { static_cast<AudioSystem*>(userdata) };
    const int sampleCount { len / static_cast<int>(sizeof(std::int16_t)) };
    auto* buffer { reinterpret_cast<std::int16_t*>(stream) };

    self->processAudioBuffer(buffer, sampleCount);
}

} // namespace montsoft_offroad
