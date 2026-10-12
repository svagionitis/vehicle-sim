#include "apps/EarthPatrol/AudioSystem.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace earth_patrol {

namespace {
    inline constexpr int kSampleRate { 44100 };
    inline constexpr double kPi { 3.14159265358979323846 };

    uint32_t gAudioRnd { 0x54321098u };

    double nextAudioRandom() noexcept
    {
        gAudioRnd = (gAudioRnd * 1103515245u) + 12345u;
        return (static_cast<double>(gAudioRnd & 0x7FFFFFFFu) / 2147483648.0) * 2.0 - 1.0;
    }
} // namespace

AudioSystem::AudioSystem() noexcept = default;

AudioSystem::~AudioSystem() noexcept
{
    if (deviceId_ != 0) {
        SDL_CloseAudioDevice(deviceId_);
        deviceId_ = 0;
    }
}

AudioSystem::AudioSystem(AudioSystem&& other) noexcept
    : deviceId_ { other.deviceId_ }
    , initialized_ { other.initialized_ }
    , muted_ { other.muted_ }
    , headless_ { other.headless_ }
{
    other.deviceId_ = 0;
    other.initialized_ = false;
}

AudioSystem& AudioSystem::operator=(AudioSystem&& other) noexcept
{
    if (this != &other) {
        if (deviceId_ != 0) {
            SDL_CloseAudioDevice(deviceId_);
        }
        deviceId_ = other.deviceId_;
        initialized_ = other.initialized_;
        muted_ = other.muted_;
        headless_ = other.headless_;

        other.deviceId_ = 0;
        other.initialized_ = false;
    }
    return *this;
}

bool AudioSystem::initialize(bool headless) noexcept
{
    headless_ = headless;
    if (headless_) {
        initialized_ = true;
        return true;
    }

    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        return false;
    }

    SDL_AudioSpec desired { };
    desired.freq = kSampleRate;
    desired.format = AUDIO_S16SYS;
    desired.channels = 1;
    desired.samples = 1024;
    desired.callback = nullptr;

    SDL_AudioSpec obtained { };
    deviceId_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);

    if (deviceId_ == 0) {
        return false;
    }

    SDL_PauseAudioDevice(deviceId_, 0);
    initialized_ = true;
    return true;
}

void AudioSystem::setMuted(bool muted) noexcept
{
    muted_ = muted;
}

bool AudioSystem::isMuted() const noexcept
{
    return muted_;
}

bool AudioSystem::isInitialized() const noexcept
{
    return initialized_;
}

void AudioSystem::playJump() noexcept
{
    playEffect(SoundEffect::JumpThruster);
}

void AudioSystem::playLanding() noexcept
{
    playEffect(SoundEffect::WheelLanding);
}

void AudioSystem::playCrash() noexcept
{
    playEffect(SoundEffect::CrashExplosion);
}

void AudioSystem::playMilestone() noexcept
{
    playEffect(SoundEffect::MilestoneChime);
}

void AudioSystem::playStageClear() noexcept
{
    playEffect(SoundEffect::StageClearFanfare);
}

void AudioSystem::playEffect(SoundEffect effect) noexcept
{
    if (muted_ || !initialized_ || deviceId_ == 0) {
        return;
    }

    std::vector<int16_t> samples { };

    if (effect == SoundEffect::JumpThruster) {
        constexpr double duration { 0.22 };
        const auto totalSamples { static_cast<std::size_t>(duration * kSampleRate) };
        samples.resize(totalSamples);

        double phase { 0.0 };
        for (std::size_t i { 0 }; i < totalSamples; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kSampleRate) };
            const double freq { 260.0 + (420.0 * (t / duration)) };
            phase += 2.0 * kPi * freq / static_cast<double>(kSampleRate);

            // Sine and soft square harmonic
            double wave { std::sin(phase) + (0.35 * ((std::sin(phase) >= 0.0) ? 1.0 : -1.0)) };
            const double env { 1.0 - (t / duration) };
            wave *= env * 0.55;

            samples[i] = static_cast<int16_t>(std::clamp(wave * 26000.0, -32000.0, 32000.0));
        }
    } else if (effect == SoundEffect::WheelLanding) {
        constexpr double duration { 0.12 };
        const auto totalSamples { static_cast<std::size_t>(duration * kSampleRate) };
        samples.resize(totalSamples);

        double phase { 0.0 };
        for (std::size_t i { 0 }; i < totalSamples; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kSampleRate) };
            const double freq { 110.0 * std::exp(-t * 22.0) };
            phase += 2.0 * kPi * freq / static_cast<double>(kSampleRate);

            double wave { std::sin(phase) + (0.2 * nextAudioRandom()) };
            const double env { std::exp(-t * 28.0) };
            wave *= env * 0.65;

            samples[i] = static_cast<int16_t>(std::clamp(wave * 28000.0, -32000.0, 32000.0));
        }
    } else if (effect == SoundEffect::CrashExplosion) {
        constexpr double duration { 0.45 };
        const auto totalSamples { static_cast<std::size_t>(duration * kSampleRate) };
        samples.resize(totalSamples);

        double phase { 0.0 };
        for (std::size_t i { 0 }; i < totalSamples; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kSampleRate) };
            phase += 2.0 * kPi * 55.0 / static_cast<double>(kSampleRate);

            const double noise { nextAudioRandom() };
            const double rumble { std::sin(phase) };
            const double env { std::exp(-t * 7.5) };
            const double wave { ((0.75 * noise) + (0.35 * rumble)) * env * 0.75 };

            samples[i] = static_cast<int16_t>(std::clamp(wave * 30000.0, -32000.0, 32000.0));
        }
    } else if (effect == SoundEffect::MilestoneChime) {
        constexpr double duration { 0.28 };
        const auto totalSamples { static_cast<std::size_t>(duration * kSampleRate) };
        samples.resize(totalSamples);

        double phase { 0.0 };
        for (std::size_t i { 0 }; i < totalSamples; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kSampleRate) };
            const double freq { (t < 0.10) ? 880.0 : 1320.0 };
            phase += 2.0 * kPi * freq / static_cast<double>(kSampleRate);

            const double env { (t < 0.10) ? (1.0 - (t / 0.10)) : (1.0 - ((t - 0.10) / 0.18)) };
            const double wave { std::sin(phase) * env * 0.50 };

            samples[i] = static_cast<int16_t>(std::clamp(wave * 26000.0, -32000.0, 32000.0));
        }
    } else if (effect == SoundEffect::StageClearFanfare) {
        constexpr double duration { 0.44 };
        const auto totalSamples { static_cast<std::size_t>(duration * kSampleRate) };
        samples.resize(totalSamples);

        double phase { 0.0 };
        for (std::size_t i { 0 }; i < totalSamples; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kSampleRate) };
            double freq { 523.25 }; // C5
            if (t >= 0.30) {
                freq = 1046.50; // C6
            } else if (t >= 0.20) {
                freq = 783.99; // G5
            } else if (t >= 0.10) {
                freq = 659.25; // E5
            }

            phase += 2.0 * kPi * freq / static_cast<double>(kSampleRate);
            const double wave { std::sin(phase) * 0.45 };

            samples[i] = static_cast<int16_t>(std::clamp(wave * 26000.0, -32000.0, 32000.0));
        }
    }

    if (!samples.empty()) {
        const auto byteCount { static_cast<Uint32>(samples.size() * sizeof(int16_t)) };
        SDL_QueueAudio(deviceId_, samples.data(), byteCount);
    }
}

} // namespace earth_patrol
