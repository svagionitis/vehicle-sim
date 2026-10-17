#include "apps/MotionLab/AudioSystem.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace motion_lab {

namespace {
    constexpr double kSampleRate { 44100.0 };
}

AudioSystem::AudioSystem() noexcept = default;

AudioSystem::~AudioSystem() noexcept
{
    shutdown();
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
    desired.freq = static_cast<int>(kSampleRate);
    desired.format = AUDIO_S16SYS;
    desired.channels = 1;
    desired.samples = 512;
    desired.callback = audioCallback;
    desired.userdata = this;

    SDL_AudioSpec obtained { };
    deviceId_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
    if (deviceId_ == 0) {
        return false;
    }

    SDL_PauseAudioDevice(deviceId_, 0);
    initialized_ = true;
    return true;
}

void AudioSystem::shutdown() noexcept
{
    if (deviceId_ != 0) {
        SDL_CloseAudioDevice(deviceId_);
        deviceId_ = 0;
    }
    initialized_ = false;
    headless_ = false;
}

void AudioSystem::updateTelemetry(MotionLabVehicleType type, double rpm, double throttle, double slipVelocity,
    double surfaceMu, double rotorSpeed, double thrusterPower) noexcept
{
    params_.vehicleType.store(static_cast<std::uint8_t>(type), std::memory_order_relaxed);
    params_.rpm.store(static_cast<float>(rpm), std::memory_order_relaxed);
    params_.throttle.store(static_cast<float>(throttle), std::memory_order_relaxed);
    params_.slipVelocity.store(static_cast<float>(slipVelocity), std::memory_order_relaxed);
    params_.surfaceMu.store(static_cast<float>(surfaceMu), std::memory_order_relaxed);
    params_.rotorSpeed.store(static_cast<float>(rotorSpeed), std::memory_order_relaxed);
    params_.thrusterPower.store(static_cast<float>(thrusterPower), std::memory_order_relaxed);
}

void AudioSystem::triggerImpact(double impulse) noexcept
{
    const float current { params_.impactImpulse.load(std::memory_order_relaxed) };
    const auto target = static_cast<float>(impulse);
    if (target > current) {
        params_.impactImpulse.store(target, std::memory_order_relaxed);
    }
}

void AudioSystem::toggleMute() noexcept
{
    const bool current { params_.muted.load(std::memory_order_relaxed) };
    params_.muted.store(!current, std::memory_order_relaxed);
}

void AudioSystem::setMuted(bool muted) noexcept
{
    params_.muted.store(muted, std::memory_order_relaxed);
}

bool AudioSystem::isMuted() const noexcept
{
    return params_.muted.load(std::memory_order_relaxed);
}

void AudioSystem::adjustVolume(float delta) noexcept
{
    const float current { params_.masterVolume.load(std::memory_order_relaxed) };
    const float next { std::clamp(current + delta, 0.0f, 1.0f) };
    params_.masterVolume.store(next, std::memory_order_relaxed);
}

float AudioSystem::getVolume() const noexcept
{
    return params_.masterVolume.load(std::memory_order_relaxed);
}

bool AudioSystem::isInitialized() const noexcept
{
    return initialized_;
}

AudioChannelLevels AudioSystem::getLevels() const noexcept
{
    AudioChannelLevels levels { };
    levels.engineLevel = rmsEngine_.load(std::memory_order_relaxed);
    levels.tireLevel = rmsTire_.load(std::memory_order_relaxed);
    levels.rotorLevel = rmsRotor_.load(std::memory_order_relaxed);
    levels.thrusterLevel = rmsThruster_.load(std::memory_order_relaxed);
    levels.impactLevel = rmsImpact_.load(std::memory_order_relaxed);
    levels.masterLevel = rmsMaster_.load(std::memory_order_relaxed);
    return levels;
}

void AudioSystem::resetSynths() noexcept
{
    engineSynth_.reset();
    tireSynth_.reset();
    rotorSynth_.reset();
    thrusterSynth_.reset();
    impactSynth_.reset();
}

void AudioSystem::synthesizeHeadless(std::int16_t* outSamples, std::size_t sampleCount) noexcept
{
    processBuffer(outSamples, sampleCount);
}

void AudioSystem::audioCallback(void* userdata, std::uint8_t* stream, int len) noexcept
{
    auto* self = static_cast<AudioSystem*>(userdata);
    if (self == nullptr || stream == nullptr || len <= 0) {
        return;
    }

    auto* outBuffer = reinterpret_cast<std::int16_t*>(stream);
    const std::size_t sampleCount { static_cast<std::size_t>(len) / sizeof(std::int16_t) };
    self->processBuffer(outBuffer, sampleCount);
}

void AudioSystem::processBuffer(std::int16_t* outStream, std::size_t sampleCount) noexcept
{
    if (outStream == nullptr || sampleCount == 0U) {
        return;
    }

    const bool muted { params_.muted.load(std::memory_order_relaxed) };
    const float masterVol { params_.masterVolume.load(std::memory_order_relaxed) };

    if (muted || masterVol <= 0.001f) {
        std::memset(outStream, 0, sampleCount * sizeof(std::int16_t));
        rmsEngine_.store(0.0f, std::memory_order_relaxed);
        rmsTire_.store(0.0f, std::memory_order_relaxed);
        rmsRotor_.store(0.0f, std::memory_order_relaxed);
        rmsThruster_.store(0.0f, std::memory_order_relaxed);
        rmsImpact_.store(0.0f, std::memory_order_relaxed);
        rmsMaster_.store(0.0f, std::memory_order_relaxed);
        return;
    }

    const auto vType = static_cast<MotionLabVehicleType>(params_.vehicleType.load(std::memory_order_relaxed));
    const double rpm { static_cast<double>(params_.rpm.load(std::memory_order_relaxed)) };
    const double throttle { static_cast<double>(params_.throttle.load(std::memory_order_relaxed)) };
    const double slipVelocity { static_cast<double>(params_.slipVelocity.load(std::memory_order_relaxed)) };
    const double surfaceMu { static_cast<double>(params_.surfaceMu.load(std::memory_order_relaxed)) };
    const double rotorSpeed { static_cast<double>(params_.rotorSpeed.load(std::memory_order_relaxed)) };
    const double thrusterPower { static_cast<double>(params_.thrusterPower.load(std::memory_order_relaxed)) };

    const float pendingImpulse { params_.impactImpulse.exchange(0.0f, std::memory_order_relaxed) };
    if (pendingImpulse > 0.1f) {
        impactSynth_.trigger(static_cast<double>(pendingImpulse));
    }

    // Determine active sound channel topology
    bool hasEngine { true };
    bool hasTires { true };
    bool hasRotor { false };
    bool hasThruster { false };
    int rotorBlades { 2 };

    switch (vType) {
    case MotionLabVehicleType::ClassicCar:
    case MotionLabVehicleType::RaceCar:
    case MotionLabVehicleType::DriftCar:
    case MotionLabVehicleType::MonsterTruck:
    case MotionLabVehicleType::SemiTruck:
    case MotionLabVehicleType::Forklift:
    case MotionLabVehicleType::TrackedTank:
        hasEngine = true;
        hasTires = true;
        break;
    case MotionLabVehicleType::Rocket:
        hasEngine = false;
        hasTires = false;
        hasThruster = true;
        break;
    case MotionLabVehicleType::Helicopter:
        hasEngine = true; // Turbine background hum
        hasTires = false;
        hasRotor = true;
        rotorBlades = 2;
        break;
    case MotionLabVehicleType::Hovercraft:
        hasEngine = false;
        hasTires = false;
        hasThruster = true;
        hasRotor = true; // Lift fan
        rotorBlades = 4;
        break;
    case MotionLabVehicleType::AntiGravity:
        hasEngine = true; // High-pitch EM drive
        hasTires = false;
        hasThruster = true; // Plasma jet
        break;
    case MotionLabVehicleType::QuadcopterDrone:
        hasEngine = false;
        hasTires = false;
        hasRotor = true;
        rotorBlades = 3;
        break;
    }

    double sumSqEngine { 0.0 };
    double sumSqTire { 0.0 };
    double sumSqRotor { 0.0 };
    double sumSqThruster { 0.0 };
    double sumSqImpact { 0.0 };
    double sumSqMaster { 0.0 };

    for (std::size_t i { 0 }; i < sampleCount; ++i) {
        double engSample { 0.0 };
        double tireSample { 0.0 };
        double rotorSample { 0.0 };
        double thrustSample { 0.0 };

        if (hasEngine) {
            engSample = engineSynth_.process(rpm, throttle, kSampleRate);
            sumSqEngine += (engSample * engSample);
        }
        if (hasTires) {
            tireSample = tireSynth_.process(slipVelocity, surfaceMu, kSampleRate);
            sumSqTire += (tireSample * tireSample);
        }
        if (hasRotor) {
            rotorSample = rotorSynth_.process(rotorSpeed, rotorBlades, throttle, kSampleRate);
            sumSqRotor += (rotorSample * rotorSample);
        }
        if (hasThruster) {
            thrustSample = thrusterSynth_.process(thrusterPower, kSampleRate);
            sumSqThruster += (thrustSample * thrustSample);
        }

        const double impactSample { impactSynth_.process(kSampleRate) };
        sumSqImpact += (impactSample * impactSample);

        // Master summer bus with calibrated channel gains
        const double masterRaw { (0.42 * engSample) + (0.48 * tireSample) + (0.38 * rotorSample)
            + (0.42 * thrustSample) + (0.55 * impactSample) };

        // Zero-latency cubic soft-knee saturation
        const double limited { audio::AudioDspMath::softLimit(masterRaw) };
        const double masterOut { limited * static_cast<double>(masterVol) };
        sumSqMaster += (masterOut * masterOut);

        const double clamped16 { std::clamp(masterOut * 32767.0, -32767.0, 32767.0) };
        outStream[i] = static_cast<std::int16_t>(clamped16);
    }

    const double invSamples { 1.0 / static_cast<double>(sampleCount) };
    rmsEngine_.store(static_cast<float>(std::sqrt(sumSqEngine * invSamples)), std::memory_order_relaxed);
    rmsTire_.store(static_cast<float>(std::sqrt(sumSqTire * invSamples)), std::memory_order_relaxed);
    rmsRotor_.store(static_cast<float>(std::sqrt(sumSqRotor * invSamples)), std::memory_order_relaxed);
    rmsThruster_.store(static_cast<float>(std::sqrt(sumSqThruster * invSamples)), std::memory_order_relaxed);
    rmsImpact_.store(static_cast<float>(std::sqrt(sumSqImpact * invSamples)), std::memory_order_relaxed);
    rmsMaster_.store(static_cast<float>(std::sqrt(sumSqMaster * invSamples)), std::memory_order_relaxed);
}

} // namespace motion_lab
