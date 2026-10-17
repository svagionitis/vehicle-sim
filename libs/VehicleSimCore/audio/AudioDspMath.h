#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vehicle_sim::audio {

/// @struct BiquadCoeffs
/// @brief Digital biquad filter transfer function coefficients.
struct BiquadCoeffs {
    double b0 { 1.0 }; ///< Feedforward coefficient 0.
    double b1 { 0.0 }; ///< Feedforward coefficient 1.
    double b2 { 0.0 }; ///< Feedforward coefficient 2.
    double a1 { 0.0 }; ///< Feedback coefficient 1.
    double a2 { 0.0 }; ///< Feedback coefficient 2.
};

/// @class BiquadFilter
/// @brief Second-order Direct Form I IIR digital filter.
/// @details Real-time, zero-allocation filter supporting lowpass and bandpass topologies.
class BiquadFilter {
public:
    constexpr BiquadFilter() noexcept = default;

    /// @brief Configures resonant low-pass filter coefficients.
    /// @param[in] sampleRate Sampling frequency in Hz (e.g. 44100).
    /// @param[in] cutoff Cutoff frequency in Hz.
    /// @param[in] q Resonance factor (default 1.414).
    void setLowPass(double sampleRate, double cutoff, double q = 1.414) noexcept
    {
        if (sampleRate <= 0.0 || cutoff <= 0.0) {
            return;
        }
        constexpr double kPi { 3.14159265358979323846 };
        const double clampedCutoff { std::clamp(cutoff, 10.0, sampleRate * 0.49) };
        const double omega { 2.0 * kPi * clampedCutoff / sampleRate };
        const double sinOmega { std::sin(omega) };
        const double cosOmega { std::cos(omega) };
        const double alpha { sinOmega / (2.0 * std::max(0.1, q)) };

        const double a0 { 1.0 + alpha };
        coeffs_.b0 = ((1.0 - cosOmega) * 0.5) / a0;
        coeffs_.b1 = (1.0 - cosOmega) / a0;
        coeffs_.b2 = ((1.0 - cosOmega) * 0.5) / a0;
        coeffs_.a1 = (-2.0 * cosOmega) / a0;
        coeffs_.a2 = (1.0 - alpha) / a0;
    }

    /// @brief Configures resonant band-pass filter coefficients.
    /// @param[in] sampleRate Sampling frequency in Hz.
    /// @param[in] center Center frequency in Hz.
    /// @param[in] q Resonance quality factor.
    void setBandPass(double sampleRate, double center, double q = 3.0) noexcept
    {
        if (sampleRate <= 0.0 || center <= 0.0) {
            return;
        }
        constexpr double kPi { 3.14159265358979323846 };
        const double clampedCenter { std::clamp(center, 20.0, sampleRate * 0.49) };
        const double omega { 2.0 * kPi * clampedCenter / sampleRate };
        const double sinOmega { std::sin(omega) };
        const double cosOmega { std::cos(omega) };
        const double alpha { sinOmega / (2.0 * std::max(0.1, q)) };

        const double a0 { 1.0 + alpha };
        coeffs_.b0 = alpha / a0;
        coeffs_.b1 = 0.0;
        coeffs_.b2 = -alpha / a0;
        coeffs_.a1 = (-2.0 * cosOmega) / a0;
        coeffs_.a2 = (1.0 - alpha) / a0;
    }

    /// @brief Processes a single input sample through the filter.
    /// @param[in] in Input audio sample.
    /// @return Filtered output sample.
    [[nodiscard]] double process(double in) noexcept
    {
        const double out { (coeffs_.b0 * in) + (coeffs_.b1 * x1_) + (coeffs_.b2 * x2_)
            - (coeffs_.a1 * y1_) - (coeffs_.a2 * y2_) };

        x2_ = x1_;
        x1_ = in;
        y2_ = y1_;
        y1_ = out;

        return out;
    }

    /// @brief Clears internal delay line states to zero.
    void reset() noexcept
    {
        x1_ = 0.0;
        x2_ = 0.0;
        y1_ = 0.0;
        y2_ = 0.0;
    }

private:
    BiquadCoeffs coeffs_ { };
    double x1_ { 0.0 };
    double x2_ { 0.0 };
    double y1_ { 0.0 };
    double y2_ { 0.0 };
};

/// @class AudioDspMath
/// @brief Pure C++17 digital signal processing algorithms for procedural audio.
/// @details Header-only, IEEE 754 deterministic DSP utilities with zero external dependencies.
class AudioDspMath {
public:
    static constexpr double kPi { 3.14159265358979323846 };
    static constexpr double kTwoPi { 2.0 * kPi };

    /// @brief Computes anti-aliased PolyBLEP residual correction for step discontinuities.
    /// @param[in] phase Current normalized phase in [0.0, 1.0).
    /// @param[in] phaseInc Phase increment per sample (frequency / sampleRate).
    /// @return PolyBLEP anti-aliasing correction factor.
    [[nodiscard]] static double polyBlep(double phase, double phaseInc) noexcept
    {
        if (phaseInc <= 0.0) {
            return 0.0;
        }
        if (phase < phaseInc) {
            const double t { phase / phaseInc };
            return (2.0 * t) - (t * t) - 1.0;
        }
        if (phase > (1.0 - phaseInc)) {
            const double t { (phase - 1.0) / phaseInc };
            return (t * t) + (2.0 * t) + 1.0;
        }
        return 0.0;
    }

    /// @brief Generates a bandlimited PolyBLEP sawtooth sample in [-1.0, 1.0].
    /// @param[in] phase Current normalized phase in [0.0, 1.0).
    /// @param[in] phaseInc Phase increment per sample.
    /// @return Bandlimited sawtooth sample.
    [[nodiscard]] static double polyBlepSaw(double phase, double phaseInc) noexcept
    {
        const double naive { (2.0 * phase) - 1.0 };
        return naive - polyBlep(phase, phaseInc);
    }

    /// @brief Advances 32-bit Xorshift state and returns uniform random in [-1.0, 1.0].
    /// @param[in,out] state PRNG state register (must be non-zero).
    /// @return Uniform floating-point white noise sample.
    [[nodiscard]] static double xorshift32(std::uint32_t& state) noexcept
    {
        if (state == 0U) {
            state = 0x54321098U;
        }
        state ^= (state << 13);
        state ^= (state >> 17);
        state ^= (state << 5);
        return (static_cast<double>(state & 0x7FFFFFFFu) / 1073741824.0) - 1.0;
    }

    /// @brief Evaluates zero-latency cubic soft-knee saturation to prevent integer clipping.
    /// @param[in] input Raw summing bus sample.
    /// @return Saturated sample strictly bounded within [-1.0, 1.0].
    [[nodiscard]] static double softLimit(double input) noexcept
    {
        if (input <= -1.0) {
            return -1.0;
        }
        if (input >= 1.0) {
            return 1.0;
        }
        return (1.5 * input) - (0.5 * input * input * input);
    }
};

} // namespace vehicle_sim::audio
