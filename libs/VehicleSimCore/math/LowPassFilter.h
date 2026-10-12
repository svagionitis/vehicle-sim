#pragma once

namespace vehicle_sim::math {

/// @class LowPassFilter
/// @brief Discrete first-order low-pass filter (exponential moving average).
/// @details Attenuates high-frequency signal noise in continuous time using single-pole discretization.
/// Parameterized by a continuous -3 dB cutoff frequency in Hertz. Adheres strictly to IEEE 754
/// floating-point determinism and pure C++17 standards with zero dynamic allocations.
class LowPassFilter {
public:
    /// @brief Mathematical constant 2 * Pi (6.28318530717958647692).
    static constexpr double kTwoPi { 6.28318530717958647692 };

    /// @brief Default constructor initializing with 10.0 Hz cutoff.
    constexpr LowPassFilter() noexcept = default;

    /// @brief Constructs a low-pass filter with a specified cutoff frequency.
    /// @param[in] cutoffHz -3 dB cutoff frequency in Hertz.
    explicit LowPassFilter(double cutoffHz) noexcept;

    /// @brief Filters an input sample across an elapsed duration.
    /// @details If uninitialized, the filter snaps immediately to the input without transient lag.
    /// Guarded against zero and negative time deltas.
    /// @param[in] input Raw input signal sample.
    /// @param[in] dt Elapsed time step in seconds.
    /// @return Filtered output signal value.
    [[nodiscard]] double update(double input, double dt) noexcept;

    /// @brief Clears internal filter history, leaving it uninitialized for next sample.
    void reset() noexcept;

    /// @brief Resets the filter to an explicit initial value, marking it initialized.
    /// @param[in] initialVal Initial value to assign to the filter accumulator.
    void reset(double initialVal) noexcept;

    /// @brief Configures filter cutoff frequency.
    /// @param[in] cutoffHz -3 dB cutoff frequency in Hertz.
    void setCutoff(double cutoffHz) noexcept;

    /// @brief Returns the configured cutoff frequency in Hertz.
    /// @return Cutoff frequency in Hz.
    [[nodiscard]] constexpr double cutoff() const noexcept
    {
        return mCutoffHz;
    }

    /// @brief Returns the current filtered value.
    /// @return Stored filter output.
    [[nodiscard]] constexpr double value() const noexcept
    {
        return mValue;
    }

    /// @brief Checks whether the filter has received at least one sample.
    /// @return True if initialized with a sample or reset with value.
    [[nodiscard]] constexpr bool isInitialized() const noexcept
    {
        return mInitialized;
    }

private:
    double mCutoffHz { 10.0 };
    double mValue { 0.0 };
    bool mInitialized { false };
};

} // namespace vehicle_sim::math
