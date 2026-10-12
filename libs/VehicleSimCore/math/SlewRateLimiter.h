#pragma once

namespace vehicle_sim::math {

/// @class SlewRateLimiter
/// @brief Rate-of-change limiter for scalar signals and actuator commands.
/// @details Prevents instantaneous step discontinuities by clamping the rate of change
/// of an output signal to maximum rising and falling rates per second.
/// Conforms to IEEE 754 floating-point determinism and pure C++17 standards with zero dynamic allocations.
class SlewRateLimiter {
public:
    /// @brief Default constructor initializing with 10.0 units/sec symmetric rate.
    constexpr SlewRateLimiter() noexcept = default;

    /// @brief Constructs a symmetric slew rate limiter.
    /// @param[in] maxRate Maximum rate of change in units per second.
    explicit SlewRateLimiter(double maxRate) noexcept;

    /// @brief Constructs an asymmetric slew rate limiter.
    /// @param[in] risingRate Maximum increasing rate in units per second.
    /// @param[in] fallingRate Maximum decreasing rate in units per second.
    SlewRateLimiter(double risingRate, double fallingRate) noexcept;

    /// @brief Steps the limiter towards the target value across time step dt.
    /// @details If uninitialized, snaps immediately to target without rate limiting.
    /// Guarded against zero and negative time deltas.
    /// @param[in] target Desired destination value.
    /// @param[in] dt Elapsed time step in seconds.
    /// @return Rate-limited output value.
    [[nodiscard]] double update(double target, double dt) noexcept;

    /// @brief Clears limiter history, leaving it uninitialized for next sample.
    void reset() noexcept;

    /// @brief Resets current value and forces initialized state.
    /// @param[in] initialVal Initial output value.
    void reset(double initialVal) noexcept;

    /// @brief Sets symmetric maximum slew rate.
    /// @param[in] maxRate Maximum rate in units per second.
    void setRateLimit(double maxRate) noexcept;

    /// @brief Sets asymmetric rising and falling slew rates.
    /// @param[in] risingRate Maximum rising rate in units per second.
    /// @param[in] fallingRate Maximum falling rate in units per second.
    void setRateLimits(double risingRate, double fallingRate) noexcept;

    /// @brief Returns the current rate-limited value.
    /// @return Stored output value.
    [[nodiscard]] constexpr double value() const noexcept
    {
        return mCurrentVal;
    }

    /// @brief Returns maximum rising rate limit.
    /// @return Rising rate in units per second.
    [[nodiscard]] constexpr double risingRate() const noexcept
    {
        return mRisingRate;
    }

    /// @brief Returns maximum falling rate limit.
    /// @return Falling rate in units per second.
    [[nodiscard]] constexpr double fallingRate() const noexcept
    {
        return mFallingRate;
    }

    /// @brief Checks whether the limiter has been initialized.
    /// @return True if initialized with a sample or explicitly reset with value.
    [[nodiscard]] constexpr bool isInitialized() const noexcept
    {
        return mInitialized;
    }

private:
    double mRisingRate { 10.0 };
    double mFallingRate { 10.0 };
    double mCurrentVal { 0.0 };
    bool mInitialized { false };
};

} // namespace vehicle_sim::math
