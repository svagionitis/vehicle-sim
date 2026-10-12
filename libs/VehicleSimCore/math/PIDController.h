#pragma once

#include <algorithm>
#include <limits>

namespace vehicle_sim::math {

/// @struct PIDGains
/// @brief Parameter bundle holding tuning coefficients for PIDController.
struct PIDGains {
    double kp { 1.0 }; ///< Proportional gain.
    double ki { 0.0 }; ///< Integral gain.
    double kd { 0.0 }; ///< Derivative gain.
};

/// @class PIDController
/// @brief Proportional-Integral-Derivative feedback controller.
/// @details Features anti-windup conditional integration, bounded integral accumulator clamping,
/// low-pass derivative filtering, and derivative-on-measurement mode.
/// Pure C++17 implementation adhering strictly to IEEE 754 determinism with zero dynamic allocations.
class PIDController {
public:
    /// @brief Default constructor.
    constexpr PIDController() noexcept = default;

    /// @brief Constructs a PIDController with specified scalar gains.
    /// @param[in] kp Proportional gain.
    /// @param[in] ki Integral gain.
    /// @param[in] kd Derivative gain.
    PIDController(double kp, double ki, double kd) noexcept;

    /// @brief Constructs a PIDController with a gain bundle.
    /// @param[in] gains Tuning parameters struct.
    explicit PIDController(const PIDGains& gains) noexcept;

    /// @brief Computes control effort for the current time step.
    /// @param[in] setpoint Target reference value.
    /// @param[in] measurement Current measured process variable.
    /// @param[in] dt Elapsed time step in seconds.
    /// @return Clamped control output.
    [[nodiscard]] double update(double setpoint, double measurement, double dt) noexcept;

    /// @brief Clears integrator accumulator, derivative state, and previous error.
    void reset() noexcept;

    /// @brief Resets controller state with an explicit initial integral accumulator.
    /// @param[in] initialIntegral Initial value for the integral accumulator.
    void reset(double initialIntegral) noexcept;

    /// @brief Updates tuning gains.
    /// @param[in] kp Proportional gain.
    /// @param[in] ki Integral gain.
    /// @param[in] kd Derivative gain.
    void setGains(double kp, double ki, double kd) noexcept;

    /// @brief Updates tuning gains from bundle.
    /// @param[in] gains Tuning parameters struct.
    void setGains(const PIDGains& gains) noexcept;

    /// @brief Sets output saturation limits.
    /// @param[in] minVal Minimum control output.
    /// @param[in] maxVal Maximum control output.
    void setOutputLimits(double minVal, double maxVal) noexcept;

    /// @brief Sets integrator anti-windup clamping boundaries.
    /// @param[in] minVal Minimum integral term value.
    /// @param[in] maxVal Maximum integral term value.
    void setIntegralLimits(double minVal, double maxVal) noexcept;

    /// @brief Configures derivative filter time constant.
    /// @param[in] filterTimeConstant Filter time constant Tf in seconds (0.0 for unfiltered).
    void setDerivativeFilter(double filterTimeConstant) noexcept;

    /// @brief Enables or disables derivative calculation based on measurement instead of error.
    /// @param[in] enable True to compute derivative on measurement.
    void setDerivativeOnMeasurement(bool enable) noexcept;

    /// @brief Returns proportional gain.
    /// @return Kp gain.
    [[nodiscard]] constexpr double kp() const noexcept
    {
        return mGains.kp;
    }

    /// @brief Returns integral gain.
    /// @return Ki gain.
    [[nodiscard]] constexpr double ki() const noexcept
    {
        return mGains.ki;
    }

    /// @brief Returns derivative gain.
    /// @return Kd gain.
    [[nodiscard]] constexpr double kd() const noexcept
    {
        return mGains.kd;
    }

    /// @brief Returns tuning gains bundle.
    /// @return Reference to PIDGains struct.
    [[nodiscard]] constexpr const PIDGains& gains() const noexcept
    {
        return mGains;
    }

    /// @brief Returns minimum output limit.
    /// @return Minimum control output.
    [[nodiscard]] constexpr double outputMin() const noexcept
    {
        return mOutputMin;
    }

    /// @brief Returns maximum output limit.
    /// @return Maximum control output.
    [[nodiscard]] constexpr double outputMax() const noexcept
    {
        return mOutputMax;
    }

    /// @brief Returns minimum integral limit.
    /// @return Minimum integral value.
    [[nodiscard]] constexpr double integralMin() const noexcept
    {
        return mIntegralMin;
    }

    /// @brief Returns maximum integral limit.
    /// @return Maximum integral value.
    [[nodiscard]] constexpr double integralMax() const noexcept
    {
        return mIntegralMax;
    }

    /// @brief Returns current accumulated integral value.
    /// @return Stored integral term.
    [[nodiscard]] constexpr double integral() const noexcept
    {
        return mIntegral;
    }

    /// @brief Returns most recent filtered derivative term.
    /// @return Stored derivative term.
    [[nodiscard]] constexpr double derivative() const noexcept
    {
        return mFilteredDerivative;
    }

    /// @brief Returns most recent error value.
    /// @return Stored error (setpoint - measurement).
    [[nodiscard]] constexpr double error() const noexcept
    {
        return mPrevError;
    }

    /// @brief Returns derivative filter time constant Tf in seconds.
    /// @return Filter time constant.
    [[nodiscard]] constexpr double derivativeFilter() const noexcept
    {
        return mDerivativeFilterTf;
    }

    /// @brief Checks whether derivative is computed on measurement.
    /// @return True if derivative-on-measurement is active.
    [[nodiscard]] constexpr bool isDerivativeOnMeasurement() const noexcept
    {
        return mDerivativeOnMeasurement;
    }

    /// @brief Checks whether the controller has received at least one sample.
    /// @return True if initialized.
    [[nodiscard]] constexpr bool isInitialized() const noexcept
    {
        return mInitialized;
    }

private:
    PIDGains mGains { 1.0, 0.0, 0.0 };
    double mOutputMin { -std::numeric_limits<double>::infinity() };
    double mOutputMax { std::numeric_limits<double>::infinity() };
    double mIntegralMin { -std::numeric_limits<double>::infinity() };
    double mIntegralMax { std::numeric_limits<double>::infinity() };
    double mDerivativeFilterTf { 0.01 };

    double mIntegral { 0.0 };
    double mPrevError { 0.0 };
    double mPrevMeasurement { 0.0 };
    double mFilteredDerivative { 0.0 };
    double mLastOutput { 0.0 };
    bool mInitialized { false };
    bool mDerivativeOnMeasurement { true };
};

} // namespace vehicle_sim::math
