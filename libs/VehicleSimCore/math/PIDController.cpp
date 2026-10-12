#include "libs/VehicleSimCore/math/PIDController.h"

namespace vehicle_sim::math {

PIDController::PIDController(double kp, double ki, double kd) noexcept
    : mGains { kp, ki, kd }
{
}

PIDController::PIDController(const PIDGains& gains) noexcept
    : mGains { gains }
{
}

double PIDController::update(double setpoint, double measurement, double dt) noexcept
{
    if (dt <= 0.0) {
        return mLastOutput;
    }

    const double currentError { setpoint - measurement };
    const double pTerm { mGains.kp * currentError };

    if (!mInitialized) {
        mPrevError = currentError;
        mPrevMeasurement = measurement;
        mFilteredDerivative = 0.0;
        mIntegral = 0.0;
        mInitialized = true;
        mLastOutput = std::clamp(pTerm, mOutputMin, mOutputMax);
        return mLastOutput;
    }

    // 1. Derivative calculation
    double rate { 0.0 };
    if (mDerivativeOnMeasurement) {
        // Derivative on measurement: -d(measurement) / dt prevents setpoint step derivative kicks
        rate = -(measurement - mPrevMeasurement) / dt;
    } else {
        // Standard derivative on error: d(error) / dt
        rate = (currentError - mPrevError) / dt;
    }

    const double rawDerivative { mGains.kd * rate };
    if (mDerivativeFilterTf > 0.0) {
        const double alpha { std::clamp(dt / (mDerivativeFilterTf + dt), 0.0, 1.0) };
        mFilteredDerivative += alpha * (rawDerivative - mFilteredDerivative);
    } else {
        mFilteredDerivative = rawDerivative;
    }
    const double dTerm { mFilteredDerivative };

    // 2. Integral calculation with trapezoidal rule & anti-windup clamping
    const double deltaIntegral { mGains.ki * 0.5 * (currentError + mPrevError) * dt };
    const double candidateIntegral { std::clamp(mIntegral + deltaIntegral, mIntegralMin, mIntegralMax) };

    // Conditional integration (anti-windup clamping on actuator saturation)
    const double candidateOutput { pTerm + candidateIntegral + dTerm };
    const bool saturatedHigh { (candidateOutput > mOutputMax) && (deltaIntegral > 0.0) };
    const bool saturatedLow { (candidateOutput < mOutputMin) && (deltaIntegral < 0.0) };

    if (!saturatedHigh && !saturatedLow) {
        mIntegral = candidateIntegral;
    }

    const double unconstrainedOutput { pTerm + mIntegral + dTerm };
    mLastOutput = std::clamp(unconstrainedOutput, mOutputMin, mOutputMax);

    mPrevError = currentError;
    mPrevMeasurement = measurement;

    return mLastOutput;
}

void PIDController::reset() noexcept
{
    mIntegral = 0.0;
    mPrevError = 0.0;
    mPrevMeasurement = 0.0;
    mFilteredDerivative = 0.0;
    mLastOutput = 0.0;
    mInitialized = false;
}

void PIDController::reset(double initialIntegral) noexcept
{
    mIntegral = std::clamp(initialIntegral, mIntegralMin, mIntegralMax);
    mPrevError = 0.0;
    mPrevMeasurement = 0.0;
    mFilteredDerivative = 0.0;
    mLastOutput = 0.0;
    mInitialized = false;
}

void PIDController::setGains(double kp, double ki, double kd) noexcept
{
    mGains.kp = kp;
    mGains.ki = ki;
    mGains.kd = kd;
}

void PIDController::setGains(const PIDGains& gains) noexcept
{
    mGains = gains;
}

void PIDController::setOutputLimits(double minVal, double maxVal) noexcept
{
    if (minVal <= maxVal) {
        mOutputMin = minVal;
        mOutputMax = maxVal;
    } else {
        mOutputMin = maxVal;
        mOutputMax = minVal;
    }
}

void PIDController::setIntegralLimits(double minVal, double maxVal) noexcept
{
    if (minVal <= maxVal) {
        mIntegralMin = minVal;
        mIntegralMax = maxVal;
    } else {
        mIntegralMin = maxVal;
        mIntegralMax = minVal;
    }
    mIntegral = std::clamp(mIntegral, mIntegralMin, mIntegralMax);
}

void PIDController::setDerivativeFilter(double filterTimeConstant) noexcept
{
    mDerivativeFilterTf = (filterTimeConstant > 0.0) ? filterTimeConstant : 0.0;
}

void PIDController::setDerivativeOnMeasurement(bool enable) noexcept
{
    mDerivativeOnMeasurement = enable;
}

} // namespace vehicle_sim::math
