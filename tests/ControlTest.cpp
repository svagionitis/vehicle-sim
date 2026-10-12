#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/math/LowPassFilter.h"
#include "libs/VehicleSimCore/math/PIDController.h"
#include "libs/VehicleSimCore/math/SlewRateLimiter.h"
#include "libs/VehicleSimCore/math/KalmanFilter1D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::KalmanFilter1D;
using vehicle_sim::math::LowPassFilter;
using vehicle_sim::math::PIDController;
using vehicle_sim::math::PIDGains;
using vehicle_sim::math::SlewRateLimiter;

inline constexpr double kEpsilon { 1e-7 };

// -----------------------------------------------------------------------------
// LowPassFilter Tests
// -----------------------------------------------------------------------------

TEST(LowPassFilterTest, DefaultConstruction)
{
    const LowPassFilter filter {};
    EXPECT_DOUBLE_EQ(filter.cutoff(), 10.0);
    EXPECT_DOUBLE_EQ(filter.value(), 0.0);
    EXPECT_FALSE(filter.isInitialized());
}

TEST(LowPassFilterTest, ParameterizedConstruction)
{
    const LowPassFilter filter { 25.0 };
    EXPECT_DOUBLE_EQ(filter.cutoff(), 25.0);
    EXPECT_DOUBLE_EQ(filter.value(), 0.0);
    EXPECT_FALSE(filter.isInitialized());

    const LowPassFilter invalidFilter { -5.0 };
    EXPECT_DOUBLE_EQ(invalidFilter.cutoff(), 0.0);
}

TEST(LowPassFilterTest, InitialSamplePassThrough)
{
    LowPassFilter filter { 5.0 };
    const double output { filter.update(42.0, 0.01) };
    EXPECT_TRUE(filter.isInitialized());
    EXPECT_DOUBLE_EQ(output, 42.0);
    EXPECT_DOUBLE_EQ(filter.value(), 42.0);
}

TEST(LowPassFilterTest, ConvergenceToConstantInput)
{
    LowPassFilter filter { 10.0 };
    filter.reset(0.0);
    EXPECT_TRUE(filter.isInitialized());

    const double target { 100.0 };
    const double dt { 0.01 };

    // Simulate for 1.0 second (100 steps)
    for (int i { 0 }; i < 100; ++i) {
        static_cast<void>(filter.update(target, dt));
    }

    EXPECT_NEAR(filter.value(), target, 1e-3);
}

TEST(LowPassFilterTest, ZeroAndNegativeDtPreservesState)
{
    LowPassFilter filter { 10.0 };
    filter.reset(50.0);

    EXPECT_DOUBLE_EQ(filter.update(100.0, 0.0), 50.0);
    EXPECT_DOUBLE_EQ(filter.update(100.0, -0.05), 50.0);
    EXPECT_DOUBLE_EQ(filter.value(), 50.0);
}

TEST(LowPassFilterTest, ZeroCutoffHoldsState)
{
    LowPassFilter filter { 0.0 };
    filter.reset(20.0);

    const double output { filter.update(80.0, 0.1) };
    EXPECT_DOUBLE_EQ(output, 20.0);
    EXPECT_DOUBLE_EQ(filter.value(), 20.0);
}

TEST(LowPassFilterTest, ResetFunctionality)
{
    LowPassFilter filter { 10.0 };
    static_cast<void>(filter.update(50.0, 0.01));
    EXPECT_TRUE(filter.isInitialized());

    filter.reset();
    EXPECT_FALSE(filter.isInitialized());
    EXPECT_DOUBLE_EQ(filter.value(), 0.0);

    filter.reset(15.0);
    EXPECT_TRUE(filter.isInitialized());
    EXPECT_DOUBLE_EQ(filter.value(), 15.0);
}

TEST(LowPassFilterTest, DynamicCutoffAdjustment)
{
    LowPassFilter filter { 5.0 };
    EXPECT_DOUBLE_EQ(filter.cutoff(), 5.0);

    filter.setCutoff(20.0);
    EXPECT_DOUBLE_EQ(filter.cutoff(), 20.0);

    filter.setCutoff(-1.0);
    EXPECT_DOUBLE_EQ(filter.cutoff(), 0.0);
}

// -----------------------------------------------------------------------------
// SlewRateLimiter Tests
// -----------------------------------------------------------------------------

TEST(SlewRateLimiterTest, DefaultConstruction)
{
    const SlewRateLimiter limiter {};
    EXPECT_DOUBLE_EQ(limiter.risingRate(), 10.0);
    EXPECT_DOUBLE_EQ(limiter.fallingRate(), 10.0);
    EXPECT_DOUBLE_EQ(limiter.value(), 0.0);
    EXPECT_FALSE(limiter.isInitialized());
}

TEST(SlewRateLimiterTest, ParameterizedConstructors)
{
    const SlewRateLimiter symLimiter { 25.0 };
    EXPECT_DOUBLE_EQ(symLimiter.risingRate(), 25.0);
    EXPECT_DOUBLE_EQ(symLimiter.fallingRate(), 25.0);

    const SlewRateLimiter asymLimiter { 30.0, 15.0 };
    EXPECT_DOUBLE_EQ(asymLimiter.risingRate(), 30.0);
    EXPECT_DOUBLE_EQ(asymLimiter.fallingRate(), 15.0);

    const SlewRateLimiter invalidLimiter { -5.0, -10.0 };
    EXPECT_DOUBLE_EQ(invalidLimiter.risingRate(), 0.0);
    EXPECT_DOUBLE_EQ(invalidLimiter.fallingRate(), 0.0);
}

TEST(SlewRateLimiterTest, InitialSamplePassThrough)
{
    SlewRateLimiter limiter { 5.0 };
    const double output { limiter.update(50.0, 0.01) };
    EXPECT_TRUE(limiter.isInitialized());
    EXPECT_DOUBLE_EQ(output, 50.0);
    EXPECT_DOUBLE_EQ(limiter.value(), 50.0);
}

TEST(SlewRateLimiterTest, RisingRateLimiting)
{
    SlewRateLimiter limiter { 2.0 }; // 2.0 units per second
    limiter.reset(0.0);

    // Target = 10.0, dt = 0.5s -> maxRise = 1.0 per step
    double val { limiter.update(10.0, 0.5) };
    EXPECT_NEAR(val, 1.0, kEpsilon);

    val = limiter.update(10.0, 0.5);
    EXPECT_NEAR(val, 2.0, kEpsilon);

    // Continue for 4 more seconds (8 steps) -> total time = 5.0s, exactly reaches 10.0
    for (int i { 0 }; i < 8; ++i) {
        val = limiter.update(10.0, 0.5);
    }
    EXPECT_NEAR(val, 10.0, kEpsilon);

    // Further updates stay at target
    val = limiter.update(10.0, 0.5);
    EXPECT_DOUBLE_EQ(val, 10.0);
}

TEST(SlewRateLimiterTest, FallingRateLimiting)
{
    SlewRateLimiter limiter { 4.0 }; // 4.0 units per second
    limiter.reset(10.0);

    // Target = 0.0, dt = 0.5s -> maxFall = 2.0 per step
    double val { limiter.update(0.0, 0.5) };
    EXPECT_NEAR(val, 8.0, kEpsilon);

    val = limiter.update(0.0, 0.5);
    EXPECT_NEAR(val, 6.0, kEpsilon);

    for (int i { 0 }; i < 3; ++i) {
        val = limiter.update(0.0, 0.5);
    }
    EXPECT_NEAR(val, 0.0, kEpsilon);
}

TEST(SlewRateLimiterTest, AsymmetricRisingAndFallingRates)
{
    SlewRateLimiter limiter { 10.0, 2.0 }; // Fast rise (10/s), slow fall (2/s)
    limiter.reset(0.0);

    // Rise step (dt = 0.1s -> rise delta = 1.0)
    double val { limiter.update(10.0, 0.1) };
    EXPECT_NEAR(val, 1.0, kEpsilon);

    // Jump to 10.0 explicitly
    limiter.reset(10.0);

    // Fall step (dt = 0.1s -> fall delta = 0.2)
    val = limiter.update(0.0, 0.1);
    EXPECT_NEAR(val, 9.8, kEpsilon);
}

TEST(SlewRateLimiterTest, SubStepConvergenceWithoutOvershoot)
{
    SlewRateLimiter limiter { 10.0 };
    limiter.reset(5.0);

    // Target = 5.3, max step is 1.0 (dt = 0.1s)
    const double val { limiter.update(5.3, 0.1) };
    EXPECT_DOUBLE_EQ(val, 5.3);
}

TEST(SlewRateLimiterTest, ZeroAndNegativeDtPreservesValue)
{
    SlewRateLimiter limiter { 5.0 };
    limiter.reset(12.0);

    EXPECT_DOUBLE_EQ(limiter.update(50.0, 0.0), 12.0);
    EXPECT_DOUBLE_EQ(limiter.update(50.0, -0.1), 12.0);
}

TEST(SlewRateLimiterTest, ResetAndRateAdjustment)
{
    SlewRateLimiter limiter { 5.0 };
    static_cast<void>(limiter.update(20.0, 0.1));
    EXPECT_TRUE(limiter.isInitialized());

    limiter.reset();
    EXPECT_FALSE(limiter.isInitialized());
    EXPECT_DOUBLE_EQ(limiter.value(), 0.0);

    limiter.reset(7.5);
    EXPECT_TRUE(limiter.isInitialized());
    EXPECT_DOUBLE_EQ(limiter.value(), 7.5);

    limiter.setRateLimit(15.0);
    EXPECT_DOUBLE_EQ(limiter.risingRate(), 15.0);
    EXPECT_DOUBLE_EQ(limiter.fallingRate(), 15.0);

    limiter.setRateLimits(20.0, 8.0);
    EXPECT_DOUBLE_EQ(limiter.risingRate(), 20.0);
    EXPECT_DOUBLE_EQ(limiter.fallingRate(), 8.0);
}

// -----------------------------------------------------------------------------
// PIDController Tests
// -----------------------------------------------------------------------------

TEST(PIDControllerTest, DefaultConstruction)
{
    const PIDController pid {};
    EXPECT_DOUBLE_EQ(pid.kp(), 1.0);
    EXPECT_DOUBLE_EQ(pid.ki(), 0.0);
    EXPECT_DOUBLE_EQ(pid.kd(), 0.0);
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.derivative(), 0.0);
    EXPECT_FALSE(pid.isInitialized());
    EXPECT_TRUE(pid.isDerivativeOnMeasurement());
}

TEST(PIDControllerTest, ParameterizedConstructors)
{
    const PIDController pid { 2.5, 0.8, 0.15 };
    EXPECT_DOUBLE_EQ(pid.kp(), 2.5);
    EXPECT_DOUBLE_EQ(pid.ki(), 0.8);
    EXPECT_DOUBLE_EQ(pid.kd(), 0.15);

    const PIDGains gains { 3.0, 1.2, 0.4 };
    const PIDController pidFromBundle { gains };
    EXPECT_DOUBLE_EQ(pidFromBundle.kp(), 3.0);
    EXPECT_DOUBLE_EQ(pidFromBundle.ki(), 1.2);
    EXPECT_DOUBLE_EQ(pidFromBundle.kd(), 0.4);
}

TEST(PIDControllerTest, ProportionalOnlyAction)
{
    PIDController pid { 2.0, 0.0, 0.0 };

    // Setpoint = 10.0, measurement = 6.0 -> error = 4.0 -> output = 8.0
    const double u1 { pid.update(10.0, 6.0, 0.01) };
    EXPECT_DOUBLE_EQ(u1, 8.0);
    EXPECT_DOUBLE_EQ(pid.error(), 4.0);

    // Next step: Setpoint = 10.0, measurement = 9.0 -> error = 1.0 -> output = 2.0
    const double u2 { pid.update(10.0, 9.0, 0.01) };
    EXPECT_DOUBLE_EQ(u2, 2.0);
    EXPECT_DOUBLE_EQ(pid.error(), 1.0);
}

TEST(PIDControllerTest, IntegralAccumulationTrapezoidal)
{
    PIDController pid { 0.0, 2.0, 0.0 };

    // Initial step initializes previous error to 5.0
    const double u1 { pid.update(10.0, 5.0, 0.1) };
    EXPECT_DOUBLE_EQ(u1, 0.0); // No integral on 1st step initialization

    // Step 2: constant error = 5.0 across dt = 0.1s
    // Trapezoidal: delta = Ki * 0.5 * (5.0 + 5.0) * 0.1 = 2.0 * 5.0 * 0.1 = 1.0
    const double u2 { pid.update(10.0, 5.0, 0.1) };
    EXPECT_NEAR(u2, 1.0, kEpsilon);
    EXPECT_NEAR(pid.integral(), 1.0, kEpsilon);

    // Step 3: same error, integral increases by another 1.0 to 2.0
    const double u3 { pid.update(10.0, 5.0, 0.1) };
    EXPECT_NEAR(u3, 2.0, kEpsilon);
    EXPECT_NEAR(pid.integral(), 2.0, kEpsilon);
}

TEST(PIDControllerTest, AntiWindupIntegralLimitsClamping)
{
    PIDController pid { 0.0, 5.0, 0.0 };
    pid.setIntegralLimits(-3.0, 3.0);

    // Initial sample
    static_cast<void>(pid.update(10.0, 0.0, 0.1));

    // Accumulate error = 10.0 over 20 steps (dt = 0.1s)
    // Without clamp, integral would be 5.0 * 10.0 * 0.1 * 20 = 100.0
    for (int i { 0 }; i < 20; ++i) {
        static_cast<void>(pid.update(10.0, 0.0, 0.1));
    }

    EXPECT_DOUBLE_EQ(pid.integral(), 3.0);
}

TEST(PIDControllerTest, OutputSaturationAndConditionalIntegration)
{
    PIDController pid { 1.0, 1.0, 0.0 };
    pid.setOutputLimits(-10.0, 10.0);

    // Initial sample
    static_cast<void>(pid.update(50.0, 0.0, 0.1));

    // Error = 50.0. P term alone = 50.0, which exceeds max output 10.0!
    // Conditional integration should prevent integral from accumulating.
    for (int i { 0 }; i < 10; ++i) {
        const double u { pid.update(50.0, 0.0, 0.1) };
        EXPECT_DOUBLE_EQ(u, 10.0);
    }

    // Integral accumulator remains 0.0 because output is saturated in the error direction
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(PIDControllerTest, DerivativeOnMeasurementNoKickOnSetpointStep)
{
    PIDController pid { 1.0, 0.0, 2.0 };
    pid.setDerivativeOnMeasurement(true);
    pid.setDerivativeFilter(0.0); // Instantaneous unfiltered derivative

    // Step 1: steady state at 0
    static_cast<void>(pid.update(0.0, 0.0, 0.01));

    // Step 2: Setpoint suddenly jumps from 0.0 to 100.0, while measurement remains 0.0
    const double u { pid.update(100.0, 0.0, 0.01) };

    // Since measurement didn't change (dy = 0), derivative on measurement is 0.0!
    // Output should be pure P term: 1.0 * 100.0 = 100.0 (no derivative kick Dirac pulse)
    EXPECT_DOUBLE_EQ(pid.derivative(), 0.0);
    EXPECT_DOUBLE_EQ(u, 100.0);
}

TEST(PIDControllerTest, DerivativeOnErrorRespondsToSetpointStep)
{
    PIDController pid { 1.0, 0.0, 2.0 };
    pid.setDerivativeOnMeasurement(false); // Derivative on error
    pid.setDerivativeFilter(0.0);

    // Step 1: steady state at 0
    static_cast<void>(pid.update(0.0, 0.0, 0.01));

    // Step 2: Setpoint jumps from 0 to 10.0, error changes by 10.0 in dt = 0.01s
    // dError / dt = 10.0 / 0.01 = 1000.0 -> Kd * rate = 2000.0
    const double u { pid.update(10.0, 0.0, 0.01) };

    EXPECT_NEAR(pid.derivative(), 2000.0, kEpsilon);
    EXPECT_NEAR(u, 10.0 + 2000.0, kEpsilon);
}

TEST(PIDControllerTest, LowPassDerivativeFilterSmoothing)
{
    PIDController pid { 0.0, 0.0, 1.0 };
    pid.setDerivativeOnMeasurement(false);
    pid.setDerivativeFilter(0.1); // Tf = 0.1s

    static_cast<void>(pid.update(0.0, 0.0, 0.01));

    // Error steps by 1.0 in dt = 0.01s -> raw derivative = 100.0
    // Alpha = dt / (Tf + dt) = 0.01 / 0.11 ~= 0.0909
    // Filtered derivative = 0 + 0.0909 * 100.0 ~= 9.09 (smoothed down by factor of ~11)
    const double u { pid.update(1.0, 0.0, 0.01) };

    EXPECT_LT(u, 15.0);
    EXPECT_GT(u, 5.0);
}

TEST(PIDControllerTest, ZeroAndNegativeDtPreservesLastOutput)
{
    PIDController pid { 2.0, 0.0, 0.0 };
    const double uInit { pid.update(10.0, 5.0, 0.01) };
    EXPECT_DOUBLE_EQ(uInit, 10.0);

    EXPECT_DOUBLE_EQ(pid.update(100.0, 0.0, 0.0), 10.0);
    EXPECT_DOUBLE_EQ(pid.update(100.0, 0.0, -0.05), 10.0);
}

TEST(PIDControllerTest, ResetAndGainUpdates)
{
    PIDController pid { 1.0, 1.0, 1.0 };
    static_cast<void>(pid.update(10.0, 0.0, 0.1));
    EXPECT_TRUE(pid.isInitialized());

    pid.reset();
    EXPECT_FALSE(pid.isInitialized());
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.derivative(), 0.0);

    pid.reset(5.0);
    EXPECT_DOUBLE_EQ(pid.integral(), 5.0);

    pid.setGains(3.0, 2.0, 0.5);
    EXPECT_DOUBLE_EQ(pid.kp(), 3.0);
    EXPECT_DOUBLE_EQ(pid.ki(), 2.0);
    EXPECT_DOUBLE_EQ(pid.kd(), 0.5);

    const PIDGains newGains { 4.0, 0.2, 0.1 };
    pid.setGains(newGains);
    EXPECT_DOUBLE_EQ(pid.kp(), 4.0);
    EXPECT_DOUBLE_EQ(pid.ki(), 0.2);
    EXPECT_DOUBLE_EQ(pid.kd(), 0.1);
}

// -----------------------------------------------------------------------------
// KalmanFilter1D Tests
// -----------------------------------------------------------------------------

TEST(KalmanFilter1DTest, DefaultConstruction)
{
    const KalmanFilter1D kf {};
    EXPECT_DOUBLE_EQ(kf.position(), 0.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 0.0);
    EXPECT_DOUBLE_EQ(kf.positionVariance(), 1.0);
    EXPECT_DOUBLE_EQ(kf.velocityVariance(), 1.0);
    EXPECT_FALSE(kf.isInitialized());
}

TEST(KalmanFilter1DTest, ParameterizedConstruction)
{
    const KalmanFilter1D kf { 10.0, 5.0, 0.2, 0.5 };
    EXPECT_DOUBLE_EQ(kf.position(), 10.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 5.0);
    EXPECT_TRUE(kf.isInitialized());
}

TEST(KalmanFilter1DTest, PredictDeadReckoning)
{
    KalmanFilter1D kf { 0.0, 10.0, 0.1, 1.0 };
    kf.predict(0.5); // dt = 0.5s -> x = 0 + 10 * 0.5 = 5.0
    EXPECT_DOUBLE_EQ(kf.position(), 5.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 10.0);
    EXPECT_GT(kf.positionVariance(), 1.0);
}

TEST(KalmanFilter1DTest, PredictWithControlAcceleration)
{
    KalmanFilter1D kf { 0.0, 0.0, 0.1, 1.0 };
    // Acceleration a = 2.0 m/s^2, dt = 1.0s -> x = 0.5 * 2 * 1 = 1.0, v = 2.0
    kf.predict(1.0, 2.0);
    EXPECT_DOUBLE_EQ(kf.position(), 1.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 2.0);
}

TEST(KalmanFilter1DTest, PositionUpdateReducesVariance)
{
    KalmanFilter1D kf { 0.0, 0.0, 0.1, 2.0 };
    const double priorVariance { kf.positionVariance() };
    kf.updatePosition(1.0);
    EXPECT_LT(kf.positionVariance(), priorVariance);
    EXPECT_GT(kf.position(), 0.0);
}

TEST(KalmanFilter1DTest, VelocityEstimationFromPositionOnly)
{
    KalmanFilter1D kf { 0.0, 0.0, 0.5, 0.1 };

    // Vehicle moving at constant true velocity 5.0 m/s (dt = 0.1s -> dx = 0.5m)
    for (int i { 1 }; i <= 30; ++i) {
        const double truePos { static_cast<double>(i) * 0.5 };
        static_cast<void>(kf.update(truePos, 0.1));
    }

    EXPECT_NEAR(kf.velocity(), 5.0, 0.25);
}

TEST(KalmanFilter1DTest, VelocityUpdate)
{
    KalmanFilter1D kf { 0.0, 0.0, 0.1, 1.0 };
    kf.updateVelocity(12.0, 0.5);
    EXPECT_GT(kf.velocity(), 0.0);
    EXPECT_LT(kf.velocityVariance(), 1.0);
}

TEST(KalmanFilter1DTest, NoiseRejectionSmoothing)
{
    KalmanFilter1D kf { 10.0, 0.0, 0.01, 1.0 };

    // Alternating noisy readings: 11.0, 9.0, 11.0, 9.0 (true position = 10.0)
    double estimatedPos { 10.0 };
    for (int i { 0 }; i < 30; ++i) {
        const double noisy { (i % 2 == 0) ? 11.0 : 9.0 };
        estimatedPos = kf.update(noisy, 0.1);
    }

    EXPECT_NEAR(estimatedPos, 10.0, 0.25);
}

TEST(KalmanFilter1DTest, ZeroDtPreservesState)
{
    KalmanFilter1D kf { 5.0, 2.0 };
    kf.predict(0.0);
    EXPECT_DOUBLE_EQ(kf.position(), 5.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 2.0);

    kf.predict(-0.1);
    EXPECT_DOUBLE_EQ(kf.position(), 5.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 2.0);
}

TEST(KalmanFilter1DTest, ResetFunctionality)
{
    KalmanFilter1D kf { 10.0, 5.0 };
    kf.reset(2.0, 1.0, 0.5, 0.5);
    EXPECT_DOUBLE_EQ(kf.position(), 2.0);
    EXPECT_DOUBLE_EQ(kf.velocity(), 1.0);
    EXPECT_DOUBLE_EQ(kf.positionVariance(), 0.5);
    EXPECT_DOUBLE_EQ(kf.velocityVariance(), 0.5);
    EXPECT_FALSE(kf.isInitialized());
}

} // namespace vehicle_sim::testing
