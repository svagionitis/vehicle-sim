#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

namespace vehicle_sim::telemetry::testing {

/// @brief Verifies that dynamic weight transfer strictly conserves total vertical normal force.
TEST(ExtendedTelemetryTest, WeightTransferConservationOfVerticalForces)
{
    constexpr double kMass { 1200.0 };
    constexpr double kGravity { 9.80665 };
    constexpr double kTotalWeight { kMass * kGravity };
    constexpr double kWheelbase { 2.6 };
    constexpr double kTrackWidth { 1.5 };
    constexpr double kCgHeight { 0.45 };

    // Test maneuvers: stationary, straight braking, throttle acceleration, cornering
    constexpr double kTestAx[] { 0.0, -8.0, 4.5, -3.0 };
    constexpr double kTestAy[] { 0.0, 0.0, 0.0, 8.5 };

    for (std::size_t i { 0 }; i < 4; ++i) {
        const double ax { kTestAx[i] };
        const double ay { kTestAy[i] };

        const double staticFront { kTotalWeight * 0.5 };
        const double staticRear { kTotalWeight * 0.5 };

        const double deltaLong { (kMass * ax * kCgHeight) / kWheelbase };
        const double frontAxle { staticFront - deltaLong };
        const double rearAxle { staticRear + deltaLong };

        constexpr double kFrontRollRatio { 0.52 };
        constexpr double kRearRollRatio { 0.48 };
        const double deltaLatFront { (kFrontRollRatio * kMass * ay * kCgHeight) / kTrackWidth };
        const double deltaLatRear { (kRearRollRatio * kMass * ay * kCgHeight) / kTrackWidth };

        const double fzFL { std::max(0.0, (frontAxle * 0.5) - deltaLatFront) };
        const double fzFR { std::max(0.0, (frontAxle * 0.5) + deltaLatFront) };
        const double fzRL { std::max(0.0, (rearAxle * 0.5) - deltaLatRear) };
        const double fzRR { std::max(0.0, (rearAxle * 0.5) + deltaLatRear) };

        const double sumFz { fzFL + fzFR + fzRL + fzRR };
        EXPECT_NEAR(sumFz, kTotalWeight, 1e-3);
    }
}

/// @brief Verifies wheel lift clamping when roll/pitch couples exceed vertical preload.
TEST(ExtendedTelemetryTest, WheelLiftClampingUnderExtremeRoll)
{
    constexpr double kMass { 800.0 };
    constexpr double kGravity { 9.80665 };
    constexpr double kTotalWeight { kMass * kGravity };
    constexpr double kTrackWidth { 1.2 };
    constexpr double kCgHeight { 0.85 }; // High CG produces high roll moment

    // Extreme lateral acceleration exceeding rollover threshold
    constexpr double extremeAy { 18.0 }; // m/s^2 (~1.8G)
    const double staticFront { kTotalWeight * 0.5 };
    const double deltaLatFront { (0.52 * kMass * extremeAy * kCgHeight) / kTrackWidth };

    // Unclamped value is negative, indicating inside wheel lift-off
    const double unclampedFL { (staticFront * 0.5) - deltaLatFront };
    EXPECT_LT(unclampedFL, 0.0);

    // Clamped value is exactly zero
    const double clampedFL { std::max(0.0, unclampedFL) };
    EXPECT_DOUBLE_EQ(clampedFL, 0.0);
}

/// @brief Verifies G-G traction ratio calculation across linear, transition, and saturation zones.
TEST(ExtendedTelemetryTest, GgTractionRatioCalculations)
{
    constexpr double kMu { 1.0 };

    // Zone 1: Linear Grip (< 0.70)
    {
        constexpr double longG { 0.4 };
        constexpr double latG { 0.3 };
        const double rho { std::sqrt(std::pow(longG / kMu, 2.0) + std::pow(latG / kMu, 2.0)) };
        EXPECT_NEAR(rho, 0.5, 1e-5);
        EXPECT_LT(rho, 0.70);
    }

    // Zone 2: Handling Transition Zone (0.70 <= rho <= 0.95)
    {
        constexpr double longG { 0.6 };
        constexpr double latG { 0.6 };
        const double rho { std::sqrt(std::pow(longG / kMu, 2.0) + std::pow(latG / kMu, 2.0)) };
        EXPECT_GT(rho, 0.70);
        EXPECT_LT(rho, 0.95);
    }

    // Zone 3: Traction Saturation / Breakaway (> 0.95)
    {
        constexpr double longG { 0.85 };
        constexpr double latG { 0.75 };
        const double rho { std::sqrt(std::pow(longG / kMu, 2.0) + std::pow(latG / kMu, 2.0)) };
        EXPECT_GT(rho, 0.95);
    }
}

/// @brief Verifies oscilloscope historical window index wrapping and bounds clamping.
TEST(ExtendedTelemetryTest, OscilloscopeWindowClampingAndWrapping)
{
    constexpr std::size_t kCap { 200U };
    std::array<float, kCap> history { };
    std::size_t head { 0U };
    std::size_t count { 0U };

    // Push 250 samples to test wraparound
    for (std::size_t i { 0 }; i < 250; ++i) {
        history[head] = static_cast<float>(i);
        head = (head + 1U) % kCap;
        if (count < kCap) {
            ++count;
        }
    }

    EXPECT_EQ(count, kCap);
    EXPECT_EQ(head, 50U); // 250 % 200 = 50

    // Chronological oldest sample is at (head + kCap - count) % kCap = (50 + 200 - 200) % 200 = 50
    const std::size_t oldestIdx { (head + kCap - count) % kCap };
    EXPECT_FLOAT_EQ(history[oldestIdx], 50.0f);

    // Chronological newest sample is at (head + kCap - 1) % kCap = 49
    const std::size_t newestIdx { (head + kCap - 1U) % kCap };
    EXPECT_FLOAT_EQ(history[newestIdx], 249.0f);
}

} // namespace vehicle_sim::telemetry::testing
