#include <gtest/gtest.h>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/TireThermalModel.h"

using namespace vehicle_sim::dynamics;

TEST(TireThermalModelTest, AmbientEquilibrium_StationaryWheel)
{
    TireThermalConfig config {};
    TireThermalModel model { config };
    model.reset(25.0);

    // Advance 500 steps with zero loads and zero slip
    for (int i { 0 }; i < 500; ++i) {
        model.update(0.0, 0.0, 4000.0, 0.0, 0.0, 0.0, 0.32, 0.0, 25.0, 25.0, 0.016);
    }

    EXPECT_NEAR(model.getSurfaceTemp(), 25.0, 1e-2);
    EXPECT_NEAR(model.getCarcassTemp(), 25.0, 1e-2);
    EXPECT_NEAR(model.getCavityPressure(), 2.0, 1e-2);
}

TEST(TireThermalModelTest, PureScrubHeating_TemperatureIncreases)
{
    TireThermalConfig config {};
    TireThermalModel model { config };
    model.reset(25.0);

    // Apply high sliding slip velocity under 5000 N normal load
    constexpr double normalLoad { 5000.0 };
    constexpr double slipVel { 8.0 }; // 8 m/s sliding scrub
    constexpr double tractiveForce { 4500.0 };

    for (int i { 0 }; i < 200; ++i) {
        model.update(tractiveForce, 0.0, normalLoad, slipVel, 0.0, 30.0, 0.32, 10.0, 25.0, 25.0, 0.016);
    }

    EXPECT_GT(model.getSurfaceTemp(), 75.0);
    EXPECT_GT(model.getCarcassTemp(), 35.0);
    EXPECT_GT(model.getState().frictionPower, 1000.0);
}

TEST(TireThermalModelTest, CarcassHysteresisHeating_ZeroSlip)
{
    TireThermalConfig config {};
    TireThermalModel model { config };
    model.reset(25.0);

    // Pure rolling without slip: slipVel = 0, high load and high speed
    constexpr double normalLoad { 8000.0 };
    constexpr double angularSpeed { 90.0 }; // ~100 km/h
    constexpr double wheelRadius { 0.32 };

    for (int i { 0 }; i < 500; ++i) {
        model.update(0.0, 0.0, normalLoad, 0.0, 0.0, angularSpeed, wheelRadius, 28.0, 25.0, 25.0, 0.016);
    }

    // Carcass temperature must warm up due to hysteresis dissipation
    EXPECT_GT(model.getCarcassTemp(), 27.0);
    EXPECT_GT(model.getState().hysteresisPower, 100.0);
}

TEST(TireThermalModelTest, NonLinearGripCharacteristic_PeakAndDrop)
{
    TireThermalConfig config {};
    config.optimalTempMin = 75.0;
    config.optimalTempMax = 95.0;
    config.maxThermalDecay = 0.35;
    config.coldGripDeficit = 0.20;

    TireThermalModel model { config };

    // Cold regime: grip reduced
    const double coldGrip { model.evaluateGripFactor(20.0) };
    EXPECT_LT(coldGrip, 0.85);

    // Optimal window: peak grip
    const double optGrip1 { model.evaluateGripFactor(80.0) };
    const double optGrip2 { model.evaluateGripFactor(90.0) };
    EXPECT_NEAR(optGrip1, 1.0, 0.02);
    EXPECT_NEAR(optGrip2, 1.0, 0.02);

    // Overheated regime: grip degraded
    const double hotGrip { model.evaluateGripFactor(150.0) };
    EXPECT_LT(hotGrip, 0.70);
    EXPECT_GT(hotGrip, 0.60);
}

TEST(TireThermalModelTest, CavityPressure_IdealGasExpansion)
{
    TireThermalConfig config {};
    config.baseInflationPressure = 2.0;
    config.referenceColdTemp = 25.0; // 298.15 K

    TireThermalModel model { config };
    model.reset(25.0);

    // Heat carcass and cavity through prolonged intense scrub
    for (int i { 0 }; i < 1000; ++i) {
        model.update(6000.0, 3000.0, 6000.0, 10.0, 5.0, 40.0, 0.32, 5.0, 25.0, 25.0, 0.016);
    }

    const double cavityT { model.getState().cavityTemp };
    const double expectedPressure { 2.0 * ((cavityT + 273.15) / 298.15) };
    EXPECT_NEAR(model.getCavityPressure(), expectedPressure, 0.05);
    EXPECT_GT(model.getCavityPressure(), 2.05);
}

TEST(TireThermalModelTest, ConvectiveAirspeedCooling)
{
    TireThermalConfig config {};
    TireThermalModel modelSlow { config };
    TireThermalModel modelFast { config };
    modelSlow.reset(25.0);
    modelFast.reset(25.0);

    // Run identical heat input: one with zero airspeed, one with 50 m/s airspeed
    for (int i { 0 }; i < 300; ++i) {
        modelSlow.update(3000.0, 1000.0, 4000.0, 4.0, 2.0, 20.0, 0.32, 0.0, 25.0, 25.0, 0.016);
        modelFast.update(3000.0, 1000.0, 4000.0, 4.0, 2.0, 20.0, 0.32, 50.0, 25.0, 25.0, 0.016);
    }

    // Faster moving air must result in lower surface and carcass temperatures
    EXPECT_LT(modelFast.getSurfaceTemp(), modelSlow.getSurfaceTemp());
    EXPECT_LT(modelFast.getCarcassTemp(), modelSlow.getCarcassTemp());
}
