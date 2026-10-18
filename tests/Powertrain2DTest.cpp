#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "libs/VehicleSimCore/dynamics/Differential2D.h"
#include "libs/VehicleSimCore/dynamics/ElectricMotor2D.h"
#include "libs/VehicleSimCore/dynamics/FlywheelClutch2D.h"
#include "libs/VehicleSimCore/dynamics/InternalCombustionEngine2D.h"
#include "libs/VehicleSimCore/dynamics/Powertrain2D.h"
#include "libs/VehicleSimCore/dynamics/Transmission2D.h"

namespace vehicle_sim::dynamics {

TEST(Powertrain2DTest, ICETorqueCurveWOT) {
    ICEConfig cfg {};
    cfg.peakTorque = 450.0;
    cfg.peakTorqueRpm = 4800.0;
    InternalCombustionEngine2D ice { cfg };

    const double tPeak { ice.computeWotTorque(4800.0) };
    EXPECT_NEAR(tPeak, 450.0, 5.0);

    const double tIdle { ice.computeWotTorque(900.0) };
    EXPECT_GT(tIdle, 100.0);
    EXPECT_LT(tIdle, 450.0);

    const double tRedline { ice.computeWotTorque(7200.0) };
    EXPECT_GT(tRedline, 50.0);
    EXPECT_LT(tRedline, 450.0);
}

TEST(Powertrain2DTest, ICEIdleGovernor) {
    ICEConfig cfg {};
    cfg.idleRpm = 900.0;
    InternalCombustionEngine2D ice { cfg };

    constexpr double kBelowIdleSpeed { 80.0 }; // ~764 RPM
    constexpr double kDt { 0.01 };

    EngineOutput out {};
    for (int i { 0 }; i < 50; ++i) {
        out = ice.step(0.0, kBelowIdleSpeed, kDt);
    }

    EXPECT_GT(out.torque, 0.0);
    EXPECT_FALSE(out.isStalled);
    EXPECT_FALSE(out.isRedline);
}

TEST(Powertrain2DTest, ICERedlineFuelCut) {
    ICEConfig cfg {};
    cfg.redlineRpm = 7000.0;
    cfg.hysteresisRpm = 200.0;
    InternalCombustionEngine2D ice { cfg };

    constexpr double kOverRedlineSpeed { 750.0 }; // ~7162 RPM
    constexpr double kDt { 0.01 };

    const EngineOutput out { ice.step(1.0, kOverRedlineSpeed, kDt) };
    EXPECT_TRUE(out.isRedline);
    EXPECT_LT(out.torque, 0.0); // Engine friction only, gross combustion is cut
    EXPECT_TRUE(ice.isFuelCut());

    // Step below hysteresis threshold
    constexpr double kBelowHysteresisSpeed { 700.0 }; // ~6684 RPM
    const EngineOutput out2 { ice.step(1.0, kBelowHysteresisSpeed, kDt) };
    (void)out2;
    EXPECT_FALSE(ice.isFuelCut());
}

TEST(Powertrain2DTest, ICEStallBehavior) {
    ICEConfig cfg {};
    cfg.stallRpm = 450.0;
    InternalCombustionEngine2D ice { cfg };

    constexpr double kStallSpeed { 30.0 }; // ~286 RPM
    constexpr double kDt { 0.01 };

    const EngineOutput out { ice.step(1.0, kStallSpeed, kDt) };
    EXPECT_TRUE(out.isStalled);
    EXPECT_FALSE(ice.isRunning());

    ice.start();
    EXPECT_TRUE(ice.isRunning());
    EXPECT_FALSE(ice.isStalled());
}

TEST(Powertrain2DTest, EVDualRegionCurve) {
    ElectricMotorConfig cfg {};
    cfg.maxTorque = 600.0;
    cfg.baseSpeedRpm = 4000.0;
    cfg.maxSpeedRpm = 15000.0;
    cfg.maxPower = 280000.0;
    ElectricMotor2D ev { cfg };

    const double tLow { ev.computeMaxTorque(2000.0) };
    EXPECT_NEAR(tLow, 600.0, 1.0);

    const double tBase { ev.computeMaxTorque(4000.0) };
    EXPECT_NEAR(tBase, 600.0, 1.0);

    const double tHigh { ev.computeMaxTorque(8000.0) };
    EXPECT_NEAR(tHigh, 300.0, 15.0); // Constant power decays inversely with speed
    EXPECT_LT(tHigh, tBase);
}

TEST(Powertrain2DTest, EVRegenerativeBraking) {
    ElectricMotorConfig cfg {};
    cfg.maxRegenTorque = 300.0;
    ElectricMotor2D ev { cfg };

    const double regen { ev.computeRegen(3000.0, 1.0) };
    EXPECT_LT(regen, 0.0);
    EXPECT_GE(std::abs(regen), 100.0);

    const double regenStop { ev.computeRegen(10.0, 1.0) };
    EXPECT_LT(std::abs(regenStop), std::abs(regen)); // Speed taper prevents reverse jerk
}

TEST(Powertrain2DTest, ClutchCoulombSlipLock) {
    ClutchConfig cfg {};
    cfg.maxClutchTorque = 500.0;
    cfg.slipThreshold = 2.0;
    FlywheelClutch2D clutch { cfg };

    constexpr double kDt { 0.01 };
    constexpr double kEngineInertia { 0.20 };
    constexpr double kTransInertia { 0.10 };

    // Step with full engagement and matched speeds -> lockup
    clutch.setTransSpeed(900.0 * (2.0 * 3.14159265358979323846 / 60.0));
    ClutchState state {};
    for (int i { 0 }; i < 10; ++i) {
        state = clutch.step(150.0, 150.0, kEngineInertia, kTransInertia, 0.0, kDt);
    }

    EXPECT_TRUE(state.isLocked);
    EXPECT_FALSE(state.isSlipping);

    // Disengage clutch pedal over several ticks to account for slew rate
    for (int i { 0 }; i < 15; ++i) {
        state = clutch.step(300.0, 0.0, kEngineInertia, kTransInertia, 1.0, kDt);
    }
    EXPECT_FALSE(state.isLocked);
}

TEST(Powertrain2DTest, TransmissionRatio) {
    TransmissionConfig cfg {};
    cfg.gearRatios = { 3.5, 2.0, 1.4, 1.0, 0.8, 0.6, 0.0, 0.0 };
    cfg.finalDriveRatio = 4.0;
    cfg.efficiency = 1.0;
    Transmission2D trans { cfg };

    // 1st gear: ratio = 3.5 * 4.0 = 14.0
    TransmissionOutput out { trans.step(100.0, 10.0, 3000.0, 0.01) };
    EXPECT_NEAR(out.propTorque, 1400.0, 1e-4);
    EXPECT_NEAR(out.inputSpeed, 140.0, 1e-4);

    trans.selectGear(-1); // Reverse
    // Wait for shift to complete
    for (int i { 0 }; i < 30; ++i) {
        out = trans.step(100.0, 10.0, 3000.0, 0.01);
    }
    EXPECT_LT(out.propTorque, 0.0);
}

TEST(Powertrain2DTest, TransmissionShiftSeq) {
    TransmissionConfig cfg {};
    cfg.shiftDuration = 0.10;
    Transmission2D trans { cfg };

    EXPECT_FALSE(trans.isShifting());
    EXPECT_TRUE(trans.shiftUp());
    EXPECT_TRUE(trans.isShifting());

    TransmissionOutput out { trans.step(200.0, 20.0, 4000.0, 0.01) };
    EXPECT_TRUE(out.isTorqueCut);
    EXPECT_LT(out.propTorque, 200.0 * trans.getCurrentRatio() * cfg.finalDriveRatio);

    // Run until shift finishes
    for (int i { 0 }; i < 20; ++i) {
        out = trans.step(200.0, 20.0, 4000.0, 0.01);
    }
    EXPECT_FALSE(trans.isShifting());
    EXPECT_EQ(trans.getGear(), 2);
}

TEST(Powertrain2DTest, DifferentialOpenSplit) {
    DifferentialConfig cfg {};
    cfg.type = DifferentialType::Open;
    Differential2D diff { cfg };

    constexpr double kInputTorque { 400.0 };
    constexpr double kLeftSpeed { 50.0 };
    constexpr double kRightSpeed { 20.0 };

    const DifferentialOutput out { diff.step(kInputTorque, kLeftSpeed, kRightSpeed) };
    EXPECT_NEAR(out.leftTorque, out.rightTorque, 1e-4);
    EXPECT_NEAR(out.lockingTorque, 0.0, 1e-4);
}

TEST(Powertrain2DTest, DifferentialSalisbury) {
    DifferentialConfig cfg {};
    cfg.type = DifferentialType::LimitedSlipSalisbury;
    cfg.driveRampRatio = 0.6;
    cfg.preloadTorque = 50.0;
    Differential2D diff { cfg };

    constexpr double kInputTorque { 500.0 };
    constexpr double kLeftSpeed { 60.0 }; // Left slipping faster
    constexpr double kRightSpeed { 20.0 };

    const DifferentialOutput out { diff.step(kInputTorque, kLeftSpeed, kRightSpeed) };
    EXPECT_GT(out.rightTorque, out.leftTorque); // Slower wheel gets more torque
    EXPECT_GT(out.lockingTorque, 50.0);
}

TEST(Powertrain2DTest, DifferentialTorsen) {
    DifferentialConfig cfg {};
    cfg.type = DifferentialType::Torsen;
    cfg.torsenBiasRatio = 4.0;
    Differential2D diff { cfg };

    constexpr double kInputTorque { 500.0 };
    constexpr double kLeftSpeed { 50.0 }; // Left spinning faster
    constexpr double kRightSpeed { 20.0 };

    const DifferentialOutput out { diff.step(kInputTorque, kLeftSpeed, kRightSpeed) };
    EXPECT_GT(out.rightTorque, out.leftTorque);
    // Ratio of high to low torque should approach TBR (4:1)
    const double ratio { out.rightTorque / std::max(1.0, out.leftTorque) };
    EXPECT_NEAR(ratio, 4.0, 0.2);
}

TEST(Powertrain2DTest, DifferentialSpool) {
    DifferentialConfig cfg {};
    cfg.type = DifferentialType::LockedSpool;
    Differential2D diff { cfg };

    constexpr double kInputTorque { 400.0 };
    constexpr double kLeftSpeed { 35.0 };
    constexpr double kRightSpeed { 30.0 };

    const DifferentialOutput out { diff.step(kInputTorque, kLeftSpeed, kRightSpeed) };
    EXPECT_GT(out.rightTorque, out.leftTorque);
    EXPECT_GT(out.lockingTorque, 1000.0);
}

TEST(Powertrain2DTest, PowertrainFullAssembly) {
    Powertrain2D pt {};
    constexpr double kDt { 0.01 };

    double leftWheelSpeed { 10.0 };
    double rightWheelSpeed { 10.0 };

    PowertrainState state {};
    for (int i { 0 }; i < 100; ++i) {
        state = pt.step(1.0, 0.0, 0.0, leftWheelSpeed, rightWheelSpeed, kDt);
        leftWheelSpeed += 0.2;
        rightWheelSpeed += 0.2;
    }

    EXPECT_GT(state.engineRpm, 1000.0);
    EXPECT_GT(state.leftWheelTorque, 0.0);
    EXPECT_GT(state.rightWheelTorque, 0.0);
    EXPECT_FALSE(std::isnan(state.engineRpm));
    EXPECT_FALSE(std::isnan(state.leftWheelTorque));

    // Upshift test
    EXPECT_TRUE(pt.shiftUp());
    for (int i { 0 }; i < 30; ++i) {
        state = pt.step(1.0, 0.0, 0.0, leftWheelSpeed, rightWheelSpeed, kDt);
    }
    EXPECT_EQ(state.activeGear, 2);
}

} // namespace vehicle_sim::dynamics
