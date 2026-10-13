#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/dynamics/GroundEffectAero2D.h"
#include "libs/VehicleSimCore/dynamics/MonsterCarDynamics.h"
#include "libs/VehicleSimCore/physics/FluidField2D.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::physics;
using namespace vehicle_sim::math;

class GroundEffectAeroTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        mAero.reset();
        mFluidSample.density = 1.225; // standard sea-level air density
        mFluidSample.velocity = Vec2 { 0.0, 0.0 };
        mFluidSample.medium = FluidMediumType::Air;
    }

    GroundEffectAero2D mAero {};
    FluidSample2D mFluidSample {};
};

TEST_F(GroundEffectAeroTest, VenturiDownforceScalingTest)
{
    const Vec2 velocity { 50.0, 0.0 }; // 50 m/s (~180 km/h)
    const double heading { 0.0 };
    const double dt { 0.01 };

    GroundEffectAeroInput inputHigh {};
    inputHigh.frontRideHeight = 0.10;
    inputHigh.rearRideHeight = 0.10;

    GroundEffectAeroInput inputMed {};
    inputMed.frontRideHeight = 0.05;
    inputMed.rearRideHeight = 0.05;

    GroundEffectAeroInput inputLow {};
    inputLow.frontRideHeight = 0.03;
    inputLow.rearRideHeight = 0.03;

    const auto outHigh = mAero.update(inputHigh, velocity, heading, mFluidSample, dt);
    const auto outMed = mAero.update(inputMed, velocity, heading, mFluidSample, dt);
    const auto outLow = mAero.update(inputLow, velocity, heading, mFluidSample, dt);

    EXPECT_GT(outHigh.downforce, 0.0);
    EXPECT_GT(outMed.downforce, outHigh.downforce);
    EXPECT_GT(outLow.downforce, outMed.downforce);

    // Singular protection check at 0.0 ride height
    GroundEffectAeroInput inputZero {};
    inputZero.frontRideHeight = 0.0;
    inputZero.rearRideHeight = 0.0;
    const auto outZero = mAero.update(inputZero, velocity, heading, mFluidSample, dt);
    EXPECT_FALSE(std::isnan(outZero.downforce));
    EXPECT_FALSE(std::isinf(outZero.downforce));
    EXPECT_TRUE(outZero.isStalled);
}

TEST_F(GroundEffectAeroTest, AerodynamicStallCutoffTest)
{
    const Vec2 velocity { 60.0, 0.0 };
    const double heading { 0.0 };
    const double dt { 0.01 };

    // Just above stall threshold (0.025m)
    GroundEffectAeroInput inputNominal {};
    inputNominal.frontRideHeight = 0.030;
    inputNominal.rearRideHeight = 0.030;
    const auto outNominal = mAero.update(inputNominal, velocity, heading, mFluidSample, dt);
    EXPECT_FALSE(outNominal.isStalled);
    EXPECT_FALSE(mAero.isStalled());

    // In stall transition zone (0.015m)
    GroundEffectAeroInput inputTransition {};
    inputTransition.frontRideHeight = 0.015;
    inputTransition.rearRideHeight = 0.015;
    const auto outTransition = mAero.update(inputTransition, velocity, heading, mFluidSample, dt);
    EXPECT_TRUE(outTransition.isStalled);
    EXPECT_TRUE(mAero.isStalled());
    EXPECT_LT(outTransition.downforce, outNominal.downforce);

    // Fully choked (at or below 0.005m)
    GroundEffectAeroInput inputChoked {};
    inputChoked.frontRideHeight = 0.004;
    inputChoked.rearRideHeight = 0.004;
    const auto outChoked = mAero.update(inputChoked, velocity, heading, mFluidSample, dt);
    EXPECT_TRUE(outChoked.isStalled);
    EXPECT_LT(outChoked.downforce, outTransition.downforce);
    EXPECT_GT(outChoked.downforce, 0.0); // Residual floor
}

TEST_F(GroundEffectAeroTest, PitchRakeSensitivityTest)
{
    const Vec2 velocity { 50.0, 0.0 };
    const double heading { 0.0 };
    const double dt { 0.01 };

    // Neutral rake: front = rear = 0.05m
    GroundEffectAeroInput inputNeutral {};
    inputNeutral.frontRideHeight = 0.050;
    inputNeutral.rearRideHeight = 0.050;
    const auto outNeutral = mAero.update(inputNeutral, velocity, heading, mFluidSample, dt);

    // Positive rake (nose down, tail up): front = 0.04m, rear = 0.06m (mean is still 0.05m)
    GroundEffectAeroInput inputRaked {};
    inputRaked.frontRideHeight = 0.040;
    inputRaked.rearRideHeight = 0.060;
    const auto outRaked = mAero.update(inputRaked, velocity, heading, mFluidSample, dt);

    // Negative rake (nose high): front = 0.06m, rear = 0.04m
    GroundEffectAeroInput inputReverse {};
    inputReverse.frontRideHeight = 0.060;
    inputReverse.rearRideHeight = 0.040;
    const auto outReverse = mAero.update(inputReverse, velocity, heading, mFluidSample, dt);

    EXPECT_GT(outRaked.downforce, outNeutral.downforce);
    EXPECT_GT(outNeutral.downforce, outReverse.downforce);
}

TEST_F(GroundEffectAeroTest, CenterOfPressureMigrationTest)
{
    const Vec2 velocity { 50.0, 0.0 };
    const double heading { 0.0 };
    const double dt { 0.01 };

    GroundEffectAeroInput inputNeutral {};
    inputNeutral.pitchAngle = 0.0;
    const auto outNeutral = mAero.update(inputNeutral, velocity, heading, mFluidSample, dt);
    EXPECT_DOUBLE_EQ(outNeutral.centerOfPressure, mAero.getConfig().staticCpOffset);
    EXPECT_DOUBLE_EQ(outNeutral.pitchMoment, outNeutral.downforce * outNeutral.centerOfPressure);

    // Positive pitch angle shifts CoP forward
    GroundEffectAeroInput inputPitch {};
    inputPitch.pitchAngle = 0.05; // ~2.8 deg
    const auto outPitch = mAero.update(inputPitch, velocity, heading, mFluidSample, dt);
    EXPECT_GT(outPitch.centerOfPressure, outNeutral.centerOfPressure);
}

TEST_F(GroundEffectAeroTest, DrsFlapSlewRateTimingTest)
{
    const Vec2 velocity { 60.0, 0.0 };
    const double heading { 0.0 };

    GroundEffectAeroInput input {};
    input.requestDrs = true;

    // At 125 deg/s, after 0.10s flap should reach ~12.5 deg
    auto out = mAero.update(input, velocity, heading, mFluidSample, 0.10);
    EXPECT_NEAR(out.flapAngleDeg, 12.5, 0.01);
    EXPECT_TRUE(out.isDrsActive);
    EXPECT_TRUE(mAero.isDrsActive());

    // After another 0.10s (total 0.20s), flap should reach 25 deg
    out = mAero.update(input, velocity, heading, mFluidSample, 0.10);
    EXPECT_NEAR(out.flapAngleDeg, 25.0, 0.01);

    // Emergency close at 250 deg/s: should fully close in 0.10s
    input.requestDrs = false;
    out = mAero.update(input, velocity, heading, mFluidSample, 0.10);
    EXPECT_NEAR(out.flapAngleDeg, 0.0, 0.01);
    EXPECT_FALSE(out.isDrsActive);
    EXPECT_FALSE(mAero.isDrsActive());
}

TEST_F(GroundEffectAeroTest, DrsDragAndDownforceSheddingTest)
{
    const Vec2 velocity { 70.0, 0.0 };
    const double heading { 0.0 };

    // Baseline closed DRS
    GroundEffectAeroInput inputClosed {};
    inputClosed.requestDrs = false;
    const auto outClosed = mAero.update(inputClosed, velocity, heading, mFluidSample, 0.01);

    // Open DRS fully (advance 0.3s)
    GroundEffectAeroInput inputOpen {};
    inputOpen.requestDrs = true;
    static_cast<void>(mAero.update(inputOpen, velocity, heading, mFluidSample, 0.30));
    const auto outOpen = mAero.update(inputOpen, velocity, heading, mFluidSample, 0.01);

    EXPECT_NEAR(outOpen.flapAngleDeg, 25.0, 0.01);
    // Drag should be lower with DRS open
    EXPECT_LT(outOpen.totalDrag, outClosed.totalDrag);
    // Downforce should be slightly lower due to rear wing shedding
    EXPECT_LT(outOpen.downforce, outClosed.downforce);
}

TEST_F(GroundEffectAeroTest, DrsSafetyInterlockBrakeTest)
{
    const Vec2 velocity { 60.0, 0.0 };
    const double heading { 0.0 };

    // Fully open DRS
    GroundEffectAeroInput input {};
    input.requestDrs = true;
    static_cast<void>(mAero.update(input, velocity, heading, mFluidSample, 0.30));
    EXPECT_TRUE(mAero.isDrsActive());

    // Apply brakes (> threshold of 0.05)
    input.brakeDemand = 0.5;
    const auto outBraking = mAero.update(input, velocity, heading, mFluidSample, 0.10);

    EXPECT_TRUE(outBraking.isDrsInhibited);
    EXPECT_NEAR(outBraking.flapAngleDeg, 0.0, 0.01);
    EXPECT_FALSE(outBraking.isDrsActive);
}

TEST_F(GroundEffectAeroTest, DrsSafetyInterlockSteerTest)
{
    const Vec2 velocity { 60.0, 0.0 };
    const double heading { 0.0 };

    GroundEffectAeroInput input {};
    input.requestDrs = true;
    static_cast<void>(mAero.update(input, velocity, heading, mFluidSample, 0.30));
    EXPECT_TRUE(mAero.isDrsActive());

    // Aggressive steer command (> 0.35 threshold)
    input.steerDemand = 0.60;
    const auto outSteering = mAero.update(input, velocity, heading, mFluidSample, 0.10);

    EXPECT_TRUE(outSteering.isDrsInhibited);
    EXPECT_NEAR(outSteering.flapAngleDeg, 0.0, 0.01);
}

TEST_F(GroundEffectAeroTest, DrsSafetyInterlockAirborneTest)
{
    const Vec2 velocity { 60.0, 0.0 };
    const double heading { 0.0 };

    GroundEffectAeroInput input {};
    input.requestDrs = true;
    static_cast<void>(mAero.update(input, velocity, heading, mFluidSample, 0.30));
    EXPECT_TRUE(mAero.isDrsActive());

    // Airborne transition
    input.isAirborne = true;
    const auto outAirborne = mAero.update(input, velocity, heading, mFluidSample, 0.10);

    EXPECT_TRUE(outAirborne.isDrsInhibited);
    EXPECT_NEAR(outAirborne.flapAngleDeg, 0.0, 0.01);
}

TEST_F(GroundEffectAeroTest, FluidFieldCouplingTest)
{
    const Vec2 velocity { 50.0, 0.0 };
    const double heading { 0.0 };
    const double dt { 0.01 };
    GroundEffectAeroInput input {};

    // Still air
    const auto outStill = mAero.update(input, velocity, heading, mFluidSample, dt);

    // Headwind (+10 m/s relative airspeed)
    FluidSample2D headwindSample = mFluidSample;
    headwindSample.velocity = Vec2 { -10.0, 0.0 };
    const auto outHeadwind = mAero.update(input, velocity, heading, headwindSample, dt);

    // Tailwind (-10 m/s relative airspeed)
    FluidSample2D tailwindSample = mFluidSample;
    tailwindSample.velocity = Vec2 { 10.0, 0.0 };
    const auto outTailwind = mAero.update(input, velocity, heading, tailwindSample, dt);

    EXPECT_GT(outHeadwind.downforce, outStill.downforce);
    EXPECT_GT(outStill.downforce, outTailwind.downforce);
}

TEST_F(GroundEffectAeroTest, ApplyForcesTest)
{
    RigidBody2D body {};
    body.setMass(1000.0);
    body.setInertia(1500.0);
    body.setLinearVelocity(Vec2 { 50.0, 0.0 });
    body.setOrientation(0.0);

    GroundEffectAeroInput input {};
    input.frontRideHeight = 0.04;
    input.rearRideHeight = 0.04;

    const auto out = mAero.applyForces(body, input, mFluidSample, 0.01);

    EXPECT_GT(out.downforce, 0.0);
    EXPECT_GT(out.totalDrag, 0.0);
    // At heading 0, drag pushes in -X, downforce pushes in -Y
    EXPECT_LT(out.totalForceWorld.x, 0.0);
    EXPECT_LT(out.totalForceWorld.y, 0.0);
}

TEST_F(GroundEffectAeroTest, DeterministicSnapshotTest)
{
    GroundEffectAero2D aeroA {};
    GroundEffectAero2D aeroB {};

    GroundEffectAeroInput input {};
    input.frontRideHeight = 0.035;
    input.rearRideHeight = 0.045;
    input.pitchAngle = 0.02;
    input.requestDrs = true;

    const Vec2 vel { 55.5, 0.0 };
    const double heading { 0.12 };

    const auto outA = aeroA.update(input, vel, heading, mFluidSample, 0.05);
    const auto outB = aeroB.update(input, vel, heading, mFluidSample, 0.05);

    EXPECT_DOUBLE_EQ(outA.downforce, outB.downforce);
    EXPECT_DOUBLE_EQ(outA.totalDrag, outB.totalDrag);
    EXPECT_DOUBLE_EQ(outA.pitchMoment, outB.pitchMoment);
    EXPECT_DOUBLE_EQ(outA.effectiveCd, outB.effectiveCd);
    EXPECT_DOUBLE_EQ(outA.effectiveCl, outB.effectiveCl);
    EXPECT_DOUBLE_EQ(outA.flapAngleDeg, outB.flapAngleDeg);
    EXPECT_DOUBLE_EQ(outA.totalForceWorld.x, outB.totalForceWorld.x);
    EXPECT_DOUBLE_EQ(outA.totalForceWorld.y, outB.totalForceWorld.y);
}

TEST_F(GroundEffectAeroTest, MonsterCarDynamicsIntegrationTest)
{
    MonsterCarConfig config {};
    config.useGroundEffect = true;
    config.groundEffectConfig.venturiFactor = 0.15;
    config.groundEffectConfig.baseGroundLiftCoeff = 1.20;

    MonsterCarDynamics car { config };
    RigidBody2D body {};
    body.setMass(config.mass);
    body.setInertia(2000.0);

    MonsterCarInput input {};
    input.throttle = 1.0;
    input.requestDrs = true;
    car.setInput(input);

    // Simulate several seconds of forward acceleration
    const double dt { 0.02 };
    for (int i = 0; i < 150; ++i) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Vehicle has accelerated to speed
    EXPECT_GT(car.getSpeed(), 10.0);
    // Active DRS should be deployed
    EXPECT_TRUE(car.getGroundEffect().isDrsActive());
    EXPECT_GT(car.getGroundEffect().getFlapAngle(), 15.0);

    // Now apply heavy braking
    input.throttle = 0.0;
    input.brake = 0.8;
    car.setInput(input);

    for (int i = 0; i < 20; ++i) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // DRS flap should have closed rapidly
    EXPECT_NEAR(car.getGroundEffect().getFlapAngle(), 0.0, 0.01);
    EXPECT_FALSE(car.getGroundEffect().isDrsActive());
}
