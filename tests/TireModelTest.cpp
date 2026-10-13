#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "libs/VehicleSimCore/dynamics/MonsterCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::physics;
using namespace vehicle_sim::terrain;

TEST(TireModelTest, PureLongitudinal_ZeroSlip_ZeroForce)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 10.0;
    state.lateralSpeed = 0.0;
    state.angularVelocity = 10.0 / tire.getConfig().wheelRadius; // zero slip ratio
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;
    state.surfaceDrag = 0.0; // isolate tire tractive force

    const TireForces forces { tire.evaluateForces(state, 0.016) };
    EXPECT_NEAR(forces.slipRatio, 0.0, 1e-3);
    EXPECT_NEAR(forces.fLong, 0.0, 1.0);
    EXPECT_NEAR(forces.fLat, 0.0, 1.0);
}

TEST(TireModelTest, PureLongitudinal_PositiveAccelerationPeak)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 10.0;
    state.lateralSpeed = 0.0;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;
    state.surfaceDrag = 0.0;

    // Slip ratio ~ 0.12 (near typical peak)
    const double radius { tire.getConfig().wheelRadius };
    state.angularVelocity = (10.0 * 1.12) / radius;

    // Let transient relaxation settle
    TireForces forces {};
    for (int i { 0 }; i < 50; ++i) {
        forces = tire.evaluateForces(state, 0.016);
    }

    EXPECT_GT(forces.fLong, 3500.0);
    EXPECT_LT(forces.fLong, 4500.0); // D = 1.0 * Fz = 4000 N nominal
    EXPECT_NEAR(forces.fLat, 0.0, 1e-2);
}

TEST(TireModelTest, PureLongitudinal_LockedWheelBraking)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 20.0;
    state.lateralSpeed = 0.0;
    state.angularVelocity = 0.0; // locked wheel: kappa = -1.0
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;
    state.surfaceDrag = 0.0;

    TireForces forces {};
    for (int i { 0 }; i < 50; ++i) {
        forces = tire.evaluateForces(state, 0.016);
    }

    EXPECT_NEAR(forces.slipRatio, -1.0, 1e-2);
    // Locked wheel produces strong opposing braking force (negative Fx)
    EXPECT_LT(forces.fLong, -2500.0);
    EXPECT_NEAR(forces.fLat, 0.0, 1e-2);
}

TEST(TireModelTest, PureLateral_ZeroAngle_ZeroForce)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 15.0;
    state.lateralSpeed = 0.0; // zero slip angle
    state.angularVelocity = 15.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;

    const TireForces forces { tire.evaluateForces(state, 0.016) };
    EXPECT_NEAR(forces.slipAngle, 0.0, 1e-4);
    EXPECT_NEAR(forces.fLat, 0.0, 1.0);
    EXPECT_NEAR(forces.alignTorque, 0.0, 1.0);
}

TEST(TireModelTest, PureLateral_PeakSlipAngleSaturation)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 15.0;
    state.angularVelocity = 15.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;

    // Small slip angle: ~3 degrees (0.052 rad)
    state.lateralSpeed = 15.0 * std::tan(0.052);
    TireForces forcesSmall {};
    for (int i { 0 }; i < 50; ++i) {
        forcesSmall = tire.evaluateForces(state, 0.016);
    }

    // Near peak slip angle: ~8 degrees (0.14 rad)
    state.lateralSpeed = 15.0 * std::tan(0.14);
    TireForces forcesPeak {};
    for (int i { 0 }; i < 50; ++i) {
        forcesPeak = tire.evaluateForces(state, 0.016);
    }

    // Force should be restoring (negative sign opposite to lateral slide)
    EXPECT_LT(forcesSmall.fLat, 0.0);
    EXPECT_LT(forcesPeak.fLat, forcesSmall.fLat); // more negative, higher magnitude
    EXPECT_NEAR(std::abs(forcesPeak.fLat), 3800.0, 500.0);
}

TEST(TireModelTest, PureLateral_PostPeakDropOff)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 20.0;
    state.angularVelocity = 20.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;

    // Peak slip angle: ~8 deg (0.14 rad)
    state.lateralSpeed = 20.0 * std::tan(0.14);
    TireForces forcesPeak {};
    for (int i { 0 }; i < 50; ++i) {
        forcesPeak = tire.evaluateForces(state, 0.016);
    }

    // High slip angle deep in slide: ~30 deg (0.52 rad)
    state.lateralSpeed = 20.0 * std::tan(0.52);
    TireForces forcesSlide {};
    for (int i { 0 }; i < 50; ++i) {
        forcesSlide = tire.evaluateForces(state, 0.016);
    }

    // Peak cornering grip should exceed deep slide grip due to negative curvature factor E
    EXPECT_GT(std::abs(forcesPeak.fLat), std::abs(forcesSlide.fLat));
}

TEST(TireModelTest, CombinedSlip_FrictionEllipseClamping)
{
    PacejkaTireModel tire {};
    PacejkaConfig cfg { tire.getConfig() };
    cfg.combinedMethod = CombinedSlipMethod::FrictionEllipse;
    tire.setConfig(cfg);

    TireState state {};
    state.forwardSpeed = 15.0;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;
    state.surfaceDrag = 0.0;

    // Demanding high acceleration slip AND high lateral slip simultaneously
    state.angularVelocity = (15.0 * 1.30) / cfg.wheelRadius;
    state.lateralSpeed = 15.0 * std::tan(0.20);

    TireForces forces {};
    for (int i { 0 }; i < 50; ++i) {
        forces = tire.evaluateForces(state, 0.016);
    }

    EXPECT_TRUE(forces.isSaturated);
    const double Dx { cfg.longitudinal.D * state.normalLoad };
    const double Dy { cfg.lateral.D * state.normalLoad };
    const double combinedDemand { std::hypot(forces.fLong / Dx, forces.fLat / Dy) };
    EXPECT_LE(combinedDemand, 1.0001);
}

TEST(TireModelTest, SelfAligningTorque_PneumaticTrailReversal)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 15.0;
    state.angularVelocity = 15.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;

    // Moderate slip angle: 4 deg (0.07 rad) -> strong restoring torque
    state.lateralSpeed = 15.0 * std::tan(0.07);
    TireForces forcesModerate {};
    for (int i { 0 }; i < 50; ++i) {
        forcesModerate = tire.evaluateForces(state, 0.016);
    }

    // High slip angle: 25 deg (0.436 rad) -> pneumatic trail collapse
    state.lateralSpeed = 15.0 * std::tan(0.436);
    TireForces forcesExtreme {};
    for (int i { 0 }; i < 50; ++i) {
        forcesExtreme = tire.evaluateForces(state, 0.016);
    }

    EXPECT_GT(std::abs(forcesModerate.alignTorque), 30.0);
    EXPECT_LT(std::abs(forcesExtreme.alignTorque), std::abs(forcesModerate.alignTorque));
}

TEST(TireModelTest, ZeroSpeed_NumericalStability_ZeroNaN)
{
    PacejkaTireModel tire {};
    std::mt19937_64 rng { 42 };
    std::uniform_real_distribution<double> tinyVelDist { -1e-5, 1e-5 };
    std::uniform_real_distribution<double> loadDist { 500.0, 8000.0 };

    for (int i { 0 }; i < 1000; ++i) {
        TireState state {};
        state.forwardSpeed = tinyVelDist(rng);
        state.lateralSpeed = tinyVelDist(rng);
        state.angularVelocity = tinyVelDist(rng);
        state.normalLoad = loadDist(rng);
        state.surfaceFriction = 1.0;

        const TireForces forces { tire.evaluateForces(state, 0.016) };
        EXPECT_FALSE(std::isnan(forces.fLong));
        EXPECT_FALSE(std::isnan(forces.fLat));
        EXPECT_FALSE(std::isnan(forces.alignTorque));
        EXPECT_FALSE(std::isnan(forces.rollingDrag));
        EXPECT_FALSE(std::isinf(forces.fLong));
        EXPECT_FALSE(std::isinf(forces.fLat));
    }
}

TEST(TireModelTest, ZeroNormalLoad_AirborneZeroForce)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 20.0;
    state.lateralSpeed = 5.0;
    state.angularVelocity = 100.0;
    state.normalLoad = 0.0; // Airborne wheel

    const TireForces forces { tire.evaluateForces(state, 0.016) };
    EXPECT_DOUBLE_EQ(forces.fLong, 0.0);
    EXPECT_DOUBLE_EQ(forces.fLat, 0.0);
    EXPECT_DOUBLE_EQ(forces.alignTorque, 0.0);
    EXPECT_DOUBLE_EQ(forces.rollingDrag, 0.0);
}

TEST(TireModelTest, SurfaceMaterialCoupling_IceVsTarmac)
{
    PacejkaTireModel tire {};
    const SurfaceMaterial tarmacMat { SurfaceMaterial::getPreset(MaterialType::Tarmac) };
    const SurfaceMaterial iceMat { SurfaceMaterial::getPreset(MaterialType::Ice) };

    TireState state {};
    state.forwardSpeed = 15.0;
    state.lateralSpeed = 15.0 * std::tan(0.12);
    state.angularVelocity = 15.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;

    state.surfaceFriction = tarmacMat.friction;
    tire.reset();
    TireForces tarmacForces {};
    for (int i { 0 }; i < 50; ++i) {
        tarmacForces = tire.evaluateForces(state, 0.016);
    }

    state.surfaceFriction = iceMat.friction;
    tire.reset();
    TireForces iceForces {};
    for (int i { 0 }; i < 50; ++i) {
        iceForces = tire.evaluateForces(state, 0.016);
    }

    EXPECT_GT(std::abs(tarmacForces.fLat), std::abs(iceForces.fLat) * 3.0);
}

TEST(TireModelTest, RelaxationFilter_TransientStepResponse)
{
    PacejkaTireModel tire {};
    TireState state {};
    state.forwardSpeed = 15.0;
    state.lateralSpeed = 15.0 * std::tan(0.10); // instantaneous step lateral slip
    state.angularVelocity = 15.0 / tire.getConfig().wheelRadius;
    state.normalLoad = 4000.0;

    tire.reset();
    const TireForces step1 { tire.evaluateForces(state, 0.005) };
    TireForces step5 {};
    for (int i { 0 }; i < 4; ++i) {
        step5 = tire.evaluateForces(state, 0.005);
    }
    TireForces step50 {};
    for (int i { 0 }; i < 45; ++i) {
        step50 = tire.evaluateForces(state, 0.005);
    }

    // Effective slip angle should grow progressively toward target geometric slip angle
    EXPECT_LT(std::abs(step1.effSlipAngle), std::abs(step5.effSlipAngle));
    EXPECT_LT(std::abs(step5.effSlipAngle), std::abs(step50.effSlipAngle));
    EXPECT_NEAR(step50.effSlipAngle, step50.slipAngle, 0.01);
}

TEST(TireModelTest, Presets_ValidCoefficients)
{
    PacejkaTireModel tire {};

    tire.loadPreset(TirePresetType::RacingSlick);
    EXPECT_GT(tire.getConfig().lateral.D, 1.2);
    EXPECT_LT(tire.getConfig().relaxationLat, 0.20);

    tire.loadPreset(TirePresetType::OffRoadKnobby);
    EXPECT_LT(tire.getConfig().lateral.D, 0.9);
    EXPECT_GT(tire.getConfig().wheelRadius, 0.35);

    tire.loadPreset(TirePresetType::LowGripVintage);
    EXPECT_LT(tire.getConfig().longitudinal.D, 0.8);

    tire.loadPreset(TirePresetType::StandardRoad);
    EXPECT_NEAR(tire.getConfig().wheelRadius, 0.32, 1e-4);
}

TEST(TireModelTest, PacejkaWeightingMethod_CombinedSlip)
{
    PacejkaTireModel tire {};
    PacejkaConfig cfg { tire.getConfig() };
    cfg.combinedMethod = CombinedSlipMethod::PacejkaWeighting;
    tire.setConfig(cfg);

    TireState state {};
    state.forwardSpeed = 15.0;
    state.normalLoad = 4000.0;
    state.surfaceFriction = 1.0;
    state.surfaceDrag = 0.0;

    state.angularVelocity = (15.0 * 1.25) / cfg.wheelRadius;
    state.lateralSpeed = 15.0 * std::tan(0.15);

    TireForces forces {};
    for (int i { 0 }; i < 50; ++i) {
        forces = tire.evaluateForces(state, 0.016);
    }

    EXPECT_GT(std::abs(forces.fLong), 1000.0);
    EXPECT_GT(std::abs(forces.fLat), 1000.0);
    EXPECT_TRUE(forces.isSaturated);
}

TEST(TireModelTest, MonsterCarDynamics_PacejkaMode_AcceleratesAndSteers)
{
    MonsterCarConfig cfg {};
    cfg.usePacejkaTire = true;
    cfg.pacejkaConfig = PacejkaTireModel::getPreset(TirePresetType::StandardRoad);

    MonsterCarDynamics car { cfg };
    RigidBodyConfig bodyConfig {};
    bodyConfig.mass = cfg.mass;
    bodyConfig.inertia = 2500.0;
    RigidBody2D body { bodyConfig };

    MonsterCarInput input {};
    input.throttle = 1.0;
    input.steer = 0.5;
    car.setInput(input);

    constexpr double dt { 1.0 / 60.0 };
    for (int step { 0 }; step < 100; ++step) {
        car.step(body, dt);
        Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(car.getSpeed(), 2.0);
    EXPECT_GT(std::abs(body.getAngularVelocity()), 0.05);
}
