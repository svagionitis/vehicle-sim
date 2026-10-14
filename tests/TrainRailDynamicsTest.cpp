#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/dynamics/TrainRailDynamics.h"
#include "libs/VehicleSimCore/math/Spline2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

using namespace vehicle_sim;
using namespace vehicle_sim::dynamics;
using namespace vehicle_sim::math;
using namespace vehicle_sim::physics;

namespace {

Spline2D createStraightTrack(double length = 1000.0)
{
    std::vector<Vec2> points
        = { { 0.0, 0.0 }, { length * 0.25, 0.0 }, { length * 0.5, 0.0 }, { length * 0.75, 0.0 }, { length, 0.0 } };
    return Spline2D(points, false, SplineKnotMode::Centripetal);
}

Spline2D createCurvedTrack(double radius = 300.0)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr std::size_t kSegments = 16;
    std::vector<Vec2> points;
    points.reserve(kSegments + 1);
    for (std::size_t i = 0; i <= kSegments; ++i) {
        const double angle = (static_cast<double>(i) / static_cast<double>(kSegments)) * (kPi * 0.5);
        points.push_back({ radius * std::sin(angle), radius * (1.0 - std::cos(angle)) });
    }
    return Spline2D(points, false, SplineKnotMode::Centripetal);
}

} // namespace

TEST(TrainRailDynamicsTest, DefaultConstructionAndSanity)
{
    TrainRailDynamics train;
    EXPECT_FALSE(train.isAirborne());
    EXPECT_FALSE(train.isDerailed());
    EXPECT_DOUBLE_EQ(train.getTrackSpeed(), 0.0);
    EXPECT_DOUBLE_EQ(train.getArcLength(), 0.0);
    EXPECT_DOUBLE_EQ(train.getLateralOffset(), 0.0);
}

TEST(TrainRailDynamicsTest, SplineTrackAdherence)
{
    const auto track = createStraightTrack(500.0);
    TrainRailDynamics train;
    train.setTrack(&track);
    train.setArcLength(10.0);

    RigidBody2D body;
    body.setPosition({ 0.0, 0.0 });

    TrainRailInput input;
    input.throttle = 1.0;
    input.reverser = 1;
    train.setInput(input);

    for (int i = 0; i < 50; ++i) {
        train.step(body, 0.1);
    }

    EXPECT_GT(train.getTrackSpeed(), 0.0);
    EXPECT_GT(train.getArcLength(), 10.0);
    EXPECT_NEAR(body.getPosition().y, 0.0, 0.05);
}

TEST(TrainRailDynamicsTest, EquilibriumCantSpeed)
{
    const double radius = 400.0;
    const double cantAngle = 0.08; // ~4.5 degrees
    const auto curvedTrack = createCurvedTrack(radius);

    TrainRailConfig config;
    config.cantAngle = cantAngle;
    TrainRailDynamics train(config);
    train.setTrack(&curvedTrack);
    train.setArcLength(50.0);

    // Theoretical equilibrium balancing speed: v_eq = sqrt(g * R * tan(cant))
    constexpr double kGravity = 9.81;
    const double expectedEquilibriumSpeed = std::sqrt(kGravity * radius * std::tan(cantAngle));

    train.setTrackSpeed(expectedEquilibriumSpeed);

    RigidBody2D body;
    train.step(body, 0.01);

    const auto& telemetry = train.getTelemetry();
    // At equilibrium speed, uncompensated lateral acceleration should be close to 0
    EXPECT_NEAR(telemetry.uncompLateralAcc, 0.0, 0.5);
}

TEST(TrainRailDynamicsTest, KlingelHuntingFrequency)
{
    TrainRailConfig config;
    config.wheelConicity = 0.05;
    config.wheelRadius = 0.457;
    config.trackGauge = 1.435;
    TrainRailDynamics train(config);

    const double forwardSpeed = 25.0; // 25 m/s (~90 km/h)
    train.setTrackSpeed(forwardSpeed);

    RigidBody2D body;
    train.step(body, 0.01);

    const auto& telemetry = train.getTelemetry();
    // Klingel frequency: f = (v / 2pi) * sqrt(gamma / (r0 * b))
    constexpr double kPi = 3.14159265358979323846;
    const double b = config.trackGauge * 0.5;
    const double expectedFreq
        = (forwardSpeed / (2.0 * kPi)) * std::sqrt(config.wheelConicity / (config.wheelRadius * b));

    EXPECT_NEAR(telemetry.huntingFrequency, expectedFreq, 0.01);
}

TEST(TrainRailDynamicsTest, FlangeStrikeAndNadal)
{
    TrainRailConfig config;
    config.flangeClearance = 0.010; // 10 mm
    TrainRailDynamics train(config);

    // Initial offset exceeding flange clearance
    train.setLateralOffset(0.018); // 18 mm (8 mm penetration)

    RigidBody2D body;
    train.step(body, 0.01);

    const auto& telemetry = train.getTelemetry();
    EXPECT_GT(std::abs(telemetry.flangeForce), 0.0);
    EXPECT_GT(telemetry.nadalRatio, 0.0);
    EXPECT_GT(telemetry.nadalLimit, 0.0);
}

TEST(TrainRailDynamicsTest, SanderAdhesionBoost)
{
    TrainRailConfig config;
    config.dryAdhesion = 0.32;
    config.sandedAdhesion = 0.45;
    config.maxTractiveForce = 500000.0;

    TrainRailDynamics trainNormal(config);
    TrainRailInput inputNormal;
    inputNormal.throttle = 1.0;
    inputNormal.sanderActive = false;
    trainNormal.setInput(inputNormal);

    RigidBody2D bodyNormal;
    trainNormal.step(bodyNormal, 0.05);
    const double normalTraction = trainNormal.getTelemetry().tractiveForce;

    TrainRailDynamics trainSanded(config);
    TrainRailInput inputSanded;
    inputSanded.throttle = 1.0;
    inputSanded.sanderActive = true;
    trainSanded.setInput(inputSanded);

    RigidBody2D bodySanded;
    trainSanded.step(bodySanded, 0.05);
    const double sandedTraction = trainSanded.getTelemetry().tractiveForce;

    EXPECT_GT(sandedTraction, normalTraction);
    EXPECT_TRUE(trainSanded.getTelemetry().isSanding);
}

TEST(TrainRailDynamicsTest, CouplerDraftSlackAction)
{
    TrainRailConfig config;
    config.carLength = 20.0;
    config.draftSlack = 0.030; // 30 mm
    config.draftStiffness = 5.0e6;

    TrainRailDynamics leadCar(config);
    leadCar.setArcLength(100.0);
    leadCar.setTrackSpeed(10.0);

    TrainRailDynamics followCar(config);
    followCar.setTrackSpeed(10.0);
    followCar.setLeadCar(&leadCar);

    RigidBody2D body;

    // Case 1: Separation matches exactly carLength (20.0m) -> 0 slack, 0 force
    followCar.setArcLength(80.0);
    followCar.step(body, 0.01);
    EXPECT_NEAR(followCar.getTelemetry().couplerForce, 0.0, 1.0);

    // Case 2: Separation is 20.02m (20 mm displacement) -> within 30 mm slack -> 0 force
    followCar.setArcLength(79.98);
    followCar.step(body, 0.01);
    EXPECT_NEAR(followCar.getTelemetry().couplerForce, 0.0, 1.0);

    // Case 3: Separation is 20.06m (60 mm displacement) -> exceeds 30 mm slack -> tension force
    followCar.setArcLength(79.94);
    followCar.step(body, 0.01);
    EXPECT_GT(followCar.getTelemetry().couplerForce, 0.0);
}
