#include <cstdint>
#include <gtest/gtest.h>

#include "apps/MotionLab/MotionLabApp.h"

using namespace motion_lab;

TEST(MotionLabAppTest, HeadlessInitializationAndDefaultState)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());
    EXPECT_TRUE(app.isRunning());
    EXPECT_FALSE(app.isPaused());
    EXPECT_FALSE(app.isBarrierActive());
    EXPECT_FALSE(app.isGravityActive());

    EXPECT_DOUBLE_EQ(app.getBody().getPosition().x, 0.0);
    EXPECT_DOUBLE_EQ(app.getBody().getPosition().y, 0.0);
    EXPECT_DOUBLE_EQ(app.getBody().getLinearVelocity().x, 0.0);
    EXPECT_DOUBLE_EQ(app.getBody().getLinearVelocity().y, 0.0);
}

TEST(MotionLabAppTest, PhysicsStepAdvancesSimulation)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Advance 60 physics steps (0.5s at 120Hz)
    constexpr double dt { 1.0 / 120.0 };
    for (int i { 0 }; i < 60; ++i) {
        app.stepPhysics(dt);
    }

    // At standstill without inputs, vehicle should remain at origin
    EXPECT_DOUBLE_EQ(app.getBody().getPosition().x, 0.0);
    EXPECT_DOUBLE_EQ(app.getBody().getPosition().y, 0.0);
}

TEST(MotionLabAppTest, TogglesAlterAppState)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    app.togglePause();
    EXPECT_TRUE(app.isPaused());
    app.togglePause();
    EXPECT_FALSE(app.isPaused());

    app.toggleBarrier();
    EXPECT_TRUE(app.isBarrierActive());
    EXPECT_TRUE(app.getCamera().isFixedCamera());
    app.toggleBarrier();
    EXPECT_FALSE(app.isBarrierActive());
    EXPECT_FALSE(app.getCamera().isFixedCamera());

    app.toggleGravity();
    EXPECT_TRUE(app.isGravityActive());
    app.toggleGravity();
    EXPECT_FALSE(app.isGravityActive());

    EXPECT_TRUE(app.getCamera().isDeadZoneEnabled());
    app.toggleDeadZone();
    EXPECT_FALSE(app.getCamera().isDeadZoneEnabled());

    EXPECT_TRUE(app.getCamera().isSpeedZoomEnabled());
    app.toggleSpeedZoom();
    EXPECT_FALSE(app.getCamera().isSpeedZoomEnabled());
}

TEST(MotionLabAppTest, CameraResizeUpdatesBaseZoomAndBounds)
{
    MotionLabAppConfig config {};
    config.headless = true;
    config.windowWidth = 1280;
    config.windowHeight = 720;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    const double initialZoom { app.getCamera().getZoom() };
    EXPECT_GT(initialZoom, 0.0);

    // Resize viewport to mobile / compact dimensions
    app.getCamera().resize(400, 300);
    EXPECT_NE(app.getCamera().getZoom(), 0.0);

    const auto bounds = app.getCamera().getBounds();
    EXPECT_LT(bounds.left, bounds.right);
    EXPECT_LT(bounds.bottom, bounds.top);
}

TEST(MotionLabAppTest, TerrainDefaultAndDirectSelection)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Default terrain must be Tarmac
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(app.getSurfaceMaterial().friction, 1.0);
    EXPECT_DOUBLE_EQ(app.getSurfaceMaterial().rollingDrag, 1.0);

    // Direct selection: Ice
    app.setTerrain(vehicle_sim::terrain::MaterialType::Ice);
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Ice);
    EXPECT_DOUBLE_EQ(app.getSurfaceMaterial().friction, 0.10);

    // Direct selection: Sand
    app.setTerrain(vehicle_sim::terrain::MaterialType::Sand);
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Sand);
    EXPECT_DOUBLE_EQ(app.getSurfaceMaterial().rollingDrag, 2.40);
    EXPECT_DOUBLE_EQ(app.getSurfaceMaterial().friction, 0.45);
}

TEST(MotionLabAppTest, CycleTerrainSequencesAllPresets)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Tarmac);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Ice);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Sand);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Mud);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Grass);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Gravel);

    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Snow);

    // Wraps around to Tarmac
    app.cycleTerrain();
    EXPECT_EQ(app.getTerrain(), vehicle_sim::terrain::MaterialType::Tarmac);
}

TEST(MotionLabAppTest, VehicleDefaultAndDirectSelection)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Default vehicle: ClassicCar
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::ClassicCar);
    EXPECT_EQ(app.getVehicleName(), "CLASSIC CAR");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1000.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 1666.667);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.0);

    // Direct selection: RaceCar
    app.setVehicleType(MotionLabVehicleType::RaceCar);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::RaceCar);
    EXPECT_EQ(app.getVehicleName(), "RACE CAR");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 750.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 900.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 1.80);

    // Direct selection: Rocket
    app.setVehicleType(MotionLabVehicleType::Rocket);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Rocket);
    EXPECT_EQ(app.getVehicleName(), "ROCKET");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 5000.0);
    EXPECT_GT(app.getBody().getInertia(), 10000.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 1.60);

    // Direct selection: Helicopter
    app.setVehicleType(MotionLabVehicleType::Helicopter);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Helicopter);
    EXPECT_EQ(app.getVehicleName(), "HELICOPTER");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 2200.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 4800.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.40);

    // Direct selection: Hovercraft
    app.setVehicleType(MotionLabVehicleType::Hovercraft);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Hovercraft);
    EXPECT_EQ(app.getVehicleName(), "HOVERCRAFT");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1200.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 2084.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.20);

    // Direct selection: TrackedTank
    app.setVehicleType(MotionLabVehicleType::TrackedTank);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::TrackedTank);
    EXPECT_EQ(app.getVehicleName(), "TRACKED TANK");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 12000.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 30000.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.40);

    // Direct selection: AntiGravity
    app.setVehicleType(MotionLabVehicleType::AntiGravity);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::AntiGravity);
    EXPECT_EQ(app.getVehicleName(), "ANTI-GRAVITY");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1050.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 950.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.40);
    EXPECT_GT(app.getAntiGravity().getConfig().suspensionStiffness, 0.0);

    // Direct selection: DriftCar
    app.setVehicleType(MotionLabVehicleType::DriftCar);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::DriftCar);
    EXPECT_EQ(app.getVehicleName(), "DRIFT CAR");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1250.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 1800.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 1.85);
    EXPECT_GT(app.getDriftCar().getConfig().mass, 0.0);

    // Direct selection: MonsterTruck
    app.setVehicleType(MotionLabVehicleType::MonsterTruck);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::MonsterTruck);
    EXPECT_EQ(app.getVehicleName(), "MONSTER TRUCK");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1500.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 12000.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 3.00);
    EXPECT_GT(app.getMonsterCar().getConfig().wheelRadius, 0.0);

    // Direct selection: SemiTruck
    app.setVehicleType(MotionLabVehicleType::SemiTruck);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::SemiTruck);
    EXPECT_EQ(app.getVehicleName(), "SEMI-TRUCK");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 31500.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 52000.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 2.50);
    EXPECT_GT(app.getArticulatedTruck().getConfig().trailerLength, 0.0);

    // Direct selection: Forklift
    app.setVehicleType(MotionLabVehicleType::Forklift);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Forklift);
    EXPECT_EQ(app.getVehicleName(), "FORKLIFT");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 3500.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 4200.0);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 1.30);
    EXPECT_GT(app.getForklift().getConfig().wheelbase, 0.0);

    // Direct selection: QuadcopterDrone
    app.setVehicleType(MotionLabVehicleType::QuadcopterDrone);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::QuadcopterDrone);
    EXPECT_EQ(app.getVehicleName(), "DRONE (UAV)");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1.5);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 0.025);
    EXPECT_DOUBLE_EQ(app.getMooseTest().getVehicleWidth(), 1.20);
    EXPECT_GT(app.getDrone().getConfig().armLength, 0.0);
}

TEST(MotionLabAppTest, CycleVehicleSequencesAllPresets)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::ClassicCar);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::RaceCar);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Rocket);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Helicopter);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Hovercraft);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::TrackedTank);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::AntiGravity);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::DriftCar);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::MonsterTruck);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::SemiTruck);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Forklift);

    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::QuadcopterDrone);

    // Wraps around to ClassicCar
    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::ClassicCar);
}

TEST(MotionLabAppTest, PhysicsStepAdvancesAllVehicleModels)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    constexpr double dt { 1.0 / 120.0 };
    const std::array<MotionLabVehicleType, 12> vehicles { { MotionLabVehicleType::ClassicCar,
        MotionLabVehicleType::RaceCar, MotionLabVehicleType::Rocket, MotionLabVehicleType::Helicopter,
        MotionLabVehicleType::Hovercraft, MotionLabVehicleType::TrackedTank, MotionLabVehicleType::AntiGravity,
        MotionLabVehicleType::DriftCar, MotionLabVehicleType::MonsterTruck, MotionLabVehicleType::SemiTruck,
        MotionLabVehicleType::Forklift, MotionLabVehicleType::QuadcopterDrone } };

    for (const auto vType : vehicles) {
        app.setVehicleType(vType);
        EXPECT_EQ(app.getVehicleType(), vType);

        // Advance 30 physics steps without crashing
        for (int i { 0 }; i < 30; ++i) {
            app.stepPhysics(dt);
        }

        EXPECT_GT(app.getBody().getMass(), 0.0);
        EXPECT_GT(app.getBody().getInertia(), 0.0);
        EXPECT_FALSE(std::isnan(app.getBody().getPosition().x));
        EXPECT_FALSE(std::isnan(app.getBody().getPosition().y));
    }
}

TEST(MotionLabAppTest, CircuitTrackInitializationAndGeneration)
{
    CircuitTrackCoordinator coordinator {};
    EXPECT_FALSE(coordinator.isEnabled());
    coordinator.toggleEnabled();
    EXPECT_TRUE(coordinator.isEnabled());

    const auto& track = coordinator.getTrack();
    EXPECT_TRUE(track.isClosed);
    EXPECT_GT(track.trackLength, 150.0);
    EXPECT_GT(track.centerline.size(), 16U);
    EXPECT_EQ(track.centerline.size(), track.leftBoundary.size());
    EXPECT_EQ(track.centerline.size(), track.rightBoundary.size());
}

TEST(MotionLabAppTest, CircuitTrackResetPositionsVehicleOnGrid)
{
    CircuitTrackCoordinator coordinator {};
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body { vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 } };
    coordinator.resetToGrid(body);

    const auto& track = coordinator.getTrack();
    EXPECT_NEAR(body.getPosition().x, track.startGridPosition.x, 1e-4);
    EXPECT_NEAR(body.getPosition().y, track.startGridPosition.y, 1e-4);
    EXPECT_NEAR(body.getOrientation(), track.startHeading, 1e-4);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().length(), 0.0);
    EXPECT_DOUBLE_EQ(coordinator.getMetrics().currentLapTime, 0.0);
}

TEST(MotionLabAppTest, CircuitTrackSurfaceMaterialTransitions)
{
    CircuitTrackCoordinator coordinator {};
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body { vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 } };
    coordinator.resetToGrid(body);

    // On start grid -> roadway ribbon tarmac (friction = 1.0)
    coordinator.stepPhysics(body, 1.0 / 120.0);
    EXPECT_EQ(coordinator.getActiveMaterial().type, vehicle_sim::terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(coordinator.getActiveMaterial().friction, 1.0);

    // 7m perpendicular offset -> runoff gravel shoulder (friction = 0.6)
    const auto& track = coordinator.getTrack();
    body.setPosition(track.startGridPosition + vehicle_sim::math::Vec2 { 7.0, 0.0 });
    coordinator.stepPhysics(body, 1.0 / 120.0);
    EXPECT_EQ(coordinator.getActiveMaterial().type, vehicle_sim::terrain::MaterialType::Gravel);
    EXPECT_DOUBLE_EQ(coordinator.getActiveMaterial().friction, 0.60);

    // Teleport vehicle into infield center (0, 0) -> grass (friction = 0.70)
    body.setPosition(vehicle_sim::math::Vec2::zero());
    coordinator.stepPhysics(body, 1.0 / 120.0);
    EXPECT_EQ(coordinator.getActiveMaterial().type, vehicle_sim::terrain::MaterialType::Grass);
    EXPECT_DOUBLE_EQ(coordinator.getActiveMaterial().friction, 0.70);
}

TEST(MotionLabAppTest, CircuitTrackAppTogglesAndRegeneration)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_FALSE(app.isCircuitActive());
    app.toggleCircuit();
    EXPECT_TRUE(app.isCircuitActive());

    const auto gridPos1 = app.getCircuit().getTrack().startGridPosition;
    EXPECT_NEAR(app.getBody().getPosition().x, gridPos1.x, 1e-4);
    EXPECT_NEAR(app.getBody().getPosition().y, gridPos1.y, 1e-4);

    // Regenerate circuit with new procedural seed
    app.regenerateCircuit();
    EXPECT_TRUE(app.isCircuitActive());
    const auto gridPos2 = app.getCircuit().getTrack().startGridPosition;
    EXPECT_NEAR(app.getBody().getPosition().x, gridPos2.x, 1e-4);
    EXPECT_NEAR(app.getBody().getPosition().y, gridPos2.y, 1e-4);

    // Step physics with circuit active
    constexpr double dt { 1.0 / 120.0 };
    for (int i { 0 }; i < 30; ++i) {
        app.stepPhysics(dt);
    }
}

TEST(MotionLabAppTest, CircuitTrackTimingGatesTraverseLap)
{
    CircuitTrackCoordinator coordinator {};
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body { vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 } };
    coordinator.resetToGrid(body);

    const auto& track = coordinator.getTrack();
    const std::size_t numPoints { track.centerline.size() };

    // Traverse all centerline points forward around the track
    for (std::size_t i { 0U }; i < numPoints; ++i) {
        body.setPosition(track.centerline[i]);
        coordinator.stepPhysics(body, 0.05);
    }
    // Return to start to trigger completion
    body.setPosition(track.centerline[0]);
    coordinator.stepPhysics(body, 0.05);

    EXPECT_GE(coordinator.getMetrics().completedLaps, 1U);
    EXPECT_TRUE(coordinator.getMetrics().hasValidBest);
    EXPECT_GT(coordinator.getMetrics().bestLapTime, 0.0);
}

TEST(MotionLabAppTest, SlalomCourseGeometryLayout)
{
    SlalomCourseCoordinator coordinator {};
    const auto& pylons = coordinator.getPylons();

    EXPECT_EQ(pylons.size(), 10U);
    for (std::size_t i { 0U }; i < pylons.size(); ++i) {
        EXPECT_NEAR(pylons[i].position.x, static_cast<double>(i + 1U) * 10.0, 1e-6);
        EXPECT_NEAR(pylons[i].position.y, 0.0, 1e-6);
        EXPECT_NEAR(pylons[i].radius, 0.4, 1e-6);
        EXPECT_FALSE(pylons[i].wasHit);
        EXPECT_FALSE(pylons[i].passed);
        EXPECT_TRUE(pylons[i].correctSide);
        EXPECT_EQ(pylons[i].coneIndex, static_cast<std::uint32_t>(i + 1U));
    }
}

TEST(MotionLabAppTest, SlalomStartGateTriggerSpeedSensitivity)
{
    SlalomCourseCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Staged behind start line
    body.setPosition(vehicle_sim::math::Vec2 { -1.0, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.2, 0.0 }); // Below 0.5 m/s threshold
    coordinator.stepPhysics(body, 0.1);
    EXPECT_EQ(coordinator.getState(), SlalomState::IdleAtStart);

    // Cross start line but too slow
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.1);
    EXPECT_EQ(coordinator.getState(), SlalomState::IdleAtStart);

    // Re-stage and cross with sufficient speed
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 1.2, 0.0 });
    coordinator.stepPhysics(body, 0.1);

    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.1);
    EXPECT_EQ(coordinator.getState(), SlalomState::InProgress);
}

TEST(MotionLabAppTest, SlalomPylonHitLatchingAndPenalties)
{
    SlalomCourseCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Trigger start
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 2.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    ASSERT_EQ(coordinator.getState(), SlalomState::InProgress);

    // Drive directly into Cone 1 at x = 10.0, y = 0.0
    body.setPosition(vehicle_sim::math::Vec2 { 10.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);

    const auto& pylons = coordinator.getPylons();
    EXPECT_TRUE(pylons[0].wasHit);
    EXPECT_GT(pylons[0].toppleAngle, 1.0);
    EXPECT_EQ(coordinator.getMetrics().hitCount, 1U);
    EXPECT_NEAR(coordinator.getMetrics().penaltyTime, 2.0, 1e-6);

    // Stay in contact for several steps; verify single-penalty latching
    for (int step { 0 }; step < 20; ++step) {
        coordinator.stepPhysics(body, 0.05);
    }
    EXPECT_EQ(coordinator.getMetrics().hitCount, 1U);
    EXPECT_NEAR(coordinator.getMetrics().penaltyTime, 2.0, 1e-6);
}

TEST(MotionLabAppTest, SlalomWeaveComplianceValidation)
{
    SlalomCourseCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Start run
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 5.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.05);

    // Pass Cone 1 (x = 10m) on Left side (y = +2.0m)
    body.setPosition(vehicle_sim::math::Vec2 { 9.0, 2.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 11.0, 2.0 });
    coordinator.stepPhysics(body, 0.05);

    EXPECT_TRUE(coordinator.getPylons()[0].passed);
    EXPECT_TRUE(coordinator.getPylons()[0].correctSide);
    EXPECT_EQ(coordinator.getMetrics().missedGates, 0U);

    // Pass Cone 2 (x = 20m) on Left side again (y = +2.0m instead of expected Right y < 0)
    body.setPosition(vehicle_sim::math::Vec2 { 19.0, 2.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 21.0, 2.0 });
    coordinator.stepPhysics(body, 0.05);

    EXPECT_TRUE(coordinator.getPylons()[1].passed);
    EXPECT_FALSE(coordinator.getPylons()[1].correctSide);
    EXPECT_EQ(coordinator.getMetrics().missedGates, 1U);
}

TEST(MotionLabAppTest, SlalomSectorSplitsAndFinishTiming)
{
    SlalomCourseCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Start run
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 10.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.05);

    // Advance through Sector 1 (x = 35m)
    body.setPosition(vehicle_sim::math::Vec2 { 36.0, 0.0 });
    coordinator.stepPhysics(body, 0.5);
    EXPECT_GT(coordinator.getMetrics().sector1Time, 0.0);

    // Advance through Sector 2 (x = 75m)
    body.setPosition(vehicle_sim::math::Vec2 { 76.0, 0.0 });
    coordinator.stepPhysics(body, 0.5);
    EXPECT_GT(coordinator.getMetrics().sector2Time, 0.0);

    // Cross Finish line (x = 110m)
    body.setPosition(vehicle_sim::math::Vec2 { 111.0, 0.0 });
    coordinator.stepPhysics(body, 0.5);
    EXPECT_EQ(coordinator.getState(), SlalomState::Finished);
    EXPECT_GT(coordinator.getMetrics().sector3Time, 0.0);
    EXPECT_TRUE(coordinator.getMetrics().hasValidBest);
    EXPECT_NEAR(coordinator.getMetrics().finalTime, coordinator.getMetrics().bestTime, 1e-6);
}

TEST(MotionLabAppTest, SlalomResetRestoresCleanState)
{
    SlalomCourseCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Start and hit a cone
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 3.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 10.0, 0.0 });
    coordinator.stepPhysics(body, 0.05);
    ASSERT_EQ(coordinator.getMetrics().hitCount, 1U);

    // Reset course
    coordinator.resetCourse(body);
    EXPECT_EQ(coordinator.getState(), SlalomState::IdleAtStart);
    EXPECT_NEAR(coordinator.getMetrics().elapsedTime, 0.0, 1e-6);
    EXPECT_NEAR(coordinator.getMetrics().penaltyTime, 0.0, 1e-6);
    EXPECT_EQ(coordinator.getMetrics().hitCount, 0U);
    EXPECT_NEAR(body.getPosition().x, -15.0, 1e-6);
    EXPECT_NEAR(body.getPosition().y, 0.0, 1e-6);

    for (const auto& pylon : coordinator.getPylons()) {
        EXPECT_FALSE(pylon.wasHit);
        EXPECT_FALSE(pylon.passed);
        EXPECT_NEAR(pylon.toppleAngle, 0.0, 1e-6);
    }
}

TEST(MotionLabAppTest, SlalomAppTogglesAndExclusivity)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };

    EXPECT_FALSE(app.isSlalomActive());
    app.toggleSlalom();
    EXPECT_TRUE(app.isSlalomActive());
    EXPECT_FALSE(app.isCircuitActive());

    // Activating circuit should deactivate slalom
    app.toggleCircuit();
    EXPECT_TRUE(app.isCircuitActive());
    EXPECT_FALSE(app.isSlalomActive());

    // Activating slalom should deactivate circuit
    app.toggleSlalom();
    EXPECT_TRUE(app.isSlalomActive());
    EXPECT_FALSE(app.isCircuitActive());
}

TEST(MotionLabAppTest, MooseTestDimensionsScaling)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);

    // Default vehicle width: 2.0m
    EXPECT_DOUBLE_EQ(coordinator.getVehicleWidth(), 2.0);
    const auto& secDefault = coordinator.getSections();
    EXPECT_NEAR(secDefault[0].width, 1.1 * 2.0 + 0.25, 1e-6); // 2.45m
    EXPECT_NEAR(secDefault[2].width, 2.0 + 1.0, 1e-6); // 3.00m
    EXPECT_NEAR(secDefault[4].width, 3.0, 1e-6); // 1.3 * 2.0 + 0.25 = 2.85m -> clamped to 3.0m

    // Verify cone count
    const auto& cones = coordinator.getCones();
    EXPECT_EQ(cones.size(), 24U);
    EXPECT_NEAR(cones[0].position.x, 0.0, 1e-6);
    EXPECT_NEAR(cones[0].position.y, 2.45 * 0.5 + 0.4, 1e-6);
    EXPECT_NEAR(cones[1].position.y, -(2.45 * 0.5 + 0.4), 1e-6);

    // Scaled narrow vehicle (e.g. Forklift width: 1.3m)
    coordinator.updateDimensions(1.3, 2.8);
    EXPECT_DOUBLE_EQ(coordinator.getVehicleWidth(), 1.3);
    const auto& secNarrow = coordinator.getSections();
    EXPECT_NEAR(secNarrow[0].width, 1.1 * 1.3 + 0.25, 1e-6); // 1.68m
    EXPECT_NEAR(secNarrow[2].width, 1.3 + 1.0, 1e-6); // 2.30m
    EXPECT_NEAR(secNarrow[4].width, 3.0, 1e-6); // clamped to min 3.0m

    // Scaled wide vehicle (e.g. Semi-Truck width: 2.5m)
    coordinator.updateDimensions(2.5, 12.0);
    EXPECT_DOUBLE_EQ(coordinator.getVehicleWidth(), 2.5);
    const auto& secWide = coordinator.getSections();
    EXPECT_NEAR(secWide[0].width, 1.1 * 2.5 + 0.25, 1e-6); // 3.00m
    EXPECT_NEAR(secWide[2].width, 2.5 + 1.0, 1e-6); // 3.50m
    EXPECT_NEAR(secWide[4].width, 1.3 * 2.5 + 0.25, 1e-6); // 3.50m
}

TEST(MotionLabAppTest, MooseTestStartGateSensitivity)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Stage behind line at x = -1.0m, creeping forward below 1.0 m/s threshold
    body.setPosition(vehicle_sim::math::Vec2 { -1.0, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.2, 0.0, 0.05);
    EXPECT_EQ(coordinator.getState(), MooseTestState::IdleAtStaging);

    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.2, 0.0, 0.05);
    EXPECT_EQ(coordinator.getState(), MooseTestState::IdleAtStaging);

    // Restage and approach with sufficient speed (e.g. 60 km/h = 16.67 m/s)
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 16.6667, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);

    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);

    EXPECT_EQ(coordinator.getState(), MooseTestState::InManeuver);
    EXPECT_NEAR(coordinator.getMetrics().entrySpeed, 60.0, 0.1);
}

TEST(MotionLabAppTest, MooseTestPylonCollisionAndSingleHitLatching)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Trigger start at 60 km/h
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 16.6667, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    ASSERT_EQ(coordinator.getState(), MooseTestState::InManeuver);

    // Drive into Cone 2 at x = 6.0m, y = +w1/2 + 0.4 = 1.625m
    const double targetY { coordinator.getSections()[0].width * 0.5 + 0.4 };
    body.setPosition(vehicle_sim::math::Vec2 { 6.0, targetY });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);

    const auto& cones = coordinator.getCones();
    EXPECT_TRUE(cones[2].wasHit);
    EXPECT_GT(cones[2].toppleAngle, 1.0);
    EXPECT_EQ(coordinator.getMetrics().hitCount, 1U);
    EXPECT_EQ(coordinator.getMetrics().result, MooseRunResult::FailedConeHit);

    // Stay in contact for 30 steps; ensure single hit latching
    for (int step { 0 }; step < 30; ++step) {
        coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    }
    EXPECT_EQ(coordinator.getMetrics().hitCount, 1U);
}

TEST(MotionLabAppTest, MooseTestThrottleAndBrakeViolations)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Trigger start
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 18.0, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    ASSERT_EQ(coordinator.getState(), MooseTestState::InManeuver);

    // Advance past Section 1 (x > 12.0m) into Section 2 with throttle applied
    body.setPosition(vehicle_sim::math::Vec2 { 15.0, 1.0 });
    coordinator.stepPhysics(body, 0.50, 0.0, 0.05); // 50% throttle applied!

    EXPECT_TRUE(coordinator.getMetrics().throttleFault);
    EXPECT_EQ(coordinator.getMetrics().result, MooseRunResult::Disqualified);
}

TEST(MotionLabAppTest, MooseTestCleanPassAndBestSpeedRecord)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Start run at 70 km/h (19.444 m/s)
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 19.4444, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 0.5, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    ASSERT_EQ(coordinator.getState(), MooseTestState::InManeuver);

    // Cross Evasion midpoint at x = 31.0m, y = 3.5m at 64 km/h (17.778 m/s)
    body.setPosition(vehicle_sim::math::Vec2 { 30.0, 3.5 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 17.7778, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 32.0, 3.5 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    EXPECT_NEAR(coordinator.getMetrics().evasionSpeed, 64.0, 0.1);

    // Cross Finish line at x = 61.0m, y = 0.0m at 58 km/h (16.111 m/s)
    body.setPosition(vehicle_sim::math::Vec2 { 60.0, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 16.1111, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 62.0, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);

    EXPECT_EQ(coordinator.getState(), MooseTestState::Completed);
    EXPECT_EQ(coordinator.getMetrics().result, MooseRunResult::CleanPass);
    EXPECT_NEAR(coordinator.getMetrics().exitSpeed, 58.0, 0.1);
    EXPECT_NEAR(coordinator.getMetrics().speedDrop, 12.0, 0.2);
    EXPECT_TRUE(coordinator.getMetrics().hasValidBest);
    EXPECT_NEAR(coordinator.getMetrics().bestPassSpeed, 70.0, 0.1);
}

TEST(MotionLabAppTest, MooseTestResetIntegrity)
{
    MooseTestCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Trigger start and hit a cone
    body.setPosition(vehicle_sim::math::Vec2 { -0.5, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 15.0, 0.0 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    body.setPosition(vehicle_sim::math::Vec2 { 6.0, 1.625 });
    coordinator.stepPhysics(body, 0.0, 0.0, 0.05);
    ASSERT_EQ(coordinator.getMetrics().hitCount, 1U);

    // Reset course
    coordinator.resetCourse(body);
    EXPECT_EQ(coordinator.getState(), MooseTestState::IdleAtStaging);
    EXPECT_NEAR(body.getPosition().x, -30.0, 1e-6);
    EXPECT_NEAR(body.getPosition().y, 0.0, 1e-6);
    EXPECT_EQ(coordinator.getMetrics().hitCount, 0U);
    EXPECT_FALSE(coordinator.getMetrics().throttleFault);
    EXPECT_FALSE(coordinator.getMetrics().brakeFault);

    for (const auto& cone : coordinator.getCones()) {
        EXPECT_FALSE(cone.wasHit);
        EXPECT_NEAR(cone.toppleAngle, 0.0, 1e-6);
    }
}

TEST(MotionLabAppTest, MooseTestAppTogglesAndExclusivity)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };

    EXPECT_FALSE(app.isMooseTestActive());
    app.toggleMooseTest();
    EXPECT_TRUE(app.isMooseTestActive());
    EXPECT_FALSE(app.isCircuitActive());
    EXPECT_FALSE(app.isSlalomActive());

    // Activating slalom should deactivate Moose Test
    app.toggleSlalom();
    EXPECT_TRUE(app.isSlalomActive());
    EXPECT_FALSE(app.isMooseTestActive());

    // Activating Moose Test should deactivate Slalom
    app.toggleMooseTest();
    EXPECT_TRUE(app.isMooseTestActive());
    EXPECT_FALSE(app.isSlalomActive());

    // Activating circuit should deactivate Moose Test
    app.toggleCircuit();
    EXPECT_TRUE(app.isCircuitActive());
    EXPECT_FALSE(app.isMooseTestActive());
}

TEST(MotionLabAppTest, SkidpadPylonLayout15m)
{
    SkidpadCoordinator coordinator {};
    coordinator.setRadiusPreset(SkidpadRadiusPreset::Radius15m);
    EXPECT_EQ(coordinator.getRadiusPreset(), SkidpadRadiusPreset::Radius15m);
    EXPECT_DOUBLE_EQ(coordinator.getTargetRadius(), 15.0);
    EXPECT_EQ(coordinator.getActivePylonCount(), 36U);

    const auto& pylons = coordinator.getPylons();
    constexpr double rInnerExpected { 15.0 - 1.75 };
    constexpr double rOuterExpected { 15.0 + 1.75 };

    // Check first 16 inner cones
    for (std::size_t i { 0U }; i < 16U; ++i) {
        EXPECT_FALSE(pylons[i].isOuterRing);
        EXPECT_NEAR(pylons[i].position.length(), rInnerExpected, 1e-4);
    }
    // Check remaining 20 outer cones
    for (std::size_t i { 16U }; i < 36U; ++i) {
        EXPECT_TRUE(pylons[i].isOuterRing);
        EXPECT_NEAR(pylons[i].position.length(), rOuterExpected, 1e-4);
    }
}

TEST(MotionLabAppTest, SkidpadPylonLayout30m)
{
    SkidpadCoordinator coordinator {};
    coordinator.setRadiusPreset(SkidpadRadiusPreset::Radius30m);
    EXPECT_EQ(coordinator.getRadiusPreset(), SkidpadRadiusPreset::Radius30m);
    EXPECT_DOUBLE_EQ(coordinator.getTargetRadius(), 30.0);
    EXPECT_EQ(coordinator.getActivePylonCount(), 68U);

    const auto& pylons = coordinator.getPylons();
    constexpr double rInnerExpected { 30.0 - 1.75 };
    constexpr double rOuterExpected { 30.0 + 1.75 };

    // Check first 32 inner cones
    for (std::size_t i { 0U }; i < 32U; ++i) {
        EXPECT_FALSE(pylons[i].isOuterRing);
        EXPECT_NEAR(pylons[i].position.length(), rInnerExpected, 1e-4);
    }
    // Check remaining 36 outer cones
    for (std::size_t i { 32U }; i < 68U; ++i) {
        EXPECT_TRUE(pylons[i].isOuterRing);
        EXPECT_NEAR(pylons[i].position.length(), rOuterExpected, 1e-4);
    }
}

TEST(MotionLabAppTest, SkidpadKinematicLatAcc)
{
    SkidpadCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Speed corresponding to exactly 1.0G on 30m circle: v = sqrt(9.80665 * 30) = 17.152 m/s
    constexpr double targetSpeed { 17.15227 };
    body.setPosition(vehicle_sim::math::Vec2 { 30.0, 0.0 });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, targetSpeed });

    coordinator.stepPhysics(body, 0.08, 2.6, 0.02);
    EXPECT_NEAR(coordinator.getMetrics().instantaneousLatAcc, 1.0, 0.02);
    EXPECT_NEAR(coordinator.getMetrics().instantaneousSpeed, targetSpeed * 3.6, 0.1);
    EXPECT_NEAR(coordinator.getMetrics().radiusError, 0.0, 1e-4);
}

TEST(MotionLabAppTest, SkidpadDirectionAndGateEvaluation)
{
    SkidpadCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    constexpr double r { 30.0 };
    constexpr double dt { 0.05 };
    constexpr double speed { 15.0 };
    constexpr int steps { 120 };

    // Simulate smooth CCW orbit: theta increases from -pi to +pi
    for (int i { 0 }; i <= steps; ++i) {
        const double frac { static_cast<double>(i) / static_cast<double>(steps) };
        const double theta { -3.14159265358979323846 + frac * 2.0 * 3.14159265358979323846 };
        body.setPosition(vehicle_sim::math::Vec2 { r * std::cos(theta), r * std::sin(theta) });
        body.setLinearVelocity(vehicle_sim::math::Vec2 { -speed * std::sin(theta), speed * std::cos(theta) });
        coordinator.stepPhysics(body, 0.08, 2.6, dt);
    }

    EXPECT_EQ(coordinator.getMetrics().direction, SkidpadDirection::CounterClockwise);
    EXPECT_TRUE(coordinator.getMetrics().hasValidLap);
    EXPECT_GE(coordinator.getMetrics().completedLaps, 1U);
    EXPECT_GT(coordinator.getMetrics().lapAverageLatAcc, 0.0);
    EXPECT_GT(coordinator.getMetrics().bestCcwLapTime, 0.0);
}

TEST(MotionLabAppTest, SkidpadUndersteerGradientAndBalance)
{
    SkidpadCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    constexpr double r { 30.0 };
    constexpr double wheelbase { 2.6 };
    // delta0 = 2.6 / 30 = 0.08667 rad (~4.96 deg)
    // At v = 17.15 m/s, ay = 1.0G.
    // If road steer is 0.12 rad (~6.87 deg), deltaDiff = 0.0333 rad = +1.91 deg
    // Kus = +1.91 deg / 1.0G = +1.91 deg/G -> Understeer
    constexpr double steerInput { 0.12 };

    body.setPosition(vehicle_sim::math::Vec2 { 0.0, -r });
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 17.15, 0.0 });

    // Step steadily to acquire steady-state lock (timer >= 0.35s)
    for (int i { 0 }; i < 30; ++i) {
        coordinator.stepPhysics(body, steerInput, wheelbase, 0.02);
    }

    EXPECT_TRUE(coordinator.getMetrics().isSteadyState);
    EXPECT_GT(coordinator.getMetrics().understeerGradient, 0.5);
    EXPECT_EQ(coordinator.getMetrics().balance, HandlingClassification::Understeer);
}

TEST(MotionLabAppTest, SkidpadPylonCollisionOBB)
{
    SkidpadCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Find first inner pylon position
    const auto& pylons = coordinator.getPylons();
    const vehicle_sim::math::Vec2 targetPylonPos = pylons[0].position;

    body.setPosition(targetPylonPos);
    body.setLinearVelocity(vehicle_sim::math::Vec2 { 10.0, 0.0 });
    coordinator.stepPhysics(body, 0.0, 2.6, 0.02);

    EXPECT_EQ(coordinator.getMetrics().hitCount, 1U);
    EXPECT_TRUE(pylons[0].wasHit);
    EXPECT_GT(pylons[0].toppleAngle, 1.0);
}

TEST(MotionLabAppTest, SkidpadStagingReset)
{
    SkidpadCoordinator coordinator {};
    coordinator.setEnabled(true);
    vehicle_sim::physics::RigidBody2D body {};

    // Move somewhere and hit a cone
    body.setPosition(coordinator.getPylons()[0].position);
    coordinator.stepPhysics(body, 0.0, 2.6, 0.02);
    ASSERT_EQ(coordinator.getMetrics().hitCount, 1U);

    // Call reset
    coordinator.resetToStaging(body);
    EXPECT_EQ(coordinator.getState(), SkidpadState::Staging);
    EXPECT_NEAR(body.getPosition().x, -40.0, 1e-4);
    EXPECT_NEAR(body.getPosition().y, -30.0, 1e-4);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().length(), 0.0);
    EXPECT_EQ(coordinator.getMetrics().hitCount, 0U);
    EXPECT_FALSE(coordinator.getPylons()[0].wasHit);
}

TEST(MotionLabAppTest, SkidpadAppTogglesAndExclusivity)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };

    EXPECT_FALSE(app.isSkidpadActive());
    app.toggleSkidpad();
    EXPECT_TRUE(app.isSkidpadActive());
    EXPECT_FALSE(app.isCircuitActive());
    EXPECT_FALSE(app.isSlalomActive());
    EXPECT_FALSE(app.isMooseTestActive());

    // Toggling Slalom deactivates Skidpad
    app.toggleSlalom();
    EXPECT_TRUE(app.isSlalomActive());
    EXPECT_FALSE(app.isSkidpadActive());

    // Toggling Skidpad deactivates Slalom
    app.toggleSkidpad();
    EXPECT_TRUE(app.isSkidpadActive());
    EXPECT_FALSE(app.isSlalomActive());

    // Toggling Moose Test deactivates Skidpad
    app.toggleMooseTest();
    EXPECT_TRUE(app.isMooseTestActive());
    EXPECT_FALSE(app.isSkidpadActive());

    // Toggling Skidpad deactivates Moose Test
    app.toggleSkidpad();
    EXPECT_TRUE(app.isSkidpadActive());
    EXPECT_FALSE(app.isMooseTestActive());

    // Toggling Circuit deactivates Skidpad
    app.toggleCircuit();
    EXPECT_TRUE(app.isCircuitActive());
    EXPECT_FALSE(app.isSkidpadActive());

    // Toggling Skidpad deactivates Circuit
    app.toggleSkidpad();
    EXPECT_TRUE(app.isSkidpadActive());
    EXPECT_FALSE(app.isCircuitActive());

    // Setting radius preset resets position
    app.setSkidpadRadius(SkidpadRadiusPreset::Radius15m);
    EXPECT_EQ(app.getSkidpad().getRadiusPreset(), SkidpadRadiusPreset::Radius15m);
    EXPECT_NEAR(app.getBody().getPosition().y, -15.0, 1e-4);
}

TEST(MotionLabAppTest, FleetExpansionDistinctModelProperties)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    constexpr std::array<MotionLabVehicleType, 12> allVehicles { { MotionLabVehicleType::ClassicCar,
        MotionLabVehicleType::RaceCar, MotionLabVehicleType::Rocket, MotionLabVehicleType::Helicopter,
        MotionLabVehicleType::Hovercraft, MotionLabVehicleType::TrackedTank, MotionLabVehicleType::AntiGravity,
        MotionLabVehicleType::DriftCar, MotionLabVehicleType::MonsterTruck, MotionLabVehicleType::SemiTruck,
        MotionLabVehicleType::Forklift, MotionLabVehicleType::QuadcopterDrone } };

    for (const auto vType : allVehicles) {
        app.setVehicleType(vType);
        EXPECT_EQ(app.getVehicleType(), vType);
        EXPECT_FALSE(app.getVehicleName().empty());
        EXPECT_NE(app.getVehicleName(), "UNKNOWN");
        EXPECT_GT(app.getBody().getMass(), 0.0);
        EXPECT_GT(app.getBody().getInertia(), 0.0);
        EXPECT_GT(app.getMooseTest().getVehicleWidth(), 0.5);
    }
}

TEST(MotionLabAppTest, SlalomDimensionsDynamicScaling)
{
    SlalomCourseCoordinator coordinator {};
    EXPECT_DOUBLE_EQ(coordinator.getPylonSpacing(), 10.0);
    EXPECT_NEAR(coordinator.getApexOffset(), 1.55, 1e-4);
    EXPECT_DOUBLE_EQ(coordinator.getStagingX(), -15.0);
    EXPECT_DOUBLE_EQ(coordinator.getFinishX(), 110.0);

    // Scale for Semi-Truck (W = 2.5, L = 12.0)
    coordinator.updateDimensions(2.5, 12.0);
    // Spacing: max(10.0, 2.5 * 12.0) = 30.0m
    EXPECT_DOUBLE_EQ(coordinator.getPylonSpacing(), 30.0);
    // Apex offset: 0.40 + 2.5/2 + 0.15 = 1.80m
    EXPECT_NEAR(coordinator.getApexOffset(), 1.80, 1e-4);
    // Staging: -1.5 * 30.0 = -45.0m
    EXPECT_DOUBLE_EQ(coordinator.getStagingX(), -45.0);
    // Finish: 11.0 * 30.0 = 330.0m
    EXPECT_DOUBLE_EQ(coordinator.getFinishX(), 330.0);

    const auto& pylons = coordinator.getPylons();
    EXPECT_EQ(pylons.size(), 10U);
    for (std::size_t i { 0U }; i < pylons.size(); ++i) {
        EXPECT_NEAR(pylons[i].position.x, static_cast<double>(i + 1U) * 30.0, 1e-6);
        EXPECT_NEAR(pylons[i].position.y, 0.0, 1e-6);
    }

    // Scale for Forklift (W = 1.3, L = 2.8)
    coordinator.updateDimensions(1.3, 2.8);
    // Spacing: max(10.0, 2.5 * 2.8 = 7.0) = 10.0m
    EXPECT_DOUBLE_EQ(coordinator.getPylonSpacing(), 10.0);
    // Apex offset: 0.40 + 1.3/2 + 0.15 = 1.20m
    EXPECT_NEAR(coordinator.getApexOffset(), 1.20, 1e-4);
}

TEST(MotionLabAppTest, SkidpadDimensionsDynamicScaling)
{
    SkidpadCoordinator coordinator {};
    // Default Car (W = 2.0, L = 4.0) at 30m preset
    EXPECT_DOUBLE_EQ(coordinator.getTargetRadius(), 30.0);
    EXPECT_DOUBLE_EQ(coordinator.getCorridorWidth(), 3.5);
    EXPECT_DOUBLE_EQ(coordinator.getStagingLength(), 40.0);

    // Scale for Semi-Truck (W = 2.5, L = 12.0) at 15m preset
    coordinator.setRadiusPreset(SkidpadRadiusPreset::Radius15m);
    coordinator.updateDimensions(2.5, 12.0);
    EXPECT_DOUBLE_EQ(coordinator.getTargetRadius(), 15.0);
    // Swept path at R=15: (0.75 * 12)^2 / 30 = 81 / 30 = 2.70m
    // Corridor: max(3.50, 2.5 + 2.70 + 1.20) = 6.40m
    EXPECT_NEAR(coordinator.getCorridorWidth(), 6.40, 1e-4);
    // Staging: max(40.0, 5.0 * 12.0) = 60.0m
    EXPECT_DOUBLE_EQ(coordinator.getStagingLength(), 60.0);

    // Verify pylon positions conform to 6.4m corridor (inner: 15 - 3.2 = 11.8, outer: 15 + 3.2 = 18.2)
    const auto& pylons = coordinator.getPylons();
    for (std::size_t i { 0U }; i < 16U; ++i) {
        EXPECT_NEAR(pylons[i].position.length(), 11.80, 1e-4);
    }
    for (std::size_t i { 16U }; i < 36U; ++i) {
        EXPECT_NEAR(pylons[i].position.length(), 18.20, 1e-4);
    }
}

TEST(MotionLabAppTest, CircuitTrackDimensionsDynamicScaling)
{
    CircuitTrackCoordinator coordinator {};
    EXPECT_DOUBLE_EQ(coordinator.getTrackWidth(), 10.0);

    // Scale for Semi-Truck (W = 2.5, L = 12.0)
    coordinator.updateDimensions(2.5, 12.0);
    // Width: max(10.0, 3.5 * 2.5 + 1.0 = 9.75, 12.0) = 12.0m
    EXPECT_DOUBLE_EQ(coordinator.getTrackWidth(), 12.0);
}

TEST(MotionLabAppTest, VehicleCyclingPropagatesDimensionsToAllCoordinators)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Default vehicle is ClassicCar (W=2.0, L=4.0)
    EXPECT_DOUBLE_EQ(app.getSlalom().getPylonSpacing(), 10.0);
    EXPECT_DOUBLE_EQ(app.getSkidpad().getCorridorWidth(), 3.5);
    EXPECT_DOUBLE_EQ(app.getCircuit().getTrackWidth(), 10.0);

    // Switch to Semi-Truck (W=2.5, L=12.0)
    app.setVehicleType(MotionLabVehicleType::SemiTruck);
    EXPECT_DOUBLE_EQ(app.getSlalom().getPylonSpacing(), 30.0);
    EXPECT_NEAR(app.getSkidpad().getCorridorWidth(), 5.05, 1e-4);
    EXPECT_DOUBLE_EQ(app.getCircuit().getTrackWidth(), 12.0);
}

TEST(MotionLabAppTest, TelemetryCoordinatorBasicOperations)
{
    TelemetryCoordinator coordinator {};
    EXPECT_TRUE(coordinator.getBuffer().isEmpty());
    EXPECT_FALSE(coordinator.isRecording());

    vehicle_sim::physics::RigidBody2D body {};
    body.setPosition({ 10.0, 20.0 });
    body.setLinearVelocity({ 15.0, 0.0 });

    coordinator.update(body, 1200.0, 2.6, 1.5, 0.45, 1.0, 0.05, 0.02, 1.0, 0.01);
    EXPECT_FALSE(coordinator.getBuffer().isEmpty());
    EXPECT_EQ(coordinator.getBuffer().getCount(), 1U);

    ExtendedTelemetrySnapshot snapshot {};
    coordinator.populateSnapshot(snapshot);
    EXPECT_NEAR(snapshot.speedMps, 15.0, 1e-3);
    EXPECT_GT(snapshot.wheelLoads.frontLeftLoad, 0.0);
    EXPECT_GT(snapshot.wheelLoads.rearRightLoad, 0.0);
    EXPECT_EQ(snapshot.ggTrailCount, 1U);
}

TEST(MotionLabAppTest, TelemetryCoordinatorRecordingSession)
{
    TelemetryCoordinator coordinator {};
    EXPECT_FALSE(coordinator.isRecording());

    // Start recording
    EXPECT_TRUE(coordinator.toggleRecording());
    EXPECT_TRUE(coordinator.isRecording());
    EXPECT_FALSE(coordinator.getRecordFile().empty());

    vehicle_sim::physics::RigidBody2D body {};
    body.setPosition({ 0.0, 0.0 });
    body.setLinearVelocity({ 20.0, 0.0 });

    for (int i { 0 }; i < 5; ++i) {
        coordinator.update(body, 1000.0, 2.5, 1.5, 0.4, 1.0, 0.0, 0.0, i * 0.01, 0.01);
    }
    EXPECT_EQ(coordinator.getRecordFrames(), 5U);

    // Stop recording
    EXPECT_FALSE(coordinator.toggleRecording());
    EXPECT_FALSE(coordinator.isRecording());
}

TEST(MotionLabAppTest, MotionLabAppTelemetryIntegration)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_TRUE(app.isAdvancedHudVisible());
    EXPECT_FALSE(app.toggleAdvancedHud());
    EXPECT_FALSE(app.isAdvancedHudVisible());
    EXPECT_TRUE(app.toggleAdvancedHud());
    EXPECT_TRUE(app.isAdvancedHudVisible());

    EXPECT_TRUE(app.isAllHudVisible());
    EXPECT_FALSE(app.toggleAllHud());
    EXPECT_FALSE(app.isAllHudVisible());
    EXPECT_TRUE(app.toggleAllHud());
    EXPECT_TRUE(app.isAllHudVisible());
    app.setAllHudVisible(false);
    EXPECT_FALSE(app.isAllHudVisible());
    app.setAllHudVisible(true);
    EXPECT_TRUE(app.isAllHudVisible());

    EXPECT_FALSE(app.getTelemetryCoordinator().isRecording());
    EXPECT_TRUE(app.toggleRecording());
    EXPECT_TRUE(app.getTelemetryCoordinator().isRecording());
    EXPECT_FALSE(app.toggleRecording());
    EXPECT_FALSE(app.getTelemetryCoordinator().isRecording());

    // Advance physics and check buffer fills
    app.stepPhysics(0.01);
    EXPECT_FALSE(app.getTelemetryCoordinator().getBuffer().isEmpty());
}

// -----------------------------------------------------------------------------
// Dynamic Wind & Turbulence Vector Coordinator Tests
// -----------------------------------------------------------------------------

TEST(MotionLabAppTest, WindCoordinatorPresetsAndCycling)
{
    WindTurbulenceCoordinator coordinator {};
    EXPECT_TRUE(coordinator.isWindActive());
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::Calm);
    EXPECT_DOUBLE_EQ(coordinator.getSnapshot().trueWindSpeed, 0.0);

    // Cycle through presets sequentially
    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::LightBreeze);

    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::CrosswindGale);

    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::SevereStorm);

    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::Microburst);

    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::DynamicStorm);

    coordinator.cycleNextPreset();
    EXPECT_EQ(coordinator.getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::Calm);

    // Speed adjustment and heading rotation
    coordinator.setPreset(vehicle_sim::physics::WindEnvironmentPreset::CrosswindGale);
    const double initialSpeed = coordinator.getSnapshot().trueWindSpeed;
    coordinator.adjustWindSpeed(1.25);
    coordinator.rotateWindHeading(15.0);

    vehicle_sim::physics::RigidBody2D body {};
    coordinator.stepPhysics(body, MotionLabVehicleType::ClassicCar, 0.01);
    EXPECT_GT(coordinator.getSnapshot().trueWindSpeed, initialSpeed);
}

TEST(MotionLabAppTest, WindCoordinatorAeroInjectionAcrossFleet)
{
    WindTurbulenceCoordinator coordinator {};
    coordinator.setPreset(vehicle_sim::physics::WindEnvironmentPreset::CrosswindGale);

    vehicle_sim::physics::RigidBody2D body {};
    body.setBodyType(vehicle_sim::physics::BodyType::Dynamic);
    body.setMass(1200.0);

    // Step physics at standstill with crosswind
    coordinator.stepPhysics(body, MotionLabVehicleType::ClassicCar, 0.01);
    EXPECT_NE(coordinator.getSnapshot().lateralAeroForce, 0.0);

    // Test distinct aero properties for different models
    const auto raceAero = coordinator.getAeroProps(MotionLabVehicleType::RaceCar);
    const auto semiAero = coordinator.getAeroProps(MotionLabVehicleType::SemiTruck);
    const auto droneAero = coordinator.getAeroProps(MotionLabVehicleType::QuadcopterDrone);

    EXPECT_LT(raceAero.sideArea, semiAero.sideArea);
    EXPECT_GT(semiAero.sideArea, 30.0); // Semi-truck trailer broadside
    EXPECT_LT(droneAero.sideArea, 1.0); // Micro drone silhouette

    EXPECT_GT(coordinator.getDownforceCoeff(MotionLabVehicleType::RaceCar), 2.0);
    EXPECT_DOUBLE_EQ(coordinator.getDownforceCoeff(MotionLabVehicleType::SemiTruck), 0.0);

    // Toggle active state
    coordinator.toggleWindActive();
    EXPECT_FALSE(coordinator.isWindActive());
    coordinator.stepPhysics(body, MotionLabVehicleType::ClassicCar, 0.01);
    coordinator.toggleWindActive();
    EXPECT_TRUE(coordinator.isWindActive());
}

TEST(MotionLabAppTest, WindVectorFieldRendererParticleLifecycle)
{
    WindVectorFieldRenderer renderer {};
    EXPECT_TRUE(renderer.isOverlayEnabled());

    renderer.toggleOverlay();
    EXPECT_FALSE(renderer.isOverlayEnabled());
    renderer.setOverlayEnabled(true);
    EXPECT_TRUE(renderer.isOverlayEnabled());

    WindTurbulenceCoordinator coordinator {};
    coordinator.setPreset(vehicle_sim::physics::WindEnvironmentPreset::SevereStorm);
    MotionLabCamera camera {};

    // Advance particle updates across multiple frames
    for (int i { 0 }; i < 30; ++i) {
        renderer.updateParticles(coordinator, camera, 0.016);
    }

    // Advance with moving vehicle body to verify aerodynamic wake emission
    vehicle_sim::physics::RigidBody2D body {};
    body.setLinearVelocity({ 25.0, 0.0 });
    for (int i { 0 }; i < 20; ++i) {
        renderer.updateParticles(coordinator, camera, 0.016, &body);
    }
}

TEST(MotionLabAppTest, MotionLabAppWindIntegration)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_TRUE(app.getWindCoordinator().isWindActive());
    EXPECT_TRUE(app.getWindRenderer().isOverlayEnabled());

    // Switch to Severe Storm and step physics
    app.getWindCoordinator().setPreset(vehicle_sim::physics::WindEnvironmentPreset::SevereStorm);
    EXPECT_EQ(
        app.getWindCoordinator().getSnapshot().activePreset, vehicle_sim::physics::WindEnvironmentPreset::SevereStorm);

    // Step physics: crosswind force pushes vehicle from origin
    for (int i { 0 }; i < 60; ++i) {
        app.stepPhysics(1.0 / 120.0);
    }

    // Chassis velocity should be deflected by environmental wind
    EXPECT_NE(app.getBody().getLinearVelocity().lengthSquared(), 0.0);
    EXPECT_GT(app.getWindCoordinator().getSnapshot().trueWindSpeed, 10.0);
}

TEST(MotionLabAppTest, WindHudVisibilityToggle)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    EXPECT_TRUE(app.isWindHudVisible());
    EXPECT_FALSE(app.toggleWindHud());
    EXPECT_FALSE(app.isWindHudVisible());
    EXPECT_TRUE(app.toggleWindHud());
    EXPECT_TRUE(app.isWindHudVisible());

    app.setWindHudVisible(false);
    EXPECT_FALSE(app.isWindHudVisible());
    app.setWindHudVisible(true);
    EXPECT_TRUE(app.isWindHudVisible());
}

TEST(MotionLabAppTest, WindHudNoOverlapWithOtherHuds)
{
    auto intersects = [](int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2) noexcept {
        const bool noOverlap = (x1 + w1 <= x2) || (x2 + w2 <= x1) || (y1 + h1 <= y2) || (y2 + h2 <= y1);
        return !noOverlap;
    };

    const std::vector<std::pair<int, int>> testResolutions { { 1280, 720 }, { 1920, 1080 }, { 1024, 768 },
        { 800, 600 } };

    for (const auto& [w, h] : testResolutions) {
        for (const bool circuitActive : { false, true }) {
            for (const bool advancedHudActive : { false, true }) {
                const auto pos = WindVectorFieldRenderer::computeHudCardPos(w, h, circuitActive, advancedHudActive);

                // Ensure HUD card fits inside viewport
                EXPECT_GE(pos.x, 0);
                EXPECT_LE(pos.x + WindVectorFieldRenderer::kCardWidth, w);
                EXPECT_GE(pos.y, 0);
                EXPECT_LE(pos.y + WindVectorFieldRenderer::kCardHeight, h);

                // 1. Verify no overlap with Advanced Telemetry HUD
                if (advancedHudActive) {
                    const int advX { w - 228 - 16 };
                    const int advY { circuitActive ? 170 : 20 };
                    constexpr int advW { 228 };
                    constexpr int advH { 232 };

                    EXPECT_FALSE(intersects(pos.x, pos.y, WindVectorFieldRenderer::kCardWidth,
                        WindVectorFieldRenderer::kCardHeight, advX, advY, advW, advH))
                        << "Overlap detected with Advanced HUD at res (" << w << "x" << h
                        << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive;
                }

                // 2. Verify no overlap with Circuit Radar
                if (circuitActive) {
                    const int radarX { w - 170 };
                    constexpr int radarY { 30 };
                    constexpr int radarW { 140 };
                    constexpr int radarH { 140 };

                    EXPECT_FALSE(intersects(pos.x, pos.y, WindVectorFieldRenderer::kCardWidth,
                        WindVectorFieldRenderer::kCardHeight, radarX, radarY, radarW, radarH))
                        << "Overlap detected with Circuit Radar at res (" << w << "x" << h << ")";
                }

                // 3. Verify no overlap with Input Indicator HUD
                {
                    const int inputX { w - 264 };
                    const int inputY { h - 64 };
                    constexpr int inputW { 250 };
                    constexpr int inputH { 20 };

                    EXPECT_FALSE(intersects(pos.x, pos.y, WindVectorFieldRenderer::kCardWidth,
                        WindVectorFieldRenderer::kCardHeight, inputX, inputY, inputW, inputH))
                        << "Overlap detected with Input HUD at res (" << w << "x" << h
                        << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive;
                }

                // 4. Verify no overlap with Live Telemetry metrics text (top-left)
                {
                    constexpr int textX { 24 };
                    constexpr int textY { 20 };
                    constexpr int textW { 180 };
                    constexpr int textH { 180 };

                    EXPECT_FALSE(intersects(pos.x, pos.y, WindVectorFieldRenderer::kCardWidth,
                        WindVectorFieldRenderer::kCardHeight, textX, textY, textW, textH))
                        << "Overlap detected with Live Metrics text at res (" << w << "x" << h
                        << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive;
                }
            }
        }
    }
}

TEST(MotionLabAppTest, TireHudNoOverlapWithOtherHuds)
{
    auto intersects = [](int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2) noexcept {
        const bool noOverlap = (x1 + w1 <= x2) || (x2 + w2 <= x1) || (y1 + h1 <= y2) || (y2 + h2 <= y1);
        return !noOverlap;
    };

    const std::vector<std::pair<int, int>> testResolutions { { 1280, 720 }, { 1920, 1080 }, { 1024, 768 },
        { 800, 600 } };

    for (const auto& [w, h] : testResolutions) {
        for (const bool circuitActive : { false, true }) {
            for (const bool advancedHudActive : { false, true }) {
                for (const bool windHudActive : { false, true }) {
                    for (const bool provingGroundActive : { false, true }) {
                        const auto pos = TireThermalRenderer::computeHudCardPos(
                            w, h, circuitActive, advancedHudActive, windHudActive, provingGroundActive);

                        // Ensure HUD card fits inside viewport
                        EXPECT_GE(pos.x, 0);
                        EXPECT_LE(pos.x + TireThermalRenderer::kCardWidth, w);
                        EXPECT_GE(pos.y, 0);
                        EXPECT_LE(pos.y + TireThermalRenderer::kCardHeight, h);

                        // 1. Verify no overlap with Advanced Telemetry HUD
                        if (advancedHudActive) {
                            const int advX { w - 228 - 16 };
                            const int advY { circuitActive ? 170 : 20 };
                            constexpr int advW { 228 };
                            constexpr int advH { 232 };

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, advX, advY, advW, advH))
                                << "Overlap detected with Advanced HUD at res (" << w << "x" << h
                                << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive
                                << ", wind: " << windHudActive;
                        }

                        // 2. Verify no overlap with Circuit Radar
                        if (circuitActive) {
                            const int radarX { w - 170 };
                            constexpr int radarY { 30 };
                            constexpr int radarW { 140 };
                            constexpr int radarH { 140 };

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, radarX, radarY, radarW, radarH))
                                << "Overlap detected with Circuit Radar at res (" << w << "x" << h << ")";
                        }

                        // 3. Verify no overlap with Wind HUD
                        if (windHudActive) {
                            const auto wPos
                                = WindVectorFieldRenderer::computeHudCardPos(w, h, circuitActive, advancedHudActive);

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, wPos.x, wPos.y, WindVectorFieldRenderer::kCardWidth,
                                WindVectorFieldRenderer::kCardHeight))
                                << "Overlap detected with Wind HUD at res (" << w << "x" << h
                                << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive
                                << ", wind: " << windHudActive;
                        }

                        // 4. Verify no overlap with Proving Ground HUD
                        if (provingGroundActive) {
                            constexpr int pgX { 10 };
                            const int pgY { h - 155 };
                            constexpr int pgW { 220 };
                            constexpr int pgH { 145 };

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, pgX, pgY, pgW, pgH))
                                << "Overlap detected with Proving Ground HUD at res (" << w << "x" << h << ")";
                        }

                        // 5. Verify no overlap with Input Indicator HUD
                        {
                            const int inputX { w - 264 };
                            const int inputY { h - 64 };
                            constexpr int inputW { 250 };
                            constexpr int inputH { 20 };

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, inputX, inputY, inputW, inputH))
                                << "Overlap detected with Input HUD at res (" << w << "x" << h
                                << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive;
                        }

                        // 6. Verify no overlap with Live Telemetry metrics text (top-left)
                        {
                            constexpr int textX { 24 };
                            constexpr int textY { 20 };
                            constexpr int textW { 180 };
                            constexpr int textH { 180 };

                            EXPECT_FALSE(intersects(pos.x, pos.y, TireThermalRenderer::kCardWidth,
                                TireThermalRenderer::kCardHeight, textX, textY, textW, textH))
                                << "Overlap detected with Live Metrics text at res (" << w << "x" << h
                                << "), circuit: " << circuitActive << ", advanced: " << advancedHudActive;
                        }
                    }
                }
            }
        }
    }
}

TEST(MotionLabAppTest, TireThermalCoordinatorFleetConfiguration)
{
    TireThermalCoordinator coordinator {};

    // Wheeled vehicles
    coordinator.setVehicleType(MotionLabVehicleType::ClassicCar);
    EXPECT_TRUE(coordinator.isWheeledVehicle());
    EXPECT_DOUBLE_EQ(coordinator.getWheelModel(0).getConfig().optimalTempMin, 65.0);

    coordinator.setVehicleType(MotionLabVehicleType::RaceCar);
    EXPECT_TRUE(coordinator.isWheeledVehicle());
    EXPECT_DOUBLE_EQ(coordinator.getWheelModel(0).getConfig().optimalTempMin, 85.0);

    coordinator.setVehicleType(MotionLabVehicleType::DriftCar);
    EXPECT_TRUE(coordinator.isWheeledVehicle());
    EXPECT_DOUBLE_EQ(coordinator.getWheelModel(0).getConfig().optimalTempMin, 75.0);

    coordinator.setVehicleType(MotionLabVehicleType::MonsterTruck);
    EXPECT_TRUE(coordinator.isWheeledVehicle());
    EXPECT_DOUBLE_EQ(coordinator.getWheelModel(0).getConfig().optimalTempMin, 50.0);

    // Non-wheeled bypass vehicles
    coordinator.setVehicleType(MotionLabVehicleType::Rocket);
    EXPECT_FALSE(coordinator.isWheeledVehicle());

    coordinator.setVehicleType(MotionLabVehicleType::Helicopter);
    EXPECT_FALSE(coordinator.isWheeledVehicle());

    coordinator.setVehicleType(MotionLabVehicleType::Hovercraft);
    EXPECT_FALSE(coordinator.isWheeledVehicle());

    coordinator.setVehicleType(MotionLabVehicleType::QuadcopterDrone);
    EXPECT_FALSE(coordinator.isWheeledVehicle());
}

TEST(MotionLabAppTest, TireThermalScrubHeatingAcrossFleet)
{
    TireThermalCoordinator coordinator {};
    coordinator.setVehicleType(MotionLabVehicleType::DriftCar);
    coordinator.reset(25.0);

    TireSmokeParticleBuffer smokeBuffer {};

    // Intense rear wheel scrub
    for (int i { 0 }; i < 300; ++i) {
        coordinator.updateWheel(
            2, 4000.0, 2000.0, 4500.0, 6.0, 3.0, 35.0, 0.32, 12.0, { -1.3, 0.8 }, 0.016, smokeBuffer);
    }

    const auto& stateRL = coordinator.getWheelState(2);
    EXPECT_GT(stateRL.surfaceTemp, 70.0);
    EXPECT_GT(stateRL.carcassTemp, 30.0);
    EXPECT_GT(stateRL.frictionPower, 500.0);
}

TEST(MotionLabAppTest, TireSmokeParticleEmissionLifecycle)
{
    TireSmokeParticleBuffer buffer {};
    EXPECT_EQ(buffer.getActiveCount(), 0U);

    buffer.emit({ 0.0, 0.0 }, { 2.0, 1.0 }, 0.8);
    EXPECT_GE(buffer.getActiveCount(), 1U);

    // Advance 0.5s: particle should still be alive
    buffer.update(0.5);
    EXPECT_GE(buffer.getActiveCount(), 1U);

    // Advance 2.0s: particle should expire
    buffer.update(2.0);
    EXPECT_EQ(buffer.getActiveCount(), 0U);
}

TEST(MotionLabAppTest, MotionLabAppTireThermalIntegrationAndToggles)
{
    MotionLabAppConfig config {};
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Verify initial thermal state
    auto& coordinator = app.getTireThermalCoordinator();
    EXPECT_TRUE(coordinator.isWheeledVehicle());

    // Run 50 physics steps
    for (int i { 0 }; i < 50; ++i) {
        app.stepPhysics(config.fixedDt);
    }

    // Toggle HUD
    app.toggleTireThermalHud();
    app.resetTireThermalStates();

    EXPECT_NEAR(coordinator.getWheelState(0).surfaceTemp, 25.0, 0.5);
}

TEST(MotionLabAppTest, AudioSystemAppIntegrationAndToggles)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // AudioSystem initialized
    auto& audio = app.getAudio();
    EXPECT_TRUE(audio.isInitialized());
    EXPECT_TRUE(audio.isMuted());

    // Toggle mute
    app.toggleAudio();
    EXPECT_FALSE(audio.isMuted());
    app.toggleAudio();
    EXPECT_TRUE(audio.isMuted());

    // Toggle audio HUD
    EXPECT_TRUE(app.isAudioHudActive());
    app.toggleAudioHud();
    EXPECT_FALSE(app.isAudioHudActive());
    app.toggleAudioHud();
    EXPECT_TRUE(app.isAudioHudActive());

    // Run physics steps and assert audio updates
    for (int i { 0 }; i < 30; ++i) {
        app.stepPhysics(config.fixedDt);
    }

    const auto levels = audio.getLevels();
    EXPECT_GE(levels.engineLevel, 0.0f);
    EXPECT_GE(levels.masterLevel, 0.0f);
}

TEST(MotionLabAppTest, BarrierImpactTriggersAudioClang)
{
    MotionLabAppConfig config {};
    config.headless = true;
    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    app.toggleBarrier();
    EXPECT_TRUE(app.isBarrierActive());

    // Set high velocity heading directly into barrier wall
    app.getBody().setPosition({ 48.0, 0.0 });
    app.getBody().setLinearVelocity({ 25.0, 0.0 });

    for (int i { 0 }; i < 5; ++i) {
        app.stepPhysics(config.fixedDt);
    }

    // Impact synthesizer should have triggered
    app.getAudio().setMuted(false);
    std::vector<std::int16_t> buffer(512, 0);
    app.getAudio().synthesizeHeadless(buffer.data(), buffer.size());
    const auto levels = app.getAudio().getLevels();
    EXPECT_GT(levels.impactLevel, 0.01f);
}
