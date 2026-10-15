#include <gtest/gtest.h>

#include "apps/MotionLab/MotionLabApp.h"

using namespace motion_lab;

TEST(MotionLabAppTest, HeadlessInitializationAndDefaultState)
{
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
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
    MotionLabAppConfig config { };
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    // Default vehicle: ClassicCar
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::ClassicCar);
    EXPECT_EQ(app.getVehicleName(), "CLASSIC CAR");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1000.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 1666.667);

    // Direct selection: RaceCar
    app.setVehicleType(MotionLabVehicleType::RaceCar);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::RaceCar);
    EXPECT_EQ(app.getVehicleName(), "RACE CAR");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 750.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 900.0);

    // Direct selection: Rocket
    app.setVehicleType(MotionLabVehicleType::Rocket);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Rocket);
    EXPECT_EQ(app.getVehicleName(), "ROCKET");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 5000.0);
    EXPECT_GT(app.getBody().getInertia(), 10000.0);

    // Direct selection: Helicopter
    app.setVehicleType(MotionLabVehicleType::Helicopter);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Helicopter);
    EXPECT_EQ(app.getVehicleName(), "HELICOPTER");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 2200.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 4800.0);

    // Direct selection: Hovercraft
    app.setVehicleType(MotionLabVehicleType::Hovercraft);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::Hovercraft);
    EXPECT_EQ(app.getVehicleName(), "HOVERCRAFT");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 1200.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 2084.0);

    // Direct selection: TrackedTank
    app.setVehicleType(MotionLabVehicleType::TrackedTank);
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::TrackedTank);
    EXPECT_EQ(app.getVehicleName(), "TRACKED TANK");
    EXPECT_DOUBLE_EQ(app.getBody().getMass(), 12000.0);
    EXPECT_DOUBLE_EQ(app.getBody().getInertia(), 30000.0);
}

TEST(MotionLabAppTest, CycleVehicleSequencesAllPresets)
{
    MotionLabAppConfig config { };
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

    // Wraps around to ClassicCar
    app.cycleVehicle();
    EXPECT_EQ(app.getVehicleType(), MotionLabVehicleType::ClassicCar);
}

TEST(MotionLabAppTest, PhysicsStepAdvancesAllVehicleModels)
{
    MotionLabAppConfig config { };
    config.headless = true;

    MotionLabApp app { config };
    EXPECT_TRUE(app.initialize());

    constexpr double dt { 1.0 / 120.0 };
    const std::array<MotionLabVehicleType, 6> vehicles {{
        MotionLabVehicleType::ClassicCar,
        MotionLabVehicleType::RaceCar,
        MotionLabVehicleType::Rocket,
        MotionLabVehicleType::Helicopter,
        MotionLabVehicleType::Hovercraft,
        MotionLabVehicleType::TrackedTank
    }};

    for (const auto vType : vehicles) {
        app.setVehicleType(vType);
        EXPECT_EQ(app.getVehicleType(), vType);

        // Advance 30 physics steps without crashing
        for (int i { 0 }; i < 30; ++i) {
            app.stepPhysics(dt);
        }

        EXPECT_GT(app.getBody().getMass(), 0.0);
        EXPECT_GT(app.getBody().getInertia(), 0.0);
    }
}

TEST(MotionLabAppTest, CircuitTrackInitializationAndGeneration)
{
    CircuitTrackCoordinator coordinator { };
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
    CircuitTrackCoordinator coordinator { };
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body {
        vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 }
    };
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
    CircuitTrackCoordinator coordinator { };
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body {
        vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 }
    };
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
    MotionLabAppConfig config { };
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
    CircuitTrackCoordinator coordinator { };
    coordinator.toggleEnabled();

    vehicle_sim::physics::RigidBody2D body {
        vehicle_sim::physics::RigidBodyConfig { 1000.0, 1666.667 }
    };
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

