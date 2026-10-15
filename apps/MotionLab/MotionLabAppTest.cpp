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

