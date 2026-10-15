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

