#pragma once

#include <string>
#include <SDL2/SDL.h>

#include "apps/MotionLab/MotionLabCamera.h"
#include "apps/MotionLab/SkidRibbonBuffer.h"
#include "apps/MotionLab/TelemetryHud.h"
#include "apps/MotionLab/TrailParticleBuffer.h"
#include "apps/MotionLab/WireframeRenderer.h"
#include "libs/VehicleSimCore/dynamics/MotionLabCarDynamics.h"
#include "libs/VehicleSimCore/physics/BarrierSolver2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace motion_lab {

/// @struct MotionLabAppConfig
/// @brief Configuration settings for the Motion Lab desktop application.
struct MotionLabAppConfig {
    int windowWidth { 1280 };                              ///< Viewport width in pixels.
    int windowHeight { 720 };                             ///< Viewport height in pixels.
    std::string windowTitle { "Motion Lab 2D (1996 Remaster)" }; ///< Window title string.
    double fixedDt { 1.0 / 120.0 };                       ///< 120 Hz fixed physics step (8.333 ms).
    bool headless { false };                              ///< Run in headless test mode without window.
    double targetFps { 60.0 };                            ///< Target display frame rate.
};

/// @class MotionLabApp
/// @brief Main interactive application coordinator running the 120 Hz Motion Lab simulation.
/// @details Coordinates SDL2 hardware rendering, fixed-clock accumulator stepping,
/// keyboard driving controls, camera dead zone and zoom tracking, and HUD metrics.
class MotionLabApp {
public:
    /// @brief Default constructor.
    MotionLabApp();

    /// @brief Parameterized constructor using custom settings.
    /// @param[in] config Application configuration settings.
    explicit MotionLabApp(const MotionLabAppConfig& config);

    /// @brief Destructor releasing graphics resources.
    ~MotionLabApp() noexcept;

    /// @brief Initializes SDL2 context, window, renderer, and simulation state.
    /// @return True on success, false on error.
    [[nodiscard]] bool initialize();

    /// @brief Executes main application loop until exit requested.
    void run();

    /// @brief Requests termination of the application loop.
    void stop() noexcept;

    /// @brief Polls and processes pending SDL input and window events.
    void processEvents() noexcept;

    /// @brief Steps physics forward by dt (called by fixed accumulator).
    /// @param[in] dt Simulation timestep duration in seconds.
    void stepPhysics(double dt) noexcept;

    /// @brief Renders the simulation scene and HUD into the SDL window.
    void renderFrame() noexcept;

    /// @brief Resets vehicle, camera, skids, and trail to initial state.
    void resetSimulation() noexcept;

    /// @brief Toggles simulation pause state.
    void togglePause() noexcept;

    /// @brief Toggles arena boundary barriers.
    void toggleBarrier() noexcept;

    /// @brief Toggles vertical gravity and aero terminal drag.
    void toggleGravity() noexcept;

    /// @brief Toggles camera dead zone tracking.
    void toggleDeadZone() noexcept;

    /// @brief Toggles dynamic camera speed zoom.
    void toggleSpeedZoom() noexcept;

    /// @brief Toggles live input HUD overlay.
    void toggleInputHud() noexcept;

    /// @brief Toggles borderless fullscreen mode.
    void toggleFullscreen() noexcept;

    /// @brief Cycles to the next available terrain surface preset.
    void cycleTerrain() noexcept;

    /// @brief Sets active terrain surface material preset.
    /// @param[in] type Target terrain material type.
    void setTerrain(vehicle_sim::terrain::MaterialType type) noexcept;

    /// @brief Gets active terrain material type.
    /// @return Active MaterialType enum.
    [[nodiscard]] vehicle_sim::terrain::MaterialType getTerrain() const noexcept;

    /// @brief Gets active surface material configuration.
    /// @return Reference to active SurfaceMaterial.
    [[nodiscard]] const vehicle_sim::terrain::SurfaceMaterial& getSurfaceMaterial() const noexcept;

    /// @brief Queries whether application is running.
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief Queries whether simulation is paused.
    [[nodiscard]] bool isPaused() const noexcept;

    /// @brief Queries whether arena barrier is active.
    [[nodiscard]] bool isBarrierActive() const noexcept;

    /// @brief Queries whether gravity is active.
    [[nodiscard]] bool isGravityActive() const noexcept;

    /// @brief Gets constant reference to active car coordinator.
    [[nodiscard]] const vehicle_sim::dynamics::MotionLabCarDynamics& getCar() const noexcept;

    /// @brief Gets mutable reference to active car coordinator.
    [[nodiscard]] vehicle_sim::dynamics::MotionLabCarDynamics& getCar() noexcept;

    /// @brief Gets constant reference to vehicle chassis rigid body.
    [[nodiscard]] const vehicle_sim::physics::RigidBody2D& getBody() const noexcept;

    /// @brief Gets mutable reference to vehicle chassis rigid body.
    [[nodiscard]] vehicle_sim::physics::RigidBody2D& getBody() noexcept;

    /// @brief Gets constant reference to active camera tracker.
    [[nodiscard]] const MotionLabCamera& getCamera() const noexcept;

    /// @brief Gets mutable reference to active camera tracker.
    [[nodiscard]] MotionLabCamera& getCamera() noexcept;

    /// @brief Gets active application configuration.
    [[nodiscard]] const MotionLabAppConfig& getConfig() const noexcept;

private:
    struct RearTireState {
        vehicle_sim::math::Vec2 position { 0.0, 0.0 };
        bool skidding { false };
    };

    void updateTireSkidmarks(double dt) noexcept;

    MotionLabAppConfig mConfig { };
    SDL_Window* mWindow { nullptr };
    SDL_Renderer* mRenderer { nullptr };
    bool mRunning { false };
    bool mPaused { false };
    bool mBarrier { false };
    bool mGravity { false };
    bool mInputHudVisible { true };
    bool mIsFullscreen { false };
    std::size_t mTerrainIndex { 0U };

    vehicle_sim::physics::RigidBody2D mBody { };
    vehicle_sim::dynamics::MotionLabCarDynamics mCar { };
    MotionLabCamera mCamera { };
    SkidRibbonBuffer mSkids { 10.0 };
    TrailParticleBuffer mTrail { 12.0 };
    WireframeRenderer mWireframe { };

    RearTireState mRearLeftState { };
    RearTireState mRearRightState { };

    // Keyboard inputs
    bool mKeyW { false };
    bool mKeyS { false };
    bool mKeyA { false };
    bool mKeyD { false };
    bool mKeySpace { false };

    // FPS & Timing statistics
    double mTimeAccumulator { 0.0 };
    double mDroppedTime { 0.0 };
    double mCurrentFps { 60.0 };
    int mFrameCounter { 0 };
    double mFpsTimer { 0.0 };
};

} // namespace motion_lab
