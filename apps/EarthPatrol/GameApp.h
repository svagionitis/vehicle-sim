#pragma once

#include <string>

#include "apps/EarthPatrol/Camera2D.h"
#include "apps/EarthPatrol/CourseManager.h"
#include "apps/EarthPatrol/SDLWrappers.h"
#include "libs/VehicleSimCore/dynamics/EarthRoverDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace earth_patrol {

/// @struct GameAppConfig
/// @brief Configuration settings for the Earth Patrol game application.
struct GameAppConfig {
    int windowWidth { 1280 }; ///< Window viewport width in pixels.
    int windowHeight { 720 }; ///< Window viewport height in pixels.
    std::string windowTitle { "Earth Patrol" }; ///< Title string displayed in titlebar.
    double targetFps { 60.0 }; ///< Target frame rate in Hertz.
    double fixedDt { 1.0 / 60.0 }; ///< Fixed physics simulation timestep in seconds.
    bool headless { false }; ///< True to run in headless test mode without SDL window.
};

/// @class GameApp
/// @brief Main application coordinator for Earth Patrol running a fixed-timestep game loop.
/// @details Coordinates SDL window lifecycle, hardware-accelerated 2D rendering, event handling,
/// camera tracking, course progression, and fixed-interval physics stepping of EarthRoverDynamics.
class GameApp {
public:
    /// @brief Default constructor with nominal window and simulation settings.
    GameApp();

    /// @brief Parameterized constructor using custom application configuration.
    /// @param[in] config Application settings.
    explicit GameApp(const GameAppConfig& config);

    /// @brief Destructor releasing graphics resources.
    ~GameApp() noexcept = default;

    /// @brief Initializes SDL context, window, and renderer.
    /// @return True if initialized successfully, false on error.
    [[nodiscard]] bool initialize();

    /// @brief Executes the main application loop until stopped or window closed.
    void run();

    /// @brief Requests termination of the main application loop.
    void stop() noexcept;

    /// @brief Polls and processes pending SDL input and window events.
    void processEvents() noexcept;

    /// @brief Advances simulation physics, course progress, and camera tracking by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    void updatePhysics(double dt) noexcept;

    /// @brief Renders terrain ribbon, rover chassis, suspension struts, checkpoints, and HUD.
    void renderFrame() noexcept;

    /// @brief Queries whether the application loop is actively executing.
    /// @return True if running.
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief Gets active application configuration.
    /// @return Constant reference to GameAppConfig.
    [[nodiscard]] const GameAppConfig& getConfig() const noexcept;

    /// @brief Gets rover dynamics coordinator.
    /// @return Constant reference to EarthRoverDynamics.
    [[nodiscard]] const vehicle_sim::dynamics::EarthRoverDynamics& getRover() const noexcept;

    /// @brief Gets mutable rover dynamics coordinator.
    /// @return Mutable reference to EarthRoverDynamics.
    [[nodiscard]] vehicle_sim::dynamics::EarthRoverDynamics& getRover() noexcept;

    /// @brief Gets rover rigid body chassis.
    /// @return Constant reference to RigidBody2D.
    [[nodiscard]] const vehicle_sim::physics::RigidBody2D& getRoverBody() const noexcept;

    /// @brief Gets mutable rover rigid body chassis.
    /// @return Mutable reference to RigidBody2D.
    [[nodiscard]] vehicle_sim::physics::RigidBody2D& getRoverBody() noexcept;

    /// @brief Gets active 2D camera tracker.
    /// @return Constant reference to Camera2D.
    [[nodiscard]] const Camera2D& getCamera() const noexcept;

    /// @brief Gets mutable active 2D camera tracker.
    /// @return Mutable reference to Camera2D.
    [[nodiscard]] Camera2D& getCamera() noexcept;

    /// @brief Gets course and checkpoint manager.
    /// @return Constant reference to CourseManager.
    [[nodiscard]] const CourseManager& getCourse() const noexcept;

    /// @brief Gets mutable course and checkpoint manager.
    /// @return Mutable reference to CourseManager.
    [[nodiscard]] CourseManager& getCourse() noexcept;

private:
    /// @brief Renders the continuous procedural terrain heightfield ribbon.
    void renderTerrain() noexcept;

    /// @brief Renders the 3-axle articulated suspension rover.
    void renderRover() noexcept;

    /// @brief Renders checkpoint milestone pylons (Points A to E).
    void renderCheckpoints() noexcept;

    /// @brief Renders the arcade status HUD (Milestones, speed, timer, progress bar).
    void renderHUD() noexcept;

    GameAppConfig config_ { };
    bool running_ { false };

    SDLContext sdlContext_ { };
    UniqueWindow window_ { nullptr };
    UniqueRenderer renderer_ { nullptr };

    Camera2D camera_ { };
    CourseManager course_ { };
    vehicle_sim::dynamics::EarthRoverDynamics rover_ { };
    vehicle_sim::physics::RigidBody2D roverBody_ { };
};

} // namespace earth_patrol
