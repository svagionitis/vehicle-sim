#pragma once

#include <string>

#include "apps/EarthPatrol/AudioSystem.h"
#include "apps/EarthPatrol/Camera2D.h"
#include "apps/EarthPatrol/CourseManager.h"
#include "apps/EarthPatrol/GameState.h"
#include "apps/EarthPatrol/ObstacleManager.h"
#include "apps/EarthPatrol/ParallaxRenderer.h"
#include "apps/EarthPatrol/ParticleSystem.h"
#include "apps/EarthPatrol/PixelFont.h"
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
    int initialLives { 3 }; ///< Initial player rover lives.
};

/// @class GameApp
/// @brief Main application coordinator for Earth Patrol running a fixed-timestep game loop.
/// @details Coordinates SDL window lifecycle, hardware-accelerated 2D rendering, event handling,
/// camera tracking, course progression, obstacle collisions, and fixed-interval physics stepping.
class GameApp {
public:
    /// @brief Default constructor with nominal window and simulation settings.
    GameApp();

    /// @brief Parameterized constructor using custom application configuration.
    /// @param[in] config Application settings.
    explicit GameApp(const GameAppConfig& config);

    /// @brief Destructor releasing graphics resources.
    ~GameApp() noexcept = default;

    /// @brief Initializes SDL context, window, renderer, course, and obstacles.
    /// @return True if initialized successfully, false on error.
    [[nodiscard]] bool initialize();

    /// @brief Executes the main application loop until stopped or window closed.
    void run();

    /// @brief Requests termination of the main application loop.
    void stop() noexcept;

    /// @brief Polls and processes pending SDL input and window events.
    void processEvents() noexcept;

    /// @brief Advances simulation physics, course progress, obstacles, and camera by dt.
    /// @param[in] dt Timestep duration in seconds.
    void updatePhysics(double dt) noexcept;

    /// @brief Renders parallax sky/mountains, terrain ribbon, obstacles, rover, and HUD.
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

    /// @brief Gets obstacle hazard manager.
    /// @return Constant reference to ObstacleManager.
    [[nodiscard]] const ObstacleManager& getObstacles() const noexcept;

    /// @brief Gets remaining player lives.
    /// @return Lives count.
    [[nodiscard]] int getLives() const noexcept;

    /// @brief Queries whether rover is currently destroyed and awaiting respawn.
    /// @return True if crashed.
    [[nodiscard]] bool isCrashed() const noexcept;

    /// @brief Resets rover state to last crossed checkpoint after crash.
    void respawnRover() noexcept;

    /// @brief Sets active terrestrial biome index.
    /// @param[in] biomeIndex Biome identifier (0: Highway, 1: Desert, 2: Alpine).
    void setBiomeIndex(int biomeIndex);

    /// @brief Gets active terrestrial biome index.
    /// @return Biome index.
    [[nodiscard]] int getBiomeIndex() const noexcept;

    /// @brief Gets active top-level game execution state.
    /// @return Active GameState.
    [[nodiscard]] GameState getState() const noexcept;

    /// @brief Sets top-level game execution state.
    /// @param[in] state Target GameState.
    void setState(GameState state) noexcept;

    /// @brief Gets current player score.
    /// @return Score points.
    [[nodiscard]] int getScore() const noexcept;

    /// @brief Gets high score.
    /// @return High score points.
    [[nodiscard]] int getHighScore() const noexcept;

    /// @brief Gets elapsed time in current stage in seconds.
    /// @return Elapsed seconds.
    [[nodiscard]] double getStageTime() const noexcept;

    /// @brief Gets current stage number.
    /// @return Stage index (1-based).
    [[nodiscard]] int getStage() const noexcept;

    /// @brief Initializes a brand new game playthrough.
    void startNewGame() noexcept;

    /// @brief Advances to next biome course stage after Milestone E clearance.
    void advanceStage() noexcept;

    /// @brief Gets active particle effects coordinator.
    /// @return Constant reference to ParticleSystem.
    [[nodiscard]] const ParticleSystem& getParticles() const noexcept;

    /// @brief Gets mutable active particle effects coordinator.
    /// @return Mutable reference to ParticleSystem.
    [[nodiscard]] ParticleSystem& getParticles() noexcept;

    /// @brief Gets procedural retro audio synthesizer.
    /// @return Constant reference to AudioSystem.
    [[nodiscard]] const AudioSystem& getAudio() const noexcept;

    /// @brief Gets mutable procedural retro audio synthesizer.
    /// @return Mutable reference to AudioSystem.
    [[nodiscard]] AudioSystem& getAudio() noexcept;

private:
    /// @brief Renders the continuous procedural terrain heightfield ribbon.
    void renderTerrain() noexcept;

    /// @brief Renders the 3-axle articulated suspension rover.
    void renderRover() noexcept;

    /// @brief Renders animated crash explosion debris puff.
    void renderCrashPuff() noexcept;

    /// @brief Renders checkpoint milestone pylons (Points A to E).
    void renderCheckpoints() noexcept;

    /// @brief Renders the arcade status HUD (Milestones, speed, timer, lives, progress bar).
    void renderHUD() noexcept;

    /// @brief Renders the TitleScreen arcade overlay and controls banner.
    void renderTitleScreen() noexcept;

    /// @brief Renders the StageClear milestone celebration and score tally.
    void renderStageClear() noexcept;

    /// @brief Renders the GameOver screen with final score summary.
    void renderGameOver() noexcept;

    /// @brief Loads persistent high score record from local disk storage.
    void loadHighScore() noexcept;

    /// @brief Saves persistent high score record to local disk storage.
    void saveHighScore() const noexcept;

    GameAppConfig config_ { };
    bool running_ { false };

    SDLContext sdlContext_ { };
    UniqueWindow window_ { nullptr };
    UniqueRenderer renderer_ { nullptr };
    UniqueController controller_ { nullptr };

    Camera2D camera_ { };
    CourseManager course_ { };
    ParallaxRenderer parallax_ { };
    ObstacleManager obstacles_ { };
    ParticleSystem particles_ { };
    AudioSystem audio_ { };

    vehicle_sim::dynamics::EarthRoverDynamics rover_ { };
    vehicle_sim::physics::RigidBody2D roverBody_ { };

    GameState state_ { GameState::TitleScreen };
    int biomeIndex_ { 0 };
    int lives_ { 3 };
    int score_ { 0 };
    int highScore_ { 50000 };
    int stage_ { 1 };
    double stageTime_ { 0.0 };
    double totalTime_ { 0.0 };
    double stageClearTimer_ { 0.0 };
    int timeBonus_ { 0 };
    int lastCheckpointIdx_ { -1 };
    bool crashed_ { false };
    bool wasAirborne_ { false };
    double respawnTimer_ { 0.0 };
};

} // namespace earth_patrol
