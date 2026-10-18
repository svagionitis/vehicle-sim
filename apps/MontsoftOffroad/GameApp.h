#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "apps/MontsoftOffroad/AudioSystem.h"
#include "apps/MontsoftOffroad/BossController.h"
#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CampaignManager.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/CombatManager.h"
#include "apps/MontsoftOffroad/CourseManager.h"
#include "apps/MontsoftOffroad/EnemyManager.h"
#include "apps/MontsoftOffroad/InputController.h"
#include "apps/MontsoftOffroad/ParallaxRenderer.h"
#include "apps/MontsoftOffroad/ParticleSystem.h"
#include "apps/MontsoftOffroad/SDLWrappers.h"
#include "apps/MontsoftOffroad/TruckRenderer.h"
#include "libs/VehicleSimCore/dynamics/OffroadBallistics2D.h"
#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace montsoft_offroad {

/// @struct GameAppConfig
/// @brief Configuration settings for the Montsoft Offroad application.
struct GameAppConfig {
    int windowWidth { 1280 };                ///< Initial window width in pixels.
    int windowHeight { 800 };                ///< Initial window height in pixels.
    std::string windowTitle { "Montsoft Offroad (1988)" }; ///< Window titlebar string.
    double targetFps { 60.0 };               ///< Display refresh target framerate.
    double fixedDt { 1.0 / 60.0 };           ///< Invariant physics simulation timestep in seconds.
    bool headless { false };                 ///< True to execute without opening an SDL window.
    PaletteMode initialPalette { PaletteMode::CgaPalette1 }; ///< Starting retro color mode.
};

/// @class GameApp
/// @brief Central application coordinator for Montsoft Offroad.
/// @details Manages fixed 60 Hz accumulator game loop, input polling, dynamics simulation step,
/// CGA emulation rendering, and headless automated testing.
class GameApp final {
public:
    /// @brief Default constructor using standard application configuration.
    GameApp();

    /// @brief Parameterized constructor using custom configuration.
    /// @param[in] config Application settings.
    explicit GameApp(const GameAppConfig& config);

    /// @brief Destructor.
    ~GameApp() noexcept = default;

    GameApp(const GameApp&) = delete;
    GameApp& operator=(const GameApp&) = delete;
    GameApp(GameApp&&) = delete;
    GameApp& operator=(GameApp&&) = delete;

    /// @brief Initializes SDL context, window, renderer, and simulation subsystems.
    /// @return True on successful initialization, false on error.
    [[nodiscard]] bool initialize();

    /// @brief Executes the main application loop until stopped or window closed.
    void run();

    /// @brief Requests termination of the main application loop.
    void stop() noexcept;

    /// @brief Polls and processes pending SDL input and window events.
    void processEvents() noexcept;

    /// @brief Advances simulation dynamics and ballistics by timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    void updateSimulation(double dt) noexcept;

    /// @brief Renders active scene into the 320x200 CGA framebuffer and presents to window.
    void renderFrame() noexcept;

    /// @brief Queries whether the application loop is actively executing.
    /// @return True if running.
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief Gets active application configuration.
    /// @return Constant reference to GameAppConfig.
    [[nodiscard]] const GameAppConfig& getConfig() const noexcept;

    /// @brief Sets active driver control inputs.
    /// @param[in] input Control input commands.
    void setInput(const vehicle_sim::dynamics::OffroadTruckInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Constant reference to active OffroadTruckInput.
    [[nodiscard]] const vehicle_sim::dynamics::OffroadTruckInput& getInput() const noexcept;

    /// @brief Gets 4WD truck dynamics coordinator.
    /// @return Constant reference to OffroadTruckDynamics.
    [[nodiscard]] const vehicle_sim::dynamics::OffroadTruckDynamics& getTruck() const noexcept;

    /// @brief Gets mutable 4WD truck dynamics coordinator.
    /// @return Mutable reference to OffroadTruckDynamics.
    [[nodiscard]] vehicle_sim::dynamics::OffroadTruckDynamics& getTruck() noexcept;

    /// @brief Gets truck rigid body chassis.
    /// @return Constant reference to RigidBody2D.
    [[nodiscard]] const vehicle_sim::physics::RigidBody2D& getTruckBody() const noexcept;

    /// @brief Gets mutable truck rigid body chassis.
    /// @return Mutable reference to RigidBody2D.
    [[nodiscard]] vehicle_sim::physics::RigidBody2D& getTruckBody() noexcept;

    /// @brief Gets ballistics coordinator.
    /// @return Constant reference to OffroadBallistics2D.
    [[nodiscard]] const vehicle_sim::dynamics::OffroadBallistics2D& getBallistics() const noexcept;

    /// @brief Gets mutable ballistics coordinator.
    /// @return Mutable reference to OffroadBallistics2D.
    [[nodiscard]] vehicle_sim::dynamics::OffroadBallistics2D& getBallistics() noexcept;

    /// @brief Gets combat coordinator.
    /// @return Constant reference to CombatManager.
    [[nodiscard]] const CombatManager& getCombatManager() const noexcept;

    /// @brief Gets mutable combat coordinator.
    /// @return Mutable reference to CombatManager.
    [[nodiscard]] CombatManager& getCombatManager() noexcept;

    /// @brief Gets enemy coordinator.
    /// @return Constant reference to EnemyManager.
    [[nodiscard]] const EnemyManager& getEnemyManager() const noexcept;

    /// @brief Gets mutable enemy coordinator.
    /// @return Mutable reference to EnemyManager.
    [[nodiscard]] EnemyManager& getEnemyManager() noexcept;

    /// @brief Gets CGA palette renderer.
    /// @return Constant reference to CgaPaletteRenderer.
    [[nodiscard]] const CgaPaletteRenderer& getRenderer() const noexcept;

    /// @brief Gets mutable CGA palette renderer.
    /// @return Mutable reference to CgaPaletteRenderer.
    [[nodiscard]] CgaPaletteRenderer& getRenderer() noexcept;

    /// @brief Gets active 2D camera tracker.
    /// @return Constant reference to Camera2D.
    [[nodiscard]] const Camera2D& getCamera() const noexcept;

    /// @brief Gets mutable 2D camera tracker.
    /// @return Mutable reference to Camera2D.
    [[nodiscard]] Camera2D& getCamera() noexcept;

    /// @brief Gets active course and terrain coordinator.
    /// @return Constant reference to CourseManager.
    [[nodiscard]] const CourseManager& getCourse() const noexcept;

    /// @brief Gets mutable course and terrain coordinator.
    /// @return Mutable reference to CourseManager.
    [[nodiscard]] CourseManager& getCourse() noexcept;

    /// @brief Gets multi-tier desert parallax renderer.
    /// @return Constant reference to ParallaxRenderer.
    [[nodiscard]] const ParallaxRenderer& getParallax() const noexcept;

    /// @brief Gets mutable multi-tier desert parallax renderer.
    /// @return Mutable reference to ParallaxRenderer.
    [[nodiscard]] ParallaxRenderer& getParallax() noexcept;

    /// @brief Gets input controller coordinator.
    /// @return Constant reference to InputController.
    [[nodiscard]] const InputController& getInputController() const noexcept;

    /// @brief Gets mutable input controller coordinator.
    /// @return Mutable reference to InputController.
    [[nodiscard]] InputController& getInputController() noexcept;

    /// @brief Gets terramechanic particle system.
    /// @return Constant reference to ParticleSystem.
    [[nodiscard]] const ParticleSystem& getParticleSystem() const noexcept;

    /// @brief Gets mutable terramechanic particle system.
    /// @return Mutable reference to ParticleSystem.
    [[nodiscard]] ParticleSystem& getParticleSystem() noexcept;

    /// @brief Gets truck procedural renderer.
    /// @return Constant reference to TruckRenderer.
    [[nodiscard]] const TruckRenderer& getTruckRenderer() const noexcept;

    /// @brief Gets mutable truck procedural renderer.
    /// @return Mutable reference to TruckRenderer.
    [[nodiscard]] TruckRenderer& getTruckRenderer() noexcept;

    /// @brief Gets boss battle coordinator.
    /// @return Constant reference to BossController.
    [[nodiscard]] const BossController& getBossController() const noexcept;

    /// @brief Gets mutable boss battle coordinator.
    /// @return Mutable reference to BossController.
    [[nodiscard]] BossController& getBossController() noexcept;

    /// @brief Gets audio system coordinator.
    /// @return Constant reference to AudioSystem.
    [[nodiscard]] const AudioSystem& getAudio() const noexcept;

    /// @brief Gets mutable audio system coordinator.
    /// @return Mutable reference to AudioSystem.
    [[nodiscard]] AudioSystem& getAudio() noexcept;

    /// @brief Gets campaign lifecycle coordinator.
    /// @return Constant reference to CampaignManager.
    [[nodiscard]] const CampaignManager& getCampaign() const noexcept;

    /// @brief Gets mutable campaign lifecycle coordinator.
    /// @return Mutable reference to CampaignManager.
    [[nodiscard]] CampaignManager& getCampaign() noexcept;

    /// @brief Queries whether simulation is currently paused.
    /// @return True if paused.
    [[nodiscard]] bool isPaused() const noexcept;

    /// @brief Toggles simulation pause state.
    void togglePause() noexcept;

private:
    GameAppConfig config_ {};
    std::unique_ptr<SDLContext> sdlContext_ { nullptr };
    UniqueWindow window_ { nullptr };
    UniqueRenderer renderer_ { nullptr };
    CgaPaletteRenderer cgaRenderer_ {};
    Camera2D camera_ {};
    CourseManager courseManager_ {};
    ParallaxRenderer parallax_ {};
    InputController inputController_ {};
    ParticleSystem particleSystem_ {};
    TruckRenderer truckRenderer_ {};
    AudioSystem audioSystem_ {};
    CampaignManager campaignManager_ {};

    vehicle_sim::physics::RigidBody2D truckBody_ {};
    vehicle_sim::dynamics::OffroadTruckDynamics truckDynamics_ {};
    CombatManager combatManager_ {};
    EnemyManager enemyManager_ {};
    BossController bossController_ {};
    std::vector<CombatTargetAabb> combatTargets_ {};

    bool running_ { false };
    bool paused_ { false };
    double accumulator_ { 0.0 };
    std::uint64_t lastCounter_ { 0 };
    double prevRearComp_ { 0.0 };
    double prevFrontComp_ { 0.0 };
    bool prevJump_ { false };
    bool prevPrimary_ { false };
    bool prevSecondary_ { false };
    std::size_t prevShockwaveCount_ { 0 };
};

} // namespace montsoft_offroad
