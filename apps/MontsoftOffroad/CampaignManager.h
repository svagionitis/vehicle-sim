#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <SDL2/SDL.h>

#include "apps/MontsoftOffroad/AudioSystem.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "apps/MontsoftOffroad/CourseManager.h"
#include "apps/MontsoftOffroad/HighScoreManager.h"
#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace montsoft_offroad {

/// @enum GameState
/// @brief Master lifecycle states for the Montsoft Offroad campaign.
enum class GameState : std::uint8_t {
    TitleScreen = 0,
    MissionBriefing,
    Playing,
    CheckpointPassed,
    StageClear,
    GameOver,
    NameEntry,
    CampaignVictory,
    HighScores
};

/// @class CampaignManager
/// @brief Coordinates campaign flow, state transitions, scoring, and UI displays.
class CampaignManager final {
public:
    static constexpr int kStartingLives { 3 };
    static constexpr int kExtraLifeInterval { 25000 };

    /// @brief Default constructor.
    CampaignManager() noexcept;

    /// @brief Destructor.
    ~CampaignManager() noexcept = default;

    CampaignManager(const CampaignManager&) = delete;
    CampaignManager& operator=(const CampaignManager&) = delete;
    CampaignManager(CampaignManager&&) noexcept = default;
    CampaignManager& operator=(CampaignManager&&) noexcept = default;

    /// @brief Advances active campaign state and updates timers.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in,out] course CourseManager reference.
    /// @param[in,out] audio AudioSystem reference.
    /// @param[in,out] truckBody RigidBody2D truck reference.
    /// @param[in,out] truckDynamics OffroadTruckDynamics reference.
    void update(double dt,
                CourseManager& course,
                AudioSystem& audio,
                vehicle_sim::physics::RigidBody2D& truckBody,
                vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics) noexcept;

    /// @brief Handles UI and keyboard interaction for active campaign state.
    /// @param[in] key SDL keycode pressed.
    /// @param[in,out] course CourseManager reference.
    /// @param[in,out] audio AudioSystem reference.
    void handleKeyPress(SDL_Keycode key, CourseManager& course, AudioSystem& audio) noexcept;

    /// @brief Renders campaign UI, briefings, title screen, and banners.
    /// @param[in,out] renderer CgaPaletteRenderer reference.
    /// @param[in] course CourseManager reference.
    /// @param[in] truckBody RigidBody2D truck reference.
    void render(CgaPaletteRenderer& renderer,
                const CourseManager& course,
                const vehicle_sim::physics::RigidBody2D& truckBody) const noexcept;

    /// @brief Notifies manager that player truck was destroyed.
    /// @param[in,out] course CourseManager reference.
    /// @param[in,out] audio AudioSystem reference.
    /// @param[in,out] truckBody RigidBody2D truck reference.
    /// @param[in,out] truckDynamics OffroadTruckDynamics reference.
    void handleTruckCrash(CourseManager& course,
                          AudioSystem& audio,
                          vehicle_sim::physics::RigidBody2D& truckBody,
                          vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics) noexcept;

    /// @brief Notifies manager that active stage was completed.
    /// @param[in,out] course CourseManager reference.
    /// @param[in,out] audio AudioSystem reference.
    void handleStageClear(CourseManager& course, AudioSystem& audio) noexcept;

    /// @brief Notifies manager that Stage 8 Boss was neutralized.
    /// @param[in,out] course CourseManager reference.
    /// @param[in,out] audio AudioSystem reference.
    void handleBossVictory(CourseManager& course, AudioSystem& audio) noexcept;

    /// @brief Sets active game state.
    /// @param[in] state New GameState.
    void setState(GameState state) noexcept;

    /// @brief Gets active game state.
    /// @return Active GameState.
    [[nodiscard]] GameState getState() const noexcept;

    /// @brief Gets remaining player lives.
    /// @return Integer life count.
    [[nodiscard]] int getLives() const noexcept;

    /// @brief Sets remaining player lives.
    /// @param[in] lives Life count.
    void setLives(int lives) noexcept;

    /// @brief Gets remaining invulnerability shield duration.
    /// @return Duration in seconds.
    [[nodiscard]] double getInvulnerability() const noexcept;

    /// @brief Checks whether the truck is currently invulnerable.
    /// @return True if invulnerable shield is active.
    [[nodiscard]] bool isInvulnerable() const noexcept;

    /// @brief Resets campaign to Stage 1 with default lives and score.
    /// @param[in,out] course CourseManager reference.
    void startNewGame(CourseManager& course) noexcept;

    /// @brief Gets persistent high score coordinator.
    /// @return Constant reference to HighScoreManager.
    [[nodiscard]] const HighScoreManager& getHighScores() const noexcept;

    /// @brief Gets mutable high score coordinator.
    /// @return Mutable reference to HighScoreManager.
    [[nodiscard]] HighScoreManager& getHighScores() noexcept;

private:
    void renderTitleScreen(CgaPaletteRenderer& renderer) const noexcept;
    void renderMissionBriefing(CgaPaletteRenderer& renderer, const CourseManager& course) const noexcept;
    void renderInGameHud(CgaPaletteRenderer& renderer,
                         const CourseManager& course,
                         const vehicle_sim::physics::RigidBody2D& truckBody) const noexcept;
    void renderStageClear(CgaPaletteRenderer& renderer, const CourseManager& course) const noexcept;
    void renderGameOver(CgaPaletteRenderer& renderer) const noexcept;
    void renderNameEntry(CgaPaletteRenderer& renderer) const noexcept;
    void renderCampaignVictory(CgaPaletteRenderer& renderer) const noexcept;
    void renderHighScoreTable(CgaPaletteRenderer& renderer) const noexcept;

    GameState state_ { GameState::TitleScreen };
    HighScoreManager highScoreManager_ {};
    int lives_ { kStartingLives };
    int nextLifeThreshold_ { kExtraLifeInterval };
    double stateTimer_ { 0.0 };
    double bannerTimer_ { 0.0 };
    double invulnerableTimer_ { 0.0 };
    char lastCheckpointLetter_ { 'A' };
    int lastStageClearBonus_ { 0 };

    // High score name entry state
    int nameEntryCursor_ { 0 };
    std::array<char, 3> enteredInitials_ { 'A', 'A', 'A' };
    int pendingHighScore_ { 0 };
    int pendingStage_ { 1 };
};

} // namespace montsoft_offroad
