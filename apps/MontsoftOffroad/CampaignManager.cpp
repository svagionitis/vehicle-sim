#include "apps/MontsoftOffroad/CampaignManager.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace montsoft_offroad {

namespace {

struct StageBriefingIntel {
    const char* terrain;
    const char* threats;
};

constexpr std::array<StageBriefingIntel, 8> kStageIntel = {{
    { "ROLLING SAND DUNES & DEEP MOGULS", "SCOUT DIRT BIKES & BURIED MINES" },
    { "STEEP SANDSTONE WASHES & CHASMS", "FALLING BOULDERS & JET FIGHTERS" },
    { "HIGH-SPEED CRUST WITH LOW DRAG", "CHASE BUGGIES & BOUNCING DRONES" },
    { "DEEP BEKKER SOIL SINKAGE & RUTS", "PATROL HOVERCRAFT & SINKHOLES" },
    { "BASALT LAVA CRAGS & MAGMA CREVASSE", "TRACKED ARTILLERY & LAVA BOMBS" },
    { "TOXIC SLUDGE & JUNKYARD RUBBLE", "ARMORED GUNSHIPS & CAR BARRIERS" },
    { "LOOSE GRAVEL SLIDES & CLIFF FACES", "HIGH-ALTITUDE BOMBER ZEPPELINS" },
    { "CONCRETE BUNKERS & LASER APRON", "GIANT MECH HEAD & DREADNOUGHT" }
}};

} // namespace

CampaignManager::CampaignManager() noexcept
{
    highScoreManager_.loadScores();
}

void CampaignManager::update(double dt,
                             CourseManager& course,
                             AudioSystem& audio,
                             vehicle_sim::physics::RigidBody2D& truckBody,
                             vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics) noexcept
{
    stateTimer_ += dt;

    if (invulnerableTimer_ > 0.0) {
        invulnerableTimer_ = std::max(0.0, invulnerableTimer_ - dt);
    }
    if (bannerTimer_ > 0.0) {
        bannerTimer_ = std::max(0.0, bannerTimer_ - dt);
    }

    switch (state_) {
    case GameState::TitleScreen:
    case GameState::HighScores:
        break;

    case GameState::MissionBriefing:
        if (stateTimer_ >= 3.5) {
            state_ = GameState::Playing;
            stateTimer_ = 0.0;
            audio.setEngineSpeed(truckBody.getLinearVelocity().x, 0.0, false);
        }
        break;

    case GameState::Playing:
    case GameState::CheckpointPassed: {
        // Continuous engine sound feedback
        const auto& rearAxle { truckDynamics.getAxleState(0) };
        const auto& frontAxle { truckDynamics.getAxleState(1) };
        const bool isAirborne { !rearAxle.isGrounded && !frontAxle.isGrounded };
        audio.setEngineSpeed(truckBody.getLinearVelocity().x, truckDynamics.getInput().throttle, isAirborne);

        // Check for bonus life acquisition
        if (course.getScore() >= nextLifeThreshold_) {
            ++lives_;
            nextLifeThreshold_ += kExtraLifeInterval;
            audio.playSfx(SoundEffect::CheckpointChime);
        }

        // Check for milestone crossings
        for (const auto& cp : course.getCheckpoints()) {
            if (cp.reached && cp.letter != lastCheckpointLetter_) {
                lastCheckpointLetter_ = cp.letter;
                bannerTimer_ = 1.8;
                audio.playSfx(SoundEffect::CheckpointChime);
                break;
            }
        }

        // Check time expiration
        if (course.getRemainingTime() <= 0.0) {
            handleTruckCrash(course, audio, truckBody, truckDynamics);
        }

        // Check regular stage finish (Stages 1 through 7)
        if (course.isStageComplete() && course.getCurrentStageIndex() < 8) {
            handleStageClear(course, audio);
        }
        break;
    }

    case GameState::StageClear:
        if (stateTimer_ >= 4.0) {
            const int nextStage { course.getCurrentStageIndex() + 1 };
            if (nextStage <= 8) {
                course.loadStage(nextStage);
                lastCheckpointLetter_ = 'A';
                state_ = GameState::MissionBriefing;
                stateTimer_ = 0.0;

                // Reset truck position for start of next stage
                const double groundY { course.getHeightfield().getHeight(10.0) };
                truckBody.setPosition(vehicle_sim::math::Vec2{ 10.0, groundY + 0.8 });
                truckBody.setLinearVelocity(vehicle_sim::math::Vec2{ 0.0, 0.0 });
                truckBody.setAngularVelocity(0.0);
                truckBody.setOrientation(0.0);
                invulnerableTimer_ = 2.0;
            } else {
                handleBossVictory(course, audio);
            }
        }
        break;

    case GameState::GameOver:
        if (stateTimer_ >= 4.5) {
            if (highScoreManager_.isHighScore(pendingHighScore_)) {
                state_ = GameState::NameEntry;
                stateTimer_ = 0.0;
                nameEntryCursor_ = 0;
                enteredInitials_ = { 'A', 'A', 'A' };
            } else {
                state_ = GameState::TitleScreen;
                stateTimer_ = 0.0;
            }
        }
        break;

    case GameState::CampaignVictory:
        if (stateTimer_ >= 7.0) {
            if (highScoreManager_.isHighScore(pendingHighScore_)) {
                state_ = GameState::NameEntry;
                stateTimer_ = 0.0;
                nameEntryCursor_ = 0;
                enteredInitials_ = { 'A', 'A', 'A' };
            } else {
                state_ = GameState::TitleScreen;
                stateTimer_ = 0.0;
            }
        }
        break;

    case GameState::NameEntry:
        break;
    }
}

void CampaignManager::handleKeyPress(SDL_Keycode key, CourseManager& course, AudioSystem& audio) noexcept
{
    switch (state_) {
    case GameState::TitleScreen:
        if (key == SDLK_SPACE || key == SDLK_RETURN) {
            startNewGame(course);
        } else if (key == SDLK_h) {
            state_ = GameState::HighScores;
            stateTimer_ = 0.0;
        }
        break;

    case GameState::HighScores:
        if (key == SDLK_ESCAPE || key == SDLK_SPACE || key == SDLK_RETURN || key == SDLK_h) {
            state_ = GameState::TitleScreen;
            stateTimer_ = 0.0;
        }
        break;

    case GameState::MissionBriefing:
        if (key == SDLK_SPACE || key == SDLK_RETURN) {
            state_ = GameState::Playing;
            stateTimer_ = 0.0;
        }
        break;

    case GameState::StageClear:
        if ((key == SDLK_SPACE || key == SDLK_RETURN) && stateTimer_ > 1.0) {
            stateTimer_ = 4.0; // Advance immediately
        }
        break;

    case GameState::GameOver:
        if ((key == SDLK_SPACE || key == SDLK_RETURN) && stateTimer_ > 1.5) {
            stateTimer_ = 5.0; // Advance immediately
        }
        break;

    case GameState::CampaignVictory:
        if ((key == SDLK_SPACE || key == SDLK_RETURN) && stateTimer_ > 2.0) {
            stateTimer_ = 8.0; // Advance immediately
        }
        break;

    case GameState::NameEntry:
        if (key == SDLK_LEFT) {
            nameEntryCursor_ = (nameEntryCursor_ > 0) ? nameEntryCursor_ - 1 : 2;
            audio.playSfx(SoundEffect::ArmorClink);
        } else if (key == SDLK_RIGHT) {
            nameEntryCursor_ = (nameEntryCursor_ < 2) ? nameEntryCursor_ + 1 : 0;
            audio.playSfx(SoundEffect::ArmorClink);
        } else if (key == SDLK_UP) {
            char& c { enteredInitials_[static_cast<std::size_t>(nameEntryCursor_)] };
            c = (c == 'Z') ? 'A' : static_cast<char>(c + 1);
            audio.playSfx(SoundEffect::ArmorClink);
        } else if (key == SDLK_DOWN) {
            char& c { enteredInitials_[static_cast<std::size_t>(nameEntryCursor_)] };
            c = (c == 'A') ? 'Z' : static_cast<char>(c - 1);
            audio.playSfx(SoundEffect::ArmorClink);
        } else if (key == SDLK_RETURN || key == SDLK_SPACE) {
            highScoreManager_.submitScore(enteredInitials_, pendingHighScore_, pendingStage_);
            audio.playSfx(SoundEffect::StageClear);
            state_ = GameState::HighScores;
            stateTimer_ = 0.0;
        }
        break;

    case GameState::Playing:
    case GameState::CheckpointPassed:
    default:
        break;
    }
}

void CampaignManager::handleTruckCrash(CourseManager& course,
                                      AudioSystem& audio,
                                      vehicle_sim::physics::RigidBody2D& truckBody,
                                      vehicle_sim::dynamics::OffroadTruckDynamics& truckDynamics) noexcept
{
    static_cast<void>(truckDynamics);
    lives_ = std::max(0, lives_ - 1);
    audio.playSfx(SoundEffect::ExplosionBlast);

    if (lives_ == 0) {
        state_ = GameState::GameOver;
        stateTimer_ = 0.0;
        pendingHighScore_ = course.getScore();
        pendingStage_ = course.getCurrentStageIndex();
        audio.stopEngine();
        audio.playSfx(SoundEffect::GameOver);
    } else {
        // Respawn truck at most recently reached checkpoint marker or stage origin
        double respawnX { 10.0 };
        for (const auto& cp : course.getCheckpoints()) {
            if (cp.reached) {
                respawnX = cp.worldX + 2.0;
            }
        }

        const double groundY { course.getHeightfield().getHeight(respawnX) };
        truckBody.setPosition(vehicle_sim::math::Vec2{ respawnX, groundY + 0.8 });
        truckBody.setLinearVelocity(vehicle_sim::math::Vec2{ 0.0, 0.0 });
        truckBody.setAngularVelocity(0.0);
        truckBody.setOrientation(0.0);
        invulnerableTimer_ = 2.5;
    }
}

void CampaignManager::handleStageClear(CourseManager& course, AudioSystem& audio) noexcept
{
    state_ = GameState::StageClear;
    stateTimer_ = 0.0;
    audio.stopEngine();
    audio.playSfx(SoundEffect::StageClear);

    const int timeBonus { static_cast<int>(std::floor(std::max(0.0, course.getRemainingTime()))) * 100 };
    const int stageBonus { course.getCurrentStageIndex() * 5000 };
    lastStageClearBonus_ = timeBonus + stageBonus;
    course.addScore(lastStageClearBonus_);
}

void CampaignManager::handleBossVictory(CourseManager& course, AudioSystem& audio) noexcept
{
    state_ = GameState::CampaignVictory;
    stateTimer_ = 0.0;
    audio.stopEngine();
    audio.playSfx(SoundEffect::StageClear);

    const int grandBonus { 50000 + (lives_ * 10000) };
    course.addScore(grandBonus);
    pendingHighScore_ = course.getScore();
    pendingStage_ = 8;
}

void CampaignManager::startNewGame(CourseManager& course) noexcept
{
    lives_ = kStartingLives;
    nextLifeThreshold_ = kExtraLifeInterval;
    lastCheckpointLetter_ = 'A';
    invulnerableTimer_ = 2.0;
    stateTimer_ = 0.0;

    course.loadStage(1);
    course.reset();

    state_ = GameState::MissionBriefing;
}

void CampaignManager::setState(GameState state) noexcept
{
    state_ = state;
    stateTimer_ = 0.0;
}

GameState CampaignManager::getState() const noexcept
{
    return state_;
}

int CampaignManager::getLives() const noexcept
{
    return lives_;
}

void CampaignManager::setLives(int lives) noexcept
{
    lives_ = std::max(0, lives);
}

double CampaignManager::getInvulnerability() const noexcept
{
    return invulnerableTimer_;
}

bool CampaignManager::isInvulnerable() const noexcept
{
    return invulnerableTimer_ > 0.0;
}

const HighScoreManager& CampaignManager::getHighScores() const noexcept
{
    return highScoreManager_;
}

HighScoreManager& CampaignManager::getHighScores() noexcept
{
    return highScoreManager_;
}

void CampaignManager::render(CgaPaletteRenderer& renderer,
                             const CourseManager& course,
                             const vehicle_sim::physics::RigidBody2D& truckBody) const noexcept
{
    switch (state_) {
    case GameState::TitleScreen:
        renderTitleScreen(renderer);
        break;

    case GameState::MissionBriefing:
        renderMissionBriefing(renderer, course);
        break;

    case GameState::Playing:
    case GameState::CheckpointPassed:
        renderInGameHud(renderer, course, truckBody);
        break;

    case GameState::StageClear:
        renderInGameHud(renderer, course, truckBody);
        renderStageClear(renderer, course);
        break;

    case GameState::GameOver:
        renderGameOver(renderer);
        break;

    case GameState::NameEntry:
        renderNameEntry(renderer);
        break;

    case GameState::CampaignVictory:
        renderCampaignVictory(renderer);
        break;

    case GameState::HighScores:
        renderHighScoreTable(renderer);
        break;
    }
}

void CampaignManager::renderTitleScreen(CgaPaletteRenderer& renderer) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    // Decorative double border
    renderer.drawRect(4, 4, 312, 192, ColorRgba{ 85, 255, 85, 255 });
    renderer.drawRect(6, 6, 308, 188, ColorRgba{ 255, 85, 85, 255 });

    // Title header
    renderer.drawText(68, 22, "M O N T S O F T   O F F R O A D", ColorRgba{ 255, 255, 85, 255 });
    renderer.drawText(56, 34, "TERRESTRIAL COMBAT DESERT SIMULATOR", ColorRgba{ 85, 255, 85, 255 });
    renderer.drawLine(20, 46, 300, 46, ColorRgba{ 255, 85, 85, 255 });

    // Historical copyright and credits
    renderer.drawText(44, 60, "(C) 1988 MONTE VARIAKOJIS - SHAREWARE", ColorRgba{ 255, 85, 85, 255 });
    renderer.drawText(50, 72, "REIMAGINED IN C++17 VEHICLESIM ENGINE", ColorRgba{ 85, 255, 85, 255 });

    // Truck wireframe silhouette
    renderer.drawRect(120, 92, 80, 20, ColorRgba{ 255, 85, 85, 255 });
    renderer.drawRect(132, 82, 40, 10, ColorRgba{ 255, 255, 85, 255 });
    renderer.drawCircle(135, 115, 8, ColorRgba{ 255, 255, 85, 255 });
    renderer.drawCircle(185, 115, 8, ColorRgba{ 255, 255, 85, 255 });

    // Instructions
    renderer.drawText(82, 134, "PRESS  SPACE  TO START", ColorRgba{ 255, 255, 85, 255 });
    renderer.drawText(88, 148, "[H] VIEW HIGH SCORES", ColorRgba{ 85, 255, 85, 255 });

    // Control summary
    renderer.drawText(24, 168, "KEYS: A/D ACCEL  SPACE JUMP  Z ROCKET  X MORTAR", ColorRgba{ 255, 255, 85, 255 });
    renderer.drawText(36, 180, "F1 CGA-1  F2 CGA-0  F3 NTSC  F4 EGA  P PAUSE", ColorRgba{ 255, 85, 85, 255 });
}

void CampaignManager::renderMissionBriefing(CgaPaletteRenderer& renderer, const CourseManager& course) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    renderer.drawRect(8, 8, 304, 184, ColorRgba{ 85, 255, 255, 255 });
    renderer.drawRect(10, 10, 300, 180, ColorRgba{ 255, 85, 255, 255 });

    const auto& stage { course.getCurrentStage() };
    const int sIdx { std::clamp(stage.stageIndex, 1, 8) };

    std::ostringstream titleStream {};
    titleStream << "STAGE " << sIdx << ": " << stage.title;
    renderer.drawText(24, 24, titleStream.str(), ColorRgba{ 255, 255, 255, 255 });
    renderer.drawLine(24, 36, 296, 36, ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream biomeStream {};
    biomeStream << "BIOME:    " << stage.biome;
    renderer.drawText(24, 48, biomeStream.str(), ColorRgba{ 255, 85, 255, 255 });

    std::ostringstream parStream {};
    parStream << "PAR TIME: " << static_cast<int>(stage.parTime) << " SECONDS";
    renderer.drawText(24, 62, parStream.str(), ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream distStream {};
    distStream << "COURSE:   500 METERS (MILESTONES A - E)";
    renderer.drawText(24, 76, distStream.str(), ColorRgba{ 255, 255, 255, 255 });

    const auto& intel { kStageIntel[static_cast<std::size_t>(sIdx - 1)] };
    renderer.drawText(24, 98, "TERRAIN RECON:", ColorRgba{ 255, 85, 255, 255 });
    renderer.drawText(36, 110, intel.terrain, ColorRgba{ 255, 255, 255, 255 });

    renderer.drawText(24, 128, "HOSTILE THREATS:", ColorRgba{ 255, 85, 255, 255 });
    renderer.drawText(36, 140, intel.threats, ColorRgba{ 255, 255, 255, 255 });

    renderer.drawText(66, 168, "PRESS SPACE TO COMMENCE", ColorRgba{ 85, 255, 255, 255 });
}

void CampaignManager::renderInGameHud(CgaPaletteRenderer& renderer,
                                     const CourseManager& course,
                                     const vehicle_sim::physics::RigidBody2D& truckBody) const noexcept
{
    // Top background bar
    renderer.fillRect(0, 0, 320, 12, ColorRgba{ 0, 0, 0, 255 });
    renderer.drawLine(0, 12, 320, 12, ColorRgba{ 85, 255, 85, 255 });

    // Format telemetry items
    const int score { course.getScore() };
    const int hiScore { highScoreManager_.getRecords()[0].score };
    const int stageIdx { course.getCurrentStageIndex() };
    const int timeLeft { std::max(0, static_cast<int>(std::floor(course.getRemainingTime()))) };
    const double speedKmh { std::abs(truckBody.getLinearVelocity().x) * 3.6 };

    std::ostringstream hud {};
    hud << "SC:" << std::setw(6) << std::setfill('0') << score << " "
        << "HI:" << std::setw(6) << std::setfill('0') << hiScore << " "
        << "S" << stageIdx << " "
        << "T:" << std::setw(2) << timeLeft << "s "
        << static_cast<int>(speedKmh) << "KM/H ";

    renderer.drawText(4, 3, hud.str(), ColorRgba{ 255, 255, 85, 255 });

    // Lives icons
    renderer.drawText(268, 3, "TRK:", ColorRgba{ 255, 85, 85, 255 });
    for (int i { 0 }; i < std::min(5, lives_); ++i) {
        renderer.fillRect(296 + (i * 4), 4, 3, 5, ColorRgba{ 255, 85, 85, 255 });
    }

    // Bottom telemetry guide
    renderer.fillRect(0, 190, 320, 10, ColorRgba{ 0, 0, 0, 255 });
    renderer.drawLine(0, 190, 320, 190, ColorRgba{ 85, 255, 85, 255 });

    std::ostringstream bottomGuide {};
    const double distNext { course.getDistanceToNext() };
    bottomGuide << "[Z:RKT] [X:MTR]   NEXT "
                << course.getNextCheckpointLetter() << ": "
                << static_cast<int>(distNext) << "M";
    renderer.drawText(6, 192, bottomGuide.str(), ColorRgba{ 255, 255, 85, 255 });

    // Checkpoint banner overlay if bannerTimer_ is active
    if (bannerTimer_ > 0.0) {
        renderer.fillRect(50, 40, 220, 26, ColorRgba{ 0, 0, 0, 255 });
        renderer.drawRect(50, 40, 220, 26, ColorRgba{ 85, 255, 85, 255 });

        std::ostringstream bannerText {};
        bannerText << "POINT " << lastCheckpointLetter_ << " ATTAINED!";
        renderer.drawText(92, 45, bannerText.str(), ColorRgba{ 255, 255, 85, 255 });
        renderer.drawText(82, 54, "+15 SEC   +1,000 PTS", ColorRgba{ 255, 85, 85, 255 });
    }
}

void CampaignManager::renderStageClear(CgaPaletteRenderer& renderer, const CourseManager& course) const noexcept
{
    renderer.fillRect(40, 60, 240, 80, ColorRgba{ 0, 0, 0, 255 });
    renderer.drawRect(40, 60, 240, 80, ColorRgba{ 85, 255, 255, 255 });
    renderer.drawRect(42, 62, 236, 76, ColorRgba{ 255, 85, 255, 255 });

    renderer.drawText(86, 70, "** STAGE CLEAR! **", ColorRgba{ 255, 255, 255, 255 });

    std::ostringstream bonusStream {};
    bonusStream << "STAGE BONUS: +" << (course.getCurrentStageIndex() * 5000) << " PTS";
    renderer.drawText(60, 86, bonusStream.str(), ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream timeStream {};
    const int timeSec { std::max(0, static_cast<int>(std::floor(course.getRemainingTime()))) };
    timeStream << "TIME BONUS:  +" << (timeSec * 100) << " PTS";
    renderer.drawText(60, 98, timeStream.str(), ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream totalStream {};
    totalStream << "TOTAL BONUS: +" << lastStageClearBonus_ << " PTS";
    renderer.drawText(60, 112, totalStream.str(), ColorRgba{ 255, 255, 255, 255 });

    renderer.drawText(74, 126, "PRESS SPACE TO ADVANCE", ColorRgba{ 255, 85, 255, 255 });
}

void CampaignManager::renderGameOver(CgaPaletteRenderer& renderer) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    renderer.drawRect(20, 30, 280, 140, ColorRgba{ 255, 85, 255, 255 });
    renderer.drawRect(22, 32, 276, 136, ColorRgba{ 85, 255, 255, 255 });

    renderer.drawText(106, 55, "G A M E   O V E R", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawLine(40, 72, 280, 72, ColorRgba{ 255, 85, 255, 255 });

    std::ostringstream scoreStream {};
    scoreStream << "FINAL SCORE: " << pendingHighScore_ << " PTS";
    renderer.drawText(76, 88, scoreStream.str(), ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream stageStream {};
    stageStream << "STAGE ATTAINED: " << pendingStage_;
    renderer.drawText(82, 102, stageStream.str(), ColorRgba{ 255, 255, 255, 255 });

    if (highScoreManager_.isHighScore(pendingHighScore_)) {
        renderer.drawText(60, 124, "** NEW HIGH SCORE QUALIFIED! **", ColorRgba{ 255, 85, 255, 255 });
    }

    renderer.drawText(86, 145, "PRESS SPACE TO CONTINUE", ColorRgba{ 85, 255, 255, 255 });
}

void CampaignManager::renderNameEntry(CgaPaletteRenderer& renderer) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    renderer.drawRect(20, 20, 280, 160, ColorRgba{ 85, 255, 255, 255 });
    renderer.drawRect(22, 22, 276, 156, ColorRgba{ 255, 85, 255, 255 });

    renderer.drawText(72, 36, "GREATEST DESERT PILOTS", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawText(60, 48, "ENTER YOUR 3-LETTER INITIALS", ColorRgba{ 85, 255, 255, 255 });

    std::ostringstream scoreStream {};
    scoreStream << "SCORE: " << pendingHighScore_;
    renderer.drawText(106, 70, scoreStream.str(), ColorRgba{ 255, 85, 255, 255 });

    // Draw initials large
    const int startX { 128 };
    for (int i { 0 }; i < 3; ++i) {
        const int px { startX + (i * 24) };
        renderer.drawChar(px, 100, enteredInitials_[static_cast<std::size_t>(i)], ColorRgba{ 255, 255, 255, 255 });
        if (i == nameEntryCursor_) {
            // Blinking cursor underline
            renderer.fillRect(px - 1, 110, 8, 2, ColorRgba{ 85, 255, 255, 255 });
        }
    }

    renderer.drawText(40, 134, "UP/DOWN: CHANGE LETTER   LEFT/RIGHT: MOVE", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawText(76, 150, "PRESS ENTER TO CONFIRM", ColorRgba{ 85, 255, 255, 255 });
}

void CampaignManager::renderCampaignVictory(CgaPaletteRenderer& renderer) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    renderer.drawRect(10, 10, 300, 180, ColorRgba{ 255, 255, 255, 255 });
    renderer.drawRect(12, 12, 296, 176, ColorRgba{ 85, 255, 255, 255 });

    renderer.drawText(64, 25, "VICTORY! FORTRESS LIBERATED!", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawLine(30, 37, 290, 37, ColorRgba{ 255, 85, 255, 255 });

    renderer.drawText(42, 50, "THE DREADNOUGHT CRUISER HAS BEEN DESTROYED", ColorRgba{ 85, 255, 255, 255 });
    renderer.drawText(36, 62, "MONTSOFT SECRET FORTRESS IS SECURED FOREVER", ColorRgba{ 255, 255, 255, 255 });

    std::ostringstream victoryBonus {};
    victoryBonus << "GRAND MASTER BONUS: +50,000 PTS";
    renderer.drawText(56, 85, victoryBonus.str(), ColorRgba{ 255, 85, 255, 255 });

    std::ostringstream livesBonus {};
    livesBonus << "SURVIVING LIVES (" << lives_ << "x): +" << (lives_ * 10000) << " PTS";
    renderer.drawText(56, 98, livesBonus.str(), ColorRgba{ 85, 255, 255, 255 });

    renderer.drawText(48, 122, "TRIBUTE TO MONTE VARIAKOJIS (1988)", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawText(72, 134, "MS-DOS SHAREWARE LEGEND", ColorRgba{ 85, 255, 255, 255 });

    renderer.drawText(70, 165, "PRESS SPACE TO PROCEED", ColorRgba{ 85, 255, 255, 255 });
}

void CampaignManager::renderHighScoreTable(CgaPaletteRenderer& renderer) const noexcept
{
    renderer.clear(ColorRgba{ 0, 0, 0, 255 });

    renderer.drawRect(10, 10, 300, 180, ColorRgba{ 85, 255, 255, 255 });
    renderer.drawRect(12, 12, 296, 176, ColorRgba{ 255, 85, 255, 255 });

    renderer.drawText(66, 20, "MONTSOFT OFFROAD HALL OF FAME", ColorRgba{ 255, 255, 255, 255 });
    renderer.drawLine(24, 32, 296, 32, ColorRgba{ 255, 85, 255, 255 });

    renderer.drawText(30, 40, "RANK   PILOT    SCORE     STAGE", ColorRgba{ 85, 255, 255, 255 });
    renderer.drawLine(28, 50, 292, 50, ColorRgba{ 255, 255, 255, 255 });

    const auto& records { highScoreManager_.getRecords() };
    for (std::size_t i { 0 }; i < records.size(); ++i) {
        const auto& rec { records[i] };
        const int yPos { 56 + static_cast<int>(i * 10) };

        std::ostringstream row {};
        row << std::setw(2) << (i + 1) << ".     "
            << rec.initials.data() << "     "
            << std::setw(6) << std::setfill('0') << rec.score << "     "
            << "STAGE " << rec.stageReached;

        const ColorRgba rowColor { (i == 0) ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 85, 255, 255, 255 } };
        renderer.drawText(30, yPos, row.str(), rowColor);
    }

    renderer.drawText(78, 168, "PRESS ESC OR SPACE TO EXIT", ColorRgba{ 255, 85, 255, 255 });
}

} // namespace montsoft_offroad
