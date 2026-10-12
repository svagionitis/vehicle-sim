#include "GameApp.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "apps/EarthPatrol/PixelFont.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace earth_patrol {

namespace {
    inline constexpr double kMaxFrameTime { 0.25 };

    std::string formatScore(int value, int digits)
    {
        std::string s { std::to_string(std::max(0, value)) };
        if (static_cast<int>(s.length()) < digits) {
            s.insert(0, static_cast<std::size_t>(digits - static_cast<int>(s.length())), '0');
        }
        return s;
    }

    std::string formatTime(double seconds)
    {
        const int sec { static_cast<int>(seconds) };
        const int tenths { static_cast<int>((seconds - static_cast<double>(sec)) * 10.0) };
        return formatScore(sec, 3) + "." + std::to_string(tenths) + "S";
    }
} // namespace

GameApp::GameApp()
    : GameApp(GameAppConfig { })
{
}

GameApp::GameApp(const GameAppConfig& config)
    : config_ { config }
    , lives_ { config.initialLives }
{
    CameraConfig camConfig { };
    camConfig.viewportWidth = config_.windowWidth;
    camConfig.viewportHeight = config_.windowHeight;
    camera_.setConfig(camConfig);

    rover_.setTerrain(&course_.getHeightfield());

    vehicle_sim::physics::RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 850.0;
    bodyConfig.inertia = 650.0;
    bodyConfig.position = vehicle_sim::math::Vec2 { 0.0, 1.20 };
    roverBody_ = vehicle_sim::physics::RigidBody2D { bodyConfig };

    camera_.reset(roverBody_.getPosition());
    obstacles_.generateObstacles(course_, biomeIndex_);
}

bool GameApp::initialize()
{
    static_cast<void>(audio_.initialize(config_.headless));

    if (config_.headless) {
        return true;
    }

    if (!sdlContext_.isInitialized()) {
        return false;
    }

    SDL_Window* const rawWindow { SDL_CreateWindow(config_.windowTitle.c_str(), SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, config_.windowWidth, config_.windowHeight, SDL_WINDOW_SHOWN) };

    if (rawWindow == nullptr) {
        return false;
    }
    window_.reset(rawWindow);

    SDL_Renderer* const rawRenderer { SDL_CreateRenderer(
        window_.get(), -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC) };

    if (rawRenderer == nullptr) {
        return false;
    }
    renderer_.reset(rawRenderer);

    return true;
}

void GameApp::run()
{
    running_ = true;

    double accumulator { 0.0 };
    Uint64 lastTime { SDL_GetPerformanceCounter() };
    const double frequency { static_cast<double>(SDL_GetPerformanceFrequency()) };

    while (running_) {
        processEvents();

        const Uint64 currentTime { SDL_GetPerformanceCounter() };
        double frameTime { static_cast<double>(currentTime - lastTime) / frequency };
        lastTime = currentTime;

        if (frameTime > kMaxFrameTime) {
            frameTime = kMaxFrameTime;
        }

        accumulator += frameTime;
        while (accumulator >= config_.fixedDt) {
            updatePhysics(config_.fixedDt);
            accumulator -= config_.fixedDt;
        }

        if (!config_.headless && renderer_ != nullptr) {
            renderFrame();
        }
    }
}

void GameApp::stop() noexcept
{
    running_ = false;
}

void GameApp::startNewGame() noexcept
{
    score_ = 0;
    lives_ = config_.initialLives;
    stage_ = 1;
    stageTime_ = 0.0;
    totalTime_ = 0.0;
    biomeIndex_ = 0;
    lastCheckpointIdx_ = -1;

    course_.buildCourse(biomeIndex_);
    rover_.setTerrain(&course_.getHeightfield());
    obstacles_.generateObstacles(course_, biomeIndex_);

    roverBody_.setPosition(vehicle_sim::math::Vec2 { 0.0, 1.20 });
    roverBody_.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    roverBody_.setOrientation(0.0);
    roverBody_.setAngularVelocity(0.0);

    rover_.reset();
    camera_.reset(roverBody_.getPosition());
    particles_.reset();
    wasAirborne_ = false;
    audio_.playMilestone();
    crashed_ = false;
    state_ = GameState::Playing;
}

void GameApp::advanceStage() noexcept
{
    stage_++;
    stageTime_ = 0.0;
    biomeIndex_ = (biomeIndex_ + 1) % 3;
    lastCheckpointIdx_ = -1;

    course_.buildCourse(biomeIndex_);
    rover_.setTerrain(&course_.getHeightfield());
    obstacles_.generateObstacles(course_, biomeIndex_);

    roverBody_.setPosition(vehicle_sim::math::Vec2 { 0.0, 1.20 });
    roverBody_.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    roverBody_.setOrientation(0.0);
    roverBody_.setAngularVelocity(0.0);

    rover_.reset();
    camera_.reset(roverBody_.getPosition());
    particles_.reset();
    wasAirborne_ = false;
    audio_.playMilestone();
    crashed_ = false;
    state_ = GameState::Playing;
}

void GameApp::processEvents() noexcept
{
    SDL_Event event { };
    auto input { rover_.getInput() };

    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
            stop();
            return;
        }

        if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
            const auto key { event.key.keysym.sym };
            if (key == SDLK_ESCAPE) {
                stop();
                return;
            }

            if (state_ == GameState::TitleScreen) {
                if (key == SDLK_SPACE || key == SDLK_RETURN) {
                    startNewGame();
                    return;
                }
            } else if (state_ == GameState::StageClear) {
                if (key == SDLK_SPACE || key == SDLK_RETURN) {
                    advanceStage();
                    return;
                }
            } else if (state_ == GameState::GameOver) {
                if (key == SDLK_SPACE || key == SDLK_RETURN) {
                    state_ = GameState::TitleScreen;
                    return;
                }
            } else if (state_ == GameState::Playing) {
                switch (key) {
                case SDLK_SPACE:
                case SDLK_w:
                case SDLK_UP:
                    input.jump = true;
                    audio_.playJump();
                    particles_.emitThrusterSparks(roverBody_.getPosition() - vehicle_sim::math::Vec2 { 0.8, 0.4 }, 6);
                    break;
                case SDLK_d:
                case SDLK_RIGHT:
                    input.throttle = 1.0;
                    break;
                case SDLK_a:
                case SDLK_LEFT:
                    input.throttle = -0.5;
                    break;
                case SDLK_s:
                case SDLK_DOWN:
                    input.brake = 1.0;
                    break;
                case SDLK_b:
                    setBiomeIndex((biomeIndex_ + 1) % 3);
                    break;
                case SDLK_r:
                    respawnRover();
                    break;
                default:
                    break;
                }
            }
        } else if (event.type == SDL_KEYUP && state_ == GameState::Playing) {
            switch (event.key.keysym.sym) {
            case SDLK_SPACE:
            case SDLK_w:
            case SDLK_UP:
                input.jump = false;
                break;
            case SDLK_d:
            case SDLK_RIGHT:
            case SDLK_a:
            case SDLK_LEFT:
                input.throttle = 0.0;
                break;
            case SDLK_s:
            case SDLK_DOWN:
                input.brake = 0.0;
                break;
            default:
                break;
            }
        }
    }

    if (state_ == GameState::Playing) {
        rover_.setInput(input);
    }
}

void GameApp::updatePhysics(double dt) noexcept
{
    particles_.update(dt);

    if (state_ == GameState::TitleScreen) {
        parallax_.update(dt);
        return;
    }

    if (state_ == GameState::StageClear) {
        stageClearTimer_ += dt;
        parallax_.update(dt);
        return;
    }

    if (state_ == GameState::GameOver) {
        parallax_.update(dt);
        return;
    }

    if (state_ == GameState::Crashed) {
        respawnTimer_ -= dt;
        parallax_.update(dt);
        if (respawnTimer_ <= 0.0) {
            if (lives_ > 0) {
                respawnRover();
                state_ = GameState::Playing;
            } else {
                state_ = GameState::GameOver;
                if (score_ > highScore_) {
                    highScore_ = score_;
                }
            }
        }
        return;
    }

    // Active playing state
    stageTime_ += dt;
    totalTime_ += dt;
    const int prevCleared { obstacles_.getClearedCount() };

    rover_.step(roverBody_, dt);
    vehicle_sim::physics::Integrator2D::integrate(roverBody_, dt);
    roverBody_.clearForces();

    course_.update(roverBody_.getPosition().x, dt);
    camera_.update(roverBody_.getPosition(), rover_.getSpeed(), dt);
    parallax_.update(dt);

    // Ground contact evaluation, tire dust, and landing thump
    bool currentlyGrounded { false };
    for (std::size_t i { 0 }; i < 3; ++i) {
        const auto& ws { rover_.getWheelState(i) };
        if (ws.isGrounded) {
            currentlyGrounded = true;
            if (std::abs(rover_.getSpeed()) > 1.2) {
                particles_.emitTireDust(ws.contactPoint, biomeIndex_, 1);
            }
        }
    }
    if (wasAirborne_ && currentlyGrounded) {
        audio_.playLanding();
        camera_.addScreenShake(0.18, 0.22);
        particles_.emitLandingDust(roverBody_.getPosition() - vehicle_sim::math::Vec2 { 0.0, 0.35 }, biomeIndex_);
    }
    wasAirborne_ = !currentlyGrounded;

    // Milestone checkpoints reached
    const auto& checkpoints { course_.getCheckpoints() };
    for (int i { 0 }; i < static_cast<int>(checkpoints.size()); ++i) {
        if (checkpoints[static_cast<std::size_t>(i)].reached && i > lastCheckpointIdx_) {
            lastCheckpointIdx_ = i;
            score_ += (i + 1) * 1000;
            audio_.playMilestone();
            if (score_ > highScore_) {
                highScore_ = score_;
            }
        }
    }

    // Obstacle collision and passing
    bool fatal { false };
    if (obstacles_.checkCollision(roverBody_, rover_, fatal)) {
        if (fatal) {
            crashed_ = true;
            respawnTimer_ = 1.6;
            lives_ = std::max(0, lives_ - 1);
            camera_.addScreenShake(0.60, 0.60);
            particles_.emitCrashExplosion(roverBody_.getPosition());
            audio_.playCrash();
            state_ = GameState::Crashed;
            return;
        }
    }

    const int newCleared { obstacles_.getClearedCount() };
    if (newCleared > prevCleared) {
        score_ += (newCleared - prevCleared) * 200;
        if (score_ > highScore_) {
            highScore_ = score_;
        }
    }

    // Course completion (Milestone E reached at 500m)
    if (course_.isStageComplete()) {
        constexpr double parTime { 45.0 };
        timeBonus_ = std::max(0, static_cast<int>((parTime - stageTime_) * 100.0));
        score_ += 5000 + timeBonus_;
        audio_.playStageClear();
        if (score_ > highScore_) {
            highScore_ = score_;
        }
        state_ = GameState::StageClear;
        stageClearTimer_ = 0.0;
    }
}

void GameApp::respawnRover() noexcept
{
    double respawnX { 0.0 };
    for (const auto& cp : course_.getCheckpoints()) {
        if (cp.reached) {
            respawnX = cp.worldX;
        }
    }

    const double elev { course_.getHeightfield().getHeight(respawnX) };
    roverBody_.setPosition(vehicle_sim::math::Vec2 { respawnX, elev + 1.20 });
    roverBody_.setLinearVelocity(vehicle_sim::math::Vec2 { 0.0, 0.0 });
    roverBody_.setOrientation(0.0);
    roverBody_.setAngularVelocity(0.0);

    rover_.reset();
    camera_.reset(roverBody_.getPosition());
    crashed_ = false;
}

void GameApp::setBiomeIndex(int biomeIndex)
{
    biomeIndex_ = std::clamp(biomeIndex, 0, 2);
    course_.buildCourse(biomeIndex_);
    rover_.setTerrain(&course_.getHeightfield());
    obstacles_.generateObstacles(course_, biomeIndex_);
    respawnRover();
}

int GameApp::getBiomeIndex() const noexcept
{
    return biomeIndex_;
}

GameState GameApp::getState() const noexcept
{
    return state_;
}

void GameApp::setState(GameState state) noexcept
{
    state_ = state;
}

int GameApp::getScore() const noexcept
{
    return score_;
}

int GameApp::getHighScore() const noexcept
{
    return highScore_;
}

double GameApp::getStageTime() const noexcept
{
    return stageTime_;
}

int GameApp::getStage() const noexcept
{
    return stage_;
}

void GameApp::renderFrame() noexcept
{
    if (renderer_ == nullptr) {
        return;
    }

    parallax_.render(renderer_.get(), camera_, biomeIndex_);
    renderTerrain();
    obstacles_.render(renderer_.get(), camera_);
    renderCheckpoints();
    particles_.render(renderer_.get(), camera_);

    if (state_ == GameState::TitleScreen) {
        renderRover();
        renderHUD();
        renderTitleScreen();
    } else if (state_ == GameState::Playing) {
        if (!crashed_) {
            renderRover();
        } else {
            renderCrashPuff();
        }
        renderHUD();
    } else if (state_ == GameState::Crashed) {
        renderCrashPuff();
        renderHUD();
    } else if (state_ == GameState::StageClear) {
        renderRover();
        renderHUD();
        renderStageClear();
    } else if (state_ == GameState::GameOver) {
        renderHUD();
        renderGameOver();
    }

    SDL_RenderPresent(renderer_.get());
}

void GameApp::renderTerrain() noexcept
{
    const double minX { camera_.getVisibleMinX() - 4.0 };
    const double maxX { camera_.getVisibleMaxX() + 4.0 };
    constexpr double dx { 0.40 };

    const auto& heightfield { course_.getHeightfield() };
    const int screenBottom { config_.windowHeight };

    double prevX { minX };
    auto prevSample { heightfield.sampleSurface(prevX) };
    auto prevScreen { camera_.worldToScreen(vehicle_sim::math::Vec2 { prevX, prevSample.elevation }) };

    for (double x { minX + dx }; x <= maxX; x += dx) {
        const auto sample { heightfield.sampleSurface(x) };
        const auto screen { camera_.worldToScreen(vehicle_sim::math::Vec2 { x, sample.elevation }) };

        if (biomeIndex_ == 1) {
            SDL_SetRenderDrawColor(renderer_.get(), 110, 52, 34, 255);
        } else if (biomeIndex_ == 2) {
            SDL_SetRenderDrawColor(renderer_.get(), 45, 52, 40, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 48, 42, 40, 255);
        }

        const int topY { static_cast<int>(std::min(prevScreen.y, screen.y)) };
        const int fillHeight { std::max(0, screenBottom - topY) };
        const int leftX { static_cast<int>(prevScreen.x) };
        const int width { static_cast<int>(std::max(1.0, screen.x - prevScreen.x + 1.0)) };
        SDL_Rect soilRect { leftX, topY, width, fillHeight };
        SDL_RenderFillRect(renderer_.get(), &soilRect);

        if (biomeIndex_ == 1) {
            SDL_SetRenderDrawColor(renderer_.get(), 205, 125, 65, 255);
        } else if (biomeIndex_ == 2) {
            SDL_SetRenderDrawColor(renderer_.get(), 75, 155, 65, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 92, 168, 64, 255);
        }

        SDL_RenderDrawLine(renderer_.get(), static_cast<int>(prevScreen.x), static_cast<int>(prevScreen.y),
            static_cast<int>(screen.x), static_cast<int>(screen.y));

        prevX = x;
        prevSample = sample;
        prevScreen = screen;
    }
}

void GameApp::renderCheckpoints() noexcept
{
    const double ppm { camera_.getConfig().pixelsPerMeter };

    for (const auto& cp : course_.getCheckpoints()) {
        const double elev { course_.getHeightfield().getHeight(cp.worldX) };
        const auto screenBase { camera_.worldToScreen(vehicle_sim::math::Vec2 { cp.worldX, elev }) };

        const int pylonX { static_cast<int>(screenBase.x) };
        const int pylonBottomY { static_cast<int>(screenBase.y) };
        const int pylonHeight { static_cast<int>(3.2 * ppm) };
        const int pylonTopY { pylonBottomY - pylonHeight };

        SDL_SetRenderDrawColor(renderer_.get(), 200, 210, 220, 255);
        SDL_RenderDrawLine(renderer_.get(), pylonX, pylonBottomY, pylonX, pylonTopY);
        SDL_RenderDrawLine(renderer_.get(), pylonX + 1, pylonBottomY, pylonX + 1, pylonTopY);

        if (cp.reached) {
            SDL_SetRenderDrawColor(renderer_.get(), 50, 220, 90, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 240, 190, 40, 255);
        }
        SDL_Rect flagRect { pylonX, pylonTopY, 26, 16 };
        SDL_RenderFillRect(renderer_.get(), &flagRect);

        // Render milestone letter flag
        PixelFont::drawText(renderer_.get(), std::string(1, cp.letter), pylonX + 8, pylonTopY + 3, 2,
            { 20, 25, 35, 255 });
    }
}

void GameApp::renderRover() noexcept
{
    const auto roverPos { roverBody_.getPosition() };
    const auto roverScreen { camera_.worldToScreen(roverPos) };
    const int roverX { static_cast<int>(roverScreen.x) };
    const int roverY { static_cast<int>(roverScreen.y) };

    const auto& roverConfig { rover_.getConfig() };

    for (std::size_t i { 0 }; i < 3; ++i) {
        const double wheelXOffset { roverConfig.wheels[i].longitudinalOffset };
        const auto& ws { rover_.getWheelState(i) };
        const auto hubScreen { camera_.worldToScreen(ws.worldPos) };

        SDL_SetRenderDrawColor(renderer_.get(), 180, 185, 195, 255);
        SDL_RenderDrawLine(renderer_.get(), roverX + static_cast<int>(wheelXOffset * 22.0),
            roverY, static_cast<int>(hubScreen.x), static_cast<int>(hubScreen.y));

        SDL_SetRenderDrawColor(renderer_.get(), 30, 32, 40, 255);
        SDL_Rect tireRect { static_cast<int>(hubScreen.x) - 10, static_cast<int>(hubScreen.y) - 10, 20, 20 };
        SDL_RenderFillRect(renderer_.get(), &tireRect);

        SDL_SetRenderDrawColor(renderer_.get(), 140, 150, 165, 255);
        SDL_Rect rimRect { static_cast<int>(hubScreen.x) - 5, static_cast<int>(hubScreen.y) - 5, 10, 10 };
        SDL_RenderFillRect(renderer_.get(), &rimRect);
    }

    SDL_SetRenderDrawColor(renderer_.get(), 220, 160, 40, 255);
    SDL_Rect chassisRect { roverX - 44, roverY - 14, 88, 24 };
    SDL_RenderFillRect(renderer_.get(), &chassisRect);

    SDL_SetRenderDrawColor(renderer_.get(), 190, 130, 30, 255);
    SDL_Rect noseRect { roverX + 24, roverY - 8, 24, 14 };
    SDL_RenderFillRect(renderer_.get(), &noseRect);

    SDL_SetRenderDrawColor(renderer_.get(), 100, 210, 255, 255);
    SDL_Rect cockpitRect { roverX - 8, roverY - 24, 28, 12 };
    SDL_RenderFillRect(renderer_.get(), &cockpitRect);

    SDL_SetRenderDrawColor(renderer_.get(), 60, 65, 75, 255);
    SDL_Rect exhaustRect { roverX - 48, roverY - 10, 8, 12 };
    SDL_RenderFillRect(renderer_.get(), &exhaustRect);
}

void GameApp::renderCrashPuff() noexcept
{
    const auto roverScreen { camera_.worldToScreen(roverBody_.getPosition()) };
    const int rx { static_cast<int>(roverScreen.x) };
    const int ry { static_cast<int>(roverScreen.y) };

    const double progress { 1.6 - respawnTimer_ };
    const int radius { static_cast<int>(progress * 35.0) };

    SDL_SetRenderDrawColor(renderer_.get(), 220, 100, 30, 220);
    SDL_Rect puffCore { rx - radius, ry - radius, radius * 2, radius * 2 };
    SDL_RenderFillRect(renderer_.get(), &puffCore);

    SDL_SetRenderDrawColor(renderer_.get(), 240, 200, 50, 255);
    SDL_Rect sparkA { rx - (radius / 2), ry - radius - 8, radius, 12 };
    SDL_Rect sparkB { rx + radius, ry - (radius / 2), 14, 14 };
    SDL_RenderFillRect(renderer_.get(), &sparkA);
    SDL_RenderFillRect(renderer_.get(), &sparkB);
}

void GameApp::renderHUD() noexcept
{
    // Top-left: Player 1 Score & High Score
    const std::string scoreStr { "1P " + formatScore(score_, 6) };
    const std::string hiStr { "HI " + formatScore(highScore_, 6) };
    PixelFont::drawText(renderer_.get(), scoreStr, 24, 16, 2, { 255, 255, 255, 255 });
    PixelFont::drawText(renderer_.get(), hiStr, 24, 32, 2, { 255, 220, 50, 255 });

    // Top-right: Stage & Timer
    const std::string stageStr { "STAGE " + std::to_string(stage_) };
    const std::string timeStr { "TIME " + formatTime(stageTime_) };
    const int stageX { config_.windowWidth - 160 };
    PixelFont::drawText(renderer_.get(), stageStr, stageX, 16, 2, { 100, 220, 255, 255 });
    PixelFont::drawText(renderer_.get(), timeStr, stageX, 32, 2, { 240, 240, 240, 255 });

    // Center Course progress bar
    constexpr int barWidth { 500 };
    constexpr int barHeight { 8 };
    const int barX { (config_.windowWidth - barWidth) / 2 };
    constexpr int barY { 28 };

    SDL_SetRenderDrawColor(renderer_.get(), 40, 48, 65, 255);
    SDL_Rect bgBar { barX, barY, barWidth, barHeight };
    SDL_RenderFillRect(renderer_.get(), &bgBar);

    const int fillW { static_cast<int>(course_.getProgressFraction() * static_cast<double>(barWidth)) };
    SDL_SetRenderDrawColor(renderer_.get(), 80, 210, 95, 255);
    SDL_Rect fillBar { barX, barY, fillW, barHeight };
    SDL_RenderFillRect(renderer_.get(), &fillBar);

    // Checkpoint letter markers A-E along progress bar
    const auto& checkpoints { course_.getCheckpoints() };
    for (std::size_t i { 0 }; i < checkpoints.size(); ++i) {
        const double frac { static_cast<double>(i + 1) / 5.0 };
        const int tickX { barX + static_cast<int>(frac * static_cast<double>(barWidth)) };

        SDL_Color tickColor { 240, 190, 40, 255 };
        if (checkpoints[i].reached) {
            tickColor = SDL_Color { 60, 240, 100, 255 };
        }
        SDL_SetRenderDrawColor(renderer_.get(), tickColor.r, tickColor.g, tickColor.b, 255);
        SDL_Rect tickRect { tickX - 2, barY - 4, 4, barHeight + 8 };
        SDL_RenderFillRect(renderer_.get(), &tickRect);

        // Milestone letter label above tick
        PixelFont::drawText(renderer_.get(), std::string(1, checkpoints[i].letter), tickX - 5, barY - 14, 2, tickColor);
    }

    // Rover marker icon on progress bar
    const int roverMarkerX { barX + fillW };
    SDL_SetRenderDrawColor(renderer_.get(), 255, 240, 80, 255);
    SDL_Rect roverMarker { roverMarkerX - 4, barY - 6, 8, 20 };
    SDL_RenderFillRect(renderer_.get(), &roverMarker);

    // Rover life silhouettes (top-left below score)
    PixelFont::drawText(renderer_.get(), "ROVERS", 24, 52, 2, { 180, 190, 205, 255 });
    for (int i { 0 }; i < lives_; ++i) {
        SDL_SetRenderDrawColor(renderer_.get(), 240, 180, 45, 255);
        SDL_Rect lifeRect { 92 + (i * 22), 52, 14, 10 };
        SDL_RenderFillRect(renderer_.get(), &lifeRect);
    }

    // Speed indicator bar (bottom-left)
    const double speedKmh { std::max(0.0, rover_.getSpeed() * 3.6) };
    const int speedBarW { static_cast<int>(std::clamp(speedKmh * 2.0, 0.0, 200.0)) };

    const std::string speedStr { "SPEED: " + std::to_string(static_cast<int>(speedKmh)) + " KM/H" };
    PixelFont::drawText(renderer_.get(), speedStr, 24, config_.windowHeight - 56, 2, { 240, 190, 50, 255 });

    SDL_SetRenderDrawColor(renderer_.get(), 40, 45, 60, 255);
    SDL_Rect speedBg { 24, config_.windowHeight - 38, 200, 14 };
    SDL_RenderFillRect(renderer_.get(), &speedBg);

    SDL_SetRenderDrawColor(renderer_.get(), 240, 160, 30, 255);
    SDL_Rect speedFill { 24, config_.windowHeight - 38, speedBarW, 14 };
    SDL_RenderFillRect(renderer_.get(), &speedFill);

    // Biome badge (bottom-right)
    std::string biomeName { "HIGHWAY ZONE" };
    if (biomeIndex_ == 1) {
        biomeName = "DESERT BADLANDS";
    } else if (biomeIndex_ == 2) {
        biomeName = "ALPINE PASS";
    }
    const int badgeW { PixelFont::getTextWidth(biomeName, 2) };
    PixelFont::drawText(renderer_.get(), biomeName, config_.windowWidth - badgeW - 24,
        config_.windowHeight - 38, 2, { 160, 200, 220, 255 });
}

void GameApp::renderTitleScreen() noexcept
{
    SDL_SetRenderDrawBlendMode(renderer_.get(), SDL_BLENDMODE_BLEND);

    constexpr int cardW { 680 };
    constexpr int cardH { 480 };
    const int cardX { (config_.windowWidth - cardW) / 2 };
    const int cardY { (config_.windowHeight - cardH) / 2 };

    SDL_SetRenderDrawColor(renderer_.get(), 12, 16, 28, 220);
    SDL_Rect cardBg { cardX, cardY, cardW, cardH };
    SDL_RenderFillRect(renderer_.get(), &cardBg);

    SDL_SetRenderDrawColor(renderer_.get(), 255, 180, 35, 255);
    SDL_RenderDrawRect(renderer_.get(), &cardBg);

    // Title banner
    const std::string title { "EARTH PATROL" };
    const int titleW { PixelFont::getTextWidth(title, 7) };
    PixelFont::drawText(renderer_.get(), title, (config_.windowWidth - titleW) / 2, cardY + 36, 7,
        { 255, 190, 40, 255 });

    const std::string subTitle { "TERRESTRIAL SURFACE DEFENSE SQUADRON" };
    const int subW { PixelFont::getTextWidth(subTitle, 2) };
    PixelFont::drawText(renderer_.get(), subTitle, (config_.windowWidth - subW) / 2, cardY + 98, 2,
        { 100, 220, 255, 255 });

    // Start prompt
    const std::string startPrompt { "PRESS SPACE OR ENTER TO START" };
    const int promptW { PixelFont::getTextWidth(startPrompt, 3) };
    PixelFont::drawText(renderer_.get(), startPrompt, (config_.windowWidth - promptW) / 2, cardY + 150, 3,
        { 255, 255, 255, 255 });

    // Mission controls box
    const int boxX { cardX + 40 };
    const int boxY { cardY + 210 };
    SDL_SetRenderDrawColor(renderer_.get(), 25, 32, 50, 255);
    SDL_Rect boxRect { boxX, boxY, cardW - 80, 190 };
    SDL_RenderFillRect(renderer_.get(), &boxRect);

    PixelFont::drawText(renderer_.get(), "MISSION DIRECTIVE & CONTROLS:", boxX + 20, boxY + 16, 2,
        { 255, 220, 50, 255 });
    PixelFont::drawText(renderer_.get(), "[D] / [RIGHT]  ACCELERATE", boxX + 20, boxY + 48, 2,
        { 220, 230, 245, 255 });
    PixelFont::drawText(renderer_.get(), "[A] / [LEFT]   BRAKE / REVERSE", boxX + 20, boxY + 76, 2,
        { 220, 230, 245, 255 });
    PixelFont::drawText(renderer_.get(), "[SPACE] / [W]  VERTICAL JUMP THRUSTERS", boxX + 20, boxY + 104, 2,
        { 220, 230, 245, 255 });
    PixelFont::drawText(renderer_.get(), "[B] CYCLE BIOME   [ESC] ABORT MISSION", boxX + 20, boxY + 132, 2,
        { 180, 190, 205, 255 });
    PixelFont::drawText(renderer_.get(), "CLEAR CRATERS & REACH MILESTONE E", boxX + 20, boxY + 160, 2,
        { 80, 240, 140, 255 });

    const std::string hiNotice { "BEST RECORD: " + formatScore(highScore_, 6) + " PTS" };
    const int hiNoticeW { PixelFont::getTextWidth(hiNotice, 2) };
    PixelFont::drawText(renderer_.get(), hiNotice, (config_.windowWidth - hiNoticeW) / 2, cardY + 430, 2,
        { 255, 220, 50, 255 });
}

void GameApp::renderStageClear() noexcept
{
    SDL_SetRenderDrawBlendMode(renderer_.get(), SDL_BLENDMODE_BLEND);

    constexpr int cardW { 600 };
    constexpr int cardH { 420 };
    const int cardX { (config_.windowWidth - cardW) / 2 };
    const int cardY { (config_.windowHeight - cardH) / 2 };

    SDL_SetRenderDrawColor(renderer_.get(), 12, 24, 18, 230);
    SDL_Rect cardBg { cardX, cardY, cardW, cardH };
    SDL_RenderFillRect(renderer_.get(), &cardBg);

    SDL_SetRenderDrawColor(renderer_.get(), 60, 240, 110, 255);
    SDL_RenderDrawRect(renderer_.get(), &cardBg);

    const std::string clearTitle { "STAGE CLEAR!" };
    const int titleW { PixelFont::getTextWidth(clearTitle, 6) };
    PixelFont::drawText(renderer_.get(), clearTitle, (config_.windowWidth - titleW) / 2, cardY + 40, 6,
        { 60, 240, 110, 255 });

    const std::string reachedStr { "MILESTONE E REACHED" };
    const int reachW { PixelFont::getTextWidth(reachedStr, 3) };
    PixelFont::drawText(renderer_.get(), reachedStr, (config_.windowWidth - reachW) / 2, cardY + 110, 3,
        { 255, 255, 255, 255 });

    const std::string timeStr { "STAGE TIME: " + formatTime(stageTime_) };
    const std::string parStr { "PAR TIME:   045.0S" };
    const std::string bonusStr { "TIME BONUS: +" + std::to_string(timeBonus_) + " PTS" };
    const std::string scoreStr { "TOTAL SCORE: " + formatScore(score_, 6) + " PTS" };

    PixelFont::drawText(renderer_.get(), timeStr, cardX + 110, cardY + 170, 3, { 220, 230, 245, 255 });
    PixelFont::drawText(renderer_.get(), parStr, cardX + 110, cardY + 210, 3, { 240, 190, 40, 255 });
    PixelFont::drawText(renderer_.get(), bonusStr, cardX + 110, cardY + 250, 3, { 60, 240, 100, 255 });
    PixelFont::drawText(renderer_.get(), scoreStr, cardX + 110, cardY + 290, 3, { 255, 220, 50, 255 });

    const std::string advancePrompt { "PRESS SPACE FOR NEXT BIOME" };
    const int advW { PixelFont::getTextWidth(advancePrompt, 3) };
    PixelFont::drawText(renderer_.get(), advancePrompt, (config_.windowWidth - advW) / 2, cardY + 355, 3,
        { 100, 220, 255, 255 });
}

void GameApp::renderGameOver() noexcept
{
    SDL_SetRenderDrawBlendMode(renderer_.get(), SDL_BLENDMODE_BLEND);

    constexpr int cardW { 580 };
    constexpr int cardH { 380 };
    const int cardX { (config_.windowWidth - cardW) / 2 };
    const int cardY { (config_.windowHeight - cardH) / 2 };

    SDL_SetRenderDrawColor(renderer_.get(), 28, 12, 14, 230);
    SDL_Rect cardBg { cardX, cardY, cardW, cardH };
    SDL_RenderFillRect(renderer_.get(), &cardBg);

    SDL_SetRenderDrawColor(renderer_.get(), 240, 50, 40, 255);
    SDL_RenderDrawRect(renderer_.get(), &cardBg);

    const std::string goTitle { "GAME OVER" };
    const int titleW { PixelFont::getTextWidth(goTitle, 7) };
    PixelFont::drawText(renderer_.get(), goTitle, (config_.windowWidth - titleW) / 2, cardY + 40, 7,
        { 240, 50, 40, 255 });

    const std::string lostStr { "ALL ROVER UNITS LOST" };
    const int lostW { PixelFont::getTextWidth(lostStr, 3) };
    PixelFont::drawText(renderer_.get(), lostStr, (config_.windowWidth - lostW) / 2, cardY + 120, 3,
        { 255, 255, 255, 255 });

    const std::string finalScore { "FINAL SCORE: " + formatScore(score_, 6) + " PTS" };
    const std::string bestScore { "HIGH SCORE:  " + formatScore(highScore_, 6) + " PTS" };
    const int fsW { PixelFont::getTextWidth(finalScore, 3) };
    const int bsW { PixelFont::getTextWidth(bestScore, 3) };
    PixelFont::drawText(renderer_.get(), finalScore, (config_.windowWidth - fsW) / 2, cardY + 180, 3,
        { 255, 220, 50, 255 });
    PixelFont::drawText(renderer_.get(), bestScore, (config_.windowWidth - bsW) / 2, cardY + 225, 3,
        { 240, 190, 40, 255 });

    const std::string returnPrompt { "PRESS SPACE FOR TITLE SCREEN" };
    const int retW { PixelFont::getTextWidth(returnPrompt, 3) };
    PixelFont::drawText(renderer_.get(), returnPrompt, (config_.windowWidth - retW) / 2, cardY + 310, 3,
        { 100, 220, 255, 255 });
}

bool GameApp::isRunning() const noexcept
{
    return running_;
}

const GameAppConfig& GameApp::getConfig() const noexcept
{
    return config_;
}

const vehicle_sim::dynamics::EarthRoverDynamics& GameApp::getRover() const noexcept
{
    return rover_;
}

vehicle_sim::dynamics::EarthRoverDynamics& GameApp::getRover() noexcept
{
    return rover_;
}

const vehicle_sim::physics::RigidBody2D& GameApp::getRoverBody() const noexcept
{
    return roverBody_;
}

vehicle_sim::physics::RigidBody2D& GameApp::getRoverBody() noexcept
{
    return roverBody_;
}

const Camera2D& GameApp::getCamera() const noexcept
{
    return camera_;
}

Camera2D& GameApp::getCamera() noexcept
{
    return camera_;
}

const CourseManager& GameApp::getCourse() const noexcept
{
    return course_;
}

CourseManager& GameApp::getCourse() noexcept
{
    return course_;
}

const ObstacleManager& GameApp::getObstacles() const noexcept
{
    return obstacles_;
}

int GameApp::getLives() const noexcept
{
    return lives_;
}

bool GameApp::isCrashed() const noexcept
{
    return crashed_;
}

const ParticleSystem& GameApp::getParticles() const noexcept
{
    return particles_;
}

ParticleSystem& GameApp::getParticles() noexcept
{
    return particles_;
}

const AudioSystem& GameApp::getAudio() const noexcept
{
    return audio_;
}

AudioSystem& GameApp::getAudio() noexcept
{
    return audio_;
}

} // namespace earth_patrol
