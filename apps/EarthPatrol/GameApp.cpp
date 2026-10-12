#include "GameApp.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace earth_patrol {

namespace {
    inline constexpr double kMaxFrameTime { 0.25 };
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
            switch (event.key.keysym.sym) {
            case SDLK_ESCAPE:
                stop();
                return;
            case SDLK_SPACE:
            case SDLK_w:
            case SDLK_UP:
                input.jump = true;
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
                // Cycle Earth biomes (Highway -> Desert -> Alpine)
                setBiomeIndex((biomeIndex_ + 1) % 3);
                break;
            case SDLK_r:
                respawnRover();
                break;
            default:
                break;
            }
        } else if (event.type == SDL_KEYUP) {
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

    rover_.setInput(input);
}

void GameApp::updatePhysics(double dt) noexcept
{
    if (crashed_) {
        respawnTimer_ -= dt;
        if (respawnTimer_ <= 0.0) {
            respawnRover();
        }
        return;
    }

    rover_.step(roverBody_, dt);
    vehicle_sim::physics::Integrator2D::integrate(roverBody_, dt);
    roverBody_.clearForces();

    course_.update(roverBody_.getPosition().x, dt);
    camera_.update(roverBody_.getPosition(), rover_.getSpeed(), dt);
    parallax_.update(dt);

    bool fatal { false };
    if (obstacles_.checkCollision(roverBody_, rover_, fatal)) {
        if (fatal) {
            crashed_ = true;
            respawnTimer_ = 1.6;
            lives_ = std::max(0, lives_ - 1);
        }
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

void GameApp::renderFrame() noexcept
{
    if (renderer_ == nullptr) {
        return;
    }

    parallax_.render(renderer_.get(), camera_, biomeIndex_);
    renderTerrain();
    obstacles_.render(renderer_.get(), camera_);
    renderCheckpoints();

    if (!crashed_) {
        renderRover();
    } else {
        renderCrashPuff();
    }

    renderHUD();

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

        // Underground soil fill
        if (biomeIndex_ == 1) { // Desert red sandstone
            SDL_SetRenderDrawColor(renderer_.get(), 110, 52, 34, 255);
        } else if (biomeIndex_ == 2) { // Alpine loam
            SDL_SetRenderDrawColor(renderer_.get(), 45, 52, 40, 255);
        } else { // Industrial gravel/soil
            SDL_SetRenderDrawColor(renderer_.get(), 48, 42, 40, 255);
        }

        const int topY { static_cast<int>(std::min(prevScreen.y, screen.y)) };
        const int fillHeight { std::max(0, screenBottom - topY) };
        const int leftX { static_cast<int>(prevScreen.x) };
        const int width { static_cast<int>(std::max(1.0, screen.x - prevScreen.x + 1.0)) };
        SDL_Rect soilRect { leftX, topY, width, fillHeight };
        SDL_RenderFillRect(renderer_.get(), &soilRect);

        // Surface grass / gravel / sand rim
        if (biomeIndex_ == 1) {
            SDL_SetRenderDrawColor(renderer_.get(), 205, 125, 65, 255); // Sand crest
        } else if (biomeIndex_ == 2) {
            SDL_SetRenderDrawColor(renderer_.get(), 75, 155, 65, 255); // Alpine grass
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 92, 168, 64, 255); // Meadow turf
        }

        SDL_RenderDrawLine(renderer_.get(), static_cast<int>(prevScreen.x), static_cast<int>(prevScreen.y),
            static_cast<int>(screen.x), static_cast<int>(screen.y));

        SDL_RenderDrawLine(renderer_.get(), static_cast<int>(prevScreen.x), static_cast<int>(prevScreen.y + 1.0),
            static_cast<int>(screen.x), static_cast<int>(screen.y + 1.0));

        prevX = x;
        prevScreen = screen;
    }
}

void GameApp::renderCheckpoints() noexcept
{
    const double minX { camera_.getVisibleMinX() - 4.0 };
    const double maxX { camera_.getVisibleMaxX() + 4.0 };
    const auto& heightfield { course_.getHeightfield() };

    for (const auto& cp : course_.getCheckpoints()) {
        if (cp.worldX < minX || cp.worldX > maxX) {
            continue;
        }

        const double elev { heightfield.getHeight(cp.worldX) };
        const auto baseScreen { camera_.worldToScreen(vehicle_sim::math::Vec2 { cp.worldX, elev }) };

        // Pylon mast
        SDL_SetRenderDrawColor(renderer_.get(), 200, 200, 210, 255);
        const int pylonX { static_cast<int>(baseScreen.x) };
        const int pylonBaseY { static_cast<int>(baseScreen.y) };
        const int pylonTopY { pylonBaseY - 65 };
        SDL_RenderDrawLine(renderer_.get(), pylonX, pylonBaseY, pylonX, pylonTopY);
        SDL_RenderDrawLine(renderer_.get(), pylonX + 1, pylonBaseY, pylonX + 1, pylonTopY);

        // Milestone sign flag
        if (cp.reached) {
            SDL_SetRenderDrawColor(renderer_.get(), 50, 220, 90, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 240, 190, 40, 255);
        }
        SDL_Rect flagRect { pylonX, pylonTopY, 26, 16 };
        SDL_RenderFillRect(renderer_.get(), &flagRect);

        // Mast beacon light
        SDL_SetRenderDrawColor(renderer_.get(), 255, 60, 60, 255);
        SDL_Rect lightRect { pylonX - 2, pylonTopY - 5, 5, 5 };
        SDL_RenderFillRect(renderer_.get(), &lightRect);
    }
}

void GameApp::renderRover() noexcept
{
    const auto roverScreen { camera_.worldToScreen(roverBody_.getPosition()) };
    const int roverX { static_cast<int>(roverScreen.x) };
    const int roverY { static_cast<int>(roverScreen.y) };

    // Suspension struts and wheels
    for (std::size_t i { 0 }; i < vehicle_sim::dynamics::EarthRoverDynamics::kWheelCount; ++i) {
        const auto& wheel { rover_.getWheelState(i) };
        const auto wheelScreen { camera_.worldToScreen(wheel.worldPos) };

        const int wScreenX { static_cast<int>(wheelScreen.x) };
        const int wScreenY { static_cast<int>(wheelScreen.y) };

        // Strut rod
        SDL_SetRenderDrawColor(renderer_.get(), 180, 180, 190, 255);
        SDL_RenderDrawLine(renderer_.get(), roverX, roverY, wScreenX, wScreenY);

        // Wheel tire
        if (wheel.isGrounded) {
            SDL_SetRenderDrawColor(renderer_.get(), 35, 35, 42, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 70, 70, 80, 255);
        }
        SDL_Rect tireRect { wScreenX - 11, wScreenY - 11, 22, 22 };
        SDL_RenderFillRect(renderer_.get(), &tireRect);

        // Wheel hub rim
        SDL_SetRenderDrawColor(renderer_.get(), 200, 170, 60, 255);
        SDL_Rect rimRect { wScreenX - 4, wScreenY - 4, 8, 8 };
        SDL_RenderFillRect(renderer_.get(), &rimRect);
    }

    // Chassis body
    SDL_SetRenderDrawColor(renderer_.get(), 220, 160, 40, 255);
    SDL_Rect chassisRect { roverX - 44, roverY - 14, 88, 24 };
    SDL_RenderFillRect(renderer_.get(), &chassisRect);

    // Front nose angle
    SDL_SetRenderDrawColor(renderer_.get(), 245, 185, 55, 255);
    SDL_Rect noseRect { roverX + 24, roverY - 10, 24, 16 };
    SDL_RenderFillRect(renderer_.get(), &noseRect);

    // Cockpit dome
    SDL_SetRenderDrawColor(renderer_.get(), 100, 210, 255, 255);
    SDL_Rect cockpitRect { roverX - 8, roverY - 24, 28, 12 };
    SDL_RenderFillRect(renderer_.get(), &cockpitRect);

    // Thruster exhaust nozzle
    SDL_SetRenderDrawColor(renderer_.get(), 70, 70, 75, 255);
    SDL_Rect exhaustRect { roverX - 48, roverY - 6, 8, 12 };
    SDL_RenderFillRect(renderer_.get(), &exhaustRect);
}

void GameApp::renderCrashPuff() noexcept
{
    const auto roverScreen { camera_.worldToScreen(roverBody_.getPosition()) };
    const int rx { static_cast<int>(roverScreen.x) };
    const int ry { static_cast<int>(roverScreen.y) };

    const double progress { 1.6 - respawnTimer_ };
    const int radius { static_cast<int>(progress * 35.0) };

    // Smoke puff clouds
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
    // Top course progress bar
    constexpr int barWidth { 500 };
    constexpr int barHeight { 8 };
    const int barX { (config_.windowWidth - barWidth) / 2 };
    constexpr int barY { 24 };

    // Background track line
    SDL_SetRenderDrawColor(renderer_.get(), 50, 56, 75, 255);
    SDL_Rect bgBar { barX, barY, barWidth, barHeight };
    SDL_RenderFillRect(renderer_.get(), &bgBar);

    // Completed progress fill
    const int fillW { static_cast<int>(course_.getProgressFraction() * static_cast<double>(barWidth)) };
    SDL_SetRenderDrawColor(renderer_.get(), 80, 210, 95, 255);
    SDL_Rect fillBar { barX, barY, fillW, barHeight };
    SDL_RenderFillRect(renderer_.get(), &fillBar);

    // Checkpoint ticks along progress bar
    for (std::size_t i { 0 }; i < course_.getCheckpoints().size(); ++i) {
        const double frac { static_cast<double>(i + 1) / 5.0 };
        const int tickX { barX + static_cast<int>(frac * static_cast<double>(barWidth)) };

        if (course_.getCheckpoints()[i].reached) {
            SDL_SetRenderDrawColor(renderer_.get(), 60, 240, 100, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 230, 200, 60, 255);
        }
        SDL_Rect tickRect { tickX - 2, barY - 4, 4, barHeight + 8 };
        SDL_RenderFillRect(renderer_.get(), &tickRect);
    }

    // Rover marker icon on progress bar
    const int roverMarkerX { barX + fillW };
    SDL_SetRenderDrawColor(renderer_.get(), 255, 240, 80, 255);
    SDL_Rect roverMarker { roverMarkerX - 4, barY - 6, 8, 20 };
    SDL_RenderFillRect(renderer_.get(), &roverMarker);

    // Lives icons (top-left)
    for (int i { 0 }; i < lives_; ++i) {
        SDL_SetRenderDrawColor(renderer_.get(), 240, 180, 45, 255);
        SDL_Rect lifeRect { 30 + (i * 24), 22, 16, 12 };
        SDL_RenderFillRect(renderer_.get(), &lifeRect);
    }

    // Speed indicator bar (bottom-left)
    const double speedKmh { std::max(0.0, rover_.getSpeed() * 3.6) };
    const int speedBarW { static_cast<int>(std::clamp(speedKmh * 2.0, 0.0, 200.0)) };
    SDL_SetRenderDrawColor(renderer_.get(), 40, 45, 60, 255);
    SDL_Rect speedBg { 30, config_.windowHeight - 40, 200, 14 };
    SDL_RenderFillRect(renderer_.get(), &speedBg);

    SDL_SetRenderDrawColor(renderer_.get(), 240, 160, 30, 255);
    SDL_Rect speedFill { 30, config_.windowHeight - 40, speedBarW, 14 };
    SDL_RenderFillRect(renderer_.get(), &speedFill);
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

} // namespace earth_patrol
