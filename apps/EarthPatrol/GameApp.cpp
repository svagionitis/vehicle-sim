#include "GameApp.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace earth_patrol {

namespace {
    inline constexpr double kMaxFrameTime { 0.25 };
    inline constexpr double kPixelsPerMeter { 40.0 };
} // namespace

GameApp::GameApp()
    : GameApp(GameAppConfig { })
{
}

GameApp::GameApp(const GameAppConfig& config)
    : config_ { config }
{
    terrain_.setBaseHeight(0.0);
    rover_.setTerrain(&terrain_);

    vehicle_sim::physics::RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 850.0;
    bodyConfig.inertia = 650.0;
    bodyConfig.position = vehicle_sim::math::Vec2 { 0.0, 1.20 };
    roverBody_ = vehicle_sim::physics::RigidBody2D { bodyConfig };
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
    rover_.step(roverBody_, dt);
    vehicle_sim::physics::Integrator2D::integrate(roverBody_, dt);
    roverBody_.clearForces();
}

void GameApp::renderFrame() noexcept
{
    if (renderer_ == nullptr) {
        return;
    }

    // Sky background
    SDL_SetRenderDrawColor(renderer_.get(), 26, 32, 48, 255);
    SDL_RenderClear(renderer_.get());

    const int originX { config_.windowWidth / 4 };
    const int originY { config_.windowHeight * 3 / 4 };

    // Render ground profile
    SDL_SetRenderDrawColor(renderer_.get(), 80, 140, 70, 255);
    const int groundScreenY { originY };
    SDL_RenderDrawLine(renderer_.get(), 0, groundScreenY, config_.windowWidth, groundScreenY);

    // Rover chassis world position
    const auto& pos { roverBody_.getPosition() };
    const int roverScreenX { originX + static_cast<int>(pos.x * kPixelsPerMeter) };
    const int roverScreenY { originY - static_cast<int>(pos.y * kPixelsPerMeter) };

    // Draw chassis body rectangle
    SDL_SetRenderDrawColor(renderer_.get(), 210, 160, 50, 255);
    SDL_Rect chassisRect { roverScreenX - 44, roverScreenY - 14, 88, 24 };
    SDL_RenderFillRect(renderer_.get(), &chassisRect);

    // Draw 3 suspension struts and wheel hubs
    for (std::size_t i { 0 }; i < vehicle_sim::dynamics::EarthRoverDynamics::kWheelCount; ++i) {
        const auto& wheel { rover_.getWheelState(i) };
        const int wheelScreenX { originX + static_cast<int>(wheel.worldPos.x * kPixelsPerMeter) };
        const int wheelScreenY { originY - static_cast<int>(wheel.worldPos.y * kPixelsPerMeter) };

        // Suspension strut line
        SDL_SetRenderDrawColor(renderer_.get(), 180, 180, 180, 255);
        SDL_RenderDrawLine(renderer_.get(), roverScreenX, roverScreenY, wheelScreenX, wheelScreenY);

        // Wheel hub
        if (wheel.isGrounded) {
            SDL_SetRenderDrawColor(renderer_.get(), 50, 50, 60, 255);
        } else {
            SDL_SetRenderDrawColor(renderer_.get(), 100, 100, 110, 255);
        }
        SDL_Rect wheelRect { wheelScreenX - 10, wheelScreenY - 10, 20, 20 };
        SDL_RenderFillRect(renderer_.get(), &wheelRect);
    }

    SDL_RenderPresent(renderer_.get());
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

} // namespace earth_patrol
