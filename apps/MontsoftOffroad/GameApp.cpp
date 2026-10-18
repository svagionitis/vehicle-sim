#include "apps/MontsoftOffroad/GameApp.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;

GameApp::GameApp()
    : GameApp(GameAppConfig{})
{
}

GameApp::GameApp(const GameAppConfig& config)
    : config_{ config }
{
}

bool GameApp::initialize()
{
    truckBody_.setMass(truckDynamics_.getConfig().mass);
    truckBody_.setInertia(truckDynamics_.getConfig().momentOfInertia);
    truckBody_.setPosition(Vec2{ 0.0, 1.5 });
    truckBody_.setLinearVelocity(Vec2{ 0.0, 0.0 });
    truckBody_.setOrientation(0.0);
    truckBody_.setAngularVelocity(0.0);
    cgaRenderer_.setPaletteMode(config_.initialPalette);

    if (config_.headless) {
        running_ = true;
        return true;
    }

    sdlContext_ = std::make_unique<SDLContext>();
    if (!sdlContext_->isInitialized()) {
        return false;
    }

    SDL_Window* rawWin { SDL_CreateWindow(
        config_.windowTitle.c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        config_.windowWidth,
        config_.windowHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    ) };

    if (rawWin == nullptr) {
        return false;
    }
    window_.reset(rawWin);

    SDL_Renderer* rawRnd { SDL_CreateRenderer(
        window_.get(),
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    ) };

    if (rawRnd == nullptr) {
        rawRnd = SDL_CreateRenderer(window_.get(), -1, SDL_RENDERER_ACCELERATED);
    }
    if (rawRnd == nullptr) {
        rawRnd = SDL_CreateRenderer(window_.get(), -1, SDL_RENDERER_SOFTWARE);
    }
    if (rawRnd == nullptr) {
        return false;
    }
    renderer_.reset(rawRnd);

    if (!cgaRenderer_.initialize(renderer_.get())) {
        return false;
    }

    lastCounter_ = SDL_GetPerformanceCounter();
    running_ = true;
    return true;
}

void GameApp::run()
{
    const double perfFreq { static_cast<double>(SDL_GetPerformanceFrequency()) };

    while (running_) {
        processEvents();

        if (!paused_) {
            const std::uint64_t nowCounter { SDL_GetPerformanceCounter() };
            double frameTime { static_cast<double>(nowCounter - lastCounter_) / perfFreq };
            lastCounter_ = nowCounter;

            // Clamp frameTime to 0.25 seconds to prevent spiral-of-death
            frameTime = std::min(frameTime, 0.25);
            accumulator_ += frameTime;

            while (accumulator_ >= config_.fixedDt) {
                updateSimulation(config_.fixedDt);
                accumulator_ -= config_.fixedDt;
            }
        } else {
            lastCounter_ = SDL_GetPerformanceCounter();
            SDL_Delay(16);
        }

        renderFrame();
    }
}

void GameApp::stop() noexcept
{
    running_ = false;
}

void GameApp::processEvents() noexcept
{
    if (config_.headless) {
        return;
    }

    SDL_Event event {};
    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
            stop();
        } else if (event.type == SDL_KEYDOWN) {
            handleKeyDown(event.key.keysym.sym);
        } else if (event.type == SDL_KEYUP) {
            handleKeyUp(event.key.keysym.sym);
        }
    }
}

void GameApp::handleKeyDown(SDL_Keycode key) noexcept
{
    switch (key) {
    case SDLK_UP:
    case SDLK_w:
        currentInput_.throttle = 1.0;
        break;
    case SDLK_DOWN:
    case SDLK_s:
        currentInput_.brake = 1.0;
        break;
    case SDLK_SPACE:
        currentInput_.jump = true;
        break;
    case SDLK_z:
    case SDLK_j:
        currentInput_.firePrimary = true;
        break;
    case SDLK_x:
    case SDLK_k:
        currentInput_.fireSecondary = true;
        break;
    case SDLK_c:
    case SDLK_TAB:
        cgaRenderer_.cyclePaletteMode();
        break;
    case SDLK_F1:
        cgaRenderer_.setPaletteMode(PaletteMode::CgaPalette1);
        break;
    case SDLK_F2:
        cgaRenderer_.setPaletteMode(PaletteMode::CgaPalette0);
        break;
    case SDLK_F3:
        cgaRenderer_.setPaletteMode(PaletteMode::CompositeNtsc);
        break;
    case SDLK_F4:
        cgaRenderer_.setPaletteMode(PaletteMode::EnhancedEga);
        break;
    case SDLK_p:
        togglePause();
        break;
    case SDLK_ESCAPE:
        stop();
        break;
    default:
        break;
    }
}

void GameApp::handleKeyUp(SDL_Keycode key) noexcept
{
    switch (key) {
    case SDLK_UP:
    case SDLK_w:
        currentInput_.throttle = 0.0;
        break;
    case SDLK_DOWN:
    case SDLK_s:
        currentInput_.brake = 0.0;
        break;
    case SDLK_SPACE:
        currentInput_.jump = false;
        break;
    case SDLK_z:
    case SDLK_j:
        currentInput_.firePrimary = false;
        break;
    case SDLK_x:
    case SDLK_k:
        currentInput_.fireSecondary = false;
        break;
    default:
        break;
    }
}

void GameApp::updateSimulation(double dt) noexcept
{
    if (paused_) {
        return;
    }

    truckDynamics_.setInput(currentInput_);
    truckDynamics_.step(truckBody_, dt);
    vehicle_sim::physics::Integrator2D::integrate(truckBody_, dt);
    truckBody_.clearForces();

    if (currentInput_.firePrimary) {
        ballistics_.fireRocket(
            truckBody_.getPosition() + Vec2{ 1.2, 0.2 },
            truckBody_.getOrientation(),
            truckBody_.getLinearVelocity()
        );
        currentInput_.firePrimary = false;
    }

    if (currentInput_.fireSecondary) {
        ballistics_.fireMortar(
            truckBody_.getPosition() + Vec2{ 0.0, 0.4 },
            truckBody_.getLinearVelocity()
        );
        currentInput_.fireSecondary = false;
    }

    ballistics_.step(dt, nullptr);
}

void GameApp::renderFrame() noexcept
{
    cgaRenderer_.clear(ColorRgba{ 0, 0, 0, 255 });

    // Ground reference baseline
    cgaRenderer_.drawLine(0, 160, 319, 160, ColorRgba{ 85, 255, 255, 255 });

    // Chassis placeholder
    cgaRenderer_.fillRect(60, 125, 45, 18, ColorRgba{ 255, 85, 255, 255 });
    cgaRenderer_.drawRect(60, 125, 45, 18, ColorRgba{ 255, 255, 255, 255 });

    // Wheels
    cgaRenderer_.fillRect(64, 143, 10, 10, ColorRgba{ 255, 255, 255, 255 });
    cgaRenderer_.fillRect(91, 143, 10, 10, ColorRgba{ 255, 255, 255, 255 });

    // Projectiles
    for (const auto& proj : ballistics_.getProjectiles()) {
        const int px { static_cast<int>(proj.position.x * 20.0) % 320 };
        const int py { static_cast<int>(160.0 - proj.position.y * 20.0) };
        cgaRenderer_.fillRect(px - 1, py - 1, 3, 3, ColorRgba{ 255, 255, 255, 255 });
    }

    if (!config_.headless && window_ && renderer_) {
        int w { config_.windowWidth };
        int h { config_.windowHeight };
        SDL_GetWindowSize(window_.get(), &w, &h);
        cgaRenderer_.present(renderer_.get(), w, h);
    }
}

bool GameApp::isRunning() const noexcept
{
    return running_;
}

const GameAppConfig& GameApp::getConfig() const noexcept
{
    return config_;
}

const vehicle_sim::dynamics::OffroadTruckDynamics& GameApp::getTruck() const noexcept
{
    return truckDynamics_;
}

vehicle_sim::dynamics::OffroadTruckDynamics& GameApp::getTruck() noexcept
{
    return truckDynamics_;
}

const vehicle_sim::physics::RigidBody2D& GameApp::getTruckBody() const noexcept
{
    return truckBody_;
}

vehicle_sim::physics::RigidBody2D& GameApp::getTruckBody() noexcept
{
    return truckBody_;
}

const vehicle_sim::dynamics::OffroadBallistics2D& GameApp::getBallistics() const noexcept
{
    return ballistics_;
}

vehicle_sim::dynamics::OffroadBallistics2D& GameApp::getBallistics() noexcept
{
    return ballistics_;
}

const CgaPaletteRenderer& GameApp::getRenderer() const noexcept
{
    return cgaRenderer_;
}

CgaPaletteRenderer& GameApp::getRenderer() noexcept
{
    return cgaRenderer_;
}

bool GameApp::isPaused() const noexcept
{
    return paused_;
}

void GameApp::togglePause() noexcept
{
    paused_ = !paused_;
}

} // namespace montsoft_offroad
