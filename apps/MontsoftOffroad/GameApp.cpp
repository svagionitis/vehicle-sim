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
    truckDynamics_.setTerrain(&courseManager_.getHeightfield());
    const double startY { courseManager_.getHeightfield().getHeight(0.0) + 1.2 };
    truckBody_.setMass(truckDynamics_.getConfig().mass);
    truckBody_.setInertia(truckDynamics_.getConfig().momentOfInertia);
    truckBody_.setPosition(Vec2{ 0.0, startY });
    truckBody_.setLinearVelocity(Vec2{ 0.0, 0.0 });
    truckBody_.setOrientation(0.0);
    truckBody_.setAngularVelocity(0.0);
    cgaRenderer_.setPaletteMode(config_.initialPalette);
    camera_.reset(truckBody_.getPosition());
    inputController_.reset();
    particleSystem_.reset();
    truckRenderer_.reset();
    prevRearComp_ = 0.0;
    prevFrontComp_ = 0.0;

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
        } else {
            inputController_.handleEvent(event);
        }
    }
}

void GameApp::updateSimulation(double dt) noexcept
{
    // Process latched system-level actions
    const SystemAction action { inputController_.consumeSystemAction() };
    switch (action) {
    case SystemAction::CyclePalette:
        cgaRenderer_.cyclePaletteMode();
        break;
    case SystemAction::SetPaletteMode1:
        cgaRenderer_.setPaletteMode(PaletteMode::CgaPalette1);
        break;
    case SystemAction::SetPaletteMode0:
        cgaRenderer_.setPaletteMode(PaletteMode::CgaPalette0);
        break;
    case SystemAction::SetPaletteComposite:
        cgaRenderer_.setPaletteMode(PaletteMode::CompositeNtsc);
        break;
    case SystemAction::SetPaletteEga:
        cgaRenderer_.setPaletteMode(PaletteMode::EnhancedEga);
        break;
    case SystemAction::TogglePause:
        togglePause();
        break;
    case SystemAction::Quit:
        stop();
        break;
    case SystemAction::None:
    default:
        break;
    }

    if (paused_) {
        return;
    }

    inputController_.update(dt);
    const auto truckInput { inputController_.getTruckInput() };

    courseManager_.update(truckBody_.getPosition().x, dt);

    truckDynamics_.setInput(truckInput);
    truckDynamics_.step(truckBody_, dt);
    vehicle_sim::physics::Integrator2D::integrate(truckBody_, dt);
    truckBody_.clearForces();

    // Weapon projectile firing and cadence coordinator
    combatManager_.handleFireInputs(truckInput, truckBody_, truckDynamics_, camera_);

    // Terramechanic dust and spark emission
    const auto& rearAxle { truckDynamics_.getAxleState(0) };
    const auto& frontAxle { truckDynamics_.getAxleState(1) };
    const std::size_t stageIdx { static_cast<std::size_t>(std::max(0, courseManager_.getCurrentStageIndex())) };

    // 1. Sand Rooster Tails
    if (rearAxle.isGrounded) {
        particleSystem_.emitRoosterTail(rearAxle.worldPos, truckBody_.getLinearVelocity(), rearAxle.soilSinkage, stageIdx);
    }
    if (frontAxle.isGrounded) {
        particleSystem_.emitRoosterTail(frontAxle.worldPos, truckBody_.getLinearVelocity(), frontAxle.soilSinkage, stageIdx);
    }

    // 2. Landing Dust Plumes
    const double rearCompVel { (rearAxle.compression - prevRearComp_) / dt };
    const double frontCompVel { (frontAxle.compression - prevFrontComp_) / dt };
    prevRearComp_ = rearAxle.compression;
    prevFrontComp_ = frontAxle.compression;

    if (rearAxle.isGrounded && rearCompVel > 1.8) {
        particleSystem_.emitLandingDust(rearAxle.worldPos, rearCompVel, stageIdx);
    }
    if (frontAxle.isGrounded && frontCompVel > 1.8) {
        particleSystem_.emitLandingDust(frontAxle.worldPos, frontCompVel, stageIdx);
    }

    // 3. Jump Thruster Sparks
    if (truckInput.jump) {
        particleSystem_.emitThrusterSparks(truckBody_.getPosition() - Vec2{ 0.0, 0.15 }, truckBody_.getLinearVelocity());
    }

    particleSystem_.update(dt);
    truckRenderer_.updateAnimation(truckBody_.getLinearVelocity().x, dt);

    const std::vector<vehicle_sim::physics::RigidBody2D*> sceneBodies{ &truckBody_ };
    combatManager_.update(
        dt,
        &courseManager_.getHeightfield(),
        particleSystem_,
        camera_,
        sceneBodies,
        nullptr,
        stageIdx
    );

    camera_.update(truckBody_.getPosition(), truckBody_.getLinearVelocity().x, dt);
    parallax_.update(dt);
}

void GameApp::renderFrame() noexcept
{
    // 1. Render 4-tier parallax scenery and foreground heightfield track
    parallax_.render(
        cgaRenderer_,
        camera_,
        courseManager_.getHeightfield(),
        courseManager_.getCurrentStageIndex()
    );

    // 2. Render checkpoint gantries & milestone flags
    for (const auto& cp : courseManager_.getCheckpoints()) {
        const double groundY { courseManager_.getHeightfield().getHeight(cp.worldX) };
        const Vec2 baseScreen { camera_.worldToScreen(Vec2{ cp.worldX, groundY }) };
        const int bx { static_cast<int>(std::round(baseScreen.x)) };
        const int by { static_cast<int>(std::round(baseScreen.y)) };

        if (bx >= -20 && bx < CgaPaletteRenderer::kNativeWidth + 20) {
            // Checkpoint striped gantry pole (height 30px)
            cgaRenderer_.fillRect(bx - 1, by - 30, 3, 30, ColorRgba{ 255, 255, 255, 255 });
            cgaRenderer_.fillRect(bx - 1, by - 24, 3, 6, ColorRgba{ 255, 85, 85, 255 });
            cgaRenderer_.fillRect(bx - 1, by - 12, 3, 6, ColorRgba{ 255, 85, 85, 255 });

            // Milestone letter flag
            const ColorRgba flagColor { cp.reached ? ColorRgba{ 85, 255, 85, 255 } : ColorRgba{ 255, 255, 85, 255 } };
            cgaRenderer_.fillRect(bx + 2, by - 30, 8, 8, flagColor);
        }
    }

    // 3. Render terramechanic dust plumes & rooster tails (Layer behind truck)
    particleSystem_.render(cgaRenderer_, camera_);

    // 4. Render articulated 4WD desert truck chassis, suspension, knobby tires, weapons
    truckRenderer_.render(cgaRenderer_, camera_, truckBody_, truckDynamics_);

    // 5. Render active ballistic projectiles, rockets, mortars, and shockwaves
    combatManager_.render(cgaRenderer_, camera_);

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

void GameApp::setInput(const vehicle_sim::dynamics::OffroadTruckInput& input) noexcept
{
    inputController_.setTruckInput(input);
    truckDynamics_.setInput(input);
}

const vehicle_sim::dynamics::OffroadTruckInput& GameApp::getInput() const noexcept
{
    return inputController_.getTruckInput();
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
    return combatManager_.getBallistics();
}

vehicle_sim::dynamics::OffroadBallistics2D& GameApp::getBallistics() noexcept
{
    return combatManager_.getBallistics();
}

const CombatManager& GameApp::getCombatManager() const noexcept
{
    return combatManager_;
}

CombatManager& GameApp::getCombatManager() noexcept
{
    return combatManager_;
}

const CgaPaletteRenderer& GameApp::getRenderer() const noexcept
{
    return cgaRenderer_;
}

CgaPaletteRenderer& GameApp::getRenderer() noexcept
{
    return cgaRenderer_;
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
    return courseManager_;
}

CourseManager& GameApp::getCourse() noexcept
{
    return courseManager_;
}

const ParallaxRenderer& GameApp::getParallax() const noexcept
{
    return parallax_;
}

ParallaxRenderer& GameApp::getParallax() noexcept
{
    return parallax_;
}

const InputController& GameApp::getInputController() const noexcept
{
    return inputController_;
}

InputController& GameApp::getInputController() noexcept
{
    return inputController_;
}

const ParticleSystem& GameApp::getParticleSystem() const noexcept
{
    return particleSystem_;
}

ParticleSystem& GameApp::getParticleSystem() noexcept
{
    return particleSystem_;
}

const TruckRenderer& GameApp::getTruckRenderer() const noexcept
{
    return truckRenderer_;
}

TruckRenderer& GameApp::getTruckRenderer() noexcept
{
    return truckRenderer_;
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
