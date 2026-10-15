#include "apps/MotionLab/MotionLabApp.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include "libs/VehicleSimCore/physics/Integrator2D.h"

namespace motion_lab {

namespace {
constexpr std::array<vehicle_sim::terrain::MaterialType, 7> kAvailableTerrains {{
    vehicle_sim::terrain::MaterialType::Tarmac,
    vehicle_sim::terrain::MaterialType::Ice,
    vehicle_sim::terrain::MaterialType::Sand,
    vehicle_sim::terrain::MaterialType::Mud,
    vehicle_sim::terrain::MaterialType::Grass,
    vehicle_sim::terrain::MaterialType::Gravel,
    vehicle_sim::terrain::MaterialType::Snow
}};
} // namespace

MotionLabApp::MotionLabApp()
    : MotionLabApp(MotionLabAppConfig { })
{
}

MotionLabApp::MotionLabApp(const MotionLabAppConfig& config)
    : mConfig { config }
{
    vehicle_sim::physics::RigidBodyConfig bodyConfig { };
    bodyConfig.mass = 1000.0;
    bodyConfig.inertia = 1666.667;
    bodyConfig.position = vehicle_sim::math::Vec2 { 0.0, 0.0 };
    bodyConfig.orientation = 0.0;

    mBody = vehicle_sim::physics::RigidBody2D { bodyConfig };

    CameraConfig camConfig { };
    camConfig.viewportWidth = config.windowWidth;
    camConfig.viewportHeight = config.windowHeight;
    camConfig.baseZoom = 28.0;
    mCamera = MotionLabCamera { camConfig };
}

MotionLabApp::~MotionLabApp() noexcept
{
    if (mRenderer != nullptr) {
        SDL_DestroyRenderer(mRenderer);
        mRenderer = nullptr;
    }
    if (mWindow != nullptr) {
        SDL_DestroyWindow(mWindow);
        mWindow = nullptr;
    }
    if (!mConfig.headless) {
        SDL_Quit();
    }
}

bool MotionLabApp::initialize()
{
    if (mConfig.headless) {
        mRunning = true;
        resetSimulation();
        return true;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        return false;
    }

    mWindow = SDL_CreateWindow(
        mConfig.windowTitle.c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        mConfig.windowWidth,
        mConfig.windowHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (mWindow == nullptr) {
        return false;
    }

    mRenderer = SDL_CreateRenderer(
        mWindow,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (mRenderer == nullptr) {
        return false;
    }

    mRunning = true;
    resetSimulation();
    return true;
}

void MotionLabApp::resetSimulation() noexcept
{
    mBody.setPosition(vehicle_sim::math::Vec2::zero());
    mBody.setLinearVelocity(vehicle_sim::math::Vec2::zero());
    mBody.setOrientation(0.0);
    mBody.setAngularVelocity(0.0);
    mBody.clearForces();

    mCar.reset();
    mCamera.reset(vehicle_sim::math::Vec2::zero());
    mSkids.clear();
    mTrail.clear();

    mRearLeftState = RearTireState { };
    mRearRightState = RearTireState { };

    mKeyW = false;
    mKeyS = false;
    mKeyA = false;
    mKeyD = false;
    mKeySpace = false;
}

void MotionLabApp::stop() noexcept
{
    mRunning = false;
}

void MotionLabApp::togglePause() noexcept
{
    mPaused = !mPaused;
}

void MotionLabApp::toggleBarrier() noexcept
{
    mBarrier = !mBarrier;
    mCamera.setFixedCamera(mBarrier);
}

void MotionLabApp::toggleGravity() noexcept
{
    mGravity = !mGravity;
}

void MotionLabApp::toggleDeadZone() noexcept
{
    mCamera.setDeadZoneEnabled(!mCamera.isDeadZoneEnabled());
}

void MotionLabApp::toggleSpeedZoom() noexcept
{
    mCamera.setSpeedZoomEnabled(!mCamera.isSpeedZoomEnabled());
}

void MotionLabApp::toggleInputHud() noexcept
{
    mInputHudVisible = !mInputHudVisible;
}

void MotionLabApp::toggleFullscreen() noexcept
{
    if (mWindow == nullptr) {
        return;
    }
    mIsFullscreen = !mIsFullscreen;
    SDL_SetWindowFullscreen(mWindow, mIsFullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

void MotionLabApp::cycleTerrain() noexcept
{
    mTerrainIndex = (mTerrainIndex + 1U) % kAvailableTerrains.size();
    setTerrain(kAvailableTerrains[mTerrainIndex]);
}

void MotionLabApp::setTerrain(vehicle_sim::terrain::MaterialType type) noexcept
{
    for (std::size_t i { 0U }; i < kAvailableTerrains.size(); ++i) {
        if (kAvailableTerrains[i] == type) {
            mTerrainIndex = i;
            break;
        }
    }
    mCar.setSurfaceMaterial(vehicle_sim::terrain::SurfaceMaterial::getPreset(type));
}

vehicle_sim::terrain::MaterialType MotionLabApp::getTerrain() const noexcept
{
    return mCar.getSurfaceMaterial().type;
}

const vehicle_sim::terrain::SurfaceMaterial& MotionLabApp::getSurfaceMaterial() const noexcept
{
    return mCar.getSurfaceMaterial();
}

void MotionLabApp::processEvents() noexcept
{
    SDL_Event event { };
    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
            mRunning = false;
        } else if (event.type == SDL_WINDOWEVENT) {
            if (event.window.event == SDL_WINDOWEVENT_RESIZED ||
                event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                mConfig.windowWidth = event.window.data1;
                mConfig.windowHeight = event.window.data2;
                mCamera.resize(mConfig.windowWidth, mConfig.windowHeight);
            }
        } else if (event.type == SDL_KEYDOWN) {
            const SDL_Keycode key { event.key.keysym.sym };
            if (!event.key.repeat) {
                switch (key) {
                case SDLK_w:
                case SDLK_UP:
                    mKeyW = true;
                    break;
                case SDLK_s:
                case SDLK_DOWN:
                    mKeyS = true;
                    break;
                case SDLK_a:
                case SDLK_LEFT:
                    mKeyA = true;
                    break;
                case SDLK_d:
                case SDLK_RIGHT:
                    mKeyD = true;
                    break;
                case SDLK_SPACE:
                    mKeySpace = true;
                    break;
                case SDLK_r:
                    resetSimulation();
                    break;
                case SDLK_p:
                    togglePause();
                    break;
                case SDLK_b:
                    toggleBarrier();
                    break;
                case SDLK_g:
                    toggleGravity();
                    break;
                case SDLK_z:
                    toggleDeadZone();
                    break;
                case SDLK_x:
                    toggleSpeedZoom();
                    break;
                case SDLK_h:
                    toggleInputHud();
                    break;
                case SDLK_f:
                    toggleFullscreen();
                    break;
                case SDLK_t:
                    cycleTerrain();
                    break;
                case SDLK_1:
                case SDLK_KP_1:
                    setTerrain(vehicle_sim::terrain::MaterialType::Tarmac);
                    break;
                case SDLK_2:
                case SDLK_KP_2:
                    setTerrain(vehicle_sim::terrain::MaterialType::Ice);
                    break;
                case SDLK_3:
                case SDLK_KP_3:
                    setTerrain(vehicle_sim::terrain::MaterialType::Sand);
                    break;
                case SDLK_4:
                case SDLK_KP_4:
                    setTerrain(vehicle_sim::terrain::MaterialType::Mud);
                    break;
                case SDLK_5:
                case SDLK_KP_5:
                    setTerrain(vehicle_sim::terrain::MaterialType::Grass);
                    break;
                case SDLK_6:
                case SDLK_KP_6:
                    setTerrain(vehicle_sim::terrain::MaterialType::Gravel);
                    break;
                case SDLK_7:
                case SDLK_KP_7:
                    setTerrain(vehicle_sim::terrain::MaterialType::Snow);
                    break;
                case SDLK_ESCAPE:
                    mRunning = false;
                    break;
                default:
                    break;
                }
            }
        } else if (event.type == SDL_KEYUP) {
            const SDL_Keycode key { event.key.keysym.sym };
            switch (key) {
            case SDLK_w:
            case SDLK_UP:
                mKeyW = false;
                break;
            case SDLK_s:
            case SDLK_DOWN:
                mKeyS = false;
                break;
            case SDLK_a:
            case SDLK_LEFT:
                mKeyA = false;
                break;
            case SDLK_d:
            case SDLK_RIGHT:
                mKeyD = false;
                break;
            case SDLK_SPACE:
                mKeySpace = false;
                break;
            default:
                break;
            }
        }
    }
}

void MotionLabApp::updateTireSkidmarks(double dt) noexcept
{
    mSkids.update(dt);

    const auto& tires = mCar.getTires();
    const auto& rearLeft = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearLeft];
    const auto& rearRight = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearRight];

    // Rear-Left tire skids
    if (rearLeft.skidding && mRearLeftState.skidding) {
        mSkids.addSegment(mRearLeftState.position, rearLeft.worldPosition);
    }
    mRearLeftState.position = rearLeft.worldPosition;
    mRearLeftState.skidding = rearLeft.skidding;

    // Rear-Right tire skids
    if (rearRight.skidding && mRearRightState.skidding) {
        mSkids.addSegment(mRearRightState.position, rearRight.worldPosition);
    }
    mRearRightState.position = rearRight.worldPosition;
    mRearRightState.skidding = rearRight.skidding;
}

void MotionLabApp::stepPhysics(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
    const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
    const bool handbrake { mKeySpace };

    mCar.setInput({ throttle, steer, handbrake });

    // Optional downward vertical gravity and linear atmospheric terminal drag
    if (mGravity) {
        constexpr double kGravityAcc { 3.0 }; // m/s^2
        constexpr double kTerminalSpeed { 30.0 }; // m/s
        const double dragCoeff { (mBody.getMass() * kGravityAcc) / kTerminalSpeed };
        const auto v = mBody.getLinearVelocity();

        const vehicle_sim::math::Vec2 gravForce {
            -dragCoeff * v.x,
            -mBody.getMass() * kGravityAcc - dragCoeff * v.y
        };
        mBody.applyForce(gravForce);
    }

    mCar.step(mBody, dt);
    vehicle_sim::physics::Integrator2D::integrate(mBody, dt);

    if (mBarrier) {
        vehicle_sim::physics::BarrierSolver2D::solveInside(
            mBody,
            mCamera.getBounds(),
            [this](const vehicle_sim::math::Vec2& normal) noexcept {
                return mCar.computeSupportPoint(mBody, normal);
            });
    }

    mTrail.update(mBody.getPosition(), dt);
    updateTireSkidmarks(dt);
}

void MotionLabApp::renderFrame() noexcept
{
    if (mRenderer == nullptr) {
        return;
    }

    mWireframe.renderScene(
        mRenderer,
        mCamera,
        mCar,
        mBody,
        mSkids,
        mTrail,
        mBarrier
    );

    const auto v = mBody.getLinearVelocity();
    const double speedMps { v.length() };
    const double speedMph { speedMps / 0.44704 };

    TelemetrySnapshot snapshot { };
    snapshot.speedMps = speedMps;
    snapshot.speedMph = speedMph;
    snapshot.angularVelocity = mBody.getAngularVelocity();
    snapshot.stepDurationMs = mConfig.fixedDt * 1000.0;
    snapshot.droppedSeconds = mDroppedTime;
    snapshot.fps = mCurrentFps;
    snapshot.barrierOn = mBarrier;
    snapshot.gravityOn = mGravity;
    snapshot.deadZoneOn = mCamera.isDeadZoneEnabled();
    snapshot.speedZoomOn = mCamera.isSpeedZoomEnabled();
    snapshot.inputHudVisible = mInputHudVisible;
    snapshot.steerLeft = mKeyA;
    snapshot.steerRight = mKeyD;
    snapshot.throttleForward = mKeyW;
    snapshot.throttleReverse = mKeyS;
    snapshot.handbrake = mKeySpace;
    snapshot.isPaused = mPaused;
    snapshot.terrainName = std::string(vehicle_sim::terrain::SurfaceMaterial::getName(getTerrain()));
    snapshot.terrainFriction = mCar.getSurfaceMaterial().friction;
    snapshot.terrainRollingDrag = mCar.getSurfaceMaterial().rollingDrag;

    TelemetryHud::render(mRenderer, snapshot, mConfig.windowWidth, mConfig.windowHeight);

    SDL_RenderPresent(mRenderer);
}

void MotionLabApp::run()
{
    constexpr int kMaxSubSteps { 12 };
    const double fixedDt { mConfig.fixedDt };

    uint64_t lastCounter { SDL_GetPerformanceCounter() };
    const double counterFreq { static_cast<double>(SDL_GetPerformanceFrequency()) };

    while (mRunning) {
        processEvents();

        const uint64_t currentCounter { SDL_GetPerformanceCounter() };
        double elapsed { static_cast<double>(currentCounter - lastCounter) / counterFreq };
        lastCounter = currentCounter;

        if (elapsed > 0.25) {
            elapsed = 0.25;
        }

        // FPS tracking
        ++mFrameCounter;
        mFpsTimer += elapsed;
        if (mFpsTimer >= 0.5) {
            mCurrentFps = static_cast<double>(mFrameCounter) / mFpsTimer;
            mFrameCounter = 0;
            mFpsTimer = 0.0;
        }

        if (!mPaused) {
            const double accepted { std::min(elapsed, fixedDt * kMaxSubSteps) };
            mDroppedTime += (elapsed - accepted);
            mTimeAccumulator += accepted;

            int steps { 0 };
            while (mTimeAccumulator >= fixedDt && steps < kMaxSubSteps) {
                stepPhysics(fixedDt);
                mTimeAccumulator -= fixedDt;
                ++steps;
            }
        }

        mCamera.update(mBody.getPosition(), mBody.getLinearVelocity(), elapsed);
        renderFrame();
    }
}

bool MotionLabApp::isRunning() const noexcept
{
    return mRunning;
}

bool MotionLabApp::isPaused() const noexcept
{
    return mPaused;
}

bool MotionLabApp::isBarrierActive() const noexcept
{
    return mBarrier;
}

bool MotionLabApp::isGravityActive() const noexcept
{
    return mGravity;
}

const vehicle_sim::dynamics::MotionLabCarDynamics& MotionLabApp::getCar() const noexcept
{
    return mCar;
}

vehicle_sim::dynamics::MotionLabCarDynamics& MotionLabApp::getCar() noexcept
{
    return mCar;
}

const vehicle_sim::physics::RigidBody2D& MotionLabApp::getBody() const noexcept
{
    return mBody;
}

vehicle_sim::physics::RigidBody2D& MotionLabApp::getBody() noexcept
{
    return mBody;
}

const MotionLabCamera& MotionLabApp::getCamera() const noexcept
{
    return mCamera;
}

MotionLabCamera& MotionLabApp::getCamera() noexcept
{
    return mCamera;
}

const MotionLabAppConfig& MotionLabApp::getConfig() const noexcept
{
    return mConfig;
}

} // namespace motion_lab
