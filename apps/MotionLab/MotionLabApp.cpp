#include "apps/MotionLab/MotionLabApp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

constexpr std::array<MotionLabVehicleType, 6> kAvailableVehicles {{
    MotionLabVehicleType::ClassicCar,
    MotionLabVehicleType::RaceCar,
    MotionLabVehicleType::Rocket,
    MotionLabVehicleType::Helicopter,
    MotionLabVehicleType::Hovercraft,
    MotionLabVehicleType::TrackedTank
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

    applyVehicleProperties();
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

void MotionLabApp::applyVehicleProperties() noexcept
{
    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        mBody.setMass(1000.0);
        mBody.setInertia(1666.667);
        mCar.setSurfaceMaterial(getSurfaceMaterial());
        break;
    case MotionLabVehicleType::RaceCar:
        mBody.setMass(mRaceCar.getConfig().mass);
        mBody.setInertia(mRaceCar.getConfig().inertia);
        break;
    case MotionLabVehicleType::Rocket:
        mBody.setMass(mRocket.getTotalMass());
        mBody.setInertia(mRocket.getInertia());
        break;
    case MotionLabVehicleType::Helicopter:
        mBody.setMass(mHelicopter.getConfig().mass);
        mBody.setInertia(4800.0);
        break;
    case MotionLabVehicleType::Hovercraft:
        mBody.setMass(mHovercraft.getConfig().mass);
        mBody.setInertia(2084.0);
        mHovercraft.setSurfaceFriction(getSurfaceMaterial().friction);
        break;
    case MotionLabVehicleType::TrackedTank:
        mBody.setMass(mTank.getConfig().mass);
        mBody.setInertia(30000.0);
        mTank.setSurfaceFriction(getSurfaceMaterial().friction);
        break;
    }
}

void MotionLabApp::resetSimulation() noexcept
{
    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.resetToGrid(mBody);
    } else if (mSlalomCoordinator.isEnabled()) {
        mSlalomCoordinator.resetCourse(mBody);
    } else {
        mBody.setPosition(vehicle_sim::math::Vec2::zero());
        mBody.setLinearVelocity(vehicle_sim::math::Vec2::zero());
        mBody.setOrientation(0.0);
        mBody.setAngularVelocity(0.0);
        mBody.clearForces();
    }

    mCar.reset();
    mRaceCar.reset();
    mRocket.reset();
    mHelicopter.reset();
    mHovercraft.reset();
    mTank.reset();

    applyVehicleProperties();

    mCamera.reset(mBody.getPosition());
    mSkids.clear();
    mTrail.clear();
    mAnimTimer = 0.0;

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

void MotionLabApp::toggleCircuit() noexcept
{
    mCircuitCoordinator.toggleEnabled();
    if (mCircuitCoordinator.isEnabled()) {
        if (mSlalomCoordinator.isEnabled()) {
            mSlalomCoordinator.setEnabled(false);
        }
        mCircuitCoordinator.resetToGrid(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::regenerateCircuit() noexcept
{
    mCircuitSeed += 137U;
    mCircuitCoordinator.regenerate(mCircuitSeed);
    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.resetToGrid(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isCircuitActive() const noexcept
{
    return mCircuitCoordinator.isEnabled();
}

const CircuitTrackCoordinator& MotionLabApp::getCircuit() const noexcept
{
    return mCircuitCoordinator;
}

CircuitTrackCoordinator& MotionLabApp::getCircuit() noexcept
{
    return mCircuitCoordinator;
}

void MotionLabApp::toggleSlalom() noexcept
{
    mSlalomCoordinator.toggleEnabled();
    if (mSlalomCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        mSlalomCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

void MotionLabApp::setSlalom(bool enabled) noexcept
{
    mSlalomCoordinator.setEnabled(enabled);
    if (mSlalomCoordinator.isEnabled()) {
        if (mCircuitCoordinator.isEnabled()) {
            mCircuitCoordinator.setEnabled(false);
        }
        mSlalomCoordinator.resetCourse(mBody);
        mCamera.reset(mBody.getPosition());
    }
}

bool MotionLabApp::isSlalomActive() const noexcept
{
    return mSlalomCoordinator.isEnabled();
}

const SlalomCourseCoordinator& MotionLabApp::getSlalom() const noexcept
{
    return mSlalomCoordinator;
}

SlalomCourseCoordinator& MotionLabApp::getSlalom() noexcept
{
    return mSlalomCoordinator;
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
    const auto mat = vehicle_sim::terrain::SurfaceMaterial::getPreset(type);
    mCar.setSurfaceMaterial(mat);
    mHovercraft.setSurfaceFriction(mat.friction);
    mTank.setSurfaceFriction(mat.friction);
}

vehicle_sim::terrain::MaterialType MotionLabApp::getTerrain() const noexcept
{
    return mCar.getSurfaceMaterial().type;
}

const vehicle_sim::terrain::SurfaceMaterial& MotionLabApp::getSurfaceMaterial() const noexcept
{
    return mCar.getSurfaceMaterial();
}

void MotionLabApp::cycleVehicle() noexcept
{
    mVehicleIndex = (mVehicleIndex + 1U) % kAvailableVehicles.size();
    setVehicleType(kAvailableVehicles[mVehicleIndex]);
}

void MotionLabApp::setVehicleType(MotionLabVehicleType type) noexcept
{
    mVehicleType = type;
    for (std::size_t i { 0U }; i < kAvailableVehicles.size(); ++i) {
        if (kAvailableVehicles[i] == type) {
            mVehicleIndex = i;
            break;
        }
    }
    applyVehicleProperties();
}

MotionLabVehicleType MotionLabApp::getVehicleType() const noexcept
{
    return mVehicleType;
}

std::string_view MotionLabApp::getVehicleName() const noexcept
{
    return getVehicleTypeName(mVehicleType);
}

vehicle_sim::math::Vec2 MotionLabApp::computeSupportPoint(const vehicle_sim::math::Vec2& normal) const noexcept
{
    double halfLen { 2.0 };
    double halfWid { 1.0 };

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        return mCar.computeSupportPoint(mBody, normal);
    case MotionLabVehicleType::RaceCar:
        halfLen = 2.25;
        halfWid = 0.90;
        break;
    case MotionLabVehicleType::Rocket:
        halfLen = 3.00;
        halfWid = 0.80;
        break;
    case MotionLabVehicleType::Helicopter:
        halfLen = 3.00;
        halfWid = 1.20;
        break;
    case MotionLabVehicleType::Hovercraft:
        halfLen = 2.00;
        halfWid = 1.10;
        break;
    case MotionLabVehicleType::TrackedTank:
        halfLen = 2.50;
        halfWid = 1.20;
        break;
    }

    const std::array<vehicle_sim::math::Vec2, 4> points {{
        mBody.toWorldPoint(vehicle_sim::math::Vec2 { -halfLen, -halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 {  halfLen, -halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 {  halfLen,  halfWid }),
        mBody.toWorldPoint(vehicle_sim::math::Vec2 { -halfLen,  halfWid })
    }};

    double minProj { points[0].dot(normal) };
    for (std::size_t i { 1 }; i < points.size(); ++i) {
        minProj = std::min(minProj, points[i].dot(normal));
    }

    vehicle_sim::math::Vec2 sum { 0.0, 0.0 };
    double count { 0.0 };
    for (const auto& pt : points) {
        if (std::abs(pt.dot(normal) - minProj) < 1e-8) {
            sum += pt;
            count += 1.0;
        }
    }
    return (count > 0.0) ? (sum / count) : points[0];
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
                case SDLK_c:
                    toggleCircuit();
                    break;
                case SDLK_l:
                    toggleSlalom();
                    break;
                case SDLK_n:
                    regenerateCircuit();
                    break;
                case SDLK_t:
                    cycleTerrain();
                    break;
                case SDLK_v:
                    cycleVehicle();
                    break;
                case SDLK_F1:
                    setVehicleType(MotionLabVehicleType::ClassicCar);
                    break;
                case SDLK_F2:
                    setVehicleType(MotionLabVehicleType::RaceCar);
                    break;
                case SDLK_F3:
                    setVehicleType(MotionLabVehicleType::Rocket);
                    break;
                case SDLK_F4:
                    setVehicleType(MotionLabVehicleType::Helicopter);
                    break;
                case SDLK_F5:
                    setVehicleType(MotionLabVehicleType::Hovercraft);
                    break;
                case SDLK_F6:
                    setVehicleType(MotionLabVehicleType::TrackedTank);
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

    if (mVehicleType == MotionLabVehicleType::ClassicCar) {
        const auto& tires = mCar.getTires();
        const auto& rearLeft = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearLeft];
        const auto& rearRight = tires[vehicle_sim::dynamics::MotionLabCarDynamics::kRearRight];

        if (rearLeft.skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, rearLeft.worldPosition);
        }
        mRearLeftState.position = rearLeft.worldPosition;
        mRearLeftState.skidding = rearLeft.skidding;

        if (rearRight.skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, rearRight.worldPosition);
        }
        mRearRightState.position = rearRight.worldPosition;
        mRearRightState.skidding = rearRight.skidding;
    } else if (mVehicleType == MotionLabVehicleType::RaceCar) {
        const auto& wheels = mRaceCar.getWheelStates();
        const auto worldRL = mBody.toWorldPoint(wheels[2].localOffset);
        const auto worldRR = mBody.toWorldPoint(wheels[3].localOffset);
        const bool skiddingRL { std::abs(wheels[2].slipAngle) > 0.15 || std::abs(wheels[2].slipRatio) > 0.15 };
        const bool skiddingRR { std::abs(wheels[3].slipAngle) > 0.15 || std::abs(wheels[3].slipRatio) > 0.15 };

        if (skiddingRL && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldRL);
        }
        mRearLeftState.position = worldRL;
        mRearLeftState.skidding = skiddingRL;

        if (skiddingRR && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldRR);
        }
        mRearRightState.position = worldRR;
        mRearRightState.skidding = skiddingRR;
    } else if (mVehicleType == MotionLabVehicleType::TrackedTank) {
        const auto worldL = mBody.toWorldPoint({ -1.8, 1.05 });
        const auto worldR = mBody.toWorldPoint({ -1.8, -1.05 });
        const bool skidding { std::abs(mBody.getAngularVelocity()) > 0.25 };

        if (skidding && mRearLeftState.skidding) {
            mSkids.addSegment(mRearLeftState.position, worldL);
        }
        mRearLeftState.position = worldL;
        mRearLeftState.skidding = skidding;

        if (skidding && mRearRightState.skidding) {
            mSkids.addSegment(mRearRightState.position, worldR);
        }
        mRearRightState.position = worldR;
        mRearRightState.skidding = skidding;
    }
}

void MotionLabApp::stepPhysics(double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    mAnimTimer += dt;

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool handbrake { mKeySpace };
        mCar.setInput({ throttle, steer, handbrake });
        mCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::RaceCar: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double brake { mKeyS ? 1.0 : 0.0 };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool drs { mKeySpace };
        mRaceCar.setThrottle(throttle);
        mRaceCar.setBrake(brake);
        mRaceCar.setSteer(steer);
        mRaceCar.setDrsActive(drs);
        mRaceCar.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Rocket: {
        const double throttle { mKeyW ? 1.0 : 0.0 };
        const double gimbal { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double rcsRot { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const double rcsTransX { (mKeySpace ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        mRocket.setInput({ throttle, gimbal, rcsRot, { rcsTransX, 0.0 }, true });
        mRocket.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Helicopter: {
        const double cyclic { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double rudder { (mKeyD ? 1.0 : 0.0) - (mKeyA ? 1.0 : 0.0) };
        const double collective { mKeySpace ? 1.0 : (mKeyW || mKeyS ? 0.8 : 0.68) };
        mHelicopter.setInput({ collective, cyclic, rudder, true, 10.0 });
        mHelicopter.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::Hovercraft: {
        const double throttle { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double steer { (mKeyA ? 1.0 : 0.0) - (mKeyD ? 1.0 : 0.0) };
        const bool liftFan { !mKeySpace };
        mHovercraft.setInput({ throttle, steer, liftFan, 0.35 });
        mHovercraft.step(mBody, dt);
        break;
    }
    case MotionLabVehicleType::TrackedTank: {
        double leftTrack { 0.0 };
        double rightTrack { 0.0 };
        const double fwd { (mKeyW ? 1.0 : 0.0) - (mKeyS ? 1.0 : 0.0) };
        const double turn { (mKeyD ? 1.0 : 0.0) - (mKeyA ? 1.0 : 0.0) };

        if (std::abs(fwd) > 0.01) {
            leftTrack = fwd + turn * 0.4;
            rightTrack = fwd - turn * 0.4;
        } else {
            leftTrack = turn;
            rightTrack = -turn;
        }

        leftTrack = std::clamp(leftTrack, -1.0, 1.0);
        rightTrack = std::clamp(rightTrack, -1.0, 1.0);

        if (mKeySpace) {
            mTank.fireRecoil();
        }
        mTank.setInput({ leftTrack, rightTrack, mKeySpace, 0.0 });
        mTank.step(mBody, dt);
        break;
    }
    }

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

    if (mCircuitCoordinator.isEnabled()) {
        mCircuitCoordinator.stepPhysics(mBody, dt);
        const auto mat = mCircuitCoordinator.getActiveMaterial();
        mCar.setSurfaceMaterial(mat);
        mHovercraft.setSurfaceFriction(mat.friction);
        mTank.setSurfaceFriction(mat.friction);
    }

    if (mSlalomCoordinator.isEnabled()) {
        mSlalomCoordinator.stepPhysics(mBody, dt);
    }

    vehicle_sim::physics::Integrator2D::integrate(mBody, dt);

    if (mBarrier) {
        vehicle_sim::physics::BarrierSolver2D::solveInside(
            mBody,
            mCamera.getBounds(),
            [this](const vehicle_sim::math::Vec2& normal) noexcept {
                return computeSupportPoint(normal);
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

    VehicleRenderContext context { };
    context.type = mVehicleType;
    context.terrain = getTerrain();
    context.car = &mCar;
    context.raceCar = &mRaceCar;
    context.rocket = &mRocket;
    context.helicopter = &mHelicopter;
    context.hovercraft = &mHovercraft;
    context.tank = &mTank;
    context.animTimer = mAnimTimer;

    mWireframe.renderScene(
        mRenderer,
        mCamera,
        context,
        mBody,
        mSkids,
        mTrail,
        mBarrier
    );

    if (mCircuitCoordinator.isEnabled()) {
        CircuitTrackRenderer::renderTrack(mRenderer, mCamera, mCircuitCoordinator);
    }

    if (mSlalomCoordinator.isEnabled()) {
        SlalomCourseRenderer::renderCourse(mRenderer, mCamera, mSlalomCoordinator);
    }

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
    snapshot.vehicleName = std::string(getVehicleName());

    snapshot.circuitMode = mCircuitCoordinator.isEnabled();
    snapshot.slalomMode = mSlalomCoordinator.isEnabled();
    if (mCircuitCoordinator.isEnabled()) {
        const auto& metrics = mCircuitCoordinator.getMetrics();
        snapshot.currentLapTime = metrics.currentLapTime;
        snapshot.bestLapTime = metrics.bestLapTime;
        snapshot.lastLapTime = metrics.lastLapTime;
        snapshot.currentSector = metrics.currentSector;
        snapshot.completedLaps = metrics.completedLaps;
        snapshot.terrainName = std::string(vehicle_sim::terrain::SurfaceMaterial::getName(mCircuitCoordinator.getActiveMaterial().type));
        snapshot.terrainFriction = mCircuitCoordinator.getActiveMaterial().friction;
        snapshot.terrainRollingDrag = mCircuitCoordinator.getActiveMaterial().rollingDrag;
    }

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        snapshot.vehicleDetails = mCar.isSkidding() ? "TIRES SKIDDING" : "GRIP OK";
        break;
    case MotionLabVehicleType::RaceCar:
        snapshot.vehicleDetails = mRaceCar.getDrsAngleDeg() > 1.0 ? "DRS ACTIVE (LOW DRAG)" : "AERO DOWNFORCE ENGAGED";
        break;
    case MotionLabVehicleType::Rocket:
        snapshot.vehicleDetails = std::string("FUEL ") + std::to_string(static_cast<int>(mRocket.getFuelRatio() * 100.0)) + "% / THRUST " + std::to_string(static_cast<int>(mRocket.getThrustForce() / 1000.0)) + " KN";
        break;
    case MotionLabVehicleType::Helicopter:
        snapshot.vehicleDetails = std::string("ROTOR ") + std::to_string(static_cast<int>(mHelicopter.getRotorSpeed())) + " RPM / " + (mHelicopter.isAirborne() ? "AIRBORNE" : "SURFACE");
        break;
    case MotionLabVehicleType::Hovercraft:
        snapshot.vehicleDetails = mHovercraft.isHovering() ? "AIR CUSHION ACTIVE" : "SKIRT BRAKING";
        break;
    case MotionLabVehicleType::TrackedTank:
        snapshot.vehicleDetails = mTank.getTelemetry().isRecoiling ? "CANNON FIRING RECOIL" : "DUAL TRACK READY";
        break;
    }

    TelemetryHud::render(mRenderer, snapshot, mConfig.windowWidth, mConfig.windowHeight);

    if (mCircuitCoordinator.isEnabled()) {
        CircuitTrackRenderer::renderRadar(
            mRenderer, mCircuitCoordinator, mBody.getPosition(),
            mConfig.windowWidth - 170, 30, 140, 140);
    }

    if (mSlalomCoordinator.isEnabled()) {
        SlalomCourseRenderer::renderHud(
            mRenderer, mSlalomCoordinator, 10, mConfig.windowHeight - 145);
    }

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

const vehicle_sim::dynamics::RaceCarDynamics& MotionLabApp::getRaceCar() const noexcept
{
    return mRaceCar;
}

vehicle_sim::dynamics::RaceCarDynamics& MotionLabApp::getRaceCar() noexcept
{
    return mRaceCar;
}

const vehicle_sim::dynamics::RocketDynamics& MotionLabApp::getRocket() const noexcept
{
    return mRocket;
}

vehicle_sim::dynamics::RocketDynamics& MotionLabApp::getRocket() noexcept
{
    return mRocket;
}

const vehicle_sim::dynamics::HelicopterDynamics& MotionLabApp::getHelicopter() const noexcept
{
    return mHelicopter;
}

vehicle_sim::dynamics::HelicopterDynamics& MotionLabApp::getHelicopter() noexcept
{
    return mHelicopter;
}

const vehicle_sim::dynamics::HovercraftDynamics& MotionLabApp::getHovercraft() const noexcept
{
    return mHovercraft;
}

vehicle_sim::dynamics::HovercraftDynamics& MotionLabApp::getHovercraft() noexcept
{
    return mHovercraft;
}

const vehicle_sim::dynamics::TrackedTankDynamics& MotionLabApp::getTank() const noexcept
{
    return mTank;
}

vehicle_sim::dynamics::TrackedTankDynamics& MotionLabApp::getTank() noexcept
{
    return mTank;
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
